/*
 * MCDMA Crypto QEMU Backend
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Implements a PCI device (vendor 0x1172 / device 0x0001 / class 0x1080)
 * that handles MCDMA crypto descriptors submitted by the DPDK MCDMA crypto
 * PMD running inside the guest.
 *
 * The device reuses the same MCDMA descriptor-ring CSR layout as the netdev
 * backend (mcdma.c), but only the TX-queue region (BAR0 offset 0x80000) is
 * used.  For each submitted descriptor the backend:
 *
 *   1. Reads the McdmaDesc from the TX queue ring in guest memory.
 *   2. Follows desc.src to read the McdmaCryptoHwDesc.
 *   3. Copies data_length bytes from hw_desc.src_addr → hw_desc.dst_addr
 *      (loopback – no actual encryption).
 *   4. Sets hw_desc.completion_status = MCDMA_CRYPTO_STATUS_SUCCESS and
 *      writes the updated hw_desc back to desc.src.
 *   5. Advances the queue head and writes it to the consumed-head wb_addr
 *      so the DPDK driver can complete the operation.
 *
 * This is sufficient to verify the DPDK MCDMA crypto PMD data-path without
 * a real hardware accelerator.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "exec/memory.h"
#include "qapi/error.h"

#include "mcdma_crypto.h"
#include "mcdma_regs.h"

/* -------------------------------------------------------------------------
 * PCI identity for the crypto device
 * -----------------------------------------------------------------------*/
#define MCDMA_CRYPTO_VENDOR_ID   0x1172
#define MCDMA_CRYPTO_DEVICE_ID   0x0001
#define MCDMA_CRYPTO_REVISION    0x01
/* PCI class 0x1080 = Encryption/Decryption controller */
#define MCDMA_CRYPTO_CLASS_CODE  0x1080

#define MCDMA_CRYPTO_NR_VECTORS  4

/* -------------------------------------------------------------------------
 * BAR0 register read/write
 *
 * The crypto device exposes only the TX-queue CSR region (starting at
 * MCDMA_TX_Q_OFFSET = 0x80000) and the global registers at 0x200000.
 * RX-queue registers are accepted but silently ignored.
 * -----------------------------------------------------------------------*/
static uint64_t
mcdma_crypto_bar0_read(void *opaque, hwaddr addr, unsigned size)
{
    McdmaCryptoState *s = MCDMA_CRYPTO(opaque);
    uint32_t val = 0;
    uint32_t qid;
    hwaddr reg;
    McdmaQueue *q;

    /* TX queue region: 0x080000 – 0x1FFFFF */
    if (addr >= MCDMA_TX_Q_OFFSET && addr < MCDMA_GLOBAL_OFFSET) {
        hwaddr tx_base = addr - MCDMA_TX_Q_OFFSET;
        qid = tx_base / MCDMA_QUEUE_CSR_SIZE;
        reg = tx_base % MCDMA_QUEUE_CSR_SIZE;
        if (qid >= s->num_queues) {
            goto out;
        }
        q = &s->tx_queues[qid];
        switch (reg) {
        case Q_CTRL:            val = q->ctrl;                              break;
        case Q_START_ADDR_L:    val = (uint32_t)(q->ring_base & 0xFFFFFFFF); break;
        case Q_START_ADDR_H:    val = (uint32_t)(q->ring_base >> 32);        break;
        case Q_SIZE:            val = q->ring_size_log2;                    break;
        case Q_TAIL_POINTER:    val = q->tail & Q_TAIL_PTR_MASK;            break;
        case Q_HEAD_POINTER:    val = q->head & Q_HEAD_PTR_MASK;            break;
        case Q_COMPLETED_POINTER: val = q->completed & Q_COMPLETED_PTR_MASK; break;
        case Q_CONSUMED_HEAD_ADDR_L:
            val = (uint32_t)(q->wb_addr & 0xFFFFFFFF); break;
        case Q_CONSUMED_HEAD_ADDR_H:
            val = (uint32_t)(q->wb_addr >> 32);         break;
        case Q_CPL_TIMEOUT:     val = 0;                                    break;
        default:                                                            break;
        }
        goto out;
    }

    /* Global register region: 0x200000+ */
    switch (addr) {
    case GLB_VERSION:
        val = MCDMA_RTL_VERSION;
        break;
    case GLB_LINK_STATUS:
        val = GLB_LINK_STATUS_UP;
        break;
    default:
        break;
    }

out:
    return val;
}

static void
mcdma_crypto_bar0_write(void *opaque, hwaddr addr, uint64_t data,
                         unsigned size)
{
    McdmaCryptoState *s = MCDMA_CRYPTO(opaque);
    uint32_t val = (uint32_t)data;
    uint32_t qid;
    hwaddr reg;
    McdmaQueue *q;

    /* Ignore RX queue region writes (not used by crypto driver) */
    if (addr < MCDMA_TX_Q_OFFSET) {
        return;
    }

    /* TX queue region */
    if (addr >= MCDMA_TX_Q_OFFSET && addr < MCDMA_GLOBAL_OFFSET) {
        hwaddr tx_base = addr - MCDMA_TX_Q_OFFSET;
        qid = tx_base / MCDMA_QUEUE_CSR_SIZE;
        reg = tx_base % MCDMA_QUEUE_CSR_SIZE;
        if (qid >= s->num_queues) {
            return;
        }
        q = &s->tx_queues[qid];
        switch (reg) {
        case Q_CTRL:
            q->ctrl = val;
            q->enabled = (val & Q_CTRL_Q_EN) != 0;
            break;
        case Q_START_ADDR_L:
            q->ring_base = (q->ring_base & 0xFFFFFFFF00000000ULL) | val;
            break;
        case Q_START_ADDR_H:
            q->ring_base = (q->ring_base & 0x00000000FFFFFFFFULL) |
                           ((uint64_t)val << 32);
            break;
        case Q_SIZE:
            q->ring_size_log2 = val & Q_SIZE_MASK;
            q->ring_entries = 1u << q->ring_size_log2;
            if (q->ring_entries > 65536) {
                q->ring_entries = 65536;
            }
            break;
        case Q_TAIL_POINTER:
            q->tail = val & Q_TAIL_PTR_MASK;
            /* Writing tail triggers descriptor processing */
            if (q->enabled && !s->bh_scheduled) {
                s->bh_scheduled = true;
                qemu_bh_schedule(s->bh);
            }
            break;
        case Q_CONSUMED_HEAD_ADDR_L:
            q->wb_addr = (q->wb_addr & 0xFFFFFFFF00000000ULL) | val;
            break;
        case Q_CONSUMED_HEAD_ADDR_H:
            q->wb_addr = (q->wb_addr & 0x00000000FFFFFFFFULL) |
                         ((uint64_t)val << 32);
            break;
        case Q_RESET:
            if (val & Q_RESET_MASK) {
                q->ctrl = 0;
                q->ring_base = 0;
                q->ring_size_log2 = 0;
                q->ring_entries = 0;
                q->head = 0;
                q->tail = 0;
                q->completed = 0;
                q->wb_addr = 0;
                q->enabled = false;
                q->running = false;
                q->desc_processed = 0;
                q->bytes_copied = 0;
                q->invalid_skipped = 0;
                q->backpressure_stalls = 0;
            }
            break;
        default:
            break;
        }
        return;
    }
}

static const MemoryRegionOps mcdma_crypto_bar0_ops = {
    .read  = mcdma_crypto_bar0_read,
    .write = mcdma_crypto_bar0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid  = { .min_access_size = 4, .max_access_size = 4, },
    .impl   = { .min_access_size = 4, .max_access_size = 4, },
};

static uint64_t mcdma_crypto_bar2_read(void *o, hwaddr a, unsigned s) { return 0; }
static void     mcdma_crypto_bar2_write(void *o, hwaddr a, uint64_t d, unsigned s) {}
static const MemoryRegionOps mcdma_crypto_bar2_ops = {
    .read = mcdma_crypto_bar2_read, .write = mcdma_crypto_bar2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4, },
};

static uint64_t mcdma_crypto_bar4_read(void *o, hwaddr a, unsigned s) { return 0; }
static void     mcdma_crypto_bar4_write(void *o, hwaddr a, uint64_t d, unsigned s) {}
static const MemoryRegionOps mcdma_crypto_bar4_ops = {
    .read = mcdma_crypto_bar4_read, .write = mcdma_crypto_bar4_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4, },
};

/* -------------------------------------------------------------------------
 * Descriptor processing
 * -----------------------------------------------------------------------*/

/*
 * Write the consumed-head value to the guest wb_addr so the DPDK driver
 * knows how many operations have completed.
 */
static void
mcdma_crypto_writeback(McdmaCryptoState *s, McdmaQueue *q)
{
    if (q->wb_addr == 0) {
        return;
    }
    address_space_stl_le(&address_space_memory, q->wb_addr,
                          q->head & Q_HEAD_PTR_MASK,
                          MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * Process one TX queue: for each valid descriptor fetch the
 * McdmaCryptoHwDesc, copy src→dst (loopback), write back
 * completion_status = SUCCESS, and advance the head pointer.
 */
static void
mcdma_crypto_process_txq(McdmaCryptoState *s, McdmaQueue *txq)
{
    McdmaDesc desc;
    McdmaCryptoHwDesc hw_desc;
    hwaddr desc_addr;
    uint32_t idx;
    uint8_t buf[MCDMA_CRYPTO_MAX_XFER_SIZE];
    uint8_t tag_buf[32];
    uint32_t xfer_len;
    bool wb_pending = false;

    if (!txq->enabled || txq->ring_entries == 0) {
        return;
    }

    while (txq->head != txq->tail) {
        idx = txq->head % txq->ring_entries;
        desc_addr = txq->ring_base + (hwaddr)idx * sizeof(McdmaDesc);

        /* Read the MCDMA descriptor */
        address_space_read(&address_space_memory, desc_addr,
                           MEMTXATTRS_UNSPECIFIED,
                           (uint8_t *)&desc, sizeof(desc));

        /* Skip invalid or link descriptors */
        if (desc.desc_invalid) {
            txq->invalid_skipped++;
            txq->head++;
            continue;
        }
        if (desc.link) {
            /* Link descriptor: just advance */
            txq->head++;
            continue;
        }

        /* desc.src = IOVA of McdmaCryptoHwDesc in guest memory */
        if (desc.src == 0) {
            txq->head++;
            continue;
        }

        /* Read the crypto hw descriptor */
        address_space_read(&address_space_memory, desc.src,
                           MEMTXATTRS_UNSPECIFIED,
                           (uint8_t *)&hw_desc, sizeof(hw_desc));

        /* Loopback: copy data src_addr → dst_addr */
        if (hw_desc.data_length > 0 &&
            hw_desc.src_addr != 0 && hw_desc.dst_addr != 0) {
            xfer_len = hw_desc.data_length;
            if (xfer_len > sizeof(buf)) {
                xfer_len = sizeof(buf);
            }
            address_space_read(&address_space_memory, hw_desc.src_addr,
                               MEMTXATTRS_UNSPECIFIED, buf, xfer_len);
            address_space_write(&address_space_memory, hw_desc.dst_addr,
                                MEMTXATTRS_UNSPECIFIED, buf, xfer_len);
            txq->bytes_copied += xfer_len;
        }

    /*
     * For encrypt requests, emit a deterministic fake tag so the guest-side
     * test can verify that the descriptor metadata path is wired correctly.
     */
    if (hw_desc.operation == 0 && hw_desc.tag_addr != 0 &&
        hw_desc.digest_length > 0) {
        uint32_t tag_len = hw_desc.digest_length;

        if (tag_len > sizeof(tag_buf)) {
            tag_len = sizeof(tag_buf);
        }
        memset(tag_buf, 0xa5, tag_len);
        address_space_write(&address_space_memory, hw_desc.tag_addr,
                            MEMTXATTRS_UNSPECIFIED, tag_buf, tag_len);
    }

        /*
         * Set completion_status = SUCCESS and write back only the
         * completion_status field (at offset 60 in McdmaCryptoHwDesc).
         * The DPDK driver reads this field from the local hw_desc_ring[]
         * buffer (which is the same guest memory pointed to by desc.src).
         */
        hw_desc.completion_status = MCDMA_CRYPTO_STATUS_SUCCESS;
        address_space_write(&address_space_memory,
                            desc.src +
                            offsetof(McdmaCryptoHwDesc, completion_status),
                            MEMTXATTRS_UNSPECIFIED,
                            (uint8_t *)&hw_desc.completion_status,
                            sizeof(hw_desc.completion_status));

        txq->head++;
        txq->completed = txq->head;
        txq->desc_processed++;
        wb_pending = true;

        /* MSI-X interrupt if requested */
        if (desc.msix_en && (txq->ctrl & Q_CTRL_Q_INTR_EN)) {
            if (s->msix_initialized) {
                uint32_t vector = txq->qid % s->nr_vectors;
                msix_notify(&s->parent_obj, vector);
            }
        }
    }

    /* Write-back consumed head once per BH invocation */
    if (wb_pending && (txq->ctrl & Q_CTRL_Q_WB_EN)) {
        mcdma_crypto_writeback(s, txq);
    }
}

static void
mcdma_crypto_process_descq(McdmaCryptoState *s)
{
    uint32_t i;

    s->bh_scheduled = false;
    for (i = 0; i < s->num_queues; i++) {
        mcdma_crypto_process_txq(s, &s->tx_queues[i]);
    }
}

static void
mcdma_crypto_bh_cb(void *opaque)
{
    McdmaCryptoState *s = MCDMA_CRYPTO(opaque);
    mcdma_crypto_process_descq(s);
}

/* -------------------------------------------------------------------------
 * Device lifecycle
 * -----------------------------------------------------------------------*/

static Property mcdma_crypto_properties[] = {
    DEFINE_PROP_UINT32("num_queues", McdmaCryptoState, num_queues, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void
mcdma_crypto_instance_init(Object *obj)
{
    McdmaCryptoState *s = MCDMA_CRYPTO(obj);

    s->num_queues = 1;
    s->bh = NULL;
    s->bh_scheduled = false;
    s->msix_initialized = false;
}

static void
mcdma_crypto_instance_finalize(Object *obj)
{
    McdmaCryptoState *s = MCDMA_CRYPTO(obj);

    if (s->bh) {
        qemu_bh_delete(s->bh);
        s->bh = NULL;
    }
    if (s->msix_initialized) {
        msix_uninit(&s->parent_obj, &s->bar0, &s->bar0);
    }
    g_free(s->tx_queues);
}

static void
mcdma_crypto_realize(PCIDevice *pci_dev, Error **errp)
{
    McdmaCryptoState *s = MCDMA_CRYPTO(pci_dev);
    int ret;
    uint32_t i;

    /* Allocate TX queue array */
    s->tx_queues = g_new0(McdmaQueue, s->num_queues);
    for (i = 0; i < s->num_queues; i++) {
        s->tx_queues[i].qid = i;
        s->tx_queues[i].dir = MCDMA_DIR_D2H;
        s->tx_queues[i].ring_entries = 0;
    }

    /* PCI config */
    pci_config_set_vendor_id(pci_dev->config, MCDMA_CRYPTO_VENDOR_ID);
    pci_config_set_device_id(pci_dev->config, MCDMA_CRYPTO_DEVICE_ID);
    pci_config_set_revision(pci_dev->config, MCDMA_CRYPTO_REVISION);
    pci_config_set_class(pci_dev->config, MCDMA_CRYPTO_CLASS_CODE);
    pci_config_set_prog_interface(pci_dev->config, 0x00);
    pci_dev->config[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL;
    pci_dev->config[PCI_COMMAND] =
        PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;

    /* BAR0: QCSR + global registers (4 MB) */
    memory_region_init_io(&s->bar0, OBJECT(s), &mcdma_crypto_bar0_ops, s,
                          "mcdma-crypto-bar0", MCDMA_BAR0_SIZE);
    pci_register_bar(pci_dev, MCDMA_CONFIG_BAR,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->bar0);

    /* BAR2: PIO stub (64 KB) */
    memory_region_init_io(&s->bar2, OBJECT(s), &mcdma_crypto_bar2_ops, s,
                          "mcdma-crypto-bar2", MCDMA_BAR2_SIZE);
    pci_register_bar(pci_dev, MCDMA_PIO_BAR,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->bar2);

    /* BAR4: BAS stub (64 KB) */
    memory_region_init_io(&s->bar4, OBJECT(s), &mcdma_crypto_bar4_ops, s,
                          "mcdma-crypto-bar4", MCDMA_BAR4_SIZE);
    pci_register_bar(pci_dev, MCDMA_BAS_BAR,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->bar4);

    /* MSI-X: 4 vectors above the QCSR region in BAR0 */
    ret = msix_init(pci_dev, MCDMA_CRYPTO_NR_VECTORS,
                    &s->bar0, MCDMA_CONFIG_BAR, 0x300000,
                    &s->bar0, MCDMA_CONFIG_BAR, 0x300100,
                    0, NULL);
    if (ret < 0) {
        error_setg(errp, "mcdma-crypto: msix_init failed: %d", ret);
        return;
    }
    s->msix_initialized = true;
    s->nr_vectors = MCDMA_CRYPTO_NR_VECTORS;
    for (i = 0; i < MCDMA_CRYPTO_NR_VECTORS; i++) {
        msix_vector_use(pci_dev, i);
    }

    /* Bottom-half for async descriptor processing */
    s->bh = qemu_bh_new(mcdma_crypto_bh_cb, s);
}

static void
mcdma_crypto_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize   = mcdma_crypto_realize;
    k->vendor_id = MCDMA_CRYPTO_VENDOR_ID;
    k->device_id = MCDMA_CRYPTO_DEVICE_ID;
    k->revision  = MCDMA_CRYPTO_REVISION;
    k->class_id  = MCDMA_CRYPTO_CLASS_CODE;

    dc->desc = "MCDMA Crypto Loopback Device";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props(dc, mcdma_crypto_properties);
}

static const TypeInfo mcdma_crypto_info = {
    .name          = TYPE_MCDMA_CRYPTO,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(McdmaCryptoState),
    .instance_init = mcdma_crypto_instance_init,
    .instance_finalize = mcdma_crypto_instance_finalize,
    .class_init    = mcdma_crypto_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static void
mcdma_crypto_register_types(void)
{
    type_register_static(&mcdma_crypto_info);
}

type_init(mcdma_crypto_register_types);
