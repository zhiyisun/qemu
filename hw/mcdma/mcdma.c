#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "exec/memory.h"
#include "qapi/error.h"
#ifdef CONFIG_TRACE_MCDMA
#include "trace.h"
#else
#define trace_mcdma_reg_read(addr, val)         ((void)0)
#define trace_mcdma_reg_write(addr, val)        ((void)0)
#define trace_mcdma_bh_scheduled()              ((void)0)
#define trace_mcdma_bh_execute()                ((void)0)
#define trace_mcdma_queue_enable(qid, dir)      ((void)0)
#define trace_mcdma_queue_disable(qid, dir)     ((void)0)
#define trace_mcdma_queue_reset(qid, dir)       ((void)0)
#define trace_mcdma_msix_notify(vector)         ((void)0)
#endif
#include "mcdma.h"
#include "mcdma_regs.h"

#define MCDMA_NR_VECTORS      4

static uint64_t mcdma_bar0_read(void *opaque, hwaddr addr, unsigned size)
{
    McdmaState *s = MCDMA(opaque);
    uint32_t val = 0;
    uint32_t qid;
    hwaddr reg;
    McdmaQueue *q;

    /* RX queue region: 0x000000 - 0x07FFFF */
    if (addr < MCDMA_TX_Q_OFFSET) {
        qid = addr / MCDMA_QUEUE_CSR_SIZE;
        reg = addr % MCDMA_QUEUE_CSR_SIZE;
        if (qid >= s->num_queues) {
            goto out;
        }
        q = &s->rx_queues[qid];
        switch (reg) {
        case Q_CTRL:
            val = q->ctrl;
            break;
        case Q_START_ADDR_L:
            val = (uint32_t)(q->ring_base & 0xFFFFFFFF);
            break;
        case Q_START_ADDR_H:
            val = (uint32_t)(q->ring_base >> 32);
            break;
        case Q_SIZE:
            val = q->ring_size_log2;
            break;
        case Q_TAIL_POINTER:
            val = q->tail & Q_TAIL_PTR_MASK;
            break;
        case Q_HEAD_POINTER:
            val = q->head & Q_HEAD_PTR_MASK;
            break;
        case Q_COMPLETED_POINTER:
            val = q->completed & Q_COMPLETED_PTR_MASK;
            break;
        case Q_CONSUMED_HEAD_ADDR_L:
            val = (uint32_t)(q->wb_addr & 0xFFFFFFFF);
            break;
        case Q_CONSUMED_HEAD_ADDR_H:
            val = (uint32_t)(q->wb_addr >> 32);
            break;
        default:
            break;
        }
        goto out;
    }

    /* TX queue region: 0x080000 - 0x1FFFFF */
    if (addr >= MCDMA_TX_Q_OFFSET && addr < MCDMA_GLOBAL_OFFSET) {
        hwaddr tx_base = addr - MCDMA_TX_Q_OFFSET;
        qid = tx_base / MCDMA_QUEUE_CSR_SIZE;
        reg = tx_base % MCDMA_QUEUE_CSR_SIZE;
        if (qid >= s->num_queues) {
            goto out;
        }
        q = &s->tx_queues[qid];
        switch (reg) {
        case Q_CTRL:
            val = q->ctrl;
            break;
        case Q_START_ADDR_L:
            val = (uint32_t)(q->ring_base & 0xFFFFFFFF);
            break;
        case Q_START_ADDR_H:
            val = (uint32_t)(q->ring_base >> 32);
            break;
        case Q_SIZE:
            val = q->ring_size_log2;
            break;
        case Q_TAIL_POINTER:
            val = q->tail & Q_TAIL_PTR_MASK;
            break;
        case Q_HEAD_POINTER:
            val = q->head & Q_HEAD_PTR_MASK;
            break;
        case Q_COMPLETED_POINTER:
            val = q->completed & Q_COMPLETED_PTR_MASK;
            break;
        case Q_CONSUMED_HEAD_ADDR_L:
            val = (uint32_t)(q->wb_addr & 0xFFFFFFFF);
            break;
        case Q_CONSUMED_HEAD_ADDR_H:
            val = (uint32_t)(q->wb_addr >> 32);
            break;
        case Q_CPL_TIMEOUT:
            val = 0;
            break;
        default:
            break;
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
    trace_mcdma_reg_read(addr, val);
    return val;
}

static void mcdma_bar0_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    McdmaState *s = MCDMA(opaque);
    uint32_t val = data;
    uint32_t qid;
    hwaddr reg;
    McdmaQueue *q;

    trace_mcdma_reg_write(addr, val);

    /* RX queue region */
    if (addr < MCDMA_TX_Q_OFFSET) {
        qid = addr / MCDMA_QUEUE_CSR_SIZE;
        reg = addr % MCDMA_QUEUE_CSR_SIZE;
        if (qid >= s->num_queues) {
            return;
        }
        q = &s->rx_queues[qid];
        switch (reg) {
        case Q_CTRL:
            q->ctrl = val;
            q->enabled = (val & Q_CTRL_Q_EN) != 0;
            if (q->enabled) {
                trace_mcdma_queue_enable(qid, MCDMA_DIR_H2D);
            } else {
                trace_mcdma_queue_disable(qid, MCDMA_DIR_H2D);
            }
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
            if (q->enabled) {
                if (!s->bh_scheduled) {
                    s->bh_scheduled = true;
                    qemu_bh_schedule(s->bh);
                    trace_mcdma_bh_scheduled();
                }
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
                trace_mcdma_queue_reset(qid, MCDMA_DIR_H2D);
                mcdma_descq_reset(q);
            }
            break;
        default:
            break;
        }
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
            if (q->enabled) {
                trace_mcdma_queue_enable(qid, MCDMA_DIR_D2H);
            } else {
                trace_mcdma_queue_disable(qid, MCDMA_DIR_D2H);
            }
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
            if (q->enabled) {
                if (!s->bh_scheduled) {
                    s->bh_scheduled = true;
                    qemu_bh_schedule(s->bh);
                    trace_mcdma_bh_scheduled();
                }
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
                trace_mcdma_queue_reset(qid, MCDMA_DIR_D2H);
                mcdma_descq_reset(q);
            }
            break;
        default:
            break;
        }
        return;
    }
}

static const MemoryRegionOps mcdma_bar0_ops = {
    .read = mcdma_bar0_read,
    .write = mcdma_bar0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mcdma_bar2_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void mcdma_bar2_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
}

static const MemoryRegionOps mcdma_bar2_ops = {
    .read = mcdma_bar2_read,
    .write = mcdma_bar2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mcdma_bar4_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void mcdma_bar4_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
}

static const MemoryRegionOps mcdma_bar4_ops = {
    .read = mcdma_bar4_read,
    .write = mcdma_bar4_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

void mcdma_process_descq(McdmaState *s)
{
    uint32_t i;

    trace_mcdma_bh_execute();
    s->bh_scheduled = false;

    for (i = 0; i < s->num_queues; i++) {
        McdmaQueue *rxq = &s->rx_queues[i];
        McdmaQueue *txq = &s->tx_queues[i];

        if (rxq->enabled && txq->enabled) {
            uint32_t saved_head = txq->head;
            bool rx_empty = (rxq->head == rxq->tail);

            mcdma_descq_process_loopback(s, rxq, txq);

            if (rx_empty && saved_head == txq->head && txq->head != txq->tail) {
                continue;
            }
        }
        if (txq->enabled) {
            mcdma_descq_process_tx(s, txq);
        }
        if (rxq->enabled) {
            mcdma_descq_process_rx(s, rxq);
        }
    }
}

static void mcdma_bh_cb(void *opaque)
{
    McdmaState *s = MCDMA(opaque);
    mcdma_process_descq(s);
}

static Property mcdma_properties[] = {
    DEFINE_PROP_UINT32("num_queues", McdmaState, num_queues, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void mcdma_instance_init(Object *obj)
{
    McdmaState *s = MCDMA(obj);

    s->num_queues = 1;
    s->bh = NULL;
    s->bh_scheduled = false;
    s->msix_initialized = false;
}

static void mcdma_instance_finalize(Object *obj)
{
    McdmaState *s = MCDMA(obj);

    if (s->bh) {
        qemu_bh_delete(s->bh);
        s->bh = NULL;
    }
    if (s->msix_initialized) {
        msix_uninit(&s->parent_obj, &s->bar0, &s->bar0);
    }
    g_free(s->rx_queues);
    g_free(s->tx_queues);
}

static void mcdma_realize(PCIDevice *pci_dev, Error **errp)
{
    McdmaState *s = MCDMA(pci_dev);
    int ret;
    uint32_t i;

    /* Allocate queue arrays */
    s->rx_queues = g_new0(McdmaQueue, s->num_queues);
    s->tx_queues = g_new0(McdmaQueue, s->num_queues);
    for (i = 0; i < s->num_queues; i++) {
        s->rx_queues[i].qid = i;
        s->rx_queues[i].dir = MCDMA_DIR_H2D;
        s->rx_queues[i].ring_entries = 0;
        s->tx_queues[i].qid = i;
        s->tx_queues[i].dir = MCDMA_DIR_D2H;
        s->tx_queues[i].ring_entries = 0;
    }

    /* PCI config space setup */
    pci_config_set_vendor_id(pci_dev->config, MCDMA_VENDOR_ID);
    pci_config_set_device_id(pci_dev->config, MCDMA_DEVICE_ID);
    pci_config_set_revision(pci_dev->config, MCDMA_REVISION);
    pci_config_set_class(pci_dev->config, MCDMA_CLASS_CODE);
    pci_config_set_prog_interface(pci_dev->config, 0x00);
    pci_dev->config[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL;
    pci_dev->config[PCI_COMMAND] =
        PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;

    /* BAR0: QCSR + global registers */
    memory_region_init_io(&s->bar0, OBJECT(s), &mcdma_bar0_ops, s,
                          "mcdma-bar0", MCDMA_BAR0_SIZE);
    pci_register_bar(pci_dev, MCDMA_CONFIG_BAR,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->bar0);

    /* BAR2: PIO stub */
    memory_region_init_io(&s->bar2, OBJECT(s), &mcdma_bar2_ops, s,
                          "mcdma-bar2", MCDMA_BAR2_SIZE);
    pci_register_bar(pci_dev, MCDMA_PIO_BAR,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->bar2);

    /* BAR4: BAS stub */
    memory_region_init_io(&s->bar4, OBJECT(s), &mcdma_bar4_ops, s,
                          "mcdma-bar4", MCDMA_BAR4_SIZE);
    pci_register_bar(pci_dev, MCDMA_BAS_BAR,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->bar4);

    /* MSI-X: 4 vectors at high BAR0 offset (above QCSR + global regs) */
    ret = msix_init(pci_dev, MCDMA_NR_VECTORS,
                    &s->bar0, MCDMA_CONFIG_BAR, 0x300000,
                    &s->bar0, MCDMA_CONFIG_BAR, 0x300100,
                    0, NULL);
    if (ret < 0) {
        error_setg(errp, "msix_init failed: %d", ret);
        return;
    }
    s->msix_initialized = true;
    s->nr_vectors = MCDMA_NR_VECTORS;
    msix_vector_use(pci_dev, 0);
    msix_vector_use(pci_dev, 1);
    msix_vector_use(pci_dev, 2);
    msix_vector_use(pci_dev, 3);

    /* Bottom-half for async descriptor processing */
    s->bh = qemu_bh_new(mcdma_bh_cb, s);
}

static void mcdma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = mcdma_realize;
    k->vendor_id = MCDMA_VENDOR_ID;
    k->device_id = MCDMA_DEVICE_ID;
    k->revision = MCDMA_REVISION;
    k->class_id = MCDMA_CLASS_CODE;

    dc->desc = "MCDMA Loopback Device";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    device_class_set_props(dc, mcdma_properties);
}

static const TypeInfo mcdma_info = {
    .name = TYPE_MCDMA,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(McdmaState),
    .instance_init = mcdma_instance_init,
    .instance_finalize = mcdma_instance_finalize,
    .class_init = mcdma_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static void mcdma_register_types(void)
{
    type_register_static(&mcdma_info);
}

type_init(mcdma_register_types);
