/*
 * Qualcomm WCN7850 PCIe WiFi 7 device model for QEMU
 *
 * Emulates the WCN7850 PCIe endpoint enough that ath12k probes,
 * MHI reaches mission mode, QRTR/QMI handshake completes,
 * and wlan0 is created in the guest.
 */

#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/pci/pci_device.h"
#include "net/eth.h"
#include "net/net.h"
#include "net/tap.h"
#include "system/address-spaces.h"

#define WCN7850_VENDOR_ID 0x17cb
#define WCN7850_DEVICE_ID 0x1107
#define WCN7850_BAR0_SIZE (2 * 1024 * 1024)
#define WCN7850_WINDOW_START 0x80000
#define WCN7850_WINDOW_SIZE 0x80000
#define WCN7850_WINDOW_REG 0x310c
#define WCN7850_TCSR_SOC_HW_VERSION 0x1b00000
#define WCN7850_WINDOW_TCSR_SEL 0x36

#define WCN7850_MHIREGLEN 0x100
#define WCN7850_MHICFG 0x10
#define WCN7850_CHDBOFF 0x18
#define WCN7850_ERDBOFF 0x20
#define WCN7850_BHIOFF 0x28
#define WCN7850_BHIEOFF 0x2c
#define WCN7850_MHICTRL 0x38
#define WCN7850_MHISTATUS 0x48
#define WCN7850_CRDB_LOWER 0x70

#define WCN7850_BHI_BASE 0x200
#define WCN7850_BHI_EXECENV (WCN7850_BHI_BASE + 0x28)
#define WCN7850_BHI_STATUS (WCN7850_BHI_BASE + 0x2c)
#define WCN7850_BHI_INTVEC (WCN7850_BHI_BASE + 0x20)

#define WCN7850_MHICFG_VALUE ((2u << 16) | (2u << 8) | 30u)
#define WCN7850_BHIOFF_VALUE 0x200
#define WCN7850_BHIEOFF_VALUE 0x240

#define WCN7850_CHDBOFF_VALUE 0x4000
#define WCN7850_ERDBOFF_VALUE 0x5000

#define WCN7850_MSI_VECTORS 16
#define WCN7850_CE_COUNT 9

#define MHI_DEV_WAKE_DB 127

/* MHI channel definitions for WCN7850 */
#define MHI_CHAN_IPCR_UL 20
#define MHI_CHAN_IPCR_DL 21
#define MHI_CHAN_MAX 128

/* ===== SRNG register offsets (WCN7850 UMAC) ===== */
#define WCN7850_SRNG_STRIDE        0x78
#define WCN7850_NUM_TCL_RINGS      6
#define WCN7850_NUM_REO_RINGS      9
#define WCN7850_NUM_WBM_RINGS      8
#define WCN7850_NUM_RINGS_TRACKED  32

/* TCL source ring R0 registers (per-ring, stride 0x78) */
#define WCN7850_TCL_RING_BASE_LSB(_n)   (0x00000900 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_BASE_MSB(_n)   (0x00000904 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_ID(_n)         (0x00000908 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_MISC(_n)       (0x00000910 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_TP_ADDR_LSB(_n) (0x0000091c + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_TP_ADDR_MSB(_n) (0x00000920 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_MSI1_BASE_LSB(_n) (0x00000948 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_MSI1_BASE_MSB(_n) (0x0000094c + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_TCL_RING_MSI1_DATA(_n)     (0x00000950 + (_n) * WCN7850_SRNG_STRIDE)
/* TCL R2 (pointer) registers */
#define WCN7850_TCL_RING_HP(_n)     (0x00002000 + (_n) * 4)
#define WCN7850_TCL_RING_TP(_n)     (0x00002004 + (_n) * 4)

/* REO destination ring R0 registers (per-ring, stride 0x78) */
#define WCN7850_REO_RING_BASE_LSB(_n)   (0x000004e4 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_BASE_MSB(_n)   (0x000004e8 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_ID(_n)         (0x000004ec + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_MISC(_n)       (0x000004f4 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_HP_ADDR_LSB(_n) (0x000004f8 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_HP_ADDR_MSB(_n) (0x000004fc + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_PROD_INT(_n)   (0x00000508 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_MSI1_BASE_LSB(_n) (0x0000052c + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_MSI1_BASE_MSB(_n) (0x00000530 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_REO_RING_MSI1_DATA(_n)     (0x00000534 + (_n) * WCN7850_SRNG_STRIDE)
/* REO R2 (pointer) registers */
#define WCN7850_REO_RING_HP(_n)     (0x00003048 + (_n) * 4)
#define WCN7850_REO_RING_TP(_n)     (0x0000304c + (_n) * 4)

/* REO exception ring (REO2SW0) uses HP at different offset */
#define WCN7850_REO_EXC_RING_HP    0x00003088
#define WCN7850_REO_EXC_RING_TP    0x0000308c

/* WBM release ring HP */
#define WCN7850_WBM0_REL_RING_HP   0x000030c8

/* RXDMA refill ring config base (SW2RXDMA) */
#define WCN7850_RXDMA_BUF_BASE_LSB 0x00000e08

/* Buffer pool constants */
#define WCN7850_POOL_SIZE          (64 * 1024 * 1024)
#define WCN7850_POOL_BUF_SIZE      2304
#define WCN7850_POOL_NUM_BUFS      (WCN7850_POOL_SIZE / WCN7850_POOL_BUF_SIZE)
#define WCN7850_POOL_PADDR         0x100000000ULL

/* Exception ring entry */
typedef struct {
    uint64_t buf_paddr;
    uint32_t length;
    uint32_t flags;
} QEMU_PACKED Wcn7850ExcEntry;

/* Ring type identifiers */
typedef enum {
    WCN7850_RING_NONE = 0,
    WCN7850_RING_TCL_DATA,
    WCN7850_RING_REO_DST,
    WCN7850_RING_REO_EXCEPTION,
    WCN7850_RING_WBM2SW_RELEASE,
    WCN7850_RING_RXDMA_BUF,
    WCN7850_RING_SW2WBM_RELEASE,
    WCN7850_RING_UNKNOWN,
} Wcn7850RingType;

/* Per-ring tracked state */
typedef struct {
    bool configured;
    Wcn7850RingType type;
    uint32_t ring_id;
    uint64_t base_addr;
    uint32_t size;
    uint32_t entry_size;
    uint32_t hp;
    uint32_t tp;
    uint64_t hp_shadow_addr;
    uint64_t tp_shadow_addr;
    bool hp_is_mmio;
    bool tp_is_mmio;
    uint32_t hp_mmio_offset;
    uint32_t tp_mmio_offset;
    uint32_t msivec;
    bool enable;
} WCN7850RingState;

/* QRTR protocol constants */
#define QRTR_PROTO_VER_1 1
#define QRTR_TYPE_DATA 1
#define QRTR_TYPE_NEW_SERVER 4
#define QRTR_PORT_CTRL 0xfffffffeu
#define QRTR_NODE_HOST 1
#define QRTR_NODE_FW 2
#define QRTR_PORT_QMI_SERVICE 1

/* QMI service constants */
#define QMI_WLFW_SERVICE_ID 0x45
#define QMI_WLFW_SERVICE_INSTANCE 0x01

/* MHI event ring constants */
#define MHI_PKT_TYPE_STATE_CHANGE_EVENT 0x20
#define MHI_PKT_TYPE_EE_EVENT 0x40
#define MHI_PKT_TYPE_TRANSFER_EVENT 0x02

/* Channel context structure (44 bytes) */
typedef struct {
    uint32_t chcfg;
    uint32_t chtype;
    uint32_t erindex;
    uint64_t rbase;
    uint64_t rlen;
    uint64_t rp;
    uint64_t wp;
} QEMU_PACKED MhiChanCtxt;

/* Event ring context structure (48 bytes) */
typedef struct {
    uint32_t intmod;
    uint32_t ertype;
    uint32_t msivec;
    uint64_t rbase;
    uint64_t rlen;
    uint64_t rp;
    uint64_t wp;
} QEMU_PACKED MhiEventCtxt;

/* MHI ring element / TRE (16 bytes) */
typedef struct {
    uint64_t ptr;
    uint32_t dword0;
    uint32_t dword1;
} QEMU_PACKED MhiTre;

/* QRTR v1 header (32 bytes) */
typedef struct {
    uint32_t version;
    uint32_t type;
    uint32_t src_node_id;
    uint32_t src_port_id;
    uint32_t confirm_rx;
    uint32_t size;
    uint32_t dst_node_id;
    uint32_t dst_port_id;
} QEMU_PACKED QrtrHdrV1;

/* QRTR control packet */
typedef struct {
    uint32_t cmd;
    uint32_t server_service;
    uint32_t server_instance;
    uint32_t server_node;
    uint32_t server_port;
} QEMU_PACKED QrtrCtrlPkt;

/* QMI header */
typedef struct {
    uint8_t type;
    uint16_t txn_id;
    uint16_t msg_id;
    uint16_t msg_len;
} QEMU_PACKED QmiHdr;

/* QMI TLV */
typedef struct {
    uint8_t type;
    uint16_t len;
} QEMU_PACKED QmiTlvHdr;

typedef struct WCN7850State {
    PCIDevice parent_obj;
    MemoryRegion bar0_mmio;
    uint8_t *bar0_always_on;
    uint8_t *window_memory;
    uint32_t window_select;
    bool bhi_downloaded;
    uint32_t bhi_status;
    uint32_t bhi_execenv;
    uint32_t mhi_state;
    uint64_t er_ctxt_addr;
    bool ctrl_event_pending;
    QEMUTimer *ctrl_event_timer;
    uint8_t *srng_memory;

    struct {
        uint8_t *src_ring;
        uint8_t *dst_ring;
        uint8_t *status_ring;
    } ce[WCN7850_CE_COUNT];

    char *fw_path;

    /* Context base addresses written by host */
    uint64_t chan_ctxt_addr;
    uint64_t cmd_ctxt_addr;

    /* QRTR node/port for firmware QMI service */
    bool qmi_service_active;
    bool qmi_boot_pending;

    /* ===== FPGA data path emulation (Phase 3) ===== */

    /* Network backend */
    NICState *nic;
    NICConf conf;
    bool link_up;

    /* Buffer pool (64 MB carve-out emulated as RAM region) */
    MemoryRegion pool_mr;
    uint8_t *pool_mem;
    uint64_t pool_paddr;
    uint64_t pool_next_alloc;
    uint64_t pool_num_bufs;

    /* SRNG ring tracking */
    WCN7850RingState rings[WCN7850_NUM_RINGS_TRACKED];
    int num_rings;

    /* REO state for RX processing */
    int reo_dst_ring_idx;
    int reo_exc_ring_idx;

    /* RX TCL data ring index */
    int tcl_data_ring_idx;

    /* RX pending buffer for postponed processing */
    uint8_t *rx_pending_buf;
    uint32_t rx_pending_len;
    bool rx_pending;

    /* TX packet processing */
    uint8_t *tx_buffer;

    /* Configurable MAC address */
    uint8_t mac_addr[6];
} WCN7850State;

#define TYPE_WCN7850 "wcn7850"
DECLARE_INSTANCE_CHECKER(WCN7850State, WCN7850, TYPE_WCN7850)

/* Forward declarations */
static void wcn7850_process_ul_data(WCN7850State *s, PCIDevice *pci_dev,
                                     uint64_t ch_ctxt_addr, uint32_t ch_id,
                                     uint64_t er_ctxt_addr);
static bool wcn7850_mission_mode_setup(WCN7850State *s, PCIDevice *pci_dev);
static void wcn7850_try_process_rx(WCN7850State *s, PCIDevice *pci_dev);
static void wcn7850_process_tcl_data(WCN7850State *s, PCIDevice *pci_dev,
                                      WCN7850RingState *tcl);

/* Read a channel context from host DMA memory */
static bool wcn7850_read_chan_ctxt(PCIDevice *pci_dev, uint64_t ctxt_addr,
                                    MhiChanCtxt *ctxt)
{
    if (!ctxt_addr)
        return false;
    if (pci_dma_read(pci_dev, ctxt_addr, ctxt, sizeof(*ctxt)) != MEMTX_OK)
        return false;
    ctxt->rbase = le64_to_cpu(ctxt->rbase);
    ctxt->rlen = le64_to_cpu(ctxt->rlen);
    ctxt->rp = le64_to_cpu(ctxt->rp);
    ctxt->wp = le64_to_cpu(ctxt->wp);
    ctxt->erindex = le32_to_cpu(ctxt->erindex);
    return true;
}

/* Write a channel context back to host DMA memory */
static void wcn7850_write_chan_ctxt(PCIDevice *pci_dev, uint64_t ctxt_addr,
                                     const MhiChanCtxt *ctxt)
{
    MhiChanCtxt tmp = *ctxt;
    tmp.rbase = cpu_to_le64(tmp.rbase);
    tmp.rlen = cpu_to_le64(tmp.rlen);
    tmp.rp = cpu_to_le64(tmp.rp);
    tmp.wp = cpu_to_le64(tmp.wp);
    tmp.erindex = cpu_to_le32(tmp.erindex);
    (void)pci_dma_write(pci_dev, ctxt_addr, &tmp, sizeof(tmp));
}

/* Read a single TRE from the ring at a given offset from rbase */
static bool wcn7850_read_tre(PCIDevice *pci_dev, uint64_t rbase, uint64_t offset,
                              MhiTre *tre)
{
    if (pci_dma_read(pci_dev, rbase + offset, tre, sizeof(*tre)) != MEMTX_OK)
        return false;
    tre->ptr = le64_to_cpu(tre->ptr);
    tre->dword0 = le32_to_cpu(tre->dword0);
    tre->dword1 = le32_to_cpu(tre->dword1);
    return true;
}

/* Read an event ring context from host DMA memory */
static bool wcn7850_read_er_ctxt(PCIDevice *pci_dev, uint64_t er_ctxt_addr,
                                  uint64_t index, MhiEventCtxt *ectxt)
{
    uint64_t addr = er_ctxt_addr + index * sizeof(*ectxt);
    if (pci_dma_read(pci_dev, addr, ectxt, sizeof(*ectxt)) != MEMTX_OK)
        return false;
    ectxt->rbase = le64_to_cpu(ectxt->rbase);
    ectxt->rlen = le64_to_cpu(ectxt->rlen);
    ectxt->rp = le64_to_cpu(ectxt->rp);
    ectxt->wp = le64_to_cpu(ectxt->wp);
    ectxt->msivec = le32_to_cpu(ectxt->msivec);
    return true;
}

/* Write an event ring context back to host DMA memory */
static void wcn7850_write_er_ctxt(PCIDevice *pci_dev, uint64_t er_ctxt_addr,
                                   uint64_t index, const MhiEventCtxt *ectxt)
{
    MhiEventCtxt tmp = *ectxt;
    tmp.rbase = cpu_to_le64(tmp.rbase);
    tmp.rlen = cpu_to_le64(tmp.rlen);
    tmp.rp = cpu_to_le64(tmp.rp);
    tmp.wp = cpu_to_le64(tmp.wp);
    tmp.msivec = cpu_to_le32(tmp.msivec);
    uint64_t addr = er_ctxt_addr + index * sizeof(tmp);
    (void)pci_dma_write(pci_dev, addr, &tmp, sizeof(tmp));
}

/* Post an event to an event ring and send MSI */
static void wcn7850_post_event(WCN7850State *s, PCIDevice *pci_dev,
                                uint64_t er_ctxt_addr, uint32_t er_index,
                                void *ev_buf, size_t ev_size)
{
    MhiEventCtxt ectxt;
    uint64_t next_rp;

    if (!wcn7850_read_er_ctxt(pci_dev, er_ctxt_addr, er_index, &ectxt))
        return;
    if (!ectxt.rbase || !ectxt.rlen)
        return;

    if (ectxt.rp < ectxt.rbase || ectxt.rp >= ectxt.rbase + ectxt.rlen)
        ectxt.rp = ectxt.rbase;

    /* Write event at current rp */
    if (pci_dma_write(pci_dev, ectxt.rp, ev_buf, ev_size) != MEMTX_OK)
        return;

    /* Advance rp to next slot */
    next_rp = ectxt.rp + ev_size;
    if (next_rp >= ectxt.rbase + ectxt.rlen)
        next_rp = ectxt.rbase;
    ectxt.rp = next_rp;

    /* Write back updated context */
    wcn7850_write_er_ctxt(pci_dev, er_ctxt_addr, er_index, &ectxt);

    /* Send MSI for this event ring */
    if (ectxt.msivec < WCN7850_MSI_VECTORS) {
        qemu_log("WCN: post_event er=%u msivec=%u rp=0x%lx\n",
                 er_index, ectxt.msivec, (unsigned long)ectxt.rp);
        msi_notify(pci_dev, ectxt.msivec);
    }
}

/* Post a transfer completion event for a channel */
static void wcn7850_post_transfer_event(WCN7850State *s, PCIDevice *pci_dev,
                                         uint64_t er_ctxt_addr,
                                         uint32_t er_index,
                                         uint32_t ch_id,
                                         uint64_t buf_ptr, uint32_t len)
{
    uint8_t ev[16] = { 0 };
    uint8_t dword1;

    /* Transfer event: ptr=buf address, dword0=len, dword1=type << 16 */
    memcpy(ev, &(uint64_t){ cpu_to_le64(buf_ptr) }, sizeof(uint64_t));
    memcpy(ev + 8, &(uint32_t){ cpu_to_le32(len) }, sizeof(uint32_t));
    dword1 = MHI_PKT_TYPE_TRANSFER_EVENT << 16 | (ch_id & 0xff);
    memcpy(ev + 12, &(uint32_t){ cpu_to_le32(dword1) }, sizeof(uint32_t));

    qemu_log("WCN: transfer_event ch=%u ptr=0x%lx len=%u er=%u\n",
             ch_id, (unsigned long)buf_ptr, len, er_index);
    wcn7850_post_event(s, pci_dev, er_ctxt_addr, er_index, ev, sizeof(ev));
}

/* Build a QRTR v1 packet and return the total size */
static size_t wcn7850_build_qrtr_pkt(uint8_t *buf, size_t buf_size,
                                      uint32_t type,
                                      uint32_t src_node, uint32_t src_port,
                                      uint32_t dst_node, uint32_t dst_port,
                                      const void *payload, uint32_t payload_size)
{
    QrtrHdrV1 *hdr = (QrtrHdrV1 *)buf;
    size_t total = sizeof(*hdr) + payload_size;

    if (total > buf_size)
        return 0;

    memset(hdr, 0, sizeof(*hdr));
    hdr->version = cpu_to_le32(QRTR_PROTO_VER_1);
    hdr->type = cpu_to_le32(type);
    hdr->src_node_id = cpu_to_le32(src_node);
    hdr->src_port_id = cpu_to_le32(src_port);
    hdr->size = cpu_to_le32(payload_size);
    hdr->dst_node_id = cpu_to_le32(dst_node);
    hdr->dst_port_id = cpu_to_le32(dst_port);

    if (payload && payload_size)
        memcpy(buf + sizeof(*hdr), payload, payload_size);

    return total;
}

/* Write QRTR packet + optional payload to a DL channel buffer in host memory.
 * Uses erindex from channel context; the er_index parameter is unused. */
static bool wcn7850_write_dl_data(WCN7850State *s, PCIDevice *pci_dev,
                                   uint64_t ch_ctxt_addr,
                                   uint64_t er_ctxt_addr,
                                   uint32_t er_index,
                                   const void *data, size_t data_len)
{
    MhiChanCtxt ctxt;
    MhiTre tre;

    fprintf(stderr, "WCN: write_dl_data ch_ctxt=0x%lx\n",
            (unsigned long)ch_ctxt_addr);
    if (!wcn7850_read_chan_ctxt(pci_dev, ch_ctxt_addr, &ctxt)) {
        fprintf(stderr, "WCN: write_dl_data read ctxt FAILED\n");
        return false;
    }

    fprintf(stderr, "WCN: write_dl_data rbase=0x%lx rlen=0x%lx rp=0x%lx wp=0x%lx erindex=%u\n",
            (unsigned long)ctxt.rbase, (unsigned long)ctxt.rlen,
            (unsigned long)ctxt.rp, (unsigned long)ctxt.wp, ctxt.erindex);
    if (!ctxt.rbase || !ctxt.rlen) {
        fprintf(stderr, "WCN: write_dl_data ring not set up\n");
        return false;
    }

    /* Validate rp/wp */
    if (ctxt.rp < ctxt.rbase || ctxt.rp >= ctxt.rbase + ctxt.rlen)
        ctxt.rp = ctxt.rbase;
    if (ctxt.wp < ctxt.rbase || ctxt.wp >= ctxt.rbase + ctxt.rlen)
        ctxt.wp = ctxt.rbase;

    /* Check if ring is empty */
    if (ctxt.rp == ctxt.wp) {
        fprintf(stderr, "WCN: write_dl_data ring empty (rp==wp)\n");
        return false;
    }

    /* Read the next TRE at rp (this is the buffer the host pre-queued) */
    uint64_t tre_offset = ctxt.rp - ctxt.rbase;
    if (!wcn7850_read_tre(pci_dev, ctxt.rbase, tre_offset, &tre)) {
        fprintf(stderr, "WCN: write_dl_data read TRE FAILED\n");
        return false;
    }

    /* Cap write to buffer length from TRE's dword0 */
    uint32_t buf_len = tre.dword0 & 0xffffu;
    size_t write_len = (buf_len && data_len > buf_len) ? buf_len : data_len;

    /* DMA-write our data into the buffer pointed to by the TRE */
    if (pci_dma_write(pci_dev, tre.ptr, data, write_len) != MEMTX_OK)
        return false;

    /* Advance rp past this TRE */
    ctxt.rp += sizeof(MhiTre);
    if (ctxt.rp >= ctxt.rbase + ctxt.rlen)
        ctxt.rp = ctxt.rbase;

    wcn7850_write_chan_ctxt(pci_dev, ch_ctxt_addr, &ctxt);

    /* Post transfer completion event with actual written length.
     * Use erindex from channel context, not the parameter. */
    wcn7850_post_transfer_event(s, pci_dev, er_ctxt_addr, ctxt.erindex,
                                 MHI_CHAN_IPCR_DL, tre.ptr, write_len);
    return true;
}

/* ===== Ring tracking helper ===== */

static WCN7850RingState *wcn7850_find_ring(WCN7850State *s, Wcn7850RingType type,
                                             int ring_num)
{
    for (int i = 0; i < s->num_rings; i++) {
        if (s->rings[i].type == type &&
            s->rings[i].ring_id == (uint32_t)ring_num &&
            s->rings[i].configured) {
            return &s->rings[i];
        }
    }
    return NULL;
}

static WCN7850RingState *wcn7850_add_or_update_ring(WCN7850State *s,
                                                       Wcn7850RingType type,
                                                       uint32_t ring_id)
{
    for (int i = 0; i < s->num_rings; i++) {
        if (s->rings[i].type == type && s->rings[i].ring_id == ring_id) {
            return &s->rings[i];
        }
    }
    if (s->num_rings < WCN7850_NUM_RINGS_TRACKED) {
        WCN7850RingState *r = &s->rings[s->num_rings++];
        memset(r, 0, sizeof(*r));
        r->type = type;
        r->ring_id = ring_id;
        return r;
    }
    return NULL;
}

/* Parse SRNG register writes and update ring state from shadow registers */
static void wcn7850_update_tcl_ring_cfg(WCN7850State *s, int n)
{
    uint32_t base_lsb, base_msb, ring_id_reg, misc;
    WCN7850RingState *r = wcn7850_add_or_update_ring(s, WCN7850_RING_TCL_DATA, n);
    if (!r) return;

    base_lsb = *(uint32_t *)(s->bar0_always_on + WCN7850_TCL_RING_BASE_LSB(n));
    base_msb = *(uint32_t *)(s->bar0_always_on + WCN7850_TCL_RING_BASE_MSB(n));
    ring_id_reg = *(uint32_t *)(s->bar0_always_on + WCN7850_TCL_RING_ID(n));
    misc = *(uint32_t *)(s->bar0_always_on + WCN7850_TCL_RING_MISC(n));

    r->base_addr = ((uint64_t)(base_msb & 0xff) << 32) | base_lsb;
    r->size = (base_msb >> 8) & 0xffff;
    r->entry_size = (ring_id_reg & 0xff) * 4;
    r->hp_mmio_offset = WCN7850_TCL_RING_HP(n);
    r->tp_mmio_offset = WCN7850_TCL_RING_TP(n);
    r->hp_is_mmio = true;
    r->tp_is_mmio = true;
    r->enable = (misc & BIT(0)) != 0;
    r->configured = (r->base_addr != 0 && r->entry_size > 0 && r->size > 0);

    if (r->configured) {
        uint32_t msi_data = *(uint32_t *)(s->bar0_always_on + WCN7850_TCL_RING_MSI1_DATA(n));
        r->msivec = msi_data & 0xff;
        if (r->msivec >= WCN7850_MSI_VECTORS) {
            r->msivec = 8;
        }
        fprintf(stderr, "WCN: TCL ring %d base=0x%lx size=%u entry=%u enable=%d msivec=%u\n",
                n, (unsigned long)r->base_addr, r->size, r->entry_size, r->enable, r->msivec);
    }
}

static void wcn7850_update_reo_ring_cfg(WCN7850State *s, int n)
{
    uint32_t base_lsb, base_msb, ring_id_reg, misc;
    WCN7850RingState *r;

    if (n == 0) {
        r = wcn7850_add_or_update_ring(s, WCN7850_RING_REO_EXCEPTION, 0);
    } else {
        r = wcn7850_add_or_update_ring(s, WCN7850_RING_REO_DST, n);
    }
    if (!r) return;

    base_lsb = *(uint32_t *)(s->bar0_always_on + WCN7850_REO_RING_BASE_LSB(n));
    base_msb = *(uint32_t *)(s->bar0_always_on + WCN7850_REO_RING_BASE_MSB(n));
    ring_id_reg = *(uint32_t *)(s->bar0_always_on + WCN7850_REO_RING_ID(n));
    misc = *(uint32_t *)(s->bar0_always_on + WCN7850_REO_RING_MISC(n));

    r->base_addr = ((uint64_t)(base_msb & 0xff) << 32) | base_lsb;
    r->size = (base_msb >> 8) & 0xffff;
    r->entry_size = (ring_id_reg & 0xff) * 4;
    r->hp_mmio_offset = WCN7850_REO_RING_HP(n);
    r->tp_mmio_offset = WCN7850_REO_RING_TP(n);
    r->hp_is_mmio = true;
    r->tp_is_mmio = true;
    r->enable = (misc & BIT(0)) != 0;
    r->configured = (r->base_addr != 0 && r->entry_size > 0 && r->size > 0);

    if (r->configured) {
        uint32_t msi_data = *(uint32_t *)(s->bar0_always_on + WCN7850_REO_RING_MSI1_DATA(n));
        r->msivec = msi_data & 0xff;
        if (r->msivec >= WCN7850_MSI_VECTORS) {
            r->msivec = (n == 0) ? 10 : 9;
        }
        fprintf(stderr, "WCN: REO ring %d base=0x%lx size=%u entry=%u enable=%d msivec=%u type=%d\n",
                n, (unsigned long)r->base_addr, r->size, r->entry_size, r->enable, r->msivec, r->type);
    }
}

/* ===== Buffer pool management ===== */

static uint64_t wcn7850_pool_alloc_buf(WCN7850State *s)
{
    uint64_t offset = s->pool_next_alloc;
    if (offset + WCN7850_POOL_BUF_SIZE > WCN7850_POOL_SIZE) {
        offset = 0;
    }
    s->pool_next_alloc = offset + WCN7850_POOL_BUF_SIZE;
    return s->pool_paddr + offset;
}

/* ===== NetClient callbacks ===== */

static bool wcn7850_nc_can_receive(NetClientState *nc)
{
    WCN7850State *s = qemu_get_nic_opaque(nc);
    WCN7850RingState *reo = wcn7850_find_ring(s, WCN7850_RING_REO_DST, 1);
    return reo && reo->configured && reo->enable;
}

static ssize_t wcn7850_nc_receive_iov(NetClientState *nc, const struct iovec *iov,
                                       int iovcnt)
{
    WCN7850State *s = qemu_get_nic_opaque(nc);
    PCIDevice *pci_dev = PCI_DEVICE(s);
    int i;
    size_t total = 0;

    for (i = 0; i < iovcnt; i++) {
        total += iov[i].iov_len;
    }

    if (total > 65535) {
        return total;
    }

    g_free(s->rx_pending_buf);
    s->rx_pending_buf = g_malloc(total);
    s->rx_pending_len = 0;
    for (i = 0; i < iovcnt; i++) {
        memcpy(s->rx_pending_buf + s->rx_pending_len,
               iov[i].iov_base, iov[i].iov_len);
        s->rx_pending_len += iov[i].iov_len;
    }
    s->rx_pending = true;

    wcn7850_try_process_rx(s, pci_dev);
    return total;
}

static ssize_t wcn7850_nc_receive(NetClientState *nc, const uint8_t *buf,
                                   size_t size)
{
    struct iovec iov = {
        .iov_base = (void *)buf,
        .iov_len = size,
    };
    wcn7850_nc_receive_iov(nc, &iov, 1);
    return size;
}

static void wcn7850_set_link_status(NetClientState *nc)
{
    WCN7850State *s = qemu_get_nic_opaque(nc);
    s->link_up = !nc->link_down;
}

static NetClientInfo net_wcn7850_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = wcn7850_nc_can_receive,
    .receive = wcn7850_nc_receive,
    .receive_iov = wcn7850_nc_receive_iov,
    .link_status_changed = wcn7850_set_link_status,
};

/* ===== RX data path ===== */

/* Write received data to a guest buffer and post REO dest ring entry */
static void wcn7850_try_process_rx(WCN7850State *s, PCIDevice *pci_dev)
{
    WCN7850RingState *reo;
    uint64_t buf_paddr;
    uint32_t entry_size;
    uint32_t reo_hp, next_hp;
    uint8_t reo_entry[24];
    uint16_t eth_type;

    if (!s->rx_pending || !s->rx_pending_buf) {
        return;
    }

    /* Determine if this is an exception frame (EAPOL = ethertype 0x888E) */
    if (s->rx_pending_len >= 14) {
        eth_type = (s->rx_pending_buf[12] << 8) | s->rx_pending_buf[13];
    } else {
        eth_type = 0;
    }

    if (eth_type == 0x888E || eth_type == 0x88C7) {
        reo = wcn7850_find_ring(s, WCN7850_RING_REO_EXCEPTION, 0);
        if (!reo || !reo->configured) {
            reo = wcn7850_find_ring(s, WCN7850_RING_REO_DST, 1);
        }
    } else {
        reo = wcn7850_find_ring(s, WCN7850_RING_REO_DST, 1);
    }

    if (!reo || !reo->configured || !reo->enable) {
        qemu_log("WCN: RX no REO ring configured\n");
        s->rx_pending = false;
        return;
    }

    entry_size = reo->entry_size;
    if (entry_size < 24) entry_size = 24;

    reo_hp = reo->hp;
    next_hp = reo_hp + entry_size;
    if (next_hp >= reo->size) {
        next_hp = 0;
    }

    if (next_hp == reo->tp) {
        qemu_log("WCN: RX REO ring full\n");
        return;
    }

    buf_paddr = wcn7850_pool_alloc_buf(s);

    if (pci_dma_write(pci_dev, buf_paddr, s->rx_pending_buf,
                      s->rx_pending_len) != MEMTX_OK) {
        qemu_log("WCN: RX DMA write failed\n");
        return;
    }

    memset(reo_entry, 0, sizeof(reo_entry));
    memcpy(reo_entry, &(uint64_t){ cpu_to_le64(buf_paddr) }, sizeof(uint64_t));
    memcpy(reo_entry + 8, &(uint32_t){ cpu_to_le32(s->rx_pending_len) }, sizeof(uint32_t));
    reo_entry[12] = 0x02;

    if (pci_dma_write(pci_dev, reo->base_addr + reo_hp,
                      reo_entry, entry_size) != MEMTX_OK) {
        qemu_log("WCN: RX REO entry write failed\n");
        return;
    }

    reo->hp = next_hp;
    *(uint32_t *)(s->bar0_always_on + reo->hp_mmio_offset) = next_hp;

    if (reo->msivec < WCN7850_MSI_VECTORS) {
        msi_notify(pci_dev, reo->msivec);
    }

    fprintf(stderr, "WCN: RX pkt len=%u type=0x%04x buf=0x%lx reo_hp=%u\n",
            s->rx_pending_len, eth_type,
            (unsigned long)buf_paddr, next_hp);

    s->rx_pending = false;
}

/* ===== TX data path ===== */

static void wcn7850_process_tcl_data(WCN7850State *s, PCIDevice *pci_dev,
                                       WCN7850RingState *tcl)
{
    uint32_t entries, i;
    uint32_t entry_size = tcl->entry_size ?: 32;
    uint32_t tp = tcl->tp;
    uint32_t hp = tcl->hp;
    uint32_t ring_size = tcl->size ?: (512 * entry_size);

    if (tp > ring_size || hp > ring_size) {
        return;
    }

    if (tp <= hp) {
        entries = (hp - tp) / entry_size;
    } else {
        entries = (ring_size - tp + hp) / entry_size;
    }

    if (entries == 0) {
        return;
    }

    fprintf(stderr, "WCN: TX TCL ring %u tp=%u hp=%u entries=%u\n",
            tcl->ring_id, tp, hp, entries);

    for (i = 0; i < entries && i < 64; i++) {
        uint64_t desc_addr = tcl->base_addr + tp;
        uint8_t desc[32];
        uint64_t buf_addr;
        uint32_t buf_len;

        if (pci_dma_read(pci_dev, desc_addr, desc, sizeof(desc)) != MEMTX_OK) {
            break;
        }

        buf_addr = le64_to_cpu(*(uint64_t *)desc);
        buf_len = le32_to_cpu(*(uint32_t *)(desc + 8)) & 0xffff;

        if (!buf_addr || !buf_len) {
            tp += entry_size;
            if (tp >= ring_size) tp = 0;
            continue;
        }

        if (buf_len > 65535) buf_len = 65535;

        if (!s->tx_buffer) {
            s->tx_buffer = g_malloc(65536);
        }

        if (pci_dma_read(pci_dev, buf_addr, s->tx_buffer, buf_len) != MEMTX_OK) {
            tp += entry_size;
            if (tp >= ring_size) tp = 0;
            continue;
        }

        fprintf(stderr, "WCN: TX pkt len=%u buf=0x%lx\n",
                buf_len, (unsigned long)buf_addr);

        if (s->nic) {
            NetClientState *nc = qemu_get_queue(s->nic);
            if (nc->peer) {
                qemu_send_packet(nc->peer, s->tx_buffer, buf_len);
            }
        }

        tp += entry_size;
        if (tp >= ring_size) tp = 0;
    }

    tcl->tp = tp;
    *(uint32_t *)(s->bar0_always_on + tcl->tp_mmio_offset) = tp;
}

/* ===== QRTR/QMI handler ===== */

/*
 * QMI message IDs for ath12k-to-firmware handshake.
 * These match linux/drivers/net/wireless/ath/ath12k/qmi.h
 */
#define QMI_WLFW_PHY_CAP_REQ          0x0057
#define QMI_WLFW_IND_REGISTER_REQ     0x0020
#define QMI_WLFW_HOST_CAP_REQ         0x0034
#define QMI_WLFW_REQUEST_MEM_IND      0x0035
#define QMI_WLFW_RESPOND_MEM_REQ      0x0036
#define QMI_WLFW_FW_MEM_READY_IND     0x0037
#define QMI_WLFW_CAP_REQ              0x0024
#define QMI_WLFW_BDF_DOWNLOAD_REQ     0x0025
#define QMI_WLFW_M3_INFO_REQ          0x003C
#define QMI_WLFW_FW_READY_IND         0x0038
#define QMI_WLFW_WLAN_INI_REQ         0x002F
#define QMI_WLFW_WLAN_CFG_REQ         0x0023
#define QMI_WLFW_WLAN_MODE_REQ        0x0022

#define QMI_RESULT_SUCCESS 0
#define QMI_TYPE_RESP 2
#define QMI_TYPE_IND 4

/* Helper: write a QMI response header + TLV result into a buffer, returns total length */
static uint32_t wcn7850_qmi_build_resp(uint8_t *buf, uint32_t buf_size,
                                        uint16_t txn_id, uint16_t msg_id,
                                        uint16_t tlv_result)
{
    QmiHdr *hdr = (QmiHdr *)buf;
    uint8_t *tlv = buf + sizeof(*hdr);
    uint32_t tlv_len = tlv_result ? 7 : 0; /* TLV type(1) + len(2) + result(4) */
    uint32_t total = sizeof(*hdr) + tlv_len;

    if (total > buf_size)
        return 0;

    hdr->type = QMI_TYPE_RESP;
    hdr->txn_id = cpu_to_le16(txn_id);
    hdr->msg_id = cpu_to_le16(msg_id);
    hdr->msg_len = cpu_to_le16(tlv_len);

    if (tlv_result) {
        tlv[0] = 0x02; /* TLV type: result */
        tlv[1] = 0x04; tlv[2] = 0x00; /* TLV length = 4 */
        tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00; /* result = QMI_RESULT_SUCCESS */
    }
    return total;
}

/* Build PHY_CAP response with csr_phy_cap = single-radio */
static uint32_t wcn7850_qmi_build_phy_cap_resp(uint8_t *buf, uint32_t buf_size,
                                                 uint16_t txn_id)
{
    QmiHdr *hdr = (QmiHdr *)buf;
    uint8_t *tlv;
    uint32_t total;

    /* TLV: result (7 bytes) + num_phy (5 bytes) + board_id (5 bytes) */
    uint32_t tlv_len = 7 + 5 + 5;
    total = sizeof(*hdr) + tlv_len;
    if (total > buf_size)
        return 0;

    hdr->type = QMI_TYPE_RESP;
    hdr->txn_id = cpu_to_le16(txn_id);
    hdr->msg_id = cpu_to_le16(QMI_WLFW_PHY_CAP_REQ);
    hdr->msg_len = cpu_to_le16(tlv_len);

    tlv = buf + sizeof(*hdr);
    /* TLV type 0x02: result, length 4, value = success */
    tlv[0] = 0x02; tlv[1] = 0x04; tlv[2] = 0x00;
    tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00;
    tlv += 7;
    /* TLV type 0x16: num_phy, length 4, value = 1 */
    tlv[0] = 0x16; tlv[1] = 0x04; tlv[2] = 0x00;
    tlv[3] = 0x01; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00;
    tlv += 7;
    /* TLV type 0x1A: board_id, length 4, value = 0 */
    tlv[0] = 0x1A; tlv[1] = 0x04; tlv[2] = 0x00;
    tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00;

    return total;
}

/* Build CAP response with chip/board/fw info */
static uint32_t wcn7850_qmi_build_cap_resp(uint8_t *buf, uint32_t buf_size,
                                             uint16_t txn_id)
{
    QmiHdr *hdr = (QmiHdr *)buf;
    uint8_t *tlv;
    /* result(7) + chip_info(5+4+4+4=17) + board_info(4*4=16) + fw_ver(4*3=12) + fw_build_id */
    uint32_t tlv_len = 7 + 17 + 16 + 12;
    uint32_t total = sizeof(*hdr) + tlv_len;

    if (total > buf_size)
        return 0;

    hdr->type = QMI_TYPE_RESP;
    hdr->txn_id = cpu_to_le16(txn_id);
    hdr->msg_id = cpu_to_le16(QMI_WLFW_CAP_REQ);
    hdr->msg_len = cpu_to_le16(tlv_len);

    tlv = buf + sizeof(*hdr);
    /* result = success */
    tlv[0] = 0x02; tlv[1] = 0x04; tlv[2] = 0x00;
    tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00;
    tlv += 7;
    /* chip_info: type 0x15, len 0x0D (13): chip_id(4) + chip_family(4) + soc_id(4) + 1 byte reserved */
    tlv[0] = 0x15; tlv[1] = 0x0D; tlv[2] = 0x00;
    tlv[3] = 0x02; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00; /* chip_id = 2 */
    tlv[7] = 0x00; tlv[8] = 0x00; tlv[9] = 0x00; tlv[10] = 0x00; /* chip_family = 0 */
    tlv[11] = 0x00; tlv[12] = 0x00; tlv[13] = 0x00; tlv[14] = 0x00; /* soc_id = 0 */
    tlv[15] = 0x00; /* reserved */
    tlv += 17;
    /* board_info: type 0x16, len 0x0C (12): board_id(4) + num_macs(4) + board_id_ext(4) */
    tlv[0] = 0x16; tlv[1] = 0x0C; tlv[2] = 0x00;
    tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00; /* board_id = 0 */
    tlv[7] = 0x01; tlv[8] = 0x00; tlv[9] = 0x00; tlv[10] = 0x00; /* num_macs = 1 */
    tlv[11] = 0x00; tlv[12] = 0x00; tlv[13] = 0x00; tlv[14] = 0x00; /* board_id_ext = 0 */
    tlv += 16;
    /* fw_version: type 0x17, len 0x08 (8): fw_ver(4) + fw_build_timestamp(4) */
    tlv[0] = 0x17; tlv[1] = 0x08; tlv[2] = 0x00;
    tlv[3] = 0x01; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00; /* fw_ver = 1 */
    tlv[7] = 0x00; tlv[8] = 0x00; tlv[9] = 0x00; tlv[10] = 0x00; /* timestamp = 0 */

    return total;
}

/* Build WLAN_CFG response */
static uint32_t wcn7850_qmi_build_cfg_resp(uint8_t *buf, uint32_t buf_size,
                                             uint16_t txn_id)
{
    /* WLAN_CFG response: result success + num_cfg = 0 */
    QmiHdr *hdr = (QmiHdr *)buf;
    uint32_t tlv_len = 7;
    uint32_t total = sizeof(*hdr) + tlv_len;

    if (total > buf_size)
        return 0;

    hdr->type = QMI_TYPE_RESP;
    hdr->txn_id = cpu_to_le16(txn_id);
    hdr->msg_id = cpu_to_le16(QMI_WLFW_WLAN_CFG_REQ);
    hdr->msg_len = cpu_to_le16(tlv_len);

    buf[sizeof(*hdr) + 0] = 0x02;
    buf[sizeof(*hdr) + 1] = 0x04; buf[sizeof(*hdr) + 2] = 0x00;
    buf[sizeof(*hdr) + 3] = 0x00; buf[sizeof(*hdr) + 4] = 0x00;
    buf[sizeof(*hdr) + 5] = 0x00; buf[sizeof(*hdr) + 6] = 0x00;
    return total;
}

/* Process an incoming QMI message and generate response */
static void wcn7850_process_qmi(WCN7850State *s, PCIDevice *pci_dev,
                                  uint64_t ch_ctxt_addr,
                                  uint64_t er_ctxt_addr,
                                  uint32_t er_index,
                                  const uint8_t *data, size_t data_len)
{
    const QmiHdr *req = (const QmiHdr *)data;
    uint16_t msg_id, txn_id;
    uint8_t resp_buf[512];
    uint32_t resp_len = 0;

    if (data_len < sizeof(*req))
        return;

    msg_id = le16_to_cpu(req->msg_id);
    txn_id = le16_to_cpu(req->txn_id);

    qemu_log("WCN: QMI request msg_id=0x%04x txn=%u\n", msg_id, txn_id);

    switch (msg_id) {
    case QMI_WLFW_PHY_CAP_REQ:
        resp_len = wcn7850_qmi_build_phy_cap_resp(resp_buf, sizeof(resp_buf), txn_id);
        break;
    case QMI_WLFW_IND_REGISTER_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_HOST_CAP_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_RESPOND_MEM_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_CAP_REQ:
        resp_len = wcn7850_qmi_build_cap_resp(resp_buf, sizeof(resp_buf), txn_id);
        break;
    case QMI_WLFW_BDF_DOWNLOAD_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_M3_INFO_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_WLAN_INI_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_WLAN_CFG_REQ:
        resp_len = wcn7850_qmi_build_cfg_resp(resp_buf, sizeof(resp_buf), txn_id);
        break;
    case QMI_WLFW_WLAN_MODE_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    default:
        qemu_log("WCN: unknown QMI msg 0x%04x\n", msg_id);
        return;
    }

    if (!resp_len)
        return;

    /* Wrap the QMI response in QRTR DATA */
    qemu_log("WCN: sending QMI response msg_id=0x%04x len=%u\n", msg_id, resp_len);

    /* Build QRTR packet: from FW node to host (ath12k), port = QRTR data to QMI client */
    uint8_t qrtr_pkt[512];
    size_t qrtr_len = wcn7850_build_qrtr_pkt(qrtr_pkt, sizeof(qrtr_pkt),
                                               QRTR_TYPE_DATA,
                                               QRTR_NODE_FW, QRTR_PORT_QMI_SERVICE,
                                               QRTR_NODE_HOST, 0, /* port filled by QRTR layer */
                                               resp_buf, resp_len);
    if (!qrtr_len)
        return;

    /* Write the QRTR packet to DL channel 21 */
    uint64_t dl_ctxt_addr = ch_ctxt_addr + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
    if (!wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr,
                                er_index, qrtr_pkt, qrtr_len)) {
        qemu_log("WCN: failed to write DL data\n");
    }
}

/* Process a QRTR packet from the host */
static void wcn7850_process_qrtr(WCN7850State *s, PCIDevice *pci_dev,
                                   uint64_t ch_ctxt_addr,
                                   uint64_t er_ctxt_addr,
                                   uint32_t er_index,
                                   const uint8_t *data, size_t data_len)
{
    const QrtrHdrV1 *hdr;

    if (data_len < sizeof(*hdr))
        return;

    hdr = (const QrtrHdrV1 *)data;
    uint32_t type = le32_to_cpu(hdr->type);
    uint32_t src_node = le32_to_cpu(hdr->src_node_id);
    uint32_t src_port = le32_to_cpu(hdr->src_port_id);
    uint32_t payload_size = le32_to_cpu(hdr->size);
    const uint8_t *payload = data + sizeof(*hdr);

    (void)src_node;
    (void)src_port;

    qemu_log("WCN: QRTR type=%u src=%u:%u size=%u\n",
             type, src_node, src_port, payload_size);

    if (payload_size + sizeof(*hdr) > data_len)
        return;

    switch (type) {
    case QRTR_TYPE_DATA:
        wcn7850_process_qmi(s, pci_dev, ch_ctxt_addr, er_ctxt_addr, er_index,
                             payload, payload_size);
        break;
    default:
        qemu_log("WCN: unhandled QRTR type %u\n", type);
        break;
    }
}

/* Process UL channel data: read TREs from ring and process QRTR */
static void wcn7850_process_ul_data(WCN7850State *s, PCIDevice *pci_dev,
                                     uint64_t ch_ctxt_addr, uint32_t ch_id,
                                     uint64_t er_ctxt_addr)
{
    MhiChanCtxt ctxt;

    if (!wcn7850_read_chan_ctxt(pci_dev, ch_ctxt_addr, &ctxt))
        return;
    if (!ctxt.rbase || !ctxt.rlen)
        return;

    if (ctxt.rp < ctxt.rbase || ctxt.rp >= ctxt.rbase + ctxt.rlen)
        ctxt.rp = ctxt.rbase;
    if (ctxt.wp < ctxt.rbase || ctxt.wp >= ctxt.rbase + ctxt.rlen)
        ctxt.wp = ctxt.rbase;

    /* Process all pending TREs from rp to wp */
    while (ctxt.rp != ctxt.wp) {
        MhiTre tre;
        uint64_t tre_offset = ctxt.rp - ctxt.rbase;

        if (!wcn7850_read_tre(pci_dev, ctxt.rbase, tre_offset, &tre))
            break;

        uint32_t len = tre.dword0 & 0xffff;
        uint8_t *buf = g_malloc(len);

        qemu_log("WCN: UL TRE ch=%u ptr=0x%lx len=%u\n",
                 ch_id, (unsigned long)tre.ptr, len);

        if (pci_dma_read(pci_dev, tre.ptr, buf, len) == MEMTX_OK) {
            if (ch_id == MHI_CHAN_IPCR_UL) {
                wcn7850_process_qrtr(s, pci_dev,
                                      ch_ctxt_addr - ch_id * sizeof(MhiChanCtxt),
                                      er_ctxt_addr, 0, buf, len);
            }
        }

        g_free(buf);

        /* Advance rp */
        ctxt.rp += sizeof(MhiTre);
        if (ctxt.rp >= ctxt.rbase + ctxt.rlen)
            ctxt.rp = ctxt.rbase;
    }

    wcn7850_write_chan_ctxt(pci_dev, ch_ctxt_addr, &ctxt);
}

/* Send QRTR NEW_SERVER to announce QMI service */
static bool wcn7850_send_new_server(WCN7850State *s, PCIDevice *pci_dev,
                                    uint64_t ch_ctxt_addr,
                                    uint64_t er_ctxt_addr)
{
    uint8_t qrtr_buf[128];
    QrtrCtrlPkt ctrl;

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.cmd = cpu_to_le32(QRTR_TYPE_NEW_SERVER);
    ctrl.server_service = cpu_to_le32(QMI_WLFW_SERVICE_ID);
    ctrl.server_instance = cpu_to_le32(QMI_WLFW_SERVICE_INSTANCE);
    ctrl.server_node = cpu_to_le32(QRTR_NODE_FW);
    ctrl.server_port = cpu_to_le32(QRTR_PORT_QMI_SERVICE);

    size_t qrtr_len = wcn7850_build_qrtr_pkt(qrtr_buf, sizeof(qrtr_buf),
                                               QRTR_TYPE_NEW_SERVER,
                                               QRTR_NODE_FW, QRTR_PORT_CTRL,
                                               QRTR_NODE_HOST, QRTR_PORT_CTRL,
                                               &ctrl, sizeof(ctrl));
    if (!qrtr_len)
        return false;

    uint64_t dl_ctxt_addr = ch_ctxt_addr + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
    if (!wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr, 0,
                               qrtr_buf, qrtr_len)) {
        return false;
    }

    s->qmi_service_active = true;
    qemu_log("WCN: sent NEW_SERVER service=0x%x node=%u port=%u\n",
             QMI_WLFW_SERVICE_ID, QRTR_NODE_FW, QRTR_PORT_QMI_SERVICE);
    return true;
}

/* ===== Doorbell handlers ===== */

static void wcn7850_handle_channel_db(WCN7850State *s, PCIDevice *pci_dev,
                                       hwaddr db_addr, uint64_t db_val)
{
    uint32_t ch_id = (db_addr - WCN7850_CHDBOFF_VALUE) / 8;
    int db_offset = (db_addr - WCN7850_CHDBOFF_VALUE) % 8;

    (void)db_val;

    qemu_log("WCN: chan_db ch=%u off=%d val=0x%lx\n",
             ch_id, db_offset, (unsigned long)db_val);
    fprintf(stderr, "WCN: chan_db ch=%u off=%d\n", ch_id, db_offset);

    if (ch_id >= MHI_CHAN_MAX)
        return;

    if (!s->chan_ctxt_addr) {
        qemu_log("WCN: chan_db ch=%u but no chan_ctxt_addr\n", ch_id);
        return;
    }
    uint64_t ch_ctxt_addr = s->chan_ctxt_addr + ch_id * sizeof(MhiChanCtxt);

    if (db_offset == 0) {
        /* UL doorbell (host -> device): process incoming data */
        wcn7850_process_ul_data(s, pci_dev, ch_ctxt_addr, ch_id,
                                 s->er_ctxt_addr);
    }
}

static void wcn7850_handle_event_ring_db(WCN7850State *s, PCIDevice *pci_dev,
                                          hwaddr db_addr, uint64_t db_val)
{
    uint32_t er_index = (db_addr - WCN7850_ERDBOFF_VALUE) / 8;
    (void)db_val;
    qemu_log("WCN: er_db er=%u val=0x%lx\n", er_index,
             (unsigned long)db_val);
}

/* ===== Event emitters (MHI boot) ===== */

static void wcn7850_emit_ee_event(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t ctxt[48] = { 0 };
    uint8_t ev[16] = { 0 };
    uint64_t rbase, rlen, rp, next_rp;
    uint32_t msivec;

    if (!s->ctrl_event_pending || !s->er_ctxt_addr)
        return;

    if (pci_dma_read(pci_dev, s->er_ctxt_addr, ctxt, sizeof(ctxt)) != MEMTX_OK)
        return;

    memcpy(&msivec, ctxt + 8, sizeof(msivec));
    memcpy(&rbase, ctxt + 12, sizeof(rbase));
    memcpy(&rlen, ctxt + 20, sizeof(rlen));
    memcpy(&rp, ctxt + 28, sizeof(rp));
    msivec = le32_to_cpu(msivec);
    rbase = le64_to_cpu(rbase);
    rlen = le64_to_cpu(rlen);
    rp = le64_to_cpu(rp);

    if (!rbase || !rlen)
        return;

    if (rp < rbase || rp >= rbase + rlen)
        rp = rbase;

    memcpy(ev + 8, &(uint32_t){ cpu_to_le32(2u << 24) }, sizeof(uint32_t));
    memcpy(ev + 12, &(uint32_t){ cpu_to_le32(0x40u << 16) }, sizeof(uint32_t));

    if (pci_dma_write(pci_dev, rp, ev, sizeof(ev)) != MEMTX_OK)
        return;

    next_rp = rp + sizeof(ev);
    if (next_rp >= rbase + rlen)
        next_rp = rbase;

    next_rp = cpu_to_le64(next_rp);
    memcpy(ctxt + 28, &next_rp, sizeof(next_rp));
    (void)pci_dma_write(pci_dev, s->er_ctxt_addr, ctxt, sizeof(ctxt));
    msi_notify(pci_dev, msivec);
    s->ctrl_event_pending = false;
    qemu_log("WCN: EE event emitted pending=%d ee=%u\n", s->ctrl_event_pending, 2u);
}

static void wcn7850_emit_state_change_event(WCN7850State *s, PCIDevice *pci_dev,
                                             uint8_t state)
{
    uint8_t ctxt[48] = { 0 };
    uint8_t ev[16] = { 0 };
    uint64_t rbase, rlen, rp, next_rp;
    uint32_t msivec;

    if (!s->er_ctxt_addr)
        return;

    if (pci_dma_read(pci_dev, s->er_ctxt_addr, ctxt, sizeof(ctxt)) != MEMTX_OK)
        return;

    memcpy(&msivec, ctxt + 8, sizeof(msivec));
    memcpy(&rbase, ctxt + 12, sizeof(rbase));
    memcpy(&rlen, ctxt + 20, sizeof(rlen));
    memcpy(&rp, ctxt + 28, sizeof(rp));
    msivec = le32_to_cpu(msivec);
    rbase = le64_to_cpu(rbase);
    rlen = le64_to_cpu(rlen);
    rp = le64_to_cpu(rp);

    if (!rbase || !rlen)
        return;

    if (rp < rbase || rp >= rbase + rlen)
        rp = rbase;

    memcpy(ev + 8, &(uint32_t){ cpu_to_le32((uint32_t)state << 24) }, sizeof(uint32_t));
    memcpy(ev + 12, &(uint32_t){ cpu_to_le32(0x20u << 16) }, sizeof(uint32_t));

    if (pci_dma_write(pci_dev, rp, ev, sizeof(ev)) != MEMTX_OK)
        return;

    next_rp = rp + sizeof(ev);
    if (next_rp >= rbase + rlen)
        next_rp = rbase;

    next_rp = cpu_to_le64(next_rp);
    memcpy(ctxt + 28, &next_rp, sizeof(next_rp));
    (void)pci_dma_write(pci_dev, s->er_ctxt_addr, ctxt, sizeof(ctxt));
    msi_notify(pci_dev, msivec);
    qemu_log("WCN: state event state=%u\n", state);
}

/* ===== MMIO handlers ===== */

static uint64_t wcn7850_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    WCN7850State *s = opaque;
    uint64_t ret = 0;
    hwaddr win_off;

    switch (addr) {
    case WCN7850_MHIREGLEN:
        ret = WCN7850_BAR0_SIZE;
        goto done;
    case WCN7850_MHICFG:
        ret = WCN7850_MHICFG_VALUE;
        goto done;
    case WCN7850_CHDBOFF:
        ret = WCN7850_CHDBOFF_VALUE;
        goto done;
    case WCN7850_ERDBOFF:
        ret = WCN7850_ERDBOFF_VALUE;
        goto done;
    case WCN7850_BHIOFF:
        ret = WCN7850_BHIOFF_VALUE;
        goto done;
    case WCN7850_BHIEOFF:
        ret = WCN7850_BHIEOFF_VALUE;
        goto done;
    case WCN7850_MHICTRL:
        ret = s->mhi_state << 8;
        goto done;
    case WCN7850_MHISTATUS:
        ret = (s->mhi_state << 8) | 0x1;
        goto done;
    case WCN7850_TCSR_SOC_HW_VERSION:
        ret = 0x200;
        goto done;
    case WCN7850_BHI_EXECENV:
        ret = s->bhi_execenv;
        goto done;
    case WCN7850_BHI_STATUS:
        ret = s->bhi_status;
        goto done;
    case 0x58:
        ret = (uint32_t)(s->chan_ctxt_addr & 0xffffffffu);
        goto done;
    case 0x5c:
        ret = (uint32_t)(s->chan_ctxt_addr >> 32);
        goto done;
    case 0x60:
        ret = (uint32_t)(s->er_ctxt_addr & 0xffffffffu);
        goto done;
    case 0x64:
        ret = (uint32_t)(s->er_ctxt_addr >> 32);
        goto done;
    case 0x68:
        ret = (uint32_t)(s->cmd_ctxt_addr & 0xffffffffu);
        goto done;
    case 0x6c:
        ret = (uint32_t)(s->cmd_ctxt_addr >> 32);
        goto done;
    default:
        break;
    }

    if (addr < WCN7850_WINDOW_START) {
        memcpy(&ret, s->bar0_always_on + addr, size);
        goto done;
    }

    if (addr < WCN7850_WINDOW_START + WCN7850_WINDOW_SIZE) {
        win_off = addr - WCN7850_WINDOW_START;
        if (s->window_select == WCN7850_WINDOW_TCSR_SEL &&
            win_off == (WCN7850_TCSR_SOC_HW_VERSION & (WCN7850_WINDOW_SIZE - 1))) {
            ret = 0x200;
        } else {
            memcpy(&ret, s->window_memory + win_off, size);
        }
    }

done:
    return ret;
}

static void wcn7850_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    WCN7850State *s = opaque;
    PCIDevice *pci_dev = PCI_DEVICE(s);
    /* Handle doorbell writes */
    if (addr >= WCN7850_CHDBOFF_VALUE &&
        addr < WCN7850_CHDBOFF_VALUE + MHI_CHAN_MAX * 8) {
        wcn7850_handle_channel_db(s, pci_dev, addr, val);
        memcpy(s->bar0_always_on + addr, &val, size);
        return;
    }

    if (addr >= WCN7850_ERDBOFF_VALUE &&
        addr < WCN7850_ERDBOFF_VALUE + 16 * 8) {
        wcn7850_handle_event_ring_db(s, pci_dev, addr, val);
        memcpy(s->bar0_always_on + addr, &val, size);
        return;
    }

    /* MHI reset */
    if (addr == WCN7850_MHICTRL && (val & BIT(1))) {
        s->mhi_state = 0;
        s->bhi_downloaded = false;
        s->bhi_status = 0;
        s->bhi_execenv = 0;
        return;
    }

    /* MHI state transition to M2 */
    if (addr == WCN7850_MHICTRL && ((val & 0x0000ff00u) == 0x00000200u)) {
        s->mhi_state = 2;
        wcn7850_emit_state_change_event(s, pci_dev, 2);
        return;
    }

    if (addr == WCN7850_BHI_EXECENV) {
        s->bhi_execenv = val;
        return;
    }

    if (addr == WCN7850_BHIOFF) {
        *(uint32_t *)(s->bar0_always_on + addr) = WCN7850_BHIOFF_VALUE;
        return;
    }

    if (addr == WCN7850_BHIEOFF) {
        *(uint32_t *)(s->bar0_always_on + addr) = WCN7850_BHIEOFF_VALUE;
        return;
    }

    if (addr == WCN7850_BHI_STATUS) {
        s->bhi_status = val & 0xc0000000u;
        return;
    }

    if (addr == WCN7850_BHI_BASE + 0x18) {
        s->bhi_downloaded = true;
        s->bhi_status = 0x80000000u;
        s->bhi_execenv = 2;
        s->mhi_state = 1;
        s->ctrl_event_pending = true;
        msi_notify(pci_dev, 0);
        qemu_log("WCN: BHI trig pending=1 timer=5ms\n");
        timer_mod(s->ctrl_event_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000LL);
        return;
    }

    if (addr == WCN7850_BHI_INTVEC) {
        return;
    }

    if (addr == WCN7850_WINDOW_REG) {
        s->window_select = val & 0x3f;
        return;
    }

    if (addr == 0x58 || addr == 0x5c || addr == 0x60 || addr == 0x64 ||
        addr == 0x68 || addr == 0x6c) {
        memcpy(s->bar0_always_on + addr, &val, size);
        {
            uint32_t lo = *(uint32_t *)(s->bar0_always_on + 0x58);
            uint32_t hi = *(uint32_t *)(s->bar0_always_on + 0x5c);
            s->chan_ctxt_addr = ((uint64_t)hi << 32) | lo;
        }
        {
            uint32_t lo = *(uint32_t *)(s->bar0_always_on + 0x60);
            uint32_t hi = *(uint32_t *)(s->bar0_always_on + 0x64);
            uint64_t old_er = s->er_ctxt_addr;
            s->er_ctxt_addr = ((uint64_t)hi << 32) | lo;
            if (!old_er && s->er_ctxt_addr && !s->qmi_service_active) {
                qemu_log("WCN: ECABAP er_ctxt=0x%lx\n",
                         (unsigned long)s->er_ctxt_addr);
            }
        }
        {
            uint32_t lo = *(uint32_t *)(s->bar0_always_on + 0x68);
            uint32_t hi = *(uint32_t *)(s->bar0_always_on + 0x6c);
            s->cmd_ctxt_addr = ((uint64_t)hi << 32) | lo;
        }
        timer_mod(s->ctrl_event_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000LL);
        return;
    }

    /* ===== SRNG register intercept ===== */
    /* TCL ring config registers (R0) */
    for (int i = 0; i < WCN7850_NUM_TCL_RINGS; i++) {
        if (addr == WCN7850_TCL_RING_BASE_LSB(i) ||
            addr == WCN7850_TCL_RING_BASE_MSB(i) ||
            addr == WCN7850_TCL_RING_ID(i) ||
            addr == WCN7850_TCL_RING_MISC(i) ||
            addr == WCN7850_TCL_RING_MSI1_DATA(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            wcn7850_update_tcl_ring_cfg(s, i);
            return;
        }
    }

    /* REO ring config registers (R0) */
    for (int i = 0; i < WCN7850_NUM_REO_RINGS; i++) {
        if (addr == WCN7850_REO_RING_BASE_LSB(i) ||
            addr == WCN7850_REO_RING_BASE_MSB(i) ||
            addr == WCN7850_REO_RING_ID(i) ||
            addr == WCN7850_REO_RING_MISC(i) ||
            addr == WCN7850_REO_RING_MSI1_DATA(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            wcn7850_update_reo_ring_cfg(s, i);
            return;
        }
    }

    /* TCL HP write (SW producing TX descriptors → trigger TX processing) */
    for (int i = 0; i < WCN7850_NUM_TCL_RINGS; i++) {
        if (addr == WCN7850_TCL_RING_HP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_TCL_DATA, i);
            if (r) {
                r->hp = val;
                if (r->configured && r->enable) {
                    wcn7850_process_tcl_data(s, pci_dev, r);
                }
            }
            return;
        }
        if (addr == WCN7850_TCL_RING_TP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_TCL_DATA, i);
            if (r) {
                r->tp = val;
            }
            return;
        }
    }

    /* REO HP/TP writes (SW reading RX completions) */
    for (int i = 0; i < WCN7850_NUM_REO_RINGS; i++) {
        if (addr == WCN7850_REO_RING_HP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = (i == 0)
                ? wcn7850_find_ring(s, WCN7850_RING_REO_EXCEPTION, 0)
                : wcn7850_find_ring(s, WCN7850_RING_REO_DST, i);
            if (r) {
                r->hp = val;
            }
            return;
        }
        if (addr == WCN7850_REO_RING_TP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = (i == 0)
                ? wcn7850_find_ring(s, WCN7850_RING_REO_EXCEPTION, 0)
                : wcn7850_find_ring(s, WCN7850_RING_REO_DST, i);
            if (r) {
                r->tp = val;
            }
            return;
        }
    }

    if (addr < WCN7850_WINDOW_START) {
        memcpy(s->bar0_always_on + addr, &val, size);
        return;
    }

    if (addr < WCN7850_WINDOW_START + WCN7850_WINDOW_SIZE) {
        memcpy(s->window_memory + (addr - WCN7850_WINDOW_START), &val, size);
        return;
    }
}

/* Send QMI FW_READY_IND to trigger ath12k QMI handshake */
static bool wcn7850_send_fw_ready_ind(WCN7850State *s, PCIDevice *pci_dev,
                                      uint64_t ch_ctxt_base,
                                      uint64_t er_ctxt_addr)
{
    uint8_t qmi_buf[64];
    QmiHdr *hdr = (QmiHdr *)qmi_buf;
    uint8_t qrtr_buf[128];
    size_t qrtr_len;

    /* Build QMI FW_READY indication (type=4, msg_id=0x0038, no payload) */
    memset(qmi_buf, 0, sizeof(qmi_buf));
    hdr->type = QMI_TYPE_IND;
    hdr->txn_id = cpu_to_le16(0);
    hdr->msg_id = cpu_to_le16(QMI_WLFW_FW_READY_IND);
    hdr->msg_len = cpu_to_le16(0);

    /* Wrap in QRTR DATA from FW to host */
    qrtr_len = wcn7850_build_qrtr_pkt(qrtr_buf, sizeof(qrtr_buf),
                                       QRTR_TYPE_DATA,
                                       QRTR_NODE_FW, QRTR_PORT_QMI_SERVICE,
                                       QRTR_NODE_HOST, 0,
                                       qmi_buf, sizeof(*hdr));
    if (!qrtr_len)
        return false;

    uint64_t dl_ctxt_addr = ch_ctxt_base + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
    if (!wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr,
                                0, qrtr_buf, qrtr_len)) {
        qemu_log("WCN: FW_READY_IND write DL failed\n");
        return false;
    }

    return true;
}

/* ===== After mission mode, send FW_READY_IND and NEW_SERVER ===== */
static bool wcn7850_mission_mode_setup(WCN7850State *s, PCIDevice *pci_dev)
{
    fprintf(stderr, "WCN: mission_mode_setup er=0x%lx chan=0x%lx\n",
            (unsigned long)s->er_ctxt_addr,
            (unsigned long)s->chan_ctxt_addr);
    if (!s->er_ctxt_addr || !s->chan_ctxt_addr)
        return false;

    /* Send QRTR NEW_SERVER first so ath12k can kernel_connect() to us */
    fprintf(stderr, "WCN: sending NEW_SERVER\n");
    if (!wcn7850_send_new_server(s, pci_dev, s->chan_ctxt_addr, s->er_ctxt_addr)) {
        return false;
    }

    /* Then send QMI FW_READY_IND to trigger ath12k QMI handshake */
    fprintf(stderr, "WCN: sending FW_READY_IND\n");
    if (!wcn7850_send_fw_ready_ind(s, pci_dev, s->chan_ctxt_addr, s->er_ctxt_addr)) {
        return false;
    }

    s->qmi_service_active = true;
    s->qmi_boot_pending = false;
    return true;
}

/* Timer callback: emit EE event, then send FW_READY_IND and NEW_SERVER */
static void wcn7850_ctrl_event_timer_ext(void *opaque)
{
    WCN7850State *s = opaque;
    PCIDevice *pci_dev = PCI_DEVICE(s);
    bool was_pending = s->ctrl_event_pending;

    qemu_log("WCN: timer fire ctxt=0x%lx pending=%d\n",
             (unsigned long)s->er_ctxt_addr, s->ctrl_event_pending);

    if (was_pending) {
        wcn7850_emit_ee_event(s, pci_dev);
        s->qmi_boot_pending = true;
    }

    if (s->qmi_boot_pending && !s->qmi_service_active) {
        if (!wcn7850_mission_mode_setup(s, pci_dev)) {
            timer_mod(s->ctrl_event_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000LL);
        }
    }
}

static const MemoryRegionOps wcn7850_mmio_ops = {
    .read = wcn7850_mmio_read,
    .write = wcn7850_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void wcn7850_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    WCN7850State *s = WCN7850(pci_dev);

    pci_config_set_vendor_id(pci_dev->config, WCN7850_VENDOR_ID);
    pci_config_set_device_id(pci_dev->config, WCN7850_DEVICE_ID);
    pci_config_set_revision(pci_dev->config, 0x01);
    pci_config_set_class(pci_dev->config, PCI_CLASS_NETWORK_OTHER);

    memory_region_init_io(&s->bar0_mmio, OBJECT(s), &wcn7850_mmio_ops, s,
                          "wcn7850-bar0", WCN7850_BAR0_SIZE);
    s->bar0_always_on = g_malloc0(WCN7850_WINDOW_START);
    s->window_memory = g_malloc0(WCN7850_WINDOW_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0_mmio);

    if (msi_init(pci_dev, 0x50, WCN7850_MSI_VECTORS, true, false, errp) < 0)
        return;

    s->srng_memory = g_malloc0(4 * 1024 * 1024);
    for (int i = 0; i < WCN7850_CE_COUNT; i++) {
        s->ce[i].src_ring = g_malloc0(32 * 1024);
        s->ce[i].dst_ring = g_malloc0(32 * 1024);
        s->ce[i].status_ring = g_malloc0(32 * 1024);
    }

    s->mhi_state = 0;
    s->bhi_downloaded = false;
    s->er_ctxt_addr = 0;
    s->ctrl_event_pending = false;
    s->qmi_boot_pending = false;
    s->ctrl_event_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        wcn7850_ctrl_event_timer_ext, s);
    s->qmi_service_active = false;

    /* ===== FPGA data path init ===== */

    /* Initialize with a default MAC if none set */
    s->mac_addr[0] = 0x02;
    s->mac_addr[1] = 0x1a;
    s->mac_addr[2] = 0x11;
    s->mac_addr[3] = 0xfe;
    s->mac_addr[4] = 0xca;
    s->mac_addr[5] = 0x01;
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    memcpy(s->mac_addr, s->conf.macaddr.a, 6);

    /* Create NetClient for network backend */
    s->nic = qemu_new_nic(&net_wcn7850_info, &s->conf,
        object_get_typename(OBJECT(s)),
        DEVICE(pci_dev)->id,
        &DEVICE(pci_dev)->mem_reentrancy_guard,
        s);
    s->link_up = true;
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->mac_addr);

    /* Initialize buffer pool as a RAM region in guest address space */
    s->pool_paddr = WCN7850_POOL_PADDR;
    s->pool_num_bufs = WCN7850_POOL_NUM_BUFS;
    s->pool_next_alloc = 0;
    memory_region_init_ram(&s->pool_mr, OBJECT(s), "wcn7850-pool",
                           WCN7850_POOL_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), s->pool_paddr, &s->pool_mr);
    s->pool_mem = memory_region_get_ram_ptr(&s->pool_mr);
    memset(s->pool_mem, 0, WCN7850_POOL_SIZE);

    /* Initialize ring tracking */
    s->num_rings = 0;
    memset(s->rings, 0, sizeof(s->rings));
    s->reo_dst_ring_idx = -1;
    s->reo_exc_ring_idx = -1;
    s->tcl_data_ring_idx = -1;

    /* Initialize RX/TX state */
    s->rx_pending = false;
    s->rx_pending_buf = NULL;
    s->rx_pending_len = 0;
    s->tx_buffer = NULL;
}

static void wcn7850_pci_uninit(PCIDevice *pci_dev)
{
    WCN7850State *s = WCN7850(pci_dev);

    timer_free(s->ctrl_event_timer);
    g_free(s->bar0_always_on);
    g_free(s->window_memory);
    g_free(s->srng_memory);
    for (int i = 0; i < WCN7850_CE_COUNT; i++) {
        g_free(s->ce[i].src_ring);
        g_free(s->ce[i].dst_ring);
        g_free(s->ce[i].status_ring);
    }
    g_free(s->fw_path);
    g_free(s->rx_pending_buf);
    g_free(s->tx_buffer);
    if (s->pool_mem) {
        memory_region_del_subregion(get_system_memory(), &s->pool_mr);
    }
    if (s->nic) {
        qemu_del_nic(s->nic);
    }
}

static const VMStateDescription vmstate_wcn7850 = {
    .name = "wcn7850",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(window_select, WCN7850State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property wcn7850_properties[] = {
    DEFINE_PROP_STRING("fw-path", WCN7850State, fw_path),
    DEFINE_NIC_PROPERTIES(WCN7850State, conf),
};

static void wcn7850_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = wcn7850_pci_realize;
    k->exit = wcn7850_pci_uninit;
    k->vendor_id = WCN7850_VENDOR_ID;
    k->device_id = WCN7850_DEVICE_ID;
    k->class_id = PCI_CLASS_NETWORK_OTHER;
    k->revision = 0x01;
    dc->vmsd = &vmstate_wcn7850;
    device_class_set_props(dc, wcn7850_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo wcn7850_info = {
    .name = TYPE_WCN7850,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(WCN7850State),
    .class_init = wcn7850_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void wcn7850_register_types(void)
{
    type_register_static(&wcn7850_info);
}

type_init(wcn7850_register_types)
