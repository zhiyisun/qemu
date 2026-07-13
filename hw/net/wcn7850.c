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

/* CE shadow register base (doorbell writes for CE HP/TP) */
#define WCN7850_SHADOW_BASE     0x000008fc
#define WCN7850_SHADOW_MAX      64

/* CE SRNG register layout (through window with select=55) */
#define WCN7850_CE0_SRC_BASE    0x01b80000
#define WCN7850_CE0_DST_BASE    0x01b81000
#define WCN7850_CE_STRIDE       0x2000
/* CE source ring R0 (config) offset within CE block */
#define WCN7850_CE_RING_BASE_LSB  0x000
#define WCN7850_CE_RING_BASE_MSB  0x004
#define WCN7850_CE_RING_ID        0x008
#define WCN7850_CE_RING_MISC      0x010
#define WCN7850_CE_RING_HP_ADDR_LSB 0x01c
#define WCN7850_CE_RING_HP_ADDR_MSB 0x020
/* CE R2 (pointer) register offset from CE block base */
#define WCN7850_CE_RING_HP_OFFSET  0x400
#define WCN7850_CE_RING_TP_OFFSET  0x404
/* Window offset = full_reg & (WCN7850_WINDOW_SIZE - 1)
   for WINDOW_SIZE=0x80000, mask = 0x7FFFF */
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
#define WCN7850_CRDB_HIGHER 0x74

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

/* ===== CE (Copy Engine) register offsets (windowed, window_select=55) ===== */
#define WCN7850_CE_WINDOW_SEL      55
#define WCN7850_CE_SRC_HP(_ce)     (0x400 + (_ce) * 0x2000)
#define WCN7850_CE_SRC_TP(_ce)     (0x404 + (_ce) * 0x2000)
#define WCN7850_CE_DST_HP(_ce)     (0xC00 + (_ce) * 0x2000)
#define WCN7850_CE_DST_TP(_ce)     (0xC04 + (_ce) * 0x2000)
#define WCN7850_CE_DST_STATUS_HP(_ce) (0x1400 + (_ce) * 0x2000)
#define WCN7850_CE_SRC_RING_SIZE   512
#define WCN7850_CE_DST_RING_SIZE   1024
#define WCN7850_MAX_CE_DESC_SIZE   32

/* WMI protocol constants */
#define WMI_CMD_HDR_CMD_ID         GENMASK(23, 0)
#define WMI_TLV_TAG                GENMASK(31, 16)
#define WMI_TLV_LEN                GENMASK(15, 0)

/* WMI command groups */
#define WMI_GRP_INIT       0x1
#define WMI_GRP_VDEV       0x13
#define WMI_GRP_PEER       0x14
#define WMI_GRP_MGMT       0x1B
#define WMI_GRP_SCAN       0x10
#define WMI_GRP_PDEV       0x12
#define WMI_GRP_WMI        0x100 /* not real group, for service ready */

#define WMI_TLV_CMD(grp_id) (((grp_id) << 12) | 0x1)

/* WMI command IDs */
#define WMI_SERVICE_READY_EVENTID  0x1
#define WMI_READY_EVENTID          0x2
#define WMI_INIT_CMDID             WMI_TLV_CMD(WMI_GRP_INIT)
#define WMI_VDEV_CREATE_CMDID      WMI_TLV_CMD(WMI_GRP_VDEV)
#define WMI_PEER_CREATE_CMDID      WMI_TLV_CMD(WMI_GRP_PEER)

/* WMI TLV tags (matching driver enum) */
#define WMI_TAG_ARRAY_UINT32              16
#define WMI_TAG_ARRAY_FIXED_STRUCT        19
#define WMI_TAG_SERVICE_READY_EVENT       32
#define WMI_TAG_READY_EVENT               35
#define WMI_TAG_INIT_CMD                  74
#define WMI_TAG_RESOURCE_CONFIG           75
#define WMI_TAG_WLAN_HOST_MEMORY_CHUNK    76

/* HTC protocol */
#define HTC_EP_CTRL          0
#define HTC_EP_WMI           2  /* WMI RX from model */
#define HTC_EP_WMI_TX        3  /* WMI TX to model */

#define HTC_MSG_READY_ID                 1
#define HTC_MSG_CONNECT_SERVICE_ID       2
#define HTC_MSG_CONNECT_SERVICE_RESP_ID  3
#define HTC_MSG_SETUP_COMPLETE_EX_ID     5

#define HTC_HDR_FLAG_CREDITS             BIT(0)

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
#define QMI_WLFW_SERVICE_INSTANCE 0x0101 /* ath12k lookup: (version<<8)|ins_id = (1<<8)|1 = 0x101 */

/* MHI event ring constants */
#define MHI_PKT_TYPE_STATE_CHANGE_EVENT 0x20
#define MHI_PKT_TYPE_CMD_COMPLETION_EVENT 0x21
#define MHI_PKT_TYPE_TX_EVENT 0x22
#define MHI_PKT_TYPE_EE_EVENT 0x40
#define MHI_PKT_TYPE_TRANSFER_EVENT 0x22   /* alias for host's TX_EVENT */

/* MHI transfer event completion codes (dword0[31:24]) */
#define MHI_EV_CC_EOT 0x2
#define MHI_EV_CC_OVERFLOW 0x3
#define MHI_EV_CC_EOB 0x4

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

/* QMI header (7 bytes, matching kernel's struct qmi_header) */
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

/* HTC header (8 bytes) */
typedef struct {
    uint32_t hdr_info;
    uint32_t ctrl_info;
} QEMU_PACKED HtcHdr;

#define HTC_HDR_EPID(_h)       ((_h)->hdr_info & 0xff)
#define HTC_HDR_FLAGS(_h)      (((_h)->hdr_info >> 8) & 0xff)
#define HTC_HDR_PAYLOAD_LEN(_h) ((_h)->hdr_info >> 16)
#define HTC_HDR_MSG_ID(_h)     ((_h)->ctrl_info & 0xffff)
#define HTC_HDR_SERVICE_ID(_h) (((_h)->ctrl_info >> 16) & 0xffff)

/* HTC connect service request */
typedef struct {
    uint32_t msg_svc_id;
    uint32_t flags_len;
} QEMU_PACKED HtcConnSvc;

#define HTC_CONN_MSG_ID(_c)    ((_c)->msg_svc_id & 0xffff)
#define HTC_CONN_SVC_ID(_c)    (((_c)->msg_svc_id >> 16) & 0xffff)

/* HTC connect service response */
typedef struct {
    uint32_t msg_svc_id;
    uint32_t flags_len;
    uint32_t svc_meta_pad;
} QEMU_PACKED HtcConnSvcResp;

#define HTC_CONN_RESP_STATUS(_c) ((_c)->flags_len & 0xff)
#define HTC_CONN_RESP_EPID(_c)   (((_c)->flags_len >> 8) & 0xff)
#define HTC_CONN_RESP_MAX_MSG    ((_c)->flags_len >> 16)

/* HTC setup complete extended */
typedef struct {
    uint32_t msg_id;
    uint32_t flags;
    uint32_t max_msgs_per_bundled_recv;
} QEMU_PACKED HtcSetupCompleteExt;

/* HTC ready message */
typedef struct {
    uint32_t msg_id;
    uint32_t credits;
    uint32_t credit_size;
    uint32_t max_ep;
    uint32_t pad;
} QEMU_PACKED HtcReadyMsg;

/* WMI command header (4 bytes) */
typedef struct {
    uint32_t cmd_id;
} QEMU_PACKED WmiCmdHdr;

/* WMI TLV header (4 bytes: tag in [31:16], len in [15:0]) */
typedef struct {
    uint32_t header;
} QEMU_PACKED WmiTlv;

#define WMI_TLV_HDR(tag, len) cpu_to_le32(((tag) << 16) | (len))

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

    char *fw_path;

    /* Context base addresses written by host */
    uint64_t chan_ctxt_addr;
    uint64_t cmd_ctxt_addr;

    /* QRTR node/port for firmware QMI service */
    bool qmi_service_active;
    bool qmi_newserver_sent;
    bool qmi_newserver_resent;
    bool qmi_boot_pending;
    bool qmi_newserver_delivered;
    uint16_t qmi_pending_ind;
    uint32_t qmi_client_node;
    uint32_t qmi_client_port;

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

    /* CE poll timer */
    QEMUTimer *ce_poll_timer;

    /* ===== CE/HTC/WMI state ===== */

    /* CE doorbell delivery: true after QMI boot completes */
    bool ce_ready;

    /* CE source ring doorbell state (shadow of HP values from driver) */
    uint32_t ce_src_hp[WCN7850_CE_COUNT];
    uint32_t ce_src_tp[WCN7850_CE_COUNT];

    /* CE dest ring doorbell state */
    uint32_t ce_dst_hp[WCN7850_CE_COUNT];
    uint32_t ce_dst_tp[WCN7850_CE_COUNT];

    /* CE source ring config (populated when driver writes ring config) */
    uint64_t ce_src_base[WCN7850_CE_COUNT];
    uint32_t ce_src_size[WCN7850_CE_COUNT];
    uint32_t ce_src_entry_size[WCN7850_CE_COUNT];
    uint64_t ce_src_hp_addr[WCN7850_CE_COUNT];
    uint64_t ce_src_tp_addr[WCN7850_CE_COUNT];

    /* CE dest ring config */
    uint64_t ce_dst_base[WCN7850_CE_COUNT];
    uint32_t ce_dst_size[WCN7850_CE_COUNT];
    uint32_t ce_dst_entry_size[WCN7850_CE_COUNT];
    uint32_t ce_dst_drv_hp[WCN7850_CE_COUNT]; /* driver's HP (producer ptr, from shadow) */
    uint32_t ce_dst_drv_hp_prev[WCN7850_CE_COUNT]; /* prev driver HP to track delta */
    uint32_t ce_dst_pending[WCN7850_CE_COUNT]; /* pending descriptors for model to consume */
    bool ce_dst_doorbell_seen[WCN7850_CE_COUNT]; /* true after first doorbell */
    uint32_t ce_dst_cons[WCN7850_CE_COUNT]; /* model's consumer ptr (bytes) */
    uint64_t ce_dst_hp_addr[WCN7850_CE_COUNT]; /* RDP phys addr for HP */
    uint64_t ce_dst_tp_addr[WCN7850_CE_COUNT];

    /* CE DST status ring config (offset 0x58 within CE dest block) */
    uint64_t ce_sts_base[WCN7850_CE_COUNT];
    uint32_t ce_sts_size[WCN7850_CE_COUNT];
    uint32_t ce_sts_entry_size[WCN7850_CE_COUNT];
    uint32_t ce_sts_hp[WCN7850_CE_COUNT];
    uint64_t ce_sts_hp_addr[WCN7850_CE_COUNT]; /* RDP phys addr for STATUS HP */

    /* HTC endpoint mapping: [0..HTC_EP_MAX] = CE pipe pair */
#define HTC_EP_MAX 8
    struct {
        int ul_pipe;
        int dl_pipe;
        int service_id;
        bool connected;
    } htc_ep[HTC_EP_MAX];
    uint32_t htc_credits;

    /* WMI state */
    bool wmi_service_ready_sent;
    bool wmi_ready_sent;
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
    uint32_t dword0, dword1;

    /* Transfer event (MHI spec):
     *   ptr  = the completed TRE's ring address (host recovers buffer from it)
     *   dword0[15:0]  = transferred length
     *   dword0[31:24] = completion code (EOT)
     *   dword1[23:16] = event type (TX_EVENT)
     *   dword1[31:24] = channel id
     */
    memcpy(ev, &(uint64_t){ cpu_to_le64(buf_ptr) }, sizeof(uint64_t));
    dword0 = cpu_to_le32(((uint32_t)MHI_EV_CC_EOT << 24) | (len & 0xffffu));
    memcpy(ev + 8, &dword0, sizeof(uint32_t));
    dword1 = cpu_to_le32(((ch_id & 0xffu) << 24) | ((uint32_t)MHI_PKT_TYPE_TX_EVENT << 16));
    memcpy(ev + 12, &dword1, sizeof(uint32_t));

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
    size_t padded = (payload_size + 3) & ~(size_t)3;
    size_t total = sizeof(*hdr) + padded;

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

    if (payload && payload_size) {
        memcpy(buf + sizeof(*hdr), payload, payload_size);
        memset(buf + sizeof(*hdr) + payload_size, 0, padded - payload_size);
    }

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
    /* Host MHI expects the transfer-event ptr to be the TRE's ring address
     * (so it can recover the buffer from the TRE), not the data buffer. */
    uint64_t tre_addr = ctxt.rp;
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
     * Use erindex from channel context, not the parameter.
     * ptr must be the completed TRE's ring address (tre_addr). */
    wcn7850_post_transfer_event(s, pci_dev, er_ctxt_addr, ctxt.erindex,
                                 MHI_CHAN_IPCR_DL, tre_addr, write_len);
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

/* ===== CE/HTC/WMI handler ===== */

/* Send a WMI event via CE destination ring (pipe EP 2 for RX WMI).
 * Writes WMI event data into a CE dest ring buffer, advances HP, triggers MSI.
 */
static void wcn7850_wmi_send_event(WCN7850State *s, PCIDevice *pci_dev,
                                      int ce_pipe, const void *data,
                                      uint32_t data_len, int msivec)
{
    uint64_t base = s->ce_dst_base[ce_pipe];
    uint32_t esize = s->ce_dst_entry_size[ce_pipe] ?: 16;
    uint32_t ring_sz = s->ce_dst_size[ce_pipe] ?: (WCN7850_CE_DST_RING_SIZE * esize);
    if (!base || !ring_sz)
        return;

    /* Check pending: must have at least one descriptor available */
    if (!s->ce_dst_pending[ce_pipe])
        return;

    /* Read dest ring entry at current consumer position to get buffer address */
    uint64_t desc_addr = base + s->ce_dst_cons[ce_pipe];
    struct {
        uint32_t buf_addr_low;
        uint32_t buf_addr_info;
    } desc;
    if (pci_dma_read(pci_dev, desc_addr, &desc, sizeof(desc)) != MEMTX_OK)
        return;

    uint64_t buf_addr = (uint64_t)desc.buf_addr_low |
                        ((uint64_t)(desc.buf_addr_info & 0xff) << 32);
    if (!buf_addr)
        return;

    if (data_len > 2048)
        data_len = 2048;

    /* Write WMI data into the buffer */
    if (pci_dma_write(pci_dev, buf_addr, data, data_len) != MEMTX_OK)
        return;

    /* Advance consumer pointer and decrement pending count */
    s->ce_dst_cons[ce_pipe] += esize;
    if (s->ce_dst_cons[ce_pipe] >= ring_sz)
        s->ce_dst_cons[ce_pipe] = 0;
    s->ce_dst_pending[ce_pipe]--;

    /* Write CE dst status descriptor and advance STATUS HP */
    if (s->ce_sts_base[ce_pipe]) {
        uint32_t sts_esize = s->ce_sts_entry_size[ce_pipe] ?: 16;
        uint32_t sts_hp = s->ce_sts_hp[ce_pipe];
        uint8_t sts_desc[16] = {0};
        *(uint32_t *)sts_desc = cpu_to_le32(data_len << 16);
        pci_dma_write(pci_dev, s->ce_sts_base[ce_pipe] + sts_hp,
                      sts_desc, sts_esize);
        sts_hp += sts_esize;
        if (s->ce_sts_size[ce_pipe] &&
            sts_hp >= s->ce_sts_size[ce_pipe])
            sts_hp = 0;
        s->ce_sts_hp[ce_pipe] = sts_hp;

        /* RDP write-back: driver reads status HP from terms of RDP */
        if (s->ce_sts_hp_addr[ce_pipe]) {
            uint32_t sts_hp_le = cpu_to_le32(sts_hp);
            pci_dma_write(pci_dev, s->ce_sts_hp_addr[ce_pipe],
                          &sts_hp_le, sizeof(sts_hp_le));
        }
    }

    /* Send MSI to notify the driver */
    if (msivec >= 0) {
        msi_notify(pci_dev, msivec);
    }
}

/* Build and send a WMI_SERVICE_READY_EVENT to the driver */
static void wcn7850_send_wmi_service_ready(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[512];
    uint32_t off = 0;

    /* WMI command header */
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    hdr->cmd_id = cpu_to_le32(WMI_SERVICE_READY_EVENTID);
    off += sizeof(*hdr);

    /* TLV: WMI_TAG_SERVICE_READY_EVENT = 32, len = sizeof(wmi_service_ready_event) */
    uint32_t sre_len = 128; /* struct wmi_service_ready_event total */
    WmiTlv *tlv = (WmiTlv *)(buf + off);
    tlv->header = WMI_TLV_HDR(WMI_TAG_SERVICE_READY_EVENT, sre_len);
    off += sizeof(*tlv);

    /* struct wmi_service_ready_event body (128 bytes, fill key fields) */
    memset(buf + off, 0, sre_len);
    *(uint32_t *)(buf + off + 0)  = cpu_to_le32(0x01000000); /* fw_build_vers */
    /* fw_abi_vers = 6×uint32_t at offset 4, keep zero */
    *(uint32_t *)(buf + off + 28) = cpu_to_le32(0x07);       /* phy_capability */
    *(uint32_t *)(buf + off + 36) = cpu_to_le32(2);          /* num_rf_chains */
    *(uint32_t *)(buf + off + 40) = cpu_to_le32(1);          /* ht_cap_info */
    *(uint32_t *)(buf + off + 44) = cpu_to_le32(1);          /* vht_cap_info */
    *(uint32_t *)(buf + off + 48) = cpu_to_le32(0x1ff);      /* vht_supp_mcs */
    *(uint32_t *)(buf + off + 60) = cpu_to_le32(0x0f);       /* sys_cap_info */
    *(uint32_t *)(buf + off + 76) = cpu_to_le32(32);         /* max_num_scan_channels */
    *(uint32_t *)(buf + off + 80) = cpu_to_le32(0);          /* hw_bd_id */
    /* hw_bd_info[5]: keep zero */
    *(uint32_t *)(buf + off + 104)= cpu_to_le32(1);          /* max_supported_macs */
    *(uint32_t *)(buf + off + 108)= cpu_to_le32(0x80200000); /* wmi_fw_sub_feat_caps */
    *(uint32_t *)(buf + off + 112)= cpu_to_le32(0);          /* num_dbs_hw_modes */
    *(uint32_t *)(buf + off + 116)= cpu_to_le32(0x03030303); /* txrx_chainmask: 2G/5G TX/RX */
    *(uint32_t *)(buf + off + 124)= cpu_to_le32(128);        /* num_msdu_desc */
    off += sre_len;

    /* TLV: service bitmap (WMI_TAG_ARRAY_UINT32, len=128 for 32×uint32_t, all zeros) */
    uint32_t bm_words = (128 + 31) / 32; /* WMI_MAX_SERVICE=128 → 4 words */
    uint32_t bm_len = bm_words * 4;
    tlv = (WmiTlv *)(buf + off);
    tlv->header = WMI_TLV_HDR(WMI_TAG_ARRAY_UINT32, bm_len);
    off += sizeof(*tlv);
    memset(buf + off, 0, bm_len);
    off += bm_len;

    uint32_t total_len = off;

    /* Wrap in HTC header for EP 2 (WMI RX) */
    uint8_t htc_buf[2048];
    HtcHdr *hdr2 = (HtcHdr *)htc_buf;
    hdr2->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    hdr2->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*hdr2), buf, total_len);

    wcn7850_wmi_send_event(s, pci_dev, s->htc_ep[HTC_EP_WMI].dl_pipe ?: 2,
                            htc_buf, sizeof(*hdr2) + total_len, 0);
    s->wmi_service_ready_sent = true;
}

/* Build and send a WMI_READY_EVENT to the driver */
static void wcn7850_send_wmi_ready(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[256];
    uint32_t off = 0;

    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    hdr->cmd_id = cpu_to_le32(WMI_READY_EVENTID);
    off += sizeof(*hdr);

    /* TLV: WMI_TAG_READY_EVENT = 35, len = sizeof(ath12k_wmi_ready_event_min_params) = 52 */
    /* struct ath12k_wmi_ready_event_min_params:
     *   fw_abi_vers(24) + mac_addr(8) + status(4) + num_dscp_table(4) +
     *   num_extra_mac_addr(4) + num_total_peers(4) + num_extra_peers(4) = 52
     */
    uint32_t re_len = 52;
    WmiTlv *tlv = (WmiTlv *)(buf + off);
    tlv->header = WMI_TLV_HDR(WMI_TAG_READY_EVENT, re_len);
    off += sizeof(*tlv);

    memset(buf + off, 0, re_len);
    /* fw_abi_vers at offset 0..23 (6×uint32_t), keep zero */
    /* mac_addr at offset 24..31 (ETH_ALEN + 2 padding), keep zero */
    *(uint32_t *)(buf + off + 32) = cpu_to_le32(0);  /* status = 0 (success) */
    *(uint32_t *)(buf + off + 36) = cpu_to_le32(0);  /* num_dscp_table */
    *(uint32_t *)(buf + off + 40) = cpu_to_le32(0);  /* num_extra_mac_addr */
    *(uint32_t *)(buf + off + 44) = cpu_to_le32(500); /* num_total_peers */
    *(uint32_t *)(buf + off + 48) = cpu_to_le32(0);  /* num_extra_peers */
    off += re_len;

    uint32_t total_len = off;

    uint8_t htc_buf[1024];
    HtcHdr *h2 = (HtcHdr *)htc_buf;
    h2->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    h2->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h2), buf, total_len);

    wcn7850_wmi_send_event(s, pci_dev, s->htc_ep[HTC_EP_WMI].dl_pipe ?: 2,
                            htc_buf, sizeof(*h2) + total_len, 0);
    s->wmi_ready_sent = true;
}

/* Initialize HTC EP0 after getting HTC_SETUP_COMPLETE */
static void wcn7850_htc_send_ready(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[64];
    HtcReadyMsg *ready = (HtcReadyMsg *)buf;

    ready->msg_id = cpu_to_le32(HTC_MSG_READY_ID);
    ready->credits = cpu_to_le32(128);
    ready->credit_size = cpu_to_le32(4096);
    ready->max_ep = cpu_to_le32(HTC_EP_MAX);
    ready->pad = 0;

    HtcHdr *hdr = (HtcHdr *)buf;
    hdr->hdr_info = cpu_to_le32(0); /* EP 0, flags 0 */
    hdr->ctrl_info = cpu_to_le32(HTC_MSG_READY_ID);

    /* Send via CE dest ring (CE pipe 1 = HTC control DL) */
    wcn7850_wmi_send_event(s, pci_dev, 1, buf, sizeof(*ready) + 0, 0);
    s->htc_credits = 128;
}

/* Handle HTC connect service request */
static void wcn7850_htc_handle_connect(WCN7850State *s, PCIDevice *pci_dev,
                                        const uint8_t *payload)
{
    const HtcConnSvc *req = (const HtcConnSvc *)payload;
    uint16_t svc_id = HTC_CONN_SVC_ID(req);
    uint16_t msg_id = HTC_CONN_MSG_ID(req);
    uint8_t resp_buf[64];
    HtcConnSvcResp *resp = (HtcConnSvcResp *)resp_buf;
    HtcHdr *hdr = (HtcHdr *)resp_buf;

    (void)msg_id;

    int ep = -1;
    if (svc_id == 0x0401) { /* WMI service */
        ep = HTC_EP_WMI;
    } else {
        return;
    }

    if (ep < 0 || ep >= HTC_EP_MAX)
        return;

    /* Assign endpoint and record mapping */
    s->htc_ep[ep].connected = true;
    s->htc_ep[ep].service_id = svc_id;
    s->htc_ep[ep].ul_pipe = 3;    /* WMI TX goes to CE pipe 3 */
    s->htc_ep[ep].dl_pipe = 2;    /* WMI RX comes from CE pipe 2 */

    uint32_t msg_svc_id = (svc_id << 16) | HTC_MSG_CONNECT_SERVICE_RESP_ID;
    resp->msg_svc_id = cpu_to_le32(msg_svc_id);
    resp->flags_len = cpu_to_le32(0 | (ep << 8) | (4096 << 16));

    memset(hdr, 0, sizeof(*hdr));
    hdr->hdr_info = cpu_to_le32(0);
    hdr->ctrl_info = cpu_to_le32(HTC_MSG_CONNECT_SERVICE_RESP_ID);

    /* Send response via CE pipe 1 (HTC control DL) */
    wcn7850_wmi_send_event(s, pci_dev, 1, resp_buf, sizeof(*resp) + 0, 0);

    /* After WMI service connected, send WMI_SERVICE_READY */
    if (svc_id == 0x0401) {
        qemu_log("WCN: WMI connected on EP %d (ul=%d dl=%d)\n",
                 ep, 3, 2);
        wcn7850_send_wmi_service_ready(s, pci_dev);
    }
}

/* Handle WMI command from driver (sent via HTC EP WMI_TX) */
static void wcn7850_handle_wmi_cmd(WCN7850State *s, PCIDevice *pci_dev,
                                    const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(WmiCmdHdr)) {
        qemu_log("WCN: WMI cmd too short (%u)\n", len);
        return;
    }
    const WmiCmdHdr *hdr = (const WmiCmdHdr *)payload;
    uint32_t cmd_id = le32_to_cpu(hdr->cmd_id) & 0xffffff;

    switch (cmd_id) {
    case WMI_INIT_CMDID:
        qemu_log("WCN: WMI INIT cmd\n");
        wcn7850_send_wmi_ready(s, pci_dev);
        break;
    default:
        qemu_log("WCN: WMI cmd 0x%x len=%u (unhandled)\n", cmd_id, len);
        break;
    }
}

/* Dispatch HTC message to appropriate handler */
static void wcn7850_htc_dispatch(WCN7850State *s, PCIDevice *pci_dev,
                                  uint8_t epid, const uint8_t *payload,
                                  uint32_t len)
{
    switch (epid) {
    case HTC_EP_CTRL: {
        HtcHdr *hdr = (HtcHdr *)(payload - 8);
        uint16_t msg_id = HTC_HDR_MSG_ID(hdr);

        switch (msg_id) {
        case HTC_MSG_SETUP_COMPLETE_EX_ID:
            qemu_log("WCN: HTC setup complete\n");
            wcn7850_htc_send_ready(s, pci_dev);
            break;
        case HTC_MSG_CONNECT_SERVICE_ID:
            wcn7850_htc_handle_connect(s, pci_dev, payload);
            break;
        default:
            qemu_log("WCN: unknown HTC ctrl msg %u\n", msg_id);
            break;
        }
        break;
    }
    case HTC_EP_WMI_TX: {
        wcn7850_handle_wmi_cmd(s, pci_dev, payload, len);
        break;
    }
    default:
        qemu_log("WCN: unhandled HTC EP %u len=%u\n", epid, len);
        break;
    }
}

/* Process CE source ring after driver writes HP doorbell.
 * Reads CE descriptors from guest memory, extracts HTC payload,
 * and dispatches to the HTC handler.
 */
static void wcn7850_process_ce_src(WCN7850State *s, PCIDevice *pci_dev,
                                    int ce_pipe)
{
    uint64_t base = s->ce_src_base[ce_pipe];
    uint32_t esize = s->ce_src_entry_size[ce_pipe] ?: 16;
    uint32_t ring_sz = s->ce_src_size[ce_pipe] ?: (WCN7850_CE_SRC_RING_SIZE * esize);
    uint32_t tp = s->ce_src_tp[ce_pipe];
    uint32_t hp = s->ce_src_hp[ce_pipe];

    if (!base || !ring_sz)
        return;

    while (tp != hp) {
        /* Read CE source descriptor */
        uint64_t desc_addr = base + tp;
        uint8_t desc[WCN7850_MAX_CE_DESC_SIZE];
        if (pci_dma_read(pci_dev, desc_addr, desc, esize) != MEMTX_OK)
            break;

        /* Extract buffer address (buffer_addr_low + addr_hi from buffer_addr_info) */
        uint64_t buf_addr = (uint64_t)le32_to_cpu(*(uint32_t *)desc) |
                            ((uint64_t)(le32_to_cpu(*(uint32_t *)(desc + 4)) & 0xff) << 32);
        if (!buf_addr) {
            tp += esize;
            if (tp >= ring_sz) tp = 0;
            continue;
        }

        /* Extract length from buffer_addr_info bits 31:16 */
        uint32_t info = le32_to_cpu(*(uint32_t *)(desc + 4));
        uint32_t buf_len = (info >> 16) & 0xffff;
        if (buf_len > 4096) buf_len = 4096;
        if (!buf_len) {
            tp += esize;
            if (tp >= ring_sz) tp = 0;
            continue;
        }

        /* Read the buffer data (contains HTC message) */
        uint8_t htc_buf[4096];
        if (pci_dma_read(pci_dev, buf_addr, htc_buf, buf_len) != MEMTX_OK) {
            tp += esize;
            if (tp >= ring_sz) tp = 0;
            continue;
        }

        /* Parse HTC header */
        HtcHdr *htc_hdr = (HtcHdr *)htc_buf;
        uint8_t epid = HTC_HDR_EPID(htc_hdr);
        uint32_t payload_len = HTC_HDR_PAYLOAD_LEN(htc_hdr);
        if (payload_len > buf_len - sizeof(*htc_hdr))
            payload_len = buf_len - sizeof(*htc_hdr);

        /* Dispatch to HTC handler */
        wcn7850_htc_dispatch(s, pci_dev, epid,
                              htc_buf + sizeof(*htc_hdr), payload_len);

        tp += esize;
        if (tp >= ring_sz) tp = 0;
    }

    s->ce_src_tp[ce_pipe] = tp;

    /* Write TP back to shared memory so driver HP updates work */
    if (s->ce_src_tp_addr[ce_pipe]) {
        uint32_t tp_le = cpu_to_le32(tp);
        pci_dma_write(pci_dev, s->ce_src_tp_addr[ce_pipe],
                      &tp_le, sizeof(tp_le));
    }
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
#
/* Annotated QMI/TLV hexdump to help compare model bytes with kernel decoder.
 * buf must point at the QMI header (QmiHdr) and len is the total size
 * including the 7-byte QMI header. prefix is a short label like "req"/"resp"/"ind".
 */
static void wcn7850_qmi_annotated_dump(const uint8_t *buf, size_t len, const char *prefix)
{
    const QmiHdr *hdr = (const QmiHdr *)buf;
    uint32_t payload_len = 0;
    size_t off = 0;

    if (!buf || len < sizeof(*hdr))
        return;
    payload_len = le16_to_cpu(hdr->msg_len);

    /* Full hex (with offsets) */
    qemu_log("WCN: QMI %s hex (len=%zu)\n", prefix, len);
    for (size_t i = 0; i < len; i += 16) {
        qemu_log("WCN: %04zx: ", i);
        for (size_t j = 0; j < 16 && i + j < len; j++)
            qemu_log("%02x ", buf[i + j]);
        qemu_log("\n");
    }

    qemu_log("WCN: QMI %s hdr type=%u txn=%u msg_id=0x%04x msg_len=%u\n",
             prefix, hdr->type, le16_to_cpu(hdr->txn_id), le16_to_cpu(hdr->msg_id), payload_len);

    /* Walk TLVs */
    off = sizeof(*hdr);
    while (off + 3 <= sizeof(*hdr) + payload_len && off + 3 <= len) {
        uint8_t tlv_type = buf[off];
        uint16_t tlv_len = buf[off + 1] | (buf[off + 2] << 8);
        qemu_log("WCN: QMI %s TLV @%zu type=0x%02x len=%u\n", prefix, off, tlv_type, tlv_len);

        /* Special-case REQUEST_MEM_IND TLV type 0x01 to decode mem_seg array and validate sizes */
        if (tlv_type == 0x01) {
            size_t p = off + 3;
            if (p + 1 <= len) {
                uint32_t mem_seg_len = buf[p]; /* QMI_DATA_LEN, 1-byte prefix */
                qemu_log("WCN: QMI %s mem_seg_len=%u\n", prefix, mem_seg_len);
                p += 1;
                for (uint32_t si = 0; si < mem_seg_len && p + 9 <= off + 3 + tlv_len && p + 9 <= len; si++) {
                    uint32_t size_le = le32_to_cpu(*(uint32_t *)(buf + p));
                    uint32_t type_le = le32_to_cpu(*(uint32_t *)(buf + p + 4));
                    uint32_t cfg_len = buf[p + 8]; /* 1-byte prefix */
                    qemu_log("WCN: QMI %s mem_seg[%u] size=%u type=%d mem_cfg_len=%u (raw @%zu..%zu)\n",
                             prefix, si, size_le, (int32_t)type_le, cfg_len, p, p + 8);
                    /* Basic validation: size should be non-zero and reasonable */
                    if (size_le == 0) {
                        qemu_log("WCN: QMI %s WARNING: mem_seg[%u] has zero size\n", prefix, si);
                    }
                    if (type_le > 0x10) {
                        qemu_log("WCN: QMI %s WARNING: mem_seg[%u] unexpected type=%d\n", prefix, si, (int32_t)type_le);
                    }
                    p += 9;
                    /* skip mem_cfg if present (each = offset u64 + size u32 + secure_flag u8 = 13) */
                    p += (size_t)cfg_len * 13;
                }
            }
        }

        off += 3 + tlv_len;
    }
}


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

    /* TLVs: result(7) + num_phy(5) + board_id(8) + single_chip_mlo_support(5) */
    uint32_t tlv_len = 7 + 5 + 8 + 5;
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
    /* TLV type 0x10: num_phy, length 2 = valid(1) + value(1) */
    tlv[0] = 0x10; tlv[1] = 0x02; tlv[2] = 0x00;
    tlv[3] = 0x01; /* valid = true */
    tlv[4] = 0x01; /* num_phy = 1 */
    tlv += 5;
    /* TLV type 0x11: board_id, length 5 = valid(1) + board_id(4) */
    tlv[0] = 0x11; tlv[1] = 0x05; tlv[2] = 0x00;
    tlv[3] = 0x01; /* valid = true */
    tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00; tlv[7] = 0x00; /* board_id = 0 */
    tlv += 8;
    /* TLV type 0x13: single_chip_mlo_support, length 2 = valid(1) + value(1) */
    tlv[0] = 0x13; tlv[1] = 0x02; tlv[2] = 0x00;
    tlv[3] = 0x01; /* valid = true */
    tlv[4] = 0x00; /* single_chip_mlo_support = false */

    return total;
}

/* Build CAP response with chip/board/fw info */
static uint32_t wcn7850_qmi_build_cap_resp(uint8_t *buf, uint32_t buf_size,
                                             uint16_t txn_id)
{
    QmiHdr *hdr = (QmiHdr *)buf;
    uint8_t *tlv;
    /* result + chip_info(8) + board_info(4) + fw_version_info(4+1+9) */
    const char *fw_ts = "2025-01-01";
    uint8_t fw_ts_len = strlen(fw_ts);
    uint32_t tlv_val = 0;
    uint32_t tlv_len = 0;

    /* result TLV (type=0x02, val=4 bytes): 3 header + 4 = 7 */
    /* chip_info TLV (type=0x10): opt_flag(1) + chip_id(4) + chip_family(4) = 9 */
    /* board_info TLV (type=0x11): opt_flag(1) + board_id(4) = 5 */
    /* fw_version_info TLV (type=0x13): opt_flag(1) + fw_version(4)
     *   + string_len(1) + fw_ts_len */
    tlv_len = 7 + (3 + 1 + 8) + (3 + 1 + 4) + (3 + 1 + 4 + 1 + fw_ts_len);
    uint32_t total = sizeof(*hdr) + tlv_len;

    if (total > buf_size)
        return 0;

    hdr->type = QMI_TYPE_RESP;
    hdr->txn_id = cpu_to_le16(txn_id);
    hdr->msg_id = cpu_to_le16(QMI_WLFW_CAP_REQ);
    hdr->msg_len = cpu_to_le16(tlv_len);

    tlv = buf + sizeof(*hdr);
    /* TLV: result = success (type 0x02, len 0x04) */
    tlv[0] = 0x02; tlv[1] = 0x04; tlv[2] = 0x00;
    tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00;
    tlv += 7;
    /* TLV: chip_info (type 0x10, len 0x09): opt_flag(1) + chip_id(4) + chip_family(4) */
    tlv[0] = 0x10; tlv[1] = (1 + 8) & 0xff; tlv[2] = 0x00;
    tlv[3] = 0x01; /* chip_info_valid = true */
    tlv_val = cpu_to_le32(2);
    memcpy(&tlv[4], &tlv_val, 4); /* chip_id = 2 */
    tlv_val = cpu_to_le32(0);
    memcpy(&tlv[8], &tlv_val, 4); /* chip_family = 0 */
    tlv += 3 + 1 + 8;
    /* TLV: board_info (type 0x11, len 0x05): opt_flag(1) + board_id(4) */
    tlv[0] = 0x11; tlv[1] = (1 + 4) & 0xff; tlv[2] = 0x00;
    tlv[3] = 0x01; /* board_info_valid = true */
    tlv_val = cpu_to_le32(0);
    memcpy(&tlv[4], &tlv_val, 4); /* board_id = 0 */
    tlv += 3 + 1 + 4;
    /* TLV: fw_version_info (type 0x13): opt_flag(1) + fw_version(4) + string */
    tlv[0] = 0x13;
    tlv[1] = (1 + 4 + 1 + fw_ts_len) & 0xff;
    tlv[2] = ((1 + 4 + 1 + fw_ts_len) >> 8) & 0xff;
    tlv[3] = 0x01; /* fw_version_info_valid = true */
    tlv_val = cpu_to_le32(1);
    memcpy(&tlv[4], &tlv_val, 4); /* fw_version = 1 */
    tlv[8] = fw_ts_len; /* string length prefix (1 byte) */
    memcpy(&tlv[9], fw_ts, fw_ts_len); /* string content */

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


/* Helper: send a QMI indication (type=4) with empty payload via DL channel */
static bool wcn7850_send_qmi_ind(WCN7850State *s, PCIDevice *pci_dev,
                                  uint64_t ch_ctxt_base,
                                  uint64_t er_ctxt_addr,
                                  uint16_t msg_id)
{
    uint8_t qmi_buf[64];
    QmiHdr *hdr = (QmiHdr *)qmi_buf;
    uint8_t qrtr_buf[128];
    size_t qrtr_len;

    memset(qmi_buf, 0, sizeof(qmi_buf));
    hdr->type = QMI_TYPE_IND;
    hdr->txn_id = cpu_to_le16(0);
    hdr->msg_id = cpu_to_le16(msg_id);
    /* Build payload for specific indications when needed. */
    if (msg_id == QMI_WLFW_REQUEST_MEM_IND) {
        /* Build TLV "mem_seg" (TLV type 0x01). The guest kernel EI
         * (qmi_wlanfw_request_mem_ind_msg_v01_ei) declares mem_seg_len and
         * mem_cfg_len as QMI_DATA_LEN with elem_size == sizeof(u8), so both
         * are 1-byte length prefixes on the wire. Per mem_seg entry is:
         *   size      (4 bytes LE, u32)
         *   type      (4 bytes LE, signed enum)
         *   mem_cfg_len (1 byte)
         *   mem_cfg[mem_cfg_len] each (offset u64 + size u32 + secure_flag u8)
         * We send one DDR segment of 2 MiB with mem_cfg_len = 0.
         */
        uint8_t *tlv = qmi_buf + sizeof(*hdr);
        uint32_t seg_size = 0x00200000; /* 2 MiB */
        uint32_t seg_type = 1; /* QMI_WLANFW_MEM_TYPE_DDR_V01 */
        /* payload = mem_seg_len(1) + mem_seg{ size(4)+type(4)+mem_cfg_len(1) } */
        uint32_t tlv_payload_len = 1 + (4 + 4 + 1);

        /* TLV header: type(1), len(2 little-endian) */
        tlv[0] = 0x01;
        tlv[1] = tlv_payload_len & 0xff;
        tlv[2] = (tlv_payload_len >> 8) & 0xff;

        /* mem_seg_len = 1 (1 byte) */
        tlv[3] = 0x01;

        /* mem_seg[0].size (little-endian) */
        memcpy(&tlv[4], &(uint32_t){ cpu_to_le32(seg_size) }, 4);
        /* mem_seg[0].type (little-endian signed) */
        memcpy(&tlv[8], &(uint32_t){ cpu_to_le32(seg_type) }, 4);
        /* mem_seg[0].mem_cfg_len = 0 (1 byte) */
        tlv[12] = 0x00;

    /* QMI header msg_len is the TOTAL body length = 3-byte TLV header + value.
     * The TLV header's own length field (tlv[1..2]) is just the value length. */
    hdr->msg_len = cpu_to_le16(tlv_payload_len + 3);
    /* Validation: verify TLV length field and mem_seg encoding match what we built */
    {
        uint16_t check_msg_len = le16_to_cpu(hdr->msg_len);
        if (check_msg_len != tlv_payload_len + 3) {
            qemu_log("WCN: QMI ind build error: hdr->msg_len (%u) != computed tlv_payload_len+3 (%u)\n",
                     check_msg_len, tlv_payload_len + 3);
        }
        /* quick decode of first mem_seg entry to ensure endianness/values */
        if (tlv_payload_len >= 5) {
            uint8_t *p = tlv + 3; /* points to mem_seg_len (1 byte) */
            uint32_t mem_seg_len_enc = p[0];
            p += 1;
            if (mem_seg_len_enc >= 1 && (size_t)(p + 8 - qmi_buf) <= sizeof(qmi_buf)) {
                uint32_t enc_size = le32_to_cpu(*(uint32_t *)p);
                uint32_t enc_type = (int32_t)le32_to_cpu(*(uint32_t *)(p + 4));
                qemu_log("WCN: QMI ind self-check mem_seg_len=%u first.size=%u first.type=%d\n",
                         mem_seg_len_enc, enc_size, (int32_t)enc_type);
                if (enc_size != seg_size) {
                    qemu_log("WCN: QMI ind WARNING: encoded seg_size=%u != intended=%u\n",
                             enc_size, seg_size);
                }
            }
        }
    }
    } else {
        hdr->msg_len = cpu_to_le16(0);
    }

    /* Payload for QRTR is the QMI header + QMI payload (hdr->msg_len bytes).
     * Compute the actual QMI payload size from the QMI header (stored little-endian).
     */
    uint32_t qmi_payload_size = sizeof(*hdr) + le16_to_cpu(hdr->msg_len);

    /* Debug: annotated dump of the QMI bytes we're about to send */
    wcn7850_qmi_annotated_dump(qmi_buf, qmi_payload_size, "ind");

    qrtr_len = wcn7850_build_qrtr_pkt(qrtr_buf, sizeof(qrtr_buf),
                                       QRTR_TYPE_DATA,
                                       QRTR_NODE_FW, QRTR_PORT_QMI_SERVICE,
                                       s->qmi_client_node, s->qmi_client_port,
                                       qmi_buf, qmi_payload_size);
    if (!qrtr_len)
        return false;

    uint64_t dl_ctxt_addr = ch_ctxt_base + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
    if (!wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr,
                                0, qrtr_buf, qrtr_len)) {
        qemu_log("WCN: QMI ind 0x%04x write DL failed\n", msg_id);
        return false;
    }

    qemu_log("WCN: sent QMI ind 0x%04x\n", msg_id);
    return true;
}

/* Process an incoming QMI message and generate response */
static void wcn7850_process_qmi(WCN7850State *s, PCIDevice *pci_dev,
                                  uint64_t ch_ctxt_addr,
                                  uint64_t er_ctxt_addr,
                                  uint32_t er_index,
                                  uint32_t qrtr_src_node,
                                  uint32_t qrtr_src_port,
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
    if (data_len) {
        wcn7850_qmi_annotated_dump(data, data_len, "req");
    }

    s->qmi_client_node = qrtr_src_node;
    s->qmi_client_port = qrtr_src_port;

    switch (msg_id) {
    case QMI_WLFW_PHY_CAP_REQ:
        resp_len = wcn7850_qmi_build_phy_cap_resp(resp_buf, sizeof(resp_buf), txn_id);
        break;
    case QMI_WLFW_IND_REGISTER_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_HOST_CAP_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        /* After HOST_CAP, send REQUEST_MEM_IND to start memory negotiation */
        s->qmi_pending_ind = QMI_WLFW_REQUEST_MEM_IND;
        break;
    case QMI_WLFW_RESPOND_MEM_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        /* After memory response, send FW_MEM_READY_IND to continue boot */
        s->qmi_pending_ind = QMI_WLFW_FW_MEM_READY_IND;
        break;
    case QMI_WLFW_CAP_REQ:
        resp_len = wcn7850_qmi_build_cap_resp(resp_buf, sizeof(resp_buf), txn_id);
        break;
    case QMI_WLFW_BDF_DOWNLOAD_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_M3_INFO_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        /* After M3 info, send FW_READY_IND to signal firmware is ready */
        s->qmi_pending_ind = QMI_WLFW_FW_READY_IND;
        break;
    case QMI_WLFW_WLAN_INI_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    case QMI_WLFW_WLAN_CFG_REQ:
        resp_len = wcn7850_qmi_build_cfg_resp(resp_buf, sizeof(resp_buf), txn_id);
        break;
    case QMI_WLFW_WLAN_MODE_REQ:
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        s->qmi_client_node = qrtr_src_node;
        s->qmi_client_port = qrtr_src_port;
        break;
    default:
        qemu_log("WCN: unknown QMI msg 0x%04x -> ack\n", msg_id);
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    }

    if (!resp_len)
        return;

    /* Wrap the QMI response in QRTR DATA */
    qemu_log("WCN: sending QMI response msg_id=0x%04x len=%u\n", msg_id, resp_len);
    if (resp_len) {
        wcn7850_qmi_annotated_dump(resp_buf, resp_len, "resp");
    }

    /* Build QRTR packet: from FW node to the requesting QMI client */
    uint8_t qrtr_pkt[512];
    size_t qrtr_len = wcn7850_build_qrtr_pkt(qrtr_pkt, sizeof(qrtr_pkt),
                                               QRTR_TYPE_DATA,
                                               QRTR_NODE_FW, QRTR_PORT_QMI_SERVICE,
                                               qrtr_src_node, qrtr_src_port,
                                               resp_buf, resp_len);
    if (qrtr_len) {
        /* Write the QRTR packet to DL channel 21 */
        uint64_t dl_ctxt_addr = ch_ctxt_addr + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
        if (!wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr,
                                    er_index, qrtr_pkt, qrtr_len)) {
            qemu_log("WCN: failed to write DL data\n");
        }
    }

    /* Send any pending QMI indication (REQUEST_MEM_IND, FW_MEM_READY_IND) */
    if (s->qmi_pending_ind) {
        uint16_t ind = s->qmi_pending_ind;
        qemu_log("WCN: sending pending QMI ind 0x%04x\n", ind);
        if (wcn7850_send_qmi_ind(s, pci_dev, ch_ctxt_addr, er_ctxt_addr, ind)) {
            s->qmi_pending_ind = 0;
        } else {
            qemu_log("WCN: will retry pending QMI ind 0x%04x\n", ind);
        }
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
    if (type == QRTR_TYPE_DATA) {
        qemu_log("WCN: QRTR DATA hex");
        for (size_t _i = 0; _i < payload_size && _i < 64; _i++)
            qemu_log(" %02x", payload[_i]);
        qemu_log("\n");
    }

    if (payload_size + sizeof(*hdr) > data_len)
        return;

    switch (type) {
    case QRTR_TYPE_DATA:
        wcn7850_process_qmi(s, pci_dev, ch_ctxt_addr, er_ctxt_addr, er_index,
                             src_node, src_port,
                             payload, payload_size);
        break;
    default:
        qemu_log("WCN: unhandled QRTR type %u\n", type);
        break;
    }
}

/* Process the MHI command ring: walk TREs between rp and wp and post a
 * command-completion event for each (CCS = SUCCESS) so the host can move
 * channels to ENABLED. The device never needs to program ring context. */
static void wcn7850_process_cmd_ring(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t ctxt[48];
    uint64_t rbase, rlen, rp, wp, rb, rl, rpv;

    qemu_log("WCN: process_cmd_ring ctxt=0x%lx\n", (unsigned long)s->cmd_ctxt_addr);
    if (!s->cmd_ctxt_addr)
        return;
    if (pci_dma_read(pci_dev, s->cmd_ctxt_addr, ctxt, sizeof(ctxt)) != MEMTX_OK) {
        qemu_log("WCN: cmd ctxt read failed\n");
        return;
    }
    /* mhi_cmd_ctxt: 3x reserved __le32, then __le64 rbase/rlen/rp/wp */
    memcpy(&rb, ctxt + 12, 8); rbase = le64_to_cpu(rb);
    memcpy(&rl, ctxt + 20, 8); rlen  = le64_to_cpu(rl);
    memcpy(&rpv, ctxt + 28, 8); rp    = le64_to_cpu(rpv);
    {
        uint32_t lo = *(uint32_t *)(s->bar0_always_on + WCN7850_CRDB_LOWER);
        uint32_t hi = *(uint32_t *)(s->bar0_always_on + WCN7850_CRDB_HIGHER);
        wp = ((uint64_t)hi << 32) | lo;
    }
    if (!rbase || !rlen)
        return;
    if (rp < rbase || rp >= rbase + rlen) rp = rbase;
    if (wp < rbase || wp >= rbase + rlen) wp = rbase;
    qemu_log("WCN: cmd ring rbase=0x%lx rp=0x%lx wp=0x%lx rlen=0x%lx\n",
             (unsigned long)rbase, (unsigned long)rp, (unsigned long)wp,
             (unsigned long)rlen);

    while (rp != wp) {
        MhiTre tre;

        if (!wcn7850_read_tre(pci_dev, rbase, rp - rbase, &tre)) {
            qemu_log("WCN: cmd TRE read failed\n");
            break;
        }
        uint32_t dword1 = le32_to_cpu(tre.dword1);
        uint32_t cmdtype = (dword1 >> 16) & 0xff;
        uint32_t chid = (dword1 >> 24) & 0xff;
        qemu_log("WCN: CMD ch=%u type=0x%x\n", chid, cmdtype);
        fprintf(stderr, "WCN: CMD ch=%u type=0x%x\n", chid, cmdtype);

        uint8_t ev[16];
        uint64_t le_ptr = cpu_to_le64(rp);
        uint32_t le_dw0 = cpu_to_le32(0x01000000u); /* code=CCS_SUCCESS */
        uint32_t le_dw1 = cpu_to_le32((chid << 24) | (0x21u << 16));
        memcpy(ev, &le_ptr, 8);
        memcpy(ev + 8, &le_dw0, 4);
        memcpy(ev + 12, &le_dw1, 4);
        wcn7850_post_event(s, pci_dev, s->er_ctxt_addr, 0, ev, sizeof(ev));

        rp += sizeof(MhiTre);
        if (rp >= rbase + rlen)
            rp = rbase;
    }

    uint64_t wrp = cpu_to_le64(rp);
    memcpy(ctxt + 28, &wrp, 8);
    pci_dma_write(pci_dev, s->cmd_ctxt_addr, ctxt, sizeof(ctxt));
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
                if (!s->qmi_newserver_delivered) {
                    fprintf(stderr, "WCN: first IPCR UL data — NEW_SERVER delivered\n");
                    s->qmi_newserver_delivered = true;
                }
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

    if (db_offset == 0 && ch_id == MHI_CHAN_IPCR_UL) {
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
    memcpy(ev + 12, &(uint32_t){ cpu_to_le32((0x40u << 16) | 2u) }, sizeof(uint32_t));

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

    /* MHI command ring doorbell */
    if (addr == WCN7850_CRDB_LOWER || addr == WCN7850_CRDB_HIGHER) {
        memcpy(s->bar0_always_on + addr, &val, size);
        wcn7850_process_cmd_ring(s, pci_dev);
        return;
    }

    /* MHI reset */
    if (addr == WCN7850_MHICTRL && (val & BIT(1))) {
        s->mhi_state = 0;
        s->bhi_downloaded = false;
        s->bhi_status = 0;
        s->bhi_execenv = 2; /* stay in AMSS; we emulate mission mode */
        return;
    }

    /* MHI state transition to M0 -> mission mode */
    if (addr == WCN7850_MHICTRL && ((val & 0x0000ff00u) == 0x00000200u)) {
        s->mhi_state = 2;
        wcn7850_emit_state_change_event(s, pci_dev, 2);
        s->ctrl_event_pending = true;
        timer_mod(s->ctrl_event_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000LL);
        qemu_log("WCN: M0 -> schedule mission mode\n");
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

    /* Shadow register doorbells (0x8fc + 4*n).
     * CE DST is SRC_DIR → driver writes HP (producer) via shadow.
     * CE DST STATUS is DST_DIR → driver writes TP (consumer) via shadow.
     * Shadow order per WCN7850 CE init order:
     *   idx 0: CE0 src HP
     *   idx 1: CE1 dst HP (CE DST configured as SRC_DIR)
     *   idx 2: CE1 dst status TP
     *   idx 3: CE2 dst HP (CE DST configured as SRC_DIR)
     *   idx 4: CE2 dst status TP
     *   idx 5: CE3 src HP
     */
    if (addr >= WCN7850_SHADOW_BASE &&
        addr < WCN7850_SHADOW_BASE + WCN7850_SHADOW_MAX * 4) {
        memcpy(s->bar0_always_on + addr, &val, size);
        unsigned int sidx = (addr - WCN7850_SHADOW_BASE) / 4;
        int ce_pipe = -1;
        int ce_type = -1; /* 0=src HP, 1=dst HP, 2=dst status TP */
        if (sidx == 0) { ce_pipe = 0; ce_type = 0; }
        else if (sidx == 1) { ce_pipe = 1; ce_type = 1; } /* CE1 dst HP */
        else if (sidx == 2) { ce_pipe = 1; ce_type = 2; } /* CE1 dst status TP */
        else if (sidx == 3) { ce_pipe = 2; ce_type = 1; } /* CE2 dst HP */
        else if (sidx == 4) { ce_pipe = 2; ce_type = 2; } /* CE2 dst status TP */
        else if (sidx == 5) { ce_pipe = 3; ce_type = 0; } /* CE3 src HP */
        if (ce_pipe >= 0) {
            uint32_t doorbell_val = val;
            if (ce_type == 0) {
                s->ce_src_hp[ce_pipe] = doorbell_val;
                wcn7850_process_ce_src(s, pci_dev, ce_pipe);
            } else if (ce_type == 1) {
                /* Driver produced new entries in the CE DST ring.
                 * CE DST is SRC_DIR: driver writes HP via shadow.
                 * Track pending entries to handle HP wrap to 0. */
                uint32_t prev_hp = s->ce_dst_drv_hp_prev[ce_pipe];
                s->ce_dst_drv_hp[ce_pipe] = doorbell_val;
                if (!s->ce_dst_doorbell_seen[ce_pipe]) {
                    s->ce_dst_doorbell_seen[ce_pipe] = true;
                    /* First doorbell: assume full ring */
                    s->ce_dst_pending[ce_pipe] = (s->ce_dst_size[ce_pipe] ?:
                        (WCN7850_CE_DST_RING_SIZE * (s->ce_dst_entry_size[ce_pipe] ?: 16)))
                        / (s->ce_dst_entry_size[ce_pipe] ?: 16);
                } else if (doorbell_val != prev_hp) {
                    uint32_t ring_sz = s->ce_dst_size[ce_pipe] ?:
                        (WCN7850_CE_DST_RING_SIZE * (s->ce_dst_entry_size[ce_pipe] ?: 16));
                    uint32_t delta = (doorbell_val - prev_hp + ring_sz) % ring_sz;
                    s->ce_dst_pending[ce_pipe] += delta / (s->ce_dst_entry_size[ce_pipe] ?: 16);
                }
                s->ce_dst_drv_hp_prev[ce_pipe] = doorbell_val;
            } else if (ce_type == 2) {
                /* Driver consumed status entries: no action needed */
            }
        }
        return;
    }

    if (addr < WCN7850_WINDOW_START) {
        memcpy(s->bar0_always_on + addr, &val, size);
        return;
    }

    /* Window region: decode based on window_select */
    if (addr < WCN7850_WINDOW_START + WCN7850_WINDOW_SIZE) {
        hwaddr win_off = addr - WCN7850_WINDOW_START;

        /* CE registers: window_select = WCN7850_CE0_SRC_BASE / WCN7850_WINDOW_SIZE */
        if (s->window_select == (WCN7850_CE0_SRC_BASE / WCN7850_WINDOW_SIZE)) {
            /* Determine CE pipe (0..8) from window offset */
            int ce_pipe = -1;
            bool is_dst = false;
            if (win_off < WCN7850_CE_STRIDE * WCN7850_CE_COUNT) {
                ce_pipe = win_off / WCN7850_CE_STRIDE;
                /* dest rings are at offset 0x1000 within each CE block */
                if ((win_off % WCN7850_CE_STRIDE) >= 0x1000) {
                    is_dst = true;
                }
            }

            if (ce_pipe >= 0 && ce_pipe < WCN7850_CE_COUNT) {
                hwaddr ce_blk_off = win_off % WCN7850_CE_STRIDE;
                /* R0 config registers (within first 0x100 of CE block for src,
                   0x100 offset within CE block for dest) */
                if (ce_blk_off < 0x100) {
                    memcpy(s->window_memory + win_off, &val, size);
                    /* Read all config registers for this CE block */
                    uint32_t base_lsb_cfg, base_msb_cfg, ring_id_cfg;
                    hwaddr cfg_base_src = win_off - ce_blk_off;
                    if (is_dst) {
                        /* For dest: remove the 0x1000 offset within CE block */
                        cfg_base_src = (win_off - ce_blk_off) - 0x1000;
                    }
                    base_lsb_cfg = *(uint32_t *)(s->window_memory + cfg_base_src + WCN7850_CE_RING_BASE_LSB);
                    base_msb_cfg = *(uint32_t *)(s->window_memory + cfg_base_src + WCN7850_CE_RING_BASE_MSB);
                    ring_id_cfg  = *(uint32_t *)(s->window_memory + cfg_base_src + WCN7850_CE_RING_ID);

                    if (base_lsb_cfg || base_msb_cfg) {
                        uint64_t base_addr = base_lsb_cfg |
                            ((uint64_t)(base_msb_cfg & 0xff) << 32);
                        uint32_t ring_sz = (base_msb_cfg >> 8) & 0xffff;
                        uint32_t entry_size = (ring_id_cfg & 0xff) * 4;
                        if (!entry_size) entry_size = 16;
                        if (!ring_sz) ring_sz = 512 * entry_size;
                        if (is_dst) {
                            s->ce_dst_base[ce_pipe] = base_addr;
                            s->ce_dst_size[ce_pipe] = ring_sz;
                            s->ce_dst_entry_size[ce_pipe] = entry_size;
                            /* Also check for CE dst status ring config at offset 0x58 */
                            uint32_t sts_lsb = *(uint32_t *)(s->window_memory + cfg_base_src + 0x58);
                            uint32_t sts_msb = *(uint32_t *)(s->window_memory + cfg_base_src + 0x5c);
                            if (sts_lsb || sts_msb) {
                                s->ce_sts_base[ce_pipe] = sts_lsb |
                                    ((uint64_t)(sts_msb & 0xff) << 32);
                                uint32_t sts_rind = *(uint32_t *)(s->window_memory + cfg_base_src + 0x60);
                                 s->ce_sts_size[ce_pipe] = (sts_msb >> 8) & 0xffff;
                                 s->ce_sts_entry_size[ce_pipe] = (sts_rind & 0xff) * 4;
                                 /* Capture status ring HP RDP address at offset 0x6C and 0x70 */
                                 uint32_t hp_lsb = *(uint32_t *)(s->window_memory + cfg_base_src + 0x6c);
                                 uint32_t hp_msb = *(uint32_t *)(s->window_memory + cfg_base_src + 0x70);
                                 s->ce_sts_hp_addr[ce_pipe] = hp_lsb |
                                     ((uint64_t)(hp_msb & 0xff) << 32);
                             }
                        } else {
                            s->ce_src_base[ce_pipe] = base_addr;
                            s->ce_src_size[ce_pipe] = ring_sz;
                            s->ce_src_entry_size[ce_pipe] = entry_size;
                        }
                    }
                    return;
                }
                /* R2 pointer registers (HP/TP at offset 0x400/0x404) */
                if (ce_blk_off >= WCN7850_CE_RING_HP_OFFSET &&
                    ce_blk_off < WCN7850_CE_RING_HP_OFFSET + 0x10) {
                    memcpy(s->window_memory + win_off, &val, size);
                    /* These are the actual HW HP/TP - also stored as shadow regs */
                    if (ce_blk_off == WCN7850_CE_RING_HP_OFFSET) {
                        if (is_dst) {
                            s->ce_dst_hp[ce_pipe] = val;
                        } else {
                            s->ce_src_hp[ce_pipe] = val;
                            /* Process CE source ring when HP advances */
                            wcn7850_process_ce_src(s, pci_dev, ce_pipe);
                        }
                    } else if (ce_blk_off == WCN7850_CE_RING_TP_OFFSET) {
                        if (is_dst) {
                            s->ce_dst_tp[ce_pipe] = val;
                        } else {
                            s->ce_src_tp[ce_pipe] = val;
                        }
                    } else if (ce_blk_off == (WCN7850_CE_RING_HP_OFFSET + 8)) {
                        /* Status ring HP at CE dest block offset 0x408 */
                        if (is_dst) {
                            s->ce_sts_hp[ce_pipe] = val;
                        }
                    }
                    return;
                }
            }
        }

        memcpy(s->window_memory + win_off, &val, size);
        return;
    }
}

/* ===== After mission mode, send FW_READY_IND and NEW_SERVER ===== */
static bool wcn7850_mission_mode_setup(WCN7850State *s, PCIDevice *pci_dev)
{
    fprintf(stderr, "WCN: mission_mode_setup er=0x%lx chan=0x%lx\n",
            (unsigned long)s->er_ctxt_addr,
            (unsigned long)s->chan_ctxt_addr);
    if (!s->er_ctxt_addr || !s->chan_ctxt_addr)
        return false;

    /* Send QRTR NEW_SERVER so ath12k can kernel_connect() to us. */
    fprintf(stderr, "WCN: sending NEW_SERVER\n");
    if (!wcn7850_send_new_server(s, pci_dev, s->chan_ctxt_addr,
                                 s->er_ctxt_addr)) {
        return false;
    }
    s->qmi_newserver_sent = true;

    /* Mark resent so the CE poll timer doesn't retry */
    s->qmi_newserver_resent = true;
    s->qmi_service_active = true;
    s->qmi_boot_pending = false;
    s->ce_ready = true;
    timer_mod(s->ce_poll_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000LL);
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

/* CE poll timer: scan CE source rings for new descriptors from driver.
 * Also retries NEW_SERVER when the host (qrtr_mhi) posts DL receive buffers. */
static void wcn7850_ce_poll_timer(void *opaque)
{
    WCN7850State *s = opaque;
    PCIDevice *pci_dev = PCI_DEVICE(s);

    /* Retry NEW_SERVER if it was sent before qrtr_mhi opened the channel.
     * Run this check even before ce_ready, because mission_mode_setup is
     * stuck retrying FW_READY_IND and will never set ce_ready=true until
     * NEW_SERVER is redelivered into a live channel that posts buffers.
     * Keep retrying until we get evidence the host received it (first
     * QMI data on IPCR UL sets qmi_newserver_delivered). */
    if (s->qmi_newserver_sent && !s->qmi_newserver_delivered && !s->qmi_newserver_resent) {
        MhiChanCtxt dl_ctxt;
        uint64_t dl_ctxt_addr = s->chan_ctxt_addr + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
        bool has_chan = s->chan_ctxt_addr != 0;
        bool read_ok = false;
        bool rp_ne_wp = false;
        if (has_chan) {
            read_ok = wcn7850_read_chan_ctxt(pci_dev, dl_ctxt_addr, &dl_ctxt);
            if (read_ok)
                rp_ne_wp = dl_ctxt.rp != dl_ctxt.wp;
        }
        if (!has_chan || !read_ok || !rp_ne_wp) {
            fprintf(stderr, "WCN: ce_poll retry_NS check has_chan=%d read_ok=%d rp_ne_wp=%d\n",
                    has_chan, read_ok, rp_ne_wp);
        }
        if (has_chan && read_ok && rp_ne_wp) {
            fprintf(stderr, "WCN: DL ring has buffers, retrying NEW_SERVER\n");
            if (wcn7850_send_new_server(s, pci_dev, s->chan_ctxt_addr,
                                         s->er_ctxt_addr)) {
                s->qmi_newserver_resent = true;
                fprintf(stderr, "WCN: NEW_SERVER retry succeeded\n");
            }
        }
    }

    /* Retry pending QMI indication if DL ring has buffers */
    if (s->qmi_pending_ind && s->chan_ctxt_addr) {
        MhiChanCtxt dl_ctxt;
        uint64_t dl_ctxt_addr = s->chan_ctxt_addr + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
        if (wcn7850_read_chan_ctxt(pci_dev, dl_ctxt_addr, &dl_ctxt) &&
            dl_ctxt.rp != dl_ctxt.wp) {
            qemu_log("WCN: retrying pending QMI ind 0x%04x (DL ring has entries)\n",
                     s->qmi_pending_ind);
            if (wcn7850_send_qmi_ind(s, pci_dev, s->chan_ctxt_addr,
                                      s->er_ctxt_addr, s->qmi_pending_ind)) {
                s->qmi_pending_ind = 0;
            }
        }
    }

    if (!s->ce_ready)
        goto reschedule;

    for (int i = 0; i < WCN7850_CE_COUNT; i++) {
        if (s->ce_src_base[i] && s->ce_src_hp[i] != s->ce_src_tp[i]) {
            wcn7850_process_ce_src(s, pci_dev, i);
        }
    }

reschedule:
    timer_mod(s->ce_poll_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000LL); /* 1ms */
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

    s->mhi_state = 0;
    s->bhi_execenv = 2; /* MHI_EE_AMSS: skip fw load, go straight to mission mode */
    s->bhi_downloaded = false;
    s->er_ctxt_addr = 0;
    s->ctrl_event_pending = false;
    s->qmi_boot_pending = false;
    s->qmi_newserver_sent = false;
    s->qmi_newserver_resent = false;
    s->qmi_newserver_delivered = false;

    s->qmi_pending_ind = 0;
    s->qmi_client_node = QRTR_NODE_HOST;
    s->qmi_client_port = 0;
    s->ctrl_event_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        wcn7850_ctrl_event_timer_ext, s);
    s->ce_poll_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                     wcn7850_ce_poll_timer, s);
    timer_mod(s->ce_poll_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000LL); /* start 1ms after realize */
    s->qmi_service_active = false;

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
    timer_free(s->ce_poll_timer);
    g_free(s->bar0_always_on);
    g_free(s->window_memory);
    g_free(s->srng_memory);
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
