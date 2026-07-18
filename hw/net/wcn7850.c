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
/* With static_window_map the driver maps CE to 2*WINDOW_START and UMAC to
 * 3*WINDOW_START and accesses them directly, without the window register. */
#define WCN7850_CE_WINDOW_BASE   (2 * WCN7850_WINDOW_START)
#define WCN7850_UMAC_WINDOW_BASE (3 * WCN7850_WINDOW_START)
#define WCN7850_WINDOW_REG 0x310c

/* CE shadow register base (doorbell writes for CE HP/TP) */
#define WCN7850_SHADOW_BASE     0x000008fc
#define WCN7850_SHADOW_MAX      64

/* CE MSI base vector (WCN7850: MHI=0-2, CE=3-7, DP=8-15) */
#define WCN7850_CE_MSI_BASE     3

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

/* WCN7850 MSI layout (ath12k_msi_config in wifi7/pci.c):
 *   MHI: base_vector 0 (vectors 0..2)
 *   CE:  base_vector 3 (vectors 3..7 for CE0..CE3, CE5)
 * The ath12k driver assigns each CE pipe an MSI vector index of
 *   CE_MSI_BASE + (count of non-CE_ATTR_DIS_INTR CEs before this pipe),
 * which matches ath12k_pci_get_ce_msi_idx. */
#define WCN7850_CE_MSI_BASE 3

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

/* WBM2SW release ring R0 registers (per-ring, stride 0x78) */
#define WCN7850_WBM_RING_BASE_LSB(_n)   (0x00000e08 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_WBM_RING_BASE_MSB(_n)   (0x00000e0c + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_WBM_RING_ID(_n)         (0x00000e10 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_WBM_RING_MISC(_n)       (0x00000e18 + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_WBM_RING_HP_ADDR_LSB(_n) (0x00000e1c + (_n) * WCN7850_SRNG_STRIDE)
#define WCN7850_WBM_RING_MSI1_DATA(_n)   (0x00000e58 + (_n) * WCN7850_SRNG_STRIDE)
/* WBM2SW R2 (pointer) registers — stride 8 (HP at base, TP at base+4) */
#define WCN7850_WBM_RING_HP(_n)     (0x000030c8 + (_n) * 8)
#define WCN7850_WBM_RING_TP(_n)     (0x000030c8 + (_n) * 8 + 4)

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
/* These MUST match the driver's enum wmi_cmd_group */
#define WMI_GRP_SCAN       0x3
#define WMI_GRP_PDEV       0x4
#define WMI_GRP_VDEV       0x5
#define WMI_GRP_PEER       0x6
#define WMI_GRP_MGMT       0x7

#define WMI_TLV_CMD(grp_id) (((grp_id) << 12) | 0x1)
#define WMI_EVT_GRP_START_ID(grp_id) (((grp_id) << 12) | 0x1)

/* WMI command IDs */
#define WMI_SERVICE_READY_EVENTID  0x1
#define WMI_READY_EVENTID          0x2
#define WMI_INIT_CMDID             0x1
#define WMI_VDEV_CREATE_CMDID      WMI_TLV_CMD(WMI_GRP_VDEV)
#define WMI_PEER_CREATE_CMDID      WMI_TLV_CMD(WMI_GRP_PEER)

/* Additional WMI command/event IDs used by the STA connect path.
 * (event groups use WMI_EVT_GRP_START_ID(grp) = ((grp)<<12)|0x1) */
#define WMI_START_SCAN_CMDID       WMI_TLV_CMD(WMI_GRP_SCAN)        /* 0x1001 */
#define WMI_SCAN_CHAN_LIST_CMDID   (WMI_TLV_CMD(WMI_GRP_SCAN) + 2)  /* 0x1003 */
#define WMI_VDEV_START_REQUEST_CMDID (WMI_TLV_CMD(WMI_GRP_VDEV) + 2) /* 0x5003 */
#define WMI_VDEV_UP_CMDID          (WMI_TLV_CMD(WMI_GRP_VDEV) + 4)  /* 0x5005 */
#define WMI_PEER_ASSOC_CMDID       (WMI_TLV_CMD(WMI_GRP_PEER) + 4)  /* 0x6005 */
#define WMI_VDEV_INSTALL_KEY_CMDID (WMI_TLV_CMD(WMI_GRP_VDEV) + 0xd) /* 0x500e */

#define WMI_SCAN_EVENTID           WMI_EVT_GRP_START_ID(WMI_GRP_SCAN)    /* 0x1001 */
#define WMI_MGMT_RX_EVENTID        WMI_EVT_GRP_START_ID(WMI_GRP_MGMT)    /* 0x7001 */
#define WMI_MGMT_TX_SEND_CMDID     (WMI_TLV_CMD(WMI_GRP_MGMT) + 1)       /* 0x7002 */
#define WMI_PEER_ASSOC_CONF_EVENTID (WMI_TLV_CMD(WMI_GRP_PEER) + 5)      /* 0x6006 */
#define WMI_VDEV_INSTALL_KEY_COMPLETE_EVENTID (WMI_TLV_CMD(WMI_GRP_VDEV) + 2) /* 0x5003 */
#define WMI_MGMT_TX_COMPLETION_EVENTID (WMI_TLV_CMD(WMI_GRP_MGMT) + 5)   /* 0x7006 */
#define WMI_VDEV_START_RESP_EVENTID WMI_TLV_CMD(WMI_GRP_VDEV)            /* 0x5001 */

/* WMI scan event types (wmi_scan_event_type) */
#define WMI_SCAN_EVENT_STARTED      BIT(0)
#define WMI_SCAN_EVENT_COMPLETED    BIT(1)
#define WMI_SCAN_EVENT_BSS_CHANNEL  BIT(2)
#define WMI_SCAN_EVENT_FOREIGN_CHAN BIT(3)
#define WMI_SCAN_REASON_COMPLETED   0x1

/* WMI TLV tags used by the connect path */
#define WMI_TAG_START_SCAN_CMD      1301
#define WMI_TAG_SCAN_EVENT          36
#define WMI_TAG_MGMT_RX_HDR         44
#define WMI_TAG_ARRAY_BYTE          17
#define WMI_TAG_PEER_ASSOC_CONF_EVENT 1658
#define WMI_TAG_VDEV_INSTALL_KEY_COMPLETE_EVENT 1261
#define WMI_TAG_MGMT_TX_SEND_CMD    1265
#define WMI_TAG_MGMT_TX_COMPL_EVENT 1266

/* HTT T2H message types (subset the model emits) */
#define HTT_T2H_MSG_TYPE_PEER_MAP   0x0  /* PEER_MAP v1 */
#define HTT_T2H_MSG_TYPE_PEER_MAP2  0x1a
#define HTT_T2H_MSG_TYPE_PEER_MAP3  0x1f
#define HTT_T2H_MSG_TYPE_VERSION_CONF 0x0  /* version req/conf share msg_type 0 */

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
#define QRTR_TYPE_RESUME_TX 7
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

/* WMI service-ready EXT event (driver ath12k parses this to complete
 * ab->wmi_ab.service_ready). */
#define WMI_SERVICE_READY_EXT_EVENTID  0x4009
#define WMI_TAG_ARRAY_STRUCT           18
#define WMI_TAG_SERVICE_READY_EXT_EVENT   428
#define WMI_TAG_MAC_PHY_CAPABILITIES   528
#define WMI_TAG_HW_MODE_CAPABILITIES   529
#define WMI_TAG_SOC_MAC_PHY_HW_MODE_CAPS   530
#define WMI_TAG_HAL_REG_CAPABILITIES_EXT   531
#define WMI_TAG_SOC_HAL_REG_CAPABILITIES   532

/* WMI_SERVICE_AVAILABLE_EVENT: tells the driver about TLV services
 * (bits 128-255). Without this, ath12k_mac_setup_vdev_params_mbssid
 * believes the firmware lacks WMI_TLV_SERVICE_MBSS_PARAM_IN_VDEV_START_SUPPORT
 * (bit 253), enters the legacy code-path for a scan-link (link_id >= 15) and
 * returns -ENOLINK because ath12k_mac_get_link_bss_conf returns NULL. */
#define WMI_SERVICE_AVAILABLE_EVENTID       0x3
#define WMI_TAG_SERVICE_AVAILABLE_EVENT     559
#define WMI_TLV_SERVICE_MBSS_PARAM_IN_VDEV_START_SUPPORT  253
#define WMI_SERVICE_SEGMENT_BM_SIZE32       4
#define WMI_MAX_SERVICE                     128

/* Regulatory channel-list event (sent during init so the driver populates
 * ab->default_regd instead of falling back to the world domain). */
#define WMI_TAG_REG_CHAN_LIST_CC_EXT_EVENT  938
#define WMI_REG_CHAN_LIST_CC_EVENTID        0x3a003
#define WMI_REG_CLIENT_MAX                  4
#define WMI_REG_SET_CC_STATUS_PASS          0

#define WMI_HOST_WLAN_2GHZ_CAP  1
#define WMI_HOST_WLAN_5GHZ_CAP  2
#define WMI_HOST_HW_MODE_DBS    1

typedef struct {
    uint32_t numss_m1;
    uint32_t ru_info;
    uint32_t ppet16_ppet8_ru3_ru0[8];
} QEMU_PACKED WmiPpeThreshold;

typedef struct {
    uint32_t default_conc_scan_config_bits;
    uint32_t default_fw_config_bits;
    WmiPpeThreshold ppet;
    uint32_t he_cap_info;
    uint32_t mpdu_density;
    uint32_t max_bssid_rx_filters;
    uint32_t fw_build_vers_ext;
    uint32_t max_nlo_ssids;
    uint32_t max_bssid_indicator;
    uint32_t he_cap_info_ext;
} QEMU_PACKED WmiServiceReadyExt;

typedef struct {
    uint32_t num_hw_modes;
    uint32_t num_chainmask_tables;
} QEMU_PACKED WmiSocMacPhyHwModeCaps;

typedef struct {
    uint32_t tlv_header;
    uint32_t hw_mode_id;
    uint32_t phy_id_map;
    uint32_t hw_mode_config_type;
} QEMU_PACKED WmiHwModeCap;

typedef struct {
    uint32_t hw_mode_id;
    uint32_t pdev_and_hw_link_ids;
    uint32_t phy_id;
    uint32_t supported_flags;
    uint32_t supported_bands;
    uint32_t ampdu_density;
    uint32_t max_bw_supported_2g;
    uint32_t ht_cap_info_2g;
    uint32_t vht_cap_info_2g;
    uint32_t vht_supp_mcs_2g;
    uint32_t he_cap_info_2g;
    uint32_t he_supp_mcs_2g;
    uint32_t tx_chain_mask_2g;
    uint32_t rx_chain_mask_2g;
    uint32_t max_bw_supported_5g;
    uint32_t ht_cap_info_5g;
    uint32_t vht_cap_info_5g;
    uint32_t vht_supp_mcs_5g;
    uint32_t he_cap_info_5g;
    uint32_t he_supp_mcs_5g;
    uint32_t tx_chain_mask_5g;
    uint32_t rx_chain_mask_5g;
    uint32_t he_cap_phy_info_2g[3];
    uint32_t he_cap_phy_info_5g[3];
    WmiPpeThreshold he_ppet2g;
    WmiPpeThreshold he_ppet5g;
    uint32_t chainmask_table_id;
    uint32_t lmac_id;
    uint32_t he_cap_info_2g_ext;
    uint32_t he_cap_info_5g_ext;
    uint32_t he_cap_info_internal;
    uint32_t wireless_modes;
    uint32_t low_2ghz_chan_freq;
    uint32_t high_2ghz_chan_freq;
    uint32_t low_5ghz_chan_freq;
    uint32_t high_5ghz_chan_freq;
    uint32_t nss_ratio;
} QEMU_PACKED WmiMacPhyCaps;

typedef struct {
    uint32_t num_phy;
} QEMU_PACKED WmiSocHalRegCaps;

typedef struct {
    uint32_t tlv_header;
    uint32_t phy_id;
    uint32_t eeprom_reg_domain;
    uint32_t eeprom_reg_domain_ext;
    uint32_t regcap1;
    uint32_t regcap2;
    uint32_t wireless_modes;
    uint32_t low_2ghz_chan;
    uint32_t high_2ghz_chan;
    uint32_t low_5ghz_chan;
    uint32_t high_5ghz_chan;
} QEMU_PACKED WmiHalRegCapsExt;

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

    /* HTC ready sent: set after we successfully send HTC ready to driver */
    bool htc_ready_sent;

    /* WMI_SERVICE_READY pending: set when WMI service connects, sent once
     * the driver has posted rx buffers on the WMI downlink CE (pipe 2). */
    bool wmi_service_ready_pending;

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

    /* ===== Simulated AP + station connect state ===== */
    /* Simulated AP BSS (open network by default). */
    uint8_t ap_bssid[6];
    uint8_t ap_ssid[33];
    uint8_t ap_ssid_len;
    uint32_t ap_channel;      /* 802.11 channel number */
    uint32_t ap_freq;         /* MHz */

    /* VDEV / peer assigned by the driver during connect. */
    uint32_t vdev_id;
    uint8_t peer_mac[6];      /* the AP peer MAC (== ap_bssid) */
    uint16_t peer_id;         /* firmware-assigned peer id */
    bool peer_mapped;         /* HTT PEER_MAP v1 emitted */

    /* scan request details (so we can emit the matching scan events) */
    uint32_t scan_id;
    uint32_t scan_vdev_id;

    /* data-path offload: synthetic RX frame generation toggle */
    bool data_offload_enabled;
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
static void wcn7850_handle_peer_create(WCN7850State *s, PCIDevice *pci_dev,
                                       const uint8_t *payload, uint32_t len);
static void wcn7850_handle_peer_assoc(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len);
static void wcn7850_handle_scan_start(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len);
static void wcn7850_handle_mgmt_tx(WCN7850State *s, PCIDevice *pci_dev,
                                   const uint8_t *payload, uint32_t len);
static void wcn7850_handle_install_key(WCN7850State *s, PCIDevice *pci_dev,
                                       const uint8_t *payload, uint32_t len);

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

static void wcn7850_update_wbm_release_ring_cfg(WCN7850State *s, int n)
{
    uint32_t base_lsb, base_msb, ring_id_reg, misc;
    WCN7850RingState *r = wcn7850_add_or_update_ring(s, WCN7850_RING_WBM2SW_RELEASE, n);
    if (!r) return;

    base_lsb = *(uint32_t *)(s->bar0_always_on + WCN7850_WBM_RING_BASE_LSB(n));
    base_msb = *(uint32_t *)(s->bar0_always_on + WCN7850_WBM_RING_BASE_MSB(n));
    ring_id_reg = *(uint32_t *)(s->bar0_always_on + WCN7850_WBM_RING_ID(n));
    misc = *(uint32_t *)(s->bar0_always_on + WCN7850_WBM_RING_MISC(n));

    r->base_addr = ((uint64_t)(base_msb & 0xff) << 32) | base_lsb;
    r->size = (base_msb >> 8) & 0xffff;
    r->entry_size = (ring_id_reg & 0xff) * 4;
    r->hp_mmio_offset = WCN7850_WBM_RING_HP(n);
    r->tp_mmio_offset = WCN7850_WBM_RING_TP(n);
    r->hp_is_mmio = true;
    r->tp_is_mmio = true;
    r->enable = (misc & BIT(0)) != 0;
    r->configured = (r->base_addr != 0 && r->entry_size > 0 && r->size > 0);

    if (r->configured) {
        uint32_t msi_data = *(uint32_t *)(s->bar0_always_on + WCN7850_WBM_RING_MSI1_DATA(n));
        r->msivec = msi_data & 0xff;
        if (r->msivec >= WCN7850_MSI_VECTORS)
            r->msivec = 9;
        fprintf(stderr, "WCN: WBM2SW ring %d base=0x%lx size=%u entry=%u enable=%d msivec=%u\n",
                n, (unsigned long)r->base_addr, r->size, r->entry_size, r->enable, r->msivec);
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
        uint32_t tcl_info1;

        if (pci_dma_read(pci_dev, desc_addr, desc, sizeof(desc)) != MEMTX_OK) {
            break;
        }

        buf_addr = le64_to_cpu(*(uint64_t *)desc);
        buf_len = le32_to_cpu(*(uint32_t *)(desc + 16)) & 0xffff;
        tcl_info1 = le32_to_cpu(*(uint32_t *)(desc + 4));

        if (!buf_addr || !buf_len) {
            /* Empty descriptor (driver has retired it); skip. */
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

        /* Post TX completion to the WBM2SW release ring */
        {
            static const int tcl_to_wbm[] = { 0, 2, 4 };
            int wbm_idx = (tcl->ring_id < 3) ? tcl_to_wbm[tcl->ring_id] : 0;
            WCN7850RingState *wbm = wcn7850_find_ring(s,
                WCN7850_RING_WBM2SW_RELEASE, wbm_idx);
            if (wbm && wbm->configured && wbm->enable && wbm->entry_size >= 32) {
                uint32_t wbm_hp = wbm->hp;
                uint32_t wbm_tp = wbm->tp;
                uint32_t wbm_sz = wbm->size ?: (512 * wbm->entry_size);
                uint32_t next_hp = wbm_hp + wbm->entry_size;
                if (next_hp >= wbm_sz) next_hp = 0;

                if (next_hp != wbm_tp) {
                    uint8_t comp[32] = {0};
                    uint32_t *comp32 = (uint32_t *)comp;
                    /* buf_addr_info.info0 = 0 (not used for SW cookie path) */
                    comp32[0] = 0;
                    /* buf_addr_info.info1 = captured TCL info1 (SW cookie in bits 31:12) */
                    comp32[1] = cpu_to_le32(tcl_info1);
                    /* info0: REL_SRC_MODULE=TQM(0), DESC_TYPE=REL_MSDU(0),
                     * TQM_RELEASE_REASON=FRAME_ACKED(0), CC_DONE=0 (SW path) */
                    comp32[2] = 0;
                    /* info1: TRANSMIT_COUNT = 1 */
                    comp32[3] = cpu_to_le32(1u << 24);
                    /* info2: FIRST_MSDU | LAST_MSDU */
                    comp32[4] = cpu_to_le32((1u << 8) | (1u << 9));
                    /* info3..5 = 0 (maps to rate_stats.info0, tsf, info3 in
                     * hal_wbm_completion_ring_tx) */
                    comp32[5] = 0;
                    comp32[6] = 0;
                    comp32[7] = 0;

                    if (pci_dma_write(pci_dev, wbm->base_addr + wbm_hp,
                                      comp, sizeof(comp)) == MEMTX_OK) {
                        wbm->hp = next_hp;
                        *(uint32_t *)(s->bar0_always_on + wbm->hp_mmio_offset) = next_hp;
                        if (wbm->msivec < WCN7850_MSI_VECTORS) {
                            msi_notify(pci_dev, wbm->msivec);
                        }
                    }
                }
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
/* Map a CE pipe to its MSI vector index, matching the ath12k driver's
 * ath12k_pci_get_ce_msi_idx: count of CE pipes without CE_ATTR_DIS_INTR
 * that come before this pipe. WCN7850 config: CE4/6/7/8 have DIS_INTR. */
static int wcn7850_ce_msi_vector(int ce_pipe)
{
    static const int dis_intr[] = { 4, 6, 7, 8 };
    int idx = 0, i, d;
    for (i = 0; i < ce_pipe && i < WCN7850_CE_COUNT; i++) {
        int skip = 0;
        for (d = 0; d < (int)ARRAY_SIZE(dis_intr); d++)
            if (dis_intr[d] == i) { skip = 1; break; }
        if (!skip)
            idx++;
    }
    return WCN7850_CE_MSI_BASE + idx;
}

static void wcn7850_wmi_send_event(WCN7850State *s, PCIDevice *pci_dev,
                                  int ce_pipe, const void *data,
                                  uint32_t data_len)
{
    uint64_t base = s->ce_dst_base[ce_pipe];
    uint32_t esize = s->ce_dst_entry_size[ce_pipe] ?: 16;
    uint32_t ring_sz = s->ce_dst_size[ce_pipe] ?: (WCN7850_CE_DST_RING_SIZE * esize);
    if (!base || !ring_sz) {
        return;
    }

    /* We don't reliably track the driver's posted-buffer count (the driver
     * assigns CE shadow indices dynamically), so instead of gating on a
     * pending counter we read the descriptor at the current consumer slot:
     * if the driver hasn't posted a buffer there yet (buf_addr == 0) we
     * simply return and the caller retries later. */

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
    if (s->ce_dst_pending[ce_pipe])
        s->ce_dst_pending[ce_pipe]--;

    /* Write CE dst status descriptor and advance STATUS HP */
    if (s->ce_sts_base[ce_pipe]) {
        uint32_t sts_esize = s->ce_sts_entry_size[ce_pipe] ?: 16;
        uint32_t sts_hp = s->ce_sts_hp[ce_pipe];
        uint8_t sts_desc[16] = {0};
        *(uint32_t *)sts_desc = cpu_to_le32(data_len << 16);
        fprintf(stderr, "WCN: write sts_desc pipe=%d at 0x%lx len=%u hp=%u\n",
                 ce_pipe, (unsigned long)(s->ce_sts_base[ce_pipe] + sts_hp),
                 data_len, sts_hp);
        pci_dma_write(pci_dev, s->ce_sts_base[ce_pipe] + sts_hp,
                      sts_desc, sts_esize);
        {
            uint32_t w0, w1, w2, w3;
            pci_dma_read(pci_dev, s->ce_sts_base[ce_pipe] + sts_hp, &w0, 4);
            pci_dma_read(pci_dev, s->ce_sts_base[ce_pipe] + sts_hp + 4, &w1, 4);
            pci_dma_read(pci_dev, s->ce_sts_base[ce_pipe] + sts_hp + 8, &w2, 4);
            pci_dma_read(pci_dev, s->ce_sts_base[ce_pipe] + sts_hp + 12, &w3, 4);
            fprintf(stderr, "WCN: sts_desc written: 0x%08x 0x%08x 0x%08x 0x%08x\n",
                    w0, w1, w2, w3);
        }
        sts_hp += sts_esize;
        if (s->ce_sts_size[ce_pipe] &&
            sts_hp >= s->ce_sts_size[ce_pipe])
            sts_hp = 0;
        s->ce_sts_hp[ce_pipe] = sts_hp;

        /* RDP write-back: driver reads status HP from the RDP slot, but the
         * ath12k HAL treats cached_hp/tp/entry_size/ring_size in units of
         * 4-byte WORDS (see hal.c:596 ath12k_hal_srng_setup).  sts_hp is kept
         * in bytes for the ring offset, so convert to words here. */
        if (s->ce_sts_hp_addr[ce_pipe]) {
            uint32_t sts_hp_words = (sts_hp / sts_esize) * (sts_esize / 4);
            uint32_t sts_hp_le = cpu_to_le32(sts_hp_words);
            fprintf(stderr, "WCN: write sts_hp pipe=%d to 0x%lx val=%u (bytes %u)\n",
                     ce_pipe, (unsigned long)s->ce_sts_hp_addr[ce_pipe],
                     sts_hp_words, sts_hp);
            pci_dma_write(pci_dev, s->ce_sts_hp_addr[ce_pipe],
                          &sts_hp_le, sizeof(sts_hp_le));
            {
                uint32_t slot, sd[8];
                int i;
                pci_dma_read(pci_dev, s->ce_sts_hp_addr[ce_pipe], &slot, 4);
                fprintf(stderr, "WCN: rdp slot now=0x%x (ce_sts_hp_words=%u, bytes=%u)\n",
                        slot, sts_hp_words, sts_hp);
                for (i = 0; i < 8; i++) {
                    pci_dma_read(pci_dev, s->ce_sts_base[ce_pipe] + i * 16,
                                 &sd[i], 4);
                }
                fprintf(stderr, "WCN: CE%d status ring[0..7]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                        ce_pipe, sd[0], sd[1], sd[2], sd[3], sd[4], sd[5], sd[6], sd[7]);
            }
            if (ce_pipe == 1) {
                uint32_t s2slot, s2d[2];
                pci_dma_read(pci_dev, s->ce_sts_hp_addr[2], &s2slot, 4);
                pci_dma_read(pci_dev, s->ce_sts_base[2], &s2d[0], 4);
                pci_dma_read(pci_dev, s->ce_sts_base[2] + 16, &s2d[1], 4);
                fprintf(stderr, "WCN: CE2 sts_hp slot=0x%x base=0x%lx entry0=0x%08x entry1=0x%08x\n",
                        s2slot, (unsigned long)s->ce_sts_base[2], s2d[0], s2d[1]);
            }
        }
    }

    /* Send MSI to notify the driver (vector derived from the CE pipe,
     * matching the ath12k driver's ath12k_pci_get_ce_msi_idx mapping) */
    {
        int msivec = wcn7850_ce_msi_vector(ce_pipe);
        fprintf(stderr, "WCN: sending MSI %d for CE pipe %d\n", msivec, ce_pipe);
        msi_notify(pci_dev, msivec);
    }
}

/* Build and send a WMI_REG_CHAN_LIST_CC_EVENT so the driver populates
 * ab->default_regd during init (instead of falling back to the world
 * regulatory domain). Mirrors the firmware's regulatory channel-list EXT
 * event: a top-level TLV carrying the event struct, followed by a flat array
 * of ext reg-rule params (2 GHz + 5 GHz only; 6 GHz counts are left 0). */
static void wcn7850_send_wmi_reg_chan_list_cc(WCN7850State *s, PCIDevice *pci_dev)
{
    struct QEMU_PACKED wmi_reg_rule_ext_params {
        uint32_t tlv_header;
        uint32_t freq_info;
        uint32_t bw_pwr_info;
        uint32_t flag_info;
        uint32_t psd_power_info;
    };

    struct QEMU_PACKED wmi_reg_chan_list_cc_ext_event {
        uint32_t status_code;
        uint32_t phy_id;
        uint32_t alpha2;
        uint32_t num_phy;
        uint32_t country_id;
        uint32_t domain_code;
        uint32_t dfs_region;
        uint32_t phybitmap;
        uint32_t min_bw_2g;
        uint32_t max_bw_2g;
        uint32_t min_bw_5g;
        uint32_t max_bw_5g;
        uint32_t num_2g_reg_rules;
        uint32_t num_5g_reg_rules;
        uint32_t client_type;
        uint32_t rnr_tpe_usable;
        uint32_t unspecified_ap_usable;
        uint32_t domain_code_6g_ap_lpi;
        uint32_t domain_code_6g_ap_sp;
        uint32_t domain_code_6g_ap_vlp;
        uint32_t domain_code_6g_client_lpi[WMI_REG_CLIENT_MAX];
        uint32_t domain_code_6g_client_sp[WMI_REG_CLIENT_MAX];
        uint32_t domain_code_6g_client_vlp[WMI_REG_CLIENT_MAX];
        uint32_t domain_code_6g_super_id;
        uint32_t min_bw_6g_ap_sp;
        uint32_t max_bw_6g_ap_sp;
        uint32_t min_bw_6g_ap_lpi;
        uint32_t max_bw_6g_ap_lpi;
        uint32_t min_bw_6g_ap_vlp;
        uint32_t max_bw_6g_ap_vlp;
        uint32_t min_bw_6g_client_sp[WMI_REG_CLIENT_MAX];
        uint32_t max_bw_6g_client_sp[WMI_REG_CLIENT_MAX];
        uint32_t min_bw_6g_client_lpi[WMI_REG_CLIENT_MAX];
        uint32_t max_bw_6g_client_lpi[WMI_REG_CLIENT_MAX];
        uint32_t min_bw_6g_client_vlp[WMI_REG_CLIENT_MAX];
        uint32_t max_bw_6g_client_vlp[WMI_REG_CLIENT_MAX];
        uint32_t num_6g_reg_rules_ap_sp;
        uint32_t num_6g_reg_rules_ap_lpi;
        uint32_t num_6g_reg_rules_ap_vlp;
        uint32_t num_6g_reg_rules_cl_sp[WMI_REG_CLIENT_MAX];
        uint32_t num_6g_reg_rules_cl_lpi[WMI_REG_CLIENT_MAX];
        uint32_t num_6g_reg_rules_cl_vlp[WMI_REG_CLIENT_MAX];
    };

    /* 2 GHz + 5 GHz rules (start/end MHz, max bandwidth MHz, reg power dBm). */
    static const struct { uint32_t start, end, max_bw, reg_pwr; } rules_2g[] = {
        { 2412, 2484, 40, 20 },
    };
    static const struct { uint32_t start, end, max_bw, reg_pwr; } rules_5g[] = {
        { 5180, 5320, 80, 23 },
        { 5500, 5720, 160, 23 },
        { 5745, 5895, 80, 23 },
    };
    const uint32_t n2 = sizeof(rules_2g) / sizeof(rules_2g[0]);
    const uint32_t n5 = sizeof(rules_5g) / sizeof(rules_5g[0]);

    uint8_t buf[1024];
    uint32_t off = 0;
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    WmiTlv *tlv;
    uint32_t tlv_start;

    hdr->cmd_id = cpu_to_le32(WMI_REG_CHAN_LIST_CC_EVENTID);
    off += sizeof(*hdr);

#define OPEN_TLV(_tag) do { \
        tlv_start = off; \
        tlv = (WmiTlv *)(buf + off); \
        off += sizeof(*tlv); \
    } while (0)
#define CLOSE_TLV(_tag) do { \
        tlv->header = WMI_TLV_HDR((_tag), off - (tlv_start + sizeof(*tlv))); \
    } while (0)

    OPEN_TLV(WMI_TAG_REG_CHAN_LIST_CC_EXT_EVENT);
    struct wmi_reg_chan_list_cc_ext_event *ev =
        (struct wmi_reg_chan_list_cc_ext_event *)(buf + off);
    memset(ev, 0, sizeof(*ev));
    ev->status_code = cpu_to_le32(WMI_REG_SET_CC_STATUS_PASS);
    ev->phy_id = cpu_to_le32(0);
    ev->alpha2 = cpu_to_le32(0x3030); /* "00" world regulatory domain */
    ev->num_phy = cpu_to_le32(1);
    ev->phybitmap = cpu_to_le32(1);
    ev->min_bw_2g = cpu_to_le32(20);
    ev->max_bw_2g = cpu_to_le32(40);
    ev->min_bw_5g = cpu_to_le32(20);
    ev->max_bw_5g = cpu_to_le32(160);
    ev->num_2g_reg_rules = cpu_to_le32(n2);
    ev->num_5g_reg_rules = cpu_to_le32(n5);
    off += sizeof(*ev);
    CLOSE_TLV(WMI_TAG_REG_CHAN_LIST_CC_EXT_EVENT);

    /* The driver skips an (ignored) wmi_tlv header between the event struct
     * and the flat reg-rule array. */
    tlv = (WmiTlv *)(buf + off);
    tlv->header = WMI_TLV_HDR(0, (n2 + n5) * sizeof(struct wmi_reg_rule_ext_params));
    off += sizeof(*tlv);

    for (uint32_t i = 0; i < n2; i++) {
        struct wmi_reg_rule_ext_params *r = (struct wmi_reg_rule_ext_params *)(buf + off);
        r->tlv_header = 0;
        r->freq_info = cpu_to_le32((rules_2g[i].start & 0xffff) |
                                   ((rules_2g[i].end & 0xffff) << 16));
        r->bw_pwr_info = cpu_to_le32((rules_2g[i].max_bw & 0xffff) |
                                     ((rules_2g[i].reg_pwr & 0xff) << 16));
        r->flag_info = 0;
        r->psd_power_info = 0;
        off += sizeof(*r);
    }
    for (uint32_t i = 0; i < n5; i++) {
        struct wmi_reg_rule_ext_params *r = (struct wmi_reg_rule_ext_params *)(buf + off);
        r->tlv_header = 0;
        r->freq_info = cpu_to_le32((rules_5g[i].start & 0xffff) |
                                   ((rules_5g[i].end & 0xffff) << 16));
        r->bw_pwr_info = cpu_to_le32((rules_5g[i].max_bw & 0xffff) |
                                     ((rules_5g[i].reg_pwr & 0xff) << 16));
        r->flag_info = 0;
        r->psd_power_info = 0;
        off += sizeof(*r);
    }

#undef OPEN_TLV
#undef CLOSE_TLV

    uint32_t total_len = off;
    uint8_t htc_buf[2048];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total_len);
    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*h) + total_len);
}

/* Build and send a WMI_SERVICE_READY_EVENT to the driver */
static void wcn7850_send_wmi_service_ready_ext(WCN7850State *s, PCIDevice *pci_dev);

/* Send WMI_SERVICE_AVAILABLE_EVENT (0x3) so the driver populates
 * svc_map for TLV services 128-255 (including
 * WMI_TLV_SERVICE_MBSS_PARAM_IN_VDEV_START_SUPPORT = 253).
 * Without this, scan links get -ENOLINK. */
static void wcn7850_send_wmi_service_available(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[128];
    uint32_t off = 0;

    struct QEMU_PACKED wmi_service_available_event {
        uint32_t wmi_service_segment_offset;
        uint32_t wmi_service_segment_bitmap[WMI_SERVICE_SEGMENT_BM_SIZE32];
    };

    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    hdr->cmd_id = cpu_to_le32(WMI_SERVICE_AVAILABLE_EVENTID);
    off += sizeof(*hdr);

    WmiTlv *tlv = (WmiTlv *)(buf + off);
    tlv->header = WMI_TLV_HDR(WMI_TAG_SERVICE_AVAILABLE_EVENT,
                              sizeof(struct wmi_service_available_event));
    off += sizeof(*tlv);

    struct wmi_service_available_event *ev = (struct wmi_service_available_event *)(buf + off);
    memset(ev, 0, sizeof(*ev));
    ev->wmi_service_segment_offset = cpu_to_le32(WMI_MAX_SERVICE);
    /* Set bit for service 253 (WMI_TLV_SERVICE_MBSS_PARAM_IN_VDEV_START_SUPPORT).
     * Service 253 maps to segment bitmap index 3, bit 29
     * (offset = 253 - WMI_MAX_SERVICE = 125, word = 125/32 = 3, bit = 125%32 = 29). */
    ev->wmi_service_segment_bitmap[3] = cpu_to_le32(BIT(29));
    off += sizeof(*ev);

    uint32_t total_len = off;

    uint8_t htc_buf[256];
    HtcHdr *hdr2 = (HtcHdr *)htc_buf;
    hdr2->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    hdr2->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*hdr2), buf, total_len);

    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*hdr2) + total_len);
    fprintf(stderr, "WCN: sent WMI_SERVICE_AVAILABLE (0x3) for bit 253 cons=%u\n",
            s->ce_dst_cons[2]);
}

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

    /* TLV: service bitmap (WMI_TAG_ARRAY_UINT32, 128 bytes = 32×uint32_t for WMI_MAX_SERVICE=128) */
    uint32_t bm_words = 32; /* WMI_SERVICE_BM_SIZE = (128+3)/4 */
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

    wcn7850_wmi_send_event(s, pci_dev, 2, /* WMI DL pipe */
                            htc_buf, sizeof(*hdr2) + total_len);

    wcn7850_send_wmi_service_ready_ext(s, pci_dev);
    wcn7850_send_wmi_reg_chan_list_cc(s, pci_dev);
    wcn7850_send_wmi_service_available(s, pci_dev);
    s->wmi_service_ready_sent = true;
}

/* Build and send a WMI_SERVICE_READY_EXT_EVENT so the driver completes
 * ab->wmi_ab.service_ready. Mirrors the firmware capability payload. */
static void wcn7850_send_wmi_service_ready_ext(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[1024];
    uint32_t off = 0;
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    WmiTlv *tlv;
    uint8_t *p;

    hdr->cmd_id = cpu_to_le32(WMI_SERVICE_READY_EXT_EVENTID);
    off += sizeof(*hdr);

    /* Helper to open a top-level TLV whose length is filled in once the
     * content has been written (off-delta), so lengths always match bytes. */
#define OPEN_TLV(_tag) do { \
        tlv_start = off; \
        tlv = (WmiTlv *)(buf + off); \
        off += sizeof(*tlv); \
    } while (0)
#define CLOSE_TLV(_tag) do { \
        tlv->header = WMI_TLV_HDR((_tag), off - (tlv_start + sizeof(*tlv))); \
    } while (0)

    /* 1) main ext struct */
    uint32_t tlv_start;
    OPEN_TLV(WMI_TAG_SERVICE_READY_EXT_EVENT);
    memset(buf + off, 0, sizeof(WmiServiceReadyExt));
    off += sizeof(WmiServiceReadyExt);
    CLOSE_TLV(WMI_TAG_SERVICE_READY_EXT_EVENT);

    /* 2) SOC_MAC_PHY_HW_MODE_CAPS */
    OPEN_TLV(WMI_TAG_SOC_MAC_PHY_HW_MODE_CAPS);
    {
        WmiSocMacPhyHwModeCaps *c = (WmiSocMacPhyHwModeCaps *)(buf + off);
        c->num_hw_modes = cpu_to_le32(1);
        c->num_chainmask_tables = cpu_to_le32(1);
    }
    off += sizeof(WmiSocMacPhyHwModeCaps);
    CLOSE_TLV(WMI_TAG_SOC_MAC_PHY_HW_MODE_CAPS);

    /* 3) ARRAY_STRUCT: hw mode caps (1 entry). Content is the nested
     * WmiHwModeCap TLV (which carries its own tlv_header). */
    OPEN_TLV(WMI_TAG_ARRAY_STRUCT);
    {
        WmiHwModeCap *c = (WmiHwModeCap *)(buf + off);
        c->tlv_header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_HW_MODE_CAPABILITIES,
                                  sizeof(WmiHwModeCap) - sizeof(uint32_t)));
        c->hw_mode_id = cpu_to_le32(WMI_HOST_HW_MODE_DBS);
        c->phy_id_map = cpu_to_le32(0x1);
        c->hw_mode_config_type = 0;
    }
    off += sizeof(WmiHwModeCap);
    CLOSE_TLV(WMI_TAG_ARRAY_STRUCT);

    /* 4) ARRAY_STRUCT: mac phy caps (1 entry). Content is the nested
     * MAC_PHY_CAPABILITIES TLV (header + WmiMacPhyCaps). */
    OPEN_TLV(WMI_TAG_ARRAY_STRUCT);
    p = buf + off;
    *(uint32_t *)p = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_MAC_PHY_CAPABILITIES,
                                  sizeof(WmiMacPhyCaps)));
    p += sizeof(uint32_t);
    off += sizeof(uint32_t);   /* account for the separate nested header */
    {
        WmiMacPhyCaps *c = (WmiMacPhyCaps *)p;
        memset(c, 0, sizeof(*c));
        c->hw_mode_id = cpu_to_le32(WMI_HOST_HW_MODE_DBS);
        c->pdev_and_hw_link_ids = 0;
        c->phy_id = cpu_to_le32(0);
        c->supported_bands = cpu_to_le32(WMI_HOST_WLAN_2GHZ_CAP |
                                         WMI_HOST_WLAN_5GHZ_CAP);
        c->wireless_modes = cpu_to_le32(0x1f);
        c->low_2ghz_chan_freq = cpu_to_le32(2412);
        c->high_2ghz_chan_freq = cpu_to_le32(2484);
        c->low_5ghz_chan_freq = cpu_to_le32(5180);
        c->high_5ghz_chan_freq = cpu_to_le32(5885);
    }
    off += sizeof(WmiMacPhyCaps);
    CLOSE_TLV(WMI_TAG_ARRAY_STRUCT);
    /* 5) SOC_HAL_REG_CAPABILITIES */
    OPEN_TLV(WMI_TAG_SOC_HAL_REG_CAPABILITIES);
    {
        WmiSocHalRegCaps *c = (WmiSocHalRegCaps *)(buf + off);
        c->num_phy = cpu_to_le32(1);
    }
    off += sizeof(WmiSocHalRegCaps);
    CLOSE_TLV(WMI_TAG_SOC_HAL_REG_CAPABILITIES);

    /* 6) ARRAY_STRUCT: ext hal reg caps (1 entry). Content is the nested
     * WmiHalRegCapsExt TLV (carries its own tlv_header). */
    OPEN_TLV(WMI_TAG_ARRAY_STRUCT);
    {
        WmiHalRegCapsExt *c = (WmiHalRegCapsExt *)(buf + off);
        c->tlv_header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_HAL_REG_CAPABILITIES_EXT,
                                  sizeof(WmiHalRegCapsExt) - sizeof(uint32_t)));
        c->phy_id = cpu_to_le32(0);
        c->low_2ghz_chan = cpu_to_le32(2412);
        c->high_2ghz_chan = cpu_to_le32(2484);
        c->low_5ghz_chan = cpu_to_le32(5180);
        c->high_5ghz_chan = cpu_to_le32(5885);
    }
    off += sizeof(WmiHalRegCapsExt);
    CLOSE_TLV(WMI_TAG_ARRAY_STRUCT);

#undef OPEN_TLV
#undef CLOSE_TLV

    uint32_t total_len = off;

    uint8_t htc_buf[2048];
    HtcHdr *h2 = (HtcHdr *)htc_buf;
    h2->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    h2->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h2), buf, total_len);

    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*h2) + total_len);
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
    /* mac_addr at offset 24..31 (ETH_ALEN + 2 padding): set device MAC */
    {
        const uint8_t dev_mac[6] = { 0x00, 0x14, 0x6c, 0x9a, 0x00, 0x02 };
        memcpy(buf + off + 24, dev_mac, 6);
    }
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

    fprintf(stderr, "WCN: calling wcn7850_wmi_send_event for WMI ready len=%u cons=%u\n",
            sizeof(*h2) + total_len, s->ce_dst_cons[2]);
    wcn7850_wmi_send_event(s, pci_dev, 2, /* WMI DL pipe */
                            htc_buf, sizeof(*h2) + total_len);
    s->wmi_ready_sent = true;
    fprintf(stderr, "WCN: wmi_ready_sent=true, cons now=%u\n", s->ce_dst_cons[2]);
}

/* Initialize HTC EP0 after getting HTC_SETUP_COMPLETE */
static void wcn7850_htc_send_ready(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[64];
    HtcHdr *hdr = (HtcHdr *)buf;
    hdr->hdr_info = cpu_to_le32((sizeof(uint32_t) * 2) << 16); /* EP 0, flags 0, payload_len */
    hdr->ctrl_info = cpu_to_le32(HTC_MSG_READY_ID);

    /* Kernel struct ath12k_htc_ready:
     *   id_credit_count: msg_id [15:0], credit_count [31:16]
     *   size_ep:         credit_size [15:0], (ep info) [31:16] */
    uint32_t *buf32 = (uint32_t *)(buf + sizeof(HtcHdr));
    buf32[0] = cpu_to_le32(HTC_MSG_READY_ID | (128 << 16));  /* id_credit_count */
    buf32[1] = cpu_to_le32(4096);                             /* size_ep */

    /* Send via CE dest ring (CE pipe 1 = HTC control DL) */
    wcn7850_wmi_send_event(s, pci_dev, 1, buf, sizeof(HtcHdr) + sizeof(uint32_t) * 2);
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
    HtcHdr *hdr = (HtcHdr *)resp_buf;
    memset(hdr, 0, sizeof(*hdr));
    hdr->hdr_info = cpu_to_le32(sizeof(HtcConnSvcResp) << 16);
    hdr->ctrl_info = cpu_to_le32(HTC_MSG_CONNECT_SERVICE_RESP_ID);

    HtcConnSvcResp *resp = (HtcConnSvcResp *)(resp_buf + sizeof(HtcHdr));

    (void)msg_id;

    int ep = -1;
    int ul_pipe = 3, dl_pipe = 2;
    switch (svc_id) {
    case 0x0300: /* HTT data (host->target): SVC(HTT,0) */
        ep = 1; ul_pipe = 4; dl_pipe = 1; break;
    case 0x0100: /* WMI control: SVC(WMI,0) */
        ep = HTC_EP_WMI; ul_pipe = 3; dl_pipe = 2; break;
    case 0x0105: /* WMI control MAC1: SVC(WMI,5) */
        ep = 3; ul_pipe = 3; dl_pipe = 2; break;
    case 0x0106: /* WMI control MAC2: SVC(WMI,6) */
        ep = 4; ul_pipe = 3; dl_pipe = 2; break;
    default:
        fprintf(stderr, "WCN: HTC connect unknown svc_id=0x%04x\n", svc_id);
        return;
    }

    if (ep < 0 || ep >= HTC_EP_MAX)
        return;

    /* Assign endpoint and record mapping */
    s->htc_ep[ep].connected = true;
    s->htc_ep[ep].service_id = svc_id;
    s->htc_ep[ep].ul_pipe = ul_pipe;
    s->htc_ep[ep].dl_pipe = dl_pipe;

    uint32_t msg_svc_id = (svc_id << 16) | HTC_MSG_CONNECT_SERVICE_RESP_ID;
    resp->msg_svc_id = cpu_to_le32(msg_svc_id);
    resp->flags_len = cpu_to_le32(0 | (ep << 8) | (4096 << 16));

    fprintf(stderr, "WCN: HTC connect svc=0x%04x -> ep=%d ul=%d dl=%d\n",
            svc_id, ep, ul_pipe, dl_pipe);

    /* Send response via CE pipe 1 (HTC control DL) */
    wcn7850_wmi_send_event(s, pci_dev, 1, resp_buf, sizeof(HtcHdr) + sizeof(*resp));
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
    case WMI_VDEV_CREATE_CMDID:
        qemu_log("WCN: WMI VDEV CREATE cmd\n");
        /* Fire-and-forget: no response expected by the driver. */
        break;
    case WMI_VDEV_UP_CMDID:
        qemu_log("WCN: WMI VDEV UP cmd\n");
        /* Fire-and-forget. */
        break;
    case WMI_PEER_CREATE_CMDID:
        wcn7850_handle_peer_create(s, pci_dev, payload, len);
        break;
    case WMI_PEER_ASSOC_CMDID:
        wcn7850_handle_peer_assoc(s, pci_dev, payload, len);
        break;
    case WMI_START_SCAN_CMDID:
        wcn7850_handle_scan_start(s, pci_dev, payload, len);
        break;
    case WMI_SCAN_CHAN_LIST_CMDID:
        qemu_log("WCN: WMI SCAN CHAN LIST cmd (ignored)\n");
        break;
    case WMI_MGMT_TX_SEND_CMDID:
        wcn7850_handle_mgmt_tx(s, pci_dev, payload, len);
        break;
    case WMI_VDEV_INSTALL_KEY_CMDID:
        wcn7850_handle_install_key(s, pci_dev, payload, len);
        break;
    default:
        qemu_log("WCN: WMI cmd 0x%x len=%u (unhandled)\n", cmd_id, len);
        break;
    }
}

/* Emit an HTT T2H PEER_MAP v1 message on the HTT/WMI DL CE pipe (pipe 1,
 * EPID 1) so the driver's ath12k_wait_for_peer_created() completes.
 * WCN7850 uses peer_map_unmap_version = 0x1 (PEER_MAP v1). */
static void wcn7850_send_htt_peer_map(WCN7850State *s, PCIDevice *pci_dev,
                                      uint32_t vdev_id, uint16_t peer_id,
                                      const uint8_t *mac)
{
    uint8_t buf[32];
    HtcHdr *h = (HtcHdr *)buf;
    /* HTT T2H PEER_MAP v1 fixed layout (host byte order fields):
     *   msg_type (1B) | vdev_id (1B) | peer_id (2B) | mac (6B) | ast_hash (4B) */
    uint8_t *p = buf + sizeof(*h);
    p[0] = HTT_T2H_MSG_TYPE_PEER_MAP;          /* msg_type */
    p[1] = (uint8_t)vdev_id;                    /* vdev_id */
    p[2] = peer_id & 0xff;                       /* peer_id low */
    p[3] = (peer_id >> 8) & 0xff;                /* peer_id high */
    memcpy(p + 4, mac, 6);                       /* peer MAC */
    /* ast_hash / next_hop (4 bytes) — set to peer_id for simplicity */
    p[10] = peer_id & 0xff; p[11] = (peer_id >> 8) & 0xff;
    p[12] = 0; p[13] = 0;
    uint32_t total = sizeof(*h) + 14;

    h->hdr_info = cpu_to_le32((total - sizeof(*h)) << 16 | 1); /* EPID 1 */
    h->ctrl_info = 0;
    wcn7850_wmi_send_event(s, pci_dev, 1, buf, total);
    fprintf(stderr, "WCN: HTT PEER_MAP v1 vdev=%u peer_id=%u mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
            vdev_id, peer_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* Build and send a WMI_PEER_ASSOC_CONF_EVENT so ath12k_bss_assoc()'s
 * wait_for_completion_timeout(&ar->peer_assoc_done) fires. */
static void wcn7850_send_peer_assoc_conf(WCN7850State *s, PCIDevice *pci_dev,
                                         uint32_t vdev_id, const uint8_t *mac)
{
    uint8_t buf[128];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t off = 0;
    hdr->cmd_id = cpu_to_le32(WMI_PEER_ASSOC_CONF_EVENTID);
    off += sizeof(*hdr);

    uint32_t tlv_start = off;
    WmiTlv *tlv = (WmiTlv *)(buf + off);
    off += sizeof(*tlv);
    struct {
        uint32_t vdev_id;
        uint8_t mac[6];
        uint8_t rsvd[2];
    } QEMU_PACKED conf;
    conf.vdev_id = cpu_to_le32(vdev_id);
    memcpy(conf.mac, mac, 6);
    conf.rsvd[0] = conf.rsvd[1] = 0;
    memcpy(buf + off, &conf, sizeof(conf));
    off += sizeof(conf);
    tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_PEER_ASSOC_CONF_EVENT,
                                          off - (tlv_start + sizeof(*tlv))));

    uint32_t total = off;
    uint8_t htc_buf[256];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    fprintf(stderr, "WCN: PEER ASSOC CONF vdev=%u\n", vdev_id);
}

/* Build and send a WMI_VDEV_INSTALL_KEY_COMPLETE_EVENT. */
static void wcn7850_send_install_key_compl(WCN7850State *s, PCIDevice *pci_dev,
                                           uint32_t vdev_id)
{
    uint8_t buf[128];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t off = 0;
    hdr->cmd_id = cpu_to_le32(WMI_VDEV_INSTALL_KEY_COMPLETE_EVENTID);
    off += sizeof(*hdr);

    uint32_t tlv_start = off;
    WmiTlv *tlv = (WmiTlv *)(buf + off);
    off += sizeof(*tlv);
    struct {
        uint32_t vdev_id;
        uint32_t key_idx;
        uint32_t status;
    } QEMU_PACKED kc;
    kc.vdev_id = cpu_to_le32(vdev_id);
    kc.key_idx = 0;
    kc.status = 0;
    memcpy(buf + off, &kc, sizeof(kc));
    off += sizeof(kc);
    tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_VDEV_INSTALL_KEY_COMPLETE_EVENT,
                                          off - (tlv_start + sizeof(*tlv))));

    uint32_t total = off;
    uint8_t htc_buf[256];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    fprintf(stderr, "WCN: INSTALL KEY COMPLETE vdev=%u\n", vdev_id);
}

/* Emit a WMI_SCAN_EVENT with the given event_type/reason. */
static void wcn7850_send_scan_event(WCN7850State *s, PCIDevice *pci_dev,
                                    uint32_t event_type, uint32_t reason,
                                    uint32_t freq)
{
    uint8_t buf[128];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t off = 0;
    hdr->cmd_id = cpu_to_le32(WMI_SCAN_EVENTID);
    off += sizeof(*hdr);

    uint32_t tlv_start = off;
    WmiTlv *tlv = (WmiTlv *)(buf + off);
    off += sizeof(*tlv);
    struct {
        uint32_t event_type;
        uint32_t reason;
        uint32_t channel_freq;
        uint32_t scan_req_id;
        uint32_t scan_id;
        uint32_t vdev_id;
        uint32_t tsf;
    } QEMU_PACKED se;
    se.event_type = cpu_to_le32(event_type);
    se.reason = cpu_to_le32(reason);
    se.channel_freq = cpu_to_le32(freq);
    se.scan_req_id = 0;
    se.scan_id = cpu_to_le32(s->scan_id);
    se.vdev_id = cpu_to_le32(s->scan_vdev_id);
    se.tsf = 0;
    memcpy(buf + off, &se, sizeof(se));
    off += sizeof(se);
    tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_SCAN_EVENT,
                                          off - (tlv_start + sizeof(*tlv))));

    uint32_t total = off;
    uint8_t htc_buf[256];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
}

/* Synthesize a minimal but valid 802.11 Beacon frame for the simulated AP
 * and deliver it via WMI_MGMT_RX_EVENTID so mac80211 builds the BSS. */
static void wcn7850_send_beacon(WCN7850State *s, PCIDevice *pci_dev)
{
    /* 802.11 Beacon: MAC header (24B) + timestamp(8) + interval(2) +
     * capability(2) + SSID IE + supported rates IE + DS IE + (optional RSN). */
    uint8_t frame[256];
    uint32_t off = 0;
    /* MAC header: FC=Beacon(0x80), dur=0, DA=bcast, SA=BSSID, BSSID, seq=0 */
    frame[off++] = 0x80; frame[off++] = 0x00;
    frame[off++] = 0x00; frame[off++] = 0x00;
    memset(frame + off, 0xff, 6); off += 6;            /* DA = broadcast */
    memcpy(frame + off, s->ap_bssid, 6); off += 6;     /* SA = BSSID */
    memcpy(frame + off, s->ap_bssid, 6); off += 6;     /* BSSID */
    frame[off++] = 0x00; frame[off++] = 0x00;          /* seq */
    /* beacon interval = 100 TU */
    frame[off++] = 0x64; frame[off++] = 0x00;
    /* capability: ESS(0x0001), short preamble */
    frame[off++] = 0x01; frame[off++] = 0x00;
    /* SSID IE (id 0, len, ssid) */
    frame[off++] = 0x00; frame[off++] = s->ap_ssid_len;
    memcpy(frame + off, s->ap_ssid, s->ap_ssid_len); off += s->ap_ssid_len;
    /* Supported rates IE (id 1): 6,9,12,18,24,36,48,54 Mbps */
    frame[off++] = 0x01; frame[off++] = 8;
    static const uint8_t rates[8] = {0x8c,0x12,0x98,0x24,0xb0,0x48,0x60,0x6c};
    memcpy(frame + off, rates, 8); off += 8;
    /* DS parameter set IE (id 3): channel */
    frame[off++] = 0x03; frame[off++] = 1;
    frame[off++] = (uint8_t)s->ap_channel;
    uint32_t frame_len = off;

    /* Wrap in WMI_MGMT_RX_EVENTID: MGMT_RX_HDR TLV + ARRAY_BYTE (frame). */
    uint8_t buf[512];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t b = 0;
    hdr->cmd_id = cpu_to_le32(WMI_MGMT_RX_EVENTID);
    b += sizeof(*hdr);

    /* MGMT_RX_HDR TLV */
    uint32_t hdr_start = b;
    WmiTlv *tlv = (WmiTlv *)(buf + b);
    b += sizeof(*tlv);
    /* Must match struct ath12k_wmi_mgmt_rx_params (68 bytes, ATH_MAX_ANTENNA=4):
     * channel, snr, rate, phy_mode, buf_len, status, rssi_ctl[4], flags,
     * rssi, tsf_delta, rx_tsf_l32, rx_tsf_u32, pdev_id, chan_freq. */
    struct {
        uint32_t channel;
        uint32_t snr;
        uint32_t rate;
        uint32_t phy_mode;
        uint32_t buf_len;
        uint32_t status;
        uint32_t rssi_ctl[4];
        uint32_t flags;
        int32_t  rssi;
        uint32_t tsf_delta;
        uint32_t rx_tsf_l32;
        uint32_t rx_tsf_u32;
        uint32_t pdev_id;
        uint32_t chan_freq;
    } QEMU_PACKED mh;
    memset(&mh, 0, sizeof(mh));
    mh.channel = cpu_to_le32(s->ap_channel);
    mh.snr = cpu_to_le32(40);
    mh.rate = cpu_to_le32(120); /* 12 Mbps */
    mh.phy_mode = cpu_to_le32(0xb); /* 11A */
    mh.buf_len = cpu_to_le32(frame_len);
    mh.status = 0;
    mh.rssi = cpu_to_le32((uint32_t)(-50));
    mh.pdev_id = 0;
    mh.chan_freq = cpu_to_le32(s->ap_freq);
    memcpy(buf + b, &mh, sizeof(mh));
    b += sizeof(mh);
    tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_MGMT_RX_HDR,
                                          b - (hdr_start + sizeof(*tlv))));

    /* ARRAY_BYTE TLV carrying the raw frame */
    uint32_t arr_start = b;
    WmiTlv *tlv2 = (WmiTlv *)(buf + b);
    b += sizeof(*tlv2);
    memcpy(buf + b, frame, frame_len);
    b += frame_len;
    tlv2->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_ARRAY_BYTE,
                                           b - (arr_start + sizeof(*tlv2))));

    uint32_t total = b;
    uint8_t htc_buf[768];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    fprintf(stderr, "WCN: MGMT RX beacon sent (chan %u, %u bytes)\n",
            s->ap_channel, frame_len);
}

/* WMI command handlers (connect path) */

static void wcn7850_handle_peer_create(WCN7850State *s, PCIDevice *pci_dev,
                                       const uint8_t *payload, uint32_t len)
{
    /* Wire layout: WmiCmdHdr(4) + struct wmi_peer_create_cmd
     *   { tlv_header(4), vdev_id(4), peer_macaddr(8), peer_type(4) } */
    if (len < 16)
        return;
    const uint8_t *body = payload + sizeof(WmiCmdHdr); /* start of struct */
    uint32_t vdev_id = le32_to_cpu(*(const uint32_t *)(body + 4));
    /* peer_macaddr is at struct offset 8; we force it to the simulated AP. */
    s->vdev_id = vdev_id;
    memcpy(s->peer_mac, s->ap_bssid, 6);
    qemu_log("WCN: WMI PEER CREATE vdev=%u mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
             vdev_id, s->peer_mac[0], s->peer_mac[1], s->peer_mac[2],
             s->peer_mac[3], s->peer_mac[4], s->peer_mac[5]);
    /* The driver waits for an HTT T2H PEER_MAP v1 (peer_map_unmap_version=1). */
    wcn7850_send_htt_peer_map(s, pci_dev, vdev_id, s->peer_id, s->ap_bssid);
    s->peer_mapped = true;
}

static void wcn7850_handle_peer_assoc(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len)
{
    qemu_log("WCN: WMI PEER ASSOC cmd len=%u\n", len);
    wcn7850_send_peer_assoc_conf(s, pci_dev, s->vdev_id, s->ap_bssid);
}

static void wcn7850_handle_scan_start(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len)
{
    /* Wire layout: WmiCmdHdr(4) + struct wmi_start_scan_cmd
     *   { tlv_header(4), scan_id(4), scan_req_id(4), vdev_id(4), ... } */
    if (len < 16)
        return;
    const uint8_t *body = payload + sizeof(WmiCmdHdr);
    s->scan_id = le32_to_cpu(*(const uint32_t *)(body + 4));
    s->scan_vdev_id = le32_to_cpu(*(const uint32_t *)(body + 12));
    qemu_log("WCN: WMI SCAN START scan_id=0x%x vdev=%u\n", s->scan_id,
             s->scan_vdev_id);

    /* Scan flow the driver expects:
     *   STARTED -> (per channel) FOREIGN_CHAN + beacon (MGMT_RX) + BSS_CHANNEL
     *   -> COMPLETED */
    wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_STARTED, 0, 0);
    wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_FOREIGN_CHAN, 0,
                            s->ap_freq);
    /* Deliver the simulated AP beacon so mac80211 records the BSS. */
    wcn7850_send_beacon(s, pci_dev);
    wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_BSS_CHANNEL, 0,
                            s->ap_freq);
    wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_COMPLETED,
                            WMI_SCAN_REASON_COMPLETED, 0);
    fprintf(stderr, "WCN: scan sequence emitted for SSID '%.*s'\n",
            s->ap_ssid_len, s->ap_ssid);
}

static void wcn7850_handle_mgmt_tx(WCN7850State *s, PCIDevice *pci_dev,
                                   const uint8_t *payload, uint32_t len)
{
    /* Wire layout: WmiCmdHdr(4) + struct wmi_mgmt_send_cmd
     *   { tlv_header(4), vdev_id(4), desc_id(4), chanfreq(4),
     *     paddr_lo(4), paddr_hi(4), frame_len(4), ... } */
    if (len < 28)
        return;
    const uint8_t *body = payload + sizeof(WmiCmdHdr);
    uint32_t vdev_id = le32_to_cpu(*(const uint32_t *)(body + 4));
    uint32_t frame_len = le32_to_cpu(*(const uint32_t *)(body + 24));
    qemu_log("WCN: WMI MGMT TX vdev=%u frame_len=%u\n", vdev_id, frame_len);

    /* Emit MGMT_TX_COMPLETION_EVENT so the driver's tx completion fires. */
    uint8_t buf[64];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t b = 0;
    hdr->cmd_id = cpu_to_le32(WMI_MGMT_TX_COMPLETION_EVENTID);
    b += sizeof(*hdr);
    uint32_t tlv_start = b;
    WmiTlv *tlv = (WmiTlv *)(buf + b);
    b += sizeof(*tlv);
    struct {
        uint32_t vdev_id;
        uint32_t tx_status;
        uint32_t cookie; /* echoes frame pointer cookie from the cmd */
    } QEMU_PACKED mc;
    mc.vdev_id = cpu_to_le32(vdev_id);
    mc.tx_status = 0;
    mc.cookie = 0;
    memcpy(buf + b, &mc, sizeof(mc));
    b += sizeof(mc);
    tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_MGMT_TX_COMPL_EVENT,
                                          b - (tlv_start + sizeof(*tlv))));
    uint32_t total = b;
    uint8_t htc_buf[128];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_wmi_send_event(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
}

static void wcn7850_handle_install_key(WCN7850State *s, PCIDevice *pci_dev,
                                       const uint8_t *payload, uint32_t len)
{
    qemu_log("WCN: WMI INSTALL KEY cmd len=%u\n", len);
    wcn7850_send_install_key_compl(s, pci_dev, s->vdev_id);
}

/* Handle HTT command from driver (sent via HTC EP HTT on CE pipe 4).
 * The only command during core bring-up is the version request; reply with
 * HTT_T2H_MSG_TYPE_VERSION_CONF so the driver's htt_tgt_version_received
 * completion fires. */
static void wcn7850_handle_htt_cmd(WCN7850State *s, PCIDevice *pci_dev,
                                   const uint8_t *payload, uint32_t len)
{
    if (len < 4) {
        return;
    }
    uint32_t ver_reg_info = le32_to_cpu(*(const uint32_t *)payload);
    uint32_t msg_type = ver_reg_info & 0xff;  /* HTT_VER_REQ_INFO_MSG_ID */

    if (msg_type == 0) {  /* HTT_H2T_MSG_TYPE_VERSION_REQ */
        uint8_t buf[16];
        HtcHdr *hdr = (HtcHdr *)buf;
        /* eid = 1 (HTT), payload_len = 4 */
        hdr->hdr_info = cpu_to_le32((4 << 16) | 1);
        hdr->ctrl_info = 0;
        uint32_t *ver = (uint32_t *)(buf + sizeof(HtcHdr));
        /* HTT_T2H_VERSION_CONF_MAJOR = bits 23..16, MINOR = bits 15..8 */
        *ver = cpu_to_le32((3 << 16) | (0 << 8));  /* major 3, minor 0 */
        /* Deliver on CE pipe 1 (HTT / HTC control DL). */
        wcn7850_wmi_send_event(s, pci_dev, 1, buf, sizeof(HtcHdr) + 4);
    }
}

/* Dispatch HTC message to appropriate handler */
static void wcn7850_htc_dispatch(WCN7850State *s, PCIDevice *pci_dev,
                                  uint8_t epid, const uint8_t *payload,
                                  uint32_t len)
{
    fprintf(stderr, "WCN: htc_dispatch epid=%u len=%u\n", epid, len);
    switch (epid) {
    case HTC_EP_CTRL: {
        HtcHdr *hdr = (HtcHdr *)(payload - 8);
        uint16_t msg_id = (payload[0] | (payload[1] << 8)) & 0x3f;
        fprintf(stderr, "WCN: HTC ctrl msg_id=%u hdr_info=0x%08x ctrl_info=0x%08x\n",
                msg_id, hdr->hdr_info, hdr->ctrl_info);

        switch (msg_id) {
        case HTC_MSG_SETUP_COMPLETE_EX_ID:
            fprintf(stderr, "WCN: HTC setup complete\n");
            /* Host sends setup complete only after all endpoint rx_cb's are
             * registered. Now it is safe to deliver WMI service-ready events. */
            s->wmi_service_ready_pending = true;
            break;
        case HTC_MSG_CONNECT_SERVICE_ID:
            fprintf(stderr, "WCN: HTC connect svc\n");
            wcn7850_htc_handle_connect(s, pci_dev, payload);
            break;
        default:
            qemu_log("WCN: unknown HTC ctrl msg %u\n", msg_id);
            break;
        }
        break;
    }
    case 1: {  /* HTT endpoint */
        wcn7850_handle_htt_cmd(s, pci_dev, payload, len);
        break;
    }
    case HTC_EP_WMI:
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
    qemu_log("WCN: process_ce_src pipe=%d tp=%u hp=%u base=0x%lx\n",
             ce_pipe, s->ce_src_tp[ce_pipe], s->ce_src_hp[ce_pipe],
             (unsigned long)s->ce_src_base[ce_pipe]);
    uint64_t base = s->ce_src_base[ce_pipe];
    uint32_t esize = s->ce_src_entry_size[ce_pipe] ?: 16;
    uint32_t ring_sz = s->ce_src_size[ce_pipe] ?: (WCN7850_CE_SRC_RING_SIZE * esize);
    uint32_t tp = s->ce_src_tp[ce_pipe];
    uint32_t hp = s->ce_src_hp[ce_pipe];

    if (!base || !ring_sz)
        return;

    if (tp != hp)
        qemu_log("WCN: process_ce_src pipe=%d tp=%u hp=%u -> %d descs to process\n",
                 ce_pipe, tp, hp, (hp >= tp ? hp - tp : ring_sz - tp + hp) / esize);
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
            fprintf(stderr, "WCN: SRC buf read FAILED at 0x%lx len=%u\n",
                    (unsigned long)buf_addr, buf_len);
            tp += esize;
            if (tp >= ring_sz) tp = 0;
            continue;
        }

        /* Parse HTC header */
        HtcHdr *htc_hdr = (HtcHdr *)htc_buf;
        uint8_t epid = HTC_HDR_EPID(htc_hdr);
        uint32_t payload_len = HTC_HDR_PAYLOAD_LEN(htc_hdr);
        fprintf(stderr, "WCN: SRC desc at 0x%lx buf=0x%lx len=%u hdr=0x%08x ctrl=0x%08x\n",
                (unsigned long)desc_addr, (unsigned long)buf_addr, buf_len,
                htc_hdr->hdr_info, htc_hdr->ctrl_info);
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
        uint32_t tp_le = cpu_to_le32(tp / 4);
        pci_dma_write(pci_dev, s->ce_src_tp_addr[ce_pipe],
                      &tp_le, sizeof(tp_le));
    }
}

/* Decode a CE register access. win_off is the offset within the CE register
 * window (addr & (WINDOW_SIZE - 1)). The per-pipe layout (stride 0x2000,
 * destination block at +0x1000) matches the driver's HAL_CE_WFSS_CE_REG_BASE
 * (0x01b80000) register map. Invoked both for the legacy dynamic window path
 * (window_select routed to 0x37) and for the static CE window at BAR0 0x100000
 * used when the driver sets static_window_map. */
static void wcn7850_handle_ce_mmio(WCN7850State *s, PCIDevice *pci_dev,
                                  hwaddr win_off, uint64_t val, unsigned size)
{
    int ce_pipe = -1;
    bool is_dst = false;

    if (win_off < WCN7850_CE_STRIDE * WCN7850_CE_COUNT) {
        ce_pipe = win_off / WCN7850_CE_STRIDE;
        if ((win_off % WCN7850_CE_STRIDE) >= 0x1000)
            is_dst = true;
    }

    if (ce_pipe >= 0 && ce_pipe < WCN7850_CE_COUNT) {
        hwaddr ce_blk_off = win_off % WCN7850_CE_STRIDE;
        hwaddr blk = ce_blk_off % 0x1000;
        if (ce_pipe == 2)
            fprintf(stderr, "WCN: CE2 mmio win_off=0x%lx blk=0x%lx is_dst=%d val=0x%x\n",
                    (unsigned long)win_off, (unsigned long)blk, is_dst,
                    (uint32_t)val);

        /* R0 config registers (base LSB/MSB + ring id) at offset 0x0 of the
         * SRC or DST block (SRC block at +0x0, DST block at +0x1000). */
        if (blk < 0x100) {
            memcpy(s->window_memory + win_off, &val, size);
            hwaddr cfg_base = win_off - blk;
            uint32_t base_lsb_cfg, base_msb_cfg, ring_id_cfg;
            base_lsb_cfg = *(uint32_t *)(s->window_memory + cfg_base +
                                        WCN7850_CE_RING_BASE_LSB);
            base_msb_cfg = *(uint32_t *)(s->window_memory + cfg_base +
                                        WCN7850_CE_RING_BASE_MSB);
            ring_id_cfg  = *(uint32_t *)(s->window_memory + cfg_base +
                                        WCN7850_CE_RING_ID);
            if (base_lsb_cfg || base_msb_cfg) {
                uint64_t base_addr = base_lsb_cfg |
                    ((uint64_t)(base_msb_cfg & 0xff) << 32);
                uint32_t ring_sz = (base_msb_cfg >> 8) & 0xffff;
                uint32_t entry_size = (ring_id_cfg & 0xff) * 4;
                if (!entry_size) entry_size = 16;
                if (!ring_sz) ring_sz = 512 * entry_size;
                /* host address where the device writes its pointer
                 * (TP for SRC, HP for DST) - CE_RING_HP_ADDR at +0x1c/+0x20. */
                uint32_t hp_addr_lsb = *(uint32_t *)(s->window_memory +
                    cfg_base + WCN7850_CE_RING_HP_ADDR_LSB);
                uint32_t hp_addr_msb = *(uint32_t *)(s->window_memory +
                    cfg_base + WCN7850_CE_RING_HP_ADDR_MSB);
                uint64_t hp_addr = hp_addr_lsb |
                    ((uint64_t)(hp_addr_msb & 0xff) << 32);
                uint32_t ring_sz_bytes = ring_sz * entry_size;
                if (is_dst) {
                    s->ce_dst_base[ce_pipe] = base_addr;
                    s->ce_dst_size[ce_pipe] = ring_sz_bytes;
                    s->ce_dst_entry_size[ce_pipe] = entry_size;
                    if (hp_addr)
                        s->ce_dst_hp_addr[ce_pipe] = hp_addr;
                    fprintf(stderr, "WCN: CE dst cfg pipe=%d base=0x%lx sz=%u esz=%u\n",
                             ce_pipe, (unsigned long)base_addr, ring_sz_bytes, entry_size);
                } else {
                    s->ce_src_base[ce_pipe] = base_addr;
                    s->ce_src_size[ce_pipe] = ring_sz_bytes;
                    s->ce_src_entry_size[ce_pipe] = entry_size;
                    if (hp_addr)
                        s->ce_src_tp_addr[ce_pipe] = hp_addr;
                    qemu_log("WCN: CE src cfg pipe=%d base=0x%lx sz=%u esz=%u tp_addr=0x%lx\n",
                             ce_pipe, (unsigned long)base_addr, ring_sz_bytes,
                             entry_size, (unsigned long)hp_addr);
                }
                /* DST status ring config at +0x58..+0x70 */
                if (is_dst) {
                    uint32_t sts_lsb = *(uint32_t *)(s->window_memory +
                        cfg_base + 0x58);
                    uint32_t sts_msb = *(uint32_t *)(s->window_memory +
                        cfg_base + 0x5c);
                    if (sts_lsb || sts_msb) {
                        s->ce_sts_base[ce_pipe] = sts_lsb |
                            ((uint64_t)(sts_msb & 0xff) << 32);
                        uint32_t sts_rind = *(uint32_t *)(s->window_memory +
                            cfg_base + 0x60);
                        uint32_t sts_entry_size = (sts_rind & 0xff) * 4;
                        if (!sts_entry_size) sts_entry_size = 16;
                        s->ce_sts_size[ce_pipe] = ((sts_msb >> 8) & 0xffff) * sts_entry_size;
                        s->ce_sts_entry_size[ce_pipe] = sts_entry_size;
                        uint32_t st_hp_lsb = *(uint32_t *)(s->window_memory +
                            cfg_base + 0x6c);
                        uint32_t st_hp_msb = *(uint32_t *)(s->window_memory +
                            cfg_base + 0x70);
                        s->ce_sts_hp_addr[ce_pipe] = st_hp_lsb |
                            ((uint64_t)(st_hp_msb & 0xff) << 32);
                        fprintf(stderr, "WCN: CE%d sts base=0x%lx sz=%u esz=%u hp_addr=0x%lx\n",
                                ce_pipe,
                                (unsigned long)s->ce_sts_base[ce_pipe],
                                s->ce_sts_size[ce_pipe],
                                s->ce_sts_entry_size[ce_pipe],
                                (unsigned long)s->ce_sts_hp_addr[ce_pipe]);
                    }
                }
            }
            return;
        }

        /* R2 pointer registers: HP/TP at +0x400/+0x404 (status HP at +0x408),
         * within the SRC or DST block. */
        if (blk >= WCN7850_CE_RING_HP_OFFSET &&
            blk < WCN7850_CE_RING_HP_OFFSET + 0x10) {
            memcpy(s->window_memory + win_off, &val, size);
            fprintf(stderr, "WCN: R2 win_off=0x%lx blk=0x%lx pipe=%d is_dst=%d val=0x%x\n",
                    (unsigned long)win_off, (unsigned long)blk, ce_pipe,
                    is_dst, (uint32_t)val);
            if (blk == WCN7850_CE_RING_HP_OFFSET) {
                if (is_dst) {
                    s->ce_dst_hp[ce_pipe] = val;
                    int wmi_dl = s->htc_ep[HTC_EP_WMI].dl_pipe ?: 2;
                    if (ce_pipe == wmi_dl) {
                        uint32_t ring_sz = s->ce_dst_size[ce_pipe] ?:
                            (WCN7850_CE_DST_RING_SIZE *
                             (s->ce_dst_entry_size[ce_pipe] ?: 16));
                        uint32_t esize = s->ce_dst_entry_size[ce_pipe] ?: 16;
                        uint32_t doorbell_bytes = val * 4;
                        uint32_t prev = s->ce_dst_drv_hp_prev[ce_pipe];
                        if (doorbell_bytes != prev) {
                            uint32_t delta = (doorbell_bytes - prev
                                             + ring_sz) % ring_sz;
                            s->ce_dst_pending[ce_pipe] += delta / esize;
                        }
                        s->ce_dst_drv_hp_prev[ce_pipe] = doorbell_bytes;
                        s->ce_dst_drv_hp[ce_pipe] = doorbell_bytes;
                        fprintf(stderr, "WCN: CE-win DST HP pipe=%d val=0x%x pending=%u\n",
                                ce_pipe, val, s->ce_dst_pending[ce_pipe]);
                    }
                } else {
                    qemu_log("WCN: CE-dir HP pipe=%d val=0x%x\n", ce_pipe, (uint32_t)val);
                    s->ce_src_hp[ce_pipe] = val * 4;
                    wcn7850_process_ce_src(s, pci_dev, ce_pipe);
                }
            } else if (blk == WCN7850_CE_RING_TP_OFFSET) {
                if (is_dst)
                    s->ce_dst_tp[ce_pipe] = val * 4;
                else
                    s->ce_src_tp[ce_pipe] = val * 4;
            } else if (blk == (WCN7850_CE_RING_HP_OFFSET + 8)) {
                if (is_dst)
                    s->ce_sts_hp[ce_pipe] = val;
            }
            return;
        }
    }

    memcpy(s->window_memory + win_off, &val, size);
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

    /* TLVs: result(7) + num_phy(4) + board_id(7) + single_chip_mlo_support(4)
     * Optional fields carry NO validity prefix on the wire; the TLV's
     * presence alone signals validity to the vanilla QMI decoder. */
    uint32_t tlv_len = 7 + 4 + 7 + 4;
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
    /* TLV type 0x10: num_phy, length 1 = value(1) */
    tlv[0] = 0x10; tlv[1] = 0x01; tlv[2] = 0x00;
    tlv[3] = 0x01; /* num_phy = 1 */
    tlv += 4;
    /* TLV type 0x11: board_id, length 4 = board_id(4) */
    tlv[0] = 0x11; tlv[1] = 0x04; tlv[2] = 0x00;
    tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00; /* board_id = 0 */
    tlv += 7;
    /* TLV type 0x13: single_chip_mlo_support, length 1 = value(1) */
    tlv[0] = 0x13; tlv[1] = 0x01; tlv[2] = 0x00;
    tlv[3] = 0x00; /* single_chip_mlo_support = false */

    return total;
}

/* Build CAP response with chip/board/fw info */
static uint32_t wcn7850_qmi_build_cap_resp(uint8_t *buf, uint32_t buf_size,
                                             uint16_t txn_id)
{
    QmiHdr *hdr = (QmiHdr *)buf;
    uint8_t *tlv;
    /* result + chip_info(8) + board_info(4) + fw_version_info(4+1+9)
     * Optional fields carry NO validity prefix on the wire; the TLV's
     * presence alone signals validity to the vanilla QMI decoder. */
    const char *fw_ts = "2025-01-01";
    uint8_t fw_ts_len = strlen(fw_ts);
    uint32_t tlv_val = 0;
    uint32_t tlv_len = 0;

    /* result TLV (type=0x02, val=4 bytes): 3 header + 4 = 7 */
    /* chip_info TLV (type=0x10): chip_id(4) + chip_family(4) = 8 */
    /* board_info TLV (type=0x11): board_id(4) = 4 */
    /* fw_version_info TLV (type=0x13): fw_version(4) + string_len(1) + fw_ts_len */
    tlv_len = 7 + (3 + 8) + (3 + 4) + (3 + 4 + 1 + fw_ts_len);
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
    /* TLV: chip_info (type 0x10, len 0x08): chip_id(4) + chip_family(4) */
    tlv[0] = 0x10; tlv[1] = 8; tlv[2] = 0x00;
    tlv_val = cpu_to_le32(2);
    memcpy(&tlv[3], &tlv_val, 4); /* chip_id = 2 */
    tlv_val = cpu_to_le32(0);
    memcpy(&tlv[7], &tlv_val, 4); /* chip_family = 0 */
    tlv += 3 + 8;
    /* TLV: board_info (type 0x11, len 0x04): board_id(4) */
    tlv[0] = 0x11; tlv[1] = 4; tlv[2] = 0x00;
    tlv_val = cpu_to_le32(0);
    memcpy(&tlv[3], &tlv_val, 4); /* board_id = 0 */
    tlv += 3 + 4;
    /* TLV: fw_version_info (type 0x13): fw_version(4) + string */
    tlv[0] = 0x13;
    tlv[1] = (4 + 1 + fw_ts_len) & 0xff;
    tlv[2] = ((4 + 1 + fw_ts_len) >> 8) & 0xff;
    tlv_val = cpu_to_le32(1);
    memcpy(&tlv[3], &tlv_val, 4); /* fw_version = 1 */
    tlv[7] = fw_ts_len; /* string length prefix (1 byte) */
    memcpy(&tlv[8], fw_ts, fw_ts_len); /* string content */

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
        /* After M3 info, send FW_READY_IND. The driver's wlan_enable /
         * core_start is itself triggered by FW_READY_IND, so it must be
         * sent before the driver attempts WLAN_CFG/WLAN_MODE. */
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
/* Send QRTR RESUME_TX to unblock the host's flow control when
 * QRTR_TX_FLOW_HIGH (10 pending credits) is reached. */
static bool wcn7850_send_resume_tx(WCN7850State *s, PCIDevice *pci_dev,
                                    uint64_t ch_ctxt_addr,
                                    uint64_t er_ctxt_addr)
{
    uint8_t buf[sizeof(QrtrHdrV1) + sizeof(QrtrCtrlPkt)];
    QrtrHdrV1 *hdr = (QrtrHdrV1 *)buf;
    QrtrCtrlPkt *ctrl = (QrtrCtrlPkt *)(buf + sizeof(QrtrHdrV1));
    uint64_t dl_ctxt_addr;

    memset(buf, 0, sizeof(buf));

    hdr->version = cpu_to_le32(QRTR_PROTO_VER_1);
    hdr->type = cpu_to_le32(QRTR_TYPE_RESUME_TX);
    hdr->src_node_id = cpu_to_le32(QRTR_NODE_FW);
    hdr->src_port_id = cpu_to_le32(QRTR_PORT_QMI_SERVICE);
    hdr->confirm_rx = 0;
    hdr->size = cpu_to_le32(sizeof(QrtrCtrlPkt));
    hdr->dst_node_id = cpu_to_le32(QRTR_NODE_HOST);
    hdr->dst_port_id = cpu_to_le32(QRTR_PORT_CTRL);

    /* client.node/client.port overlap server_service/server_instance */
    ctrl->cmd = cpu_to_le32(QRTR_TYPE_RESUME_TX);
    ctrl->server_service = cpu_to_le32(QRTR_NODE_FW);
    ctrl->server_instance = cpu_to_le32(QRTR_PORT_QMI_SERVICE);

    dl_ctxt_addr = ch_ctxt_addr + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
    return wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr, 0,
                                  buf, sizeof(buf));
}

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
    uint32_t confirm_rx = le32_to_cpu(hdr->confirm_rx);
    uint32_t payload_size = le32_to_cpu(hdr->size);
    const uint8_t *payload = data + sizeof(*hdr);

    (void)src_node;
    (void)src_port;

    /* If host asks us to confirm receipt, send RESUME_TX to unblock flow */
    if (confirm_rx) {
        qemu_log("WCN: QRTR confirm_rx -> send RESUME_TX\n");
        wcn7850_send_resume_tx(s, pci_dev, ch_ctxt_addr, er_ctxt_addr);
    }

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
    case QRTR_TYPE_RESUME_TX:
        /* Host sent us RESUME_TX — flow from model→host is unblocked.
         * We don't maintain pending counters, so nothing to do. */
        qemu_log("WCN: received RESUME_TX (ignored)\n");
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

        /* Post a transfer-completion event for the consumed UL TRE so the
         * host can recover/reuse the TRE. Without this the host's UL ring
         * fills up and further requests (e.g. WLAN_MODE) can never be
         * queued. */
        if (ch_id == MHI_CHAN_IPCR_UL) {
            uint64_t tre_addr = ctxt.rp;
            wcn7850_post_transfer_event(s, pci_dev, er_ctxt_addr, ctxt.erindex,
                                        MHI_CHAN_IPCR_UL, tre_addr, len);
        }

        g_free(buf);

        /* Advance rp */
        ctxt.rp += sizeof(MhiTre);
        if (ctxt.rp >= ctxt.rbase + ctxt.rlen)
            ctxt.rp = ctxt.rbase;
    }

    wcn7850_write_chan_ctxt(pci_dev, ch_ctxt_addr, &ctxt);

    /* Proactively send RESUME_TX so the host's QRTR flow control does
     * not stall at QRTR_TX_FLOW_HIGH (10) before the next QMI request
     * (e.g. WLAN_MODE) can be sent.  The host sets confirm_rx on every
     * LOW-th packet, but if those have already been processed we must
     * send one unconditionally to unblock any waiter. */
    if (ch_id == MHI_CHAN_IPCR_UL && s->chan_ctxt_addr) {
        wcn7850_send_resume_tx(s, pci_dev, s->chan_ctxt_addr, er_ctxt_addr);
    }
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

    if (addr < WCN7850_WINDOW_START + WCN7850_WINDOW_SIZE ||
        (addr >= WCN7850_CE_WINDOW_BASE &&
         addr < WCN7850_CE_WINDOW_BASE + WCN7850_WINDOW_SIZE) ||
        (addr >= WCN7850_UMAC_WINDOW_BASE &&
         addr < WCN7850_UMAC_WINDOW_BASE + WCN7850_WINDOW_SIZE)) {
        win_off = addr & (WCN7850_WINDOW_SIZE - 1);
        if (addr < WCN7850_WINDOW_START + WCN7850_WINDOW_SIZE &&
            s->window_select == WCN7850_WINDOW_TCSR_SEL &&
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

    /* ===== Shadow register doorbells (0x8fc + 4*n) =====
     * Must be checked BEFORE the TCL/REO SRNG intercept below: the driver's
     * CE shadow indices live in block 1 (sidx 29..), and some of those
     * addresses collide with TCL ring R0 config registers (e.g. sidx 35 =
     * 0x988 == WCN7850_TCL_RING_MISC(1)). Block 0 (sidx 0..6) aliases the
     * TCL/REO R0 config space, so those are NOT CE doorbells and are left
     * for the SRNG intercept further down (unmapped shadow addrs fall
     * through to it). */
    if (addr >= WCN7850_SHADOW_BASE &&
        addr < WCN7850_SHADOW_BASE + WCN7850_SHADOW_MAX * 4) {
        memcpy(s->bar0_always_on + addr, &val, size);
        unsigned int sidx = (addr - WCN7850_SHADOW_BASE) / 4;
        int ce_pipe = -1;
        int ce_type = -1;
        if (sidx == 29) { ce_pipe = 0; ce_type = 0; }
        else if (sidx == 30) { ce_pipe = 1; ce_type = 1; }
        else if (sidx == 31) { ce_pipe = 1; ce_type = 2; }
        else if (sidx == 32) { ce_pipe = 2; ce_type = 1; }
        else if (sidx == 33) { ce_pipe = 2; ce_type = 2; }
        else if (sidx == 34) { ce_pipe = 3; ce_type = 0; }
        else if (sidx == 35) { ce_pipe = 4; ce_type = 0; }
        if (ce_pipe >= 0) {
            uint32_t doorbell_val = val;
            if (ce_type == 0) {
                s->ce_src_hp[ce_pipe] = doorbell_val * 4;
                wcn7850_process_ce_src(s, pci_dev, ce_pipe);
            } else if (ce_type == 1) {
                uint32_t prev_bytes = s->ce_dst_drv_hp_prev[ce_pipe];
                uint32_t doorbell_bytes = doorbell_val * 4;
                s->ce_dst_drv_hp[ce_pipe] = doorbell_bytes;
                fprintf(stderr, "WCN: DST doorbell pipe=%d val=0x%x pending=%u seen=%d\n",
                        ce_pipe, doorbell_val, s->ce_dst_pending[ce_pipe],
                        s->ce_dst_doorbell_seen[ce_pipe]);
                if (!s->ce_dst_doorbell_seen[ce_pipe]) {
                    s->ce_dst_doorbell_seen[ce_pipe] = true;
                    s->ce_dst_pending[ce_pipe] = (s->ce_dst_size[ce_pipe] ?:
                        (WCN7850_CE_DST_RING_SIZE * (s->ce_dst_entry_size[ce_pipe] ?: 16)))
                        / (s->ce_dst_entry_size[ce_pipe] ?: 16);
                } else if (doorbell_bytes != prev_bytes) {
                    uint32_t ring_sz = s->ce_dst_size[ce_pipe] ?:
                        (WCN7850_CE_DST_RING_SIZE * (s->ce_dst_entry_size[ce_pipe] ?: 16));
                    uint32_t delta = (doorbell_bytes - prev_bytes + ring_sz) % ring_sz;
                    s->ce_dst_pending[ce_pipe] += delta / (s->ce_dst_entry_size[ce_pipe] ?: 16);
                }
                s->ce_dst_drv_hp_prev[ce_pipe] = doorbell_bytes;
            } else if (ce_type == 2) {
            }
            return;
        }
        /* Unmapped shadow address (e.g. TCL/REO R0 config aliased into this
         * range): fall through to the SRNG intercept below. */
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

    /* WBM2SW release ring config registers (R0) */
    for (int i = 0; i < WCN7850_NUM_WBM_RINGS; i++) {
        if (addr == WCN7850_WBM_RING_BASE_LSB(i) ||
            addr == WCN7850_WBM_RING_BASE_MSB(i) ||
            addr == WCN7850_WBM_RING_ID(i) ||
            addr == WCN7850_WBM_RING_MISC(i) ||
            addr == WCN7850_WBM_RING_MSI1_DATA(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            wcn7850_update_wbm_release_ring_cfg(s, i);
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

    /* WBM2SW release ring HP/TP (driver consuming TX completions) */
    for (int i = 0; i < WCN7850_NUM_WBM_RINGS; i++) {
        if (addr == WCN7850_WBM_RING_HP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_WBM2SW_RELEASE, i);
            if (r) r->hp = val;
            return;
        }
        if (addr == WCN7850_WBM_RING_TP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_WBM2SW_RELEASE, i);
            if (r) r->tp = val;
            return;
        }
    }

    if (addr < WCN7850_WINDOW_START) {
        memcpy(s->bar0_always_on + addr, &val, size);
        return;
    }

    /* Static windows (driver sets static_window_map): CE -> 0x100000,
     * UMAC -> 0x180000, accessed directly without the window register. */
    if (addr >= WCN7850_UMAC_WINDOW_BASE &&
        addr < WCN7850_UMAC_WINDOW_BASE + WCN7850_WINDOW_SIZE) {
        qemu_log("WCN: UMAC win write addr=0x%lx off=0x%lx val=0x%x sz=%u\n",
                 (unsigned long)addr, (unsigned long)(addr & (WCN7850_WINDOW_SIZE - 1)),
                 (uint32_t)val, size);
        memcpy(s->window_memory + (addr & (WCN7850_WINDOW_SIZE - 1)), &val, size);
        return;
    }
    if (addr >= WCN7850_CE_WINDOW_BASE &&
        addr < WCN7850_CE_WINDOW_BASE + WCN7850_WINDOW_SIZE) {
        qemu_log("WCN: CE win write addr=0x%lx off=0x%lx val=0x%x sz=%u\n",
                 (unsigned long)addr, (unsigned long)(addr & (WCN7850_WINDOW_SIZE - 1)),
                 (uint32_t)val, size);
        wcn7850_handle_ce_mmio(s, pci_dev, addr & (WCN7850_WINDOW_SIZE - 1),
                               val, size);
        return;
    }

    /* Dynamic window region at 0x80000, selected via the window register. */
    if (addr >= WCN7850_WINDOW_START &&
        addr < WCN7850_WINDOW_START + WCN7850_WINDOW_SIZE) {
        uint32_t win_off = addr - WCN7850_WINDOW_START;
        if (s->window_select == WCN7850_WINDOW_TCSR_SEL &&
            win_off == (WCN7850_TCSR_SOC_HW_VERSION & (WCN7850_WINDOW_SIZE - 1))) {
            return; /* TCSR is read-only */
        }
        if (s->window_select == (WCN7850_CE0_SRC_BASE / WCN7850_WINDOW_SIZE)) {
            wcn7850_handle_ce_mmio(s, pci_dev, win_off, val, size);
            return;
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

    if (!s->htc_ready_sent) {
        if (s->ce_dst_base[1] && s->ce_dst_pending[1]) {
            qemu_log("WCN: sending HTC ready\n");
            wcn7850_htc_send_ready(s, pci_dev);
            s->htc_ready_sent = true;
        }
    }

    if (s->wmi_service_ready_pending && !s->wmi_ready_sent) {
        /* WMI events are always sent through HTC control DL (CE pipe 1),
         * regardless of the WMI endpoint's configured dl_pipe.  Check that
         * CE1 has a posted buffer before proceeding. */
        if (s->ce_dst_base[1]) {
            qemu_log("WCN: sending WMI service ready (dl pipe 1)\n");
            wcn7850_send_wmi_service_ready(s, pci_dev);
            s->wmi_service_ready_pending = false;
        }
    }

    for (int i = 0; i < WCN7850_CE_COUNT; i++) {
        if (!s->ce_src_base[i])
            continue;
        /* Refresh HP from the ring's shared write-index so polled CE pipes
         * (e.g. HTT on pipe 4) are serviced even without a doorbell. */
        if (s->ce_src_tp_addr[i]) {
            uint32_t hp_le = 0;
            if (pci_dma_read(pci_dev, s->ce_src_tp_addr[i], &hp_le, 4) == MEMTX_OK)
                s->ce_src_hp[i] = le32_to_cpu(hp_le) * 4;
        }
        if (s->ce_src_hp[i] != s->ce_src_tp[i]) {
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

    s->htc_ready_sent = false;

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

    /* Initialize the simulated AP: an open BSS on channel 36 (5180 MHz). */
    s->ap_bssid[0] = 0x00; s->ap_bssid[1] = 0x14; s->ap_bssid[2] = 0x6c;
    s->ap_bssid[3] = 0x9a; s->ap_bssid[4] = 0x00; s->ap_bssid[5] = 0x01;
    memcpy(s->ap_ssid, "wcn7850-ap", 10);
    s->ap_ssid_len = 10;
    s->ap_channel = 36;
    s->ap_freq = 5180;
    s->peer_id = 1;
    s->peer_mapped = false;
    s->data_offload_enabled = true;

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
