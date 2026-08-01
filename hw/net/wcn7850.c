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
#include "net/checksum.h"
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

/* CE ring configuration and state */
typedef struct {
    uint64_t base;
    uint32_t size;
    uint32_t entry_size;
    uint64_t hp_addr;
    uint64_t tp_addr;
    uint32_t hp;
    uint32_t tp;
} CeSrcRing;

typedef struct {
    uint64_t base;
    uint32_t size;
    uint32_t entry_size;
    uint32_t hp;
    uint32_t tp;
    uint32_t drv_hp;
    uint32_t drv_hp_prev;
    uint32_t pending;
    bool doorbell_seen;
    uint32_t cons;
    uint64_t hp_addr;
    uint64_t tp_addr;
} CeDstRing;

#define WCN7850_PENDING_CE_EVENTS 8
typedef struct {
    uint8_t data[2048];
    uint32_t len;
    int32_t pipe;
} DeferredCeEvent;

#define WCN7850_PENDING_RX_FRAMES 8
typedef struct {
    uint8_t data[65535];
    uint32_t len;
} PendingRxFrame;

typedef struct {
    uint64_t base;
    uint32_t size;
    uint32_t entry_size;
    uint32_t hp;
    uint64_t hp_addr;
} CeStsRing;



/* WCN7850 MSI layout (ath12k_msi_config in wifi7/pci.c):
 *   MHI: base_vector 0 (vectors 0..2)
 *   CE:  base_vector 3 (vectors 3..7 for CE0..CE3, CE5)
 *   DP:  base_vector 8 (vectors 8..15 for 8 ext_irq groups)
 * The ath12k driver assigns each CE pipe an MSI vector index of
 *   CE_MSI_BASE + (count of non-CE_ATTR_DIS_INTR CEs before this pipe),
 * which matches ath12k_pci_get_ce_msi_idx. */
#define WCN7850_CE_MSI_BASE 3
#define WCN7850_DP_MSI_BASE 8

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

/* REO command ring (host → REO engine) */
#define WCN7850_REO_CMD_BASE_LSB   0x0000028c
#define WCN7850_REO_CMD_BASE_MSB   0x00000290
#define WCN7850_REO_CMD_RING_ID    0x00000294
#define WCN7850_REO_CMD_MISC       0x0000029c
#define WCN7850_REO_CMD_HP         0x00003020
#define WCN7850_REO_CMD_TP         0x00003024

/* REO status ring (REO engine → host) */
#define WCN7850_REO_STATUS_BASE_LSB  0x00000a84
#define WCN7850_REO_STATUS_BASE_MSB  0x00000a88
#define WCN7850_REO_STATUS_RING_ID   0x00000a8c
#define WCN7850_REO_STATUS_MISC      0x00000a94
#define WCN7850_REO_STATUS_HP        0x000030a8
#define WCN7850_REO_STATUS_TP        0x000030ac

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

/* SW2WBM_RELEASE (SRC) R0 registers — the driver returns RX buffers here.
 * WCN7850: SW0 base=0x037c, SW1 base=0x0284 (not a simple stride pattern). */
#define WCN7850_SW2WBM0_BASE_LSB   0x0000037c
#define WCN7850_SW2WBM0_BASE_MSB   0x00000380
#define WCN7850_SW2WBM0_RING_ID    0x00000384
#define WCN7850_SW2WBM0_MISC       0x0000038c
#define WCN7850_SW2WBM1_BASE_LSB   0x00000284
#define WCN7850_SW2WBM1_BASE_MSB   0x00000288
#define WCN7850_SW2WBM1_RING_ID    0x0000028c
#define WCN7850_SW2WBM1_MISC       0x00000294
/* SW2WBM_RELEASE R2 (pointer) registers */
#define WCN7850_SW2WBM0_HP         0x00003010
#define WCN7850_SW2WBM1_HP         0x00003018

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
#define WMI_GRP_STA_PS     0x9
#define WMI_GRP_DFS        0xa
#define WMI_GRP_MISC       0x1d
#define WMI_GRP_STATS      0x16

#define WMI_TLV_CMD(grp_id) (((grp_id) << 12) | 0x1)
#define WMI_EVT_GRP_START_ID(grp_id) (((grp_id) << 12) | 0x1)

/* WMI command IDs */
#define WMI_SERVICE_READY_EVENTID  0x1
#define WMI_READY_EVENTID          0x2
#define WMI_INIT_CMDID             0x1
#define WMI_VDEV_CREATE_CMDID      WMI_TLV_CMD(WMI_GRP_VDEV)
#define WMI_VDEV_DELETE_CMDID      (WMI_TLV_CMD(WMI_GRP_VDEV) + 1) /* 0x5002 */
#define WMI_PEER_CREATE_CMDID      WMI_TLV_CMD(WMI_GRP_PEER)

/* Additional WMI command/event IDs used by the STA connect path.
 * (event groups use WMI_EVT_GRP_START_ID(grp) = ((grp)<<12)|0x1) */
#define WMI_START_SCAN_CMDID       WMI_TLV_CMD(WMI_GRP_SCAN)        /* 0x3001 */
#define WMI_SCAN_CHAN_LIST_CMDID   (WMI_TLV_CMD(WMI_GRP_SCAN) + 2)  /* 0x3003 */
#define WMI_VDEV_START_REQUEST_CMDID (WMI_TLV_CMD(WMI_GRP_VDEV) + 2) /* 0x5003 */
#define WMI_VDEV_UP_CMDID          (WMI_TLV_CMD(WMI_GRP_VDEV) + 4)  /* 0x5005 */
#define WMI_PEER_ASSOC_CMDID       (WMI_TLV_CMD(WMI_GRP_PEER) + 4)  /* 0x6005 */
#define WMI_VDEV_SET_PARAM_CMDID   (WMI_TLV_CMD(WMI_GRP_VDEV) + 7)   /* 0x5008 */
#define WMI_VDEV_INSTALL_KEY_CMDID (WMI_TLV_CMD(WMI_GRP_VDEV) + 8)   /* 0x5009 */

/* STA power-save group (grp 0x9) */
#define WMI_STA_PS_PARAM_CMDID     WMI_TLV_CMD(WMI_GRP_STA_PS)        /* 0x9001 */
#define WMI_STA_PS_SET_STATE_CMDID (WMI_TLV_CMD(WMI_GRP_STA_PS) + 1)  /* 0x9002 */

/* DFS group (grp 0xa) */
#define WMI_DFS_ENABLE_CMDID       WMI_TLV_CMD(WMI_GRP_DFS)           /* 0xa001 */
#define WMI_DFS_PHYERR_FILTER_ENA_CMDID (WMI_TLV_CMD(WMI_GRP_DFS) + 1) /* 0xa002 */
#define WMI_DFS_PHYERR_FILTER_DIS_CMDID (WMI_TLV_CMD(WMI_GRP_DFS) + 2) /* 0xa003 */
#define WMI_DFS_CMDID_0xa005       (WMI_TLV_CMD(WMI_GRP_DFS) + 4)    /* 0xa005 */

/* MISC group (grp 0x1d) */
#define WMI_LRO_CONFIG_CMDID      (WMI_TLV_CMD(WMI_GRP_MISC) + 15)   /* 0x1d010 */

#define WMI_SCAN_EVENTID           WMI_EVT_GRP_START_ID(WMI_GRP_SCAN)    /* 0x3001 */
#define WMI_MGMT_RX_EVENTID        WMI_EVT_GRP_START_ID(WMI_GRP_MGMT)    /* 0x7001 */
#define WMI_MGMT_TX_SEND_CMDID     (WMI_TLV_CMD(WMI_GRP_MGMT) + 7)       /* 0x7008 */
#define WMI_PEER_ASSOC_CONF_EVENTID (WMI_TLV_CMD(WMI_GRP_PEER) + 5)      /* 0x6006 */
#define WMI_VDEV_INSTALL_KEY_COMPLETE_EVENTID (WMI_TLV_CMD(WMI_GRP_VDEV) + 2) /* 0x5003 */
#define WMI_MGMT_TX_COMPLETION_EVENTID (WMI_TLV_CMD(WMI_GRP_MGMT) + 5)   /* 0x7006 */
#define WMI_VDEV_START_RESP_EVENTID WMI_TLV_CMD(WMI_GRP_VDEV)            /* 0x5001 */
#define WMI_VDEV_DELETE_RESP_EVENTID (WMI_TLV_CMD(WMI_GRP_VDEV) + 5)     /* 0x5006 */
#define WMI_REQUEST_PDEV_STAT        BIT(2)                               /* 0x4     */
#define WCN7850_STATION_IPV4         0x0a00020fU                         /* 10.0.2.15 */
#define WMI_REQUEST_STATS_CMDID      WMI_TLV_CMD(WMI_GRP_STATS)          /* 0x16001 */
/* Not derivable from WMI_GRP_STATS: the WMI_UPDATE_STATS_EVENTID entry lives
 * in the MISC group (0x1d), auto-inc from WMI_ECHO_EVENTID=0x1d001.
 * Verified against the kernel's enum wmi_tlv_event_id in wmi.h.
 * When bumping kernel versions, re-check all event IDs with explicit or
 * non-contiguous values in the enum. */
#define WMI_UPDATE_STATS_EVENTID     0x1d004

/* PDEV group WMI commands used during DP init / hw mode setup */
#define WMI_PDEV_SET_PARAM_CMDID     (WMI_TLV_CMD(WMI_GRP_PDEV) + 2) /* 0x4003 */
#define WMI_PDEV_SET_HW_MODE_CMDID   (WMI_TLV_CMD(WMI_GRP_PDEV) + 0x1D) /* 0x401E */
#define WMI_PDEV_SET_HW_MODE_RESP_EVENTID (WMI_EVT_GRP_START_ID(WMI_GRP_PDEV) + 0xF) /* 0x4010 */

/* WMI TLV tag for PDEV_SET_HW_MODE response (enum wmi_tlv_tag) */
#define WMI_TAG_PDEV_SET_HW_MODE_RESPONSE_EVENT 518

/* WMI scan event types (wmi_scan_event_type) */
#define WMI_SCAN_EVENT_STARTED      BIT(0)
#define WMI_SCAN_EVENT_COMPLETED    BIT(1)
#define WMI_SCAN_EVENT_BSS_CHANNEL  BIT(2)
#define WMI_SCAN_EVENT_FOREIGN_CHAN BIT(3)
#define WMI_SCAN_REASON_COMPLETED   0x1

/* WMI TLV tags used by the connect path */
#define WMI_TAG_START_SCAN_CMD      77
#define WMI_TAG_SCAN_EVENT          36
#define WMI_TAG_MGMT_RX_HDR         44
#define WMI_TAG_ARRAY_BYTE          17
#define WMI_TAG_STATS_EVENT         70
#define WMI_TAG_PEER_ASSOC_CONF_EVENT 434
#define WMI_TAG_VDEV_INSTALL_KEY_COMPLETE_EVENT 42
#define WMI_TAG_MGMT_TX_SEND_CMD    422
#define WMI_TAG_MGMT_TX_COMPL_EVENT 423
#define WMI_TAG_VDEV_START_RESPONSE_EVENT 40
#define WMI_TAG_VDEV_DELETE_RESP_EVENT 450

/* HTT T2H message types (subset the model emits) */
#define HTT_T2H_MSG_TYPE_PEER_MAP   0x3  /* PEER_MAP v1 */
#define HTT_T2H_MSG_TYPE_PEER_MAP2  0x1e
#define HTT_T2H_MSG_TYPE_PEER_MAP3  0x2b
#define HTT_T2H_MSG_TYPE_VERSION_CONF 0x0  /* version req/conf share msg_type 0 */

/* HTT H2T message types the model handles */
#define HTT_H2T_MSG_TYPE_VERSION_REQ            0x0
#define HTT_H2T_MSG_TYPE_SRING_SETUP            0xb
#define HTT_H2T_MSG_TYPE_RX_RING_SELECTION_CFG  0xc

/* HTT SRNG ring IDs (sequential — no gaps, from kernel's enum htt_srng_ring_id) */
#define HTT_RXDMA_HOST_BUF_RING         0
#define HTT_RXDMA_MONITOR_STATUS_RING   1
#define HTT_RXDMA_MONITOR_BUF_RING      2
#define HTT_RXDMA_MONITOR_DESC_RING     3
#define HTT_RXDMA_MONITOR_DEST_RING     4
#define HTT_HOST1_TO_FW_RXBUF_RING      5
#define HTT_HOST2_TO_FW_RXBUF_RING      6
#define HTT_RXDMA_NON_MONITOR_DEST_RING 7
#define HTT_RXDMA_HOST_BUF_RING2        8

/* HTT SRING_SETUP message bitfields (mirror driver dp_htt.h) */
/* ring_type values */
#define HTT_HW_TO_SW_RING   0
#define HTT_SW_TO_HW_RING   1
#define HTT_SW_TO_SW_RING   2

#define HTT_SRNG_SETUP_INFO0_MSG_TYPE   0xff
#define HTT_SRNG_SETUP_INFO0_PDEV_ID    (0xff << 8)
#define HTT_SRNG_SETUP_INFO0_RING_ID    (0xff << 16)
#define HTT_SRNG_SETUP_INFO0_RING_TYPE  (0xff << 24)
#define HTT_SRNG_SETUP_INFO1_RING_SIZE  0xffff
#define HTT_SRNG_SETUP_INFO1_ENTRY_SIZE (0xff << 16)

/* SRING_SETUP message is 13 dwords (52 bytes) */
#define HTT_SRING_SETUP_MSG_LEN         52

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
#define WCN7850_POOL_PADDR         0x100000000ULL

/* CE destination register block offsets used by the UMAC window. */
#define WCN7850_CE_DST_BASE_LO     0x58
#define WCN7850_CE_DST_BASE_HI     0x5c
#define WCN7850_CE_DST_SIZE        0x60
#define WCN7850_CE_DST_MISC        0x64
#define WCN7850_CE_CFG_DST_HP      0x68
#define WCN7850_CE_CFG_DST_TP      0x6c
#define WCN7850_CE_CFG_DST_TP_HI   0x70
#define WCN7850_REO_HP_ADDR        0x3008
#define WCN7850_REO_TP_ADDR        0x3104

static const uint8_t wcn7850_supported_rates[8] = {
    0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c
};

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
    WCN7850_RING_REO_CMD,
    WCN7850_RING_REO_STATUS,
    WCN7850_RING_WBM_IDLE_LINK,
    WCN7850_RING_WBM2SW_RELEASE,
    WCN7850_RING_RXDMA_BUF,
    WCN7850_RING_SW2WBM_RELEASE,
    WCN7850_RING_UNKNOWN,
} Wcn7850RingType;

/* Per-ring tracked state */
/* Maximum tracked buffer addresses per RXDMA buf ring.
 * The driver replenishes in batches; this must exceed the max batch size. */
#define WCN7850_RXDMA_MAX_BUFS  2048

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
    uint64_t buf_addrs[WCN7850_RXDMA_MAX_BUFS];
    uint32_t buf_cookies[WCN7850_RXDMA_MAX_BUFS];
    uint32_t num_bufs;
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

#define WCN7850_MHI_SPECIAL_BASE 0x1e0e100
#define WCN7850_MAX_MHI_RING_SIZE (256 * sizeof(MhiTre))

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

    /* Opt-in verbose debug logging. Off by default; enable with the `dbg`
     * device property (-device wcn7850,dbg=on). Kept on stderr so it lands
     * in the stream the VM launcher already captures; the long-term QEMU
     * mechanism is trace events, this switch just keeps the model
     * debuggable without flooding every boot. */
    bool dbg;

    /* Buffer pool (64 MB carve-out emulated as RAM region) */
    MemoryRegion pool_mr;
    uint8_t *pool_mem;
    uint64_t pool_paddr;

    /* SRNG ring tracking */
    WCN7850RingState rings[WCN7850_NUM_RINGS_TRACKED];
    int num_rings;

    /* Stored HP_ADDR (RDP address) from UMAC window REO HP_ADDR_* register
     * writes.  The driver submits these during ath12k_wifi7_hal_srng_dst_hw_init
     * (wifi7/hal.c:~200) BEFORE the SRING_SETUP message arrives, so we cache
     * them here and consume them when the ring is created in the HTT handler. */
    uint64_t reo_hp_addr_full[WCN7850_NUM_REO_RINGS];
    uint32_t reo_hp_addr_lsb[WCN7850_NUM_REO_RINGS];

    /* Bounded backend RX queue; frames wait here for RXDMA/REO buffers. */
    PendingRxFrame rx_frames[WCN7850_PENDING_RX_FRAMES];
    uint32_t rx_head;
    uint32_t rx_count;

    /* TX packet processing */
    uint8_t *tx_buffer;

    /* Configurable MAC address */
    uint8_t mac_addr[6];

    /* CE poll timer */
    QEMUTimer *ce_poll_timer;

    /* Deferred CE events (retried when CE destination rings fill up). */
    DeferredCeEvent deferred_ce_events[WCN7850_PENDING_CE_EVENTS];
    QEMUTimer *deferred_ce_timer;
    uint32_t deferred_ce_head;
    uint32_t deferred_ce_count;

    /* Async scan event sequence. Real firmware paces the per-channel scan
     * events over tens of milliseconds; emitting them synchronously inside
     * the SCAN START handler collapses that to one microsecond burst, which
     * races the driver: the beacon (MGMT_RX) can arrive before mac80211 has
     * switched to the scan channel, so it is silently dropped and the BSS is
     * never recorded (flaky `iw scan`). We therefore emit each event from a
     * timer, one per tick, ~10ms apart in guest virtual time. */
    QEMUTimer *scan_seq_timer;
    int scan_seq_step;  /* 0..4: STARTED/FOREIGN_CHAN/beacon/BSS_CHANNEL/COMPLETED */
    bool scan_seq_ap;   /* whether the AP channel is part of this scan */

    /* ===== CE/HTC/WMI state ===== */

    /* CE doorbell delivery: true after QMI boot completes */
    bool ce_ready;

    /* HTC ready sent: set after we successfully send HTC ready to driver */
    bool htc_ready_sent;

    /* WMI_SERVICE_READY pending: set when WMI service connects, sent once
     * the driver has posted rx buffers on the WMI downlink CE (pipe 2). */
    bool wmi_service_ready_pending;

    /* CE source ring config and state */
    CeSrcRing ce_src[WCN7850_CE_COUNT];

    /* CE dest ring config and state */
    CeDstRing ce_dst[WCN7850_CE_COUNT];

    /* CE destination status ring config and state. */
    CeStsRing ce_sts[WCN7850_CE_COUNT];

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
    uint8_t sta_mac[6];       /* our own STA MAC (learned from mgmt TX SA) */
    bool sta_mac_valid;

    /* scan request details (so we can emit the matching scan events) */
    uint32_t scan_id;
    uint32_t scan_vdev_id;

} WCN7850State;

#define TYPE_WCN7850 "wcn7850"
DECLARE_INSTANCE_CHECKER(WCN7850State, WCN7850, TYPE_WCN7850)

/* Verbose model debug output (see WCN7850State.dbg). */
#define WCN_DBG(s, ...) \
    do { if ((s)->dbg) { fprintf(stderr, __VA_ARGS__); } } while (0)

/* Forward declarations */
static void wcn7850_process_ul_data(WCN7850State *s, PCIDevice *pci_dev,
                                     uint64_t ch_ctxt_addr, uint32_t ch_id,
                                     uint64_t er_ctxt_addr);
static bool wcn7850_mission_mode_setup(WCN7850State *s, PCIDevice *pci_dev);
static void wcn7850_process_tcl_data(WCN7850State *s, PCIDevice *pci_dev,
                                      WCN7850RingState *tcl);
static void wcn7850_handle_peer_create(WCN7850State *s, PCIDevice *pci_dev,
                                       const uint8_t *payload, uint32_t len);
static void wcn7850_handle_peer_assoc(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len);
static void wcn7850_handle_vdev_start(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len);
static void wcn7850_handle_scan_start(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len);
static void wcn7850_handle_mgmt_tx(WCN7850State *s, PCIDevice *pci_dev,
                                   const uint8_t *payload, uint32_t len);
static void wcn7850_handle_install_key(WCN7850State *s, PCIDevice *pci_dev,
                                        const uint8_t *payload, uint32_t len);
static void wcn7850_send_vdev_delete_resp(WCN7850State *s, PCIDevice *pci_dev,
                                           uint32_t vdev_id);
static void wcn7850_send_stats_resp(WCN7850State *s, PCIDevice *pci_dev,
                                    uint32_t stats_id, uint32_t vdev_id,
                                    uint32_t pdev_id);

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
        WCN_DBG(s, "WCN: post_event er=%u msivec=%u rp=0x%lx\n",
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

    WCN_DBG(s, "WCN: transfer_event ch=%u ptr=0x%lx len=%u er=%u\n",
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
 * Uses the event-ring index from the channel context. */
static bool wcn7850_write_dl_data(WCN7850State *s, PCIDevice *pci_dev,
                                   uint64_t ch_ctxt_addr,
                                   uint64_t er_ctxt_addr,
                                   const void *data, size_t data_len)
{
    MhiChanCtxt ctxt;
    MhiTre tre;

    WCN_DBG(s, "WCN: write_dl_data ch_ctxt=0x%lx\n",
            (unsigned long)ch_ctxt_addr);
    if (!wcn7850_read_chan_ctxt(pci_dev, ch_ctxt_addr, &ctxt)) {
        WCN_DBG(s, "WCN: write_dl_data read ctxt FAILED\n");
        return false;
    }

    WCN_DBG(s, "WCN: write_dl_data rbase=0x%lx rlen=0x%lx rp=0x%lx wp=0x%lx erindex=%u\n",
            (unsigned long)ctxt.rbase, (unsigned long)ctxt.rlen,
            (unsigned long)ctxt.rp, (unsigned long)ctxt.wp, ctxt.erindex);
    if (!ctxt.rbase || !ctxt.rlen) {
        WCN_DBG(s, "WCN: write_dl_data ring not set up\n");
        return false;
    }

    /* Validate rp/wp */
    if (ctxt.rp < ctxt.rbase || ctxt.rp >= ctxt.rbase + ctxt.rlen)
        ctxt.rp = ctxt.rbase;
    if (ctxt.wp < ctxt.rbase || ctxt.wp >= ctxt.rbase + ctxt.rlen)
        ctxt.wp = ctxt.rbase;

    /* Check if ring is empty */
    if (ctxt.rp == ctxt.wp) {
        WCN_DBG(s, "WCN: write_dl_data ring empty (rp==wp)\n");
        return false;
    }

    /* Read the next TRE at rp (this is the buffer the host pre-queued) */
    uint64_t tre_offset = ctxt.rp - ctxt.rbase;
    /* Host MHI expects the transfer-event ptr to be the TRE's ring address
     * (so it can recover the buffer from the TRE), not the data buffer. */
    uint64_t tre_addr = ctxt.rp;
    if (!wcn7850_read_tre(pci_dev, ctxt.rbase, tre_offset, &tre)) {
        WCN_DBG(s, "WCN: write_dl_data read TRE FAILED\n");
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

typedef struct {
    uint32_t base_lsb;
    uint32_t base_msb;
    uint32_t ring_id;
    uint32_t misc;
    uint32_t msi;
    uint32_t hp;
    uint32_t tp;
    uint8_t default_msivec;
} Wcn7850RingRegisters;

static Wcn7850RingType wcn7850_reo_ring_type(int ring_id)
{
    return ring_id == 0 ? WCN7850_RING_REO_EXCEPTION : WCN7850_RING_REO_DST;
}

static void wcn7850_update_ring_cfg(WCN7850State *s, Wcn7850RingType type, int n,
                                    const char *name,
                                    const Wcn7850RingRegisters *regs);

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
/* Single parameterized SRNG ring-config updater. Replaces the three
 * near-identical TCL/REO/WBM handlers. */
static void wcn7850_update_ring_cfg(WCN7850State *s, Wcn7850RingType type, int n,
                                    const char *name,
                                    const Wcn7850RingRegisters *regs)
{
    uint32_t base_lsb, base_msb, ring_id_reg, misc;
    WCN7850RingState *r = wcn7850_add_or_update_ring(s, type, n);
    if (!r) return;

    base_lsb = *(uint32_t *)(s->bar0_always_on + regs->base_lsb);
    base_msb = *(uint32_t *)(s->bar0_always_on + regs->base_msb);
    ring_id_reg = *(uint32_t *)(s->bar0_always_on + regs->ring_id);
    misc = *(uint32_t *)(s->bar0_always_on + regs->misc);

    r->base_addr = ((uint64_t)(base_msb & 0xff) << 32) | base_lsb;
    r->size = ((base_msb >> 8) & 0xfffff) * 4;
    r->entry_size = (ring_id_reg & 0xff) * 4;
    r->hp_mmio_offset = regs->hp;
    r->tp_mmio_offset = regs->tp;
    r->hp_is_mmio = true;
    r->tp_is_mmio = true;
    r->enable = (misc & BIT(6)) != 0;
    r->configured = (r->base_addr != 0 && r->entry_size > 0 && r->size > 0);

    if (r->configured) {
        uint32_t msi_data = regs->msi ?
            *(uint32_t *)(s->bar0_always_on + regs->msi) : 0;
        r->msivec = msi_data & (WCN7850_MSI_VECTORS - 1);
        if (!r->msivec) {
            r->msivec = regs->default_msivec;
        }
        WCN_DBG(s, "WCN: %s ring %d base=0x%lx size=%u entry=%u enable=%d msivec=%u\n",
                name, n, (unsigned long)r->base_addr, r->size, r->entry_size,
                r->enable, r->msivec);
    }
}

/* ===== NetClient callbacks ===== */

static bool wcn7850_nc_can_receive(NetClientState *nc)
{
    WCN7850State *s = qemu_get_nic_opaque(nc);
    WCN7850RingState *reo = wcn7850_find_ring(s, WCN7850_RING_REO_DST, 1);
    /* Keep bounded backpressure while allowing bursts from the backend. */
    bool ok = reo && reo->configured && reo->enable &&
              s->rx_count < WCN7850_PENDING_RX_FRAMES;
    if (!ok && s->dbg)
        WCN_DBG(s, "WCN: can_receive=false rx_count=%u reo=%d/%d\n",
                s->rx_count, reo ? reo->configured : -1,
                reo ? reo->enable : -1);
    return ok;
}

static ssize_t wcn7850_nc_receive_iov(NetClientState *nc, const struct iovec *iov,
                                       int iovcnt)
{
    WCN7850State *s = qemu_get_nic_opaque(nc);
    int i;
    size_t total = 0;

    for (i = 0; i < iovcnt; i++) {
        total += iov[i].iov_len;
    }

    /* Slirp ARPs for the Wi-Fi address before forwarding host TCP.  Answer
     * that link-layer query on behalf of the emulated station so the real
     * guest TCP endpoint can receive the subsequent SYN. */
    if (total >= 42) {
        uint8_t arp[42];
        uint8_t frame[64] = { 0 };
        uint8_t packet[64] = { 0 };
        size_t off = 0;
        for (i = 0; i < iovcnt; i++) {
            size_t n = MIN(iov[i].iov_len, sizeof(packet) - off);
            memcpy(packet + off, iov[i].iov_base, n);
            off += n;
        }
        uint32_t target_ip = ((uint32_t)packet[38] << 24) |
                             ((uint32_t)packet[39] << 16) |
                             ((uint32_t)packet[40] << 8) | packet[41];
        if (packet[12] == 0x08 && packet[13] == 0x06 &&
            packet[20] == 0 && packet[21] == 1 &&
            target_ip == WCN7850_STATION_IPV4) {
            memcpy(arp, packet, 6);
            memcpy(arp + 6, s->mac_addr, 6);
            arp[12] = 0x08; arp[13] = 0x06;
            arp[14] = 0; arp[15] = 1;
            arp[16] = 0x08; arp[17] = 0;
            arp[18] = 6; arp[19] = 4;
            arp[20] = 0; arp[21] = 2;
            memcpy(arp + 22, s->mac_addr, 6);
            arp[28] = (WCN7850_STATION_IPV4 >> 24) & 0xff;
            arp[29] = (WCN7850_STATION_IPV4 >> 16) & 0xff;
            arp[30] = (WCN7850_STATION_IPV4 >> 8) & 0xff;
            arp[31] = WCN7850_STATION_IPV4 & 0xff;
            memcpy(arp + 32, packet + 6, 6);
            memcpy(arp + 38, packet + 28, 4);
            memcpy(frame, arp, sizeof(arp));
            qemu_send_packet(nc, frame, sizeof(frame));
            WCN_DBG(s, "WCN: answered ARP for emulated station\n");
        }
    }

    if (total > 65535) {
        return total;
    }

    uint32_t rx_idx = (s->rx_head + s->rx_count) % WCN7850_PENDING_RX_FRAMES;
    s->rx_frames[rx_idx].len = total;
    size_t offset = 0;
    for (i = 0; i < iovcnt; i++) {
        memcpy(s->rx_frames[rx_idx].data + offset,
               iov[i].iov_base, iov[i].iov_len);
        offset += iov[i].iov_len;
    }
    s->rx_count++;

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

/* ===== TX data path ===== */

/* TX checksum offload, IPv6 L4 (TCP/UDP) variant.
 *
 * The guest requests offload via HAL_TCL_DATA_CMD_INFO2_*_CKSUM_EN and
 * leaves the checksum fields zero; we must fill them or slirp drops the
 * frame. QEMU's net_checksum_calculate() handles IPv4 (including correct
 * ihl and VLAN) but bails out on IPv6, so do the IPv6 pseudo-header sum
 * here (RFC 8200): src(16) + dst(16) + upper-layer len(4) + zero(3) +
 * next-header(1), followed by the segment.
 *
 * `f` points at an ethernet frame, `len` its length.
 */
static void wcn7850_fix_ipv6_l4_checksum(uint8_t *f, size_t len)
{
    static const size_t ETH_HDR = 14;
    static const size_t IP6_HDR = 40;

    if (len < ETH_HDR + IP6_HDR + 8) {
        return;
    }
    const uint8_t *ip6 = f + ETH_HDR;
    uint8_t next_hdr = ip6[6];
    if (next_hdr != 0x06 && next_hdr != 0x11) { /* TCP / UDP only */
        return;
    }
    uint32_t ul_len = ((uint32_t)ip6[4] << 8) | ip6[5];
    if (ETH_HDR + IP6_HDR + ul_len > len) {
        return; /* truncated frame */
    }
    if (next_hdr == 0x06 && ul_len < 20) {
        return;
    }
    if (next_hdr == 0x11) {
        if (ul_len < 8) {
            return;
        }
        /* IPv6 UDP with checksum explicitly disabled stays zero */
        if (f[ETH_HDR + IP6_HDR + 6] == 0 && f[ETH_HDR + IP6_HDR + 7] == 0) {
            return;
        }
    }
    uint8_t *t = f + ETH_HDR + IP6_HDR;
    size_t csum_off = next_hdr == 0x11 ? 6 : 16; /* UDP / TCP checksum */
    uint32_t sum = net_checksum_add(32, (uint8_t *)ip6); /* src + dst */
    sum += next_hdr + ul_len;
    sum += net_checksum_add(ul_len, t);
    uint16_t ck = net_checksum_finish(sum);
    t[csum_off] = ck >> 8;
    t[csum_off + 1] = ck & 0xff;
}

static void wcn7850_process_tcl_data(WCN7850State *s, PCIDevice *pci_dev,
                                       WCN7850RingState *tcl)
{
    uint32_t entries, i;
    uint32_t entry_size = tcl->entry_size ?: 32;
    uint32_t tp = tcl->tp;
    uint32_t hp = tcl->hp;
    uint32_t ring_size = tcl->size ?: (512 * entry_size);
    static const int tcl_to_wbm[] = { 0, 2, 4 };
    int wbm_idx = (tcl->ring_id < 3) ? tcl_to_wbm[tcl->ring_id] : 0;
    int processed = 0;
    int wbm_posted = 0;
    WCN7850RingState *wbm_ring = NULL;

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

    if (entries > 64) entries = 64;

    for (i = 0; i < entries; i++) {
        uint64_t desc_addr = tcl->base_addr + tp;
        uint8_t desc[32];
        uint64_t buf_addr;
        uint32_t buf_len;
        uint32_t tcl_info1;
        uint32_t tcl_info2;

        if (pci_dma_read(pci_dev, desc_addr, desc, sizeof(desc)) != MEMTX_OK) {
            break;
        }

        buf_addr = (uint64_t)le32_to_cpu(*(uint32_t *)desc) |
                   ((uint64_t)(le32_to_cpu(*(uint32_t *)(desc + 4)) & 0xff) << 32);
        tcl_info2 = le32_to_cpu(*(uint32_t *)(desc + 16));
        buf_len = tcl_info2 & 0xffff;   /* HAL_TCL_DATA_CMD_INFO2_DATA_LEN */
        tcl_info1 = le32_to_cpu(*(uint32_t *)(desc + 4));

        if (!buf_addr || !buf_len) {
            tp += entry_size;
            if (tp >= ring_size) tp = 0;
            continue;
        }

        if (buf_len > 65535) buf_len = 65535;

        if (!s->tx_buffer) {
            s->tx_buffer = g_malloc(65536);
        }

        if (pci_dma_read(pci_dev, buf_addr, s->tx_buffer,
                         buf_len) != MEMTX_OK) {
            tp += entry_size;
            if (tp >= ring_size) tp = 0;
            continue;
        }

        if (s->nic) {
            NetClientState *nc = qemu_get_queue(s->nic);
            if (nc->peer) {
                uint8_t ethernet[65536];
                size_t ethernet_len = buf_len;
                uint8_t *frame = s->tx_buffer;

                if (buf_len >= 32 &&
                    (le16_to_cpu(*(uint16_t *)frame) & 0x000c) == 0x0008) {
                    size_t hdr_len = 24;
                    uint16_t fc = le16_to_cpu(*(uint16_t *)frame);
                    if (fc & 0x0080)
                        hdr_len += 2;
                    if (buf_len >= hdr_len + 8 &&
                        frame[hdr_len] == 0xaa && frame[hdr_len + 1] == 0xaa &&
                        frame[hdr_len + 2] == 0x03) {
                        ethernet_len = buf_len - hdr_len - 8 + 14;
                        memcpy(ethernet, frame + 16, 6);
                        memcpy(ethernet + 6, frame + 10, 6);
                        memcpy(ethernet + 12, frame + hdr_len + 6, 2);
                        memcpy(ethernet + 14, frame + hdr_len + 8,
                               ethernet_len - 14);
                        frame = ethernet;
                    }
                }
                WCN_DBG(s, "WCN: TX send len=%zu dst=%02x:%02x:%02x:%02x:%02x:%02x proto=%04x\n",
                         ethernet_len, frame[0], frame[1], frame[2],
                         frame[3], frame[4], frame[5],
                         ethernet_len >= 14 ? le16_to_cpu(*(uint16_t *)(frame + 12)) : 0);
                /* TX checksum offload: for CHECKSUM_PARTIAL frames the guest
                 * sets the HAL_TCL_DATA_CMD_INFO2_*_CKSUM_EN bits and leaves
                 * the checksum fields zero; we must fill them or slirp drops
                 * the frame. Only frames carrying the offload bits are
                 * touched. */
                if (tcl_info2 & (BIT(16) | BIT(17) | BIT(18) | BIT(19) |
                                 BIT(20))) {
                    if (ethernet_len >= 34 && frame[12] == 0x08 &&
                        frame[13] == 0x00) {
                        uint32_t csum_flag = 0;
                        if (tcl_info2 & BIT(16)) {
                            csum_flag |= CSUM_IP;
                        }
                        if (tcl_info2 & BIT(19)) {
                            csum_flag |= CSUM_TCP;
                        }
                        if (tcl_info2 & BIT(17)) {
                            csum_flag |= CSUM_UDP;
                        }
                        net_checksum_calculate(frame, ethernet_len, csum_flag);
                    } else if (ethernet_len >= 34 && frame[12] == 0x86 &&
                               frame[13] == 0xdd) {
                        if (tcl_info2 & (BIT(18) | BIT(20))) {
                            wcn7850_fix_ipv6_l4_checksum(frame, ethernet_len);
                        }
                    }
                }
                qemu_send_packet(nc, frame, ethernet_len);
            } else {
                WCN_DBG(s, "WCN: TX drop (no peer) len=%u\n", (unsigned)buf_len);
            }
        }

        /* Post TX completion to the WBM2SW release ring */
        {
            WCN7850RingState *wbm = wcn7850_find_ring(s,
                WCN7850_RING_WBM2SW_RELEASE, wbm_idx);
            if (wbm) {
                uint32_t esize = wbm->entry_size;
                uint32_t whp = wbm->hp;
                uint32_t wtp = wbm->tp;
                uint32_t wsz = wbm->size;
                uint32_t next = whp + esize;
                if (next >= wsz) next = 0;
                if (next != wtp) {
                    /* Build a HAL WBM release-ring entry (struct
                     * hal_wbm_release_ring) from the TCL descriptor, so the
                     * driver can recover its desc_id via the SW_COOKIE it
                     * placed in buf_addr_info.info1 (BUFFER_ADDR_INFO1_SW_COOKIE).
                     * REL_SRC_MODULE=TQM(0), TQM_RELEASE_REASON=FRAME_ACKED(0),
                     * FIRST_MSDU|LAST_MSDU for a single-MSDU transmit. */
                    uint32_t buf_va_lo = le32_to_cpu(*(uint32_t *)desc);
                    uint32_t rel_src_tqm = 0;
                    uint32_t rel_reason_acked = 0;
                    uint32_t first_msdu = 1u << 8;
                    uint32_t last_msdu = 1u << 9;
                    uint8_t comp[32] = {0};
                    uint32_t *comp32 = (uint32_t *)comp;
                    comp32[0] = cpu_to_le32(buf_va_lo);
                    comp32[1] = cpu_to_le32(tcl_info1);
                    comp32[2] = cpu_to_le32(rel_src_tqm | rel_reason_acked);
                    comp32[3] = 0;
                    comp32[4] = cpu_to_le32(first_msdu | last_msdu);
                    comp32[5] = 0;
                    comp32[6] = 0;
                    comp32[7] = cpu_to_le32((uint32_t)s->peer_id);
                    if (pci_dma_write(pci_dev, wbm->base_addr + whp,
                                      comp, sizeof(comp)) == MEMTX_OK) {
                        wbm->hp = next;
                        wbm_posted++;
                        wbm_ring = wbm;
                    }
                }
            }
        }

        processed++;
        tp += entry_size;
        if (tp >= ring_size) tp = 0;
    }

    /* Flush WBM HP once for the whole batch. UMAC SRNG pointers are dword
     * offsets; publish the byte HP divided by 4 so the driver compares it
     * against its dword-units TP. */
    if (wbm_posted > 0 && wbm_ring) {
        uint32_t hp_dw = wbm_ring->hp / 4;
        uint32_t hp_le = cpu_to_le32(hp_dw);
        if (wbm_ring->hp_shadow_addr) {
            pci_dma_write(pci_dev, wbm_ring->hp_shadow_addr, &hp_le, sizeof(hp_le));
        }
        *(uint32_t *)(s->bar0_always_on + wbm_ring->hp_mmio_offset) = hp_dw;
        if (wbm_ring->msivec < WCN7850_MSI_VECTORS) {
            msi_notify(pci_dev, wbm_ring->msivec);
        }
    }

    tcl->tp = tp;
    *(uint32_t *)(s->bar0_always_on + tcl->tp_mmio_offset) = tp / 4;

    if (processed > 0) {
        WCN_DBG(s, "WCN: TX ring=%u processed=%u wbm=%u\n",
                tcl->ring_id, processed, wbm_posted);
    }
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

static bool wcn7850_ce_deliver(WCN7850State *s, PCIDevice *pci_dev,
                               int ce_pipe, const void *data,
                               uint32_t data_len)
{
    uint64_t base = s->ce_dst[ce_pipe].base;
    uint32_t esize = s->ce_dst[ce_pipe].entry_size ?: 16;
    uint32_t ring_sz = s->ce_dst[ce_pipe].size ?: (WCN7850_CE_DST_RING_SIZE * esize);
    if (!base || !ring_sz) {
        return false;
    }
    if (!s->ce_dst[ce_pipe].pending) {
        return false;
    }
    uint32_t sts_esize = s->ce_sts[ce_pipe].entry_size ?: 16;
    if (s->ce_sts[ce_pipe].base &&
        (sts_esize < sizeof(uint32_t) || sts_esize > 4096)) {
        return false;
    }

    /* We don't reliably track the driver's posted-buffer count (the driver
     * assigns CE shadow indices dynamically), so instead of gating on a
     * pending counter we read the descriptor at the current consumer slot:
     * if the driver hasn't posted a buffer there yet (buf_addr == 0) we
     * simply return and the caller retries later. */

    /* Read dest ring entry at current consumer position to get buffer address */
    uint64_t desc_addr = base + s->ce_dst[ce_pipe].cons;
    struct {
        uint32_t buf_addr_low;
        uint32_t buf_addr_info;
    } desc;
    if (pci_dma_read(pci_dev, desc_addr, &desc, sizeof(desc)) != MEMTX_OK)
        return false;

    uint64_t buf_addr = (uint64_t)desc.buf_addr_low |
                         ((uint64_t)(desc.buf_addr_info & 0xff) << 32);
      if (!buf_addr) {
          return false;
     }

     if (data_len > 2048)
         data_len = 2048;

     /* Write WMI data into the buffer */
    {
        const uint32_t *data32 = (const uint32_t *)data;
        const uint32_t *wmihdr = (const uint32_t *)((const uint8_t *)data + 8);
        WCN_DBG(s, "WCN: SEND pipe=%d len=%u cons=%u buf=0x%lx htc=0x%08x wmi=0x%08x\n",
                ce_pipe, data_len, s->ce_dst[ce_pipe].cons, (unsigned long)buf_addr,
                data32[0], wmihdr[0]);
    }
    if (pci_dma_write(pci_dev, buf_addr, data, data_len) != MEMTX_OK)
        return false;
    s->ce_dst[ce_pipe].cons += esize;
    if (s->ce_dst[ce_pipe].cons >= ring_sz)
        s->ce_dst[ce_pipe].cons = 0;
    if (s->ce_dst[ce_pipe].pending)
        s->ce_dst[ce_pipe].pending--;

    /* Advance the ring's tail pointer so the driver sees free RX-buffer
     * space. For a CE DEST ring the driver posts empty buffers (producer)
     * and the device (us) consumes them. The driver's
     * ath12k_hal_srng_src_num_free() computes free entries from
     * tp = *srng->u.src_ring.tp_addr (the pointer the *device* writes) minus
     * hp (the driver's local producer position). If we never write our
     * consume position back to hp_addr (== the DST ring's TP/RDP register),
     * tp stays at 0 and the ring looks permanently full, so
     * ath12k_ce_rx_post_pipe() returns -ENOSPC ("failed to enqueue rx buf"),
     * starving subsequent WMI RX deliveries. The HAL keeps this pointer in
     * byte units (ath12k_hal_srng_src_num_free reads tp_addr raw, and
     * srng->u.src_ring.hp is initialised to 0). Write cons (bytes) back. */
    {
        uint32_t hp_bytes = s->ce_dst[ce_pipe].cons;
        s->ce_dst[ce_pipe].hp = hp_bytes;
        if (s->ce_dst[ce_pipe].hp_addr) {
            uint32_t hp_le = cpu_to_le32(hp_bytes);
            WCN_DBG(s, "WCN: write dst hp pipe=%d addr=0x%lx val=%u\n",
                    ce_pipe, (unsigned long)s->ce_dst[ce_pipe].hp_addr, hp_bytes);
            pci_dma_write(pci_dev, s->ce_dst[ce_pipe].hp_addr,
                          &hp_le, sizeof(hp_le));
        } else {
            WCN_DBG(s, "WCN: dst hp_addr=0 pipe=%d cons=%u (writeback SKIPPED)\n",
                    ce_pipe, hp_bytes);
        }
    }

    /* Write CE dst status descriptor and advance STATUS HP */
    if (s->ce_sts[ce_pipe].base) {
        uint32_t sts_hp = s->ce_sts[ce_pipe].hp;
        uint8_t *sts_desc;
        sts_desc = g_malloc0(sts_esize);
        *(uint32_t *)sts_desc = cpu_to_le32(data_len << 16);
        WCN_DBG(s, "WCN: write sts_desc pipe=%d at 0x%lx len=%u hp=%u\n",
                 ce_pipe, (unsigned long)(s->ce_sts[ce_pipe].base + sts_hp),
                 data_len, sts_hp);
        pci_dma_write(pci_dev, s->ce_sts[ce_pipe].base + sts_hp,
                      sts_desc, sts_esize);
        if (s->dbg && sts_esize >= 16) {
            uint32_t w0, w1, w2, w3;
            pci_dma_read(pci_dev, s->ce_sts[ce_pipe].base + sts_hp, &w0, 4);
            pci_dma_read(pci_dev, s->ce_sts[ce_pipe].base + sts_hp + 4, &w1, 4);
            pci_dma_read(pci_dev, s->ce_sts[ce_pipe].base + sts_hp + 8, &w2, 4);
            pci_dma_read(pci_dev, s->ce_sts[ce_pipe].base + sts_hp + 12, &w3, 4);
            WCN_DBG(s, "WCN: sts_desc written: 0x%08x 0x%08x 0x%08x 0x%08x\n",
                    w0, w1, w2, w3);
        }
        g_free(sts_desc);
        sts_hp += sts_esize;
        if (s->ce_sts[ce_pipe].size &&
            sts_hp >= s->ce_sts[ce_pipe].size)
            sts_hp = 0;
        s->ce_sts[ce_pipe].hp = sts_hp;

        /* RDP write-back: driver reads status HP from the RDP slot, but the
         * ath12k HAL treats cached_hp/tp/entry_size/ring_size in units of
         * 4-byte WORDS (see hal.c:596 ath12k_hal_srng_setup).  sts_hp is kept
         * in bytes for the ring offset, so convert to words here. */
        if (s->ce_sts[ce_pipe].hp_addr) {
            uint32_t sts_hp_words = (sts_hp / sts_esize) * (sts_esize / 4);
            uint32_t sts_hp_le = cpu_to_le32(sts_hp_words);
            WCN_DBG(s, "WCN: write sts_hp pipe=%d to 0x%lx val=%u (bytes %u)\n",
                     ce_pipe, (unsigned long)s->ce_sts[ce_pipe].hp_addr,
                     sts_hp_words, sts_hp);
            pci_dma_write(pci_dev, s->ce_sts[ce_pipe].hp_addr,
                          &sts_hp_le, sizeof(sts_hp_le));
            if (s->dbg) {
                uint32_t slot, sd[8];
                int i;
                pci_dma_read(pci_dev, s->ce_sts[ce_pipe].hp_addr, &slot, 4);
                WCN_DBG(s, "WCN: rdp slot now=0x%x (ce_sts_hp_words=%u, bytes=%u)\n",
                        slot, sts_hp_words, sts_hp);
                for (i = 0; i < 8; i++) {
                    pci_dma_read(pci_dev, s->ce_sts[ce_pipe].base + i * 16,
                                 &sd[i], 4);
                }
                WCN_DBG(s, "WCN: CE%d status ring[0..7]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                        ce_pipe, sd[0], sd[1], sd[2], sd[3], sd[4], sd[5], sd[6], sd[7]);
            }
            if (s->dbg && ce_pipe == 1) {
                uint32_t s2slot, s2d[2];
                pci_dma_read(pci_dev, s->ce_sts[2].hp_addr, &s2slot, 4);
                pci_dma_read(pci_dev, s->ce_sts[2].base, &s2d[0], 4);
                pci_dma_read(pci_dev, s->ce_sts[2].base + 16, &s2d[1], 4);
                WCN_DBG(s, "WCN: CE2 sts_hp slot=0x%x base=0x%lx entry0=0x%08x entry1=0x%08x\n",
                        s2slot, (unsigned long)s->ce_sts[2].base, s2d[0], s2d[1]);
            }
        }
    }

    /* Send MSI to notify the driver (vector derived from the CE pipe,
     * matching the ath12k driver's ath12k_pci_get_ce_msi_idx mapping) */
    {
        int msivec = wcn7850_ce_msi_vector(ce_pipe);
        WCN_DBG(s, "WCN: sending MSI %d for CE pipe %d\n", msivec, ce_pipe);
        msi_notify(pci_dev, msivec);
    }
    return true;
}

static void wcn7850_ce_send(WCN7850State *s, PCIDevice *pci_dev,
                            int ce_pipe, const void *data, uint32_t data_len)
{
    if (wcn7850_ce_deliver(s, pci_dev, ce_pipe, data, data_len)) {
        return;
    }
    if (s->deferred_ce_count >= WCN7850_PENDING_CE_EVENTS) {
        WCN_DBG(s, "WCN: drop deferred CE event pipe=%d (queue full)\n",
                ce_pipe);
        return;
    }
    uint32_t event_idx = (s->deferred_ce_head + s->deferred_ce_count) %
                         WCN7850_PENDING_CE_EVENTS;
    s->deferred_ce_events[event_idx].pipe = ce_pipe;
    s->deferred_ce_events[event_idx].len = MIN(data_len,
        (uint32_t)sizeof(s->deferred_ce_events[event_idx].data));
    memcpy(s->deferred_ce_events[event_idx].data, data,
           s->deferred_ce_events[event_idx].len);
    s->deferred_ce_count++;
    timer_mod(s->deferred_ce_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
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
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total_len);
}

/* Build and send a WMI_SERVICE_READY_EVENT to the driver */
static void wcn7850_send_wmi_service_ready_ext(WCN7850State *s, PCIDevice *pci_dev);

/* Send WMI_SERVICE_AVAILABLE_EVENT (0x3) so the driver populates
 * svc_map for TLV services 128-255 (including
 * WMI_TLV_SERVICE_MBSS_PARAM_IN_VDEV_START_SUPPORT = 253).
 * Without this, scan links get -ENOLINK. */
static void wcn7850_send_wmi_service_available(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[256];
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
    /* Advertise service 220 (EXT2 message support) and service 461
     * (ETH_OFFLOAD) in the extended service segment. */
    ev->wmi_service_segment_bitmap[2] |= cpu_to_le32(BIT(28));
    ev->wmi_service_segment_bitmap[3] = cpu_to_le32(BIT(29));
    ev->wmi_service_segment_bitmap[3] |= cpu_to_le32(BIT(13));
    off += sizeof(*ev);

    /* EXT2 services start at service 256.  Service 461 is word 6, bit 13. */
    tlv = (WmiTlv *)(buf + off);
    tlv->header = WMI_TLV_HDR(WMI_TAG_ARRAY_UINT32, 7 * sizeof(uint32_t));
    off += sizeof(*tlv);
    memset(buf + off, 0, 7 * sizeof(uint32_t));
    ((uint32_t *)(buf + off))[6] = cpu_to_le32(BIT(13));
    off += 7 * sizeof(uint32_t);

    uint32_t total_len = off;

    uint8_t htc_buf[256];
    HtcHdr *hdr2 = (HtcHdr *)htc_buf;
    hdr2->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    hdr2->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*hdr2), buf, total_len);

    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*hdr2) + total_len);
    WCN_DBG(s, "WCN: sent WMI_SERVICE_AVAILABLE (0x3) for bit 253 cons=%u\n",
            s->ce_dst[2].cons);
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

    WCN_DBG(s, "WCN: WMI_SERVICE_READY total_len=%u\n", total_len);
    for (int i = 0; i < (int)total_len && i < 144; i += 4) {
        WCN_DBG(s, "WCN:  wmi_rdy[%02d] 0x%08x\n", i,
                le32_to_cpu(*(uint32_t *)(buf + i)));
    }

    /* Wrap in HTC header for EP 2 (WMI RX) */
    uint8_t htc_buf[2048];
    HtcHdr *hdr2 = (HtcHdr *)htc_buf;
    hdr2->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    hdr2->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*hdr2), buf, total_len);

    wcn7850_ce_send(s, pci_dev, 2, /* WMI DL pipe */
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

    WCN_DBG(s, "WCN: WMI_SERVICE_READY_EXT total_len=%u\n", total_len);
    for (int i = 0; i < (int)total_len && i < 120; i += 4) {
        WCN_DBG(s, "WCN:  wmi_rdy_ext[%02d] 0x%08x\n", i,
                le32_to_cpu(*(uint32_t *)(buf + i)));
    }

    uint8_t htc_buf[2048];
    HtcHdr *h2 = (HtcHdr *)htc_buf;
    h2->hdr_info = cpu_to_le32(HTC_EP_WMI | (total_len << 16));
    h2->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h2), buf, total_len);

    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h2) + total_len);
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

    WCN_DBG(s, "WCN: calling wcn7850_ce_send for WMI ready len=%zu cons=%u\n",
            sizeof(*h2) + total_len, s->ce_dst[2].cons);
    wcn7850_ce_send(s, pci_dev, 2, /* WMI DL pipe */
                            htc_buf, sizeof(*h2) + total_len);
    s->wmi_ready_sent = true;
    WCN_DBG(s, "WCN: wmi_ready_sent=true, cons now=%u\n", s->ce_dst[2].cons);
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
    wcn7850_ce_send(s, pci_dev, 1, buf, sizeof(HtcHdr) + sizeof(uint32_t) * 2);
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
        WCN_DBG(s, "WCN: HTC connect unknown svc_id=0x%04x\n", svc_id);
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

    WCN_DBG(s, "WCN: HTC connect svc=0x%04x -> ep=%d ul=%d dl=%d\n",
            svc_id, ep, ul_pipe, dl_pipe);

    /* Send response via CE pipe 1 (HTC control DL) */
    wcn7850_ce_send(s, pci_dev, 1, resp_buf, sizeof(HtcHdr) + sizeof(*resp));
}

/* Respond to WMI_PDEV_SET_HW_MODE_CMDID with a minimal success event.
 * The driver sends this after WMI_INIT to select DBS mode; we just
 * acknowledge to let DP init proceed (which writes UMAC REO registers). */
static void wcn7850_send_set_hw_mode_resp(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t buf[64];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t off = 0;
    hdr->cmd_id = cpu_to_le32(WMI_PDEV_SET_HW_MODE_RESP_EVENTID);
    off += sizeof(*hdr);

    /* WMI_TAG_PDEV_SET_HW_MODE_RESPONSE_EVENT TLV:
     *   struct wmi_pdev_set_hw_mode_response {
     *       uint32_t tlv_header;
     *       uint32_t status;
     *       uint32_t num_vdev_mac_entries;
     *   } */
    uint32_t resp_tlv_start = off;
    uint32_t *resp_tlv_hdr = (uint32_t *)(buf + off);
    off += sizeof(*resp_tlv_hdr);
    uint32_t status = 0;  /* success */
    uint32_t num_vdev_mac_entries = 0;
    memcpy(buf + off, &status, sizeof(status));
    off += sizeof(status);
    memcpy(buf + off, &num_vdev_mac_entries, sizeof(num_vdev_mac_entries));
    off += sizeof(num_vdev_mac_entries);
    *resp_tlv_hdr = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_PDEV_SET_HW_MODE_RESPONSE_EVENT,
                                             off - (resp_tlv_start + sizeof(*resp_tlv_hdr))));

    /* WMI_TAG_ARRAY_STRUCT with zero VDEV-MAC entries */
    WmiTlv *arr_tlv = (WmiTlv *)(buf + off);
    off += sizeof(*arr_tlv);
    arr_tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_ARRAY_STRUCT, 0));

    uint32_t total = off;
    uint8_t htc_buf[128];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    WCN_DBG(s, "WCN: WMI SET_HW_MODE resp (status=0)\n");
}

/* Handle WMI command from driver (sent via HTC EP WMI_TX) */
static void wcn7850_handle_wmi_cmd(WCN7850State *s, PCIDevice *pci_dev,
                                    const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(WmiCmdHdr)) {
        WCN_DBG(s, "WCN: WMI cmd too short (%u)\n", len);
        return;
    }
    const WmiCmdHdr *hdr = (const WmiCmdHdr *)payload;
    uint32_t cmd_id = le32_to_cpu(hdr->cmd_id) & 0xffffff;
    WCN_DBG(s, "WCN: WMI cmd 0x%x len=%u\n", cmd_id, len);

    switch (cmd_id) {
    case WMI_INIT_CMDID:
        WCN_DBG(s, "WCN: WMI INIT cmd\n");
        wcn7850_send_wmi_ready(s, pci_dev);
        break;
    case WMI_VDEV_CREATE_CMDID:
        WCN_DBG(s, "WCN: WMI VDEV CREATE cmd\n");
        /* Fire-and-forget: no response expected by the driver. */
        break;
    case WMI_VDEV_DELETE_CMDID: {
        uint32_t vdev_id = 0;
        if (len >= sizeof(WmiCmdHdr) + 8) {
            const uint8_t *body = payload + sizeof(WmiCmdHdr);
            vdev_id = le32_to_cpu(*(const uint32_t *)(body + 4));
        }
        WCN_DBG(s, "WCN: WMI VDEV DELETE cmd vdev=%u\n", vdev_id);
        wcn7850_send_vdev_delete_resp(s, pci_dev, vdev_id);
        break;
    }
    case WMI_REQUEST_STATS_CMDID: {
        uint32_t stats_id = 0, vdev_id = 0, pdev_id = 0;
        /* Wire: WmiCmdHdr(4) + tlv_header(4) + struct wmi_request_stats_cmd
         *   { tlv_header(4), stats_id(4), vdev_id(4),
         *     peer_macaddr(8), pdev_id(4) } */
        if (len >= sizeof(WmiCmdHdr) + 24) {
            const uint8_t *body = payload + sizeof(WmiCmdHdr);
            stats_id = le32_to_cpu(*(const uint32_t *)(body + 4));
            vdev_id  = le32_to_cpu(*(const uint32_t *)(body + 8));
            pdev_id  = le32_to_cpu(*(const uint32_t *)(body + 20));
        }
        WCN_DBG(s, "WCN: WMI REQUEST STATS cmd stats_id=0x%x vdev=%u pdev=%u\n",
                 stats_id, vdev_id, pdev_id);
        wcn7850_send_stats_resp(s, pci_dev, stats_id, vdev_id, pdev_id);
        break;
    }
    case WMI_VDEV_UP_CMDID:
        WCN_DBG(s, "WCN: WMI VDEV UP cmd\n");
        /* Fire-and-forget. */
        break;
    case WMI_VDEV_START_REQUEST_CMDID:
        wcn7850_handle_vdev_start(s, pci_dev, payload, len);
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
        WCN_DBG(s, "WCN: WMI SCAN CHAN LIST cmd (ignored)\n");
        break;
    case WMI_MGMT_TX_SEND_CMDID:
        wcn7850_handle_mgmt_tx(s, pci_dev, payload, len);
        break;
    case WMI_VDEV_INSTALL_KEY_CMDID:
        wcn7850_handle_install_key(s, pci_dev, payload, len);
        break;
    case WMI_PDEV_SET_PARAM_CMDID:
        WCN_DBG(s, "WCN: WMI PDEV SET PARAM (fire-and-forget)\n");
        break;
    case WMI_PDEV_SET_HW_MODE_CMDID:
        WCN_DBG(s, "WCN: WMI PDEV SET HW MODE cmd\n");
        wcn7850_send_set_hw_mode_resp(s, pci_dev);
        break;
    case WMI_VDEV_SET_PARAM_CMDID:
        WCN_DBG(s, "WCN: WMI VDEV SET PARAM (fire-and-forget)\n");
        break;
    case WMI_STA_PS_PARAM_CMDID:
    case WMI_STA_PS_SET_STATE_CMDID:
        WCN_DBG(s, "WCN: WMI STA PS cmd 0x%x (fire-and-forget)\n", cmd_id);
        break;
    case WMI_DFS_ENABLE_CMDID:
    case WMI_DFS_PHYERR_FILTER_ENA_CMDID:
    case WMI_DFS_PHYERR_FILTER_DIS_CMDID:
    case WMI_DFS_CMDID_0xa005:
        WCN_DBG(s, "WCN: WMI DFS cmd 0x%x (fire-and-forget)\n", cmd_id);
        break;
    case WMI_LRO_CONFIG_CMDID:
        WCN_DBG(s, "WCN: WMI LRO CONFIG (fire-and-forget)\n");
        break;
    default:
        WCN_DBG(s, "WCN: WMI cmd 0x%x len=%u (unhandled)\n", cmd_id, len);
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
    /* HTT T2H PEER_MAP v1: struct htt_t2h_peer_map_event (4 x __le32):
     *   info : msg_type[7:0], vdev_id[15:8], peer_id[31:16]
     *   mac_addr_l32 : MAC bytes 0..3
     *   info1 : mac_h16[15:0], hw_peer_id[31:16]
     *   info2 : ast_hash[15:0] */
    uint8_t *p = buf + sizeof(*h);
    uint32_t info = (HTT_T2H_MSG_TYPE_PEER_MAP & 0xff)
                    | ((vdev_id & 0xff) << 8)
                    | ((uint32_t)peer_id << 16);
    uint32_t mac_l32 = (uint32_t)mac[0] | ((uint32_t)mac[1] << 8)
                       | ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24);
    uint32_t info1 = ((uint32_t)mac[4] | ((uint32_t)mac[5] << 8))
                     | ((uint32_t)peer_id << 16);   /* hw_peer_id = peer_id */
    uint32_t info2 = 0;                              /* ast_hash */
    *(uint32_t *)(p + 0)  = cpu_to_le32(info);
    *(uint32_t *)(p + 4)  = cpu_to_le32(mac_l32);
    *(uint32_t *)(p + 8)  = cpu_to_le32(info1);
    *(uint32_t *)(p + 12) = cpu_to_le32(info2);
    uint32_t total = sizeof(*h) + 16;

    h->hdr_info = cpu_to_le32((total - sizeof(*h)) << 16 | 1); /* EPID 1 */
    h->ctrl_info = 0;
    wcn7850_ce_send(s, pci_dev, 1, buf, total);
    WCN_DBG(s, "WCN: HTT PEER_MAP v1 vdev=%u peer_id=%u mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
            vdev_id, peer_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* Build and send a WMI_VDEV_START_RESP_EVENT so the driver's
 * ath12k_mac_vdev_setup_sync() wait_for_completion(&ar->vdev_setup_done)
 * fires with a success status. Struct order matches the driver's
 * struct wmi_vdev_start_resp_event (vdev_id, requestor_id, resp_type,
 * status=0, chain_mask, smps_mode, pdev_id, tx_streams, rx_streams,
 * max_allowed_tx_power). */
static void wcn7850_send_vdev_start_resp(WCN7850State *s, PCIDevice *pci_dev,
                                         uint32_t vdev_id)
{
    uint8_t buf[128];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t off = 0;
    hdr->cmd_id = cpu_to_le32(WMI_VDEV_START_RESP_EVENTID);
    off += sizeof(*hdr);

    uint32_t tlv_start = off;
    WmiTlv *tlv = (WmiTlv *)(buf + off);
    off += sizeof(*tlv);
    struct {
        uint32_t vdev_id;
        uint32_t requestor_id;
        uint32_t resp_type;
        uint32_t status;   /* 0 = WMI_VDEV_START_RESPONSE_STATUS_SUCCESS */
        uint32_t chain_mask;
        uint32_t smps_mode;
        uint32_t pdev_id;
        uint32_t cfgd_tx_streams;
        uint32_t cfgd_rx_streams;
        uint32_t max_allowed_tx_power;
    } QEMU_PACKED resp;
    memset(&resp, 0, sizeof(resp));
    resp.vdev_id = cpu_to_le32(vdev_id);
    memcpy(buf + off, &resp, sizeof(resp));
    off += sizeof(resp);
    tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_VDEV_START_RESPONSE_EVENT,
                                          off - (tlv_start + sizeof(*tlv))));

    uint32_t total = off;
    uint8_t htc_buf[256];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    WCN_DBG(s, "WCN: VDEV START RESP vdev=%u status=0\n", vdev_id);
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
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    WCN_DBG(s, "WCN: PEER ASSOC CONF vdev=%u\n", vdev_id);
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
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    WCN_DBG(s, "WCN: INSTALL KEY COMPLETE vdev=%u\n", vdev_id);
}

/* Build and send a WMI_VDEV_DELETE_RESP_EVENT so the driver's
 * ath12k_mac_vdev_delete() wait_for_completion(&ar->vdev_delete_done)
 * fires. The event carries just the vdev_id (WMI_TAG_VDEV_DELETE_RESP_EVENT). */
static void wcn7850_send_vdev_delete_resp(WCN7850State *s, PCIDevice *pci_dev,
                                          uint32_t vdev_id)
{
    uint8_t buf[64];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t off = 0;
    hdr->cmd_id = cpu_to_le32(WMI_VDEV_DELETE_RESP_EVENTID);
    off += sizeof(*hdr);

    uint32_t tlv_start = off;
    WmiTlv *tlv = (WmiTlv *)(buf + off);
    off += sizeof(*tlv);
    struct {
        uint32_t vdev_id;
    } QEMU_PACKED del_resp;
    del_resp.vdev_id = cpu_to_le32(vdev_id);
    memcpy(buf + off, &del_resp, sizeof(del_resp));
    off += sizeof(del_resp);
    tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_VDEV_DELETE_RESP_EVENT,
                                          off - (tlv_start + sizeof(*tlv))));

    uint32_t total = off;
    uint8_t htc_buf[256];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    WCN_DBG(s, "WCN: VDEV DELETE RESP vdev=%u\n", vdev_id);
}

/* Build and send a WMI_UPDATE_STATS_EVENTID so the driver's
 * ath12k_mac_get_fw_stats() wait_for_completion() fires. The event carries
 * one zeroed pdev stats entry so the driver's pdev parse loop sets
 * stats->stats_id = PDEV_STAT and completes fw_stats_done unconditionally.
 * The event header's stats_id is set from the request so the event type
 * matches what was asked, even though the actual data is dummy. */
static void wcn7850_send_stats_resp(WCN7850State *s, PCIDevice *pci_dev,
                                    uint32_t stats_id, uint32_t vdev_id,
                                    uint32_t pdev_id)
{
    /* Mirror struct ath12k_wmi_pdev_stats_params from driver wmi.h */
    struct {
        struct {
            int32_t  chan_nf;
            uint32_t tx_frame_count;
            uint32_t rx_frame_count;
            uint32_t rx_clear_count;
            uint32_t cycle_count;
            uint32_t phy_err_count;
            uint32_t chan_tx_pwr;
        } QEMU_PACKED base;
        struct {
            int32_t  comp_queued;
            int32_t  comp_delivered;
            int32_t  msdu_enqued;
            int32_t  mpdu_enqued;
            int32_t  wmm_drop;
            int32_t  local_enqued;
            int32_t  local_freed;
            int32_t  hw_queued;
            int32_t  hw_reaped;
            int32_t  underrun;
            int32_t  tx_abort;
            int32_t  mpdus_requed;
            uint32_t tx_ko;
            uint32_t data_rc;
            uint32_t self_triggers;
            uint32_t sw_retry_failure;
            uint32_t illgl_rate_phy_err;
            uint32_t pdev_cont_xretry;
            uint32_t pdev_tx_timeout;
            uint32_t pdev_resets;
            uint32_t stateless_tid_alloc_failure;
            uint32_t phy_underrun;
            uint32_t txop_ovf;
        } QEMU_PACKED tx;
        struct {
            int32_t  mid_ppdu_route_change;
            int32_t  status_rcvd;
            int32_t  r0_frags;
            int32_t  r1_frags;
            int32_t  r2_frags;
            int32_t  r3_frags;
            int32_t  htt_msdus;
            int32_t  htt_mpdus;
            int32_t  loc_msdus;
            int32_t  loc_mpdus;
            int32_t  oversize_amsdu;
            int32_t  phy_errs;
            int32_t  phy_err_drop;
            int32_t  mpdu_errs;
        } QEMU_PACKED rx;
    } QEMU_PACKED pdev_stat;
#define WCN_PDEV_STAT_SZ (sizeof(pdev_stat))

    uint8_t buf[512];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t off = 0;
    hdr->cmd_id = cpu_to_le32(WMI_UPDATE_STATS_EVENTID);
    off += sizeof(*hdr);

    /* WMI_TAG_STATS_EVENT header TLV */
    uint32_t stats_tlv_start = off;
    WmiTlv *stats_tlv = (WmiTlv *)(buf + off);
    off += sizeof(*stats_tlv);
    struct {
        uint32_t stats_id;
        uint32_t num_pdev_stats;
        uint32_t num_vdev_stats;
        uint32_t num_peer_stats;
        uint32_t num_bcnflt_stats;
        uint32_t num_chan_stats;
        uint32_t num_mib_stats;
        uint32_t pdev_id;
        uint32_t num_bcn_stats;
        uint32_t num_peer_extd_stats;
        uint32_t num_peer_extd2_stats;
    } QEMU_PACKED sev;
    memset(&sev, 0, sizeof(sev));
    sev.stats_id = cpu_to_le32(stats_id);
    bool want_pdev = (stats_id & WMI_REQUEST_PDEV_STAT) != 0;
    sev.num_pdev_stats = cpu_to_le32(want_pdev ? 1 : 0);
    sev.pdev_id = cpu_to_le32(pdev_id);
    memcpy(buf + off, &sev, sizeof(sev));
    off += sizeof(sev);
    stats_tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_STATS_EVENT,
                                               off - (stats_tlv_start + sizeof(*stats_tlv))));

    /* Include pdev data only when the request asks for it. */
    uint32_t arr_tlv_start = off;
    WmiTlv *arr_tlv = (WmiTlv *)(buf + off);
    off += sizeof(*arr_tlv);
    if (want_pdev) {
        memset(&pdev_stat, 0, sizeof(pdev_stat));
        memcpy(buf + off, &pdev_stat, sizeof(pdev_stat));
        off += sizeof(pdev_stat);
    }
    arr_tlv->header = cpu_to_le32(WMI_TLV_HDR(WMI_TAG_ARRAY_BYTE,
                                              off - (arr_tlv_start + sizeof(*arr_tlv))));

    uint32_t total = off;
    uint8_t htc_buf[512];
    HtcHdr *h = (HtcHdr *)htc_buf;
    h->hdr_info = cpu_to_le32((total << 16) | HTC_EP_WMI);
    h->ctrl_info = 0;
    memcpy(htc_buf + sizeof(*h), buf, total);
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
    WCN_DBG(s, "WCN: STATS RESP stats_id=0x%x vdev=%u pdev=%u\n",
            stats_id, vdev_id, pdev_id);
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
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
}

/* Wrap a raw 802.11 management frame in WMI_MGMT_RX_EVENTID (MGMT_RX_HDR TLV +
 * ARRAY_BYTE TLV) and deliver it to the driver as if received over the air. */
static void wcn7850_send_mgmt_rx(WCN7850State *s, PCIDevice *pci_dev,
                                 const uint8_t *frame, uint32_t frame_len)
{
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
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);
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
    /* Fixed beacon body: timestamp(8) + beacon interval(2) + capability(2). */
    memset(frame + off, 0, 8); off += 8;               /* timestamp */
    /* beacon interval = 100 TU */
    frame[off++] = 0x64; frame[off++] = 0x00;
    /* capability: ESS(0x0001) only (open network, no Privacy) */
    frame[off++] = 0x01; frame[off++] = 0x00;
    /* SSID IE (id 0, len, ssid) */
    frame[off++] = 0x00; frame[off++] = s->ap_ssid_len;
    memcpy(frame + off, s->ap_ssid, s->ap_ssid_len); off += s->ap_ssid_len;
    /* Supported rates IE (id 1): 6,9,12,18,24,36,48,54 Mbps */
    frame[off++] = 0x01; frame[off++] = 8;
    memcpy(frame + off, wcn7850_supported_rates,
           sizeof(wcn7850_supported_rates));
    off += sizeof(wcn7850_supported_rates);
    /* DS parameter set IE (id 3): channel */
    frame[off++] = 0x03; frame[off++] = 1;
    frame[off++] = (uint8_t)s->ap_channel;
    uint32_t frame_len = off;

    wcn7850_send_mgmt_rx(s, pci_dev, frame, frame_len);
    WCN_DBG(s, "WCN: MGMT RX beacon sent (chan %u, %u bytes)\n",
            s->ap_channel, frame_len);
}

/* Deliver an 802.11 Authentication response (open system, seq 2, status 0)
 * from the AP to our STA so mac80211's authentication completes. */
static void wcn7850_send_auth_resp(WCN7850State *s, PCIDevice *pci_dev)
{
    uint8_t frame[64];
    uint32_t off = 0;
    frame[off++] = 0xb0; frame[off++] = 0x00;          /* FC = Auth */
    frame[off++] = 0x00; frame[off++] = 0x00;          /* duration */
    memcpy(frame + off, s->sta_mac, 6); off += 6;      /* DA = STA */
    memcpy(frame + off, s->ap_bssid, 6); off += 6;     /* SA = BSSID */
    memcpy(frame + off, s->ap_bssid, 6); off += 6;     /* BSSID */
    frame[off++] = 0x10; frame[off++] = 0x00;          /* seq ctrl */
    /* Auth body: algo=0(open), seq=2, status=0 */
    frame[off++] = 0x00; frame[off++] = 0x00;          /* algorithm */
    frame[off++] = 0x02; frame[off++] = 0x00;          /* transaction seq */
    frame[off++] = 0x00; frame[off++] = 0x00;          /* status = success */
    wcn7850_send_mgmt_rx(s, pci_dev, frame, off);
    WCN_DBG(s, "WCN: MGMT RX auth-resp sent (%u bytes)\n", off);
}

/* Deliver an 802.11 (Re)Association Response (status 0, AID 1) from the AP so
 * mac80211's association completes and it proceeds to WMI PEER_ASSOC/VDEV_UP. */
static void wcn7850_send_assoc_resp(WCN7850State *s, PCIDevice *pci_dev,
                                    bool reassoc)
{
    uint8_t frame[128];
    uint32_t off = 0;
    frame[off++] = reassoc ? 0x30 : 0x10;              /* FC = (Re)AssocResp */
    frame[off++] = 0x00;
    frame[off++] = 0x00; frame[off++] = 0x00;          /* duration */
    memcpy(frame + off, s->sta_mac, 6); off += 6;      /* DA = STA */
    memcpy(frame + off, s->ap_bssid, 6); off += 6;     /* SA = BSSID */
    memcpy(frame + off, s->ap_bssid, 6); off += 6;     /* BSSID */
    frame[off++] = 0x20; frame[off++] = 0x00;          /* seq ctrl */
    /* AssocResp body: capability(2), status(2), AID(2) */
    frame[off++] = 0x01; frame[off++] = 0x00;          /* capability: ESS */
    frame[off++] = 0x00; frame[off++] = 0x00;          /* status = success */
    frame[off++] = 0x01; frame[off++] = 0xc0;          /* AID=1 (top 2 bits set) */
    /* Supported rates IE (must echo so mac80211 accepts operating rates) */
    frame[off++] = 0x01; frame[off++] = 8;
    memcpy(frame + off, wcn7850_supported_rates,
           sizeof(wcn7850_supported_rates));
    off += sizeof(wcn7850_supported_rates);
    wcn7850_send_mgmt_rx(s, pci_dev, frame, off);
    WCN_DBG(s, "WCN: MGMT RX assoc-resp sent (reassoc=%d, %u bytes)\n",
            reassoc, off);
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
    WCN_DBG(s, "WCN: WMI PEER CREATE vdev=%u mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
             vdev_id, s->peer_mac[0], s->peer_mac[1], s->peer_mac[2],
             s->peer_mac[3], s->peer_mac[4], s->peer_mac[5]);
    /* The driver waits for an HTT T2H PEER_MAP v1 (peer_map_unmap_version=1). */
    wcn7850_send_htt_peer_map(s, pci_dev, vdev_id, s->peer_id, s->ap_bssid);
    s->peer_mapped = true;
}

static void wcn7850_handle_peer_assoc(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len)
{
    WCN_DBG(s, "WCN: WMI PEER ASSOC cmd len=%u\n", len);
    wcn7850_send_peer_assoc_conf(s, pci_dev, s->vdev_id, s->ap_bssid);
}

static void wcn7850_handle_vdev_start(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len)
{
    /* Wire layout: WmiCmdHdr(4) + struct wmi_vdev_start_request_cmd
     *   { tlv_header(4), vdev_id(4), ... } */
    uint32_t vdev_id = 0;
    if (len >= sizeof(WmiCmdHdr) + 8) {
        const uint8_t *body = payload + sizeof(WmiCmdHdr);
        vdev_id = le32_to_cpu(*(const uint32_t *)(body + 4));
    }
    s->vdev_id = vdev_id;
    WCN_DBG(s, "WCN: WMI VDEV START REQUEST vdev=%u\n", vdev_id);
    /* The driver blocks in ath12k_mac_vdev_setup_sync() on vdev_setup_done. */
    wcn7850_send_vdev_start_resp(s, pci_dev, vdev_id);
}

static void wcn7850_handle_scan_start(WCN7850State *s, PCIDevice *pci_dev,
                                      const uint8_t *payload, uint32_t len)
{
    /* Wire layout: WmiCmdHdr(4) + struct wmi_start_scan_cmd
     *   { tlv_header(4), scan_id(4), scan_req_id(4), vdev_id(4), ... }
     * The fixed struct is 160 bytes; num_chan lives at struct offset 68 and
     * is followed by a WMI_TAG_ARRAY_UINT32 TLV (4-byte hdr) carrying the
     * scanned channel frequencies (one __le32 each). */
    if (len < 16)
        return;
    const uint8_t *body = payload + sizeof(WmiCmdHdr);
    s->scan_id = le32_to_cpu(*(const uint32_t *)(body + 4));
    s->scan_vdev_id = le32_to_cpu(*(const uint32_t *)(body + 12));
    WCN_DBG(s, "WCN: WMI SCAN START scan_id=0x%x vdev=%u\n", s->scan_id,
             s->scan_vdev_id);

    /* Determine whether the simulated AP's channel is part of this scan.
     * mac80211 discards beacons reported on a channel it is not scanning,
     * so we must only emit the beacon when ap_freq is in the channel list. */
    bool ap_in_scan = false;
    const uint32_t SCAN_FIXED_LEN = 160; /* sizeof(struct wmi_start_scan_cmd) */
    const uint32_t NUM_CHAN_OFF = 68;    /* offset of num_chan within struct */
    if (len >= sizeof(WmiCmdHdr) + SCAN_FIXED_LEN + 4) {
        uint32_t num_chan =
            le32_to_cpu(*(const uint32_t *)(body + NUM_CHAN_OFF));
        /* chan_list follows the fixed struct + ARRAY_UINT32 TLV header */
        const uint8_t *chan_list = body + SCAN_FIXED_LEN + 4;
        uint32_t avail = len - (sizeof(WmiCmdHdr) + SCAN_FIXED_LEN + 4);
        if (num_chan > avail / 4)
            num_chan = avail / 4;
        for (uint32_t i = 0; i < num_chan; i++) {
            uint32_t f = le32_to_cpu(*(const uint32_t *)(chan_list + i * 4));
            if (f == s->ap_freq) {
                ap_in_scan = true;
            }
        }
    }

    /* Scan flow the driver expects:
     *   STARTED -> (per channel) FOREIGN_CHAN + beacon (MGMT_RX) + BSS_CHANNEL
     *   -> COMPLETED
     * Emit the events asynchronously via scan_seq_timer, paced ~10ms apart in
     * guest virtual time, so the driver/mac80211 have time to process each
     * event (STARTED -> RUNNING, FOREIGN_CHAN -> scan channel switch) before
     * the next one arrives. In particular the beacon must follow FOREIGN_CHAN
     * after mac80211 has set the scan channel, otherwise it is discarded. */
    s->scan_seq_ap = ap_in_scan;
    s->scan_seq_step = 0;
    timer_mod(s->scan_seq_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000);
    WCN_DBG(s, "WCN: scan sequence queued for SSID '%.*s' (ap_in_scan=%d)\n",
            s->ap_ssid_len, s->ap_ssid, ap_in_scan);
}

static void wcn7850_scan_seq_timer(void *opaque)
{
    WCN7850State *s = opaque;
    PCIDevice *pci_dev = PCI_DEVICE(s);

    /* Keep scan events ordered while a deferred CE event is waiting for a
     * destination buffer. */
    if (s->deferred_ce_count > 0) {
        timer_mod(s->scan_seq_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
        return;
    }

    switch (s->scan_seq_step) {
    case 0:
        wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_STARTED, 0, 0);
        break;
    case 1:
        if (s->scan_seq_ap) {
            wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_FOREIGN_CHAN, 0,
                                    s->ap_freq);
        }
        break;
    case 2:
        if (s->scan_seq_ap) {
            wcn7850_send_beacon(s, pci_dev);
        }
        break;
    case 3:
        if (s->scan_seq_ap) {
            wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_BSS_CHANNEL, 0,
                                    s->ap_freq);
        }
        break;
    case 4:
        wcn7850_send_scan_event(s, pci_dev, WMI_SCAN_EVENT_COMPLETED,
                                WMI_SCAN_REASON_COMPLETED, 0);
        break;
    default:
        s->scan_seq_step = 0;
        return;
    }

    s->scan_seq_step++;
    timer_mod(s->scan_seq_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000000);
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
    uint32_t desc_id = le32_to_cpu(*(const uint32_t *)(body + 8));
    uint32_t paddr_lo = le32_to_cpu(*(const uint32_t *)(body + 16));
    uint32_t paddr_hi = le32_to_cpu(*(const uint32_t *)(body + 20));
    uint32_t frame_len = le32_to_cpu(*(const uint32_t *)(body + 24));
    WCN_DBG(s, "WCN: WMI MGMT TX vdev=%u desc_id=%u frame_len=%u\n",
             vdev_id, desc_id, frame_len);

    /* DMA-read the outgoing 802.11 frame so we know whether the STA is sending
     * an Authentication or (Re)Association request, and learn our own MAC. */
    uint8_t txf[256];
    uint32_t txlen = frame_len > sizeof(txf) ? sizeof(txf) : frame_len;
    dma_addr_t fpaddr = ((dma_addr_t)paddr_hi << 32) | paddr_lo;
    uint8_t subtype = 0xff;
    bool have_frame = false;
    if (txlen >= 24 &&
        pci_dma_read(pci_dev, fpaddr, txf, txlen) == MEMTX_OK) {
        have_frame = true;
        subtype = (txf[0] >> 4) & 0xf;   /* mgmt subtype nibble */
        /* SA (transmitter) is at offset 10 in a mgmt header: that is our STA. */
        memcpy(s->sta_mac, txf + 10, 6);
        s->sta_mac_valid = true;
        WCN_DBG(s, "WCN: MGMT TX subtype=0x%x sa=%02x:%02x:%02x:%02x:%02x:%02x\n",
                 subtype, s->sta_mac[0], s->sta_mac[1], s->sta_mac[2],
                 s->sta_mac[3], s->sta_mac[4], s->sta_mac[5]);
    }

    /* Emit MGMT_TX_COMPLETION_EVENT so the driver's tx completion frees the
     * pending skb. struct wmi_mgmt_tx_compl_event begins with desc_id (the
     * idr cookie), NOT vdev_id. */
    uint8_t buf[64];
    WmiCmdHdr *hdr = (WmiCmdHdr *)buf;
    uint32_t b = 0;
    hdr->cmd_id = cpu_to_le32(WMI_MGMT_TX_COMPLETION_EVENTID);
    b += sizeof(*hdr);
    uint32_t tlv_start = b;
    WmiTlv *tlv = (WmiTlv *)(buf + b);
    b += sizeof(*tlv);
    struct {
        uint32_t desc_id;   /* must match the cmd's desc_id */
        uint32_t status;    /* 0 = success */
        uint32_t pdev_id;
        uint32_t ppdu_id;
        uint32_t ack_rssi;
    } QEMU_PACKED mc;
    memset(&mc, 0, sizeof(mc));
    mc.desc_id = cpu_to_le32(desc_id);
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
    wcn7850_ce_send(s, pci_dev, 2, htc_buf, sizeof(*h) + total);

    /* Respond to the STA's management request as the simulated AP would over
     * the air, so mac80211's SME state machine advances:
     *   Auth request (subtype 0xb)          -> Auth response (open, success)
     *   Assoc request (0x0) / Reassoc (0x2) -> (Re)Assoc response (success) */
    if (have_frame) {
        if (subtype == 0xb) {
            wcn7850_send_auth_resp(s, pci_dev);
        } else if (subtype == 0x0) {
            wcn7850_send_assoc_resp(s, pci_dev, false);
        } else if (subtype == 0x2) {
            wcn7850_send_assoc_resp(s, pci_dev, true);
        }
    }
}

static void wcn7850_handle_install_key(WCN7850State *s, PCIDevice *pci_dev,
                                       const uint8_t *payload, uint32_t len)
{
    WCN_DBG(s, "WCN: WMI INSTALL KEY cmd len=%u\n", len);
    wcn7850_send_install_key_compl(s, pci_dev, s->vdev_id);
}

/* Handle HTT command from driver (sent via HTC EP HTT on CE pipe 4).
 * Messages handled:
 *   0x0 = HTT_H2T_MSG_TYPE_VERSION_REQ — respond with version conf
 *   0xb = HTT_H2T_MSG_TYPE_SRING_SETUP — record ring config from driver */
static void wcn7850_handle_htt_cmd(WCN7850State *s, PCIDevice *pci_dev,
                                   const uint8_t *payload, uint32_t len)
{
    if (len < 4) {
        return;
    }
    uint32_t ver_reg_info = le32_to_cpu(*(const uint32_t *)payload);
    uint32_t msg_type = ver_reg_info & 0xff;

    if (msg_type == HTT_H2T_MSG_TYPE_VERSION_REQ) {
        uint8_t buf[16];
        HtcHdr *hdr = (HtcHdr *)buf;
        hdr->hdr_info = cpu_to_le32((4 << 16) | 1);
        hdr->ctrl_info = 0;
        uint32_t *ver = (uint32_t *)(buf + sizeof(HtcHdr));
        *ver = cpu_to_le32((3 << 16) | (0 << 8));  /* major 3, minor 0 */
        wcn7850_ce_send(s, pci_dev, 1, buf, sizeof(HtcHdr) + 4);

    } else if (msg_type == HTT_H2T_MSG_TYPE_RX_RING_SELECTION_CFG) {
        if (len < 4) {
            return;
        }
        const uint32_t *dw = (const uint32_t *)(payload);
        uint32_t info0 = le32_to_cpu(dw[0]);
        uint32_t ring_id = (info0 >> 16) & 0xff;
        uint32_t pdev_id = (info0 >> 8) & 0xff;
        uint32_t ss = (info0 >> 24) & 1;
        uint32_t ps = (info0 >> 25) & 1;
        uint32_t rxmon = (info0 >> 28) & 1;
        WCN_DBG(s, "WCN: HTT ring_sel_cfg ring_id=%u pdev=%u ss=%u ps=%u rxmon=%u\n",
                 ring_id, pdev_id, ss, ps, rxmon);

    } else if (msg_type == HTT_H2T_MSG_TYPE_SRING_SETUP) {
        if (len < HTT_SRING_SETUP_MSG_LEN) {
            return;
        }
        const uint32_t *dw = (const uint32_t *)(payload);
        uint32_t info0    = le32_to_cpu(dw[0]);
        uint32_t base_lo  = le32_to_cpu(dw[1]);
        uint32_t base_hi  = le32_to_cpu(dw[2]);
        uint32_t info1    = le32_to_cpu(dw[3]);
        uint32_t hp_lo    = le32_to_cpu(dw[4]);
        uint32_t hp_hi    = le32_to_cpu(dw[5]);
        uint32_t tp_lo    = le32_to_cpu(dw[6]);
        uint32_t tp_hi    = le32_to_cpu(dw[7]);
        uint32_t msi_data = len >= 44 ? le32_to_cpu(dw[10]) : 0; /* msi_data */

        uint32_t htt_ring_type = (info0 >> 24) & 0xff;
        uint32_t htt_ring_id   = (info0 >> 16) & 0xff;
        uint32_t pdev_id       = (info0 >> 8) & 0xff;
        uint64_t base_addr     = ((uint64_t)base_hi << 32) | base_lo;
        /* The driver computes ring_size as num_entries * entry_sz where both
         * are in dword units (see ath12k_dp_tx_htt_srng_setup), so the value
         * is a dword count.  Convert to bytes to match the model's internal
         * byte-unit ring offsets. */
        uint32_t ring_size_dw  = info1 & 0xffff;
        uint32_t ring_size     = ring_size_dw * 4;
        uint32_t entry_sz      = ((info1 >> 16) & 0xff) * 4;
        uint64_t hp_addr       = ((uint64_t)hp_hi << 32) | hp_lo;
        uint64_t tp_addr       = ((uint64_t)tp_hi << 32) | tp_lo;

        /* Map HTT ring ID to model ring type + index.
         * For WCN7850 with rx_mac_buf_ring=true the driver sets up two RXDMA
         * host buf rings (one per mac_id).  The firmware-facing SW_TO_SW ring
         * (HOST1_TO_FW, ring_id=5) is index 0.  Each HW-facing host buf ring
         * (RXDMA_HOST_BUF_RING, ring_id=0) gets a unique index equal to its
         * pdev_id (1 or 2). */
        Wcn7850RingType ring_type = WCN7850_RING_UNKNOWN;
        int ring_idx = 0;
        switch (htt_ring_id) {
        case HTT_RXDMA_HOST_BUF_RING:
            /* Each mac_id gets its own HW-facing RXDMA buf ring.
             * pdev_id distinguishes mac_id 0 from mac_id 1. */
            ring_type = WCN7850_RING_RXDMA_BUF;
            ring_idx = pdev_id;
            break;
        case HTT_RXDMA_HOST_BUF_RING2:
            ring_type = WCN7850_RING_RXDMA_BUF;
            ring_idx = 3;
            break;
        case HTT_HOST1_TO_FW_RXBUF_RING:
            ring_type = WCN7850_RING_RXDMA_BUF;
            ring_idx = 0;
            break;
        case HTT_HOST2_TO_FW_RXBUF_RING:
            ring_type = WCN7850_RING_RXDMA_BUF;
            ring_idx = 2;
            break;
        case HTT_RXDMA_NON_MONITOR_DEST_RING:
            /* REO destination ring (actually RXDMA DST, but the FW processes
             * descriptors on behalf of the REO block and posts completions
             * to the REO DST ring).  Track it for RX completions. */
            ring_type = WCN7850_RING_REO_DST;
            ring_idx = pdev_id;
            /* RXDMA DST rings have no ext_irq group, so msi_data=0 in the
             * HTT message.  Override with the correct REO DST ring 0 MSI
             * vector (DP_MSI_BASE + 3 = 11) so the NAPI handler polls the
             * ring on the right group. */
            if (!msi_data) {
                msi_data = WCN7850_DP_MSI_BASE + 3;
            }
            break;
        case HTT_RXDMA_MONITOR_STATUS_RING:
        case HTT_RXDMA_MONITOR_BUF_RING:
        case HTT_RXDMA_MONITOR_DESC_RING:
        case HTT_RXDMA_MONITOR_DEST_RING:
            /* Already configured via MMIO or not needed for basic data path */
            break;
        default:
            break;
        }

        if (ring_type != WCN7850_RING_UNKNOWN) {
            WCN7850RingState *r = wcn7850_add_or_update_ring(s, ring_type, ring_idx);
            if (r) {
                /* For rings previously configured via MMIO (UMAC window for
                 * REO DST/EXCEPTION, or always-on shadow for TCL), preserve
                 * the already-set configuration. SRING_SETUP only provides
                 * base/size/entry for PMAC rings, not UMAC REO rings. */
                if (!r->configured) {
                    r->configured  = true;
                    r->base_addr   = base_addr;
                    r->size        = ring_size;
                    r->entry_size  = entry_sz;
                    r->enable = true;
                    r->num_bufs = 0;
                }
                /* Always use hp_addr from SRING_SETUP when non-zero (PMAC
                 * rings like RXDMA_BUF, RXDMA_DST).  For UMAC REO rings
                 * the hp_addr=0; the real RDP address was already set by
                 * the UMAC window REO HP_ADDR intercept above. */
                if (hp_addr && !r->hp_shadow_addr) {
                    r->hp_shadow_addr = hp_addr;
                }
                if (tp_addr && !r->tp_shadow_addr) {
                    r->tp_shadow_addr = tp_addr;
                }
                /* Always update MSI vector from HTT (driver sets it last) */
                if (msi_data & (WCN7850_MSI_VECTORS - 1)) {
                    r->msivec = msi_data & (WCN7850_MSI_VECTORS - 1);
                }

                 WCN_DBG(s, "WCN: HTT SRING_SETUP ring_type=%u ring_id=%u pdev=%u"
                          " base=0x%"PRIx64" sz=%u esz=%u hp=0x%"PRIx64" tp=0x%"PRIx64
                          " msivec=%u msi_data=0x%x\n",
                          htt_ring_type, htt_ring_id, pdev_id,
                          base_addr, ring_size, entry_sz, hp_addr, tp_addr,
                          r->msivec, msi_data);
            }
        } else {
            WCN_DBG(s, "WCN: HTT SRING_SETUP (untracked) ring_type=%u ring_id=%u"
                     " pdev=%u base=0x%"PRIx64" sz=%u esz=%u\n",
                     htt_ring_type, htt_ring_id, pdev_id,
                     base_addr, ring_size, entry_sz);
        }
    }
}

/* Dispatch HTC message to appropriate handler */
static void wcn7850_htc_dispatch(WCN7850State *s, PCIDevice *pci_dev,
                                  uint8_t epid, const uint8_t *payload,
                                  uint32_t len)
{
    WCN_DBG(s, "WCN: htc_dispatch epid=%u len=%u\n", epid, len);
    switch (epid) {
    case HTC_EP_CTRL: {
        HtcHdr *hdr = (HtcHdr *)(payload - 8);
        uint16_t msg_id = (payload[0] | (payload[1] << 8)) & 0x3f;
        WCN_DBG(s, "WCN: HTC ctrl msg_id=%u hdr_info=0x%08x ctrl_info=0x%08x\n",
                msg_id, hdr->hdr_info, hdr->ctrl_info);

        switch (msg_id) {
        case HTC_MSG_SETUP_COMPLETE_EX_ID:
            WCN_DBG(s, "WCN: HTC setup complete\n");
            /* Host sends setup complete only after all endpoint rx_cb's are
             * registered. Now it is safe to deliver WMI service-ready events. */
            s->wmi_service_ready_pending = true;
            break;
        case HTC_MSG_CONNECT_SERVICE_ID:
            WCN_DBG(s, "WCN: HTC connect svc\n");
            wcn7850_htc_handle_connect(s, pci_dev, payload);
            break;
        default:
            WCN_DBG(s, "WCN: unknown HTC ctrl msg %u\n", msg_id);
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
        WCN_DBG(s, "WCN: unhandled HTC EP %u len=%u\n", epid, len);
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
    WCN_DBG(s, "WCN: process_ce_src pipe=%d tp=%u hp=%u base=0x%lx\n",
             ce_pipe, s->ce_src[ce_pipe].tp, s->ce_src[ce_pipe].hp,
             (unsigned long)s->ce_src[ce_pipe].base);
    uint64_t base = s->ce_src[ce_pipe].base;
    uint32_t esize = s->ce_src[ce_pipe].entry_size ?: 16;
    uint32_t ring_sz = s->ce_src[ce_pipe].size ?: (WCN7850_CE_SRC_RING_SIZE * esize);
    uint32_t tp = s->ce_src[ce_pipe].tp;
    uint32_t hp = s->ce_src[ce_pipe].hp;
    bool consumed = false;

    if (!base || !ring_sz || esize < sizeof(uint64_t) ||
        esize > WCN7850_MAX_CE_DESC_SIZE || ring_sz < esize ||
        ring_sz % esize)
        return;

    if (tp != hp)
        WCN_DBG(s, "WCN: process_ce_src pipe=%d tp=%u hp=%u -> %d descs to process\n",
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
            WCN_DBG(s, "WCN: SRC buf read FAILED at 0x%lx len=%u\n",
                    (unsigned long)buf_addr, buf_len);
            tp += esize;
            if (tp >= ring_sz) tp = 0;
            continue;
        }

        /* Parse HTC header */
        if (buf_len < sizeof(HtcHdr)) {
            tp += esize;
            if (tp >= ring_sz) tp = 0;
            continue;
        }
        HtcHdr *htc_hdr = (HtcHdr *)htc_buf;
        uint8_t epid = HTC_HDR_EPID(htc_hdr);
        uint32_t payload_len = HTC_HDR_PAYLOAD_LEN(htc_hdr);
        WCN_DBG(s, "WCN: SRC desc at 0x%lx buf=0x%lx len=%u hdr=0x%08x ctrl=0x%08x\n",
                (unsigned long)desc_addr, (unsigned long)buf_addr, buf_len,
                htc_hdr->hdr_info, htc_hdr->ctrl_info);
        if (payload_len > buf_len - sizeof(*htc_hdr))
            payload_len = buf_len - sizeof(*htc_hdr);

        /* Dispatch to HTC handler */
        wcn7850_htc_dispatch(s, pci_dev, epid,
                              htc_buf + sizeof(*htc_hdr), payload_len);

        tp += esize;
        if (tp >= ring_sz) tp = 0;
        consumed = true;
    }

    s->ce_src[ce_pipe].tp = tp;

    /* Write TP back to shared memory so driver HP updates work */
    if (s->ce_src[ce_pipe].tp_addr) {
        uint32_t tp_le = cpu_to_le32(tp / 4);
        pci_dma_write(pci_dev, s->ce_src[ce_pipe].tp_addr,
                      &tp_le, sizeof(tp_le));
    }

    /* Fire the CE completion MSI so the driver's send_done_cb reaps the
     * source ring and refreshes its cached_tp; otherwise ath12k_ce_send()
     * sees num_free==0 (stale cached_tp) and fails later commands with
     * -ENOBUFS once write_index wraps the ring. */
    if (consumed) {
        int msivec = wcn7850_ce_msi_vector(ce_pipe);
        msi_notify(pci_dev, msivec);
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
            WCN_DBG(s, "WCN: CE2 mmio win_off=0x%lx blk=0x%lx is_dst=%d val=0x%x\n",
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
                uint32_t ring_sz = (base_msb_cfg >> 8) & 0xfffff;
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
                    s->ce_dst[ce_pipe].base = base_addr;
                    s->ce_dst[ce_pipe].size = ring_sz_bytes;
                    s->ce_dst[ce_pipe].entry_size = entry_size;
                    if (hp_addr)
                        s->ce_dst[ce_pipe].hp_addr = hp_addr;
                    WCN_DBG(s, "WCN: CE dst cfg pipe=%d base=0x%lx sz=%u esz=%u hp_addr=0x%lx\n",
                             ce_pipe, (unsigned long)base_addr, ring_sz_bytes, entry_size,
                             (unsigned long)hp_addr);
                } else {
                    s->ce_src[ce_pipe].base = base_addr;
                    s->ce_src[ce_pipe].size = ring_sz_bytes;
                    s->ce_src[ce_pipe].entry_size = entry_size;
                    if (hp_addr)
                        s->ce_src[ce_pipe].tp_addr = hp_addr;
                    WCN_DBG(s, "WCN: CE src cfg pipe=%d base=0x%lx sz=%u esz=%u tp_addr=0x%lx\n",
                             ce_pipe, (unsigned long)base_addr, ring_sz_bytes,
                             entry_size, (unsigned long)hp_addr);
                }
                /* DST status ring config in the CE destination block. */
                if (is_dst) {
                    uint32_t sts_lsb = *(uint32_t *)(s->window_memory +
                        cfg_base + WCN7850_CE_DST_BASE_LO);
                    uint32_t sts_msb = *(uint32_t *)(s->window_memory +
                        cfg_base + WCN7850_CE_DST_BASE_HI);
                    if (sts_lsb || sts_msb) {
                        s->ce_sts[ce_pipe].base = sts_lsb |
                            ((uint64_t)(sts_msb & 0xff) << 32);
                        uint32_t sts_rind = *(uint32_t *)(s->window_memory +
                            cfg_base + WCN7850_CE_DST_SIZE);
                        uint32_t sts_entry_size = (sts_rind & 0xff) * 4;
                        if (!sts_entry_size) sts_entry_size = 16;
                        s->ce_sts[ce_pipe].size = ((sts_msb >> 8) & 0xfffff) * sts_entry_size;
                        s->ce_sts[ce_pipe].entry_size = sts_entry_size;
                        uint32_t st_hp_lsb = *(uint32_t *)(s->window_memory +
                            cfg_base + WCN7850_CE_CFG_DST_TP);
                        uint32_t st_hp_msb = *(uint32_t *)(s->window_memory +
                            cfg_base + WCN7850_CE_CFG_DST_TP_HI);
                        s->ce_sts[ce_pipe].hp_addr = st_hp_lsb |
                            ((uint64_t)(st_hp_msb & 0xff) << 32);
                        WCN_DBG(s, "WCN: CE%d sts base=0x%lx sz=%u esz=%u hp_addr=0x%lx\n",
                                ce_pipe,
                                (unsigned long)s->ce_sts[ce_pipe].base,
                                s->ce_sts[ce_pipe].size,
                                s->ce_sts[ce_pipe].entry_size,
                                (unsigned long)s->ce_sts[ce_pipe].hp_addr);
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
            WCN_DBG(s, "WCN: R2 win_off=0x%lx blk=0x%lx pipe=%d is_dst=%d val=0x%x\n",
                    (unsigned long)win_off, (unsigned long)blk, ce_pipe,
                    is_dst, (uint32_t)val);
            if (blk == WCN7850_CE_RING_HP_OFFSET) {
                if (is_dst) {
                    s->ce_dst[ce_pipe].hp = val;
                    int wmi_dl = s->htc_ep[HTC_EP_WMI].dl_pipe ?: 2;
                    if (ce_pipe == wmi_dl) {
                        uint32_t ring_sz = s->ce_dst[ce_pipe].size ?:
                            (WCN7850_CE_DST_RING_SIZE *
                             (s->ce_dst[ce_pipe].entry_size ?: 16));
                        uint32_t esize = s->ce_dst[ce_pipe].entry_size ?: 16;
                        uint32_t doorbell_bytes = val * 4;
                        uint32_t prev = s->ce_dst[ce_pipe].drv_hp_prev;
                        if (doorbell_bytes != prev) {
                            uint32_t delta = (doorbell_bytes - prev
                                             + ring_sz) % ring_sz;
                            s->ce_dst[ce_pipe].pending += delta / esize;
                        }
                        s->ce_dst[ce_pipe].drv_hp_prev = doorbell_bytes;
                        s->ce_dst[ce_pipe].drv_hp = doorbell_bytes;
                        WCN_DBG(s, "WCN: CE-win DST HP pipe=%d val=0x%" PRIx64 " pending=%u\n",
                                ce_pipe, val, s->ce_dst[ce_pipe].pending);
                    }
                } else {
                    WCN_DBG(s, "WCN: CE-dir HP pipe=%d val=0x%x\n", ce_pipe, (uint32_t)val);
                    s->ce_src[ce_pipe].hp = val * 4;
                    wcn7850_process_ce_src(s, pci_dev, ce_pipe);
                }
            } else if (blk == WCN7850_CE_RING_TP_OFFSET) {
                if (is_dst)
                    s->ce_dst[ce_pipe].tp = val * 4;
                else
                    s->ce_src[ce_pipe].tp = val * 4;
            } else if (blk == (WCN7850_CE_RING_HP_OFFSET + 8)) {
                if (is_dst)
                    s->ce_sts[ce_pipe].hp = val;
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
    /* TLV type 0x10: num_phy(1). No opt_flag byte on the wire; the
     * decoder sets num_phy_valid=1 internally (QMI_OPT_FLAG consumes no
     * bytes). */
    tlv[0] = 0x10; tlv[1] = 0x01; tlv[2] = 0x00;
    tlv[3] = 0x01; /* num_phy = 1 */
    tlv += 4;
    /* TLV type 0x11: board_id(4) */
    tlv[0] = 0x11; tlv[1] = 0x04; tlv[2] = 0x00;
    tlv[3] = 0x00; tlv[4] = 0x00; tlv[5] = 0x00; tlv[6] = 0x00; /* board_id = 0 */
    tlv += 7;
    /* TLV type 0x13: single_chip_mlo_support(1) */
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
    const char *fw_ts = "2025-01-01";
    uint8_t fw_ts_len = strlen(fw_ts);
    uint32_t tlv_val = 0;
    uint32_t tlv_len = 0;

    /* No opt_flag bytes on the wire: the decoder sets the *_valid flags
     * internally (QMI_OPT_FLAG consumes no bytes). */
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
    /* TLV: chip_info (type 0x10, len 8): chip_id(4)+chip_family(4) */
    tlv[0] = 0x10; tlv[1] = 8; tlv[2] = 0x00;
    tlv_val = cpu_to_le32(2);
    memcpy(&tlv[3], &tlv_val, 4); /* chip_id = 2 */
    tlv_val = cpu_to_le32(0);
    memcpy(&tlv[7], &tlv_val, 4); /* chip_family = 0 */
    tlv += 3 + 8;
    /* TLV: board_info (type 0x11, len 4): board_id(4) */
    tlv[0] = 0x11; tlv[1] = 4; tlv[2] = 0x00;
    tlv_val = cpu_to_le32(0);
    memcpy(&tlv[3], &tlv_val, 4); /* board_id = 0 */
    tlv += 3 + 4;
    /* TLV: fw_version_info (type 0x13): fw_version(4)+string(len(1)+bytes) */
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
    } else {
        hdr->msg_len = cpu_to_le16(0);
    }

    /* Payload for QRTR is the QMI header + QMI payload (hdr->msg_len bytes).
     * Compute the actual QMI payload size from the QMI header (stored little-endian).
     */
    uint32_t qmi_payload_size = sizeof(*hdr) + le16_to_cpu(hdr->msg_len);

    qrtr_len = wcn7850_build_qrtr_pkt(qrtr_buf, sizeof(qrtr_buf),
                                       QRTR_TYPE_DATA,
                                       QRTR_NODE_FW, QRTR_PORT_QMI_SERVICE,
                                       s->qmi_client_node, s->qmi_client_port,
                                       qmi_buf, qmi_payload_size);
    if (!qrtr_len)
        return false;

    uint64_t dl_ctxt_addr = ch_ctxt_base + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
    if (!wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr,
                                qrtr_buf, qrtr_len)) {
        WCN_DBG(s, "WCN: QMI ind 0x%04x write DL failed\n", msg_id);
        return false;
    }

    WCN_DBG(s, "WCN: sent QMI ind 0x%04x\n", msg_id);
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

    WCN_DBG(s, "WCN: QMI request msg_id=0x%04x txn=%u\n", msg_id, txn_id);
    if (data_len) {
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
        WCN_DBG(s, "WCN: unknown QMI msg 0x%04x -> ack\n", msg_id);
        resp_len = wcn7850_qmi_build_resp(resp_buf, sizeof(resp_buf), txn_id, msg_id, 1);
        break;
    }

    if (!resp_len)
        return;

    /* Wrap the QMI response in QRTR DATA */
    WCN_DBG(s, "WCN: sending QMI response msg_id=0x%04x len=%u\n", msg_id, resp_len);
    if (resp_len) {
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
                                    qrtr_pkt, qrtr_len)) {
            WCN_DBG(s, "WCN: failed to write DL data\n");
        }
    }

    /* Send any pending QMI indication (REQUEST_MEM_IND, FW_MEM_READY_IND) */
    if (s->qmi_pending_ind) {
        uint16_t ind = s->qmi_pending_ind;
        WCN_DBG(s, "WCN: sending pending QMI ind 0x%04x\n", ind);
        if (wcn7850_send_qmi_ind(s, pci_dev, ch_ctxt_addr, er_ctxt_addr, ind)) {
            s->qmi_pending_ind = 0;
        } else {
            WCN_DBG(s, "WCN: will retry pending QMI ind 0x%04x\n", ind);
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
    return wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr,
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

    if (payload_size > data_len - sizeof(*hdr))
        return;

    /* If host asks us to confirm receipt, send RESUME_TX to unblock flow */
    if (confirm_rx) {
        WCN_DBG(s, "WCN: QRTR confirm_rx -> send RESUME_TX\n");
        wcn7850_send_resume_tx(s, pci_dev, ch_ctxt_addr, er_ctxt_addr);
    }

    WCN_DBG(s, "WCN: QRTR type=%u src=%u:%u size=%u\n",
             type, src_node, src_port, payload_size);
    if (type == QRTR_TYPE_DATA) {
        WCN_DBG(s, "WCN: QRTR DATA hex");
        for (size_t _i = 0; _i < payload_size && _i < 64; _i++)
            WCN_DBG(s, " %02x", payload[_i]);
        WCN_DBG(s, "\n");
    }

    switch (type) {
    case QRTR_TYPE_DATA:
        wcn7850_process_qmi(s, pci_dev, ch_ctxt_addr, er_ctxt_addr, er_index,
                             src_node, src_port,
                             payload, payload_size);
        break;
    case QRTR_TYPE_RESUME_TX:
        /* Host sent us RESUME_TX — flow from model→host is unblocked.
         * We don't maintain pending counters, so nothing to do. */
        WCN_DBG(s, "WCN: received RESUME_TX (ignored)\n");
        break;
    default:
        WCN_DBG(s, "WCN: unhandled QRTR type %u\n", type);
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

    WCN_DBG(s, "WCN: process_cmd_ring ctxt=0x%lx\n", (unsigned long)s->cmd_ctxt_addr);
    if (!s->cmd_ctxt_addr)
        return;
    if (pci_dma_read(pci_dev, s->cmd_ctxt_addr, ctxt, sizeof(ctxt)) != MEMTX_OK) {
        WCN_DBG(s, "WCN: cmd ctxt read failed\n");
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
    WCN_DBG(s, "WCN: cmd ring rbase=0x%lx rp=0x%lx wp=0x%lx rlen=0x%lx\n",
             (unsigned long)rbase, (unsigned long)rp, (unsigned long)wp,
             (unsigned long)rlen);

    while (rp != wp) {
        MhiTre tre;

        if (!wcn7850_read_tre(pci_dev, rbase, rp - rbase, &tre)) {
            WCN_DBG(s, "WCN: cmd TRE read failed\n");
            break;
        }
        uint32_t dword1 = le32_to_cpu(tre.dword1);
        uint32_t cmdtype = (dword1 >> 16) & 0xff;
        uint32_t chid = (dword1 >> 24) & 0xff;
        WCN_DBG(s, "WCN: CMD ch=%u type=0x%x\n", chid, cmdtype);
        WCN_DBG(s, "WCN: CMD ch=%u type=0x%x\n", chid, cmdtype);

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
    if (ctxt.rlen > WCN7850_MAX_MHI_RING_SIZE ||
        ctxt.rlen % sizeof(MhiTre) ||
        ctxt.rbase + ctxt.rlen < ctxt.rbase)
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

        WCN_DBG(s, "WCN: UL TRE ch=%u ptr=0x%lx len=%u\n",
                 ch_id, (unsigned long)tre.ptr, len);

        if (pci_dma_read(pci_dev, tre.ptr, buf, len) == MEMTX_OK) {
            if (ch_id == MHI_CHAN_IPCR_UL) {
                if (!s->qmi_newserver_delivered) {
                    WCN_DBG(s, "WCN: first IPCR UL data — NEW_SERVER delivered\n");
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
    if (!wcn7850_write_dl_data(s, pci_dev, dl_ctxt_addr, er_ctxt_addr,
                                qrtr_buf, qrtr_len)) {
        return false;
    }

    WCN_DBG(s, "WCN: sent NEW_SERVER service=0x%x node=%u port=%u\n",
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

    WCN_DBG(s, "WCN: chan_db ch=%u off=%d val=0x%lx\n",
             ch_id, db_offset, (unsigned long)db_val);
    WCN_DBG(s, "WCN: chan_db ch=%u off=%d\n", ch_id, db_offset);

    if (ch_id >= MHI_CHAN_MAX)
        return;

    if (!s->chan_ctxt_addr) {
        WCN_DBG(s, "WCN: chan_db ch=%u but no chan_ctxt_addr\n", ch_id);
        return;
    }
    uint64_t ch_ctxt_addr = s->chan_ctxt_addr + ch_id * sizeof(MhiChanCtxt);

    if (db_offset == 0 && ch_id == MHI_CHAN_IPCR_UL) {
        /* UL doorbell (host -> device): process incoming data */
        wcn7850_process_ul_data(s, pci_dev, ch_ctxt_addr, ch_id,
                                 s->er_ctxt_addr);
    }

    if (ch_id == MHI_CHAN_IPCR_DL && (db_offset == 0 || db_offset == 4)) {
        /* DL doorbell (host -> device): host has queued DL buffers.
         * If NEW_SERVER hasn't been sent yet, send it now using one of
         * the host's pre-queued DL buffers. */
        if (!s->qmi_newserver_sent && s->chan_ctxt_addr) {
            if (wcn7850_send_new_server(s, pci_dev, s->chan_ctxt_addr,
                                        s->er_ctxt_addr)) {
                s->qmi_newserver_sent = true;
                s->qmi_newserver_resent = true;
                s->qmi_service_active = true;
                s->qmi_boot_pending = false;
                s->ce_ready = true;
                timer_mod(s->ce_poll_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000LL);
            }
        }
    }
}

static void wcn7850_handle_event_ring_db(WCN7850State *s, PCIDevice *pci_dev,
                                          hwaddr db_addr, uint64_t db_val)
{
    uint32_t er_index = (db_addr - WCN7850_ERDBOFF_VALUE) / 8;
    (void)db_val;
    WCN_DBG(s, "WCN: er_db er=%u val=0x%lx\n", er_index,
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
    if (rlen > WCN7850_MAX_MHI_RING_SIZE ||
        rlen % sizeof(MhiTre) || rbase + rlen < rbase)
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
    WCN_DBG(s, "WCN: EE event emitted pending=%d ee=%u\n", s->ctrl_event_pending, 2u);
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
    WCN_DBG(s, "WCN: state event state=%u\n", state);
}

/* ===== MMIO handlers ===== */

static uint64_t wcn7850_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    WCN7850State *s = opaque;
    uint64_t ret = 0;
    hwaddr win_off;

    if (addr >= WCN7850_MHI_SPECIAL_BASE &&
        addr < WCN7850_MHI_SPECIAL_BASE + WCN7850_MHIREGLEN) {
        addr = WCN7850_MHIREGLEN + (addr - WCN7850_MHI_SPECIAL_BASE);
    }

    switch (addr) {
    case WCN7850_MHIREGLEN:
        ret = WCN7850_BAR0_SIZE;
        return ret;
    case WCN7850_MHICFG:
        ret = WCN7850_MHICFG_VALUE;
        return ret;
    case WCN7850_CHDBOFF:
        ret = WCN7850_CHDBOFF_VALUE;
        return ret;
    case WCN7850_ERDBOFF:
        ret = WCN7850_ERDBOFF_VALUE;
        return ret;
    case WCN7850_BHIOFF:
        ret = WCN7850_BHIOFF_VALUE;
        return ret;
    case WCN7850_BHIEOFF:
        ret = WCN7850_BHIEOFF_VALUE;
        return ret;
    case WCN7850_MHICTRL:
        ret = s->mhi_state << 8;
        return ret;
    case WCN7850_MHISTATUS:
        ret = (s->mhi_state << 8) | 0x1;
        return ret;
    case WCN7850_TCSR_SOC_HW_VERSION:
        ret = 0x200;
        return ret;
    case WCN7850_BHI_EXECENV:
        ret = s->bhi_execenv;
        return ret;
    case WCN7850_BHI_STATUS:
        ret = s->bhi_status;
        return ret;
    case 0x58:
        ret = (uint32_t)(s->chan_ctxt_addr & 0xffffffffu);
        return ret;
    case 0x5c:
        ret = (uint32_t)(s->chan_ctxt_addr >> 32);
        return ret;
    case 0x60:
        ret = (uint32_t)(s->er_ctxt_addr & 0xffffffffu);
        return ret;
    case 0x64:
        ret = (uint32_t)(s->er_ctxt_addr >> 32);
        return ret;
    case 0x68:
        ret = (uint32_t)(s->cmd_ctxt_addr & 0xffffffffu);
        return ret;
    case 0x6c:
        ret = (uint32_t)(s->cmd_ctxt_addr >> 32);
        return ret;
    case 0x3008:
        /* PCIE_SOC_GLOBAL_RESET: report last written value. */
        ret = *(uint32_t *)(s->bar0_always_on + addr);
        return ret;
    case 0x3104:
        /* SoC wake/cookie register: model has no pending wake, return 0. */
        ret = 0;
        return ret;
    default:
        break;
    }

    if (addr < WCN7850_WINDOW_START) {
        memcpy(&ret, s->bar0_always_on + addr, size);
        return ret;
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
        return ret;
    }

    return ret;
}

/* Intercept REO R0 register writes in window memory (either static UMAC
 * window at 0x180000+ or dynamic window at 0x80000+).  The REO R0 blocks
 * sit at UMAC chip-offset 0x38000 + 0x4E4 (DST rings, stride 0x78) and
 * 0x38000 + 0x8A4 (exception ring).  Window offset win_off is the offset
 * within the window (= chip_addr & 0x7FFFF). */
static void wcn7850_handle_reo_r0_write(WCN7850State *s, uint32_t win_off,
                                        uint64_t val, unsigned size)
{
    /* Update window memory first */
    memcpy(s->window_memory + win_off, &val, size);

    /* REO exception ring R0 block at win_off 0x388A4 (+0x78) */
    if (win_off >= 0x388a4 && win_off < 0x388a4 + 0x78) {
        uint32_t reg_off = win_off - 0x388a4;
        if (reg_off <= 0x50) {
            uint32_t r0_base_off = win_off - reg_off;
            uint64_t base = ((uint64_t)(*(uint32_t*)(s->window_memory + r0_base_off + 0x04) & 0xff) << 32) |
                             *(uint32_t*)(s->window_memory + r0_base_off + 0x00);
            uint32_t ring_id = *(uint32_t*)(s->window_memory + r0_base_off + 0x08);
            uint32_t misc = *(uint32_t*)(s->window_memory + r0_base_off + 0x10);
            uint32_t hp_lsb = *(uint32_t*)(s->window_memory + r0_base_off + 0x14);
            uint32_t hp_msb = *(uint32_t*)(s->window_memory + r0_base_off + 0x18);
            uint32_t msi_data = *(uint32_t*)(s->window_memory + r0_base_off + 0x50);
            uint64_t hp_full = ((uint64_t)(hp_msb & 0xff) << 32) | hp_lsb;

            WCN7850RingState *r = wcn7850_add_or_update_ring(s,
                WCN7850_RING_REO_EXCEPTION, 0);
            if (r) {
                r->base_addr = base;
                r->size = ((*(uint32_t*)(s->window_memory + r0_base_off + 0x04) >> 8) & 0xfffff) * 4;
                r->entry_size = (ring_id & 0xff) * 4;
                r->enable = (misc & BIT(6)) != 0;
                r->hp_shadow_addr = hp_full;
                r->hp_mmio_offset = r0_base_off + 0x14;
                r->msivec = (msi_data & (WCN7850_MSI_VECTORS - 1)) ?: 10;
                r->configured = (r->base_addr && r->entry_size > 0 && r->size > 0);
                WCN_DBG(s, "WCN: REO_EXC ring cfg: base=0x%"PRIx64" sz=%u entry=%u"
                         " hp=0x%"PRIx64" en=%d msi=%u (win_off=0x%x reg_off=0x%x)\n",
                         base, r->size, r->entry_size, hp_full, r->enable,
                         r->msivec, win_off, reg_off);
            }
        }
        return;
    }

    /* REO DST ring R0 blocks at win_off 0x384E4 + dst_idx*0x78 (dst_idx 0..7) */
    if (win_off >= 0x384e4 && win_off < 0x384e4 + 8 * 0x78) {
        uint32_t dst_idx = (win_off - 0x384e4) / 0x78;
        uint32_t reg_off = (win_off - 0x384e4) % 0x78;
        if (reg_off <= 0x50) {
            uint32_t r0_base_off = 0x384e4 + dst_idx * 0x78;
            uint64_t base = ((uint64_t)(*(uint32_t*)(s->window_memory + r0_base_off + 0x04) & 0xff) << 32) |
                             *(uint32_t*)(s->window_memory + r0_base_off + 0x00);
            uint32_t ring_id = *(uint32_t*)(s->window_memory + r0_base_off + 0x08);
            uint32_t misc = *(uint32_t*)(s->window_memory + r0_base_off + 0x10);
            uint32_t hp_lsb = *(uint32_t*)(s->window_memory + r0_base_off + 0x14);
            uint32_t hp_msb = *(uint32_t*)(s->window_memory + r0_base_off + 0x18);
            uint32_t msi_data = *(uint32_t*)(s->window_memory + r0_base_off + 0x50);
            uint64_t hp_full = ((uint64_t)(hp_msb & 0xff) << 32) | hp_lsb;

            int model_ri = dst_idx + 1;
            WCN7850RingState *r = wcn7850_add_or_update_ring(s,
                WCN7850_RING_REO_DST, model_ri);
            if (r) {
                r->base_addr = base;
                r->size = ((*(uint32_t*)(s->window_memory + r0_base_off + 0x04) >> 8) & 0xfffff) * 4;
                r->entry_size = (ring_id & 0xff) * 4;
                r->enable = (misc & BIT(6)) != 0;
                r->hp_shadow_addr = hp_full;
                r->hp_mmio_offset = r0_base_off + 0x14;
                r->msivec = (msi_data & (WCN7850_MSI_VECTORS - 1)) ?: 11;
                r->configured = (r->base_addr && r->entry_size > 0 && r->size > 0);
                WCN_DBG(s, "WCN: REO_DST ring %d cfg: base=0x%"PRIx64" sz=%u entry=%u"
                         " hp=0x%"PRIx64" en=%d msi=%u (win_off=0x%x reg_off=0x%x)\n",
                         model_ri, base, r->size, r->entry_size, hp_full, r->enable,
                         r->msivec, win_off, reg_off);
            }
        }
        return;
    }
}

static void wcn7850_handle_tcl_window_write(WCN7850State *s,
                                             PCIDevice *pci_dev,
                                             uint32_t win_off, uint64_t val,
                                             unsigned size)
{
    const uint32_t r0_base = 0x44900;
    const uint32_t hp_base = 0x46000;

    memcpy(s->window_memory + win_off, &val, size);

    for (int i = 0; i < WCN7850_NUM_TCL_RINGS; i++) {
        uint32_t base = r0_base + i * WCN7850_SRNG_STRIDE;
        WCN7850RingState *r;

        if (win_off == hp_base + i * 4) {
            r = wcn7850_find_ring(s, WCN7850_RING_TCL_DATA, i);
            if (r) {
                WCN_DBG(s, "WCN: TXDBG WINHP%d wr val=0x%x (dwords->%u)\n", i, (uint32_t)val, (uint32_t)val * 4);
                r->hp = (uint32_t)val * 4;
                if (r->configured && r->enable)
                    wcn7850_process_tcl_data(s, pci_dev, r);
            }
            return;
        }

        if (win_off < base || win_off > base + 0x50)
            continue;

        r = wcn7850_add_or_update_ring(s, WCN7850_RING_TCL_DATA, i);
        if (!r)
            return;
        r->base_addr = *(uint32_t *)(s->window_memory + base) |
                       ((uint64_t)(*(uint32_t *)(s->window_memory + base + 4) & 0xff) << 32);
        r->size = ((*(uint32_t *)(s->window_memory + base + 4) >> 8) & 0xfffff) * 4;
        r->entry_size = (*(uint32_t *)(s->window_memory + base + 8) & 0xff) * 4;
        r->enable = (*(uint32_t *)(s->window_memory + base + 0x10) & BIT(6)) != 0;
        r->configured = r->base_addr && r->size && r->entry_size;
        WCN_DBG(s, "WCN: TCL ring %d cfg base=0x%"PRIx64" sz=%u entry=%u en=%d\n",
                 i, r->base_addr, r->size, r->entry_size, r->enable);
        return;
    }
}

static void wcn7850_handle_wbm_window_write(WCN7850State *s,
                                             PCIDevice *pci_dev,
                                             uint32_t win_off, uint64_t val,
                                             unsigned size)
{
    const uint32_t idle_link_r0 = 0x34d3c;
    const uint32_t wbm2sw_r0    = 0x34e08;

    memcpy(s->window_memory + win_off, &val, size);

    if (win_off >= idle_link_r0 && win_off < idle_link_r0 + 0x78) {
        uint32_t base = idle_link_r0;
        WCN7850RingState *r = wcn7850_add_or_update_ring(s, WCN7850_RING_WBM_IDLE_LINK, 0);
        if (r) {
            r->base_addr = *(uint32_t *)(s->window_memory + base) |
                ((uint64_t)(*(uint32_t *)(s->window_memory + base + 4) & 0xff) << 32);
            r->size = ((*(uint32_t *)(s->window_memory + base + 4) >> 8) & 0xfffff) * 4;
            r->entry_size = (*(uint32_t *)(s->window_memory + base + 8) & 0xff) * 4;
            r->enable = (*(uint32_t *)(s->window_memory + base + 0x10) & BIT(6)) != 0;
            r->configured = r->base_addr && r->size && r->entry_size;
            WCN_DBG(s, "WCN: WBM_IDLE_LINK cfg base=0x%"PRIx64" sz=%u entry=%u en=%d\n",
                     r->base_addr, r->size, r->entry_size, r->enable);
        }
        return;
    }

    if (win_off >= wbm2sw_r0 && win_off < wbm2sw_r0 + 8 * 0x78) {
        uint32_t idx = (win_off - wbm2sw_r0) / 0x78;
        uint32_t base = wbm2sw_r0 + idx * 0x78;
        uint32_t reg_off = (win_off - wbm2sw_r0) % 0x78;
        if (reg_off <= 0x50) {
            WCN7850RingState *r = wcn7850_add_or_update_ring(s, WCN7850_RING_WBM2SW_RELEASE, idx);
            if (r) {
                r->base_addr = *(uint32_t *)(s->window_memory + base) |
                    ((uint64_t)(*(uint32_t *)(s->window_memory + base + 4) & 0xff) << 32);
                r->size = ((*(uint32_t *)(s->window_memory + base + 4) >> 8) & 0xfffff) * 4;
                r->entry_size = (*(uint32_t *)(s->window_memory + base + 8) & 0xff) * 4;
                r->enable = (*(uint32_t *)(s->window_memory + base + 0x10) & BIT(6)) != 0;
                uint32_t hp_lsb = *(uint32_t *)(s->window_memory + base + 0x14);
                uint32_t hp_msb = *(uint32_t *)(s->window_memory + base + 0x18);
                uint64_t hp_shadow = ((uint64_t)(hp_msb & 0xff) << 32) | hp_lsb;
                if (hp_shadow) r->hp_shadow_addr = hp_shadow;
                uint32_t msi_data = *(uint32_t *)(s->window_memory + base + 0x50);
                r->msivec = msi_data & (WCN7850_MSI_VECTORS - 1);
                if (!r->msivec) r->msivec = 9;
                r->hp_mmio_offset = 0x30c8 + idx * 8;
                r->tp_mmio_offset = 0x30cc + idx * 8;
                r->configured = r->base_addr && r->size && r->entry_size;
                WCN_DBG(s, "WCN: WBM2SW_RELEASE ring %u cfg base=0x%"PRIx64" sz=%u entry=%u en=%d hp_shad=0x%"PRIx64" msivec=%u\n",
                         idx, r->base_addr, r->size, r->entry_size, r->enable, r->hp_shadow_addr, r->msivec);
            }
        }
        return;
    }
}

static void wcn7850_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    WCN7850State *s = opaque;
    PCIDevice *pci_dev = PCI_DEVICE(s);

    if (addr >= WCN7850_MHI_SPECIAL_BASE &&
        addr < WCN7850_MHI_SPECIAL_BASE + WCN7850_MHIREGLEN) {
        addr = WCN7850_MHIREGLEN + (addr - WCN7850_MHI_SPECIAL_BASE);
    }
    if (addr >= WCN7850_SHADOW_BASE &&
        addr < WCN7850_SHADOW_BASE + WCN7850_SHADOW_MAX * 4)
        WCN_DBG(s, "WCN: MMIO_WRITE addr=0x%lx val=0x%lx size=%u\n",
                (unsigned long)addr, (unsigned long)val, size);
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
        s->bhi_execenv = 2;
        return;
    }

    /* SoC global reset (PCIE_SOC_GLOBAL_RESET). The driver toggles the reset
     * bit during firmware (re)load; we just record it and stay in mission
     * mode. */
    if (addr == 0x3008) {
        *(uint32_t *)(s->bar0_always_on + addr) = (uint32_t)val;
        WCN_DBG(s, "WCN: SOC global reset val=0x%x\n", (uint32_t)val);
        return;
    }

    /* MHI state transition to M0 -> mission mode */
    if (addr == WCN7850_MHICTRL && ((val & 0x0000ff00u) == 0x00000200u)) {
        s->mhi_state = 2;
        wcn7850_emit_state_change_event(s, pci_dev, 2);
        s->ctrl_event_pending = true;
        timer_mod(s->ctrl_event_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000LL);
        WCN_DBG(s, "WCN: M0 -> schedule mission mode\n");
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
        WCN_DBG(s, "WCN: BHI trig pending=1 timer=5ms\n");
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
                WCN_DBG(s, "WCN: ECABAP er_ctxt=0x%lx\n",
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
        WCN_DBG(s, "WCN: SHADOW sidx=%u addr=0x%lx val=0x%lx\n",
                sidx, (unsigned long)addr, (unsigned long)val);
        /* CE shadow index → (pipe, type) mapping. type: 0=src, 1=dst, 2=sts.
         * Entries outside 29..35 are -1 (unmapped, handled by SRNG intercept). */
        static const int8_t ce_map[WCN7850_SHADOW_MAX][2] = {
            [0 ... WCN7850_SHADOW_MAX - 1] = {-1, -1},
            [29] = {0, 0}, [30] = {1, 1}, [31] = {1, 2}, [32] = {2, 1},
            [33] = {2, 2}, [34] = {3, 0}, [35] = {4, 0},
        };
        int ce_pipe = (sidx < WCN7850_SHADOW_MAX) ? ce_map[sidx][0] : -1;
        int ce_type = (sidx < WCN7850_SHADOW_MAX) ? ce_map[sidx][1] : -1;
        if (ce_pipe >= 0) {
            uint32_t doorbell_val = val;
            if (ce_type == 0) {
                s->ce_src[ce_pipe].hp = doorbell_val * 4;
                wcn7850_process_ce_src(s, pci_dev, ce_pipe);
            } else if (ce_type == 1) {
                uint32_t prev_bytes = s->ce_dst[ce_pipe].drv_hp_prev;
                uint32_t doorbell_bytes = doorbell_val * 4;
                s->ce_dst[ce_pipe].drv_hp = doorbell_bytes;
                WCN_DBG(s, "WCN: DST doorbell pipe=%d val=0x%x pending=%u seen=%d\n",
                        ce_pipe, doorbell_val, s->ce_dst[ce_pipe].pending,
                        s->ce_dst[ce_pipe].doorbell_seen);
                if (!s->ce_dst[ce_pipe].doorbell_seen) {
                    s->ce_dst[ce_pipe].doorbell_seen = true;
                    s->ce_dst[ce_pipe].pending = (s->ce_dst[ce_pipe].size ?:
                        (WCN7850_CE_DST_RING_SIZE * (s->ce_dst[ce_pipe].entry_size ?: 16)))
                        / (s->ce_dst[ce_pipe].entry_size ?: 16);
                } else if (doorbell_bytes != prev_bytes) {
                    uint32_t ring_sz = s->ce_dst[ce_pipe].size ?:
                        (WCN7850_CE_DST_RING_SIZE * (s->ce_dst[ce_pipe].entry_size ?: 16));
                    uint32_t delta = (doorbell_bytes - prev_bytes + ring_sz) % ring_sz;
                    s->ce_dst[ce_pipe].pending += delta / (s->ce_dst[ce_pipe].entry_size ?: 16);
                }
                s->ce_dst[ce_pipe].drv_hp_prev = doorbell_bytes;
                /* Driver just posted new buffers — try to flush any deferred event */
                if (s->deferred_ce_count &&
                    s->deferred_ce_events[s->deferred_ce_head].pipe == ce_pipe) {
                    WCN_DBG(s, "WCN: doorbell trigger pending flush pipe=%d\n", ce_pipe);
                    timer_mod(s->deferred_ce_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
                }
            } else if (ce_type == 2) {
            }
            return;
        }
        /* Unmapped shadow address (e.g. TCL/REO R0 config aliased into this
         * range): map known non-CE shadow indices to their real target
         * registers. Shadow index assignments are determined by the driver's
         * ring type iteration order in ath12k_hal_srng_shadow_config().
         *
         * Driver iteration (non-CE, non-DMAC/PMAC UMAC rings):
         *   idx 0-7  = REO_DST (ring 0..7, DST)  → TP (u32 write)
         *   idx 8    = REO_EXCEPTION (DST)        → TP
         *   idx 9    = REO_REINJECT (SRC)         → HP
         *   idx 10   = REO_CMD (SRC)              → HP
         *   idx 11   = REO_STATUS (DST)           → TP
         *   idx 12-16 = TCL_DATA (SRC)            → HP
         *   idx 17   = TCL_CMD (SRC)              → HP
         *   idx 18   = TCL_STATUS (DST)           → TP
         *   idx 19   = WBM_IDLE_LINK (SRC)        → HP
         *   idx 20   = SW2WBM_RELEASE (SRC)       → HP
         *   idx 21-28 = WBM2SW_RELEASE (DST)      → TP
         */
        {
            /* REO DST TP: sidx 0..7 → model_ri = sidx + 1 */
            if (sidx < 8) {
                WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_REO_DST, sidx + 1);
                if (r) {
                    WCN_DBG(s, "WCN: REO_DST ring %d TP shadow sidx=%u val=0x%x (prev=%u)\n",
                             sidx + 1, sidx, (uint32_t)val, r->tp);
                     /* UMAC SRNG pointers are dword offsets; the model's
                      * internal ring offsets are byte offsets. */
                     r->tp = val * 4;
                }
                return;
            }

            /* sidx 8: REO_EXCEPTION TP */
            if (sidx == 8) {
                WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_REO_EXCEPTION, 0);
                if (r) {
                    WCN_DBG(s, "WCN: REO_EXC TP shadow sidx=%u val=0x%x\n", sidx, (uint32_t)val);
                    r->tp = val;
                }
                return;
            }

            /* sidx 9, 10, 11: REO rings — tracked by the existing MMIO
             * and UMAC window handlers. These sidx writes update the
             * ring state in bar0_always_on. */
            if (sidx >= 9 && sidx <= 11) {
                WCN_DBG(s, "WCN: REO ring sidx=%u val=0x%x\n", sidx, (uint32_t)val);
                return;
            }

            /* sidx 12-16: TCL_DATA HP → trigger TX processing */
            if (sidx >= 12 && sidx <= 16) {
                WCN_DBG(s, "WCN: TCL_DATA handler REACHED sidx=%u\n", sidx);
                int tcl_idx = sidx - 12;
                WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_TCL_DATA, tcl_idx);
                if (r) {
                    uint32_t hp_bytes = (uint32_t)val * 4;
                    WCN_DBG(s, "WCN: TCL_DATA %d HP shadow sidx=%u val=0x%x hp=%u\n",
                            tcl_idx, sidx, (uint32_t)val, hp_bytes);
                    r->hp = hp_bytes;
                    if (r->configured && r->enable) {
                        wcn7850_process_tcl_data(s, PCI_DEVICE(s), r);
                    }
                }
                return;
            }

            /* sidx 17, 18: TCL CMD/TCL STATUS — not tracked by model enum;
             * just log and store. */
            if (sidx >= 17 && sidx <= 18) {
                WCN_DBG(s, "WCN: TCL sidx=%u val=0x%x (unhandled)\n", sidx, (uint32_t)val);
                return;
            }

            /* sidx 19: WBM_IDLE_LINK HP — driver added idle link descriptors */
            if (sidx == 19) {
                WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_WBM_IDLE_LINK, 0);
                if (r && r->configured && r->enable) {
                    r->hp = (uint32_t)val * 4;
                }
                return;
            }

            /* sidx 20: SW2WBM_RELEASE HP — driver releasing buffers to HW */
            if (sidx == 20) {
                WCN_DBG(s, "WCN: SW2WBM_RELEASE HP sidx=20 val=0x%x\n", (uint32_t)val);
                return;
            }

            /* sidx 21-28: WBM2SW_RELEASE TP — driver consumed completions */
            if (sidx >= 21 && sidx <= 28) {
                int wbm_idx = sidx - 21;
                WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_WBM2SW_RELEASE, wbm_idx);
                if (r) {
                    r->tp = (uint32_t)val * 4;
                }
                return;
            }
        }
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
             const Wcn7850RingRegisters regs = {
                 WCN7850_TCL_RING_BASE_LSB(i), WCN7850_TCL_RING_BASE_MSB(i),
                 WCN7850_TCL_RING_ID(i), WCN7850_TCL_RING_MISC(i),
                 WCN7850_TCL_RING_MSI1_DATA(i), WCN7850_TCL_RING_HP(i),
                 WCN7850_TCL_RING_TP(i), 8
             };
             wcn7850_update_ring_cfg(s, WCN7850_RING_TCL_DATA, i, "TCL", &regs);
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
             Wcn7850RingType rtype = wcn7850_reo_ring_type(i);
             const Wcn7850RingRegisters regs = {
                 WCN7850_REO_RING_BASE_LSB(i), WCN7850_REO_RING_BASE_MSB(i),
                 WCN7850_REO_RING_ID(i), WCN7850_REO_RING_MISC(i),
                 WCN7850_REO_RING_MSI1_DATA(i), WCN7850_REO_RING_HP(i),
                 WCN7850_REO_RING_TP(i), (i == 0) ? 10 : 9
             };
             wcn7850_update_ring_cfg(s, rtype, i, "REO", &regs);
            return;
        }
        if (addr == WCN7850_REO_RING_HP_ADDR_LSB(i) ||
            addr == WCN7850_REO_RING_HP_ADDR_MSB(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
             Wcn7850RingType rtype = wcn7850_reo_ring_type(i);
            WCN7850RingState *r = wcn7850_find_ring(s, rtype, i);
            if (r) {
                uint32_t hp_lsb = *(uint32_t *)(s->bar0_always_on + WCN7850_REO_RING_HP_ADDR_LSB(i));
                uint32_t hp_msb = *(uint32_t *)(s->bar0_always_on + WCN7850_REO_RING_HP_ADDR_MSB(i));
                uint64_t new_addr = ((uint64_t)(hp_msb & 0xff) << 32) | hp_lsb;
                if (new_addr && new_addr != r->hp_shadow_addr) {
                    r->hp_shadow_addr = new_addr;
                    WCN_DBG(s, "WCN: REO ring %d HP_RDP addr=0x%"PRIx64"\n",
                             i, r->hp_shadow_addr);
                }
            }
            return;
        }
    }

    /* REO command ring R0 config */
    if (addr == WCN7850_REO_CMD_BASE_LSB ||
        addr == WCN7850_REO_CMD_BASE_MSB ||
        addr == WCN7850_REO_CMD_RING_ID ||
        addr == WCN7850_REO_CMD_MISC) {
        memcpy(s->bar0_always_on + addr, &val, size);
         const Wcn7850RingRegisters regs = {
             WCN7850_REO_CMD_BASE_LSB, WCN7850_REO_CMD_BASE_MSB,
             WCN7850_REO_CMD_RING_ID, WCN7850_REO_CMD_MISC, 0,
             WCN7850_REO_CMD_HP, WCN7850_REO_CMD_TP, 0
         };
         wcn7850_update_ring_cfg(s, WCN7850_RING_REO_CMD, 0, "REO_CMD", &regs);
        return;
    }

    /* REO status ring R0 config */
    if (addr == WCN7850_REO_STATUS_BASE_LSB ||
        addr == WCN7850_REO_STATUS_BASE_MSB ||
        addr == WCN7850_REO_STATUS_RING_ID ||
        addr == WCN7850_REO_STATUS_MISC) {
        memcpy(s->bar0_always_on + addr, &val, size);
         const Wcn7850RingRegisters regs = {
             WCN7850_REO_STATUS_BASE_LSB, WCN7850_REO_STATUS_BASE_MSB,
             WCN7850_REO_STATUS_RING_ID, WCN7850_REO_STATUS_MISC, 0,
             WCN7850_REO_STATUS_HP, WCN7850_REO_STATUS_TP, 0
         };
         wcn7850_update_ring_cfg(s, WCN7850_RING_REO_STATUS, 0,
                                 "REO_STATUS", &regs);
        return;
    }

    /* WBM2SW release ring config registers (R0) */
    for (int i = 0; i < WCN7850_NUM_WBM_RINGS; i++) {
        if (addr == WCN7850_WBM_RING_BASE_LSB(i) ||
            addr == WCN7850_WBM_RING_BASE_MSB(i) ||
            addr == WCN7850_WBM_RING_ID(i) ||
            addr == WCN7850_WBM_RING_MISC(i) ||
            addr == WCN7850_WBM_RING_MSI1_DATA(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
             const Wcn7850RingRegisters regs = {
                 WCN7850_WBM_RING_BASE_LSB(i), WCN7850_WBM_RING_BASE_MSB(i),
                 WCN7850_WBM_RING_ID(i), WCN7850_WBM_RING_MISC(i),
                 WCN7850_WBM_RING_MSI1_DATA(i), WCN7850_WBM_RING_HP(i),
                 WCN7850_WBM_RING_TP(i), 9
             };
             wcn7850_update_ring_cfg(s, WCN7850_RING_WBM2SW_RELEASE, i,
                                     "WBM2SW", &regs);
            return;
        }
    }

    /* TCL HP write (SW producing TX descriptors → trigger TX processing) */
    for (int i = 0; i < WCN7850_NUM_TCL_RINGS; i++) {
        if (addr == WCN7850_TCL_RING_HP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_TCL_DATA, i);
            if (r) {
                WCN_DBG(s, "WCN: TXDBG R2HP%d wr val=0x%x (dwords->%u)\n", i, (uint32_t)val, (uint32_t)val * 4);
                r->hp = val * 4;
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
                WCN_DBG(s, "WCN: TXDBG R2TP%d wr val=0x%x (dwords->%u)\n", i, (uint32_t)val, (uint32_t)val * 4);
                r->tp = val * 4;
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
                /* UMAC SRNG R2 pointers are dword offsets; the model's
                 * internal ring offsets are byte offsets (see the RX
                 * injection path), so convert like the shadow-RD path. */
                r->hp = (i == 0) ? val : (uint32_t)val * 4;
            }
            return;
        }
        if (addr == WCN7850_REO_RING_TP(i)) {
            memcpy(s->bar0_always_on + addr, &val, size);
            WCN7850RingState *r = (i == 0)
                ? wcn7850_find_ring(s, WCN7850_RING_REO_EXCEPTION, 0)
                : wcn7850_find_ring(s, WCN7850_RING_REO_DST, i);
            if (r) {
                WCN_DBG(s, "WCN: REO DST ring %d TP written=%u (prev=%u)\n",
                         i, (uint32_t)val, r->tp);
                r->tp = (i == 0) ? val : (uint32_t)val * 4;
            }
            return;
        }
    }

    /* REO command ring HP/TP */
    if (addr == WCN7850_REO_CMD_HP) {
        memcpy(s->bar0_always_on + addr, &val, size);
        WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_REO_CMD, 0);
        if (r) {
            r->hp = val;
        }
        return;
    }
    if (addr == WCN7850_REO_CMD_TP) {
        memcpy(s->bar0_always_on + addr, &val, size);
        WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_REO_CMD, 0);
        if (r) {
            r->tp = val;
        }
        return;
    }

    /* REO status ring HP/TP */
    if (addr == WCN7850_REO_STATUS_HP) {
        memcpy(s->bar0_always_on + addr, &val, size);
        WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_REO_STATUS, 0);
        if (r) {
            r->hp = val;
        }
        return;
    }
    if (addr == WCN7850_REO_STATUS_TP) {
        memcpy(s->bar0_always_on + addr, &val, size);
        WCN7850RingState *r = wcn7850_find_ring(s, WCN7850_RING_REO_STATUS, 0);
        if (r) {
            r->tp = val;
        }
        return;
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

    /* SW2WBM_RELEASE ring R0/R2 registers — driver returns RX buffers here */
    {
        static const struct {
            uint32_t base_lsb, base_msb, ring_id, misc;
            uint32_t hp;
            int ring_idx;  /* 0=SW0, 1=SW1 (model convention, not SRNG ring_id) */
        } sw2wbm_cfg[] = {
            { WCN7850_SW2WBM0_BASE_LSB, WCN7850_SW2WBM0_BASE_MSB,
              WCN7850_SW2WBM0_RING_ID, WCN7850_SW2WBM0_MISC,
              WCN7850_SW2WBM0_HP, 0 },
            { WCN7850_SW2WBM1_BASE_LSB, WCN7850_SW2WBM1_BASE_MSB,
              WCN7850_SW2WBM1_RING_ID, WCN7850_SW2WBM1_MISC,
              WCN7850_SW2WBM1_HP, 1 },
        };
        for (int i = 0; i < 2; i++) {
            if (addr == sw2wbm_cfg[i].base_lsb ||
                addr == sw2wbm_cfg[i].base_msb ||
                addr == sw2wbm_cfg[i].ring_id ||
                addr == sw2wbm_cfg[i].misc) {
                memcpy(s->bar0_always_on + addr, &val, size);
                uint32_t lsb = *(uint32_t *)(s->bar0_always_on + sw2wbm_cfg[i].base_lsb);
                uint32_t msb = *(uint32_t *)(s->bar0_always_on + sw2wbm_cfg[i].base_msb);
                uint32_t rid = *(uint32_t *)(s->bar0_always_on + sw2wbm_cfg[i].ring_id);
                WCN7850RingState *r = wcn7850_add_or_update_ring(
                    s, WCN7850_RING_SW2WBM_RELEASE, sw2wbm_cfg[i].ring_idx);
                if (r) {
                    r->configured = true;
                    r->base_addr = lsb | ((uint64_t)(msb & 0xff) << 32);
                    r->entry_size = (rid & 0xff) * 4;
                    if (!r->entry_size) r->entry_size = 32;
                    r->size = ((msb >> 8) & 0x7fff) * r->entry_size;
                    r->hp_shadow_addr = 0; /* shadow-based HP not tracked */
                    r->enable = true;
                    WCN_DBG(s, "WCN: SW2WBM ring %d base=0x%"PRIx64" esz=%u sz=%u\n",
                             sw2wbm_cfg[i].ring_idx, r->base_addr,
                             r->entry_size, r->size);
                }
                return;
            }
            if (addr == sw2wbm_cfg[i].hp) {
                memcpy(s->bar0_always_on + addr, &val, size);
                WCN7850RingState *r = wcn7850_find_ring(
                    s, WCN7850_RING_SW2WBM_RELEASE, sw2wbm_cfg[i].ring_idx);
                if (r && r->configured && r->enable && r->base_addr) {
                    uint32_t old_hp = r->hp;
                    r->hp = val;
                    if (r->hp != old_hp) {
                        /* Process release entries between TP and new HP */
                        uint32_t tp = r->tp;
                        uint32_t esize = r->entry_size ?: 32;
                        uint32_t ring_bytes = r->size ?: (512 * esize);
                        int32_t diff;
                        if (r->hp >= tp) {
                            diff = r->hp - tp;
                        } else {
                            diff = (ring_bytes - tp) + r->hp;
                        }
                        int num_entries = diff / (int)esize;
                        if (num_entries > 0 && num_entries <= 256) {
                            WCN_DBG(s, "WCN: SW2WBM ring %d HP=%u old=%u tp=%u entries=%d\n",
                                     sw2wbm_cfg[i].ring_idx, r->hp, old_hp, tp, num_entries);
                            for (int j = 0; j < num_entries; j++) {
                                uint64_t desc_addr = r->base_addr + tp;
                                uint8_t entry[32];
                                if (pci_dma_read(pci_dev, desc_addr, entry,
                                                 esize > 32 ? 32 : esize) != MEMTX_OK) {
                                    break;
                                }
                                uint32_t info0 = le32_to_cpu(*(uint32_t *)entry);
                                uint32_t info1 = le32_to_cpu(*(uint32_t *)(entry + 4));
                                uint64_t buf_addr = info0 |
                                    ((uint64_t)(info1 & 0xff) << 32);
                                uint32_t cookie = (info1 >> 12) & 0xfffff;
                                /* The driver released this RX buffer back to
                                 * the WBM.  Do NOT recycle it into the
                                 * HW-facing RXDMA buf rings: the driver
                                 * re-posts fresh buffers via SW2RXDMA_BUF0
                                 * after reaping, and recycling here re-injects
                                 * the same cookie forever (NULL-skb crash on
                                 * the second reap before the driver refills).
                                 * Merely acknowledge the release by advancing
                                 * the TP below. */
                                if (buf_addr) {
                                    WCN_DBG(s, "WCN: SW2WBM released buf=0x%"PRIx64
                                             " cookie=0x%x (ack)\n",
                                             buf_addr, cookie);
                                }
                                tp += esize;
                                if (tp >= ring_bytes) tp = 0;
                            }
                            r->tp = tp;
                        }
                    }
                }
                return;
            }
        }
    }

    if (addr < WCN7850_WINDOW_START) {
        memcpy(s->bar0_always_on + addr, &val, size);
        return;
    }

    /* Static UMAC window (static_window_map=true): UMAC -> 0x180000,
     * accessed directly without the window register.  For WCN7850 with
     * static_window_map=false this path is NOT used; the driver uses the
     * dynamic window at BAR 0x80000 instead (handled below). */
    if (addr >= WCN7850_UMAC_WINDOW_BASE &&
        addr < WCN7850_UMAC_WINDOW_BASE + WCN7850_WINDOW_SIZE) {
        uint32_t win_off = addr & (WCN7850_WINDOW_SIZE - 1);
        wcn7850_handle_reo_r0_write(s, win_off, val, size);
        return;
    }
    if (addr >= WCN7850_CE_WINDOW_BASE &&
        addr < WCN7850_CE_WINDOW_BASE + WCN7850_WINDOW_SIZE) {
        WCN_DBG(s, "WCN: CE win write addr=0x%lx off=0x%lx val=0x%x sz=%u\n",
                 (unsigned long)addr, (unsigned long)(addr & (WCN7850_WINDOW_SIZE - 1)),
                 (uint32_t)val, size);
        wcn7850_handle_ce_mmio(s, pci_dev, addr & (WCN7850_WINDOW_SIZE - 1),
                               val, size);
        return;
    }

    /* Dynamic window region at 0x80000, selected via the window register.
     * WCN7850 (static_window_map=false) uses this path for ALL register
     * accesses beyond 0x80000, including the REO R0 configuration registers
     * (R0 blocks at window offset 0x384E4+ for DST rings, 0x388A4 for
     * exception ring). */
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
        if ((win_off >= 0x44900 && win_off < 0x44900 + 6 * 0x78) ||
            (win_off >= 0x46000 && win_off < 0x46000 + 6 * 4)) {
            wcn7850_handle_tcl_window_write(s, pci_dev, win_off, val, size);
            return;
        }
        if (win_off >= 0x34d3c && win_off < 0x34e08 + 8 * 0x78) {
            wcn7850_handle_wbm_window_write(s, pci_dev, win_off, val, size);
            return;
        }
        /* Intercept REO R0 register writes in the window memory */
        wcn7850_handle_reo_r0_write(s, win_off, val, size);
        return;
    }
}

/* ===== After mission mode, send FW_READY_IND and NEW_SERVER ===== */
static bool wcn7850_mission_mode_setup(WCN7850State *s, PCIDevice *pci_dev)
{
    WCN_DBG(s, "WCN: mission_mode_setup er=0x%lx chan=0x%lx\n",
            (unsigned long)s->er_ctxt_addr,
            (unsigned long)s->chan_ctxt_addr);
    if (!s->er_ctxt_addr || !s->chan_ctxt_addr)
        return false;

    /* NEW_SERVER is now sent by the DL doorbell handler once the host
     * queues DL buffers and rings the doorbell.  Here we just mark that
     * we're ready to start the boot handshake. */
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

    WCN_DBG(s, "WCN: timer fire ctxt=0x%lx pending=%d\n",
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

/* Timer callback: retry sending a deferred WMI event that failed due to
 * an empty CE dst ring (see wcn7850_ce_send). */
static void wcn7850_deferred_ce_timer(void *opaque)
{
    WCN7850State *s = opaque;
    if (!s->deferred_ce_count)
        return;
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t scanned = 0;
    bool delivered = false;
    while (scanned < s->deferred_ce_count) {
        uint32_t event_idx = (s->deferred_ce_head + scanned) %
                             WCN7850_PENDING_CE_EVENTS;
        uint32_t pipe = s->deferred_ce_events[event_idx].pipe;
        uint32_t event_len = s->deferred_ce_events[event_idx].len;
        if (wcn7850_ce_deliver(s, pci_dev, pipe,
                               s->deferred_ce_events[event_idx].data,
                               event_len)) {
            if (event_idx != s->deferred_ce_head) {
                DeferredCeEvent completed = s->deferred_ce_events[event_idx];
                for (uint32_t i = scanned; i > 0; i--) {
                    uint32_t dst = (s->deferred_ce_head + i) %
                                   WCN7850_PENDING_CE_EVENTS;
                    uint32_t src = (s->deferred_ce_head + i - 1) %
                                   WCN7850_PENDING_CE_EVENTS;
                    s->deferred_ce_events[dst] = s->deferred_ce_events[src];
                }
                s->deferred_ce_events[s->deferred_ce_head] = completed;
            }
            s->deferred_ce_head = (s->deferred_ce_head + 1) %
                                  WCN7850_PENDING_CE_EVENTS;
            s->deferred_ce_count--;
            delivered = true;
            break;
        }
        scanned++;
    }
    if (s->deferred_ce_count) {
        timer_mod(s->deferred_ce_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  (delivered ? 1 : 100000));
    }
}

/* Poll RXDMA buf rings: read driver-posted HP, consume new buffer
 * descriptors (8-byte DMA addresses), and add them to the free list. */
static void wcn7850_poll_rxdma_buf_rings(WCN7850State *s, PCIDevice *pci_dev)
{
    /* Read HP/TP for all three RXDMA buf rings
     * Ring 0: refill ring (HOST1_TO_FW_RXBUF_RING) 
     * Ring 1: per-MAC buf ring 1 (RXDMA_HOST_BUF_RING)
     * Ring 2: per-MAC buf ring 2 (RXDMA_HOST_BUF_RING)
     */
    for (int i = 0; i < s->num_rings; i++) {
        WCN7850RingState *r = &s->rings[i];
        if (!r->configured || !r->enable || r->type != WCN7850_RING_RXDMA_BUF) {
            continue;
        }
        if (!r->hp_shadow_addr || !r->base_addr) {
            continue;
        }

        uint32_t hp_le = 0;
        if (pci_dma_read(pci_dev, r->hp_shadow_addr, &hp_le, 4) != MEMTX_OK) {
            continue;
        }
        /* The driver publishes the SRNG HP in dword offsets; the model's
         * internal ring offsets are byte offsets.  Convert before comparing
         * against the byte-units TP below. */
        uint32_t hp_val = le32_to_cpu(hp_le);
        uint32_t hp = hp_val * 4;
        uint32_t tp = r->tp;
        uint32_t esize = r->entry_size ?: 8;
        uint32_t ring_bytes = r->size ?: (4096 * esize);

        if (hp == tp) {
            continue;
        }

        /* Number of new entries = (hp - tp) / esize, handling wrap */
        int32_t diff;
        if (hp >= tp) {
            diff = hp - tp;
        } else {
            diff = (ring_bytes - tp) + hp;
        }
        int num_entries = diff / (int)esize;
        if (num_entries <= 0) {
            continue;
        }

        /* Cap to available space in our tracking array */
        if (num_entries > WCN7850_RXDMA_MAX_BUFS - (int)r->num_bufs) {
            num_entries = WCN7850_RXDMA_MAX_BUFS - (int)r->num_bufs;
        }
        if (num_entries <= 0) {
            continue;
        }

        int consumed = 0;
        WCN_DBG(s, "WCN: RXBUF ring%d hp_dw=%u hp=%u tp=%u esize=%u ring_bytes=%u num_entries=%d base=0x%"PRIx64"\n",
                 r->ring_id, hp_val, hp, tp, esize, ring_bytes, num_entries, r->base_addr);
        for (int j = 0; j < num_entries; j++) {
            uint64_t desc_addr = r->base_addr + tp;
            uint8_t desc[8];
            if (pci_dma_read(pci_dev, desc_addr, desc, esize > 8 ? 8 : esize) != MEMTX_OK) {
                break;
            }
            if (j < 12) {
                WCN_DBG(s, "WCN: RXBUF ring%d entry[%d] tp=%u lo=%08x hi=%08x addr=%"PRIx64" cookie=%x\n",
                         r->ring_id, j, tp, le32_to_cpu(*(uint32_t *)desc),
                         le32_to_cpu(*(uint32_t *)(desc + 4)),
                         ((uint64_t)(le32_to_cpu(*(uint32_t *)(desc + 4)) & 0xff) << 32) |
                         le32_to_cpu(*(uint32_t *)desc),
                         le32_to_cpu(*(uint32_t *)(desc + 4)) >> 12);
            }
            uint64_t buf_addr;
            uint32_t buf_cookie = 0;
            if (esize >= 8) {
                uint32_t lo = le32_to_cpu(*(uint32_t *)desc);
                uint32_t hi = le32_to_cpu(*(uint32_t *)(desc + 4));
                buf_addr = ((uint64_t)(hi & 0xff) << 32) | lo;
                buf_cookie = hi >> 12;
            } else if (esize == 4) {
                buf_addr = le32_to_cpu(*(uint32_t *)desc);
            } else {
                buf_addr = 0;
            }

            if (buf_addr && buf_cookie) {
                r->buf_addrs[r->num_bufs++] = buf_addr;
                r->buf_cookies[r->num_bufs - 1] = buf_cookie;
            }

            tp += esize;
            if (tp >= ring_bytes) {
                tp = 0;
            }
            consumed++;
        }

        if (consumed > 0) {
            r->tp = tp;
            uint32_t tp_dw = tp / 4;
            uint32_t tp_le = cpu_to_le32(tp_dw);
            if (r->tp_shadow_addr) {
                pci_dma_write(pci_dev, r->tp_shadow_addr, &tp_le, sizeof(tp_le));
            }
        }
    }

    WCN7850RingState *ring0 = wcn7850_find_ring(s, WCN7850_RING_RXDMA_BUF, 0);
    WCN7850RingState *ring1 = wcn7850_find_ring(s, WCN7850_RING_RXDMA_BUF, 1);
    WCN7850RingState *ring2 = wcn7850_find_ring(s, WCN7850_RING_RXDMA_BUF, 2);
    if (!ring0) WCN_DBG(s, "WCN: DIST ring0=NULL\n");
    if (!ring1) WCN_DBG(s, "WCN: DIST ring1=NULL\n");
    if (!ring2) WCN_DBG(s, "WCN: DIST ring2=NULL\n");
    
    if (ring0 && ring1 && ring2 && ring0->num_bufs > 0 &&
        (ring1->num_bufs < WCN7850_RXDMA_MAX_BUFS || ring2->num_bufs < WCN7850_RXDMA_MAX_BUFS)) {
        for (int j = 0; j < (int)ring0->num_bufs && (ring1->num_bufs < WCN7850_RXDMA_MAX_BUFS || ring2->num_bufs < WCN7850_RXDMA_MAX_BUFS); j++) {
            uint64_t buf_addr = ring0->buf_addrs[j];
            
            if (buf_addr) {
                if (ring1->num_bufs < WCN7850_RXDMA_MAX_BUFS) {
                    ring1->buf_addrs[ring1->num_bufs] = buf_addr;
                    ring1->buf_cookies[ring1->num_bufs++] = ring0->buf_cookies[j];
                } else if (ring2->num_bufs < WCN7850_RXDMA_MAX_BUFS) {
                    ring2->buf_addrs[ring2->num_bufs] = buf_addr;
                    ring2->buf_cookies[ring2->num_bufs++] = ring0->buf_cookies[j];
                }
            }
        }
        
        /* Clear ring0's tracked buffers after distribution */
        ring0->num_bufs = 0;
    }

    /* RX frame injection into available HW-facing buffers */
    ring1 = wcn7850_find_ring(s, WCN7850_RING_RXDMA_BUF, 1);
    ring2 = wcn7850_find_ring(s, WCN7850_RING_RXDMA_BUF, 2);
    if (s->rx_count > 0 &&
        ((ring1 && ring1->num_bufs > 0) || (ring2 && ring2->num_bufs > 0))) {
        WCN_DBG(s, "WCN: Injecting fake RX frame into available HW-facing buffers\n");
        
        /* Choose a buffer from either ring 1 or 2 */
        WCN7850RingState *selected_ring = NULL;
        int selected_idx = -1;
        
        if (ring1 && ring1->num_bufs > 0) {
            selected_ring = ring1;
            selected_idx = ring1->ring_id;
        } else if (ring2 && ring2->num_bufs > 0) {
            selected_ring = ring2;
            selected_idx = ring2->ring_id;
        }
        
        if (selected_ring && selected_idx >= 0) {
            uint64_t buf_addr = selected_ring->buf_addrs[selected_ring->num_bufs - 1];
            uint32_t buf_cookie = selected_ring->buf_cookies[selected_ring->num_bufs - 1];

            uint8_t *pkt_data = NULL;
            size_t pkt_len = 0;

            if (s->rx_count > 0 && s->rx_frames[s->rx_head].len > 0) {
                pkt_data = s->rx_frames[s->rx_head].data;
                pkt_len = s->rx_frames[s->rx_head].len;
            } else {
                g_assert_not_reached();
            }
            /* Driver-posted buffers may be ordinary guest DMA addresses rather
             * than addresses in the model's optional pool region. */
             size_t copy_size = pkt_len < WCN7850_POOL_BUF_SIZE ? pkt_len : WCN7850_POOL_BUF_SIZE;
             const size_t rx_payload_offset = 400;
             uint8_t *rx_buf = g_malloc0(rx_payload_offset + copy_size);
            uint32_t msdu_end_tag = cpu_to_le32((197u << 1) | (36u << 10));
            uint32_t mpdu_start_tag = cpu_to_le32((194u << 1) | (61u << 10));
            uint16_t info5 = cpu_to_le16((1u << 12) | (1u << 13) |
                                         (1u << 7) | (1u << 8));
            uint32_t info10 = cpu_to_le32(copy_size & 0x3fffu);
             /* Ethernet offload delivers an Ethernet II frame. */
             uint32_t info11 = cpu_to_le32(2u << 8);
            uint32_t info14 = cpu_to_le32(1u << 31);
             uint32_t mpdu_info4 = cpu_to_le32((1u << 0) | (1u << 1) |
                                               (1u << 2) | (1u << 3) |
                                               (1u << 4) | (1u << 6));
             uint32_t mpdu_info5 = cpu_to_le32(2u << 10);
             uint32_t mpdu_info6 = cpu_to_le32((copy_size & 0x3fffu) |
                                               (1u << 14));
             uint16_t mpdu_peer_id = cpu_to_le16(s->peer_id);
            memcpy(rx_buf, &msdu_end_tag, sizeof(msdu_end_tag));
             memcpy(rx_buf + 50, &info5, sizeof(info5));
             memcpy(rx_buf + 80, &info10, sizeof(info10));
             memcpy(rx_buf + 84, &info11, sizeof(info11));
              memcpy(rx_buf + 132, &info14, sizeof(info14));
              memcpy(rx_buf + 194, &mpdu_peer_id, sizeof(mpdu_peer_id));
              memcpy(rx_buf + 196, &mpdu_info4, sizeof(mpdu_info4));
              memcpy(rx_buf + 200, &mpdu_info5, sizeof(mpdu_info5));
              memcpy(rx_buf + 204, &mpdu_info6, sizeof(mpdu_info6));
              memcpy(rx_buf + 212, pkt_data, 6);
              memcpy(rx_buf + 218, pkt_data + 6, 6);
              memcpy(rx_buf + 224, pkt_data, 6);
              memcpy(rx_buf + 144, &mpdu_start_tag, sizeof(mpdu_start_tag));
              memcpy(rx_buf + rx_payload_offset, pkt_data, copy_size);
             if ((uintptr_t)buf_addr >= WCN7850_POOL_PADDR &&
                 (uintptr_t)buf_addr < WCN7850_POOL_PADDR + WCN7850_POOL_SIZE) {
                 uintptr_t pool_offset = buf_addr - WCN7850_POOL_PADDR;
                 size_t write_size = rx_payload_offset + copy_size;
                 if (pool_offset > WCN7850_POOL_SIZE ||
                     write_size > WCN7850_POOL_SIZE - pool_offset) {
                     WCN_DBG(s, "WCN: RX pool write out of bounds addr=0x%" PRIx64
                             " len=%zu\n", buf_addr, write_size);
                 } else {
                     memcpy(s->pool_mem + pool_offset, rx_buf, write_size);
                 }
             } else if (pci_dma_write(pci_dev, buf_addr, rx_buf,
                                      rx_payload_offset + copy_size) != MEMTX_OK) {
                  WCN_DBG(s, "WCN: RX packet DMA write failed at 0x%"PRIx64" len=%zu\n",
                           buf_addr, rx_payload_offset + copy_size);
             }
             g_free(rx_buf);

            memset(s->rx_frames[s->rx_head].data, 0,
                   sizeof(s->rx_frames[s->rx_head].data));
            s->rx_frames[s->rx_head].len = 0;
            s->rx_head = (s->rx_head + 1) % WCN7850_PENDING_RX_FRAMES;
            s->rx_count--;

            /* Mark selected buffer as used */
            selected_ring->num_bufs--;
            
            /* Post RX completion to the first UMAC REO destination ring
             * (model_ri=1, configured by DP init via MMIO UMAC R0 window).
             * model_ri=0 is the HTT RXDMA_DST ring and must NOT be used
             * for data RX completions — the driver's NAPI polls the UMAC
             * REO DST rings, not the RXDMA DST ring. */
            WCN7850RingState *reo_dst = NULL;
            for (int reo_id = 1; reo_id <= WCN7850_NUM_REO_RINGS; reo_id++) {
                WCN7850RingState *candidate = wcn7850_find_ring(
                    s, WCN7850_RING_REO_DST, reo_id);
                if (candidate && candidate->base_addr && candidate->enable) {
                    reo_dst = candidate;
                    break;
                }
            }
            if (reo_dst && reo_dst->base_addr) {
                uint32_t esize = reo_dst->entry_size ?: 32; /* REO DST entries are 32 bytes */
                
                /* Format the REO destination ring entry (32 bytes = 8 dwords).
                 * The RX buffer-ring upper word carries the driver's cookie,
                 * not the high address bits beyond bit 39. */
                uint32_t entry[8] = {0};
                
                /* buf_addr_info: lower 32 bits of buffer address */
                entry[0] = cpu_to_le32(buf_addr & 0xffffffff);
                /* buf_addr_info: upper 32 bits + meta (pdev_id, rbm, etc.) */
                entry[1] = cpu_to_le32(((buf_addr >> 32) & 0xff) |
                                       (buf_cookie << 12));
                /* rx_mpdu_info: one MSDU, with a valid peer id. */
                entry[2] = cpu_to_le32(1);
                entry[3] = cpu_to_le32((uint32_t)s->peer_id);
                /* rx_msdu_info: first/last MSDU, decapped Ethernet payload. */
                 entry[4] = cpu_to_le32(1 | (1 << 1) |
                                         ((uint32_t)pkt_len << 3) |
                                         (1 << 18) | (1 << 19) | (3 << 29));
                /* buf_va is zero so the driver resolves the descriptor by cookie. */
                /* info0: MSDU buffer, routing-instruction push reason, and
                 * source link 0.  The packet is written directly to the
                 * driver-posted buffer above, so advertising a link
                 * descriptor would make ath12k interpret packet bytes as a
                 * link descriptor and lose the buffer cookie. */
                entry[7] = cpu_to_le32(1 << 1);
                
                /* Write entry at current HP (producer position for DST ring) */
                uint64_t write_addr = reo_dst->base_addr + reo_dst->hp;
                pci_dma_write(pci_dev, write_addr, entry, esize);
                
                /* Advance HP and write to hp_shadow_addr (DST ring: HW writes HP).
                 * UMAC SRNG pointers are dword offsets; the model's internal
                 * byte offset must be divided by 4 before publishing to the
                 * driver so it compares HP against its dword-units TP. */
                reo_dst->hp += esize;
                if (reo_dst->hp >= reo_dst->size) {
                    reo_dst->hp = 0;
                }
                
                uint32_t hp_dw = reo_dst->hp / 4;
                uint32_t hp_le = cpu_to_le32(hp_dw);
                if (reo_dst->hp_shadow_addr) {
                    pci_dma_write(pci_dev, reo_dst->hp_shadow_addr, &hp_le, sizeof(hp_le));
                }
                /* Keep the BAR0 mirror synchronized with the hardware head
                 * register that ath12k reads for this emulated ring. */
                if (reo_dst->hp_mmio_offset) {
                    memcpy(s->window_memory + reo_dst->hp_mmio_offset,
                           &hp_le, sizeof(hp_le));
                    memcpy(s->bar0_always_on + WCN7850_WINDOW_START +
                           reo_dst->hp_mmio_offset, &hp_le, sizeof(hp_le));
                }
                /* ath12k reads UMAC destination HP from the shadow table
                 * (REO2SW1 is shadow slot 0), not from the R0 mirror. */
                if (reo_dst->ring_id >= 1 && reo_dst->ring_id <= WCN7850_NUM_REO_RINGS) {
                    memcpy(s->bar0_always_on + WCN7850_SHADOW_BASE +
                           (reo_dst->ring_id - 1) * sizeof(hp_le),
                           &hp_le, sizeof(hp_le));
                }
                
                uint32_t reo_msivec = reo_dst->msivec;
                if (!reo_msivec) {
                    reo_msivec = WCN7850_DP_MSI_BASE + 3; /* ext_irq group 3 (REO DST ring 0) */
                }
                WCN_DBG(s, "WCN: RX completion posted, REO DST HP updated to %u, cookie=0x%x, msivec=%u\n",
                         reo_dst->hp, buf_cookie, reo_msivec);
                if (reo_msivec < WCN7850_MSI_VECTORS) {
                    msi_notify(pci_dev, reo_msivec);
                }
            }
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
            WCN_DBG(s, "WCN: DL ring has buffers, retrying NEW_SERVER\n");
            if (wcn7850_send_new_server(s, pci_dev, s->chan_ctxt_addr,
                                         s->er_ctxt_addr)) {
                s->qmi_newserver_resent = true;
                WCN_DBG(s, "WCN: NEW_SERVER retry succeeded\n");
            }
        }
    }

    /* Retry pending QMI indication if DL ring has buffers */
    if (s->qmi_pending_ind && s->chan_ctxt_addr) {
        MhiChanCtxt dl_ctxt;
        uint64_t dl_ctxt_addr = s->chan_ctxt_addr + MHI_CHAN_IPCR_DL * sizeof(MhiChanCtxt);
        if (wcn7850_read_chan_ctxt(pci_dev, dl_ctxt_addr, &dl_ctxt) &&
            dl_ctxt.rp != dl_ctxt.wp) {
            WCN_DBG(s, "WCN: retrying pending QMI ind 0x%04x (DL ring has entries)\n",
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
        if (s->ce_dst[1].base && s->ce_dst[1].pending) {
            WCN_DBG(s, "WCN: sending HTC ready\n");
            wcn7850_htc_send_ready(s, pci_dev);
            s->htc_ready_sent = true;
        }
    }

    if (s->wmi_service_ready_pending && !s->wmi_ready_sent) {
        /* WMI events are always sent through HTC control DL (CE pipe 1),
         * regardless of the WMI endpoint's configured dl_pipe.  Check that
         * CE1 has a posted buffer before proceeding. */
        if (s->ce_dst[1].base) {
            WCN_DBG(s, "WCN: sending WMI service ready (dl pipe 1)\n");
            wcn7850_send_wmi_service_ready(s, pci_dev);
            s->wmi_service_ready_pending = false;
        }
    }

    for (int i = 0; i < WCN7850_CE_COUNT; i++) {
        if (!s->ce_src[i].base)
            continue;
        /* Refresh HP from the ring's shared write-index so polled CE pipes
         * (e.g. HTT on pipe 4) are serviced even without a doorbell. */
        if (s->ce_src[i].tp_addr) {
            uint32_t hp_le = 0;
            if (pci_dma_read(pci_dev, s->ce_src[i].tp_addr, &hp_le, 4) == MEMTX_OK)
                s->ce_src[i].hp = le32_to_cpu(hp_le) * 4;
        }
        if (s->ce_src[i].hp != s->ce_src[i].tp) {
            wcn7850_process_ce_src(s, pci_dev, i);
        }
    }

    /* Dynamic UMAC-window TCL producer indices are not delivered through the
     * BAR0 mirror on this revision. Poll them so guest TX is consumed even
     * when the producer doorbell write is windowed. */
    for (int i = 0; i < WCN7850_NUM_TCL_RINGS; i++) {
        WCN7850RingState *tcl = wcn7850_find_ring(s, WCN7850_RING_TCL_DATA, i);
        uint32_t hp_window;
        uint32_t hp_shadow;
        uint32_t hp_direct;

        if (!tcl || !tcl->configured || !tcl->enable)
            continue;
        memcpy(&hp_window, s->window_memory + 0x46000 + i * 4,
               sizeof(hp_window));
        memcpy(&hp_shadow, s->bar0_always_on + WCN7850_SHADOW_BASE +
               (12 + i) * sizeof(hp_shadow), sizeof(hp_shadow));
        memcpy(&hp_direct, s->bar0_always_on + WCN7850_TCL_RING_HP(i),
               sizeof(hp_direct));
        /* Take the maximum of shadow/window/direct. The driver posts TX
         * descriptors by writing the SRNG shadow (sidx 12+i); the R2
         * window/direct registers are stale (0) for shadow-posted rings
         * and must not override an advancing shadow. */
        hp_window = le32_to_cpu(hp_window) * 4;
        hp_shadow = le32_to_cpu(hp_shadow) * 4;
        hp_direct = le32_to_cpu(hp_direct) * 4;
        uint32_t hp = hp_shadow;
        if (hp_window > hp) hp = hp_window;
        if (hp_direct > hp) hp = hp_direct;
        if (hp != tcl->hp) {
            tcl->hp = hp;
            wcn7850_process_tcl_data(s, pci_dev, tcl);
        }
    }

    /* REO destination interrupts are level-triggered in hardware. Reassert
     * the vector while an unconsumed completion remains in the ring so a
     * packet arriving during NAPI processing cannot lose its interrupt edge. */
    for (int i = 1; i <= WCN7850_NUM_REO_RINGS; i++) {
        WCN7850RingState *reo = wcn7850_find_ring(s, WCN7850_RING_REO_DST, i);
        uint32_t tp_le;
        uint32_t tp;

        if (!reo || !reo->configured || !reo->enable)
            continue;
        memcpy(&tp_le, s->bar0_always_on + WCN7850_SHADOW_BASE +
               (reo->ring_id - 1) * sizeof(tp_le), sizeof(tp_le));
        tp = le32_to_cpu(tp_le) * 4;
        if (tp != reo->tp) {
            if (reo->msivec < WCN7850_MSI_VECTORS)
                msi_notify(pci_dev, reo->msivec);
        }
    }

    /* Scan RXDMA buf rings for driver-posted RX buffer addresses */
    wcn7850_poll_rxdma_buf_rings(s, pci_dev);

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

    if (pcie_endpoint_cap_init(pci_dev, 0x80) < 0) {
        error_setg(errp, "failed to initialize PCIe endpoint capability");
        return;
    }
    if (msi_init(pci_dev, 0x50, WCN7850_MSI_VECTORS, true, false, errp) < 0)
        return;


    s->mhi_state = 0;
    s->bhi_execenv = 2; /* firmware shim: expose AMSS without a host image */
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
    s->deferred_ce_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                  wcn7850_deferred_ce_timer, s);
    s->scan_seq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                     wcn7850_scan_seq_timer, s);
    s->scan_seq_step = 0;
    s->deferred_ce_head = 0;
    s->deferred_ce_count = 0;
    timer_mod(s->ce_poll_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000LL); /* start 1ms after realize */
    s->qmi_service_active = false;

    /* Initialize with a default MAC if none set */
    s->mac_addr[0] = 0x00;
    s->mac_addr[1] = 0x14;
    s->mac_addr[2] = 0x6c;
    s->mac_addr[3] = 0x9a;
    s->mac_addr[4] = 0x00;
    s->mac_addr[5] = 0x02;
    memcpy(s->conf.macaddr.a, s->mac_addr, sizeof(s->mac_addr));
    memcpy(s->mac_addr, s->conf.macaddr.a, 6);

    /* Initialize the simulated AP: an open BSS on 2.4GHz channel 6. */
    s->ap_bssid[0] = 0x00; s->ap_bssid[1] = 0x14; s->ap_bssid[2] = 0x6c;
    s->ap_bssid[3] = 0x9a; s->ap_bssid[4] = 0x00; s->ap_bssid[5] = 0x01;
    memcpy(s->ap_ssid, "wcn7850-ap", 10);
    s->ap_ssid_len = 10;
    /* Channel 6 (2437 MHz): the simulated AP lives on a 2.4GHz channel that
     * the driver's advertised scan list actually visits, so the beacon
     * registers a BSS with mac80211 (see wcn7850_handle_scan_start). */
    s->ap_channel = 6;
    s->ap_freq = 2437;
    s->peer_id = 1;
    s->peer_mapped = false;

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
    memory_region_init_ram(&s->pool_mr, OBJECT(s), "wcn7850-pool",
                           WCN7850_POOL_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), s->pool_paddr, &s->pool_mr);
    s->pool_mem = memory_region_get_ram_ptr(&s->pool_mr);
    memset(s->pool_mem, 0, WCN7850_POOL_SIZE);

    /* Initialize ring tracking */
    s->num_rings = 0;
    memset(s->rings, 0, sizeof(s->rings));

    /* Initialize RX/TX state */
    s->rx_head = 0;
    s->rx_count = 0;
    s->tx_buffer = NULL;
}

static void wcn7850_pci_uninit(PCIDevice *pci_dev)
{
    WCN7850State *s = WCN7850(pci_dev);

    timer_free(s->ctrl_event_timer);
    timer_free(s->ce_poll_timer);
    timer_free(s->deferred_ce_timer);
    timer_free(s->scan_seq_timer);
    g_free(s->bar0_always_on);
    g_free(s->window_memory);
    g_free(s->tx_buffer);
    if (s->pool_mem) {
        memory_region_del_subregion(get_system_memory(), &s->pool_mr);
    }
    if (s->nic) {
        qemu_del_nic(s->nic);
    }
}

static const VMStateDescription vmstate_ce_src_ring = {
    .name = "wcn7850/ce_src_ring",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(base, CeSrcRing),
        VMSTATE_UINT32(size, CeSrcRing),
        VMSTATE_UINT32(entry_size, CeSrcRing),
        VMSTATE_UINT64(hp_addr, CeSrcRing),
        VMSTATE_UINT64(tp_addr, CeSrcRing),
        VMSTATE_UINT32(hp, CeSrcRing),
        VMSTATE_UINT32(tp, CeSrcRing),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ce_dst_ring = {
    .name = "wcn7850/ce_dst_ring",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(base, CeDstRing),
        VMSTATE_UINT32(size, CeDstRing),
        VMSTATE_UINT32(entry_size, CeDstRing),
        VMSTATE_UINT32(hp, CeDstRing),
        VMSTATE_UINT32(tp, CeDstRing),
        VMSTATE_UINT32(drv_hp, CeDstRing),
        VMSTATE_UINT32(drv_hp_prev, CeDstRing),
        VMSTATE_UINT32(pending, CeDstRing),
        VMSTATE_BOOL(doorbell_seen, CeDstRing),
        VMSTATE_UINT32(cons, CeDstRing),
        VMSTATE_UINT64(hp_addr, CeDstRing),
        VMSTATE_UINT64(tp_addr, CeDstRing),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ce_sts_ring = {
    .name = "wcn7850/ce_sts_ring",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(base, CeStsRing),
        VMSTATE_UINT32(size, CeStsRing),
        VMSTATE_UINT32(entry_size, CeStsRing),
        VMSTATE_UINT32(hp, CeStsRing),
        VMSTATE_UINT64(hp_addr, CeStsRing),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_deferred_ce_event = {
    .name = "wcn7850/deferred_ce_event",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER(data, DeferredCeEvent),
        VMSTATE_UINT32(len, DeferredCeEvent),
        VMSTATE_INT32(pipe, DeferredCeEvent),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_pending_rx_frame = {
    .name = "wcn7850/pending_rx_frame",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER(data, PendingRxFrame),
        VMSTATE_UINT32(len, PendingRxFrame),
        VMSTATE_END_OF_LIST()
    }
};

static int wcn7850_post_load(void *opaque, int version_id)
{
    WCN7850State *s = opaque;

    if (version_id < 2) {
        s->rx_head = 0;
        s->rx_count = 0;
        memset(s->rx_frames, 0, sizeof(s->rx_frames));
    }
    if (s->deferred_ce_head >= WCN7850_PENDING_CE_EVENTS ||
        s->deferred_ce_count > WCN7850_PENDING_CE_EVENTS) {
        return -EINVAL;
    }
    if (s->rx_head >= WCN7850_PENDING_RX_FRAMES ||
        s->rx_count > WCN7850_PENDING_RX_FRAMES) {
        return -EINVAL;
    }
    for (uint32_t i = 0; i < WCN7850_PENDING_CE_EVENTS; i++) {
        if (s->deferred_ce_events[i].pipe < 0 ||
            s->deferred_ce_events[i].pipe >= WCN7850_CE_COUNT ||
            s->deferred_ce_events[i].len > sizeof(s->deferred_ce_events[i].data)) {
            return -EINVAL;
        }
    }
    for (uint32_t i = 0; i < WCN7850_PENDING_RX_FRAMES; i++) {
        if (s->rx_frames[i].len > sizeof(s->rx_frames[i].data)) {
            return -EINVAL;
        }
    }
    if (s->deferred_ce_count) {
        timer_mod(s->deferred_ce_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
    }
    if (s->rx_count) {
        timer_mod(s->ce_poll_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
    }
    return 0;
}

static const VMStateDescription vmstate_wcn7850 = {
    .name = "wcn7850",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(window_select, WCN7850State),
        VMSTATE_BOOL(bhi_downloaded, WCN7850State),
        VMSTATE_UINT32(bhi_status, WCN7850State),
        VMSTATE_UINT32(bhi_execenv, WCN7850State),
        VMSTATE_UINT32(mhi_state, WCN7850State),
        VMSTATE_UINT64(er_ctxt_addr, WCN7850State),
        VMSTATE_BOOL(ctrl_event_pending, WCN7850State),
        VMSTATE_UINT64(chan_ctxt_addr, WCN7850State),
        VMSTATE_UINT64(cmd_ctxt_addr, WCN7850State),
        VMSTATE_BOOL(qmi_service_active, WCN7850State),
        VMSTATE_BOOL(qmi_newserver_sent, WCN7850State),
        VMSTATE_BOOL(qmi_newserver_resent, WCN7850State),
        VMSTATE_BOOL(qmi_boot_pending, WCN7850State),
        VMSTATE_BOOL(qmi_newserver_delivered, WCN7850State),
        VMSTATE_UINT16(qmi_pending_ind, WCN7850State),
        VMSTATE_UINT32(qmi_client_node, WCN7850State),
        VMSTATE_UINT32(qmi_client_port, WCN7850State),
        VMSTATE_BOOL(ce_ready, WCN7850State),
        VMSTATE_BOOL(htc_ready_sent, WCN7850State),
        VMSTATE_BOOL(wmi_service_ready_pending, WCN7850State),
        VMSTATE_BOOL(wmi_service_ready_sent, WCN7850State),
        VMSTATE_BOOL(wmi_ready_sent, WCN7850State),
        VMSTATE_UINT32(vdev_id, WCN7850State),
        VMSTATE_BUFFER(peer_mac, WCN7850State),
        VMSTATE_UINT16(peer_id, WCN7850State),
        VMSTATE_BOOL(peer_mapped, WCN7850State),
        VMSTATE_BUFFER(sta_mac, WCN7850State),
        VMSTATE_BOOL(sta_mac_valid, WCN7850State),
        VMSTATE_UINT32(scan_id, WCN7850State),
        VMSTATE_UINT32(scan_vdev_id, WCN7850State),
        VMSTATE_INT32(scan_seq_step, WCN7850State),
        VMSTATE_BOOL(scan_seq_ap, WCN7850State),
        VMSTATE_STRUCT_ARRAY(ce_src, WCN7850State, WCN7850_CE_COUNT,
                             1, vmstate_ce_src_ring, CeSrcRing),
        VMSTATE_STRUCT_ARRAY(ce_dst, WCN7850State, WCN7850_CE_COUNT,
                             1, vmstate_ce_dst_ring, CeDstRing),
        VMSTATE_STRUCT_ARRAY(ce_sts, WCN7850State, WCN7850_CE_COUNT,
                             1, vmstate_ce_sts_ring, CeStsRing),
        VMSTATE_UINT32(deferred_ce_head, WCN7850State),
        VMSTATE_UINT32(deferred_ce_count, WCN7850State),
        VMSTATE_STRUCT_ARRAY(deferred_ce_events, WCN7850State,
                             WCN7850_PENDING_CE_EVENTS, 1,
                             vmstate_deferred_ce_event, DeferredCeEvent),
        VMSTATE_UINT32_V(rx_head, WCN7850State, 2),
        VMSTATE_UINT32_V(rx_count, WCN7850State, 2),
        VMSTATE_STRUCT_ARRAY(rx_frames, WCN7850State,
                             WCN7850_PENDING_RX_FRAMES, 2,
                             vmstate_pending_rx_frame, PendingRxFrame),
        VMSTATE_END_OF_LIST()
    },
    .post_load = wcn7850_post_load,
};

static const Property wcn7850_properties[] = {
    DEFINE_NIC_PROPERTIES(WCN7850State, conf),
    DEFINE_PROP_BOOL("dbg", WCN7850State, dbg, false),
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
