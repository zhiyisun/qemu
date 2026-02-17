/*
 * QEMU Intel E810 Multi-Port PF Test Device
 *
 * Copyright (c) 2026
 *
 * This is a minimal PCI device designed to test the Intel ICE driver's
 * multi-port per-PF functionality. It does not emulate the full E810 hardware
 * but provides enough functionality to exercise the driver's multi-port logic.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "hw/resettable.h"
#include "net/eth.h"
#include "net/net.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qom/object.h"

/* PCI Configuration */
#define PCI_VENDOR_ID_INTEL     0x8086
#define PCI_DEVICE_ID_ICE_MP    0x1592  /* E810-C for QSFP */

/* BAR0 Size (4MB) */
#define ICE_MP_BAR0_SIZE        0x00800000  /* 8MB MMIO space */

/* BAR0 Register Offsets */
#define ICE_MP_REG_CAPS         0x0000
#define ICE_MP_REG_PORT_COUNT   0x0004
#define ICE_MP_REG_PORT_STATUS  0x0010  /* Array: 0x10 + port_id * 4 */
#define ICE_MP_REG_EVENT_DB     0x0100
#define ICE_MP_REG_VF_PORT_MAP  0x0200  /* Array: 0x200 + vf_id */
/* Queue and context registers (subset used for TX/RX datapath) */
#define ICE_MP_REG_QTX_COMM_HEAD     0x000E0000
#define ICE_MP_REG_QTX_COMM_DBELL    0x002C0000
#define ICE_MP_REG_QRX_CTRL          0x00120000
#define ICE_MP_REG_QRX_TAIL          0x00290000
#define ICE_MP_REG_QRX_CONTEXT       0x00280000
#define ICE_MP_QRX_CONTEXT_STRIDE    8192
#define ICE_MP_REG_QINT_TQCTL        0x00140000
#define ICE_MP_REG_QINT_RQCTL        0x00150000
#define ICE_MP_REG_GLINT_DYN_CTL     0x00160000
#define ICE_MP_REG_GLCOMM_QTX_CNTX_CTL   0x002D2DC8
#define ICE_MP_REG_GLCOMM_QTX_CNTX_DATA  0x002D2D40
#define ICE_MP_REG_VPLAN_RX_QBASE    0x00072000
#define ICE_MP_REG_VPLAN_RXQ_MAPENA  0x00073000
#define ICE_MP_REG_VPLAN_TX_QBASE    0x001D1800
#define ICE_MP_REG_VPLAN_TXQ_MAPENA  0x00073800
/* AdminQ registers */
#define ICE_MP_REG_PF_FW_ATQBAL 0x00080000
#define ICE_MP_REG_PF_FW_ATQBAH 0x00080100
#define ICE_MP_REG_PF_FW_ATQLEN 0x00080200
#define ICE_MP_REG_PF_FW_ATQH   0x00080300
#define ICE_MP_REG_PF_FW_ATQT   0x00080400
#define ICE_MP_REG_PF_FW_ARQBAL 0x00080080
#define ICE_MP_REG_PF_FW_ARQBAH 0x00080180
#define ICE_MP_REG_PF_FW_ARQLEN 0x00080280
#define ICE_MP_REG_PF_FW_ARQH   0x00080380
#define ICE_MP_REG_PF_FW_ARQT   0x00080480
/* Mailbox registers */
#define ICE_MP_REG_PF_MBX_ATQBAL 0x0022E100
#define ICE_MP_REG_PF_MBX_ATQBAH 0x0022E180
#define ICE_MP_REG_PF_MBX_ATQLEN 0x0022E200
#define ICE_MP_REG_PF_MBX_ATQH   0x0022E280
#define ICE_MP_REG_PF_MBX_ATQT   0x0022E300
#define ICE_MP_REG_PF_MBX_ARQBAL 0x0022E380
#define ICE_MP_REG_PF_MBX_ARQBAH 0x0022E400
#define ICE_MP_REG_PF_MBX_ARQLEN 0x0022E480
#define ICE_MP_REG_PF_MBX_ARQH   0x0022E500
#define ICE_MP_REG_PF_MBX_ARQT   0x0022E580
/* VF mailbox and reset registers (subset for SR-IOV enable) */
#define ICE_MP_REG_VF_MBX_ATQLEN 0x0022A800
#define ICE_MP_REG_VF_MBX_ARQLEN 0x0022BC00
#define ICE_MP_REG_VFGEN_RSTAT   0x00074000
#define ICE_MP_REG_VPGEN_VFRTRIG 0x00090000
#define ICE_MP_REG_VPGEN_VFRSTAT 0x00090800
#define ICE_MP_REG_PF_PCI_CIAD   0x0009E500
#define ICE_MP_REG_PF_PCI_CIAA   0x0009E580
#define ICE_MP_VFRSTAT_VFRD      BIT(0)
/* Reset-related registers used by the driver */
#define ICE_MP_REG_PFGEN_CTRL   0x00091000
#define ICE_MP_REG_GLNVM_ULD    0x000B6008
#define ICE_MP_REG_GLNVM_GENS   0x000B6100
#define ICE_MP_REG_GLNVM_FLA    0x000B6108
#define ICE_MP_REG_GLGEN_RSTCTL 0x000B8180
#define ICE_MP_REG_GLGEN_RSTAT  0x000B8188
#define ICE_MP_REG_GLGEN_RTRIG  0x000B8190

/* Capability Flags */
#define ICE_MP_CAP_MULTI_PORT   (1 << 0)
#define ICE_MP_CAP_SRIOV        (1 << 1)
#define ICE_MP_CAP_MSIX         (1 << 2)

/* Event Types */
#define ICE_MP_EVENT_LINK_CHANGE    1
#define ICE_MP_EVENT_RESET          2
#define ICE_MP_EVENT_VF_MAILBOX     3

/* Port Status Bits */
#define ICE_MP_PORT_LINK_UP     (1 << 0)
#define ICE_MP_PORT_SPEED_SHIFT 1
#define ICE_MP_PORT_SPEED_MASK  (0x7 << ICE_MP_PORT_SPEED_SHIFT)
#define ICE_MP_PORT_FAULT       (1 << 4)

/* Speed Values */
#define ICE_MP_SPEED_10G        0
#define ICE_MP_SPEED_25G        1
#define ICE_MP_SPEED_40G        2
#define ICE_MP_SPEED_100G       3

/* Reset done mask bits (mirrors ice_hw_autogen.h) */
#define ICE_MP_GLRST_DONE_MASK  (BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(5) | \
                                 BIT(8) | BIT(9) | BIT(10))

/* AdminQ flags */
#define ICE_MP_AQ_FLAG_DD       BIT(0)
#define ICE_MP_AQ_FLAG_CMP      BIT(1)
#define ICE_MP_AQ_FLAG_BUF      BIT(12)

/* AdminQ opcodes used during init */
#define ICE_MP_AQC_OPC_GET_VER          0x0001
#define ICE_MP_AQC_OPC_DRIVER_VER       0x0002
#define ICE_MP_AQC_OPC_Q_SHUTDOWN       0x0003
#define ICE_MP_AQC_OPC_REQ_RES          0x0008
#define ICE_MP_AQC_OPC_REL_RES          0x0009
#define ICE_MP_AQC_OPC_CLEAR_PXE_MODE   0x0110
#define ICE_MP_AQC_OPC_LIST_FUNC_CAPS   0x000A
#define ICE_MP_AQC_OPC_LIST_DEV_CAPS    0x000B
#define ICE_MP_AQC_OPC_GET_SW_CFG       0x0200
#define ICE_MP_AQC_OPC_ALLOC_RES        0x0208
#define ICE_MP_AQC_OPC_FREE_RES         0x0209
#define ICE_MP_AQC_OPC_CLEAR_PF_CFG     0x02A4
#define ICE_MP_AQC_OPC_GET_PHY_CAPS     0x0600
#define ICE_MP_AQC_OPC_GET_LINK_STATUS  0x0607
#define ICE_MP_AQC_OPC_GET_PORT_OPTIONS 0x06EA
#define ICE_MP_AQC_OPC_SET_MAC_CFG      0x0603
#define ICE_MP_AQC_OPC_NVM_READ         0x0701
#define ICE_MP_AQC_OPC_QUERY_TXSCHED    0x0412
#define ICE_MP_AQC_OPC_MANAGE_MAC_READ  0x0107
#define ICE_MP_AQC_OPC_DOWNLOAD_PKG     0x0C40
#define ICE_MP_AQC_OPC_UPLOAD_SECTION   0x0C41
#define ICE_MP_AQC_OPC_GET_PKG_INFO_LIST 0x0C43
#define ICE_MP_AQC_OPC_FW_LOGS_CONFIG   0xFF30
#define ICE_MP_AQC_OPC_FW_LOGS_REGISTER 0xFF31
#define ICE_MP_AQC_OPC_FW_LOGS_QUERY    0xFF32

/* DDP package buffer size used by ice_aq_upload_section */
#define ICE_MP_PKG_BUF_SIZE             4096

/* Maximum ports and VFs */
#define ICE_MP_MAX_PORTS        16
#define ICE_MP_MAX_VFS          256
#define ICE_MP_MSIX_VECTORS     64  /* Enough for 4 ports * num_cpus + OICR + control VSI */

/* FW logging module count (LIBIE_AQC_FW_LOG_ID_MAX) */
#define ICE_MP_FWLOG_MODULES    32

/* Resource IDs and status (libie adminq) */
#define ICE_MP_RES_ID_GLBL_LOCK 4
#define ICE_MP_RES_GLBL_SUCCESS 0

/* FW log flags (libie adminq) */
#define ICE_MP_FW_LOG_CONF_AQ_EN         BIT(1)
#define ICE_MP_FW_LOG_QUERY_REGISTERED  BIT(2)
#define ICE_MP_FW_LOG_AQ_QUERY          BIT(2)

/* SR-IOV Configuration */
#define ICE_MP_SRIOV_OFFSET     0x100
#define ICE_MP_VF_OFFSET        0x01
#define ICE_MP_VF_STRIDE        0x01
#define ICE_MP_VF_DEV_ID        0x1889  /* IAVF adaptive VF */

/* TX/RX descriptor formats (subset) */
struct ice_mp_tx_desc {
    uint64_t buf_addr;
    uint64_t cmd_type_offset_bsz;
} __attribute__((packed));

union ice_mp_rx_desc {
    struct {
        uint64_t pkt_addr;
        uint64_t hdr_addr;
        uint64_t rsvd1;
        uint64_t rsvd2;
    } read;
    struct {
        uint8_t rxdid;
        uint8_t mir_id_umb_cast;
        uint16_t ptype_flex_flags0;
        uint16_t pkt_len;
        uint16_t hdr_len_sph_flex_flags1;
        uint16_t status_error0;
        uint16_t l2tag1;
        uint32_t rss_hash;
        uint16_t status_error1;
        uint8_t flexi_flags2;
        uint8_t ts_low;
        uint16_t l2tag2_1st;
        uint16_t l2tag2_2nd;
        uint32_t flow_id;
        uint32_t ts_high;
    } wb;
} __attribute__((packed));

typedef struct ICEMPQueue {
    bool configured;
    bool enabled;
    uint64_t base;
    uint16_t qlen;
    uint16_t head;
    uint16_t tail;
    uint8_t port_id;
    uint32_t ctrl;
    uint32_t int_ctl;
} ICEMPQueue;

typedef struct ICEMPPort {
    struct ICEMPState *s;
    uint8_t port_id;
} ICEMPPort;

typedef struct IceMpRxRule {
    bool valid;
    uint8_t mac[6];
    bool has_vlan;
    uint16_t vlan;
    uint16_t vsi_id;
    uint16_t rule_id;
    uint16_t src;
} IceMpRxRule;

#define ICE_MP_MAX_TX_QUEUES    16384
#define ICE_MP_MAX_RX_QUEUES    2048

/* Scheduler node tracking for query/delete operations */
#define ICE_MP_SCHED_TEID_BASE  0x16000000
#define ICE_MP_MAX_SCHED_NODES  1024

typedef struct IceMpSchedNode {
    bool valid;
    uint32_t parent_teid;
    uint8_t elem_type;
} IceMpSchedNode;

#define ICE_MP_TX_CTX_DWORDS    10
#define ICE_MP_RX_CTX_DWORDS    8

#define ICE_MP_TX_DESC_SIZE     16
#define ICE_MP_RX_DESC_SIZE     32

#define ICE_MP_TXD_QW1_CMD_S        4
#define ICE_MP_TXD_QW1_TX_BUF_SZ_S  34

#define ICE_MP_QINT_MSIX_INDX_M     0x7FF
#define ICE_MP_QINT_CAUSE_ENA_M     BIT(30)

#define ICE_MP_QRX_CTRL_QENA_REQ_M  BIT(0)
#define ICE_MP_QRX_CTRL_QENA_STAT_M BIT(2)

/* GLINT_DYN_CTL register bits */
#define ICE_MP_GLINT_DYN_CTL_INTENA_M       BIT(0)
#define ICE_MP_GLINT_DYN_CTL_CLEARPBA_M     BIT(1)
#define ICE_MP_GLINT_DYN_CTL_SWINT_TRIG_M   BIT(2)
#define ICE_MP_GLINT_DYN_CTL_WB_ON_ITR_M    BIT(30)
#define ICE_MP_GLINT_DYN_CTL_INTENA_MSK_M   BIT(31)

#define ICE_MP_GLCOMM_QTX_CNTX_CTL_QUEUE_ID_M  0x3FFF
#define ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_M       (0x7 << 16)
#define ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_READ    0
#define ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_WRITE_NO_DYN 4
#define ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_EXEC    BIT(19)

#define ICE_MP_VPLAN_RX_QBASE_VFFIRSTQ_M   0x7FF
#define ICE_MP_VPLAN_RX_QBASE_VFNUMQ_M     (0xFF << 16)
#define ICE_MP_VPLAN_TX_QBASE_VFFIRSTQ_M   0x3FFF
#define ICE_MP_VPLAN_TX_QBASE_VFNUMQ_M     (0xFF << 16)

#define ICE_MP_VPLAN_RXQ_MAPENA_RX_ENA_M   BIT(0)
#define ICE_MP_VPLAN_TXQ_MAPENA_TX_ENA_M   BIT(0)

#define ICE_MP_RX_STATUS0_DD        BIT(0)
#define ICE_MP_RX_STATUS0_EOF       BIT(1)

#define ICE_MP_MAX_VSI             1024
#define ICE_MP_MAX_RULES           256

#define ICE_MP_SW_LKUP_MAC          1
#define ICE_MP_SW_LKUP_MAC_VLAN     2
#define ICE_MP_SW_LKUP_PROMISC      3
#define ICE_MP_SW_LKUP_PROMISC_VLAN 9

#define ICE_MP_ETH_DA_OFFSET        0
#define ICE_MP_ETH_ETHTYPE_OFFSET   12
#define ICE_MP_ETH_VLAN_TCI_OFFSET  14
#define ICE_MP_MAX_VLAN_ID          0xFFF

#define ICE_MP_SW_RULE_T_LKUP_RX    0x0
#define ICE_MP_SW_RULE_T_LKUP_TX    0x1
#define ICE_MP_SW_RULE_T_LG_ACT     0x2
#define ICE_MP_SW_RULE_T_VSI_LIST   0x3

#define ICE_MP_SW_ACT_TYPE_M        0x3
#define ICE_MP_SW_ACT_TYPE_VSI      0x0
#define ICE_MP_SW_ACT_VSI_ID_S      4
#define ICE_MP_SW_ACT_VSI_ID_M      (0x3FF << ICE_MP_SW_ACT_VSI_ID_S)
#define ICE_MP_SW_ACT_VSI_LIST      BIT(14)
#define ICE_MP_SW_ACT_VALID         BIT(17)
#define ICE_MP_SW_ACT_DROP          BIT(18)

#define ICE_TX_DESC_DTYPE_DATA      0x0
#define ICE_TX_DESC_DTYPE_CTX       0x1
#define ICE_TX_DESC_DTYPE_DESC_DONE 0xF
#define ICE_TX_DESC_CMD_EOP         0x0001

#define ICE_RXDID_FLEX_NIC          2

/* Device State */
struct ICEMPState {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    MemoryRegion bar0;
    MemoryRegion bar1;
    
    /* Configuration */
    uint32_t num_ports;
    uint32_t num_vfs;
    uint32_t rxq_map_mode;
    uint32_t queues_per_port;
    
    /* BAR0 Registers */
    uint32_t caps;
    uint32_t port_count;
    uint32_t port_status[ICE_MP_MAX_PORTS];
    uint32_t event_doorbell;
    uint8_t vf_port_map[ICE_MP_MAX_VFS];

    /* Reset-related registers */
    uint32_t pfgen_ctrl;
    uint32_t glnvm_uld;
    uint32_t glnvm_gens;
    uint32_t glnvm_fla;
    uint32_t glgen_rstctl;
    uint32_t glgen_rstat;
    uint32_t glgen_rtrig;
    
    /* MSI-X */
    uint8_t msix_bar_idx;

    /* AdminQ register state */
    uint32_t atq_bal;
    uint32_t atq_bah;
    uint32_t atq_len;
    uint32_t atq_head;
    uint32_t atq_tail;
    uint32_t arq_bal;
    uint32_t arq_bah;
    uint32_t arq_len;
    uint32_t arq_head;
    uint32_t arq_tail;

    /* NVM Flash Memory Simulation 
     * Flash structure:
     *   - Raw flash reads (module=0, FLASH_ONLY): direct access to flash arrays
     *   - Bank reads (module=0x0100-0x0300): access via bank pointers in Shadow RAM
     * Sizes must accommodate driver's flash discovery (up to 16MB) and actual reads
     */
    uint16_t *nvm_flash;         /* NVM bank: dynamically allocated 512KB */
    uint16_t *orom_flash;        /* OROM bank: dynamically allocated 128KB */
    uint16_t *netlist_flash;     /* Netlist bank: dynamically allocated 64KB */
    size_t nvm_flash_size;       /* Size in words */
    size_t orom_flash_size;      /* Size in words */
    size_t netlist_flash_size;   /* Size in words */

    /* Resource lock state machine 
     * Tracks whether global config lock is acquired/released.
     * This is critical for ice_init_hw() which requests the lock early.
     */
    bool glbl_cfg_lock_held;     /* TRUE when global config lock is held */
    uint32_t lock_timeout;       /* Timeout value when lock was acquired */

    /* Resource allocation counter (for alloc/free resource AQ commands) */
    uint16_t res_counter;

    /* Scheduler TEID counter (for add sched elems commands) */
    uint32_t next_sched_teid;

    /* Scheduler node table (TEID -> parent/type tracking for 0x0404) */
    IceMpSchedNode sched_nodes[ICE_MP_MAX_SCHED_NODES];

    /* VSI allocation counter */
    uint16_t next_vsi_num;

    /* Mailbox register state */
    uint32_t mbx_atq_bal;
    uint32_t mbx_atq_bah;
    uint32_t mbx_atq_len;
    uint32_t mbx_atq_head;
    uint32_t mbx_atq_tail;
    uint32_t mbx_arq_bal;
    uint32_t mbx_arq_bah;
    uint32_t mbx_arq_len;
    uint32_t mbx_arq_head;
    uint32_t mbx_arq_tail;

    /* VF mailbox/reset register state */
    uint32_t vf_mbx_atqlen[ICE_MP_MAX_VFS];
    uint32_t vf_mbx_arqlen[ICE_MP_MAX_VFS];
    uint32_t vfgen_rstat[ICE_MP_MAX_VFS];
    uint32_t vpgen_vfrtrig[ICE_MP_MAX_VFS];
    uint32_t vpgen_vfrstat[ICE_MP_MAX_VFS];
    uint32_t pf_pci_ciaa;
    uint32_t pf_pci_ciad;

    uint16_t vf_rxq_base[ICE_MP_MAX_VFS];
    uint16_t vf_rxq_num[ICE_MP_MAX_VFS];
    bool vf_rxq_mapena[ICE_MP_MAX_VFS];
    uint16_t vf_txq_base[ICE_MP_MAX_VFS];
    uint16_t vf_txq_num[ICE_MP_MAX_VFS];
    bool vf_txq_mapena[ICE_MP_MAX_VFS];

    uint16_t vsi_to_vf[ICE_MP_MAX_VSI];
    uint8_t vsi_to_port[ICE_MP_MAX_VSI];  /* VSI number → port ID mapping */
    uint8_t pf_vsi_count;                 /* Counter for PF-type VSI allocations */
    bool last_vsi_was_pf;                 /* Track PF/VF VSI creation boundary */
    IceMpRxRule rx_rules[ICE_MP_MAX_RULES];
    uint16_t next_rule_id;

    /* Net backends per port */
    NICState *nic[ICE_MP_MAX_PORTS];
    NICConf conf[ICE_MP_MAX_PORTS];
    ICEMPPort ports[ICE_MP_MAX_PORTS];

    /* Interrupt dynamic control per MSI-X vector */
    uint32_t glint_dyn_ctl[ICE_MP_MSIX_VECTORS];
    /* Per-vector flag: interrupt was suppressed because INTENA=0 */
    bool irq_pending[ICE_MP_MSIX_VECTORS];

    /* Queue contexts and runtime state */
    uint32_t tx_ctx[ICE_MP_MAX_TX_QUEUES][ICE_MP_TX_CTX_DWORDS];
    uint32_t rx_ctx[ICE_MP_MAX_RX_QUEUES][ICE_MP_RX_CTX_DWORDS];
    uint32_t glcomm_qtx_cntx_data[ICE_MP_TX_CTX_DWORDS];
    uint32_t glcomm_qtx_cntx_ctl;
    ICEMPQueue txq[ICE_MP_MAX_TX_QUEUES];
    ICEMPQueue rxq[ICE_MP_MAX_RX_QUEUES];

    /* Deferred TX completion interrupt timer */
    QEMUTimer *tx_irq_timer;
    uint64_t tx_irq_pending;  /* Bitmask of MSI-X vectors needing interrupt */
};

typedef struct ICEMPState ICEMPState;

struct ice_mp_aq_desc {
    uint16_t flags;
    uint16_t opcode;
    uint16_t datalen;
    uint16_t retval;
    uint32_t cookie_high;
    uint32_t cookie_low;
    uint8_t params[16];
} __attribute__((packed));

struct ice_mp_sw_rule_hdr {
    uint16_t type;
    uint16_t status;
} __attribute__((packed));

struct ice_mp_sw_rule_lkup_rx_tx_fixed {
    struct ice_mp_sw_rule_hdr hdr;
    uint16_t recipe_id;
    uint16_t src;
    uint32_t act;
    uint16_t index;
    uint16_t hdr_len;
} __attribute__((packed));

struct ice_mp_aqc_list_caps {
    uint8_t cmd_flags;
    uint8_t pf_index;
    uint8_t rsvd[2];
    uint32_t count;
    uint32_t addr_high;
    uint32_t addr_low;
} __attribute__((packed));

struct ice_mp_aqc_get_sw_cfg {
    uint16_t flags;
    uint16_t element;
    uint16_t num_elems;
    uint16_t rsvd;
    uint32_t addr_high;
    uint32_t addr_low;
} __attribute__((packed));

struct ice_mp_aqc_nvm {
    uint16_t offset_low;
    uint8_t offset_high;
    uint8_t cmd_flags;
    uint16_t module_typeid;
    uint16_t length;
    uint32_t addr_high;
    uint32_t addr_low;
} __attribute__((packed));

struct ice_mp_aqc_req_res {
    uint16_t res_id;
    uint16_t access_type;
    uint32_t timeout;
    uint32_t res_number;
    uint16_t status;
    uint8_t reserved[2];
} __attribute__((packed));

struct ice_mp_aqc_alloc_free_res_cmd {
    uint16_t num_entries;
    uint8_t reserved[6];
    uint32_t addr_high;
    uint32_t addr_low;
} __attribute__((packed));

struct ice_mp_aqc_fw_log {
    uint8_t cmd_flags;
    uint8_t rsp_flag;
    uint16_t fw_rt_msb;
    union {
        struct {
            uint32_t fw_rt_lsb;
        } sync;
        struct {
            uint16_t log_resolution;
            uint16_t mdl_cnt;
        } cfg;
    } ops;
    uint32_t addr_high;
    uint32_t addr_low;
} __attribute__((packed));

struct ice_mp_aqc_fw_log_cfg_resp {
    uint16_t module_identifier;
    uint8_t log_level;
    uint8_t rsvd0;
} __attribute__((packed));

struct ice_mp_aqc_get_ver {
    uint32_t rom_ver;
    uint32_t fw_build;
    uint8_t fw_branch;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t fw_patch;
    uint8_t api_branch;
    uint8_t api_major;
    uint8_t api_minor;
    uint8_t api_patch;
} __attribute__((packed));

struct ice_mp_aqc_list_caps_elem {
    uint16_t cap;
    uint8_t major_ver;
    uint8_t minor_ver;
    uint32_t number;
    uint32_t logical_id;
    uint32_t phys_id;
    uint64_t rsvd1;
    uint64_t rsvd2;
} __attribute__((packed));

struct ice_mp_aqc_manage_mac_read_resp {
    uint8_t lport_num;
    uint8_t addr_type;
    uint8_t mac_addr[6];
} __attribute__((packed));

/* Forward declarations of AdminQ helper functions */
static void ice_mp_adminq_complete(struct ICEMPState *s, struct ice_mp_aq_desc *desc);
static void ice_mp_adminq_add_tx_queues(struct ICEMPState *s, struct ice_mp_aq_desc *desc);
static void ice_mp_mailbox_process(struct ICEMPState *s);

static uint64_t ice_mp_dma_addr(uint32_t low, uint32_t high)
{
    return ((uint64_t)high << 32) | low;
}

/* Timer callback for deferred TX completion interrupts.
 * Real hardware fires interrupts asynchronously; doing it inline during the
 * MMIO doorbell write can cause races with the driver's NAPI scheduling. */
static void ice_mp_tx_irq_timer_cb(void *opaque)
{
    struct ICEMPState *s = opaque;
    uint64_t pending = s->tx_irq_pending;
    s->tx_irq_pending = 0;

    if (!msix_enabled(&s->parent_obj)) {
        return;
    }

    for (int vec = 0; vec < ICE_MP_MSIX_VECTORS && pending; vec++) {
        if (pending & (1ULL << vec)) {
            pending &= ~(1ULL << vec);
            s->glint_dyn_ctl[vec] &= ~ICE_MP_GLINT_DYN_CTL_INTENA_M;
            fprintf(stderr, "ice-mp: Deferred IRQ firing vec=%u\n", vec);
            msix_notify(&s->parent_obj, vec);
        }
    }
}

static uint64_t ice_mp_extract_bits(const uint32_t *dwords, uint32_t lsb,
                                    uint32_t width)
{
    uint64_t value = 0;

    for (uint32_t bit = 0; bit < width; bit++) {
        uint32_t idx = (lsb + bit) / 32;
        uint32_t shift = (lsb + bit) % 32;
        uint64_t bitval = (dwords[idx] >> shift) & 1U;
        value |= bitval << bit;
    }

    return value;
}


static uint16_t ice_mp_vlan_from_packet(const uint8_t *buf, size_t len)
{
    if (len < 16) {
        return ICE_MP_MAX_VLAN_ID + 1;
    }

    uint16_t ethertype = (buf[ICE_MP_ETH_ETHTYPE_OFFSET] << 8) |
                         buf[ICE_MP_ETH_ETHTYPE_OFFSET + 1];
    if (ethertype != 0x8100 || len < 18) {
        return ICE_MP_MAX_VLAN_ID + 1;
    }

    return ((buf[ICE_MP_ETH_VLAN_TCI_OFFSET] << 8) |
            buf[ICE_MP_ETH_VLAN_TCI_OFFSET + 1]) & ICE_MP_MAX_VLAN_ID;
}

static bool ice_mp_rule_match(const IceMpRxRule *rule, uint8_t port_id,
                              const uint8_t *buf, size_t len, uint16_t vlan)
{
    if (!rule->valid) {
        return false;
    }

    if (rule->src != 0 && rule->src != port_id && rule->src != 0xFFFF) {
        return false;
    }

    if (len < ETH_ALEN) {
        return false;
    }

    if (memcmp(rule->mac, buf + ICE_MP_ETH_DA_OFFSET, ETH_ALEN) != 0) {
        if (rule->mac[0] || rule->mac[1] || rule->mac[2] ||
            rule->mac[3] || rule->mac[4] || rule->mac[5]) {
            return false;
        }
    }

    if (rule->has_vlan && vlan != rule->vlan) {
        return false;
    }

    return true;
}

static uint16_t ice_mp_select_vf(struct ICEMPState *s, uint8_t port_id,
                                 const uint8_t *buf, size_t len)
{
    uint16_t vlan = ice_mp_vlan_from_packet(buf, len);

    for (uint16_t i = 0; i < ICE_MP_MAX_RULES; i++) {
        IceMpRxRule *rule = &s->rx_rules[i];
        if (!ice_mp_rule_match(rule, port_id, buf, len, vlan)) {
            continue;
        }

        if (rule->vsi_id < ICE_MP_MAX_VSI) {
            uint16_t vf_id = s->vsi_to_vf[rule->vsi_id];
            if (vf_id < s->num_vfs) {
                return vf_id;
            }
        }
    }

    return 0xFFFF;
}

static IceMpRxRule *ice_mp_rule_find_by_id(struct ICEMPState *s, uint16_t rule_id)
{
    for (uint16_t i = 0; i < ICE_MP_MAX_RULES; i++) {
        if (s->rx_rules[i].valid && s->rx_rules[i].rule_id == rule_id) {
            return &s->rx_rules[i];
        }
    }

    return NULL;
}

static IceMpRxRule *ice_mp_rule_alloc(struct ICEMPState *s, uint16_t rule_id)
{
    IceMpRxRule *rule = ice_mp_rule_find_by_id(s, rule_id);

    if (rule) {
        return rule;
    }

    for (uint16_t i = 0; i < ICE_MP_MAX_RULES; i++) {
        if (!s->rx_rules[i].valid) {
            return &s->rx_rules[i];
        }
    }

    return NULL;
}

static void ice_mp_rule_clear_all(struct ICEMPState *s)
{
    for (uint16_t i = 0; i < ICE_MP_MAX_RULES; i++) {
        s->rx_rules[i].valid = false;
        s->rx_rules[i].rule_id = 0;
        s->rx_rules[i].vsi_id = 0;
        s->rx_rules[i].has_vlan = false;
        s->rx_rules[i].vlan = 0;
        s->rx_rules[i].src = 0;
        memset(s->rx_rules[i].mac, 0, sizeof(s->rx_rules[i].mac));
    }
    s->next_rule_id = 1;
}

static bool ice_mp_sw_rule_extract_vsi(uint32_t act, uint16_t *vsi_id)
{
    uint32_t act_type = act & ICE_MP_SW_ACT_TYPE_M;

    if (!(act & ICE_MP_SW_ACT_VALID) || act_type != ICE_MP_SW_ACT_TYPE_VSI) {
        return false;
    }

    if (act & ICE_MP_SW_ACT_VSI_LIST) {
        return false;
    }

    *vsi_id = (uint16_t)((act & ICE_MP_SW_ACT_VSI_ID_M) >> ICE_MP_SW_ACT_VSI_ID_S);
    return true;
}

static uint16_t ice_mp_read_le16(const uint8_t *buf)
{
    uint16_t val;

    memcpy(&val, buf, sizeof(val));
    return le16_to_cpu(val);
}

static size_t ice_mp_sw_rule_elem_size(const uint8_t *buf, size_t len)
{
    if (len < sizeof(struct ice_mp_sw_rule_hdr)) {
        return 0;
    }

    uint16_t type = ice_mp_read_le16(buf);

    if (type == ICE_MP_SW_RULE_T_LKUP_RX || type == ICE_MP_SW_RULE_T_LKUP_TX) {
        if (len < sizeof(struct ice_mp_sw_rule_lkup_rx_tx_fixed)) {
            return 0;
        }
        uint16_t hdr_len = ice_mp_read_le16(buf + 18);
        return sizeof(struct ice_mp_sw_rule_lkup_rx_tx_fixed) + hdr_len;
    }

    if (type == ICE_MP_SW_RULE_T_LG_ACT) {
        if (len < 8) {
            return 0;
        }
        uint16_t act_count = ice_mp_read_le16(buf + 6);
        return 8 + (size_t)act_count * sizeof(uint32_t);
    }

    if (type == ICE_MP_SW_RULE_T_VSI_LIST) {
        if (len < 8) {
            return 0;
        }
        uint16_t num_vsi = ice_mp_read_le16(buf + 6);
        return 8 + (size_t)num_vsi * sizeof(uint16_t);
    }

    return 0;
}

static void ice_mp_sw_rule_program(struct ICEMPState *s, uint16_t rule_id,
                                   const struct ice_mp_sw_rule_lkup_rx_tx_fixed *fixed,
                                   const uint8_t *hdr_data, size_t hdr_len)
{
    uint16_t lkup_type = le16_to_cpu(fixed->recipe_id);
    uint16_t vsi_id;

    if (!ice_mp_sw_rule_extract_vsi(le32_to_cpu(fixed->act), &vsi_id)) {
        return;
    }

    if (lkup_type != ICE_MP_SW_LKUP_MAC &&
        lkup_type != ICE_MP_SW_LKUP_MAC_VLAN &&
        lkup_type != ICE_MP_SW_LKUP_PROMISC &&
        lkup_type != ICE_MP_SW_LKUP_PROMISC_VLAN) {
        return;
    }

    IceMpRxRule *rule = ice_mp_rule_alloc(s, rule_id);
    if (!rule) {
        return;
    }

    memset(rule->mac, 0, sizeof(rule->mac));
    rule->valid = true;
    rule->rule_id = rule_id;
    rule->vsi_id = vsi_id;
    rule->src = le16_to_cpu(fixed->src);
    rule->has_vlan = false;
    rule->vlan = 0;

    if (hdr_len >= ETH_ALEN) {
        memcpy(rule->mac, hdr_data + ICE_MP_ETH_DA_OFFSET, ETH_ALEN);
    }

    if (lkup_type == ICE_MP_SW_LKUP_MAC_VLAN ||
        lkup_type == ICE_MP_SW_LKUP_PROMISC_VLAN) {
        if (hdr_len >= 16) {
            uint16_t ethertype = ((uint16_t)hdr_data[ICE_MP_ETH_ETHTYPE_OFFSET] << 8) |
                                 hdr_data[ICE_MP_ETH_ETHTYPE_OFFSET + 1];
            if (ethertype == 0x8100) {
                rule->vlan = ((uint16_t)hdr_data[ICE_MP_ETH_VLAN_TCI_OFFSET] << 8) |
                             hdr_data[ICE_MP_ETH_VLAN_TCI_OFFSET + 1];
                rule->vlan &= ICE_MP_MAX_VLAN_ID;
                rule->has_vlan = true;
            }
        }
    }
}

static void ice_mp_update_txq_from_ctx(struct ICEMPState *s, uint16_t qid)
{
    if (qid >= ICE_MP_MAX_TX_QUEUES) {
        return;
    }

    const uint32_t *ctx = s->tx_ctx[qid];
    uint64_t base = ice_mp_extract_bits(ctx, 0, 57);
    uint64_t qlen = ice_mp_extract_bits(ctx, 135, 13);
    uint64_t port_num = ice_mp_extract_bits(ctx, 57, 3);
    uint64_t src_vsi = ice_mp_extract_bits(ctx, 80, 10);

    s->txq[qid].base = base << 7;
    s->txq[qid].qlen = qlen ? (uint16_t)qlen : 0;

    /* Determine port: prefer src_vsi lookup, fallback to port_num from context */
    if (src_vsi > 0 && src_vsi < ICE_MP_MAX_VSI) {
        s->txq[qid].port_id = s->vsi_to_port[src_vsi];
    } else if (port_num > 0) {
        s->txq[qid].port_id = (uint8_t)(port_num % s->num_ports);
    } else {
        s->txq[qid].port_id = 0;
    }

    /* Mark as configured if base address is valid (not just qlen) */
    s->txq[qid].configured = (base != 0);
    /* Enable queue via MMIO context path - needed when AdminQ Add TxQs
     * fails (e.g. scheduler EINVAL) but driver continues anyway */
    s->txq[qid].enabled = true;
    
    fprintf(stderr, "ice-mp: TX ctx update qid=%u base=0x%lx qlen=%lu port=%u src_vsi=%lu configured=%d enabled=%d\n",
            qid, s->txq[qid].base, qlen, s->txq[qid].port_id, src_vsi, s->txq[qid].configured, s->txq[qid].enabled);

    /* Propagate port change to RX queues that share the same MSI-X vector.
     * QINT_RQCTL matching happens at configuration time, but TX queue ports
     * may change during driver rebuild cycles. Re-sync RX ports whenever
     * a TX queue's port is (re)configured.
     */
    if (s->txq[qid].configured) {
        uint16_t tx_msix = s->txq[qid].int_ctl & ICE_MP_QINT_MSIX_INDX_M;
        for (uint16_t rxq = 0; rxq < ICE_MP_MAX_RX_QUEUES; rxq++) {
            uint16_t rx_msix = s->rxq[rxq].int_ctl & ICE_MP_QINT_MSIX_INDX_M;
            if (rx_msix == tx_msix && rx_msix != 0) {
                if (s->rxq[rxq].port_id != s->txq[qid].port_id) {
                    fprintf(stderr, "ice-mp: Syncing RX qid=%u port %u -> %u (TX qid=%u msix=%u)\n",
                            rxq, s->rxq[rxq].port_id, s->txq[qid].port_id, qid, tx_msix);
                    s->rxq[rxq].port_id = s->txq[qid].port_id;
                }
            }
        }
    }
}

static void ice_mp_update_rxq_from_ctx(struct ICEMPState *s, uint16_t qid)
{
    if (qid >= ICE_MP_MAX_RX_QUEUES) {
        return;
    }

    const uint32_t *ctx = s->rx_ctx[qid];
    uint64_t head = ice_mp_extract_bits(ctx, 0, 13);
    uint64_t base = ice_mp_extract_bits(ctx, 32, 57);
    uint64_t qlen = ice_mp_extract_bits(ctx, 89, 13);

    s->rxq[qid].base = base << 7;
    s->rxq[qid].qlen = qlen ? (uint16_t)qlen : 0;
    s->rxq[qid].configured = (s->rxq[qid].qlen > 0);
    s->rxq[qid].head = (uint16_t)head;

    if (!s->rxq[qid].configured) {
        return;
    }

    /* Do NOT set port_id here. The port_id is authoritatively set by the
     * QINT_RQCTL handler, which matches RX queues to TX queues via their
     * shared MSI-X vector. Setting port_id here would override the correct
     * QINT_RQCTL mapping when RX context is re-written during driver
     * rebuild cycles.
     */
}

/*
 * Software loopback responder for ARP and ICMP echo.
 * When the guest sends an ARP request or ICMP echo request, generate the
 * appropriate response and deliver it back via the RX path. This ensures
 * the datapath ping test works regardless of host TAP device configuration.
 */
static bool ice_mp_rx_enqueue(struct ICEMPState *s, uint8_t port_id,
                              const uint8_t *buf, size_t size);
static void ice_mp_tx_loopback(struct ICEMPState *s, uint8_t port_id,
                               const uint8_t *pkt, size_t len)
{
    if (len < 14) {
        return;
    }

    uint16_t ethertype = (pkt[12] << 8) | pkt[13];

    /* Handle ARP requests */
    if (ethertype == 0x0806 && len >= 42) {
        uint16_t oper = (pkt[20] << 8) | pkt[21];
        if (oper == 1) {  /* ARP Request */
            uint8_t reply[42];
            /* Fake peer MAC: 52:54:00:ee:ff:PP where PP = port_id */
            uint8_t peer_mac[6] = {0x52, 0x54, 0x00, 0xee, 0xff, port_id};

            /* Ethernet header: dst = sender MAC, src = peer MAC */
            memcpy(&reply[0], &pkt[6], 6);    /* dst = original sender */
            memcpy(&reply[6], peer_mac, 6);    /* src = peer MAC */
            reply[12] = 0x08; reply[13] = 0x06; /* ARP */

            /* ARP payload */
            reply[14] = 0x00; reply[15] = 0x01; /* htype = ethernet */
            reply[16] = 0x08; reply[17] = 0x00; /* ptype = IPv4 */
            reply[18] = 6;    /* hlen */
            reply[19] = 4;    /* plen */
            reply[20] = 0x00; reply[21] = 0x02; /* oper = reply */
            memcpy(&reply[22], peer_mac, 6);     /* sha = peer MAC */
            memcpy(&reply[28], &pkt[38], 4);     /* spa = target IP (what was requested) */
            memcpy(&reply[32], &pkt[6], 6);      /* tha = sender MAC (original requester) */
            memcpy(&reply[38], &pkt[28], 4);     /* tpa = sender IP */

            fprintf(stderr, "ice-mp: Loopback ARP reply port=%u "
                    "requested=%u.%u.%u.%u\n",
                    port_id, pkt[38], pkt[39], pkt[40], pkt[41]);
            ice_mp_rx_enqueue(s, port_id, reply, 42);
        }
    }
    /* Handle ICMP echo requests (IPv4 only) */
    else if (ethertype == 0x0800 && len >= 34) {
        uint8_t ihl = (pkt[14] & 0x0F) * 4;
        uint8_t proto = pkt[23];
        size_t icmp_off = 14 + ihl;

        if (proto == 1 && len >= icmp_off + 8) {  /* ICMP */
            uint8_t icmp_type = pkt[icmp_off];
            if (icmp_type == 8) {  /* Echo Request */
                uint8_t *reply = g_malloc(len);
                memcpy(reply, pkt, len);
                uint8_t peer_mac[6] = {0x52, 0x54, 0x00, 0xee, 0xff, port_id};

                /* Swap Ethernet MACs */
                memcpy(&reply[0], &pkt[6], 6);
                memcpy(&reply[6], peer_mac, 6);

                /* Swap IP src/dst */
                memcpy(&reply[26], &pkt[30], 4); /* dst IP = orig src */
                memcpy(&reply[30], &pkt[26], 4); /* src IP = orig dst */

                /* Set TTL */
                reply[22] = 64;

                /* Recalculate IP header checksum */
                reply[24] = 0; reply[25] = 0;
                uint32_t ip_sum = 0;
                for (int i = 14; i < 14 + ihl; i += 2) {
                    ip_sum += (reply[i] << 8) | reply[i + 1];
                }
                while (ip_sum >> 16) {
                    ip_sum = (ip_sum & 0xFFFF) + (ip_sum >> 16);
                }
                uint16_t ip_cksum = ~ip_sum & 0xFFFF;
                reply[24] = ip_cksum >> 8;
                reply[25] = ip_cksum & 0xFF;

                /* ICMP: type=0 (Echo Reply), recalculate checksum */
                reply[icmp_off] = 0;  /* type = echo reply */
                reply[icmp_off + 2] = 0; reply[icmp_off + 3] = 0; /* clear cksum */
                uint32_t icmp_sum = 0;
                size_t icmp_len = len - icmp_off;
                for (size_t i = 0; i < icmp_len; i += 2) {
                    uint16_t word = reply[icmp_off + i] << 8;
                    if (i + 1 < icmp_len) {
                        word |= reply[icmp_off + i + 1];
                    }
                    icmp_sum += word;
                }
                while (icmp_sum >> 16) {
                    icmp_sum = (icmp_sum & 0xFFFF) + (icmp_sum >> 16);
                }
                uint16_t icmp_cksum = ~icmp_sum & 0xFFFF;
                reply[icmp_off + 2] = icmp_cksum >> 8;
                reply[icmp_off + 3] = icmp_cksum & 0xFF;

                fprintf(stderr, "ice-mp: Loopback ICMP echo reply port=%u len=%zu\n",
                        port_id, len);
                ice_mp_rx_enqueue(s, port_id, reply, len);
                g_free(reply);
            }
        }
    }
}

static uint16_t ice_mp_tx_desc_buf_len(uint64_t qw1)
{
    return (uint16_t)((qw1 >> ICE_MP_TXD_QW1_TX_BUF_SZ_S) & 0x3FFF);
}

static uint16_t ice_mp_queue_size(const ICEMPQueue *q)
{
    /* If qlen is 0 but the queue is configured/enabled, use default size.
     * The Linux ICE driver may not populate qlen in the AdminQ TX context,
     * relying on other configuration mechanisms instead.
     */
    if (q->qlen == 0 && (q->configured || q->enabled)) {
        return 256;  /* ICE_DFLT_NUM_TX_DESC */
    }
    return q->qlen ? q->qlen : 0;
}

static void ice_mp_tx_process_queue(struct ICEMPState *s, uint16_t qid)
{
    if (qid >= ICE_MP_MAX_TX_QUEUES) {
        return;
    }

    ICEMPQueue *q = &s->txq[qid];
    uint16_t ring_size = ice_mp_queue_size(q);
    uint16_t head = q->head;
    uint16_t tail = q->tail;

    if (!q->configured || ring_size == 0 || head == tail) {
        fprintf(stderr, "ice-mp: TX early return qid=%u conf=%d size=%u head=%u tail=%u\n",
                qid, q->configured, ring_size, head, tail);
        return;
    }

    if (!q->enabled) {
        fprintf(stderr, "ice-mp: TX queue %u not enabled (conf=%d size=%u head=%u tail=%u)\n",
                qid, q->configured, ring_size, head, tail);
        return;
    }

    fprintf(stderr, "ice-mp: TX processing qid=%u head=%u tail=%u size=%u\n",
            qid, head, tail, ring_size);

    GByteArray *pkt = g_byte_array_new();

    while (head != tail) {
        uint64_t desc_addr = q->base + ((uint64_t)head * ICE_MP_TX_DESC_SIZE);
        struct ice_mp_tx_desc desc;
        uint64_t qw1;
        uint16_t cmd;

        pci_dma_read(&s->parent_obj, desc_addr, &desc, sizeof(desc));
        qw1 = le64_to_cpu(desc.cmd_type_offset_bsz);
        cmd = (qw1 >> ICE_MP_TXD_QW1_CMD_S) & 0xFFF;

        uint8_t dtype = qw1 & 0xFULL;
        fprintf(stderr, "ice-mp: TX desc qid=%u head=%u base=0x%lx desc_addr=0x%lx dtype=0x%x cmd=0x%x qw1=0x%lx\n",
                qid, head, q->base, desc_addr, dtype, cmd, qw1);

        if ((qw1 & 0xFULL) == ICE_TX_DESC_DTYPE_CTX) {
            head = (head + 1) % ring_size;
            continue;
        }

        if ((qw1 & 0xFULL) == ICE_TX_DESC_DTYPE_DATA) {
            uint16_t len = ice_mp_tx_desc_buf_len(qw1);

            if (len) {
                uint8_t *buf = g_malloc(len);
                pci_dma_read(&s->parent_obj, le64_to_cpu(desc.buf_addr), buf, len);
                g_byte_array_append(pkt, buf, len);
                g_free(buf);
            }

            if (cmd & ICE_TX_DESC_CMD_EOP) {
                if (pkt->len && s->nic[q->port_id]) {
                    NetClientState *nc = qemu_get_queue(s->nic[q->port_id]);
                    fprintf(stderr, "ice-mp: Sending packet qid=%u port=%u len=%u\n",
                            qid, q->port_id, pkt->len);
                    qemu_send_packet(nc, pkt->data, pkt->len);
                    /* Generate loopback ARP/ICMP responses for datapath */
                    ice_mp_tx_loopback(s, q->port_id, pkt->data, pkt->len);
                } else {
                    fprintf(stderr, "ice-mp: Skipping packet qid=%u port=%u pkt_len=%u nic=%p\n",
                            qid, q->port_id, pkt->len, s->nic[q->port_id]);
                }

                desc.cmd_type_offset_bsz =
                    cpu_to_le64((qw1 & ~0xFULL) | ICE_TX_DESC_DTYPE_DESC_DONE);
                /* Write only the 8-byte cmd_type_offset_bsz at offset +8
                 * to avoid overwriting buf_addr unnecessarily */
                uint64_t wb_addr = desc_addr + offsetof(struct ice_mp_tx_desc, cmd_type_offset_bsz);
                pci_dma_write(&s->parent_obj, wb_addr,
                              &desc.cmd_type_offset_bsz, sizeof(desc.cmd_type_offset_bsz));

                /* Readback verification */
                uint64_t readback = 0;
                pci_dma_read(&s->parent_obj, wb_addr, &readback, sizeof(readback));
                fprintf(stderr, "ice-mp: DD writeback qid=%u head=%u desc_addr=0x%lx "
                        "wb_addr=0x%lx written=0x%lx readback=0x%lx match=%d\n",
                        qid, head, (unsigned long)desc_addr, (unsigned long)wb_addr,
                        (unsigned long)le64_to_cpu(desc.cmd_type_offset_bsz),
                        (unsigned long)le64_to_cpu(readback),
                        (desc.cmd_type_offset_bsz == readback));

                bool msix_on = msix_enabled(&s->parent_obj);
                bool cause_ena = !!(q->int_ctl & ICE_MP_QINT_CAUSE_ENA_M);
                uint16_t msix_idx = q->int_ctl & ICE_MP_QINT_MSIX_INDX_M;
                
                if (msix_on && cause_ena && msix_idx < ICE_MP_MSIX_VECTORS) {
                    bool masked = msix_is_masked(&s->parent_obj, msix_idx);

                    fprintf(stderr,
                            "ice-mp: TX IRQ qid=%u vec=%u masked=%d\n",
                            qid, msix_idx, masked);
                    msix_notify(&s->parent_obj, msix_idx);
                }

                g_byte_array_set_size(pkt, 0);
            }
        }

        head = (head + 1) % ring_size;
    }

    q->head = head;
    g_byte_array_free(pkt, true);
}

static uint32_t ice_mp_rx_buf_len(const uint32_t *ctx)
{
    uint64_t dbuf = ice_mp_extract_bits(ctx, 102, 7);
    if (!dbuf) {
        return 0;
    }

    return (uint32_t)(dbuf << 7);
}

static bool ice_mp_rx_enqueue_queue(struct ICEMPState *s, uint16_t qid,
                                    const uint8_t *buf, size_t size)
{
    if (qid >= ICE_MP_MAX_RX_QUEUES) {
        return false;
    }

    ICEMPQueue *q = &s->rxq[qid];
    uint16_t ring_size = ice_mp_queue_size(q);
    size_t remaining = size;
    const uint8_t *cursor = buf;

    if (!q->configured || !q->enabled || ring_size == 0 || q->head == q->tail) {
        return false;
    }

    while (q->head != q->tail && remaining) {
        uint64_t desc_addr = q->base + ((uint64_t)q->head * ICE_MP_RX_DESC_SIZE);
        union ice_mp_rx_desc desc;
        uint32_t copy_len;
        uint32_t buf_len = ice_mp_rx_buf_len(s->rx_ctx[qid]);

        pci_dma_read(&s->parent_obj, desc_addr, &desc, sizeof(desc));

        if (!desc.read.pkt_addr || !buf_len) {
            return false;
        }

        copy_len = (remaining > buf_len) ? buf_len : (uint32_t)remaining;
        pci_dma_write(&s->parent_obj, le64_to_cpu(desc.read.pkt_addr),
                      cursor, copy_len);

        memset(&desc, 0, sizeof(desc));
        desc.wb.rxdid = ICE_RXDID_FLEX_NIC;
        desc.wb.pkt_len = cpu_to_le16(copy_len);
        desc.wb.status_error0 = cpu_to_le16(ICE_MP_RX_STATUS0_DD |
                                            (remaining > buf_len ? 0 : ICE_MP_RX_STATUS0_EOF));

        pci_dma_write(&s->parent_obj, desc_addr, &desc, sizeof(desc));

        q->head = (q->head + 1) % ring_size;
        cursor += copy_len;
        remaining -= copy_len;

        if (msix_enabled(&s->parent_obj) && (q->int_ctl & ICE_MP_QINT_CAUSE_ENA_M)) {
            uint16_t msix_idx = q->int_ctl & ICE_MP_QINT_MSIX_INDX_M;
            
            if (msix_idx < ICE_MP_MSIX_VECTORS) {
                bool masked = msix_is_masked(&s->parent_obj, msix_idx);

                fprintf(stderr,
                        "ice-mp: RX IRQ qid=%u vec=%u masked=%d\n",
                        qid, msix_idx, masked);
                msix_notify(&s->parent_obj, msix_idx);
            }
        }
    }

    return remaining == 0;
}

static bool ice_mp_rx_enqueue(struct ICEMPState *s, uint8_t port_id,
                              const uint8_t *buf, size_t size)
{
    for (uint16_t qid = 0; qid < ICE_MP_MAX_RX_QUEUES; qid++) {
        ICEMPQueue *q = &s->rxq[qid];
        if (!q->configured || !q->enabled || q->port_id != port_id) {
            continue;
        }

        if (ice_mp_rx_enqueue_queue(s, qid, buf, size)) {
            return true;
        }
    }

    return false;
}

static bool ice_mp_can_receive(NetClientState *nc)
{
    ICEMPPort *port = qemu_get_nic_opaque(nc);
    ICEMPState *s = port->s;

    for (uint16_t qid = 0; qid < ICE_MP_MAX_RX_QUEUES; qid++) {
        ICEMPQueue *q = &s->rxq[qid];
        if (!q->configured || !q->enabled || q->port_id != port->port_id) {
            continue;
        }
        if (q->head != q->tail) {
            return true;
        }
    }

    /* Debug: log when can_receive returns false for a port that has queues */
    static uint32_t can_recv_debug_count = 0;
    if (can_recv_debug_count < 20) {
        bool has_queues = false;
        for (uint16_t qid = 0; qid < ICE_MP_MAX_RX_QUEUES; qid++) {
            ICEMPQueue *q = &s->rxq[qid];
            if (q->configured || q->enabled) {
                if (!has_queues) {
                    fprintf(stderr, "ice-mp: can_receive port=%u returning false. RX queues:\n", port->port_id);
                    has_queues = true;
                }
                fprintf(stderr, "  qid=%u configured=%d enabled=%d port_id=%u head=%u tail=%u base=0x%lx qlen=%u\n",
                        qid, q->configured, q->enabled, q->port_id, q->head, q->tail, q->base, q->qlen);
            }
        }
        if (has_queues) can_recv_debug_count++;
    }

    return false;
}

static ssize_t ice_mp_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    ICEMPPort *port = qemu_get_nic_opaque(nc);
    ICEMPState *s = port->s;
    uint16_t vf_id;

    fprintf(stderr, "ice-mp: receive called port=%u size=%zu\n", port->port_id, size);

    vf_id = ice_mp_select_vf(s, port->port_id, buf, size);
    if (vf_id < s->num_vfs && s->vf_rxq_mapena[vf_id] && s->vf_rxq_num[vf_id]) {
        uint16_t base = s->vf_rxq_base[vf_id];
        uint16_t count = s->vf_rxq_num[vf_id];
        uint16_t qid = base + (buf[ICE_MP_ETH_DA_OFFSET] % count);

        if (ice_mp_rx_enqueue_queue(s, qid, buf, size)) {
            return size;
        }
    }

    if (ice_mp_rx_enqueue(s, port->port_id, buf, size)) {
        return size;
    }

    return 0;
}

static void ice_mp_link_status_changed(NetClientState *nc)
{
    ICEMPPort *port = qemu_get_nic_opaque(nc);
    ICEMPState *s = port->s;
    uint8_t port_id = port->port_id;
    bool link_up = !nc->link_down;

    if (port_id >= s->num_ports) {
        return;
    }

    fprintf(stderr, "ice-mp: link_status_changed port=%u link_up=%d\n",
            port_id, link_up);

    if (link_up) {
        s->port_status[port_id] = ICE_MP_PORT_LINK_UP |
                                   (ICE_MP_SPEED_100G << ICE_MP_PORT_SPEED_SHIFT);
    } else {
        s->port_status[port_id] = (ICE_MP_SPEED_100G << ICE_MP_PORT_SPEED_SHIFT);
    }

    /* Fire link change event interrupt if MSI-X is enabled */
    if (msix_enabled(&s->parent_obj)) {
        s->event_doorbell = (ICE_MP_EVENT_LINK_CHANGE << 8) | port_id;
        if (msix_enabled(&s->parent_obj) &&
            !msix_is_masked(&s->parent_obj, 0)) {
            msix_notify(&s->parent_obj, 0);
        }
    }
}

static NetClientInfo net_ice_mp_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NetClientState),
    .receive = ice_mp_receive,
    .can_receive = ice_mp_can_receive,
    .link_status_changed = ice_mp_link_status_changed,
};

static bool ice_mp_mac_is_zero(const MACAddr *mac)
{
    for (size_t i = 0; i < sizeof(mac->a); i++) {
        if (mac->a[i] != 0) {
            return false;
        }
    }

    return true;
}

static void ice_mp_init_nics(ICEMPState *s, PCIDevice *pdev)
{
    DeviceState *dev = DEVICE(pdev);

    for (uint32_t i = 0; i < s->num_ports; i++) {
        char *name;

        s->ports[i].s = s;
        s->ports[i].port_id = (uint8_t)i;

        if (i == 0) {
            qemu_macaddr_default_if_unset(&s->conf[i].macaddr);
        } else if (ice_mp_mac_is_zero(&s->conf[i].macaddr)) {
            s->conf[i].macaddr = s->conf[0].macaddr;
            s->conf[i].macaddr.a[5] += (uint8_t)i;
        }

        name = g_strdup_printf("%s-port%u", dev->id ? dev->id : "ice-mp", i);
        s->nic[i] = qemu_new_nic(&net_ice_mp_info, &s->conf[i],
                                object_get_typename(OBJECT(s)), name,
                                &dev->mem_reentrancy_guard, &s->ports[i]);
        g_free(name);

        qemu_format_nic_info_str(qemu_get_queue(s->nic[i]),
                                 s->conf[i].macaddr.a);
    }
}

static uint16_t ice_mp_nvm_shadow_word(uint16_t word)
{
    uint16_t val;
    /* Shadow RAM immediate access (for addresses 0x00-0x32)
     * These are critical configuration words that the driver reads
     * before accessing the full NVM structure.
     */
    switch (word) {
    case 0x00:
        /* Control word - bits[7:6] = 0x1 (VALID)
         * Driver checks: FIELD_GET(ICE_SR_CTRL_WORD_1_M, ctrl_word) == ICE_SR_CTRL_WORD_VALID
         * Where ICE_SR_CTRL_WORD_1_M = (0x03 << 6) = 0xC0 and ICE_SR_CTRL_WORD_VALID = 0x1
         * So bits 7:6 should be 0x1, meaning ctrl_word = 0x40
         */
        val = 0x0040;  /* Bits 7:6 = 0x1 = VALID */
        break;
    case 0x18:
        /* NVM version (major.minor) */
        val = 0x0100;
        break;
    case 0x2D:
        /* EETRACK low byte */
        val = 0x0000;
        break;
    case 0x2E:
        /* EETRACK high byte */
        val = 0x0000;
        break;
    case 0x42:
        /* 1st NVM bank ptr - use 4KB units (bit 15 set)
         * Point to 4KB unit 1 = 4096 bytes */
        val = 0x8001;  /* BIT(15) | 0x0001 */
        break;
    case 0x43:
        /* NVM bank size in 4KB units - report 128 * 4KB = 512KB */
        val = 0x0080;
        break;
    case 0x44:
        /* OROM bank ptr - 4KB unit 2 = 8192 bytes */
        val = 0x8002;  /* BIT(15) | 0x0002 */
        break;
    case 0x45:
        /* OROM bank size in 4KB units - report 32 * 4KB = 128KB */
        val = 0x0020;
        break;
    case 0x46:
        /* Netlist bank ptr - 4KB unit 3 = 12288 bytes */
        val = 0x8003;  /* BIT(15) | 0x0003 */
        break;
    case 0x47:
        /* Netlist bank size in 4KB units - report 16 * 4KB = 64KB */
        val = 0x0010;
        break;
    default:
        val = 0x0000;
        break;
    }
    
    if (word <= 0x47) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ice-mp: Shadow RAM[0x%02x] = 0x%04x\n", word, val);
    }
    
    return val;
}

static uint16_t ice_mp_nvm_read_from_bank(struct ICEMPState *s, uint16_t word,
                                           uint16_t *bank, uint16_t max_words)
{
    if (word < max_words) {
        return bank[word];
    }
    return 0;
}

static void ice_mp_adminq_nvm_read(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_nvm *cmd = (struct ice_mp_aqc_nvm *)desc->params;
    uint32_t offset = le16_to_cpu(cmd->offset_low) | (cmd->offset_high << 16);
    uint16_t length = le16_to_cpu(cmd->length);
    uint16_t module_typeid = le16_to_cpu(cmd->module_typeid);
    uint8_t cmd_flags = cmd->cmd_flags;
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(cmd->addr_low),
                                    le32_to_cpu(cmd->addr_high));
    uint8_t *buf = g_malloc0(length);
    uint32_t i;
    bool flash_only = (cmd_flags & BIT(7)) != 0;  /* ICE_AQC_NVM_FLASH_ONLY */

    fprintf(stderr,
            "ice-mp: NVM read - module=0x%04x offset=0x%x len=%u flags=0x%02x %s\n",
            module_typeid, offset, length, cmd_flags,
            flash_only ? "(flash)" : "(shadow)");

    /* Determine what to read based on module_typeid and cmd_flags.
     * When module_typeid=0:
     *   - If FLASH_ONLY flag is set: read from NVM flash bank (0x0100)
     *   - If FLASH_ONLY flag is clear: read from Shadow RAM (immediate)
     * Otherwise use module_typeid to select bank.
     */
    
    if (module_typeid == 0 && flash_only) {
        /* Flash read with module=0 - direct read from NVM flash array */
        uint16_t word_offset = offset / 2;
        for (i = 0; i + 1 < length; i += 2) {
            uint16_t val = ice_mp_nvm_read_from_bank(s, word_offset + (i / 2),
                                                      s->nvm_flash, s->nvm_flash_size);
            buf[i] = (uint8_t)(val & 0xFF);
            buf[i + 1] = (uint8_t)((val >> 8) & 0xFF);
        }
    } else if (module_typeid == 0 || (module_typeid & 0xFF00) == 0x0000) {
        /* Shadow RAM immediate read - module=0 without FLASH_ONLY flag.
         * This is used by driver to read config like bank pointers, NVM version, etc.
         * No CSS header wrapping - just the raw shadow RAM values.
         */
        uint16_t word_offset = offset / 2;
        for (i = 0; i + 1 < length; i += 2) {
            uint16_t val = ice_mp_nvm_shadow_word(word_offset + (i / 2));
            buf[i] = (uint8_t)(val & 0xFF);
            buf[i + 1] = (uint8_t)((val >> 8) & 0xFF);
        }
    } else if ((module_typeid & 0xFF00) == 0x0100) {
        /* NVM bank read - read from NVM flash bank including CSS header.
         * This is used to read the CSS header and shadow RAM copy from flash.
         */
        uint16_t word_offset = offset / 2;
        for (i = 0; i + 1 < length; i += 2) {
            uint16_t val = ice_mp_nvm_read_from_bank(s, word_offset + (i / 2),
                                                      s->nvm_flash, s->nvm_flash_size);
            buf[i] = (uint8_t)(val & 0xFF);
            buf[i + 1] = (uint8_t)((val >> 8) & 0xFF);
        }
    } else if ((module_typeid & 0xFF00) == 0x0200) {
        /* OROM bank read */
        uint16_t word_offset = offset / 2;
        for (i = 0; i + 1 < length; i += 2) {
            uint16_t val = ice_mp_nvm_read_from_bank(s, word_offset + (i / 2),
                                                      s->orom_flash, s->orom_flash_size);
            buf[i] = (uint8_t)(val & 0xFF);
            buf[i + 1] = (uint8_t)((val >> 8) & 0xFF);
        }
    } else if ((module_typeid & 0xFF00) == 0x0300) {
        /* Netlist bank read */
        uint16_t word_offset = offset / 2;
        for (i = 0; i + 1 < length; i += 2) {
            uint16_t val = ice_mp_nvm_read_from_bank(s, word_offset + (i / 2),
                                                      s->netlist_flash, s->netlist_flash_size);
            buf[i] = (uint8_t)(val & 0xFF);
            buf[i + 1] = (uint8_t)((val >> 8) & 0xFF);
        }
    } else {
        /* Unknown module, return zeros */
        qemu_log("ice-mp: NVM read unknown module type 0x%04x\n", module_typeid);
    }

    pci_dma_write(&s->parent_obj, addr, buf, length);
    g_free(buf);

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_add_tx_queues(struct ICEMPState *s,
                                        struct ice_mp_aq_desc *desc)
{
    /* Parse the Add Tx LAN Queues command buffer */
    /* 0x0C30 uses addr_high at params[8] and addr_low at params[12] */
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));
    uint16_t buf_len = le16_to_cpu(desc->datalen);
    uint8_t *buf;
    uint32_t parent_teid;
    uint8_t num_txqs;

    /* Ensure command returns success unless explicitly set otherwise */
    desc->retval = cpu_to_le16(0);

    if (!addr || !buf_len) {
        fprintf(stderr, "ice-mp: Add Tx Queues - no buffer provided\n");
        ice_mp_adminq_complete(s, desc);
        return;
    }

    /* Read the command buffer */
    buf = g_malloc(buf_len);
    pci_dma_read(&s->parent_obj, addr, buf, buf_len);

    /* Parse ice_aqc_add_tx_qgrp structure:
     *   parent_teid: 4 bytes
     *   num_txqs: 1 byte
     *   rsvd: 3 bytes
     *   txqs[]: array of ice_aqc_add_txqs_perq structures
     */
    memcpy(&parent_teid, buf, 4);
    parent_teid = le32_to_cpu(parent_teid);
    num_txqs = buf[4];

    fprintf(stderr, "ice-mp: Add Tx Queues - num_txqs=%u parent_teid=0x%x\n",
            num_txqs, parent_teid);

    /* Process each queue in the list */
    for (int i = 0; i < num_txqs; i++) {
        /* Each ice_aqc_add_txqs_perq entry:
         *   txq_id: 2 bytes (offset 8 + i*48)
         *   rsvd: 2 bytes
         *   q_teid: 4 bytes
         *   txq_ctx: 22 bytes  ← TX CONTEXT WITH BASE AND QLEN!
         *   rsvd2: 2 bytes
         *   info (ice_aqc_txsched_elem): 16 bytes
         * Total size: 48 bytes per queue
         */
        size_t offset = 8 + (i * 48);
        if (offset + 48 > buf_len) {
            fprintf(stderr, "ice-mp: Add Tx Queues - buffer too small for queue %d\n", i);
            break;
        }

        uint16_t txq_id;
        memcpy(&txq_id, buf + offset, 2);
        txq_id = le16_to_cpu(txq_id);

        if (txq_id < ICE_MP_MAX_TX_QUEUES) {
            /* Parse the 22-byte TX context to get base address, qlen, and port */
            const uint8_t *txq_ctx = buf + offset + 8;  /* Skip txq_id(2) + rsvd(2) + q_teid(4) */
            
            /* Debug: dump the raw 22-byte context */
            fprintf(stderr, "ice-mp: TX ctx raw bytes for qid=%u: ", txq_id);
            for (int j = 0; j < 22; j++) {
                fprintf(stderr, "%02x ", txq_ctx[j]);
            }
            fprintf(stderr, "\n");
            
            /* Treat the 22-byte context as an array of 32-bit words for bit extraction
             * The context fields are:
             *   base: 57 bits at bit 0
             *   port_num: 3 bits at bit 57
             *   src_vsi: 10 bits at bit 80
             *   qlen: 13 bits at bit 135
             */
            uint32_t ctx_words[6];  /* 22 bytes = 5.5 words, round up to 6 */
            memset(ctx_words, 0, sizeof(ctx_words));
            memcpy(ctx_words, txq_ctx, 22);
            
            /* Convert to little endian */
            for (int j = 0; j < 6; j++) {
                ctx_words[j] = le32_to_cpu(ctx_words[j]);
            }
            
            /* Extract fields using bit extraction */
            uint64_t base = ice_mp_extract_bits(ctx_words, 0, 57);
            uint64_t port_num = ice_mp_extract_bits(ctx_words, 57, 3);
            uint64_t src_vsi = ice_mp_extract_bits(ctx_words, 80, 10);
            uint64_t qlen = ice_mp_extract_bits(ctx_words, 135, 13);
            
            /* Configure the queue */
            s->txq[txq_id].base = base << 7;  /* Base is in units of 128 bytes */
            s->txq[txq_id].qlen = qlen ? (uint16_t)qlen : 0;

            /* Determine port: prefer src_vsi lookup, fallback to port_num */
            if (src_vsi > 0 && src_vsi < ICE_MP_MAX_VSI) {
                s->txq[txq_id].port_id = s->vsi_to_port[src_vsi];
            } else if (port_num > 0) {
                s->txq[txq_id].port_id = (uint8_t)(port_num % s->num_ports);
            } else {
                s->txq[txq_id].port_id = 0;
            }

            /* Reset head/tail on queue reconfiguration */
            s->txq[txq_id].head = 0;
            s->txq[txq_id].tail = 0;
            /* Mark as configured if base address is valid, even if qlen=0 */
            s->txq[txq_id].configured = (base != 0);
            s->txq[txq_id].enabled = true;
            
            fprintf(stderr, "ice-mp: Enabling TX queue %u: base=0x%lx qlen=%lu port=%u src_vsi=%lu configured=%d\n",
                    txq_id, s->txq[txq_id].base, qlen, s->txq[txq_id].port_id, src_vsi, s->txq[txq_id].configured);

            /* Propagate port change to RX queues sharing same MSI-X vector */
            if (s->txq[txq_id].configured) {
                uint16_t tx_msix = s->txq[txq_id].int_ctl & ICE_MP_QINT_MSIX_INDX_M;
                for (uint16_t rxq = 0; rxq < ICE_MP_MAX_RX_QUEUES; rxq++) {
                    uint16_t rx_msix = s->rxq[rxq].int_ctl & ICE_MP_QINT_MSIX_INDX_M;
                    if (rx_msix == tx_msix && rx_msix != 0) {
                        if (s->rxq[rxq].port_id != s->txq[txq_id].port_id) {
                            fprintf(stderr, "ice-mp: Syncing RX qid=%u port %u -> %u (TX qid=%u msix=%u)\n",
                                    rxq, s->rxq[rxq].port_id, s->txq[txq_id].port_id, txq_id, tx_msix);
                            s->rxq[rxq].port_id = s->txq[txq_id].port_id;
                        }
                    }
                }
            }

            /* Allocate a TEID for the queue and write it back */
            uint32_t q_teid = cpu_to_le32(s->next_sched_teid++);
            memcpy(buf + offset + 4, &q_teid, 4);
        } else {
            fprintf(stderr, "ice-mp: Warning - TX queue ID %u out of range\n", txq_id);
        }
    }

    /* Write the buffer back with assigned TEIDs */
    pci_dma_write(&s->parent_obj, addr, buf, buf_len);
    g_free(buf);

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_sw_rules(struct ICEMPState *s,
                                   struct ice_mp_aq_desc *desc,
                                   uint16_t opcode)
{
    uint16_t rule_count = le16_to_cpu(*(uint16_t *)&desc->params[0]);
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));
    uint16_t buf_len = le16_to_cpu(desc->datalen);
    uint8_t *buf;
    size_t offset = 0;

    if (!addr || !rule_count || !buf_len) {
        ice_mp_adminq_complete(s, desc);
        return;
    }

    buf = g_malloc0(buf_len);
    pci_dma_read(&s->parent_obj, addr, buf, buf_len);

    for (uint16_t i = 0; i < rule_count && offset < buf_len; i++) {
        size_t elem_len = ice_mp_sw_rule_elem_size(buf + offset, buf_len - offset);
        uint16_t type;

        if (!elem_len || offset + elem_len > buf_len) {
            break;
        }

        type = ice_mp_read_le16(buf + offset);

        if (type == ICE_MP_SW_RULE_T_LKUP_RX || type == ICE_MP_SW_RULE_T_LKUP_TX) {
            struct ice_mp_sw_rule_lkup_rx_tx_fixed fixed;
            uint16_t rule_id;
            uint16_t status = 0;

            memcpy(&fixed, buf + offset, sizeof(fixed));

            rule_id = le16_to_cpu(fixed.index);
            if (opcode == 0x02A0 && rule_id == 0) {
                rule_id = s->next_rule_id++;
            }

            if (opcode == 0x02A2) {
                IceMpRxRule *rule = ice_mp_rule_find_by_id(s, rule_id);
                if (rule) {
                    rule->valid = false;
                }
            } else if (type == ICE_MP_SW_RULE_T_LKUP_RX) {
                const uint8_t *hdr_data = buf + offset + sizeof(fixed);
                size_t hdr_len = elem_len - sizeof(fixed);
                ice_mp_sw_rule_program(s, rule_id, &fixed, hdr_data, hdr_len);
            }

            memcpy(buf + offset + offsetof(struct ice_mp_sw_rule_lkup_rx_tx_fixed, index),
                   &((uint16_t){cpu_to_le16(rule_id)}), sizeof(uint16_t));
            memcpy(buf + offset + offsetof(struct ice_mp_sw_rule_hdr, status),
                   &((uint16_t){cpu_to_le16(status)}), sizeof(uint16_t));
        } else if (type == ICE_MP_SW_RULE_T_LG_ACT || type == ICE_MP_SW_RULE_T_VSI_LIST) {
            uint16_t status = 0;
            memcpy(buf + offset + offsetof(struct ice_mp_sw_rule_hdr, status),
                   &((uint16_t){cpu_to_le16(status)}), sizeof(uint16_t));
        }

        offset += elem_len;
    }

    pci_dma_write(&s->parent_obj, addr, buf, buf_len);
    g_free(buf);

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_complete(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    uint16_t opcode = le16_to_cpu(desc->opcode);
    
    /* Only set retval to 0 if handler hasn't set it already */
    if (desc->retval == 0) {
        desc->retval = cpu_to_le16(0);
    }
    
    /* Debug for 0x0401 */
    if (opcode == 0x0401) {
        fprintf(stderr, "ice-mp: complete() BEFORE flags: flags=0x%04x retval=0x%04x\n",
                le16_to_cpu(desc->flags), le16_to_cpu(desc->retval));
    }
    
    /* Clear any stale error bits and return only DD/CMP (+BUF if present). */
    desc->flags = cpu_to_le16((le16_to_cpu(desc->flags) & ICE_MP_AQ_FLAG_BUF) |
                              ICE_MP_AQ_FLAG_DD | ICE_MP_AQ_FLAG_CMP);
    
    /* Debug for 0x0401 */
    if (opcode == 0x0401) {
        fprintf(stderr, "ice-mp: complete() AFTER flags: flags=0x%04x retval=0x%04x\n",
                le16_to_cpu(desc->flags), le16_to_cpu(desc->retval));
    }
}

/* Get PHY Capabilities (indirect 0x0600) */
static void ice_mp_adminq_get_phy_caps(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    /*
     * Return PHY capabilities matching ice_aqc_get_phy_caps_data layout.
     * The driver uses this to discover supported PHY types and link modes.
     * Without proper data, the driver may not attempt to bring the link up.
     */
    struct {
        uint64_t phy_type_low;     /* offset 0 */
        uint64_t phy_type_high;    /* offset 8 */
        uint8_t caps;              /* offset 16 */
        uint8_t low_power_ctrl_an; /* offset 17 */
        uint16_t eee_cap;          /* offset 18 */
        uint16_t eeer_value;       /* offset 20 */
        uint8_t phy_id_oui[4];    /* offset 22 */
        uint8_t phy_fw_ver[8];    /* offset 26 */
        uint8_t link_fec_options;  /* offset 34 */
        uint8_t module_compliance_enforcement; /* offset 35 */
        uint8_t extended_compliance_code;      /* offset 36 */
        uint8_t module_type[3];    /* offset 37 */
    } __attribute__((packed)) pcaps;

    uint64_t addr;
    uint8_t port_num = desc->params[0] & 0xFF;

    if (port_num >= s->num_ports) {
        port_num = 0;
    }

    memset(&pcaps, 0, sizeof(pcaps));

    /* 100GBASE-CR4 = bit 10 in phy_type_low */
    pcaps.phy_type_low = cpu_to_le64(
        (1ULL << 10) |  /* ICE_PHY_TYPE_LOW_100GBASE_CR4 */
        (1ULL << 11) |  /* ICE_PHY_TYPE_LOW_100GBASE_SR4 */
        (1ULL << 12)    /* ICE_PHY_TYPE_LOW_100GBASE_LR4 */
    );
    pcaps.phy_type_high = 0;

    /* caps: EN_LINK (bit 3) must be set for the driver to consider link capable */
    pcaps.caps = (1 << 3) |  /* ICE_AQC_PHY_EN_LINK */
                 (1 << 0) |  /* ICE_AQC_PHY_EN_TX_LINK_PAUSE */
                 (1 << 1);   /* ICE_AQC_PHY_EN_RX_LINK_PAUSE */

    pcaps.low_power_ctrl_an = 0;
    pcaps.eee_cap = 0;
    pcaps.eeer_value = 0;

    /* FEC options */
    pcaps.link_fec_options = (1 << 6);  /* ICE_AQC_PHY_FEC_25G_RS_CLAUSE91_EN */

    /* Module type - QSFP28 */
    pcaps.module_type[0] = 0x03;  /* SFF_8636 module */

    /* Write PHY caps to DMA buffer.
     * The driver passes addr via desc params: addr_high at params[2], addr_low at params[3].
     */
    addr = ((uint64_t)le32_to_cpu(*(uint32_t *)&desc->params[8]) << 32) |
            le32_to_cpu(*(uint32_t *)&desc->params[12]);

    if (addr) {
        /* Write what we have. The driver buffer (ice_aqc_get_phy_caps_data)
         * may be larger but zeros are fine for the rest since we memset 0. */
        pci_dma_write(PCI_DEVICE(s), addr, &pcaps, sizeof(pcaps));
        fprintf(stderr, "ice-mp: GET_PHY_CAPS port=%u: wrote %zu bytes to DMA 0x%lx "
                "(phy_type_low=0x%lx caps=0x%x)\n",
                port_num, sizeof(pcaps), (unsigned long)addr,
                (unsigned long)le64_to_cpu(pcaps.phy_type_low), pcaps.caps);
    } else {
        fprintf(stderr, "ice-mp: GET_PHY_CAPS port=%u: no DMA addr!\n", port_num);
    }

    ice_mp_adminq_complete(s, desc);
}

/* Set PHY Config (indirect 0x0601) - Handle link up/down */
static void ice_mp_adminq_set_phy_cfg(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct {
        uint64_t phy_type_low;
        uint64_t phy_type_high;
        uint8_t caps;
        uint8_t low_power_ctrl_an;
        uint16_t eee_cap;
        uint16_t eeer_value;
        uint8_t link_fec_opt;
        uint8_t module_compliance_enforcement;
    } __attribute__((packed)) phy_cfg;
    uint64_t addr;
    uint8_t port_num = desc->params[0] & 0xFF;
    bool link_changed = false;
    uint32_t old_status;
    
    /* Read PHY config from DMA buffer */
    addr = ((uint64_t)le32_to_cpu(desc->params[2]) << 32) |
           le32_to_cpu(desc->params[3]);
    
    if (addr) {
        pci_dma_read(PCI_DEVICE(s), addr, &phy_cfg, sizeof(phy_cfg));
    }
    
    /* Default to port 0 for multi-port */
    if (port_num >= s->num_ports) {
        port_num = 0;
    }
    
    old_status = s->port_status[port_num];
    
    /* Check if link should be enabled or disabled */
    /* Bit 0 of caps = link enabled */
    if (phy_cfg.caps & 0x01) {
        /* Enable link - set link up at 100Gbps */
        s->port_status[port_num] = ICE_MP_PORT_LINK_UP | 
                                   (ICE_MP_SPEED_100G << ICE_MP_PORT_SPEED_SHIFT);
        if (!(old_status & ICE_MP_PORT_LINK_UP)) {
            link_changed = true;
            fprintf(stderr, "ice-mp: Port %u link set to UP (100Gbps)\n", port_num);
        }
    } else {
        /* Disable link */
        s->port_status[port_num] = (ICE_MP_SPEED_100G << ICE_MP_PORT_SPEED_SHIFT);
        if (old_status & ICE_MP_PORT_LINK_UP) {
            link_changed = true;
            fprintf(stderr, "ice-mp: Port %u link set to DOWN\n", port_num);
        }
    }
    
    /* Send link change event interrupt if link state changed */
    if (link_changed && msix_enabled(&s->parent_obj)) {
        s->event_doorbell = (ICE_MP_EVENT_LINK_CHANGE << 8) | port_num;
        msix_notify(&s->parent_obj, 0);  /* Vector 0 = OICR */
        fprintf(stderr, "ice-mp: Sending link change interrupt for port %u\n", port_num);
    }
    
    ice_mp_adminq_complete(s, desc);
}

/* Update VSI (indirect 0x0211) */
static void ice_mp_adminq_update_vsi(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));
    uint16_t vsi_num = le16_to_cpu(*(uint16_t *)&desc->params[0]);
    uint8_t vf_id = desc->params[4];
    
    if (addr) {
        /* Read VSI context from buffer and write it back */
        uint8_t vsi_ctx[128];  /* ice_aqc_vsi_props structure */
        pci_dma_read(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
        pci_dma_write(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
    }

    if (vsi_num < ICE_MP_MAX_VSI) {
        if (vf_id < s->num_vfs) {
            s->vsi_to_vf[vsi_num] = vf_id;
        } else {
            s->vsi_to_vf[vsi_num] = 0xFFFF;
        }
    }
    
    /* Response ext_status = 0 */
    *(uint16_t *)&desc->params[2] = cpu_to_le16(0);
    
    ice_mp_adminq_complete(s, desc);
}

/* Get VSI (indirect 0x0212) */
static void ice_mp_adminq_get_vsi(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));
    
    if (addr) {
        /* Return VSI context - for simplicity, return zeros */
        uint8_t vsi_ctx[128] = {0};
        pci_dma_write(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
    }
    
    ice_mp_adminq_complete(s, desc);
}

/* Free VSI (0x0213) */
static void ice_mp_adminq_free_vsi(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    uint16_t vsi_num = le16_to_cpu(*(uint16_t *)&desc->params[0]);

    fprintf(stderr, "ice-mp: Free VSI (0x0213) vsi_num=%u\n", vsi_num);

    if (vsi_num < ICE_MP_MAX_VSI) {
        /* If this was a PF-type VSI, decrement the PF VSI count.
         * When all PF VSIs are freed (rebuild cycle), reset the counter
         * so the next Add VSI cycle assigns ports correctly.
         */
        if (s->vsi_to_vf[vsi_num] == 0xFFFF) {
            /* This was a PF-type VSI */
            if (s->pf_vsi_count > 0) {
                s->pf_vsi_count--;
            }
            if (s->pf_vsi_count == 0) {
                fprintf(stderr, "ice-mp: All PF VSIs freed, resetting pf_vsi_count\n");
            }
        }
        s->vsi_to_vf[vsi_num] = 0xFFFF;
        s->vsi_to_port[vsi_num] = 0;
    }

    /* Just acknowledge the free request */
    ice_mp_adminq_complete(s, desc);
}

/* Get Link Status (indirect 0x0607) */
static void ice_mp_adminq_get_link_status(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    /* Match Linux driver struct ice_aqc_get_link_status_data layout exactly */
    struct {
        uint8_t topo_media_conflict;
        uint8_t link_cfg_err;
        uint8_t link_info;            /* Byte 2: Bit 0 = ICE_AQ_LINK_UP */
        uint8_t an_info;
        uint8_t ext_info;
        uint8_t reserved2;
        uint16_t max_frame_size;
        uint8_t cfg;
        uint8_t power_desc;
        uint16_t link_speed;
        uint16_t reserved3;
        uint8_t reserved4[2];
        uint64_t phy_type_low;
        uint64_t phy_type_high;
    } __attribute__((packed)) link_status = {0};
    uint64_t addr;
    uint8_t port_num = desc->params[0] & 0xFF;  /* Port number from command */
    uint32_t port_status;
    bool link_up;
    uint8_t speed;
    
    /* Default to port 0 for multi-port */
    if (port_num >= s->num_ports) {
        port_num = 0;
    }
    
    port_status = s->port_status[port_num];
    link_up = (port_status & ICE_MP_PORT_LINK_UP) != 0;
    speed = (port_status >> ICE_MP_PORT_SPEED_SHIFT) & 0x7;
    
    fprintf(stderr, "ice-mp: Get Link Status lport_num=%u (clamped=%u): port_status=0x%x, link_up=%d, speed=%u\n",
            desc->params[0], port_num, port_status, link_up, speed);
    
    /* Populate link status from port state */
    link_status.topo_media_conflict = 0;
    link_status.link_cfg_err = 0;
    
    /* Bit 0: Link up, Bit 2: Link up via signal detection */
    link_status.link_info = link_up ? 0x01 : 0x00;
    
    /* Auto-negotiation - assume disabled for 100G */
    link_status.an_info = 0;
    link_status.ext_info = 0;
    link_status.reserved2 = 0;
    link_status.max_frame_size = cpu_to_le16(9728);  /* Standard jumbo frame */
    
    /* Full duplex when link is up */
    link_status.cfg = link_up ? 0x01 : 0x00;  /* Bit 0 = full duplex */
    link_status.power_desc = 0;
    
    /* Link speed encoding: 100G = 0x800 */
    if (link_up) {
        switch (speed) {
            case ICE_MP_SPEED_10G:
                link_status.link_speed = cpu_to_le16(0x80);   /* 10G */
                break;
            case ICE_MP_SPEED_25G:
                link_status.link_speed = cpu_to_le16(0x200);  /* 25G */
                break;
            case ICE_MP_SPEED_40G:
                link_status.link_speed = cpu_to_le16(0x400);  /* 40G */
                break;
            case ICE_MP_SPEED_100G:
            default:
                link_status.link_speed = cpu_to_le16(0x800);  /* 100G */
                break;
        }
    } else {
        link_status.link_speed = 0;
    }
    
    /* Set PHY type bits for 100GBASE-CR4 (bit 10 in low) */
    if (link_up) {
        link_status.phy_type_low = cpu_to_le64(1ULL << 10);
    } else {
        link_status.phy_type_low = 0;
    }
    link_status.phy_type_high = 0;
    
    /* Write response to DMA buffer */
    /* params layout: bytes 0-3=param0, 4-7=param1, 8-11=addr_high, 12-15=addr_low */
    uint32_t addr_high, addr_low;
    memcpy(&addr_high, &desc->params[8], 4);
    memcpy(&addr_low, &desc->params[12], 4);
    addr = ((uint64_t)le32_to_cpu(addr_high) << 32) | le32_to_cpu(addr_low);
    
    if (addr) {
        fprintf(stderr, "ice-mp: Writing link_status to DMA addr 0x%lx, size=%zu\n", addr, sizeof(link_status));
        fprintf(stderr, "ice-mp: link_status bytes: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                ((uint8_t*)&link_status)[0], ((uint8_t*)&link_status)[1],
                ((uint8_t*)&link_status)[2], ((uint8_t*)&link_status)[3],
                ((uint8_t*)&link_status)[4], ((uint8_t*)&link_status)[5],
                ((uint8_t*)&link_status)[6], ((uint8_t*)&link_status)[7]);
        pci_dma_write(PCI_DEVICE(s), addr, &link_status, sizeof(link_status));
        desc->datalen = cpu_to_le16(sizeof(link_status));
    }
    
    ice_mp_adminq_complete(s, desc);
}

/* Get Port Options (indirect 0x06EA) */
static void ice_mp_adminq_get_port_options(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct {
        uint8_t lport_num;
        uint8_t lport_num_valid;
        uint8_t port_options_count;
        uint8_t innermost_phy_index;
        uint8_t port_options;
        uint8_t pending_port_option_status;
        uint8_t rsvd[2];
        uint32_t addr_high;
        uint32_t addr_low;
    } __attribute__((packed)) *cmd = (void *)desc->params;
    
    struct {
        uint8_t pmd;
        uint8_t max_lane_speed;
        uint8_t global_scid[2];
        uint8_t phy_scid[2];
        uint8_t pf2port_cid[2];
    } __attribute__((packed)) port_opt_elems[4];
    
    uint64_t addr;
    int num_ports = s->num_ports > 0 ? s->num_ports : 4;
    int i;
    
    memset(port_opt_elems, 0, sizeof(port_opt_elems));
    
    /* Return port options for each port */
    for (i = 0; i < num_ports && i < 4; i++) {
        port_opt_elems[i].pmd = 1;       /* 1 PMD module per port */
        port_opt_elems[i].max_lane_speed = 5;  /* 25G max speed */
    }
    
    /* Fill descriptor response fields */
    cmd->port_options_count = num_ports;  /* Report actual number of ports */
    cmd->port_options = 0x80;     /* ICE_AQC_PORT_OPT_VALID | active_idx=0 */
    cmd->pending_port_option_status = 0;  /* No pending option */
    cmd->innermost_phy_index = 0;
    
    /* Get DMA buffer address from command params */
    addr = ice_mp_dma_addr(le32_to_cpu(cmd->addr_low),
                           le32_to_cpu(cmd->addr_high));
    
    if (addr) {
        /* Write port option elements to buffer */
        pci_dma_write(PCI_DEVICE(s), addr, port_opt_elems, 
                      num_ports * sizeof(port_opt_elems[0]));
        desc->datalen = cpu_to_le16(num_ports * sizeof(port_opt_elems[0]));
    }
    
    fprintf(stderr, "ice-mp: Get Port Options (0x06EA): lport=%d options_count=%d active_idx=0 num_ports=%d\n",
            cmd->lport_num, cmd->port_options_count, num_ports);
    
    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_list_caps(struct ICEMPState *s, struct ice_mp_aq_desc *desc,
                                    bool is_dev_caps)
{
    struct ice_mp_aqc_list_caps *cmd = (struct ice_mp_aqc_list_caps *)desc->params;
    struct ice_mp_aqc_list_caps_elem elems[8];
    uint64_t addr;
    int count = 0;

    memset(elems, 0, sizeof(elems));

    fprintf(stderr, "ice-mp: ice_mp_adminq_list_caps() called, is_dev_caps=%d\n", is_dev_caps);

    /* VALID_FUNCTIONS */
    elems[count].cap = cpu_to_le16(0x0005);
    elems[count].number = cpu_to_le32(0x1);
    count++;

    /* SRIOV */
    elems[count].cap = cpu_to_le16(0x0012);
    elems[count].number = cpu_to_le32(s->num_vfs ? 1 : 0);
    count++;

    /* VF */
    elems[count].cap = cpu_to_le16(0x0013);
    elems[count].number = cpu_to_le32(s->num_vfs);
    elems[count].logical_id = cpu_to_le32(0);
    count++;

    /* VSI */
    elems[count].cap = cpu_to_le16(0x0017);
    elems[count].number = cpu_to_le32(64);
    fprintf(stderr, "ice-mp: Setting VSI capability - cap=0x%04x, number=%u (index %d)\n",
            0x0017, 64, count);
    count++;

    /* RXQS */
    elems[count].cap = cpu_to_le16(0x0041);
    elems[count].number = cpu_to_le32(64);
    elems[count].phys_id = cpu_to_le32(0);
    count++;

    /* TXQS */
    elems[count].cap = cpu_to_le16(0x0042);
    elems[count].number = cpu_to_le32(64);
    elems[count].phys_id = cpu_to_le32(0);
    count++;

    /* MSIX */
    elems[count].cap = cpu_to_le16(0x0043);
    elems[count].number = cpu_to_le32(ICE_MP_MSIX_VECTORS);
    elems[count].phys_id = cpu_to_le32(0);
    count++;

    /* MAX_MTU */
    elems[count].cap = cpu_to_le16(0x0047);
    elems[count].number = cpu_to_le32(9728);
    count++;

    cmd->count = cpu_to_le32(count);
    addr = ice_mp_dma_addr(le32_to_cpu(cmd->addr_low),
                           le32_to_cpu(cmd->addr_high));
    pci_dma_write(&s->parent_obj, addr, elems, count * sizeof(elems[0]));

    desc->datalen = cpu_to_le16(count * sizeof(elems[0]));
        fprintf(stderr, "ice-mp: List %s caps count=%d addr=0x%lx msix=%u rxq=%u txq=%u\n",
            is_dev_caps ? "dev" : "func", count, (unsigned long)addr,
            ICE_MP_MSIX_VECTORS, 64u, 64u);

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_get_sw_cfg(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_get_sw_cfg *cmd = (struct ice_mp_aqc_get_sw_cfg *)desc->params;
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(cmd->addr_low),
                                    le32_to_cpu(cmd->addr_high));
    struct {
        uint16_t vsi_port_num;
        uint16_t swid;
        uint16_t pf_vf_num;
    } __attribute__((packed)) elem;

    memset(&elem, 0, sizeof(elem));
    /* type=phys port (0), lport=0 */
    elem.vsi_port_num = cpu_to_le16(0);
    elem.swid = cpu_to_le16(0);
    elem.pf_vf_num = cpu_to_le16(0);

    cmd->element = cpu_to_le16(0);
    cmd->num_elems = cpu_to_le16(1);
    pci_dma_write(&s->parent_obj, addr, &elem, sizeof(elem));
    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_query_sched(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    uint64_t addr;
    struct {
        struct {
            uint16_t phys_levels;
            uint16_t logical_levels;
            uint8_t flattening_bitmap;
            uint8_t max_device_cgds;
            uint8_t max_pf_cgds;
            uint8_t rsvd0;
            uint16_t rdma_qsets;
            uint8_t rsvd1[22];
        } __attribute__((packed)) sched_props;
        struct {
            uint8_t logical_layer;
            uint8_t chunk_size;
            uint16_t max_device_nodes;
            uint16_t max_pf_nodes;
            uint8_t rsvd0[4];
            uint16_t max_sibl_grp_sz;
            uint16_t max_cir_rl_profiles;
            uint16_t max_eir_rl_profiles;
            uint16_t max_srl_profiles;
            uint8_t rsvd1[14];
        } __attribute__((packed)) layer_props[9];
    } __attribute__((packed)) resp;

    memset(&resp, 0, sizeof(resp));
    
    /* E810 has 9-level scheduler tree:
     * 0: Root Port, 1: TC, 2: VSI, 3-7: Queue Groups, 8: Leaf/Queue
     */
    resp.sched_props.phys_levels = cpu_to_le16(9);
    resp.sched_props.logical_levels = cpu_to_le16(9);
    resp.sched_props.max_pf_cgds = 4;
    
    /* Layer properties for 9 levels
     * Driver reads: hw->max_children[i] = buf->layer_props[i + 1].max_sibl_grp_sz
     * So layer_props[1].max_sibl_grp_sz sets hw->max_children[0]
     * 
     * max_sibl_grp_sz for layer N means: max children for nodes at layer N-1
     * Layer 0 (Root): layer_props[1].max_sibl_grp_sz controls how many TCs root can have
     * Layer 1 (TC): layer_props[2].max_sibl_grp_sz controls how many VSIs each TC can have
     * etc.
     */
    for (int i = 0; i < 9; i++) {
        resp.layer_props[i].logical_layer = i;
        resp.layer_props[i].chunk_size = 1;
        resp.layer_props[i].max_device_nodes = cpu_to_le16(512);
        resp.layer_props[i].max_pf_nodes = cpu_to_le16(256);
        
        /* Set max_sibl_grp_sz
         * This defines the max number of siblings (children) at this layer
         */
        if (i == 0) {
            /* Layer 0 properties - not really used since root is special */
            resp.layer_props[i].max_sibl_grp_sz = cpu_to_le16(1);
        } else if (i == 1) {
            /* Layer 1 (TC layer): controls hw->max_children[0] = how many children root can have */
            resp.layer_props[i].max_sibl_grp_sz = cpu_to_le16(64);
        } else if (i == 2) {
            /* Layer 2 (VSI layer): controls hw->max_children[1] = how many children each TC can have */
            resp.layer_props[i].max_sibl_grp_sz = cpu_to_le16(256);
        } else if (i < 8) {
            /* Layers 3-7 (Queue Group layers): controls hw->max_children[2-6] */
            resp.layer_props[i].max_sibl_grp_sz = cpu_to_le16(8);
        } else {
            /* Layer 8 (Leaf layer): controls hw->max_children[7] 
             * This determines how many leaves layer 7 nodes can have */
            resp.layer_props[i].max_sibl_grp_sz = cpu_to_le16(16);
        }
        
        /* Rate limiting profiles - set reasonable defaults */
        resp.layer_props[i].max_cir_rl_profiles = cpu_to_le16(64);
        resp.layer_props[i].max_eir_rl_profiles = cpu_to_le16(64);
        resp.layer_props[i].max_srl_profiles = cpu_to_le16(64);
    }

    addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                           le32_to_cpu(*(uint32_t *)&desc->params[8]));
    pci_dma_write(&s->parent_obj, addr, &resp, sizeof(resp));
    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_manage_mac_read(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_manage_mac_read_resp resp;
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));

    memset(&resp, 0, sizeof(resp));
    resp.addr_type = 0;
    resp.mac_addr[0] = 0x52;
    resp.mac_addr[1] = 0x54;
    resp.mac_addr[2] = 0x00;
    resp.mac_addr[3] = 0x12;
    resp.mac_addr[4] = 0x34;
    resp.mac_addr[5] = 0x56;

    pci_dma_write(&s->parent_obj, addr, &resp, sizeof(resp));
    /* Set flags/num_addr in descriptor response */
    desc->params[0] = 0x10; /* ICE_AQC_MAN_MAC_LAN_ADDR_VALID */
    desc->params[1] = 0x00;
    desc->params[4] = 1; /* num_addr */
    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_get_ver(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_get_ver *ver = (struct ice_mp_aqc_get_ver *)desc->params;

    memset(ver, 0, sizeof(*ver));
    ver->fw_build = cpu_to_le32(1);
    ver->fw_major = 1;
    ver->fw_minor = 0;
    ver->fw_patch = 0;
    ver->api_major = 1;
    ver->api_minor = 5; /* E810 expected */
    ver->api_patch = 0;

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_req_res(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_req_res *cmd_resp = (struct ice_mp_aqc_req_res *)desc->params;
    uint16_t res_id = le16_to_cpu(cmd_resp->res_id);
    uint32_t timeout = le32_to_cpu(cmd_resp->timeout);

    /* Resource lock state machine implementation.
     * The driver expects:
     *   - res_id = 4 (ICE_RES_ID_GLBL_LOCK) for global config lock
     *   - status = 0 (ICE_RES_GLBL_SUCCESS) when lock is granted
     *   - status = 1 (ICE_RES_GLBL_IN_PROG) when lock is held by another
     * For our test device, we always grant the lock immediately.
     */
    if (res_id == ICE_MP_RES_ID_GLBL_LOCK) {
        if (!s->glbl_cfg_lock_held) {
            /* Lock available, grant it */
            s->glbl_cfg_lock_held = true;
            s->lock_timeout = timeout ? timeout : 1000;
            cmd_resp->status = cpu_to_le16(ICE_MP_RES_GLBL_SUCCESS);
            cmd_resp->timeout = cpu_to_le32(s->lock_timeout);
        } else {
            /* Lock already held - return in-progress status
             * (For single PF test device, this shouldn't happen)
             */
            cmd_resp->status = cpu_to_le16(1); /* ICE_RES_GLBL_IN_PROG */
            cmd_resp->timeout = cpu_to_le32(s->lock_timeout);
        }
    } else {
        /* Unknown resource ID */
        cmd_resp->status = cpu_to_le16(0);
        cmd_resp->timeout = cpu_to_le32(1000);
    }

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_rel_res(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_req_res *cmd_resp = (struct ice_mp_aqc_req_res *)desc->params;
    uint16_t res_id = le16_to_cpu(cmd_resp->res_id);

    /* Release resource lock.
     * Driver calls this after initialization to free the global config lock.
     */
    if (res_id == ICE_MP_RES_ID_GLBL_LOCK) {
        s->glbl_cfg_lock_held = false;
        s->lock_timeout = 0;
        cmd_resp->status = cpu_to_le16(ICE_MP_RES_GLBL_SUCCESS);
    }

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_alloc_res(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_alloc_free_res_cmd *cmd =
        (struct ice_mp_aqc_alloc_free_res_cmd *)desc->params;
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(cmd->addr_low),
                                    le32_to_cpu(cmd->addr_high));

    if (addr) {
        /* Read header (res_type + num_elems = 4 bytes) */
        struct {
            uint16_t res_type;
            uint16_t num_elems;
        } __attribute__((packed)) hdr;
        pci_dma_read(&s->parent_obj, addr, &hdr, sizeof(hdr));
        
        uint16_t num_elems = le16_to_cpu(hdr.num_elems);
        if (num_elems == 0) num_elems = 1;
        if (num_elems > 256) num_elems = 256;
        
        fprintf(stderr, "ice-mp: Alloc Res (0x0208): type=0x%x num_elems=%d\n",
                le16_to_cpu(hdr.res_type), num_elems);
        
        /* Read and fill all resource elements */
        uint16_t *elems = g_malloc0(num_elems * sizeof(uint16_t));
        pci_dma_read(&s->parent_obj, addr + sizeof(hdr), elems, num_elems * sizeof(uint16_t));
        
        int i;
        for (i = 0; i < num_elems; i++) {
            elems[i] = cpu_to_le16(s->res_counter++);
        }
        
        /* Write response back: header + all elements */
        pci_dma_write(&s->parent_obj, addr + sizeof(hdr), elems, num_elems * sizeof(uint16_t));
        
        g_free(elems);
    }

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_free_res(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_fw_logs_query(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct ice_mp_aqc_fw_log *cmd = (struct ice_mp_aqc_fw_log *)desc->params;
    struct ice_mp_aqc_fw_log_cfg_resp resp[ICE_MP_FWLOG_MODULES];
    uint64_t addr;
    int i;

    memset(resp, 0, sizeof(resp));
    for (i = 0; i < ICE_MP_FWLOG_MODULES; i++) {
        resp[i].module_identifier = cpu_to_le16(i);
        resp[i].log_level = 0;
    }

    cmd->cmd_flags = ICE_MP_FW_LOG_CONF_AQ_EN;
    cmd->ops.cfg.log_resolution = cpu_to_le16(1);
    cmd->ops.cfg.mdl_cnt = cpu_to_le16(ICE_MP_FWLOG_MODULES);

    addr = ice_mp_dma_addr(le32_to_cpu(cmd->addr_low),
                           le32_to_cpu(cmd->addr_high));
    pci_dma_write(&s->parent_obj, addr, resp, sizeof(resp));

    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_get_dflt_topo(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    /* Get Default Topology (0x0400) - Returns scheduler tree structure 
     * Structure matches ice_aqc_get_topo_elem from Linux driver
     */
    typedef struct __attribute__((packed)) {
        uint8_t elem_type;
        uint8_t valid_sections;
        uint8_t generic;
        uint8_t flags;
        uint8_t cir_bw[4];
        uint8_t eir_bw[4];
        uint16_t srl_id;
        uint16_t reserved;
    } ice_aqc_txsched_elem;
    
    typedef struct __attribute__((packed)) {
        uint32_t parent_teid;
        uint32_t node_teid;
        ice_aqc_txsched_elem data;
    } ice_aqc_txsched_elem_data;
    
    typedef struct __attribute__((packed)) {
        /* Header: ice_aqc_txsched_topo_grp_info_hdr */
        uint32_t parent_teid;      /* Group's parent TEID */
        uint16_t num_elems;
        uint16_t reserved;
        /* Array of elements */
        ice_aqc_txsched_elem_data generic[9];
    } ice_aqc_get_topo_elem;
    
    ice_aqc_get_topo_elem topo;
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));

    memset(&topo, 0, sizeof(topo));
    
    /* E810 typical 9-level tree:
     * Level 0: Root Port
     * Level 1: TC (Traffic Class)
     * Level 2: VSI (Virtual Station Interface)
     * Level 3-7: Queue Group layers (SE_GENERIC)
     * Level 8: Leaf (Queue)
     */
    
    /* Header - describes the group */
    topo.parent_teid = cpu_to_le32(0xFFFFFFFF);  /* Root's parent (invalid TEID) */
    topo.num_elems = cpu_to_le16(3);             /* 3 elements: Root + TC + ENTRY_POINT */
    topo.reserved = 0;
    
    /* Define element types */
    #define ICE_AQC_ELEM_TYPE_ROOT_PORT    0x1
    #define ICE_AQC_ELEM_TYPE_TC           0x2
    #define ICE_AQC_ELEM_TYPE_SE_GENERIC   0x3
    #define ICE_AQC_ELEM_TYPE_ENTRY_POINT  0x4
    #define ICE_AQC_ELEM_TYPE_LEAF         0x5
    
    /* Level 0: Root Port */
    topo.generic[0].parent_teid = cpu_to_le32(0xFFFFFFFF);  /* Root has invalid parent */
    topo.generic[0].node_teid = cpu_to_le32(0x16000000);    /* Root TEID */
    topo.generic[0].data.elem_type = ICE_AQC_ELEM_TYPE_ROOT_PORT;
    topo.generic[0].data.valid_sections = 0x1;  /* ICE_AQC_ELEM_VALID_GENERIC */
    
    /* Level 1: TC (Traffic Class) */
    topo.generic[1].parent_teid = cpu_to_le32(0x16000000);
    topo.generic[1].node_teid = cpu_to_le32(0x16000001);
    topo.generic[1].data.elem_type = ICE_AQC_ELEM_TYPE_TC;
    topo.generic[1].data.valid_sections = 0x1;
    
    /* Level 2: ENTRY_POINT (driver needs this to set sw_entry_point_layer, then removes it) */
    topo.generic[2].parent_teid = cpu_to_le32(0x16000001);
    topo.generic[2].node_teid = cpu_to_le32(0x16000002);
    topo.generic[2].data.elem_type = ICE_AQC_ELEM_TYPE_ENTRY_POINT;
    topo.generic[2].data.valid_sections = 0x1;
    
    fprintf(stderr, "ice-mp: 0x0400 Get Default Topo: root + TC + ENTRY_POINT (driver extracts sw_entry_point_layer=2, removes defaults, builds tree)\n");
    
    /* Write topology data to DMA buffer */
    pci_dma_write(&s->parent_obj, addr, &topo, sizeof(topo));
    
    /* Set num_branches in command response (params[1]) */
    desc->params[1] = 1;  /* num_branches = 1 */
    
    ice_mp_adminq_complete(s, desc);
}

/* Delete Scheduler Elements (0x040F) - Remove nodes from scheduler tree */
static void ice_mp_adminq_delete_sched_elems(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct {
        uint32_t parent_teid;
        uint16_t num_elems;
        uint16_t reserved;
    } __attribute__((packed)) *buf_header;
    
    uint32_t *teids;
    uint64_t addr;
    uint16_t num_elems, i;
    uint8_t *buf;
    
    /* Get DMA buffer address */
    addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                           le32_to_cpu(*(uint32_t *)&desc->params[8]));
    
    if (!addr) {
        ice_mp_adminq_complete(s, desc);
        return;
    }
    
    /* Allocate buffer */
    buf = g_malloc0(1024);
    
    /* Read header */
    pci_dma_read(&s->parent_obj, addr, buf, 8);
    buf_header = (void *)buf;
    num_elems = le16_to_cpu(buf_header->num_elems);
    
    if (num_elems > 64)
        num_elems = 64;
    
    /* Read TEIDs to delete */
    pci_dma_read(&s->parent_obj, addr + 8, buf + 8, num_elems * 4);
    teids = (uint32_t *)(buf + 8);
    
    fprintf(stderr, "ice-mp: 0x040F Delete Sched Elems: parent=0x%x num=%d\n",
            le32_to_cpu(buf_header->parent_teid), num_elems);
    
    for (i = 0; i < num_elems; i++) {
        uint32_t del_teid = le32_to_cpu(teids[i]);
        fprintf(stderr, "ice-mp:   Deleting TEID 0x%08x\n", del_teid);
        /* Clear from scheduler node table */
        uint32_t idx = del_teid - ICE_MP_SCHED_TEID_BASE;
        if (idx < ICE_MP_MAX_SCHED_NODES) {
            s->sched_nodes[idx].valid = false;
        }
    }
    
    /* Update response header */
    buf_header->num_elems = cpu_to_le16(num_elems);
    pci_dma_write(&s->parent_obj, addr, buf, 8);
    
    /* Set num_elem_resp in descriptor params[2-3] */
    *(uint16_t *)&desc->params[2] = cpu_to_le16(num_elems);
    
    g_free(buf);
    ice_mp_adminq_complete(s, desc);
}

/* Add Scheduler Elements (0x0401) - Add queue nodes to scheduler tree */
static void ice_mp_adminq_add_sched_elems(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    fprintf(stderr, "ice-mp: DEBUG in ice_mp_adminq_add_sched_elems handler\n");
    struct {
        uint32_t parent_teid;
        uint16_t num_elems;
        uint16_t reserved;
    } __attribute__((packed)) *buf_header;
    
    struct {
        uint32_t parent_teid;
        uint32_t node_teid;
        struct {
            uint8_t elem_type;
            uint8_t valid_sections;
            uint8_t generic;
            uint8_t flags;
            uint8_t cir_bw[4];
            uint8_t eir_bw[4];
            uint16_t srl_id;
            uint16_t reserved;
        } __attribute__((packed)) data;
    } __attribute__((packed)) *elems;
    
    uint64_t addr;
    uint16_t num_elems, i;
    uint8_t *buf;
    uint32_t buf_size;
    
    /* Get DMA buffer address */
    addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                           le32_to_cpu(*(uint32_t *)&desc->params[8]));
    
    if (!addr) {
        ice_mp_adminq_complete(s, desc);
        return;
    }
    
    /* Allocate buffer to read request and build response */
    buf_size = 8 + (24 * 16);  /* header + max 16 elements (each elem_data = 24 bytes) */
    buf = g_malloc0(buf_size);
    
    /* Read header to get number of elements */
    pci_dma_read(&s->parent_obj, addr, buf, 8);
    buf_header = (void *)buf;
    num_elems = le16_to_cpu(buf_header->num_elems);
    
    fprintf(stderr, "ice-mp: 0x0401 request header: parent_teid=0x%x num_elems=%d\n",
            le32_to_cpu(buf_header->parent_teid), num_elems);
    
    if (num_elems > 16)
        num_elems = 16;  /* Safety limit */
    
    /* Read element data */
    pci_dma_read(&s->parent_obj, addr + 8, buf + 8, num_elems * 24);
    elems = (void *)(buf + 8);
    
    /* Generate TEIDs for each element and track parentage */
    for (i = 0; i < num_elems; i++) {
        uint32_t new_teid = s->next_sched_teid++;
        uint32_t parent = le32_to_cpu(elems[i].parent_teid);
        elems[i].node_teid = cpu_to_le32(new_teid);

        /* Track this node in the scheduler table */
        uint32_t idx = new_teid - ICE_MP_SCHED_TEID_BASE;
        if (idx < ICE_MP_MAX_SCHED_NODES) {
            s->sched_nodes[idx].valid = true;
            s->sched_nodes[idx].parent_teid = parent;
            s->sched_nodes[idx].elem_type = elems[i].data.elem_type;
        }

        fprintf(stderr, "ice-mp: Add Sched Elem: elem[%d] TEID=0x%x type=%d parent=0x%x\n",
                i, new_teid, elems[i].data.elem_type, parent);
        fprintf(stderr, "ice-mp:   elem structure: parent_teid=%08x node_teid=%08x elem_type=%d\n",
                le32_to_cpu(elems[i].parent_teid),
                le32_to_cpu(elems[i].node_teid),
                elems[i].data.elem_type);
    }
    
    /* Update header for response - set num_elems in response */
    buf_header->num_elems = cpu_to_le16(num_elems);
    
    /* Write response back to buffer */
    pci_dma_write(&s->parent_obj, addr, buf, 8 + (num_elems * 24));
    desc->datalen = cpu_to_le16(8 + (num_elems * 24));
    
    fprintf(stderr, "ice-mp: 0x0401 wrote %d bytes to DMA buffer at 0x%lx\n",
            8 + (num_elems * 32), addr);
    
    /* Set num_elem_resp in the descriptor response
     * This is at offset 2 in the sched_elem_cmd structure (params[2-3])
     */
    fprintf(stderr, "ice-mp: 0x0401 BEFORE setting params: params[2]=0x%02x params[3]=0x%02x\n",
            desc->params[2], desc->params[3]);
    
    *(uint16_t *)&desc->params[2] = cpu_to_le16(num_elems);
    
    fprintf(stderr, "ice-mp: 0x0401 AFTER setting params: params[2]=0x%02x params[3]=0x%02x (set to %d)\n",
            desc->params[2], desc->params[3], num_elems);
    
    fprintf(stderr, "ice-mp: 0x0401 response: num_elems_req=%d->resp=%d, datalen=%d, DMA addr=0x%lx\n",
            buf_header ? le16_to_cpu(buf_header->num_elems) : 0,
            num_elems, le16_to_cpu(desc->datalen), addr);
    
    /* Explicitly set retval to 0 for success */
    desc->retval = cpu_to_le16(0);
    
    fprintf(stderr, "ice-mp: 0x0401 BEFORE complete: params[2]=0x%02x params[3]=0x%02x retval=0x%04x\n",
            desc->params[2], desc->params[3], le16_to_cpu(desc->retval));
    
    g_free(buf);
    ice_mp_adminq_complete(s, desc);
    
    /* Debug: Check desc state when handler returns */
    fprintf(stderr, "ice-mp: 0x0401 handler RETURNING: params[2]=0x%02x params[3]=0x%02x retval=0x%04x flags=0x%04x\n",
            desc->params[2], desc->params[3], le16_to_cpu(desc->retval), le16_to_cpu(desc->flags));
}

/* Set Port Parameters (0x0203) - Configure port settings */
static void ice_mp_adminq_set_port_params(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    /* Set Port Parameters command (direct 0x0203)
     * Configures port-level settings like MAC speed, VLAN mode, etc.
     * All parameters are in the descriptor params field.
     */
    uint16_t cmd_flags = le16_to_cpu(*(uint16_t *)&desc->params[0]);
    uint16_t swid = le16_to_cpu(*(uint16_t *)&desc->params[2]);
    
    fprintf(stderr, "ice-mp: Set Port Parameters (0x0203): flags=0x%x swid=0x%x\n",
            cmd_flags, swid);
    
    desc->retval = cpu_to_le16(0);
    ice_mp_adminq_complete(s, desc);
}

/* Add VSI (0x0210) - Add Virtual Station Interface */
static void ice_mp_adminq_add_vsi(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    /* Add VSI command indirect:
     * Input: 
     *   - desc->params[0:1]: vsi_num (requested from driver, with ICE_AQ_VSI_IS_VALID bit)
     *   - desc->params[4:5]: vf_id  
     *   - desc->params[8-15]: DMA addr to VSI properties buffer
     * Output: VSI info in descriptor params (ice_aqc_add_update_free_vsi_resp)
     */
    
    /* Read the requested VSI number from request
     * Driver sets: cmd->vsi_num = cpu_to_le16(vsi_ctx->vsi_num | ICE_AQ_VSI_IS_VALID)
     * If ICE_AQ_VSI_IS_VALID is set, the driver requests a specific VSI number.
     * Otherwise, the device should allocate one from the pool.
     */
    const uint16_t ICE_AQ_VSI_IS_VALID = 0x8000;
    const uint16_t ICE_AQ_VSI_NUM_M = 0x03FF;
    uint16_t raw_vsi_num = le16_to_cpu(*(uint16_t *)&desc->params[0]);
    bool vsi_num_valid = (raw_vsi_num & ICE_AQ_VSI_IS_VALID) != 0;
    uint16_t requested_vsi_num = raw_vsi_num & ICE_AQ_VSI_NUM_M;
    uint8_t vf_id = desc->params[4];
    /* vsi_flags is at params[6:7] per ice_aqc_add_get_update_free_vsi struct:
     *   params[0:1] = vsi_num, params[2:3] = cmd_flags,
     *   params[4] = vf_id (u8), params[5] = reserved,
     *   params[6:7] = vsi_flags, params[8:15] = DMA addr
     */
    uint16_t vsi_flags = le16_to_cpu(*(uint16_t *)&desc->params[6]);
    uint8_t vsi_type = vsi_flags & 0x3;  /* ICE_AQ_VSI_TYPE_M */
    
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));
    
        fprintf(stderr, "ice-mp: Add VSI (0x0210) called, raw_vsi_num=0x%04x, requested_vsi_num=%u, valid=%u, DMA addr=0x%lx, vf_id=%u, vsi_flags=0x%04x, vsi_type=%u\n",
            raw_vsi_num, requested_vsi_num, vsi_num_valid ? 1 : 0, addr, vf_id, vsi_flags, vsi_type);
    
    uint8_t vsi_ctx[128] = {0};  /* ice_aqc_vsi_props structure */
    
    if (addr) {
        /* Read VSI context from buffer */
        pci_dma_read(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
        
        /* Write back the VSI context - most fields are set by driver */
        pci_dma_write(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
        
        fprintf(stderr, "ice-mp: Add VSI - wrote back VSI context (%zu bytes)\n",
                sizeof(vsi_ctx));
    }
    
    /* IMPORTANT: If the driver requests a specific VSI number, echo it back.
     * Otherwise, allocate one from the pool and return it to the driver.
     */
    uint16_t vsi_num;
    if (vsi_num_valid) {
        vsi_num = requested_vsi_num;
    } else {
        if (s->next_vsi_num == 0) {
            s->next_vsi_num = 1;
        }
        vsi_num = s->next_vsi_num++;
        if (s->next_vsi_num >= ICE_MP_MAX_VSI) {
            s->next_vsi_num = 1;
        }
    }
    
    /* Track VSI to port mapping based on VSI type.
     * ICE_AQ_VSI_TYPE_VF = 0x0, ICE_AQ_VSI_TYPE_PF = 0x2
     * For PF-type VSIs, the multi-port driver creates them sequentially
     * for ports 0, 1, 2, 3. The first PF VSI is the default (port 0),
     * then subsequent ones are for multi-port.
     */
    if (vsi_num < ICE_MP_MAX_VSI) {
        if (vsi_type == 0x2) {  /* ICE_AQ_VSI_TYPE_PF */
            s->vsi_to_vf[vsi_num] = 0xFFFF;
            /* Reset PF VSI counter when transitioning from VF to PF creation.
             * The driver creates PF VSIs in batches (one default + one per port).
             * Between batches, VF VSIs are created. Resetting on the VF→PF
             * boundary ensures each batch starts fresh with correct port mapping.
             */
            if (!s->last_vsi_was_pf) {
                fprintf(stderr, "ice-mp: PF VSI batch start - resetting pf_vsi_count from %u to 0\n",
                        s->pf_vsi_count);
                s->pf_vsi_count = 0;
            }
            /* PF-type VSI: assign port based on creation order within batch.
             * PF VSI #0 = default (port 0)
             * PF VSI #1 = multi-port port 0 (port 0)
             * PF VSI #2 = multi-port port 1 (port 1)
             * etc.
             */
            uint8_t port;
            if (s->pf_vsi_count == 0) {
                port = 0;  /* Default PF VSI */
            } else {
                port = (s->pf_vsi_count - 1) % s->num_ports;
            }
            s->vsi_to_port[vsi_num] = port;
            s->pf_vsi_count++;
            s->last_vsi_was_pf = true;
            fprintf(stderr, "ice-mp: VSI %u (PF type) mapped to port %u (PF VSI #%u)\n",
                    vsi_num, port, s->pf_vsi_count - 1);
        } else {
            /* VF-type VSI or other types */
            s->vsi_to_vf[vsi_num] = vf_id;
            s->vsi_to_port[vsi_num] = 0;  /* VF VSIs default to port 0 */
            s->last_vsi_was_pf = false;
            fprintf(stderr, "ice-mp: VSI %u (type=%u, vf_id=%u) mapped to port 0\n",
                    vsi_num, vsi_type, vf_id);
        }
    }
    
    uint16_t vsi_used = vsi_num + 1;  /* Best-effort count for reporting */
    uint16_t vsi_free = (vsi_used < 255) ? (uint16_t)(255 - vsi_used) : 0;

    /* Fill response in descriptor params (ice_aqc_add_update_free_vsi_resp) */
    *(uint16_t *)&desc->params[0] = cpu_to_le16(vsi_num);
    *(uint16_t *)&desc->params[2] = cpu_to_le16(0);    /* ext_status = 0 */
    *(uint16_t *)&desc->params[4] = cpu_to_le16(vsi_used);
    *(uint16_t *)&desc->params[6] = cpu_to_le16(vsi_free);
    /* params[8-15] already contain addr_high/addr_low from request */
    
    /* Set retval to 0 for success */
    desc->retval = cpu_to_le16(0);
    
    fprintf(stderr, "ice-mp: Add VSI response: vsi_num=%u (echoed from request), vsi_used=%u, vsi_free=%u\n",
            vsi_num, vsi_used, vsi_free);
    
    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_query_sched_elems(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    /* Query Scheduler Elements (0x0404) - Returns detailed node information
     * Input: Array with node_teid set
     * Output: Full ice_aqc_txsched_elem_data with all details
     */
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));
    struct {
        uint32_t parent_teid;
        uint32_t node_teid;
        struct {
            uint8_t elem_type;
            uint8_t valid_sections;
            uint8_t generic;
            uint8_t flags;
            uint8_t cir_bw[4];
            uint8_t eir_bw[4];
            uint16_t srl_id;
            uint16_t reserved;
        } __attribute__((packed)) data;
    } __attribute__((packed)) elem;
    uint32_t node_teid;

    /* Read the input to get the node TEID being queried */
    pci_dma_read(&s->parent_obj, addr, &elem, sizeof(elem));
    node_teid = le32_to_cpu(elem.node_teid);

    /* Based on the TEID, return appropriate node information
     * TEIDs match those from Get Default Topology:
     * 0x16000000 = Root Port
     * 0x16000001 = TC
     * 0x16000002-0x16000008 = SE_GENERIC/LEAF nodes
     */
    memset(&elem, 0, sizeof(elem));
    elem.node_teid = cpu_to_le32(node_teid);

    /* Element types already defined in ice_mp_adminq_get_dflt_topo */

    if (node_teid == 0x16000000) {
        /* Root Port */
        elem.parent_teid = cpu_to_le32(0xFFFFFFFF);
        elem.data.elem_type = ICE_AQC_ELEM_TYPE_ROOT_PORT;
    } else if (node_teid == 0x16000001) {
        /* TC */
        elem.parent_teid = cpu_to_le32(0x16000000);
        elem.data.elem_type = ICE_AQC_ELEM_TYPE_TC;
    } else if (node_teid == 0x16000002) {
        /* ENTRY_POINT (VSI) */
        elem.parent_teid = cpu_to_le32(0x16000001);
        elem.data.elem_type = ICE_AQC_ELEM_TYPE_ENTRY_POINT;
    } else {
        /* Look up dynamically-created node in the scheduler table */
        uint32_t idx = node_teid - ICE_MP_SCHED_TEID_BASE;
        if (idx < ICE_MP_MAX_SCHED_NODES && s->sched_nodes[idx].valid) {
            elem.parent_teid = cpu_to_le32(s->sched_nodes[idx].parent_teid);
            elem.data.elem_type = s->sched_nodes[idx].elem_type;
            fprintf(stderr, "ice-mp: 0x0404 query TEID=0x%x -> parent=0x%x type=%d (from table)\n",
                    node_teid, s->sched_nodes[idx].parent_teid, s->sched_nodes[idx].elem_type);
        } else {
            /* Unknown TEID - return as SE_GENERIC with root parent */
            fprintf(stderr, "ice-mp: 0x0404 query TEID=0x%x -> UNKNOWN, returning root parent\n",
                    node_teid);
            elem.parent_teid = cpu_to_le32(0x16000002);
            elem.data.elem_type = ICE_AQC_ELEM_TYPE_SE_GENERIC;
        }
    }

    elem.data.valid_sections = 0x1;  /* ICE_AQC_ELEM_VALID_GENERIC */

    /* Write response back */
    pci_dma_write(&s->parent_obj, addr, &elem, sizeof(elem));
    
    /* Set num_elem_resp to 1 in the descriptor (params[2-3]) */
    *(uint16_t *)&desc->params[2] = cpu_to_le16(1);
    
    ice_mp_adminq_complete(s, desc);
}

/* DDP Package Download (0x0C40) - just return success */
static void ice_mp_adminq_download_pkg(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    /* The driver sends DDP buffers via this command.
     * We just return success - the "firmware" accepts everything.
     * Response: error_offset = 0, error_info = 0 (already zeroed in desc)
     */
    fprintf(stderr, "ice-mp: Download Package (0x0C40) - returning success\n");
    ice_mp_adminq_complete(s, desc);
}

/* Upload Package Section (0x0C41) - return a zeroed buffer */
static void ice_mp_adminq_upload_section(struct ICEMPState *s,
                                         struct ice_mp_aq_desc *desc)
{
    uint64_t addr;
    uint16_t data_len = le16_to_cpu(desc->datalen);
    uint8_t *buf;

    if (data_len == 0 || data_len > ICE_MP_PKG_BUF_SIZE) {
        data_len = ICE_MP_PKG_BUF_SIZE;
    }

    buf = g_malloc0(data_len);

    addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                           le32_to_cpu(*(uint32_t *)&desc->params[8]));

    fprintf(stderr, "ice-mp: Upload Section (0x0C41) DMA addr=0x%lx len=%u\n",
            (unsigned long)addr, data_len);

    pci_dma_write(&s->parent_obj, addr, buf, data_len);
    g_free(buf);

    desc->datalen = cpu_to_le16(data_len);
    ice_mp_adminq_complete(s, desc);
}

/* Get Package Info List (0x0C43) - return one active package entry */
static void ice_mp_adminq_get_pkg_info_list(struct ICEMPState *s,
                                            struct ice_mp_aq_desc *desc)
{
    uint64_t addr;
    /* Response: count(le32) + array of pkg_info entries
     * Each entry: ver(4) + name(28) + track_id(le32) + flags(4) = 40 bytes
     * struct ice_aqc_get_pkg_info {
     *   struct ice_pkg_ver ver;   // 4 bytes
     *   char name[28];
     *   __le32 track_id;
     *   u8 is_in_nvm;
     *   u8 is_active;
     *   u8 is_active_at_boot;
     *   u8 is_modified;
     * };
     * struct ice_aqc_get_pkg_info_resp {
     *   __le32 count;
     *   struct ice_aqc_get_pkg_info pkg_info[];
     * };
     */
    struct {
        uint32_t count;
        struct {
            uint8_t ver_major;
            uint8_t ver_minor;
            uint8_t ver_update;
            uint8_t ver_draft;
            char name[28];
            uint32_t track_id;
            uint8_t is_in_nvm;
            uint8_t is_active;
            uint8_t is_active_at_boot;
            uint8_t is_modified;
        } __attribute__((packed)) pkg_info[1];
    } __attribute__((packed)) resp;

    memset(&resp, 0, sizeof(resp));
    resp.count = cpu_to_le32(1);

    /* Package version must match what the DDP binary reports:
     * ver = {1, 3, 0, 0}, name = "ICE OS Default Package"
     * This ensures ice_get_ddp_pkg_state() returns ICE_DDP_PKG_SUCCESS
     * (hw->pkg_ver == hw->active_pkg_ver && hw->pkg_name == hw->active_pkg_name)
     * And ice_chk_pkg_compat() NVM version check passes
     * (seg_format_ver.major == nvm_ver.major && seg_format_ver.minor <= nvm_ver.minor)
     */
    resp.pkg_info[0].ver_major = 1;
    resp.pkg_info[0].ver_minor = 3;
    resp.pkg_info[0].ver_update = 0;
    resp.pkg_info[0].ver_draft = 0;
    memcpy(resp.pkg_info[0].name, "ICE OS Default Package", 22);
    resp.pkg_info[0].track_id = cpu_to_le32(0);
    resp.pkg_info[0].is_in_nvm = 1;
    resp.pkg_info[0].is_active = 1;
    resp.pkg_info[0].is_active_at_boot = 1;
    resp.pkg_info[0].is_modified = 0;

    addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                           le32_to_cpu(*(uint32_t *)&desc->params[8]));

    fprintf(stderr, "ice-mp: 0x0C43 DMA addr=0x%lx resp_size=%zu count=%u\n",
            (unsigned long)addr, sizeof(resp), le32_to_cpu(resp.count));
    fprintf(stderr, "ice-mp: 0x0C43 ver=%d.%d.%d.%d name='%.22s' active=%d in_nvm=%d\n",
            resp.pkg_info[0].ver_major, resp.pkg_info[0].ver_minor,
            resp.pkg_info[0].ver_update, resp.pkg_info[0].ver_draft,
            resp.pkg_info[0].name, resp.pkg_info[0].is_active,
            resp.pkg_info[0].is_in_nvm);

    pci_dma_write(&s->parent_obj, addr, &resp, sizeof(resp));

    /* Set datalen to actual response size */
    desc->datalen = cpu_to_le16(sizeof(resp));

    fprintf(stderr, "ice-mp: Get Package Info List (0x0C43) - returning 1 active package\n");
    ice_mp_adminq_complete(s, desc);
}

static void ice_mp_adminq_process(struct ICEMPState *s)
{
    uint32_t count = s->atq_len & 0x3FF;
    uint64_t base = ice_mp_dma_addr(s->atq_bal, s->atq_bah);

    if (!count)
        return;

    /* Process only ONE command per call to allow ATQH updates to be visible */
    if (s->atq_head != s->atq_tail) {
        uint64_t addr = base + (s->atq_head * sizeof(struct ice_mp_aq_desc));
        struct ice_mp_aq_desc desc;

        pci_dma_read(&s->parent_obj, addr, &desc, sizeof(desc));

        fprintf(stderr, "ice-mp: AQ opcode 0x%04x (head=%d tail=%d)\n", 
                le16_to_cpu(desc.opcode), s->atq_head, s->atq_tail);
        
        if (le16_to_cpu(desc.opcode) == ICE_MP_AQC_OPC_GET_LINK_STATUS) {
            fprintf(stderr, "ice-mp: Get Link Status descriptor params: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    desc.params[0], desc.params[1], desc.params[2], desc.params[3],
                    desc.params[4], desc.params[5], desc.params[6], desc.params[7]);
        }

        switch (le16_to_cpu(desc.opcode)) {
        case ICE_MP_AQC_OPC_GET_VER:
            ice_mp_adminq_get_ver(s, &desc);
            break;
        case ICE_MP_AQC_OPC_DRIVER_VER:
        case ICE_MP_AQC_OPC_Q_SHUTDOWN:
        case ICE_MP_AQC_OPC_CLEAR_PXE_MODE:
        case ICE_MP_AQC_OPC_CLEAR_PF_CFG:
        case ICE_MP_AQC_OPC_SET_MAC_CFG:
        case ICE_MP_AQC_OPC_FW_LOGS_CONFIG:
        case ICE_MP_AQC_OPC_FW_LOGS_REGISTER:
            ice_mp_adminq_complete(s, &desc);
            break;
        case ICE_MP_AQC_OPC_REQ_RES:
            ice_mp_adminq_req_res(s, &desc);
            break;
        case ICE_MP_AQC_OPC_REL_RES:
            ice_mp_adminq_rel_res(s, &desc);
            break;
        case ICE_MP_AQC_OPC_LIST_FUNC_CAPS:
            ice_mp_adminq_list_caps(s, &desc, false);
            break;
        case ICE_MP_AQC_OPC_LIST_DEV_CAPS:
            ice_mp_adminq_list_caps(s, &desc, true);
            break;
        case ICE_MP_AQC_OPC_GET_SW_CFG:
            ice_mp_adminq_get_sw_cfg(s, &desc);
            break;
        case 0x0203:  /* Set Port Parameters */
            ice_mp_adminq_set_port_params(s, &desc);
            break;
        case 0x0210:  /* Add VSI */
            ice_mp_adminq_add_vsi(s, &desc);
            break;
        case ICE_MP_AQC_OPC_ALLOC_RES:
            ice_mp_adminq_alloc_res(s, &desc);
            break;
        case ICE_MP_AQC_OPC_FREE_RES:
            ice_mp_adminq_free_res(s, &desc);
            break;
        case ICE_MP_AQC_OPC_GET_PHY_CAPS:
            ice_mp_adminq_get_phy_caps(s, &desc);
            break;
        case 0x040F:  /* Delete Scheduler Elements */
            ice_mp_adminq_delete_sched_elems(s, &desc);
            break;
        case ICE_MP_AQC_OPC_GET_LINK_STATUS:  /* 0x0607 also handled here for PHY status */
            ice_mp_adminq_get_link_status(s, &desc);
            break;
        case ICE_MP_AQC_OPC_GET_PORT_OPTIONS:
            ice_mp_adminq_get_port_options(s, &desc);
            break;
        case ICE_MP_AQC_OPC_NVM_READ:
            ice_mp_adminq_nvm_read(s, &desc);
            break;
        case ICE_MP_AQC_OPC_QUERY_TXSCHED:
            ice_mp_adminq_query_sched(s, &desc);
            break;
        case 0x0400:  /* Get Default Topology */
            ice_mp_adminq_get_dflt_topo(s, &desc);
            break;
        case 0x0404:  /* Query Scheduler Elements */
            ice_mp_adminq_query_sched_elems(s, &desc);
            break;
        case 0x0401:  /* Add Scheduler Elements */
            ice_mp_adminq_add_sched_elems(s, &desc);
            break;
        case 0x0403:  /* Configure Scheduler Elements - just return success */
            ice_mp_adminq_complete(s, &desc);
            break;
        case ICE_MP_AQC_OPC_MANAGE_MAC_READ:
            ice_mp_adminq_manage_mac_read(s, &desc);
            break;
        case ICE_MP_AQC_OPC_FW_LOGS_QUERY:
            ice_mp_adminq_fw_logs_query(s, &desc);
            break;
        /* SET PHY Config - handle link up/down */
        case 0x0601:  /* Set PHY Config */
            ice_mp_adminq_set_phy_cfg(s, &desc);
            break;
        /* VSI management for VF creation */
        case 0x0211:  /* Update VSI */
            ice_mp_adminq_update_vsi(s, &desc);
            break;
        case 0x0212:  /* Get VSI */
            ice_mp_adminq_get_vsi(s, &desc);
            break;
        case 0x0213:  /* Free VSI */
            ice_mp_adminq_free_vsi(s, &desc);
            break;
        /* Additional opcodes needed for full driver initialization */
        case 0x0605:  /* Restart AN */
        case 0x0613:  /* Set Event Mask */
        case 0x0302:  /* Query PFC Mode */
        case 0x0303:  /* Set PFC Mode */
        case 0x020C:  /* Set VLAN Mode */
        case 0x020D:  /* Get VLAN Mode*/
        case 0x0A05:  /* Stop LLDP */
        case 0x0A06:  /* Start LLDP */
        case 0x0A09:  /* Stop/Start LLDP Agent */
            fprintf(stderr, "ice-mp: opcode 0x%04x - returning success\n",
                    le16_to_cpu(desc.opcode));
            ice_mp_adminq_complete(s, &desc);
            break;
        case 0x02A0:  /* Add Switch Rules */
        case 0x02A1:  /* Update Switch Rules */
        case 0x02A2:  /* Remove Switch Rules */
            ice_mp_adminq_sw_rules(s, &desc, le16_to_cpu(desc.opcode));
            break;
        case 0x0C30:  /* Add Tx LAN Queues */
            ice_mp_adminq_add_tx_queues(s, &desc);
            break;
        case 0x0C31: {  /* Disable Tx LAN Queues */
            fprintf(stderr, "ice-mp: Disable Tx LAN Queues (0x0C31)\n");
            /* Parse the disable command buffer to find which queues to disable.
             * The buffer format is ice_aqc_dis_txq_item entries.
             * For simplicity, disable all queues referenced by the command.
             */
            uint64_t dis_addr = ice_mp_dma_addr(
                le32_to_cpu(*(uint32_t *)&desc.params[12]),
                le32_to_cpu(*(uint32_t *)&desc.params[8]));
            uint16_t dis_buf_len = le16_to_cpu(desc.datalen);
            if (dis_addr && dis_buf_len >= 8) {
                uint8_t *dis_buf = g_malloc(dis_buf_len);
                pci_dma_read(&s->parent_obj, dis_addr, dis_buf, dis_buf_len);
                /* Parse: first 4 bytes = parent_teid, byte 4 = num_qs */
                uint8_t num_qs = dis_buf[4];
                for (int dq = 0; dq < num_qs && (8 + dq * 8 + 8) <= dis_buf_len; dq++) {
                    /* Each ice_aqc_dis_txq_item: teid(4) + txq_id(2) + q_handle(2) = 8 bytes */
                    uint16_t dis_qid;
                    memcpy(&dis_qid, dis_buf + 8 + dq * 8 + 4, 2);  /* txq_id at offset 4 within entry */
                    dis_qid = le16_to_cpu(dis_qid);
                    if (dis_qid < ICE_MP_MAX_TX_QUEUES) {
                        s->txq[dis_qid].enabled = false;
                        s->txq[dis_qid].configured = false;
                        s->txq[dis_qid].head = 0;
                        s->txq[dis_qid].tail = 0;
                        fprintf(stderr, "ice-mp: Disabled TX queue %u\n", dis_qid);
                    }
                }
                g_free(dis_buf);
            }
            ice_mp_adminq_complete(s, &desc);
            break;
        }
        case 0x0C32:  /* Move/Reconfigure Tx Queues */
            fprintf(stderr, "ice-mp: Tx Queue opcode 0x%04x - returning success\n",
                    le16_to_cpu(desc.opcode));
            ice_mp_adminq_complete(s, &desc);
            break;
        case 0x0290:  /* Add Recipe */
        case 0x0291:  /* Set Recipe to Profile */
        case 0x0292:  /* Get Recipe */
        case 0x0293:  /* Get Recipe to Profile */
            fprintf(stderr, "ice-mp: Recipe opcode 0x%04x - returning success\n",
                    le16_to_cpu(desc.opcode));
            ice_mp_adminq_complete(s, &desc);
            break;
        case 0x0B02:  /* Set RSS Key */
        case 0x0B03:  /* Set RSS LUT */
        case 0x0B04:  /* Get RSS Key */
        case 0x0B05:  /* Get RSS LUT */
            fprintf(stderr, "ice-mp: RSS opcode 0x%04x - returning success\n",
                    le16_to_cpu(desc.opcode));
            ice_mp_adminq_complete(s, &desc);
            break;
        /* DDP (Dynamic Device Personalization) package opcodes */
        case ICE_MP_AQC_OPC_DOWNLOAD_PKG:
            ice_mp_adminq_download_pkg(s, &desc);
            break;
        case ICE_MP_AQC_OPC_UPLOAD_SECTION:
            ice_mp_adminq_upload_section(s, &desc);
            break;
        case ICE_MP_AQC_OPC_GET_PKG_INFO_LIST:
            ice_mp_adminq_get_pkg_info_list(s, &desc);
            break;
        case 0x0C42:  /* Update Package - return success */
            fprintf(stderr, "ice-mp: Update Package (0x0C42) - returning success\n");
            ice_mp_adminq_complete(s, &desc);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "ice-mp: unhandled AQ opcode 0x%04x\n",
                          le16_to_cpu(desc.opcode));
            ice_mp_adminq_complete(s, &desc);
            break;
        }

        /* Debug: For 0x0401, check desc right before DMA write */
        if (le16_to_cpu(desc.opcode) == 0x0401) {
            /* Hex dump of entire descriptor */
            fprintf(stderr, "ice-mp: 0x0401 DESC HEX: ");
            uint8_t *desc_bytes = (uint8_t *)&desc;
            for (int i = 0; i < sizeof(desc); i++) {
                fprintf(stderr, "%02x ", desc_bytes[i]);
            }
            fprintf(stderr, "\n");
            
            /* Save values to local variables to rule out structure corruption during fprintf */
            uint16_t saved_retval = le16_to_cpu(desc.retval);
            uint16_t saved_flags = le16_to_cpu(desc.flags);
            uint8_t saved_p2 = desc.params[2];
            uint8_t saved_p3 = desc.params[3];
            
            fprintf(stderr, "ice-mp: 0x0401 FIELD OFFSETS: flags@%td retval@%td p2@%td p3@%td\n",
                    (char*)&desc.flags - (char*)&desc,
                    (char*)&desc.retval - (char*)&desc,
                    (char*)&desc.params[2] - (char*)&desc,
                    (char*)&desc.params[3] - (char*)&desc);
            fprintf(stderr, "ice-mp: 0x0401 SAVED VALUES: p2=0x%02x p3=0x%02x rv=0x%04x fl=0x%04x\n",
                    saved_p2, saved_p3, saved_retval, saved_flags);
            fprintf(stderr, "ice-mp: 0x0401 DESC DIRECT: p2=0x%02x p3=0x%02x rv=0x%04x fl=0x%04x\n",
                    desc.params[2], desc.params[3], le16_to_cpu(desc.retval), le16_to_cpu(desc.flags));
        }
        
        pci_dma_write(&s->parent_obj, addr, &desc, sizeof(desc));
        
        /* Increment head pointer to signal completion */
        s->atq_head = (s->atq_head + 1) % count;
        fprintf(stderr, "ice-mp: Command 0x%04x complete, updated ATQH to %d\n", 
                le16_to_cpu(desc.opcode), s->atq_head);
        
        if (le16_to_cpu(desc.opcode) == 0x0401) {
            fprintf(stderr, "ice-mp: 0x0401 AFTER DMA - DESC HEX: ");
            uint8_t *bytes_after = (uint8_t *)&desc;
            for (int j = 0; j < sizeof(desc); j++) {
                fprintf(stderr, "%02x ", bytes_after[j]);
            }
            fprintf(stderr, "\n");
            fprintf(stderr, "ice-mp: 0x0401 AFTER DMA: retval=0x%04x flags=0x%04x\n",
                    le16_to_cpu(desc.retval), le16_to_cpu(desc.flags));
            
            /* Read back from guest memory to verify what was actually written */
            struct ice_mp_aq_desc readback;
            pci_dma_read(&s->parent_obj, addr, &readback, sizeof(readback));
            fprintf(stderr, "ice-mp: 0x0401 READBACK from guest memory: retval=0x%04x flags=0x%04x p2=0x%02x p3=0x%02x\n",
                    le16_to_cpu(readback.retval), le16_to_cpu(readback.flags),
                    readback.params[2], readback.params[3]);
        }
        
        /* Debug: For 0x0401, verify what was written */
        if (le16_to_cpu(desc.opcode) == 0x0401) {
            fprintf(stderr, "ice-mp: 0x0401 RIGHT AFTER pci_dma_write: params[2]=0x%02x params[3]=0x%02x retval=0x%04x flags=0x%04x\n",
                    desc.params[2], desc.params[3], le16_to_cpu(desc.retval), le16_to_cpu(desc.flags));
        }
    }
}

static void ice_mp_mailbox_process(struct ICEMPState *s)
{
    uint32_t count = s->mbx_atq_len & 0x3FF;
    uint64_t base = ice_mp_dma_addr(s->mbx_atq_bal, s->mbx_atq_bah);

    if (!count)
        return;

    if (s->mbx_atq_head != s->mbx_atq_tail) {
        uint64_t addr = base + (s->mbx_atq_head * sizeof(struct ice_mp_aq_desc));
        struct ice_mp_aq_desc desc;

        pci_dma_read(&s->parent_obj, addr, &desc, sizeof(desc));

        fprintf(stderr, "ice-mp: MBX opcode 0x%04x (head=%d tail=%d)\n",
                le16_to_cpu(desc.opcode), s->mbx_atq_head, s->mbx_atq_tail);

        switch (le16_to_cpu(desc.opcode)) {
        case 0x0801: /* Send message to PF */
        case 0x0802: /* Send message to VF */
            ice_mp_adminq_complete(s, &desc);
            break;
        default:
            fprintf(stderr, "ice-mp: MBX opcode 0x%04x - returning success\n",
                    le16_to_cpu(desc.opcode));
            ice_mp_adminq_complete(s, &desc);
            break;
        }

        pci_dma_write(&s->parent_obj, addr, &desc, sizeof(desc));

        s->mbx_atq_head = (s->mbx_atq_head + 1) % count;
        fprintf(stderr, "ice-mp: MBX command 0x%04x complete, updated ATQH to %d\n",
                le16_to_cpu(desc.opcode), s->mbx_atq_head);
    }
}

#define TYPE_ICE_MP "pci-ice-mp"
OBJECT_DECLARE_SIMPLE_TYPE(ICEMPState, ICE_MP)

#define TYPE_ICE_MP_VF "pci-ice-mp-vf"

/* VF BAR0 Register Offsets (from iavf_register.h) */
#define IAVF_VF_QTX_TAIL1(Q)       (0x00000000 + (Q) * 4)
#define IAVF_VF_QRX_TAIL1(Q)       (0x00002000 + (Q) * 4)
#define IAVF_VF_VFINT_ITRN1(i,Q)   (0x00002800 + (i) * 64 + (Q) * 4)
#define IAVF_VF_VFINT_DYN_CTLN1(Q) (0x00003800 + (Q) * 4)
#define IAVF_VF_VFINT_ICR01        0x00004800
#define IAVF_VF_VFINT_ICR0_ENA1    0x00005000
#define IAVF_VF_VFINT_DYN_CTL01    0x00005C00
#define IAVF_VF_ARQBAH1            0x00006000
#define IAVF_VF_ATQH1              0x00006400
#define IAVF_VF_ATQLEN1            0x00006800
#define IAVF_VF_ARQBAL1            0x00006C00
#define IAVF_VF_ARQT1              0x00007000
#define IAVF_VF_ARQH1              0x00007400
#define IAVF_VF_ATQBAH1            0x00007800
#define IAVF_VF_ATQBAL1            0x00007C00
#define IAVF_VF_ARQLEN1            0x00008000
#define IAVF_VF_ATQT1              0x00008400
#define IAVF_VF_VFGEN_RSTAT        0x00008800
#define IAVF_VF_VFQF_HENA(i)      (0x0000C400 + (i) * 4)
#define IAVF_VF_VFQF_HKEY(i)      (0x0000CC00 + (i) * 4)
#define IAVF_VF_VFQF_HLUT(i)      (0x0000D000 + (i) * 4)

#define IAVF_VF_BAR0_SIZE          0x00010000  /* 64KB to cover all registers */

/* iavf AdminQ sizes */
#define IAVF_AQ_MAX_ENTRIES        32
#define IAVF_AQ_DESC_SIZE          32  /* sizeof(libie_aq_desc) */
#define IAVF_AQ_MAX_BUF_SIZE      4096

/* VF AdminQ descriptor (matches libie_aq_desc / iavf_aq_desc) */
struct iavf_aq_desc {
    uint16_t flags;
    uint16_t opcode;
    uint16_t datalen;
    uint16_t retval;
    uint32_t cookie_high;  /* virtchnl opcode */
    uint32_t cookie_low;   /* virtchnl status */
    uint32_t param0;       /* offset 16 */
    uint32_t param1;       /* offset 20 */
    uint32_t addr_high;    /* offset 24 - data buf DMA addr high */
    uint32_t addr_low;     /* offset 28 - data buf DMA addr low */
} __attribute__((packed));

/* AQ descriptor flag bits */
#define IAVF_AQ_FLAG_DD   0x0001
#define IAVF_AQ_FLAG_CMP  0x0002
#define IAVF_AQ_FLAG_ERR  0x0004
#define IAVF_AQ_FLAG_LB   0x0200
#define IAVF_AQ_FLAG_RD   0x0400
#define IAVF_AQ_FLAG_BUF  0x1000
#define IAVF_AQ_FLAG_SI   0x2000

/* AQ opcodes */
#define IAVF_AQ_OPC_SEND_MSG_TO_PF  0x0801
#define IAVF_AQ_OPC_SEND_MSG_TO_VF  0x0802

/* virtchnl opcodes */
#define VIRTCHNL_OP_VERSION              1
#define VIRTCHNL_OP_GET_VF_RESOURCES     3
#define VIRTCHNL_OP_CONFIG_VSI_QUEUES    6
#define VIRTCHNL_OP_CONFIG_IRQ_MAP       7
#define VIRTCHNL_OP_ENABLE_QUEUES        8
#define VIRTCHNL_OP_DISABLE_QUEUES       9
#define VIRTCHNL_OP_ADD_ETH_ADDR         10
#define VIRTCHNL_OP_DEL_ETH_ADDR         11
#define VIRTCHNL_OP_ADD_VLAN             12
#define VIRTCHNL_OP_DEL_VLAN             13
#define VIRTCHNL_OP_CONFIG_PROMISCUOUS   14
#define VIRTCHNL_OP_GET_STATS            15
#define VIRTCHNL_OP_CONFIG_RSS_KEY       23
#define VIRTCHNL_OP_CONFIG_RSS_LUT       24
#define VIRTCHNL_OP_GET_OFFLOAD_VLAN_V2  51
#define VIRTCHNL_OP_GET_SUPPORTED_RXDIDS 44
#define VIRTCHNL_OP_EVENT                17

/* virtchnl status codes */
#define VIRTCHNL_STATUS_SUCCESS          0
#define VIRTCHNL_STATUS_NOT_SUPPORTED    (-64)

/* virtchnl VF offload capability flags */
#define VIRTCHNL_VF_OFFLOAD_L2           0x00000001
#define VIRTCHNL_VF_OFFLOAD_RSS_PF       0x00080000
#define VIRTCHNL_VF_OFFLOAD_VLAN         0x00010000
#define VIRTCHNL_VF_OFFLOAD_RX_POLLING   0x00020000

/* virtchnl VSI types */
#define VIRTCHNL_VSI_SRIOV               6

/* VF TX/RX queue limits */
#define IAVF_VF_MAX_QUEUES               4
#define IAVF_VF_MSIX_VECTORS             5  /* 4 queue vectors + 1 AdminQ */

typedef struct ICEMPVFQueue {
    uint64_t base;          /* Ring base DMA address */
    uint16_t qlen;          /* Number of entries */
    uint16_t head;
    uint16_t tail;
    bool enabled;
    uint8_t port_id;        /* Assigned port */
    uint32_t int_ctl;       /* MSI-X vector info (needs u32 for BIT(30) cause_ena) */
} ICEMPVFQueue;

typedef struct ICEMPVFState {
    PCIDevice parent_obj;
    MemoryRegion bar0;

    /* Link to parent PF (set during realize) */
    struct ICEMPState *pf;
    uint16_t vf_number;     /* Logical VF index (0-based) */

    /* AdminQ Send Queue (ASQ) registers */
    uint32_t atq_bal;
    uint32_t atq_bah;
    uint32_t atq_len;       /* bit 31 = enable */
    uint32_t atq_head;
    uint32_t atq_tail;

    /* AdminQ Receive Queue (ARQ) registers */
    uint32_t arq_bal;
    uint32_t arq_bah;
    uint32_t arq_len;       /* bit 31 = enable */
    uint32_t arq_head;
    uint32_t arq_tail;

    /* Reset status */
    uint32_t vfgen_rstat;   /* 2 = VFACTIVE */

    /* Interrupt registers */
    uint32_t vfint_icr01;
    uint32_t vfint_icr0_ena1;
    uint32_t vfint_dyn_ctl01;
    uint32_t vfint_dyn_ctln[IAVF_VF_MSIX_VECTORS];
    uint32_t vfint_itrn[3][16]; /* 3 throttle types, 16 queues each */

    /* RSS registers */
    uint32_t vfqf_hena[2];
    uint32_t vfqf_hkey[13];
    uint32_t vfqf_hlut[16];

    /* virtchnl state */
    bool version_negotiated;
    bool resources_configured;
    uint16_t vsi_id;        /* Assigned VSI ID */
    uint8_t mac_addr[6];    /* VF MAC address */

    /* VF TX/RX queues */
    ICEMPVFQueue txq[IAVF_VF_MAX_QUEUES];
    ICEMPVFQueue rxq[IAVF_VF_MAX_QUEUES];
    bool queues_enabled;

    /* MSI-X */
    MemoryRegion msix_bar;
} ICEMPVFState;

OBJECT_DECLARE_SIMPLE_TYPE(ICEMPVFState, ICE_MP_VF)

/* BAR0 MMIO Read */
static uint64_t ice_mp_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    ICEMPState *s = ICE_MP(opaque);
    uint64_t val = 0;

    switch (addr) {
    case ICE_MP_REG_CAPS:
        val = s->caps;
        break;
    
    case ICE_MP_REG_PORT_COUNT:
        val = s->port_count;
        break;
    
    case ICE_MP_REG_EVENT_DB:
        val = s->event_doorbell;
        /* Clear on read */
        s->event_doorbell = 0;
        break;

    case ICE_MP_REG_PF_FW_ATQBAL:
        val = s->atq_bal;
        break;
    case ICE_MP_REG_PF_FW_ATQBAH:
        val = s->atq_bah;
        break;
    case ICE_MP_REG_PF_FW_ATQLEN:
        val = s->atq_len;
        break;
    case ICE_MP_REG_PF_FW_ATQH:
        val = s->atq_head;
        fprintf(stderr, "ice-mp: MMIO READ ATQH = %u (hex 0x%x)\n", (unsigned)val, (unsigned)val);
        break;
    case ICE_MP_REG_PF_FW_ATQT:
        val = s->atq_tail;
        break;
    case ICE_MP_REG_PF_FW_ARQBAL:
        val = s->arq_bal;
        break;
    case ICE_MP_REG_PF_FW_ARQBAH:
        val = s->arq_bah;
        break;
    case ICE_MP_REG_PF_FW_ARQLEN:
        val = s->arq_len;
        break;
    case ICE_MP_REG_PF_FW_ARQH:
        val = s->arq_head;
        break;
    case ICE_MP_REG_PF_FW_ARQT:
        val = s->arq_tail;
        break;

    case ICE_MP_REG_PF_MBX_ATQBAL:
        val = s->mbx_atq_bal;
        break;
    case ICE_MP_REG_PF_MBX_ATQBAH:
        val = s->mbx_atq_bah;
        break;
    case ICE_MP_REG_PF_MBX_ATQLEN:
        val = s->mbx_atq_len;
        break;
    case ICE_MP_REG_PF_MBX_ATQH:
        val = s->mbx_atq_head;
        break;
    case ICE_MP_REG_PF_MBX_ATQT:
        val = s->mbx_atq_tail;
        break;
    case ICE_MP_REG_PF_MBX_ARQBAL:
        val = s->mbx_arq_bal;
        break;
    case ICE_MP_REG_PF_MBX_ARQBAH:
        val = s->mbx_arq_bah;
        break;
    case ICE_MP_REG_PF_MBX_ARQLEN:
        val = s->mbx_arq_len;
        break;
    case ICE_MP_REG_PF_MBX_ARQH:
        val = s->mbx_arq_head;
        break;
    case ICE_MP_REG_PF_MBX_ARQT:
        val = s->mbx_arq_tail;
        break;

    case ICE_MP_REG_PF_PCI_CIAA:
        val = s->pf_pci_ciaa;
        break;

    case ICE_MP_REG_PF_PCI_CIAD:
        val = s->pf_pci_ciad;
        break;

    case ICE_MP_REG_PFGEN_CTRL:
        val = s->pfgen_ctrl;
        break;

    case ICE_MP_REG_GLNVM_ULD:
        val = s->glnvm_uld;
        break;

    case ICE_MP_REG_GLNVM_GENS:
        val = s->glnvm_gens;
        qemu_log("ice-mp: Read GLNVM_GENS = 0x%lx (SR_SIZE=%ld)\n",
                 val, (val >> 5) & 0x7);
        break;

    case ICE_MP_REG_GLNVM_FLA:
        val = s->glnvm_fla;
        qemu_log("ice-mp: Read GLNVM_FLA = 0x%lx (LOCKED=%d)\n", 
                 val, !!(val & BIT(6)));
        break;

    case ICE_MP_REG_GLGEN_RSTCTL:
        val = s->glgen_rstctl;
        break;

    case ICE_MP_REG_GLGEN_RSTAT:
        val = s->glgen_rstat;
        break;

    case ICE_MP_REG_GLGEN_RTRIG:
        val = s->glgen_rtrig;
        break;
    
    default:
        if (addr >= ICE_MP_REG_QTX_COMM_HEAD &&
            addr < ICE_MP_REG_QTX_COMM_HEAD + (ICE_MP_MAX_TX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QTX_COMM_HEAD) / 4;
            if (qid < ICE_MP_MAX_TX_QUEUES) {
                val = s->txq[qid].head;
                fprintf(stderr, "ice-mp: QTX_COMM_HEAD read qid=%u head=%u conf=%d ena=%d base=0x%lx tail=%u\n",
                        qid, s->txq[qid].head, s->txq[qid].configured, s->txq[qid].enabled,
                        (unsigned long)s->txq[qid].base, s->txq[qid].tail);
            }
        } else if (addr >= ICE_MP_REG_QRX_CTRL &&
                   addr < ICE_MP_REG_QRX_CTRL + (ICE_MP_MAX_RX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QRX_CTRL) / 4;
            if (qid < ICE_MP_MAX_RX_QUEUES) {
                val = s->rxq[qid].ctrl;
                if (s->rxq[qid].enabled) {
                    val |= ICE_MP_QRX_CTRL_QENA_STAT_M;
                }
            }
        } else if (addr >= ICE_MP_REG_QINT_TQCTL &&
                   addr < ICE_MP_REG_QINT_TQCTL + (ICE_MP_MAX_TX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QINT_TQCTL) / 4;
            if (qid < ICE_MP_MAX_TX_QUEUES) {
                val = s->txq[qid].int_ctl;
            }
        } else if (addr >= ICE_MP_REG_QINT_RQCTL &&
                   addr < ICE_MP_REG_QINT_RQCTL + (ICE_MP_MAX_RX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QINT_RQCTL) / 4;
            if (qid < ICE_MP_MAX_RX_QUEUES) {
                val = s->rxq[qid].int_ctl;
            }
        } else if (addr >= ICE_MP_REG_GLINT_DYN_CTL &&
                   addr < ICE_MP_REG_GLINT_DYN_CTL + (ICE_MP_MSIX_VECTORS * 4)) {
            uint32_t vec = (addr - ICE_MP_REG_GLINT_DYN_CTL) / 4;
            if (vec < ICE_MP_MSIX_VECTORS) {
                val = s->glint_dyn_ctl[vec];
            }
        } else if (addr == ICE_MP_REG_GLCOMM_QTX_CNTX_CTL) {
            val = s->glcomm_qtx_cntx_ctl;
        } else if (addr >= ICE_MP_REG_GLCOMM_QTX_CNTX_DATA &&
                   addr < ICE_MP_REG_GLCOMM_QTX_CNTX_DATA +
                          (ICE_MP_TX_CTX_DWORDS * 4)) {
            uint32_t idx = (addr - ICE_MP_REG_GLCOMM_QTX_CNTX_DATA) / 4;
            if (idx < ICE_MP_TX_CTX_DWORDS) {
                val = s->glcomm_qtx_cntx_data[idx];
            }
        } else if (addr >= ICE_MP_REG_QRX_CONTEXT &&
                   addr < ICE_MP_REG_QRX_CONTEXT +
                          (ICE_MP_RX_CTX_DWORDS * ICE_MP_QRX_CONTEXT_STRIDE)) {
            /* QRX_CONTEXT(_i, _QRX) = 0x280000 + (_i * 8192 + _QRX * 4)
             * _i = DWORD index (0-7), uses 8192-byte stride
             * _QRX = queue index, uses 4-byte stride within each DWORD block
             */
            uint32_t offset = addr - ICE_MP_REG_QRX_CONTEXT;
            uint32_t idx = offset / ICE_MP_QRX_CONTEXT_STRIDE;  /* DWORD index */
            uint32_t qid = (offset % ICE_MP_QRX_CONTEXT_STRIDE) / 4;  /* queue ID */
            if (idx < ICE_MP_RX_CTX_DWORDS && qid < ICE_MP_MAX_RX_QUEUES) {
                val = s->rx_ctx[qid][idx];
            }
        }
        /* Port status registers */
        else if (addr >= ICE_MP_REG_PORT_STATUS && 
            addr < ICE_MP_REG_PORT_STATUS + (ICE_MP_MAX_PORTS * 4)) {
            uint32_t port_id = (addr - ICE_MP_REG_PORT_STATUS) / 4;
            if (port_id < s->num_ports) {
                val = s->port_status[port_id];
            }
        }
        /* VF mailbox length registers */
        else if (addr >= ICE_MP_REG_VF_MBX_ATQLEN &&
                 addr < ICE_MP_REG_VF_MBX_ATQLEN + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VF_MBX_ATQLEN) / 4;
            if (vf_id < s->num_vfs) {
                val = s->vf_mbx_atqlen[vf_id];
            }
        } else if (addr >= ICE_MP_REG_VF_MBX_ARQLEN &&
                   addr < ICE_MP_REG_VF_MBX_ARQLEN + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VF_MBX_ARQLEN) / 4;
            if (vf_id < s->num_vfs) {
                val = s->vf_mbx_arqlen[vf_id];
            }
        }
        /* VF reset/status registers */
        else if (addr >= ICE_MP_REG_VFGEN_RSTAT &&
                 addr < ICE_MP_REG_VFGEN_RSTAT + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VFGEN_RSTAT) / 4;
            if (vf_id < s->num_vfs) {
                val = s->vfgen_rstat[vf_id];
            }
        } else if (addr >= ICE_MP_REG_VPGEN_VFRTRIG &&
                   addr < ICE_MP_REG_VPGEN_VFRTRIG + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPGEN_VFRTRIG) / 4;
            if (vf_id < s->num_vfs) {
                val = s->vpgen_vfrtrig[vf_id];
            }
        } else if (addr >= ICE_MP_REG_VPGEN_VFRSTAT &&
                   addr < ICE_MP_REG_VPGEN_VFRSTAT + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPGEN_VFRSTAT) / 4;
            if (vf_id < s->num_vfs) {
                val = s->vpgen_vfrstat[vf_id];
            }
        }
        /* VF port map */
        else if (addr >= ICE_MP_REG_VF_PORT_MAP && 
                 addr < ICE_MP_REG_VF_PORT_MAP + ICE_MP_MAX_VFS) {
            uint32_t vf_id = addr - ICE_MP_REG_VF_PORT_MAP;
            if (vf_id < s->num_vfs) {
                val = s->vf_port_map[vf_id];
            }
        }
        /* VF RX queue base/map enable registers */
        else if (addr >= ICE_MP_REG_VPLAN_RX_QBASE &&
                 addr < ICE_MP_REG_VPLAN_RX_QBASE + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_RX_QBASE) / 4;
            if (vf_id < s->num_vfs) {
                val = (s->vf_rxq_base[vf_id] & ICE_MP_VPLAN_RX_QBASE_VFFIRSTQ_M) |
                      ((uint32_t)s->vf_rxq_num[vf_id] << 16);
            }
        } else if (addr >= ICE_MP_REG_VPLAN_RXQ_MAPENA &&
                   addr < ICE_MP_REG_VPLAN_RXQ_MAPENA + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_RXQ_MAPENA) / 4;
            if (vf_id < s->num_vfs) {
                val = s->vf_rxq_mapena[vf_id] ? ICE_MP_VPLAN_RXQ_MAPENA_RX_ENA_M : 0;
            }
        }
        /* VF TX queue base/map enable registers */
        else if (addr >= ICE_MP_REG_VPLAN_TX_QBASE &&
                 addr < ICE_MP_REG_VPLAN_TX_QBASE + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_TX_QBASE) / 4;
            if (vf_id < s->num_vfs) {
                val = (s->vf_txq_base[vf_id] & ICE_MP_VPLAN_TX_QBASE_VFFIRSTQ_M) |
                      ((uint32_t)s->vf_txq_num[vf_id] << 16);
            }
        } else if (addr >= ICE_MP_REG_VPLAN_TXQ_MAPENA &&
                   addr < ICE_MP_REG_VPLAN_TXQ_MAPENA + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_TXQ_MAPENA) / 4;
            if (vf_id < s->num_vfs) {
                val = s->vf_txq_mapena[vf_id] ? ICE_MP_VPLAN_TXQ_MAPENA_TX_ENA_M : 0;
            }
        }
        break;
    }

    return val;
}

/* BAR0 MMIO Write */
static void ice_mp_mmio_write(void *opaque, hwaddr addr, 
                              uint64_t val, unsigned size)
{
    ICEMPState *s = ICE_MP(opaque);

    switch (addr) {
    case ICE_MP_REG_EVENT_DB:
        /* QEMU uses this to inject events */
        s->event_doorbell = val;
        /* Trigger MSI-X interrupt on vector 0 */
        if (msix_enabled(&s->parent_obj)) {
            msix_notify(&s->parent_obj, 0);
        }
        break;

    case ICE_MP_REG_PF_FW_ATQBAL:
        s->atq_bal = val;
        break;
    case ICE_MP_REG_PF_FW_ATQBAH:
        s->atq_bah = val;
        break;
    case ICE_MP_REG_PF_FW_ATQLEN:
        s->atq_len = val;
        break;
    case ICE_MP_REG_PF_FW_ATQH:
        fprintf(stderr, "ice-mp: MMIO WRITE ATQH = %u (old was %u)\n", (unsigned)val, s->atq_head);
        s->atq_head = val;
        break;
    case ICE_MP_REG_PF_FW_ATQT:
        fprintf(stderr, "ice-mp: MMIO WRITE ATQT = %u (head is currently %u)\n", (unsigned)val, s->atq_head);
        s->atq_tail = val;
        ice_mp_adminq_process(s);
        fprintf(stderr, "ice-mp: After processing, ATQH is now %u\n", s->atq_head);
        break;
    case ICE_MP_REG_PF_FW_ARQBAL:
        s->arq_bal = val;
        break;
    case ICE_MP_REG_PF_FW_ARQBAH:
        s->arq_bah = val;
        break;
    case ICE_MP_REG_PF_FW_ARQLEN:
        s->arq_len = val;
        break;
    case ICE_MP_REG_PF_FW_ARQH:
        s->arq_head = val;
        break;
    case ICE_MP_REG_PF_FW_ARQT:
        s->arq_tail = val;
        break;

    case ICE_MP_REG_PF_MBX_ATQBAL:
        s->mbx_atq_bal = val;
        break;
    case ICE_MP_REG_PF_MBX_ATQBAH:
        s->mbx_atq_bah = val;
        break;
    case ICE_MP_REG_PF_MBX_ATQLEN:
        s->mbx_atq_len = val;
        break;
    case ICE_MP_REG_PF_MBX_ATQH:
        s->mbx_atq_head = val;
        break;
    case ICE_MP_REG_PF_MBX_ATQT:
        s->mbx_atq_tail = val;
        ice_mp_mailbox_process(s);
        break;
    case ICE_MP_REG_PF_MBX_ARQBAL:
        s->mbx_arq_bal = val;
        break;
    case ICE_MP_REG_PF_MBX_ARQBAH:
        s->mbx_arq_bah = val;
        break;
    case ICE_MP_REG_PF_MBX_ARQLEN:
        s->mbx_arq_len = val;
        break;
    case ICE_MP_REG_PF_MBX_ARQH:
        s->mbx_arq_head = val;
        break;
    case ICE_MP_REG_PF_MBX_ARQT:
        s->mbx_arq_tail = val;
        break;

    case ICE_MP_REG_PF_PCI_CIAA:
        s->pf_pci_ciaa = val;
        break;

    case ICE_MP_REG_PF_PCI_CIAD:
        s->pf_pci_ciad = val;
        break;

    case ICE_MP_REG_PFGEN_CTRL:
        /* Accept PF reset request and clear immediately */
        s->pfgen_ctrl = 0;
        break;

    case ICE_MP_REG_GLGEN_RSTCTL:
        s->glgen_rstctl = val;
        break;

    case ICE_MP_REG_GLGEN_RTRIG:
        s->glgen_rtrig = val;
        /* Indicate reset done */
        s->glgen_rstat = 0;
        s->glnvm_uld = ICE_MP_GLRST_DONE_MASK;
        break;
    
    default:
        if (addr >= ICE_MP_REG_QTX_COMM_DBELL &&
            addr < ICE_MP_REG_QTX_COMM_DBELL + (ICE_MP_MAX_TX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QTX_COMM_DBELL) / 4;
            if (qid < ICE_MP_MAX_TX_QUEUES) {
                s->txq[qid].tail = (uint16_t)val;
                fprintf(stderr, "ice-mp: TX doorbell qid=%u tail=%u head=%u enabled=%d\n",
                        qid, s->txq[qid].tail, s->txq[qid].head, s->txq[qid].enabled);
                ice_mp_tx_process_queue(s, qid);
            }
        } else if (addr >= ICE_MP_REG_QRX_TAIL &&
                   addr < ICE_MP_REG_QRX_TAIL + (ICE_MP_MAX_RX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QRX_TAIL) / 4;
            if (qid < ICE_MP_MAX_RX_QUEUES) {
                s->rxq[qid].tail = (uint16_t)val;
                fprintf(stderr, "ice-mp: QRX_TAIL write qid=%u tail=%u head=%u configured=%d enabled=%d base=0x%lx qlen=%u port=%u\n",
                        qid, s->rxq[qid].tail, s->rxq[qid].head,
                        s->rxq[qid].configured, s->rxq[qid].enabled,
                        s->rxq[qid].base, s->rxq[qid].qlen, s->rxq[qid].port_id);
                /* Flush any pending received packets now that descriptors are available */
                qemu_flush_queued_packets(qemu_get_queue(s->nic[s->rxq[qid].port_id < s->num_ports ? s->rxq[qid].port_id : 0]));
            }
        } else if (addr >= ICE_MP_REG_QRX_CTRL &&
                   addr < ICE_MP_REG_QRX_CTRL + (ICE_MP_MAX_RX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QRX_CTRL) / 4;
            fprintf(stderr, "ice-mp: QRX_CTRL write qid=%u val=0x%x\n", qid, (uint32_t)val);
            if (qid < ICE_MP_MAX_RX_QUEUES) {
                s->rxq[qid].ctrl = (uint32_t)val;
                if (val & ICE_MP_QRX_CTRL_QENA_REQ_M) {
                    /* Enable queue: set both QENA_REQ and QENA_STAT */
                    s->rxq[qid].enabled = true;
                    s->rxq[qid].ctrl |= ICE_MP_QRX_CTRL_QENA_STAT_M;
                    fprintf(stderr, "ice-mp: Enabling RX queue %u (QENA_REQ set) configured=%d base=0x%lx qlen=%u port=%u head=%u tail=%u\n",
                            qid, s->rxq[qid].configured, s->rxq[qid].base,
                            s->rxq[qid].qlen, s->rxq[qid].port_id,
                            s->rxq[qid].head, s->rxq[qid].tail);
                    /* Flush pending packets now that queue is enabled */
                    if (s->rxq[qid].configured && s->rxq[qid].port_id < s->num_ports) {
                        qemu_flush_queued_packets(qemu_get_queue(s->nic[s->rxq[qid].port_id]));
                    }
                } else {
                    /* Disable queue: clear QENA_STAT */
                    s->rxq[qid].enabled = false;
                    s->rxq[qid].ctrl &= ~ICE_MP_QRX_CTRL_QENA_STAT_M;
                    fprintf(stderr, "ice-mp: Disabling RX queue %u (QENA_REQ cleared)\n", qid);
                }
            }
        } else if (addr >= ICE_MP_REG_QINT_TQCTL &&
                   addr < ICE_MP_REG_QINT_TQCTL + (ICE_MP_MAX_TX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QINT_TQCTL) / 4;
            if (qid < ICE_MP_MAX_TX_QUEUES) {
                s->txq[qid].int_ctl = (uint32_t)val;
                fprintf(stderr, "ice-mp: QINT_TQCTL write qid=%u val=0x%x msix_idx=%u cause_ena=%d\n",
                        qid, (uint32_t)val, (uint32_t)(val & ICE_MP_QINT_MSIX_INDX_M),
                        !!(val & ICE_MP_QINT_CAUSE_ENA_M));
            }
        } else if (addr >= ICE_MP_REG_QINT_RQCTL &&
                   addr < ICE_MP_REG_QINT_RQCTL + (ICE_MP_MAX_RX_QUEUES * 4)) {
            uint32_t qid = (addr - ICE_MP_REG_QINT_RQCTL) / 4;
            if (qid < ICE_MP_MAX_RX_QUEUES) {
                s->rxq[qid].int_ctl = (uint32_t)val;
                uint16_t msix_idx = (uint16_t)(val & ICE_MP_QINT_MSIX_INDX_M);
                
                /* Determine RX queue's port by matching its MSI-X vector
                 * with a configured TX queue that uses the same vector.
                 * The driver allocates the same MSI-X vector for paired
                 * TX and RX queues on the same port.
                 */
                bool port_found = false;
                for (uint16_t txq = 0; txq < ICE_MP_MAX_TX_QUEUES; txq++) {
                    if (s->txq[txq].configured &&
                        (s->txq[txq].int_ctl & ICE_MP_QINT_MSIX_INDX_M) == msix_idx) {
                        s->rxq[qid].port_id = s->txq[txq].port_id;
                        port_found = true;
                        fprintf(stderr, "ice-mp: QINT_RQCTL write qid=%u msix_idx=%u -> port=%u (matched TX qid=%u)\n",
                                qid, msix_idx, s->rxq[qid].port_id, txq);
                        break;
                    }
                }
                if (!port_found) {
                    fprintf(stderr, "ice-mp: QINT_RQCTL write qid=%u msix_idx=%u (no TX queue match)\n",
                            qid, msix_idx);
                }
            }
        } else if (addr >= ICE_MP_REG_GLINT_DYN_CTL &&
                   addr < ICE_MP_REG_GLINT_DYN_CTL + (ICE_MP_MSIX_VECTORS * 4)) {
            uint32_t vec = (addr - ICE_MP_REG_GLINT_DYN_CTL) / 4;
            if (vec < ICE_MP_MSIX_VECTORS) {
                bool masked;
                bool new_intena = !!(val & ICE_MP_GLINT_DYN_CTL_INTENA_M);

                s->glint_dyn_ctl[vec] = (uint32_t)val;
                masked = msix_is_masked(&s->parent_obj, vec);

                fprintf(stderr,
                        "ice-mp: GLINT_DYN_CTL write vec=%u val=0x%x intena=%d msk=%d swint=%d wb_on_itr=%d masked=%d pending=%d\n",
                        vec, (uint32_t)val,
                        new_intena,
                        !!(val & ICE_MP_GLINT_DYN_CTL_INTENA_MSK_M),
                        !!(val & ICE_MP_GLINT_DYN_CTL_SWINT_TRIG_M),
                        !!(val & ICE_MP_GLINT_DYN_CTL_WB_ON_ITR_M),
                        masked,
                        s->irq_pending[vec]);

                /* SWINT_TRIG (bit 2) + INTENA (bit 0): trigger software interrupt */
                if ((val & ICE_MP_GLINT_DYN_CTL_SWINT_TRIG_M) &&
                    new_intena) {
                    if (msix_enabled(&s->parent_obj) && vec < ICE_MP_MSIX_VECTORS) {
                        s->irq_pending[vec] = false;
                        fprintf(stderr,
                                "ice-mp: SWINT IRQ vec=%u masked=%d (SWINT+INTENA triggered)\n",
                                vec, masked);
                        msix_notify(&s->parent_obj, vec);
                    }
                }
                /* When INTENA set to 1: deliver any pending interrupt that was
                 * suppressed while INTENA was 0.  This matches real E810
                 * behavior where re-enabling causes pending causes to fire.
                 */
                else if (new_intena && s->irq_pending[vec]) {
                    s->irq_pending[vec] = false;
                    if (msix_enabled(&s->parent_obj) && vec < ICE_MP_MSIX_VECTORS) {
                        fprintf(stderr,
                                "ice-mp: pending IRQ delivered vec=%u (INTENA re-enabled)\n",
                                vec);
                        msix_notify(&s->parent_obj, vec);
                    }
                }

                /* Flush any queued TAP packets for ports using this vector */
                if (new_intena) {
                    for (uint16_t qi = 0; qi < ICE_MP_MAX_RX_QUEUES; qi++) {
                        if (s->rxq[qi].configured && s->rxq[qi].enabled &&
                            (s->rxq[qi].int_ctl & ICE_MP_QINT_MSIX_INDX_M) == vec) {
                            uint8_t pid = s->rxq[qi].port_id;
                            if (pid < s->num_ports && s->nic[pid]) {
                                qemu_flush_queued_packets(
                                    qemu_get_queue(s->nic[pid]));
                            }
                            break;
                        }
                    }
                }
            }
        } else if (addr == ICE_MP_REG_GLCOMM_QTX_CNTX_CTL) {
            uint32_t cmd = (val & ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_M) >> 16;
            uint32_t qid = val & ICE_MP_GLCOMM_QTX_CNTX_CTL_QUEUE_ID_M;

            s->glcomm_qtx_cntx_ctl = (uint32_t)val;

            fprintf(stderr, "ice-mp: GLCOMM_QTX_CNTX_CTL write val=0x%lx qid=%u cmd=%u exec=%d\n",
                    val, qid, cmd, !!(val & ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_EXEC));

            if ((val & ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_EXEC) &&
                qid < ICE_MP_MAX_TX_QUEUES) {
                if (cmd == ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_WRITE_NO_DYN) {
                    fprintf(stderr, "ice-mp: Writing TX ctx for qid=%u\n", qid);
                    for (uint32_t i = 0; i < ICE_MP_TX_CTX_DWORDS; i++) {
                        s->tx_ctx[qid][i] = s->glcomm_qtx_cntx_data[i];
                    }
                    ice_mp_update_txq_from_ctx(s, (uint16_t)qid);
                } else if (cmd == ICE_MP_GLCOMM_QTX_CNTX_CTL_CMD_READ) {
                    for (uint32_t i = 0; i < ICE_MP_TX_CTX_DWORDS; i++) {
                        s->glcomm_qtx_cntx_data[i] = s->tx_ctx[qid][i];
                    }
                }
            }
        } else if (addr >= ICE_MP_REG_GLCOMM_QTX_CNTX_DATA &&
                   addr < ICE_MP_REG_GLCOMM_QTX_CNTX_DATA +
                          (ICE_MP_TX_CTX_DWORDS * 4)) {
            uint32_t idx = (addr - ICE_MP_REG_GLCOMM_QTX_CNTX_DATA) / 4;
            if (idx < ICE_MP_TX_CTX_DWORDS) {
                s->glcomm_qtx_cntx_data[idx] = (uint32_t)val;
            }
        } else if (addr >= ICE_MP_REG_QRX_CONTEXT &&
                   addr < ICE_MP_REG_QRX_CONTEXT +
                          (ICE_MP_RX_CTX_DWORDS * ICE_MP_QRX_CONTEXT_STRIDE)) {
            /* QRX_CONTEXT(_i, _QRX) = 0x280000 + (_i * 8192 + _QRX * 4)
             * _i = DWORD index (0-7), uses 8192-byte stride
             * _QRX = queue index, uses 4-byte stride within each DWORD block
             */
            uint32_t offset = addr - ICE_MP_REG_QRX_CONTEXT;
            uint32_t idx = offset / ICE_MP_QRX_CONTEXT_STRIDE;  /* DWORD index */
            uint32_t qid = (offset % ICE_MP_QRX_CONTEXT_STRIDE) / 4;  /* queue ID */
            if (idx < ICE_MP_RX_CTX_DWORDS && qid < ICE_MP_MAX_RX_QUEUES) {
                s->rx_ctx[qid][idx] = (uint32_t)val;
                ice_mp_update_rxq_from_ctx(s, (uint16_t)qid);
                if (idx == ICE_MP_RX_CTX_DWORDS - 1) {
                    fprintf(stderr, "ice-mp: RX context write complete qid=%u base=0x%lx qlen=%u configured=%d port=%u\n",
                            qid, s->rxq[qid].base, s->rxq[qid].qlen, s->rxq[qid].configured, s->rxq[qid].port_id);
                }
            }
        }
        /* Port status - allow external control for testing */
        else if (addr >= ICE_MP_REG_PORT_STATUS && 
            addr < ICE_MP_REG_PORT_STATUS + (ICE_MP_MAX_PORTS * 4)) {
            uint32_t port_id = (addr - ICE_MP_REG_PORT_STATUS) / 4;
            if (port_id < s->num_ports) {
                s->port_status[port_id] = val;
            }
        }
        /* VF mailbox length registers */
        else if (addr >= ICE_MP_REG_VF_MBX_ATQLEN &&
                 addr < ICE_MP_REG_VF_MBX_ATQLEN + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VF_MBX_ATQLEN) / 4;
            if (vf_id < s->num_vfs) {
                s->vf_mbx_atqlen[vf_id] = val;
                fprintf(stderr, "ice-mp: VF%u MBX ATQLEN <- 0x%x\n", vf_id, (unsigned)val);
            }
        } else if (addr >= ICE_MP_REG_VF_MBX_ARQLEN &&
                   addr < ICE_MP_REG_VF_MBX_ARQLEN + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VF_MBX_ARQLEN) / 4;
            if (vf_id < s->num_vfs) {
                s->vf_mbx_arqlen[vf_id] = val;
                fprintf(stderr, "ice-mp: VF%u MBX ARQLEN <- 0x%x\n", vf_id, (unsigned)val);
            }
        }
        /* VF reset/status registers */
        else if (addr >= ICE_MP_REG_VFGEN_RSTAT &&
                 addr < ICE_MP_REG_VFGEN_RSTAT + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VFGEN_RSTAT) / 4;
            if (vf_id < s->num_vfs) {
                s->vfgen_rstat[vf_id] = val;
                fprintf(stderr, "ice-mp: VF%u GEN_RSTAT <- 0x%x\n", vf_id, (unsigned)val);
            }
        } else if (addr >= ICE_MP_REG_VPGEN_VFRTRIG &&
                   addr < ICE_MP_REG_VPGEN_VFRTRIG + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPGEN_VFRTRIG) / 4;
            if (vf_id < s->num_vfs) {
                s->vpgen_vfrtrig[vf_id] = val;
                s->vpgen_vfrstat[vf_id] = ICE_MP_VFRSTAT_VFRD;
                fprintf(stderr, "ice-mp: VF%u VFRTRIG <- 0x%x (VFRSTAT=0x%x)\n",
                        vf_id, (unsigned)val, (unsigned)s->vpgen_vfrstat[vf_id]);
            }
        } else if (addr >= ICE_MP_REG_VPGEN_VFRSTAT &&
                   addr < ICE_MP_REG_VPGEN_VFRSTAT + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPGEN_VFRSTAT) / 4;
            if (vf_id < s->num_vfs) {
                s->vpgen_vfrstat[vf_id] = val;
                fprintf(stderr, "ice-mp: VF%u VFRSTAT <- 0x%x\n", vf_id, (unsigned)val);
            }
        }
        /* VF to Port mapping - allow sysfs to update mapping */
        else if (addr >= ICE_MP_REG_VF_PORT_MAP && 
                 addr < ICE_MP_REG_VF_PORT_MAP + ICE_MP_MAX_VFS) {
            uint32_t vf_id = addr - ICE_MP_REG_VF_PORT_MAP;
            uint8_t new_port = val & 0xFF;
            
            if (vf_id < s->num_vfs && new_port < s->num_ports) {
                s->vf_port_map[vf_id] = new_port;
                fprintf(stderr, "ice-mp: VF %u reassigned to port %u via sysfs\n", 
                        vf_id, new_port);
            } else {
                fprintf(stderr, "ice-mp: Invalid VF-to-port assignment: VF %u -> Port %u "
                        "(num_vfs=%u, num_ports=%u)\n",
                        vf_id, new_port, s->num_vfs, s->num_ports);
            }
        }
        /* VF RX queue base/map enable registers */
        else if (addr >= ICE_MP_REG_VPLAN_RX_QBASE &&
                 addr < ICE_MP_REG_VPLAN_RX_QBASE + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_RX_QBASE) / 4;
            if (vf_id < s->num_vfs) {
                s->vf_rxq_base[vf_id] = val & ICE_MP_VPLAN_RX_QBASE_VFFIRSTQ_M;
                s->vf_rxq_num[vf_id] = (val & ICE_MP_VPLAN_RX_QBASE_VFNUMQ_M) >> 16;
            }
        } else if (addr >= ICE_MP_REG_VPLAN_RXQ_MAPENA &&
                   addr < ICE_MP_REG_VPLAN_RXQ_MAPENA + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_RXQ_MAPENA) / 4;
            if (vf_id < s->num_vfs) {
                s->vf_rxq_mapena[vf_id] = (val & ICE_MP_VPLAN_RXQ_MAPENA_RX_ENA_M) != 0;
            }
        }
        /* VF TX queue base/map enable registers */
        else if (addr >= ICE_MP_REG_VPLAN_TX_QBASE &&
                 addr < ICE_MP_REG_VPLAN_TX_QBASE + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_TX_QBASE) / 4;
            if (vf_id < s->num_vfs) {
                s->vf_txq_base[vf_id] = val & ICE_MP_VPLAN_TX_QBASE_VFFIRSTQ_M;
                s->vf_txq_num[vf_id] = (val & ICE_MP_VPLAN_TX_QBASE_VFNUMQ_M) >> 16;
            }
        } else if (addr >= ICE_MP_REG_VPLAN_TXQ_MAPENA &&
                   addr < ICE_MP_REG_VPLAN_TXQ_MAPENA + (ICE_MP_MAX_VFS * 4)) {
            uint32_t vf_id = (addr - ICE_MP_REG_VPLAN_TXQ_MAPENA) / 4;
            if (vf_id < s->num_vfs) {
                s->vf_txq_mapena[vf_id] = (val & ICE_MP_VPLAN_TXQ_MAPENA_TX_ENA_M) != 0;
            }
        }
        break;
    }
}

static const MemoryRegionOps ice_mp_mmio_ops = {
    .read = ice_mp_mmio_read,
    .write = ice_mp_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* Device Initialization */
static void ice_mp_realize(PCIDevice *pci_dev, Error **errp)
{
    ICEMPState *s = ICE_MP(pci_dev);
    uint8_t *pci_conf = pci_dev->config;
    int ret;

    /* Initialize PCI Express device */
    if (pcie_endpoint_cap_init(pci_dev, 0x80) < 0) {
        error_setg(errp, "Failed to initialize PCIe capability");
        return;
    }

    /* Initialize PCI config space */
    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(pci_conf, PCI_DEVICE_ID_ICE_MP);
    pci_config_set_class(pci_conf, PCI_CLASS_NETWORK_ETHERNET);
    pci_config_set_revision(pci_conf, 0x01);

    /* Initialize BAR0 for MMIO */
    memory_region_init_io(&s->bar0, OBJECT(s), &ice_mp_mmio_ops, s,
                         "ice-mp-mmio", ICE_MP_BAR0_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);

    /* Initialize BAR1 for MSI-X */
    s->msix_bar_idx = 1;
    memory_region_init(&s->bar1, OBJECT(s), "ice-mp-msix", 0x4000);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar1);
    
    ret = msix_init(pci_dev, ICE_MP_MSIX_VECTORS,
                    &s->bar1, s->msix_bar_idx, 0,
                    &s->bar1, s->msix_bar_idx, 0x1000,
                    0x70, errp);
    if (ret < 0) {
        return;
    }

    /* Mark all MSI-X vectors as used so msix_notify() will deliver them.
     * Without this, msix_notify() silently drops interrupts because
     * msix_entry_used[vector] is 0 for all vectors. */
    for (int i = 0; i < ICE_MP_MSIX_VECTORS; i++) {
        msix_vector_use(pci_dev, i);
    }

    /* Initialize deferred interrupt timer */
    s->tx_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   ice_mp_tx_irq_timer_cb, s);
    s->tx_irq_pending = 0;

    /* Initialize SR-IOV if VFs are configured */
    if (s->num_vfs > 0) {
        uint32_t init_vfs = s->num_vfs;
        /* Mark PF as multifunction so VFs at non-zero functions pass
         * pci_init_multifunction() validation in QEMU's PCI core */
        pci_dev->cap_present |= QEMU_PCI_CAP_MULTIFUNCTION;
        fprintf(stderr, "ice-mp: SR-IOV total_vfs=%u initial_vfs=%u\n",
                s->num_vfs, init_vfs);
        pcie_sriov_pf_init(pci_dev, ICE_MP_SRIOV_OFFSET, "pci-ice-mp-vf",
                  ICE_MP_VF_DEV_ID, init_vfs, s->num_vfs,
                  ICE_MP_VF_OFFSET, ICE_MP_VF_STRIDE);
        pcie_sriov_pf_init_vf_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY,
                                  IAVF_VF_BAR0_SIZE);
        /* VF BAR 3 for MSI-X table/PBA */
        pcie_sriov_pf_init_vf_bar(pci_dev, 3, PCI_BASE_ADDRESS_SPACE_MEMORY,
                                  0x4000);
        fprintf(stderr,
            "ice-mp: SR-IOV cfg vf_offset=0x%x vf_stride=0x%x sup_pg=0x%x sys_pg=0x%x\n",
            pci_get_word(pci_conf + ICE_MP_SRIOV_OFFSET + PCI_SRIOV_VF_OFFSET),
            pci_get_word(pci_conf + ICE_MP_SRIOV_OFFSET + PCI_SRIOV_VF_STRIDE),
            pci_get_word(pci_conf + ICE_MP_SRIOV_OFFSET + PCI_SRIOV_SUP_PGSIZE),
            pci_get_word(pci_conf + ICE_MP_SRIOV_OFFSET + PCI_SRIOV_SYS_PGSIZE));
        
        /* We don't need actual VF BARs for our test device,
         * but we could add them here if needed */
    }

    /* Initialize register values */
    s->caps = ICE_MP_CAP_MULTI_PORT | ICE_MP_CAP_MSIX;
    if (s->num_vfs > 0) {
        s->caps |= ICE_MP_CAP_SRIOV;
    }
    s->port_count = s->num_ports;
    
    /* Initialize all ports as link UP at 100Gbps (default after power-on) */
    for (uint32_t i = 0; i < s->num_ports; i++) {
        s->port_status[i] = ICE_MP_PORT_LINK_UP | 
                           (ICE_MP_SPEED_100G << ICE_MP_PORT_SPEED_SHIFT);
        fprintf(stderr, "ice-mp: realize() - Port %u initialized: port_status=0x%x (LINK_UP=0x%x)\n",
                i, s->port_status[i], ICE_MP_PORT_LINK_UP);
    }
    
    /* Initialize VF-to-port mapping (round-robin) */
    for (uint32_t i = 0; i < s->num_vfs; i++) {
        s->vf_port_map[i] = i % s->num_ports;
    }

    for (uint32_t i = 0; i < ICE_MP_MAX_VSI; i++) {
        s->vsi_to_vf[i] = 0xFFFF;
        s->vsi_to_port[i] = 0;
    }
    s->pf_vsi_count = 0;
    s->last_vsi_was_pf = false;

    for (uint32_t i = 0; i < s->num_vfs; i++) {
        s->vf_rxq_base[i] = 0;
        s->vf_rxq_num[i] = 0;
        s->vf_rxq_mapena[i] = false;
        s->vf_txq_base[i] = 0;
        s->vf_txq_num[i] = 0;
        s->vf_txq_mapena[i] = false;
    }

    ice_mp_rule_clear_all(s);
    
    s->event_doorbell = 0;
    s->pfgen_ctrl = 0;
    s->glnvm_uld = ICE_MP_GLRST_DONE_MASK;
    /* GLNVM_GENS: bits 5-7 = 0x3 (8KB shadow RAM) */
    s->glnvm_gens = (3 << 5);  /* SR_SIZE[2:0] = 3 */
    /* GLNVM_FLA: bit 6 = LOCKED (normal programming mode) */
    s->glnvm_fla = BIT(6);  /* BIT(6) set = normal mode, clear = blank mode */
    s->glgen_rstctl = 0;
    s->glgen_rstat = 0;
    s->glgen_rtrig = 0;

    for (uint32_t i = 0; i < s->num_vfs; i++) {
        s->vf_mbx_atqlen[i] = 0;
        s->vf_mbx_arqlen[i] = 0;
        s->vfgen_rstat[i] = 0;
        s->vpgen_vfrtrig[i] = 0;
        s->vpgen_vfrstat[i] = ICE_MP_VFRSTAT_VFRD;
    }
    s->pf_pci_ciaa = 0;
    s->pf_pci_ciad = 0;

    /* Allocate NVM flash banks dynamically
     * Sizes: NVM=512KB (256K words), OROM=128KB (64K words), Netlist=64KB (32K words)
     */
    s->nvm_flash_size = 256 * 1024;      /* 512KB = 256K words */
    s->orom_flash_size = 64 * 1024;      /* 128KB = 64K words */
    s->netlist_flash_size = 32 * 1024;   /* 64KB = 32K words */
    
    s->nvm_flash = g_malloc0(s->nvm_flash_size * sizeof(uint16_t));
    s->orom_flash = g_malloc0(s->orom_flash_size * sizeof(uint16_t));
    s->netlist_flash = g_malloc0(s->netlist_flash_size * sizeof(uint16_t));

    /* Initialize NVM flash banks with proper CSS header structure.
     * Driver reads CSS header length from words 0x02-0x03 (ICE_NVM_CSS_HDR_LEN_L/H)
     * Formula: total_hdr = (css_hdr_len_dword * 2) + ICE_NVM_AUTH_HEADER_LEN (8 words)
     * We set CSS header = 4 DWORDs = 8 words, so total = 8*2 + 8 = 24 words
     * Shadow RAM copy starts at word 24
     * Note: Arrays are already zero-initialized by g_malloc0
     */

    /* NVM bank CSS header structure:
     * Word 0x00: Module type / signature
     * Word 0x01: Module vendor
     * Word 0x02: CSS header length (low) - in DWORDs
     * Word 0x03: CSS header length (high) - in DWORDs
     * Words 0x04-0x07: Reserved/additional CSS fields
     * Words 0x08-0x0F: Authentication header (8 words)
     * Words 0x10-0x1F: Padding to 32-word boundary
     * Words 0x20+: Shadow RAM copy (starts at word 32 = roundup(16, 32))
     */
    s->nvm_flash[0x00] = 0x0006;  /* Module type: NVM */
    s->nvm_flash[0x01] = 0x8086;  /* Vendor: Intel */
    s->nvm_flash[0x02] = 0x0004;  /* CSS header length = 4 DWORDs (low word) */
    s->nvm_flash[0x03] = 0x0000;  /* CSS header length (high word) */
    
    /* Fill shadow RAM copy in NVM bank starting at word 32
     * CSS header: 4 DWORDs = 8 words
     * Total CSS header length: (4 * 2) + 8 (auth) = 16 words
     * Rounded up to 32-word boundary: roundup(16, 32) = 32
     */
    for (int i = 0; i < 256; i++) {  /* Copy 256 shadow RAM words */
        s->nvm_flash[i + 32] = ice_mp_nvm_shadow_word((uint16_t)i);
    }

    /* OROM bank CSS header (similar structure) */
    s->orom_flash[0x00] = 0x0007;  /* Module type: OROM */
    s->orom_flash[0x01] = 0x8086;
    s->orom_flash[0x02] = 0x0004;
    s->orom_flash[0x03] = 0x0000;

    /* Netlist bank CSS header */
    s->netlist_flash[0x00] = 0x0008;  /* Module type: Netlist */
    s->netlist_flash[0x01] = 0x8086;
    s->netlist_flash[0x02] = 0x0004;
    s->netlist_flash[0x03] = 0x0000;

    /* Initialize resource lock state */
    s->glbl_cfg_lock_held = false;
    s->lock_timeout = 0;
    s->res_counter = 1;
    
    /* Initialize scheduler TEID counter (start after reserved nodes) */
    s->next_sched_teid = 0x16000003;  /* Next after minimal topology (root + TC + entry point) */
    s->next_vsi_num = 1;

    /* Initialize base scheduler node table entries */
    memset(s->sched_nodes, 0, sizeof(s->sched_nodes));
    /* Root node: index 0 = TEID 0x16000000, parent=0xFFFFFFFF */
    s->sched_nodes[0].valid = true;
    s->sched_nodes[0].parent_teid = 0xFFFFFFFF;
    s->sched_nodes[0].elem_type = ICE_AQC_ELEM_TYPE_ROOT_PORT;
    /* TC node: index 1 = TEID 0x16000001, parent=root */
    s->sched_nodes[1].valid = true;
    s->sched_nodes[1].parent_teid = 0x16000000;
    s->sched_nodes[1].elem_type = ICE_AQC_ELEM_TYPE_TC;
    /* Entry point: index 2 = TEID 0x16000002, parent=TC */
    s->sched_nodes[2].valid = true;
    s->sched_nodes[2].parent_teid = 0x16000001;
    s->sched_nodes[2].elem_type = ICE_AQC_ELEM_TYPE_ENTRY_POINT;

    ice_mp_init_nics(s, pci_dev);
}

/* Device cleanup */
static void ice_mp_exit(PCIDevice *pci_dev)
{
    ICEMPState *s = ICE_MP(pci_dev);

    /* Free deferred interrupt timer */
    if (s->tx_irq_timer) {
        timer_free(s->tx_irq_timer);
        s->tx_irq_timer = NULL;
    }

    for (uint32_t i = 0; i < s->num_ports; i++) {
        if (s->nic[i]) {
            qemu_del_nic(s->nic[i]);
            s->nic[i] = NULL;
        }
    }
    
    /* Free allocated flash arrays */
    g_free(s->nvm_flash);
    g_free(s->orom_flash);
    g_free(s->netlist_flash);
    
    if (s->num_vfs > 0) {
        pcie_sriov_pf_exit(pci_dev);
    }
    
    msix_uninit(pci_dev, &s->bar1, &s->bar1);
}

/* Device reset */
static void ice_mp_reset_hold(Object *obj, ResetType type)
{
    ICEMPState *s = ICE_MP(obj);
    
    /* Reset all ports to link UP at 100Gbps full duplex by default */
    for (uint32_t i = 0; i < s->num_ports; i++) {
        s->port_status[i] = ICE_MP_PORT_LINK_UP | 
                           (ICE_MP_SPEED_100G << ICE_MP_PORT_SPEED_SHIFT);
    }

    s->pfgen_ctrl = 0;
    s->glnvm_uld = ICE_MP_GLRST_DONE_MASK;
    s->glnvm_gens = (3 << 5);  /* SR_SIZE[2:0] = 3 (8KB shadow RAM) */
    s->glnvm_fla = BIT(6);  /* BIT(6) = LOCKED = normal programming mode */
    s->glgen_rstctl = 0;
    s->glgen_rstat = 0;
    s->glgen_rtrig = 0;
    
    s->next_vsi_num = 1;

    ice_mp_rule_clear_all(s);

    for (uint32_t i = 0; i < ICE_MP_MAX_VSI; i++) {
        s->vsi_to_vf[i] = 0xFFFF;
        s->vsi_to_port[i] = 0;
    }
    s->pf_vsi_count = 0;
    s->last_vsi_was_pf = false;

    for (uint32_t i = 0; i < s->num_vfs; i++) {
        s->vf_rxq_base[i] = 0;
        s->vf_rxq_num[i] = 0;
        s->vf_rxq_mapena[i] = false;
        s->vf_txq_base[i] = 0;
        s->vf_txq_num[i] = 0;
        s->vf_txq_mapena[i] = false;
    }

    for (uint32_t i = 0; i < s->num_vfs; i++) {
        s->vf_mbx_atqlen[i] = 0;
        s->vf_mbx_arqlen[i] = 0;
        s->vfgen_rstat[i] = 0;
        s->vpgen_vfrtrig[i] = 0;
        s->vpgen_vfrstat[i] = ICE_MP_VFRSTAT_VFRD;
    }
    s->pf_pci_ciaa = 0;
    s->pf_pci_ciad = 0;

    for (uint32_t i = 0; i < ICE_MP_MAX_TX_QUEUES; i++) {
        s->txq[i].head = 0;
        s->txq[i].tail = 0;
        s->txq[i].enabled = false;
    }

    for (uint32_t i = 0; i < ICE_MP_MAX_RX_QUEUES; i++) {
        s->rxq[i].head = 0;
        s->rxq[i].tail = 0;
        s->rxq[i].enabled = false;
    }

    /* Inject reset event */
    s->event_doorbell = (ICE_MP_EVENT_RESET << 8);
    if (msix_enabled(&s->parent_obj)) {
        msix_notify(&s->parent_obj, 0);
    }
}

/* Device properties */
static Property ice_mp_properties[] = {
    DEFINE_PROP_UINT32("ports", ICEMPState, num_ports, 4),
    DEFINE_PROP_UINT32("vfs", ICEMPState, num_vfs, 0),
    DEFINE_PROP_UINT32("rxq-map-mode", ICEMPState, rxq_map_mode, 0),
    DEFINE_PROP_UINT32("queues-per-port", ICEMPState, queues_per_port, 0),
    DEFINE_PROP_NETDEV("netdev0", ICEMPState, conf[0].peers),
    DEFINE_PROP_NETDEV("netdev1", ICEMPState, conf[1].peers),
    DEFINE_PROP_NETDEV("netdev2", ICEMPState, conf[2].peers),
    DEFINE_PROP_NETDEV("netdev3", ICEMPState, conf[3].peers),
    DEFINE_PROP_MACADDR("mac0", ICEMPState, conf[0].macaddr),
    DEFINE_PROP_MACADDR("mac1", ICEMPState, conf[1].macaddr),
    DEFINE_PROP_MACADDR("mac2", ICEMPState, conf[2].macaddr),
    DEFINE_PROP_MACADDR("mac3", ICEMPState, conf[3].macaddr),
    DEFINE_PROP_END_OF_LIST(),
};

/* Class initialization */
static void ice_mp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = ice_mp_realize;
    k->exit = ice_mp_exit;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_ICE_MP;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    
    rc->phases.hold = ice_mp_reset_hold;
    dc->desc = "Intel E810 Multi-Port PF Test Device";
    device_class_set_props(dc, ice_mp_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static uint64_t ice_mp_vf_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    ICEMPVFState *vf = (ICEMPVFState *)opaque;
    uint64_t val = 0;

    switch (addr) {
    case IAVF_VF_VFGEN_RSTAT:
        val = vf->vfgen_rstat;
        break;

    /* AdminQ Send Queue (ASQ) registers */
    case IAVF_VF_ATQBAL1:
        val = vf->atq_bal;
        break;
    case IAVF_VF_ATQBAH1:
        val = vf->atq_bah;
        break;
    case IAVF_VF_ATQLEN1:
        val = vf->atq_len;
        break;
    case IAVF_VF_ATQH1:
        val = vf->atq_head;
        break;
    case IAVF_VF_ATQT1:
        val = vf->atq_tail;
        break;

    /* AdminQ Receive Queue (ARQ) registers */
    case IAVF_VF_ARQBAL1:
        val = vf->arq_bal;
        break;
    case IAVF_VF_ARQBAH1:
        val = vf->arq_bah;
        break;
    case IAVF_VF_ARQLEN1:
        val = vf->arq_len;
        break;
    case IAVF_VF_ARQH1:
        val = vf->arq_head;
        break;
    case IAVF_VF_ARQT1:
        val = vf->arq_tail;
        break;

    /* Interrupt registers */
    case IAVF_VF_VFINT_ICR01:
        val = vf->vfint_icr01;
        vf->vfint_icr01 = 0;  /* Clear on read */
        break;
    case IAVF_VF_VFINT_ICR0_ENA1:
        val = vf->vfint_icr0_ena1;
        break;
    case IAVF_VF_VFINT_DYN_CTL01:
        val = vf->vfint_dyn_ctl01;
        break;

    default:
        /* TX queue tail registers: 0x0000 + Q*4 */
        if (addr < IAVF_VF_MAX_QUEUES * 4) {
            uint32_t q = addr / 4;
            if (q < IAVF_VF_MAX_QUEUES) {
                val = vf->txq[q].tail;
            }
        }
        /* RX queue tail registers: 0x2000 + Q*4 */
        else if (addr >= 0x2000 && addr < 0x2000 + IAVF_VF_MAX_QUEUES * 4) {
            uint32_t q = (addr - 0x2000) / 4;
            if (q < IAVF_VF_MAX_QUEUES) {
                val = vf->rxq[q].tail;
            }
        }
        /* Interrupt throttle: 0x2800 + i*64 + Q*4 */
        else if (addr >= 0x2800 && addr < 0x2800 + 3 * 64) {
            uint32_t offset = addr - 0x2800;
            uint32_t i = offset / 64;
            uint32_t q = (offset % 64) / 4;
            if (i < 3 && q < 16) {
                val = vf->vfint_itrn[i][q];
            }
        }
        /* Per-queue dynamic interrupt control: 0x3800 + Q*4 */
        else if (addr >= 0x3800 && addr < 0x3800 + IAVF_VF_MSIX_VECTORS * 4) {
            uint32_t q = (addr - 0x3800) / 4;
            if (q < IAVF_VF_MSIX_VECTORS) {
                val = vf->vfint_dyn_ctln[q];
            }
        }
        /* RSS HENA: 0xC400 + i*4 */
        else if (addr >= 0xC400 && addr < 0xC400 + 2 * 4) {
            uint32_t i = (addr - 0xC400) / 4;
            val = vf->vfqf_hena[i];
        }
        /* RSS HKEY: 0xCC00 + i*4 */
        else if (addr >= 0xCC00 && addr < 0xCC00 + 13 * 4) {
            uint32_t i = (addr - 0xCC00) / 4;
            val = vf->vfqf_hkey[i];
        }
        /* RSS HLUT: 0xD000 + i*4 */
        else if (addr >= 0xD000 && addr < 0xD000 + 16 * 4) {
            uint32_t i = (addr - 0xD000) / 4;
            val = vf->vfqf_hlut[i];
        }
        break;
    }

    return val;
}

/* Forward declarations */
static void ice_mp_vf_process_atq(ICEMPVFState *vf);
static void ice_mp_vf_tx_process(ICEMPVFState *vf, uint16_t qid);

static void ice_mp_vf_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    ICEMPVFState *vf = (ICEMPVFState *)opaque;

    switch (addr) {
    /* AdminQ Send Queue (ASQ) registers */
    case IAVF_VF_ATQBAL1:
        vf->atq_bal = (uint32_t)val;
        break;
    case IAVF_VF_ATQBAH1:
        vf->atq_bah = (uint32_t)val;
        break;
    case IAVF_VF_ATQLEN1:
        vf->atq_len = (uint32_t)val;
        fprintf(stderr, "ice-mp-vf%u: ATQLEN1 <- 0x%x (entries=%u enable=%d)\n",
                vf->vf_number, (unsigned)val,
                (unsigned)(val & 0x3FF), !!(val & BIT(31)));
        break;
    case IAVF_VF_ATQH1:
        vf->atq_head = (uint32_t)val;
        break;
    case IAVF_VF_ATQT1:
        vf->atq_tail = (uint32_t)val;
        fprintf(stderr, "ice-mp-vf%u: ATQT1 <- %u (triggering AdminQ processing)\n",
                vf->vf_number, (unsigned)val);
        /* Trigger AdminQ command processing */
        ice_mp_vf_process_atq(vf);
        break;

    /* AdminQ Receive Queue (ARQ) registers */
    case IAVF_VF_ARQBAL1:
        vf->arq_bal = (uint32_t)val;
        break;
    case IAVF_VF_ARQBAH1:
        vf->arq_bah = (uint32_t)val;
        break;
    case IAVF_VF_ARQLEN1:
        vf->arq_len = (uint32_t)val;
        fprintf(stderr, "ice-mp-vf%u: ARQLEN1 <- 0x%x (entries=%u enable=%d)\n",
                vf->vf_number, (unsigned)val,
                (unsigned)(val & 0x3FF), !!(val & BIT(31)));
        break;
    case IAVF_VF_ARQH1:
        vf->arq_head = (uint32_t)val;
        break;
    case IAVF_VF_ARQT1:
        vf->arq_tail = (uint32_t)val;
        break;

    /* Interrupt registers */
    case IAVF_VF_VFINT_ICR0_ENA1:
        vf->vfint_icr0_ena1 = (uint32_t)val;
        break;
    case IAVF_VF_VFINT_DYN_CTL01:
        vf->vfint_dyn_ctl01 = (uint32_t)val;
        break;

    default:
        /* TX queue tail registers: 0x0000 + Q*4 */
        if (addr < IAVF_VF_MAX_QUEUES * 4) {
            uint32_t q = addr / 4;
            if (q < IAVF_VF_MAX_QUEUES) {
                vf->txq[q].tail = (uint16_t)val;
                /* Process TX queue if enabled */
                if (vf->txq[q].enabled && vf->pf) {
                    ice_mp_vf_tx_process(vf, q);
                }
            }
        }
        /* RX queue tail registers: 0x2000 + Q*4 */
        else if (addr >= 0x2000 && addr < 0x2000 + IAVF_VF_MAX_QUEUES * 4) {
            uint32_t q = (addr - 0x2000) / 4;
            if (q < IAVF_VF_MAX_QUEUES) {
                vf->rxq[q].tail = (uint16_t)val;
            }
        }
        /* Interrupt throttle: 0x2800 + i*64 + Q*4 */
        else if (addr >= 0x2800 && addr < 0x2800 + 3 * 64) {
            uint32_t offset = addr - 0x2800;
            uint32_t i = offset / 64;
            uint32_t q = (offset % 64) / 4;
            if (i < 3 && q < 16) {
                vf->vfint_itrn[i][q] = (uint32_t)val;
            }
        }
        /* Per-queue dynamic interrupt control: 0x3800 + Q*4 */
        else if (addr >= 0x3800 && addr < 0x3800 + IAVF_VF_MSIX_VECTORS * 4) {
            uint32_t q = (addr - 0x3800) / 4;
            if (q < IAVF_VF_MSIX_VECTORS) {
                vf->vfint_dyn_ctln[q] = (uint32_t)val;
            }
        }
        /* RSS HENA: 0xC400 + i*4 */
        else if (addr >= 0xC400 && addr < 0xC400 + 2 * 4) {
            uint32_t i = (addr - 0xC400) / 4;
            vf->vfqf_hena[i] = (uint32_t)val;
        }
        /* RSS HKEY: 0xCC00 + i*4 */
        else if (addr >= 0xCC00 && addr < 0xCC00 + 13 * 4) {
            uint32_t i = (addr - 0xCC00) / 4;
            vf->vfqf_hkey[i] = (uint32_t)val;
        }
        /* RSS HLUT: 0xD000 + i*4 */
        else if (addr >= 0xD000 && addr < 0xD000 + 16 * 4) {
            uint32_t i = (addr - 0xD000) / 4;
            vf->vfqf_hlut[i] = (uint32_t)val;
        }
        break;
    }
}

/*
 * VF AdminQ: Write a response descriptor + data to the ARQ ring.
 * The ARQ ring is managed by the PF (QEMU) as the writer: we write at
 * arq_head and advance it.  The driver (consumer) reads from its
 * next_to_clean index and posts consumed buffers back via ARQT1.
 */
static void ice_mp_vf_arq_post(ICEMPVFState *vf, uint32_t virtchnl_op,
                                int32_t virtchnl_status,
                                const void *payload, uint16_t payload_len)
{
    uint32_t arq_entries = vf->arq_len & 0x3FF;
    uint64_t arq_base = ((uint64_t)vf->arq_bah << 32) | vf->arq_bal;
    uint32_t head = vf->arq_head;
    struct iavf_aq_desc resp;
    PCIDevice *pci = &vf->parent_obj;

    if (!arq_entries || !(vf->arq_len & BIT(31))) {
        fprintf(stderr, "ice-mp-vf%u: ARQ not enabled, cannot post response\n",
                vf->vf_number);
        return;
    }

    /* Read the existing ARQ descriptor to get the data buffer address */
    uint64_t desc_addr = arq_base + (uint64_t)head * IAVF_AQ_DESC_SIZE;
    struct iavf_aq_desc existing;
    pci_dma_read(pci, desc_addr, &existing, sizeof(existing));
    uint64_t data_buf = ((uint64_t)le32_to_cpu(existing.addr_high) << 32) |
                        le32_to_cpu(existing.addr_low);

    /* Build response descriptor */
    memset(&resp, 0, sizeof(resp));
    resp.flags = cpu_to_le16(IAVF_AQ_FLAG_DD | IAVF_AQ_FLAG_CMP |
                             (payload_len ? (IAVF_AQ_FLAG_BUF |
                              (payload_len > 512 ? IAVF_AQ_FLAG_LB : 0)) : 0));
    resp.opcode = cpu_to_le16(IAVF_AQ_OPC_SEND_MSG_TO_VF);
    resp.datalen = cpu_to_le16(payload_len);
    resp.retval = cpu_to_le16(0);
    resp.cookie_high = cpu_to_le32(virtchnl_op);
    resp.cookie_low = cpu_to_le32((uint32_t)virtchnl_status);

    /* Write data buffer payload if present */
    if (payload && payload_len && data_buf) {
        resp.addr_high = existing.addr_high;
        resp.addr_low = existing.addr_low;
        pci_dma_write(pci, data_buf, payload, payload_len);
    }

    /* Write response descriptor to ARQ ring */
    pci_dma_write(pci, desc_addr, &resp, sizeof(resp));

    /* Advance ARQ head */
    vf->arq_head = (head + 1) % arq_entries;

    fprintf(stderr, "ice-mp-vf%u: ARQ posted op=%u status=%d datalen=%u head=%u->%u\n",
            vf->vf_number, virtchnl_op, virtchnl_status,
            payload_len, head, vf->arq_head);
}

/*
 * VF AdminQ: Process a single virtchnl message from the ASQ.
 */
static void ice_mp_vf_handle_virtchnl(ICEMPVFState *vf, uint32_t v_opcode,
                                       const uint8_t *msg_buf,
                                       uint16_t msg_len)
{
    fprintf(stderr, "ice-mp-vf%u: virtchnl op=%u datalen=%u\n",
            vf->vf_number, v_opcode, msg_len);

    switch (v_opcode) {
    case VIRTCHNL_OP_VERSION: {
        /* VF sends its version; we reply with PF version */
        struct {
            uint32_t major;
            uint32_t minor;
        } ver_reply = {
            .major = cpu_to_le32(1),
            .minor = cpu_to_le32(1),
        };
        vf->version_negotiated = true;
        ice_mp_vf_arq_post(vf, VIRTCHNL_OP_VERSION,
                            VIRTCHNL_STATUS_SUCCESS,
                            &ver_reply, sizeof(ver_reply));
        break;
    }

    case VIRTCHNL_OP_GET_VF_RESOURCES: {
        /*
         * Response: virtchnl_vf_resource (20 bytes) + one virtchnl_vsi_resource (16 bytes)
         * Total = 36 bytes
         */
        uint8_t resp_buf[36];
        memset(resp_buf, 0, sizeof(resp_buf));

        /* Assign VSI ID based on VF number */
        vf->vsi_id = 100 + vf->vf_number;

        /* virtchnl_vf_resource header (20 bytes) */
        /* num_vsis (u16) = 1 */
        resp_buf[0] = 1; resp_buf[1] = 0;
        /* num_queue_pairs (u16) = IAVF_VF_MAX_QUEUES */
        resp_buf[2] = IAVF_VF_MAX_QUEUES; resp_buf[3] = 0;
        /* max_vectors (u16) = IAVF_VF_MSIX_VECTORS */
        resp_buf[4] = IAVF_VF_MSIX_VECTORS; resp_buf[5] = 0;
        /* max_mtu (u16) = 9710 */
        resp_buf[6] = 0xEE; resp_buf[7] = 0x25;
        /* vf_cap_flags (u32) = L2 | RSS_PF | RX_POLLING */
        uint32_t caps = VIRTCHNL_VF_OFFLOAD_L2 |
                        VIRTCHNL_VF_OFFLOAD_RSS_PF |
                        VIRTCHNL_VF_OFFLOAD_RX_POLLING;
        memcpy(&resp_buf[8], &caps, 4);
        /* rss_key_size (u32) = 52 */
        uint32_t rss_key_size = cpu_to_le32(52);
        memcpy(&resp_buf[12], &rss_key_size, 4);
        /* rss_lut_size (u32) = 64 */
        uint32_t rss_lut_size = cpu_to_le32(64);
        memcpy(&resp_buf[16], &rss_lut_size, 4);

        /* virtchnl_vsi_resource[0] (16 bytes at offset 20) */
        /* vsi_id (u16) */
        uint16_t vsi_id = cpu_to_le16(vf->vsi_id);
        memcpy(&resp_buf[20], &vsi_id, 2);
        /* num_queue_pairs (u16) */
        resp_buf[22] = IAVF_VF_MAX_QUEUES; resp_buf[23] = 0;
        /* vsi_type (s32) = VIRTCHNL_VSI_SRIOV (6) */
        int32_t vsi_type = cpu_to_le32(VIRTCHNL_VSI_SRIOV);
        memcpy(&resp_buf[24], &vsi_type, 4);
        /* qset_handle (u16) = 0 */
        resp_buf[28] = 0; resp_buf[29] = 0;
        /* default_mac_addr[6] */
        memcpy(&resp_buf[30], vf->mac_addr, 6);

        vf->resources_configured = true;
        ice_mp_vf_arq_post(vf, VIRTCHNL_OP_GET_VF_RESOURCES,
                            VIRTCHNL_STATUS_SUCCESS,
                            resp_buf, sizeof(resp_buf));
        fprintf(stderr, "ice-mp-vf%u: GET_VF_RESOURCES reply: vsi=%u queues=%u "
                "mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                vf->vf_number, vf->vsi_id, IAVF_VF_MAX_QUEUES,
                vf->mac_addr[0], vf->mac_addr[1], vf->mac_addr[2],
                vf->mac_addr[3], vf->mac_addr[4], vf->mac_addr[5]);
        break;
    }

    case VIRTCHNL_OP_CONFIG_VSI_QUEUES: {
        /*
         * Parse queue configuration from the message.
         * Format: virtchnl_vsi_queue_config_info (8 bytes header)
         *   u16 vsi_id
         *   u16 num_queue_pairs
         *   u32 pad
         *   virtchnl_queue_pair_info[] qpair (64 bytes each)
         *     txq_info (24 bytes): vsi_id(2)+queue_id(2)+ring_len(u16@4)+
         *       headwb_enabled(2)+dma_ring_addr(u64@8)+dma_headwb_addr(8)
         *     rxq_info (40 bytes): vsi_id(2)+queue_id(2)+ring_len(u32@4)+
         *       hdr_size(2)+splithdr(2)+databuf_size(4)+max_pkt(4)+
         *       crc_disable(1)+rxdid(1)+flags(1)+pad(1)+dma_ring_addr(u64@24)+
         *       rx_split_pos(4)+pad2(4)
         */
        if (msg_len >= 8 && msg_buf) {
            uint16_t num_qps;
            memcpy(&num_qps, &msg_buf[2], 2);
            num_qps = le16_to_cpu(num_qps);
            fprintf(stderr, "ice-mp-vf%u: CONFIG_VSI_QUEUES num_qps=%u\n",
                    vf->vf_number, num_qps);

            /* Parse each queue pair info (offset 8 onward, 64 bytes each) */
            size_t offset = 8;
            for (uint16_t i = 0; i < num_qps && i < IAVF_VF_MAX_QUEUES; i++) {
                if (offset + 64 > msg_len) break;

                /* TX queue info (24 bytes at offset) */
                uint16_t txq_ring_len;
                uint64_t txq_base;
                memcpy(&txq_ring_len, &msg_buf[offset + 4], 2);  /* u16 ring_len @4 */
                txq_ring_len = le16_to_cpu(txq_ring_len);
                memcpy(&txq_base, &msg_buf[offset + 8], 8);      /* u64 dma_ring_addr @8 */
                txq_base = le64_to_cpu(txq_base);
                vf->txq[i].base = txq_base;
                vf->txq[i].qlen = txq_ring_len;
                vf->txq[i].head = 0;
                vf->txq[i].port_id = vf->vf_number % (vf->pf ? vf->pf->num_ports : 4);

                /* RX queue info (40 bytes at offset + 24) */
                uint32_t rxq_ring_len;
                uint64_t rxq_base;
                memcpy(&rxq_ring_len, &msg_buf[offset + 24 + 4], 4);  /* u32 ring_len @4 */
                rxq_ring_len = le32_to_cpu(rxq_ring_len);
                memcpy(&rxq_base, &msg_buf[offset + 24 + 24], 8);     /* u64 dma_ring_addr @24 */
                rxq_base = le64_to_cpu(rxq_base);
                vf->rxq[i].base = rxq_base;
                vf->rxq[i].qlen = rxq_ring_len;
                vf->rxq[i].head = 0;
                vf->rxq[i].port_id = vf->vf_number % (vf->pf ? vf->pf->num_ports : 4);

                fprintf(stderr, "ice-mp-vf%u: Q%u TX base=0x%lx len=%u, RX base=0x%lx len=%u\n",
                        vf->vf_number, i,
                        (unsigned long)txq_base, txq_ring_len,
                        (unsigned long)rxq_base, rxq_ring_len);

                offset += 64;
            }
        }
        ice_mp_vf_arq_post(vf, VIRTCHNL_OP_CONFIG_VSI_QUEUES,
                            VIRTCHNL_STATUS_SUCCESS, NULL, 0);
        break;
    }

    case VIRTCHNL_OP_CONFIG_IRQ_MAP: {
        /* Parse IRQ map: map queues to MSI-X vectors.
         * Format: virtchnl_irq_map_info
         *   u16 num_vectors
         *   virtchnl_vector_map[] vecmap
         *     u16 vsi_id, u16 vector_id, u16 rxq_map, u16 txq_map, u16 rxitr_idx, u16 txitr_idx
         */
        if (msg_len >= 2 && msg_buf) {
            uint16_t num_vectors;
            memcpy(&num_vectors, &msg_buf[0], 2);
            num_vectors = le16_to_cpu(num_vectors);

            size_t offset = 2; /* skip num_vectors (u16, no padding) */
            for (uint16_t v = 0; v < num_vectors; v++) {
                if (offset + 12 > msg_len) break;
                uint16_t vector_id;
                uint16_t rxq_map, txq_map;
                memcpy(&vector_id, &msg_buf[offset + 2], 2);
                memcpy(&rxq_map, &msg_buf[offset + 4], 2);
                memcpy(&txq_map, &msg_buf[offset + 6], 2);
                vector_id = le16_to_cpu(vector_id);
                rxq_map = le16_to_cpu(rxq_map);
                txq_map = le16_to_cpu(txq_map);

                /* Store vector mapping for queue interrupts */
                for (uint16_t q = 0; q < IAVF_VF_MAX_QUEUES; q++) {
                    if (rxq_map & (1 << q)) {
                        vf->rxq[q].int_ctl = vector_id | ICE_MP_QINT_CAUSE_ENA_M;
                    }
                    if (txq_map & (1 << q)) {
                        vf->txq[q].int_ctl = vector_id | ICE_MP_QINT_CAUSE_ENA_M;
                    }
                }

                fprintf(stderr, "ice-mp-vf%u: IRQ MAP vec=%u rxq_map=0x%x txq_map=0x%x\n",
                        vf->vf_number, vector_id, rxq_map, txq_map);
                offset += 12;
            }
        }
        ice_mp_vf_arq_post(vf, VIRTCHNL_OP_CONFIG_IRQ_MAP,
                            VIRTCHNL_STATUS_SUCCESS, NULL, 0);
        break;
    }

    case VIRTCHNL_OP_ENABLE_QUEUES: {
        /* Enable TX/RX queues.
         * Format: virtchnl_queue_select
         *   u16 vsi_id, u16 pad, u32 rx_queues, u32 tx_queues
         */
        if (msg_len >= 12 && msg_buf) {
            uint32_t rx_queues, tx_queues;
            memcpy(&rx_queues, &msg_buf[4], 4);
            memcpy(&tx_queues, &msg_buf[8], 4);
            rx_queues = le32_to_cpu(rx_queues);
            tx_queues = le32_to_cpu(tx_queues);

            for (uint32_t q = 0; q < IAVF_VF_MAX_QUEUES; q++) {
                if (tx_queues & (1 << q)) {
                    vf->txq[q].enabled = true;
                }
                if (rx_queues & (1 << q)) {
                    vf->rxq[q].enabled = true;
                }
            }
            vf->queues_enabled = true;
            fprintf(stderr, "ice-mp-vf%u: ENABLE_QUEUES rx=0x%x tx=0x%x\n",
                    vf->vf_number, rx_queues, tx_queues);
        }
        ice_mp_vf_arq_post(vf, VIRTCHNL_OP_ENABLE_QUEUES,
                            VIRTCHNL_STATUS_SUCCESS, NULL, 0);

        /* Send LINK_CHANGE event so iavf calls netif_carrier_on().
         * Without this, the driver keeps carrier OFF and blocks all TX.
         * Format: virtchnl_pf_event (16 bytes)
         *   s32 event           @ 0  = VIRTCHNL_EVENT_LINK_CHANGE (1)
         *   u32 link_speed      @ 4  = VIRTCHNL_LINK_SPEED_40GB (4)
         *   u8  link_status     @ 8  = 1 (UP)
         *   u8  pad[3]          @ 9  = 0
         *   s32 severity        @ 12 = 0
         */
        {
            uint8_t link_evt[16];
            memset(link_evt, 0, sizeof(link_evt));
            int32_t evt_code = cpu_to_le32(1);  /* LINK_CHANGE */
            memcpy(&link_evt[0], &evt_code, 4);
            uint32_t lspeed = cpu_to_le32(4);   /* 40GB */
            memcpy(&link_evt[4], &lspeed, 4);
            link_evt[8] = 1;                    /* link UP */
            ice_mp_vf_arq_post(vf, VIRTCHNL_OP_EVENT, 0,
                                link_evt, sizeof(link_evt));
            fprintf(stderr, "ice-mp-vf%u: Posted LINK_CHANGE event (link UP)\n",
                    vf->vf_number);
        }
        break;
    }

    case VIRTCHNL_OP_DISABLE_QUEUES: {
        if (msg_len >= 12 && msg_buf) {
            uint32_t rx_queues, tx_queues;
            memcpy(&rx_queues, &msg_buf[4], 4);
            memcpy(&tx_queues, &msg_buf[8], 4);
            rx_queues = le32_to_cpu(rx_queues);
            tx_queues = le32_to_cpu(tx_queues);

            for (uint32_t q = 0; q < IAVF_VF_MAX_QUEUES; q++) {
                if (tx_queues & (1 << q)) {
                    vf->txq[q].enabled = false;
                }
                if (rx_queues & (1 << q)) {
                    vf->rxq[q].enabled = false;
                }
            }
            vf->queues_enabled = false;
        }
        ice_mp_vf_arq_post(vf, VIRTCHNL_OP_DISABLE_QUEUES,
                            VIRTCHNL_STATUS_SUCCESS, NULL, 0);
        break;
    }

    case VIRTCHNL_OP_ADD_ETH_ADDR:
    case VIRTCHNL_OP_DEL_ETH_ADDR:
    case VIRTCHNL_OP_ADD_VLAN:
    case VIRTCHNL_OP_DEL_VLAN:
    case VIRTCHNL_OP_CONFIG_PROMISCUOUS:
    case VIRTCHNL_OP_CONFIG_RSS_KEY:
    case VIRTCHNL_OP_CONFIG_RSS_LUT:
        /* Accept these silently — no special handling in emulation */
        ice_mp_vf_arq_post(vf, v_opcode,
                            VIRTCHNL_STATUS_SUCCESS, NULL, 0);
        break;

    case VIRTCHNL_OP_GET_STATS: {
        /* Return all-zero stats (64 bytes) */
        uint8_t stats[48];
        memset(stats, 0, sizeof(stats));
        ice_mp_vf_arq_post(vf, VIRTCHNL_OP_GET_STATS,
                            VIRTCHNL_STATUS_SUCCESS,
                            stats, sizeof(stats));
        break;
    }

    case VIRTCHNL_OP_GET_OFFLOAD_VLAN_V2:
    case VIRTCHNL_OP_GET_SUPPORTED_RXDIDS:
        /* Not supported — tell the driver */
        ice_mp_vf_arq_post(vf, v_opcode,
                            VIRTCHNL_STATUS_NOT_SUPPORTED, NULL, 0);
        break;

    case 2: /* VIRTCHNL_OP_RESET_VF */
        /*
         * Reset the VF. The PF does NOT send a virtchnl response.
         * Instead, the VF polls VFGEN_RSTAT for reset completion.
         * After reset, RSTAT transitions: INPROGRESS(0) -> ACTIVE(2).
         * Note: do NOT reset atq_head/arq_head here — the process_atq
         * loop is still running. The driver will reset them via MMIO
         * writes (ATQH=0, ARQH=0) during AdminQ re-initialization.
         */
        fprintf(stderr, "ice-mp-vf%u: RESET_VF — resetting state\n",
                vf->vf_number);
        /* Clear queue state */
        for (int q = 0; q < IAVF_VF_MAX_QUEUES; q++) {
            vf->txq[q].enabled = false;
            vf->txq[q].head = 0;
            vf->txq[q].tail = 0;
            vf->rxq[q].enabled = false;
            vf->rxq[q].head = 0;
            vf->rxq[q].tail = 0;
        }
        vf->queues_enabled = false;
        vf->version_negotiated = false;
        vf->resources_configured = false;
        /* Set RSTAT to ACTIVE so the driver can proceed with re-init */
        vf->vfgen_rstat = 2; /* VIRTCHNL_VFR_VFACTIVE */
        /* Do NOT send ARQ response for RESET_VF */
        break;

    default:
        fprintf(stderr, "ice-mp-vf%u: Unhandled virtchnl op=%u\n",
                vf->vf_number, v_opcode);
        ice_mp_vf_arq_post(vf, v_opcode,
                            VIRTCHNL_STATUS_NOT_SUPPORTED, NULL, 0);
        break;
    }
}

/*
 * Process pending ASQ descriptors: called when ATQT1 is written.
 * Reads descriptors from the ASQ ring, processes virtchnl messages,
 * writes responses to the ARQ ring, and fires AdminQ interrupt.
 */
static void ice_mp_vf_process_atq(ICEMPVFState *vf)
{
    uint32_t atq_entries = vf->atq_len & 0x3FF;
    uint64_t atq_base = ((uint64_t)vf->atq_bah << 32) | vf->atq_bal;
    PCIDevice *pci = &vf->parent_obj;

    if (!atq_entries || !(vf->atq_len & BIT(31))) {
        return;
    }

    while (vf->atq_head != vf->atq_tail) {
        uint64_t desc_addr = atq_base + (uint64_t)vf->atq_head * IAVF_AQ_DESC_SIZE;
        struct iavf_aq_desc desc;

        pci_dma_read(pci, desc_addr, &desc, sizeof(desc));

        uint16_t opcode = le16_to_cpu(desc.opcode);
        uint16_t datalen = le16_to_cpu(desc.datalen);
        uint32_t v_opcode = le32_to_cpu(desc.cookie_high);
        uint16_t flags = le16_to_cpu(desc.flags);

        fprintf(stderr, "ice-mp-vf%u: ASQ desc head=%u opcode=0x%04x "
                "v_opcode=%u flags=0x%04x datalen=%u\n",
                vf->vf_number, vf->atq_head, opcode, v_opcode, flags, datalen);

        /* Read data buffer if present */
        uint8_t *msg_buf = NULL;
        uint16_t msg_len = 0;
        if ((flags & IAVF_AQ_FLAG_BUF) && datalen > 0) {
            uint64_t data_addr = ((uint64_t)le32_to_cpu(desc.addr_high) << 32) |
                                 le32_to_cpu(desc.addr_low);
            if (data_addr) {
                msg_buf = g_malloc(datalen);
                pci_dma_read(pci, data_addr, msg_buf, datalen);
                msg_len = datalen;
            }
        } else if (datalen > 0 && datalen <= 16) {
            /* Inline data in params field (starts at param0, offset 16) */
            msg_buf = g_malloc(16);
            memcpy(msg_buf, &desc.param0, 16);
            msg_len = datalen;
        }

        /* Process the virtchnl message */
        if (opcode == IAVF_AQ_OPC_SEND_MSG_TO_PF) {
            ice_mp_vf_handle_virtchnl(vf, v_opcode, msg_buf, msg_len);
        }

        /* Mark the ASQ descriptor as completed */
        desc.flags = cpu_to_le16(flags | IAVF_AQ_FLAG_DD | IAVF_AQ_FLAG_CMP);
        desc.retval = cpu_to_le16(0);
        pci_dma_write(pci, desc_addr, &desc, sizeof(desc));

        g_free(msg_buf);

        /* Advance ASQ head */
        vf->atq_head = (vf->atq_head + 1) % atq_entries;
    }

    /* Fire AdminQ interrupt (MSI-X vector 0) if enabled */
    if (msix_enabled(pci) && (vf->vfint_icr0_ena1 & BIT(30))) {
        vf->vfint_icr01 |= BIT(30);  /* Set AdminQ cause bit */
        fprintf(stderr, "ice-mp-vf%u: Firing AdminQ MSI-X interrupt (vec 0)\n",
                vf->vf_number);
        msix_notify(pci, 0);
    }
}

/*
 * VF TX datapath: Process TX descriptors and generate loopback responses.
 * Similar to PF TX processing but operates on VF queue state.
 */
static void ice_mp_vf_tx_loopback(ICEMPVFState *vf, uint8_t port_id,
                                   const uint8_t *pkt, size_t len);

static void ice_mp_vf_tx_process(ICEMPVFState *vf, uint16_t qid)
{
    if (qid >= IAVF_VF_MAX_QUEUES || !vf->pf) {
        return;
    }

    ICEMPVFQueue *q = &vf->txq[qid];
    uint16_t ring_size = q->qlen ? q->qlen : 256;
    uint16_t head = q->head;
    uint16_t tail = q->tail;
    PCIDevice *pci = &vf->parent_obj;
    ICEMPState *pf = vf->pf;

    if (!q->enabled || ring_size == 0 || head == tail) {
        return;
    }

    fprintf(stderr, "ice-mp-vf%u: TX processing qid=%u head=%u tail=%u\n",
            vf->vf_number, qid, head, tail);

    GByteArray *tx_pkt = g_byte_array_new();

    while (head != tail) {
        uint64_t desc_addr = q->base + ((uint64_t)head * ICE_MP_TX_DESC_SIZE);
        struct ice_mp_tx_desc desc;
        uint64_t qw1;
        uint16_t cmd;

        pci_dma_read(pci, desc_addr, &desc, sizeof(desc));
        qw1 = le64_to_cpu(desc.cmd_type_offset_bsz);
        cmd = (qw1 >> ICE_MP_TXD_QW1_CMD_S) & 0xFFF;
        uint8_t dtype = qw1 & 0xFULL;

        if (dtype == ICE_TX_DESC_DTYPE_CTX) {
            head = (head + 1) % ring_size;
            continue;
        }

        if (dtype == ICE_TX_DESC_DTYPE_DATA) {
            uint16_t len = (uint16_t)((qw1 >> ICE_MP_TXD_QW1_TX_BUF_SZ_S) & 0x3FFF);

            if (len) {
                uint8_t *buf = g_malloc(len);
                pci_dma_read(pci, le64_to_cpu(desc.buf_addr), buf, len);
                g_byte_array_append(tx_pkt, buf, len);
                g_free(buf);
            }

            if (cmd & ICE_TX_DESC_CMD_EOP) {
                if (tx_pkt->len) {
                    /* Send packet to the network backend (same port as PF) */
                    uint8_t port_id = q->port_id;
                    if (port_id < pf->num_ports && pf->nic[port_id]) {
                        NetClientState *nc = qemu_get_queue(pf->nic[port_id]);
                        qemu_send_packet(nc, tx_pkt->data, tx_pkt->len);
                    }
                    /* Generate loopback responses */
                    ice_mp_vf_tx_loopback(vf, q->port_id, tx_pkt->data, tx_pkt->len);
                }

                /* Write back descriptor done */
                desc.cmd_type_offset_bsz =
                    cpu_to_le64((qw1 & ~0xFULL) | ICE_TX_DESC_DTYPE_DESC_DONE);
                uint64_t wb_addr = desc_addr +
                    offsetof(struct ice_mp_tx_desc, cmd_type_offset_bsz);
                pci_dma_write(pci, wb_addr,
                              &desc.cmd_type_offset_bsz,
                              sizeof(desc.cmd_type_offset_bsz));

                /* Fire TX completion interrupt */
                if (msix_enabled(pci) && (q->int_ctl & ICE_MP_QINT_CAUSE_ENA_M)) {
                    uint16_t msix_idx = q->int_ctl & ICE_MP_QINT_MSIX_INDX_M;
                    if (msix_idx < IAVF_VF_MSIX_VECTORS) {
                        msix_notify(pci, msix_idx);
                    }
                }

                g_byte_array_set_size(tx_pkt, 0);
            }
        }

        head = (head + 1) % ring_size;
    }

    q->head = head;
    g_byte_array_free(tx_pkt, true);
}

/*
 * VF RX datapath: Enqueue a received packet into VF RX ring.
 */
static bool ice_mp_vf_rx_enqueue(ICEMPVFState *vf, uint16_t qid,
                                  const uint8_t *buf, size_t size)
{
    if (qid >= IAVF_VF_MAX_QUEUES) {
        return false;
    }

    ICEMPVFQueue *q = &vf->rxq[qid];
    uint16_t ring_size = q->qlen ? q->qlen : 256;
    PCIDevice *pci = &vf->parent_obj;

    if (!q->enabled || ring_size == 0 || q->head == q->tail) {
        return false;
    }

    /* Write packet data to RX descriptor buffer */
    uint64_t desc_addr = q->base + ((uint64_t)q->head * ICE_MP_RX_DESC_SIZE);
    union ice_mp_rx_desc desc;

    pci_dma_read(pci, desc_addr, &desc, sizeof(desc));

    if (!desc.read.pkt_addr) {
        return false;
    }

    uint32_t copy_len = (size > 2048) ? 2048 : (uint32_t)size;
    pci_dma_write(pci, le64_to_cpu(desc.read.pkt_addr), buf, copy_len);

    /* Write back RX descriptor in legacy i40e format (RXDID=1).
     * The iavf driver uses legacy 32-byte descriptors when the PF
     * does not advertise flex descriptor support (GET_SUPPORTED_RXDIDS).
     * Legacy format:
     *   QW0 (bytes 0-7):  RSS hash [63:32], L2TAG1 [33:16], mirror [15:0]
     *   QW1 (bytes 8-15): hdr_len [63:52], pkt_len [51:38], SPH [37:36],
     *                     htype [35:32], error [31:19], status [18:0]
     *                     DD=bit0, EOP=bit1
     */
    memset(&desc, 0, sizeof(desc));
    desc.read.pkt_addr = 0;  /* QW0: no RSS/L2TAG */
    uint64_t qw1 = ((uint64_t)copy_len << 38) | 0x3; /* DD=1, EOP=1 */
    desc.read.hdr_addr = cpu_to_le64(qw1);  /* QW1 at offset 8 */

    pci_dma_write(pci, desc_addr, &desc, sizeof(desc));

    q->head = (q->head + 1) % ring_size;

    /* Fire RX interrupt */
    if (msix_enabled(pci) && (q->int_ctl & ICE_MP_QINT_CAUSE_ENA_M)) {
        uint16_t msix_idx = q->int_ctl & ICE_MP_QINT_MSIX_INDX_M;
        if (msix_idx < IAVF_VF_MSIX_VECTORS) {
            msix_notify(pci, msix_idx);
        }
    }

    return true;
}

/*
 * VF TX loopback: Generate ARP replies and ICMP echo replies
 * so that ping works through VF interfaces.
 */
static void ice_mp_vf_tx_loopback(ICEMPVFState *vf, uint8_t port_id,
                                   const uint8_t *pkt_data, size_t len)
{
    if (len < 14) {
        return;
    }

    uint16_t ethertype = (pkt_data[12] << 8) | pkt_data[13];

    /* Handle ARP requests */
    if (ethertype == 0x0806 && len >= 42) {
        uint16_t oper = (pkt_data[20] << 8) | pkt_data[21];
        if (oper == 1) {  /* ARP Request */
            uint8_t reply[42];
            /* Use a unique peer MAC per VF: 52:54:00:vf:PP:QQ */
            uint8_t peer_mac[6] = {0x52, 0x54, 0x00, 0xaa,
                                   (uint8_t)(vf->vf_number + 0x10), port_id};

            memcpy(&reply[0], &pkt_data[6], 6);
            memcpy(&reply[6], peer_mac, 6);
            reply[12] = 0x08; reply[13] = 0x06;
            reply[14] = 0x00; reply[15] = 0x01;
            reply[16] = 0x08; reply[17] = 0x00;
            reply[18] = 6; reply[19] = 4;
            reply[20] = 0x00; reply[21] = 0x02;
            memcpy(&reply[22], peer_mac, 6);
            memcpy(&reply[28], &pkt_data[38], 4);
            memcpy(&reply[32], &pkt_data[6], 6);
            memcpy(&reply[38], &pkt_data[28], 4);

            fprintf(stderr, "ice-mp-vf%u: Loopback ARP reply port=%u target=%u.%u.%u.%u\n",
                    vf->vf_number, port_id,
                    pkt_data[38], pkt_data[39], pkt_data[40], pkt_data[41]);

            /* Try to enqueue on first enabled RX queue */
            for (uint16_t q = 0; q < IAVF_VF_MAX_QUEUES; q++) {
                if (vf->rxq[q].enabled) {
                    ice_mp_vf_rx_enqueue(vf, q, reply, 42);
                    break;
                }
            }
        }
    }
    /* Handle ICMP echo requests */
    else if (ethertype == 0x0800 && len >= 34) {
        uint8_t ihl = (pkt_data[14] & 0x0F) * 4;
        uint8_t proto = pkt_data[23];
        size_t icmp_off = 14 + ihl;

        if (proto == 1 && len >= icmp_off + 8) {
            uint8_t icmp_type = pkt_data[icmp_off];
            if (icmp_type == 8) {  /* Echo Request */
                uint8_t *reply = g_malloc(len);
                memcpy(reply, pkt_data, len);

                uint8_t peer_mac[6] = {0x52, 0x54, 0x00, 0xaa,
                                       (uint8_t)(vf->vf_number + 0x10), port_id};

                /* Swap MACs */
                memcpy(&reply[0], &pkt_data[6], 6);
                memcpy(&reply[6], peer_mac, 6);
                /* Swap IPs */
                memcpy(&reply[26], &pkt_data[30], 4);
                memcpy(&reply[30], &pkt_data[26], 4);
                /* Set TTL */
                reply[22] = 64;

                /* Recalculate IP header checksum */
                reply[24] = 0; reply[25] = 0;
                uint32_t ip_sum = 0;
                for (int i = 14; i < 14 + ihl; i += 2) {
                    ip_sum += (reply[i] << 8) | reply[i + 1];
                }
                while (ip_sum >> 16) {
                    ip_sum = (ip_sum & 0xFFFF) + (ip_sum >> 16);
                }
                uint16_t ip_cksum = ~ip_sum & 0xFFFF;
                reply[24] = ip_cksum >> 8;
                reply[25] = ip_cksum & 0xFF;

                /* ICMP echo reply */
                reply[icmp_off] = 0;
                reply[icmp_off + 2] = 0; reply[icmp_off + 3] = 0;
                uint32_t icmp_sum = 0;
                size_t icmp_len = len - icmp_off;
                for (size_t i = 0; i < icmp_len; i += 2) {
                    uint16_t word = reply[icmp_off + i] << 8;
                    if (i + 1 < icmp_len) {
                        word |= reply[icmp_off + i + 1];
                    }
                    icmp_sum += word;
                }
                while (icmp_sum >> 16) {
                    icmp_sum = (icmp_sum & 0xFFFF) + (icmp_sum >> 16);
                }
                uint16_t icmp_cksum = ~icmp_sum & 0xFFFF;
                reply[icmp_off + 2] = icmp_cksum >> 8;
                reply[icmp_off + 3] = icmp_cksum & 0xFF;

                fprintf(stderr, "ice-mp-vf%u: Loopback ICMP reply port=%u len=%zu\n",
                        vf->vf_number, port_id, len);

                for (uint16_t q = 0; q < IAVF_VF_MAX_QUEUES; q++) {
                    if (vf->rxq[q].enabled) {
                        ice_mp_vf_rx_enqueue(vf, q, reply, len);
                        break;
                    }
                }
                g_free(reply);
            }
        }
    }
}

static const MemoryRegionOps ice_mp_vf_mmio_ops = {
    .read = ice_mp_vf_mmio_read,
    .write = ice_mp_vf_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ice_mp_vf_realize(PCIDevice *pci_dev, Error **errp)
{
    ICEMPVFState *vf = ICE_MP_VF(pci_dev);
    uint8_t *pci_conf = pci_dev->config;
    int ret;

    /* Initialize PCI config space for VF */
    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(pci_conf, ICE_MP_VF_DEV_ID);
    pci_config_set_class(pci_conf, PCI_CLASS_NETWORK_ETHERNET);
    pci_config_set_revision(pci_conf, 0x01);

    /* Enable memory space access */
    pci_set_word(pci_conf + PCI_COMMAND, PCI_COMMAND_MEMORY);

    /* Find parent PF device */
    PCIDevice *pf_dev = pcie_sriov_get_pf(pci_dev);
    if (pf_dev) {
        vf->pf = ICE_MP(pf_dev);
        vf->vf_number = pcie_sriov_vf_number(pci_dev);
    }

    fprintf(stderr, "ice-mp-vf: realize() - vf_number=%u devfn=0x%x pf=%p\n",
            vf->vf_number, pci_dev->devfn, vf->pf);

    if (pcie_endpoint_cap_init(pci_dev, 0x80) < 0) {
        error_setg(errp, "Failed to initialize PCIe capability (VF)");
        return;
    }

    /* Initialize MSI-X for VF (needed for AdminQ and queue interrupts) */
    memory_region_init(&vf->msix_bar, OBJECT(vf), "ice-mp-vf-msix", 0x4000);
    pcie_sriov_vf_register_bar(pci_dev, 3, &vf->msix_bar);

    ret = msix_init(pci_dev, IAVF_VF_MSIX_VECTORS,
                    &vf->msix_bar, 3, 0,
                    &vf->msix_bar, 3, 0x1000,
                    0x70, errp);
    if (ret < 0) {
        error_setg(errp, "Failed to initialize MSI-X for VF%u", vf->vf_number);
        return;
    }

    /* Mark all MSI-X vectors as used */
    for (int i = 0; i < IAVF_VF_MSIX_VECTORS; i++) {
        msix_vector_use(pci_dev, i);
    }

    /* Initialize BAR0 for VF MMIO registers */
    memory_region_init_io(&vf->bar0, OBJECT(vf), &ice_mp_vf_mmio_ops, vf,
                         "ice-mp-vf-mmio", IAVF_VF_BAR0_SIZE);
    pcie_sriov_vf_register_bar(pci_dev, 0, &vf->bar0);

    /* Initialize VF state */
    vf->vfgen_rstat = 2;  /* VFACTIVE - VF is ready */
    vf->version_negotiated = false;
    vf->resources_configured = false;
    vf->queues_enabled = false;

    /* Assign a unique MAC address per VF: 52:54:00:12:VF:00 */
    vf->mac_addr[0] = 0x52;
    vf->mac_addr[1] = 0x54;
    vf->mac_addr[2] = 0x00;
    vf->mac_addr[3] = 0x12;
    vf->mac_addr[4] = vf->vf_number + 1;
    vf->mac_addr[5] = 0x00;

    /* Zero out all queue state */
    memset(vf->txq, 0, sizeof(vf->txq));
    memset(vf->rxq, 0, sizeof(vf->rxq));

    fprintf(stderr, "ice-mp-vf%u: realized, RSTAT=%u, MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
            vf->vf_number, vf->vfgen_rstat,
            vf->mac_addr[0], vf->mac_addr[1], vf->mac_addr[2],
            vf->mac_addr[3], vf->mac_addr[4], vf->mac_addr[5]);
}

static void ice_mp_vf_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = ice_mp_vf_realize;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = ICE_MP_VF_DEV_ID;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    dc->desc = "Intel E810 Multi-Port VF Test Device";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo ice_mp_info = {
    .name          = TYPE_ICE_MP,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ICEMPState),
    .class_init    = ice_mp_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static const TypeInfo ice_mp_vf_info = {
    .name          = TYPE_ICE_MP_VF,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ICEMPVFState),
    .class_init    = ice_mp_vf_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void ice_mp_register_types(void)
{
    type_register_static(&ice_mp_info);
    type_register_static(&ice_mp_vf_info);
}

type_init(ice_mp_register_types)
