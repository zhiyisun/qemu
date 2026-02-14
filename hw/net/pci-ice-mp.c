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
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qom/object.h"

/* PCI Configuration */
#define PCI_VENDOR_ID_INTEL     0x8086
#define PCI_DEVICE_ID_ICE_MP    0xFFFF

/* BAR0 Size (4MB) */
#define ICE_MP_BAR0_SIZE        0x00800000  /* 8MB MMIO space */

/* BAR0 Register Offsets */
#define ICE_MP_REG_CAPS         0x0000
#define ICE_MP_REG_PORT_COUNT   0x0004
#define ICE_MP_REG_PORT_STATUS  0x0010  /* Array: 0x10 + port_id * 4 */
#define ICE_MP_REG_EVENT_DB     0x0100
#define ICE_MP_REG_VF_PORT_MAP  0x0200  /* Array: 0x200 + vf_id */
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
#define ICE_MP_MSIX_VECTORS     64

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
};

struct ice_mp_aq_desc {
    uint16_t flags;
    uint16_t opcode;
    uint16_t datalen;
    uint16_t retval;
    uint32_t cookie_high;
    uint32_t cookie_low;
    uint8_t params[16];
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
static void ice_mp_mailbox_process(struct ICEMPState *s);

static uint64_t ice_mp_dma_addr(uint32_t low, uint32_t high)
{
    return ((uint64_t)high << 32) | low;
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
    
    desc->flags |= cpu_to_le16(ICE_MP_AQ_FLAG_DD | ICE_MP_AQ_FLAG_CMP);
    
    /* Debug for 0x0401 */
    if (opcode == 0x0401) {
        fprintf(stderr, "ice-mp: complete() AFTER flags: flags=0x%04x retval=0x%04x\n",
                le16_to_cpu(desc->flags), le16_to_cpu(desc->retval));
    }
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
    
    if (addr) {
        /* Read VSI context from buffer and write it back */
        uint8_t vsi_ctx[128];  /* ice_aqc_vsi_props structure */
        pci_dma_read(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
        pci_dma_write(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
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
    /* Just acknowledge the free request */
    ice_mp_adminq_complete(s, desc);
}

/* Get Link Status (indirect 0x0607) */
static void ice_mp_adminq_get_link_status(struct ICEMPState *s, struct ice_mp_aq_desc *desc)
{
    struct {
        uint8_t topo_media_conflict;
        uint8_t link_cfg_err;
        uint8_t link_info;
        uint8_t an_info;
        uint8_t ext_info;
        uint8_t loopback;
        uint16_t max_frame_size;
        uint8_t cfg;
        uint8_t pwr_desc;
        uint16_t link_speed;
        uint16_t reserved1;
        uint8_t reserved2[2];
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
    
    /* Populate link status from port state */
    link_status.topo_media_conflict = 0;
    link_status.link_cfg_err = 0;
    
    /* Bit 0: Link up, Bit 2: Link up via signal detection */
    link_status.link_info = link_up ? 0x01 : 0x00;
    
    /* Auto-negotiation - assume disabled for 100G */
    link_status.an_info = 0;
    link_status.ext_info = 0;
    link_status.loopback = 0;
    link_status.max_frame_size = cpu_to_le16(9728);  /* Standard jumbo frame */
    
    /* Full duplex when link is up */
    link_status.cfg = link_up ? 0x01 : 0x00;  /* Bit 0 = full duplex */
    link_status.pwr_desc = 0;
    
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
    addr = ((uint64_t)le32_to_cpu(desc->params[2]) << 32) |
           le32_to_cpu(desc->params[3]);
    
    if (addr) {
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
        fprintf(stderr, "ice-mp:   Deleting TEID 0x%08x\n", le32_to_cpu(teids[i]));
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
    
    /* Generate TEIDs for each element */
    for (i = 0; i < num_elems; i++) {
        elems[i].node_teid = cpu_to_le32(s->next_sched_teid++);
        fprintf(stderr, "ice-mp: Add Sched Elem: elem[%d] TEID=0x%x type=%d parent=0x%x\n",
                i, s->next_sched_teid - 1, elems[i].data.elem_type,
                le32_to_cpu(elems[i].parent_teid));
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
     * Input: buffer with VSI parameters (ice_aqc_vsi_props structure)
     * Output: VSI info in descriptor params (ice_aqc_add_update_free_vsi_resp)
     */
    uint64_t addr = ice_mp_dma_addr(le32_to_cpu(*(uint32_t *)&desc->params[12]),
                                    le32_to_cpu(*(uint32_t *)&desc->params[8]));
    
    fprintf(stderr, "ice-mp: Add VSI (0x0210) called, DMA addr=0x%lx, datalen=%d\n",
            addr, le16_to_cpu(desc->datalen));
    
    if (addr) {
        /* Read VSI context from buffer */
        uint8_t vsi_ctx[128];  /* ice_aqc_vsi_props structure */
        pci_dma_read(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
        
        /* Write back the VSI context - most fields are set by driver */
        pci_dma_write(&s->parent_obj, addr, vsi_ctx, sizeof(vsi_ctx));
        
        fprintf(stderr, "ice-mp: Add VSI - wrote back VSI context (%zu bytes)\n",
                sizeof(vsi_ctx));
    }
    
    /* Fill response in descriptor params (ice_aqc_add_update_free_vsi_resp) */
    uint16_t vsi_num = s->next_vsi_num++;
    uint16_t vsi_used = vsi_num;
    uint16_t vsi_free = (vsi_used < 255) ? (uint16_t)(255 - vsi_used) : 0;

    *(uint16_t *)&desc->params[0] = cpu_to_le16(vsi_num);
    *(uint16_t *)&desc->params[2] = cpu_to_le16(0);    /* ext_status = 0 */
    *(uint16_t *)&desc->params[4] = cpu_to_le16(vsi_used);
    *(uint16_t *)&desc->params[6] = cpu_to_le16(vsi_free);
    /* params[8-15] already contain addr_high/addr_low from request */
    
    /* Set retval to 0 for success */
    desc->retval = cpu_to_le16(0);
    
        fprintf(stderr, "ice-mp: Add VSI response: vsi_num=%u, vsi_used=%u, vsi_free=%u\n",
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
    } else if (node_teid > 0x16000002 && node_teid < 0x16000008) {
        /* SE_GENERIC (Queue Groups) */
        elem.parent_teid = cpu_to_le32(node_teid - 1);
        elem.data.elem_type = ICE_AQC_ELEM_TYPE_SE_GENERIC;
    } else if (node_teid == 0x16000008) {
        /* Leaf (Queue) */
        elem.parent_teid = cpu_to_le32(0x16000007);
        elem.data.elem_type = ICE_AQC_ELEM_TYPE_LEAF;
    } else {
        /* Unknown TEID - return generic SE */
        elem.parent_teid = cpu_to_le32(0x16000000);
        elem.data.elem_type = ICE_AQC_ELEM_TYPE_SE_GENERIC;
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
            ice_mp_adminq_complete(s, &desc);
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
            fprintf(stderr, "ice-mp: Switch Rules opcode 0x%04x - returning success\n",
                    le16_to_cpu(desc.opcode));
            ice_mp_adminq_complete(s, &desc);
            break;
        case 0x0C30:  /* Add Tx LAN Queues */
        case 0x0C31:  /* Disable Tx LAN Queues */
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
typedef struct ICEMPVFState {
    PCIDevice parent_obj;
    MemoryRegion bar0;
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
        /* Port status registers */
        if (addr >= ICE_MP_REG_PORT_STATUS && 
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
        /* Port status - allow external control for testing */
        if (addr >= ICE_MP_REG_PORT_STATUS && 
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
    memory_region_init(&s->bar1, OBJECT(s), "ice-mp-msix", 0x2000);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar1);
    
    ret = msix_init(pci_dev, ICE_MP_MSIX_VECTORS,
                    &s->bar1, s->msix_bar_idx, 0,
                    &s->bar1, s->msix_bar_idx, 0x1000,
                    0x70, errp);
    if (ret < 0) {
        return;
    }

    /* Initialize SR-IOV if VFs are configured */
    if (s->num_vfs > 0) {
        uint32_t init_vfs = s->num_vfs;
        fprintf(stderr, "ice-mp: SR-IOV total_vfs=%u initial_vfs=%u\n",
                s->num_vfs, init_vfs);
        pcie_sriov_pf_init(pci_dev, ICE_MP_SRIOV_OFFSET, "pci-ice-mp-vf",
                  ICE_MP_VF_DEV_ID, init_vfs, s->num_vfs,
                  ICE_MP_VF_OFFSET, ICE_MP_VF_STRIDE);
        pcie_sriov_pf_init_vf_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY,
                                  0x1000);
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
    
    /* Initialize all ports as link down, 25G speed */
    for (uint32_t i = 0; i < s->num_ports; i++) {
        s->port_status[i] = (ICE_MP_SPEED_25G << ICE_MP_PORT_SPEED_SHIFT);
    }
    
    /* Initialize VF-to-port mapping (round-robin) */
    for (uint32_t i = 0; i < s->num_vfs; i++) {
        s->vf_port_map[i] = i % s->num_ports;
    }
    
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
}

/* Device cleanup */
static void ice_mp_exit(PCIDevice *pci_dev)
{
    ICEMPState *s = ICE_MP(pci_dev);
    
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

    for (uint32_t i = 0; i < s->num_vfs; i++) {
        s->vf_mbx_atqlen[i] = 0;
        s->vf_mbx_arqlen[i] = 0;
        s->vfgen_rstat[i] = 0;
        s->vpgen_vfrtrig[i] = 0;
        s->vpgen_vfrstat[i] = ICE_MP_VFRSTAT_VFRD;
    }
    s->pf_pci_ciaa = 0;
    s->pf_pci_ciad = 0;

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
    return 0;
}

static void ice_mp_vf_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    (void)opaque;
    (void)addr;
    (void)val;
    (void)size;
}

static void ice_mp_vf_realize(PCIDevice *pci_dev, Error **errp)
{
    ICEMPVFState *vf = ICE_MP_VF(pci_dev);
    static const MemoryRegionOps vf_mmio_ops = {
        .read = ice_mp_vf_mmio_read,
        .write = ice_mp_vf_mmio_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .impl = {
            .min_access_size = 4,
            .max_access_size = 4,
        },
    };

    if (pcie_endpoint_cap_init(pci_dev, 0x80) < 0) {
        error_setg(errp, "Failed to initialize PCIe capability (VF)");
        return;
    }

    memory_region_init_io(&vf->bar0, OBJECT(vf), &vf_mmio_ops, vf,
                         "ice-mp-vf-mmio", 0x1000);
    pcie_sriov_vf_register_bar(pci_dev, 0, &vf->bar0);
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
