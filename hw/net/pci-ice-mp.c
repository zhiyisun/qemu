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

/* BAR0 Size (4KB) */
#define ICE_MP_BAR0_SIZE        0x1000

/* BAR0 Register Offsets */
#define ICE_MP_REG_CAPS         0x0000
#define ICE_MP_REG_PORT_COUNT   0x0004
#define ICE_MP_REG_PORT_STATUS  0x0010  /* Array: 0x10 + port_id * 4 */
#define ICE_MP_REG_EVENT_DB     0x0100
#define ICE_MP_REG_VF_PORT_MAP  0x0200  /* Array: 0x200 + vf_id */

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

/* Maximum ports and VFs */
#define ICE_MP_MAX_PORTS        16
#define ICE_MP_MAX_VFS          256
#define ICE_MP_MSIX_VECTORS     16

/* SR-IOV Configuration */
#define ICE_MP_SRIOV_OFFSET     0x160
#define ICE_MP_VF_OFFSET        0x10
#define ICE_MP_VF_STRIDE        0x01
#define ICE_MP_VF_DEV_ID        0xFFFE

/* Device State */
struct ICEMPState {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    MemoryRegion bar0;
    
    /* Configuration */
    uint32_t num_ports;
    uint32_t num_vfs;
    
    /* BAR0 Registers */
    uint32_t caps;
    uint32_t port_count;
    uint32_t port_status[ICE_MP_MAX_PORTS];
    uint32_t event_doorbell;
    uint8_t vf_port_map[ICE_MP_MAX_VFS];
    
    /* MSI-X */
    uint8_t msix_bar_idx;
};

#define TYPE_ICE_MP "pci-ice-mp"
OBJECT_DECLARE_SIMPLE_TYPE(ICEMPState, ICE_MP)

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
    
    default:
        /* Port status registers */
        if (addr >= ICE_MP_REG_PORT_STATUS && 
            addr < ICE_MP_REG_PORT_STATUS + (ICE_MP_MAX_PORTS * 4)) {
            uint32_t port_id = (addr - ICE_MP_REG_PORT_STATUS) / 4;
            if (port_id < s->num_ports) {
                val = s->port_status[port_id];
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
    
    default:
        /* Port status - allow external control for testing */
        if (addr >= ICE_MP_REG_PORT_STATUS && 
            addr < ICE_MP_REG_PORT_STATUS + (ICE_MP_MAX_PORTS * 4)) {
            uint32_t port_id = (addr - ICE_MP_REG_PORT_STATUS) / 4;
            if (port_id < s->num_ports) {
                s->port_status[port_id] = val;
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
    if (pcie_endpoint_cap_init(pci_dev, 0) < 0) {
        error_setg(errp, "Failed to initialize PCIe capability");
        return;
    }

    /* Initialize PCI config space */
    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(pci_conf, PCI_DEVICE_ID_ICE_MP);
    pci_config_set_class(pci_conf, PCI_CLASS_NETWORK_ETHERNET);
    pci_config_set_revision(pci_conf, 0x01);

    /* Initialize MSI-X */
    s->msix_bar_idx = 1;
    ret = msix_init(pci_dev, ICE_MP_MSIX_VECTORS,
                    &s->bar0, s->msix_bar_idx, 0,
                    &s->bar0, s->msix_bar_idx, 0x1000,
                    0, errp);
    if (ret < 0) {
        return;
    }

    /* Initialize BAR0 for MMIO */
    memory_region_init_io(&s->bar0, OBJECT(s), &ice_mp_mmio_ops, s,
                         "ice-mp-mmio", ICE_MP_BAR0_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);

    /* Initialize SR-IOV if VFs are configured */
    if (s->num_vfs > 0) {
        pcie_sriov_pf_init(pci_dev, ICE_MP_SRIOV_OFFSET, "pci-ice-mp-vf",
                          ICE_MP_VF_DEV_ID, s->num_vfs, s->num_vfs,
                          ICE_MP_VF_OFFSET, ICE_MP_VF_STRIDE);
        
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
}

/* Device cleanup */
static void ice_mp_exit(PCIDevice *pci_dev)
{
    ICEMPState *s = ICE_MP(pci_dev);
    
    if (s->num_vfs > 0) {
        pcie_sriov_pf_exit(pci_dev);
    }
    
    msix_uninit(pci_dev, &s->bar0, &s->bar0);
}

/* Device reset */
static void ice_mp_reset_hold(Object *obj, ResetType type)
{
    ICEMPState *s = ICE_MP(obj);
    
    /* Reset all ports to link down */
    for (uint32_t i = 0; i < s->num_ports; i++) {
        s->port_status[i] = (ICE_MP_SPEED_25G << ICE_MP_PORT_SPEED_SHIFT);
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

static void ice_mp_register_types(void)
{
    type_register_static(&ice_mp_info);
}

type_init(ice_mp_register_types)
