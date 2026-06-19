#ifndef MCDMA_REGS_H
#define MCDMA_REGS_H

#include "qemu/osdep.h"

/* PCI IDs */
#define MCDMA_VENDOR_ID         0x1172
#define MCDMA_DEVICE_ID         0x0000
#define MCDMA_REVISION          0x01
#define MCDMA_CLASS_CODE        0x0200  /* base=0x02 (Network) << 8 | sub=0x00 (Ethernet) */

/* BAR indices */
#define MCDMA_CONFIG_BAR        0
#define MCDMA_PIO_BAR           2
#define MCDMA_BAS_BAR           4

/* BAR sizes */
#define MCDMA_BAR0_SIZE         0x400000  /* 4MB: QCSR + global + DCA */
#define MCDMA_BAR2_SIZE         0x10000   /* 64KB: PIO */
#define MCDMA_BAR4_SIZE         0x10000   /* 64KB: BAS */

/* Queue CSR layout */
#define MCDMA_QUEUE_CSR_SIZE    0x100     /* 256 bytes per queue */
#define MCDMA_TX_Q_OFFSET       0x80000   /* TX queues start here */
#define MCDMA_GLOBAL_OFFSET     0x200000  /* Global registers */
#define MCDMA_MAX_QUEUES        8

/* Per-queue register offsets (within 256-byte CSR block) */
#define Q_CTRL                  0x00
#define Q_CTRL_Q_EN             (1u << 0)
#define Q_CTRL_Q_WB_EN          (1u << 8)
#define Q_CTRL_Q_INTR_EN        (1u << 9)

#define Q_START_ADDR_L          0x08
#define Q_START_ADDR_H          0x0C
#define Q_SIZE                  0x10
#define Q_SIZE_MASK             0x1F

#define Q_TAIL_POINTER          0x14
#define Q_TAIL_PTR_MASK         0xFFFF

#define Q_HEAD_POINTER          0x18
#define Q_HEAD_PTR_MASK         0xFFFF

#define Q_COMPLETED_POINTER     0x1C
#define Q_COMPLETED_PTR_MASK    0xFFFF

#define Q_CONSUMED_HEAD_ADDR_L  0x20
#define Q_CONSUMED_HEAD_ADDR_H  0x24

#define Q_RESET                 0x48
#define Q_RESET_MASK            0x01

#define Q_CPL_TIMEOUT           0x4C

/* Global register offsets (BAR0 + MCDMA_GLOBAL_OFFSET) */
#define GLB_VERSION             0x200070
#define GLB_LINK_STATUS         0x200080
#define GLB_LINK_STATUS_UP      0x01

/* Expected version value */
#define MCDMA_RTL_VERSION       0x20210601

/* Descriptor format (32 bytes) */
typedef struct {
    uint64_t src;
    uint64_t dest;
    uint32_t len : 20;
    uint32_t rsvd1 : 12;
    uint32_t didx : 16;
    uint32_t msix_en : 1;
    uint32_t wb_en : 1;
    uint32_t rsvd2 : 14;
    uint32_t rx_pyld_cnt : 20;
    uint32_t rsvd3 : 10;
    uint32_t sof : 1;
    uint32_t eof : 1;
    uint32_t rsvd4 : 28;
    uint32_t pad_len : 2;
    uint32_t desc_invalid : 1;
    uint32_t link : 1;
} QEMU_PACKED McdmaDesc;

/* Verify descriptor size */
QEMU_BUILD_BUG_ON(sizeof(McdmaDesc) != 32);

/* Queue direction */
typedef enum {
    MCDMA_DIR_H2D = 0,  /* Host to Device (RX) */
    MCDMA_DIR_D2H = 1,  /* Device to Host (TX) */
} McdmaDir;

/* Maximum transfer size per descriptor */
#define MCDMA_MAX_XFER_SIZE     (1 * 1024 * 1024)  /* 1MB */

#endif /* MCDMA_REGS_H */
