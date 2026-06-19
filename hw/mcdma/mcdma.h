#ifndef MCDMA_H
#define MCDMA_H

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "qemu/queue.h"
#include "mcdma_regs.h"

#define TYPE_MCDMA "mcdma"
OBJECT_DECLARE_SIMPLE_TYPE(McdmaState, MCDMA)

typedef struct McdmaQueue {
    uint32_t qid;
    McdmaDir dir;

    /* CSR register state */
    uint32_t ctrl;
    uint64_t ring_base;
    uint32_t ring_size_log2;
    uint32_t ring_entries;
    uint32_t head;
    uint32_t tail;
    uint32_t completed;
    uint64_t wb_addr;

    /* Dynamic state */
    bool enabled;
    bool running;

    /* Debug stats */
    uint64_t desc_processed;
    uint64_t bytes_copied;
    uint64_t invalid_skipped;
    uint64_t backpressure_stalls;
} McdmaQueue;

typedef struct McdmaState {
    PCIDevice parent_obj;

    MemoryRegion bar0;
    MemoryRegion bar2;
    MemoryRegion bar4;

    uint32_t num_queues;
    McdmaQueue *rx_queues;
    McdmaQueue *tx_queues;

    QEMUBH *bh;
    bool bh_scheduled;

    /* MSI-X state */
    uint32_t nr_vectors;
    bool msix_initialized;
} McdmaState;

/* mcdma.c */
void mcdma_process_descq(McdmaState *s);

/* mcdma_descq.c */
void mcdma_descq_process_loopback(McdmaState *s, McdmaQueue *rxq, McdmaQueue *txq);
void mcdma_descq_process_tx(McdmaState *s, McdmaQueue *txq);
void mcdma_descq_process_rx(McdmaState *s, McdmaQueue *rxq);
void mcdma_descq_reset(McdmaQueue *q);
void mcdma_descq_writeback(McdmaState *s, McdmaQueue *q);

/* mcdma_loopback.c */
void mcdma_loopback_rx_to_tx(McdmaState *s, McdmaQueue *rxq, McdmaQueue *txq,
                              hwaddr rx_src, hwaddr tx_dest, uint32_t len,
                              bool msix_en, bool wb_en, uint16_t didx);

#endif /* MCDMA_H */
