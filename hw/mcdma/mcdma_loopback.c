#include "qemu/osdep.h"
#include "exec/memory.h"
#include "hw/pci/msix.h"
#ifdef CONFIG_TRACE_MCDMA
#include "trace.h"
#else
#define trace_mcdma_loopback(rx_qid, tx_qid, len, src, dest) ((void)0)
#define trace_mcdma_loopback_stall(rx_qid, tx_qid)           ((void)0)
#define trace_mcdma_loopback_done(rx_qid, tx_qid, didx)      ((void)0)
#define trace_mcdma_msix_notify(vector)                      ((void)0)
#define trace_mcdma_desc_writeback(qid, dir, addr, head)     ((void)0)
#endif
#include "mcdma.h"
#include "mcdma_regs.h"

void mcdma_loopback_rx_to_tx(McdmaState *s, McdmaQueue *rxq, McdmaQueue *txq,
                              hwaddr rx_src, hwaddr tx_dest, uint32_t len,
                              bool msix_en, bool wb_en, uint16_t didx)
{
    uint8_t buf[2048];
    uint32_t xfer_len = len;

    if (xfer_len > sizeof(buf)) {
        xfer_len = sizeof(buf);
    }

    trace_mcdma_loopback(rxq->qid, txq->qid, xfer_len, rx_src, tx_dest);

    /* Read data from RX descriptor source (guest memory) */
    address_space_read(&address_space_memory, rx_src,
                       MEMTXATTRS_UNSPECIFIED, buf, xfer_len);

    /* Write data to TX descriptor destination (guest memory) */
    address_space_write(&address_space_memory, tx_dest,
                        MEMTXATTRS_UNSPECIFIED, buf, xfer_len);

    /* Update TX queue state */
    txq->bytes_copied += xfer_len;
    rxq->bytes_copied += xfer_len;

    /* Write-back completion for TX queue */
    if (wb_en && (txq->ctrl & Q_CTRL_Q_WB_EN)) {
        mcdma_descq_writeback(s, txq);
    }

    /* MSI-X interrupt */
    if (msix_en && (txq->ctrl & Q_CTRL_Q_INTR_EN)) {
        if (s->msix_initialized) {
            uint32_t vector = txq->qid % s->nr_vectors;
            trace_mcdma_msix_notify(vector);
            msix_notify(&s->parent_obj, vector);
        }
    }

    trace_mcdma_loopback_done(rxq->qid, txq->qid, didx);
}
