#include "qemu/osdep.h"
#include "exec/memory.h"
#ifdef CONFIG_TRACE_MCDMA
#include "trace.h"
#else
#define trace_mcdma_desc_fetch(qid, dir, addr, head, tail)   ((void)0)
#define trace_mcdma_desc_complete(qid, dir, didx, len)       ((void)0)
#define trace_mcdma_desc_writeback(qid, dir, addr, head)     ((void)0)
#define trace_mcdma_desc_invalid(qid, dir, idx)              ((void)0)
#define trace_mcdma_desc_link(qid, dir)                      ((void)0)
#define trace_mcdma_loopback_stall(rx_qid, tx_qid)           ((void)0)
#endif
#include "mcdma.h"
#include "mcdma_regs.h"

void mcdma_descq_reset(McdmaQueue *q)
{
    q->ctrl = 0;
    q->ring_base = 0;
    q->ring_size_log2 = 0;
    q->ring_entries = 0;
    q->head = 0;
    q->tail = 0;
    q->completed = 0;
    q->wb_addr = 0;
    q->enabled = false;
    q->running = false;
    q->desc_processed = 0;
    q->bytes_copied = 0;
    q->invalid_skipped = 0;
    q->backpressure_stalls = 0;
}

void mcdma_descq_writeback(McdmaState *s, McdmaQueue *q)
{
    if (q->wb_addr == 0) {
        return;
    }

    trace_mcdma_desc_writeback(q->qid, q->dir, q->wb_addr,
                                q->head & Q_HEAD_PTR_MASK);
    address_space_stl_le(&address_space_memory, q->wb_addr,
                          q->head & Q_HEAD_PTR_MASK,
                          MEMTXATTRS_UNSPECIFIED, NULL);
}

static bool mcdma_descq_fetch_desc(McdmaState *s, McdmaQueue *q,
                                    uint32_t idx, McdmaDesc *desc)
{
    hwaddr desc_addr;

    if (q->ring_base == 0 || q->ring_entries == 0) {
        return false;
    }

    desc_addr = q->ring_base + idx * sizeof(McdmaDesc);

    trace_mcdma_desc_fetch(q->qid, q->dir, desc_addr,
                            q->head & Q_HEAD_PTR_MASK,
                            q->tail & Q_TAIL_PTR_MASK);

    address_space_read(&address_space_memory, desc_addr,
                       MEMTXATTRS_UNSPECIFIED, (uint8_t *)desc,
                       sizeof(McdmaDesc));

    if (desc->desc_invalid) {
        trace_mcdma_desc_invalid(q->qid, q->dir, idx);
        q->invalid_skipped++;
        return false;
    }

    return true;
}

static void mcdma_descq_update_len(McdmaState *s, McdmaQueue *q,
                                   uint32_t idx, uint32_t new_len)
{
    hwaddr desc_addr;
    McdmaDesc desc;

    if (q->ring_base == 0 || q->ring_entries == 0) {
        return;
    }

    desc_addr = q->ring_base + idx * sizeof(McdmaDesc);
    address_space_read(&address_space_memory, desc_addr,
                       MEMTXATTRS_UNSPECIFIED, (uint8_t *)&desc,
                       sizeof(desc));
    desc.len = new_len & 0xFFFFF;
    desc.rx_pyld_cnt = new_len & 0xFFFFF;
    desc.sof = 1;
    desc.eof = 1;
    address_space_write(&address_space_memory, desc_addr,
                        MEMTXATTRS_UNSPECIFIED, (uint8_t *)&desc,
                        sizeof(desc));
}

void mcdma_descq_process_loopback(McdmaState *s, McdmaQueue *rxq,
                                   McdmaQueue *txq)
{
    McdmaDesc rx_desc, tx_desc;
    uint32_t rx_idx, tx_idx;
    uint32_t loop_len;

    if (!rxq->enabled || rxq->ring_entries == 0) {
        return;
    }
    if (!txq->enabled || txq->ring_entries == 0) {
        return;
    }

    while (txq->head != txq->tail && rxq->head != rxq->tail) {
        tx_idx = txq->head % txq->ring_entries;

        if (!mcdma_descq_fetch_desc(s, txq, tx_idx, &tx_desc)) {
            txq->head++;
            continue;
        }

        rx_idx = rxq->head % rxq->ring_entries;
        if (!mcdma_descq_fetch_desc(s, rxq, rx_idx, &rx_desc)) {
            rxq->head++;
            continue;
        }

        if (rx_desc.len > MCDMA_MAX_XFER_SIZE) {
            rxq->invalid_skipped++;
            rxq->head++;
            continue;
        }

        loop_len = tx_desc.len;
        if (loop_len == 0) {
            loop_len = rx_desc.len;
        }
        if (loop_len == 0 || loop_len > MCDMA_MAX_XFER_SIZE) {
            txq->invalid_skipped++;
            txq->head++;
            continue;
        }
        if (rx_desc.len != 0 && loop_len > rx_desc.len) {
            loop_len = rx_desc.len;
        }

        mcdma_loopback_rx_to_tx(s, rxq, txq, tx_desc.src, rx_desc.dest,
                                 loop_len,
                                 tx_desc.msix_en || rx_desc.msix_en,
                                 tx_desc.wb_en || rx_desc.wb_en,
                                 tx_desc.didx);

        mcdma_descq_update_len(s, rxq, rx_idx, loop_len);

        txq->head++;
        txq->completed = txq->head;
        rxq->head++;
        rxq->completed = rxq->head;

        if (rxq->ctrl & Q_CTRL_Q_WB_EN) {
            mcdma_descq_writeback(s, rxq);
        }

        if (txq->ctrl & Q_CTRL_Q_WB_EN) {
            mcdma_descq_writeback(s, txq);
        }

        trace_mcdma_desc_complete(txq->qid, txq->dir, tx_desc.didx,
                                  loop_len);

        if (rx_desc.link) {
            trace_mcdma_desc_link(rxq->qid, rxq->dir);
        }
    }
}

void mcdma_descq_process_tx(McdmaState *s, McdmaQueue *txq)
{
    McdmaDesc desc;
    uint32_t idx, processed = 0;

    if (!txq->enabled || txq->ring_entries == 0) {
        return;
    }

    idx = txq->head % txq->ring_entries;

    while (txq->head != txq->tail) {
        if (!mcdma_descq_fetch_desc(s, txq, idx, &desc)) {
            txq->head++;
            idx = txq->head % txq->ring_entries;
            continue;
        }

        txq->head++;
        txq->completed = txq->head;
        idx = txq->head % txq->ring_entries;
        processed++;

        trace_mcdma_desc_complete(txq->qid, txq->dir, desc.didx, desc.len);

        if (desc.link) {
            trace_mcdma_desc_link(txq->qid, txq->dir);
            idx = 0;
        }
    }

    if (processed) {
        txq->desc_processed += processed;
        if (txq->ctrl & Q_CTRL_Q_WB_EN) {
            mcdma_descq_writeback(s, txq);
        }
    }
}

void mcdma_descq_process_rx(McdmaState *s, McdmaQueue *rxq)
{
    if (!rxq->enabled || rxq->ring_entries == 0) {
        return;
    }

    /*
     * RX descriptors are posted receive buffers. They must remain pending until
     * loopback work matches them with a TX descriptor and fills guest memory.
     * Consuming them here drains the RX ring before any packet arrives.
     */
}
