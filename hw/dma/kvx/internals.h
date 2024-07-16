/*
 * Kalray KVX MPPA cluster DMA
 *
 * Copyright (c) 2021 Kalray Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_DMA_KVX_INTERNALS_H
#define HW_DMA_KVX_INTERNALS_H

#define ADDRESS_MASK ((1ull << 41) - 1)

/*
 * Limit the number of executed bundles in one go to avoid e.g. infinite loops
 * in the micro-code.
 */
#define TX_THREAD_MAX_RUN_BUDGET 1024

/*
 * All possible errors in the DMA. Each DMA component can generate a subset of
 * those errors. The errors are reported into a common error register
 * (IT_VECTOR) and the component raising the error is flaged in the
 * corresponding ERROR_*_STATUS register.
 */
typedef enum KvxDmaError {
    KVX_DMA_ERR_RX_CLOSED_CHAN = 0,
    KVX_DMA_ERR_RX_WRITE_POINTER = 1,
    KVX_DMA_ERR_RX_BUFFER_SIZE = 2,
    KVX_DMA_ERR_RX_BUFFER_ADDR = 3,
    KVX_DMA_ERR_RX_BUFFER_DECC = 4,
    KVX_DMA_ERR_RX_COMP_QUEUE_ADDR = 5,
    KVX_DMA_ERR_RX_COMP_QUEUE_DECC = 6,
    KVX_DMA_ERR_RX_JOB_QUEUE_ADDR = 7,
    KVX_DMA_ERR_RX_JOB_QUEUE_DECC = 8,
    KVX_DMA_ERR_RX_JOB_CACHE_EMPTY_ADDR = 9,
    KVX_DMA_ERR_RX_JOB_CACHE_EMPTY_DECC = 10,
    KVX_DMA_ERR_RX_CHAN_JOB_CACHE = 11,
    KVX_DMA_ERR_TX_BUNDLE = 16,
    KVX_DMA_ERR_TX_PGRM_PERM = 17,
    KVX_DMA_ERR_TX_NOC_PERM = 18,
    KVX_DMA_ERR_TX_COM_PERM = 19,
    KVX_DMA_ERR_TX_READ_ADDR = 20,
    KVX_DMA_ERR_TX_READ_DECC = 21,
    KVX_DMA_ERR_TX_WRITE_ADDR = 22,
    KVX_DMA_ERR_TX_WRITE_DECC = 23,
    KVX_DMA_ERR_TX_COMP_QUEUE_ADDR = 24,
    KVX_DMA_ERR_TX_COMP_QUEUE_DECC = 25,
    KVX_DMA_ERR_TX_JOB_QUEUE_ADDR = 26,
    KVX_DMA_ERR_TX_JOB_QUEUE_DECC = 27,
    KVX_DMA_ERR_TX_JOB_TO_RX_JOB_PUSH = 28,
    KVX_DMA_ERR_TX_AT_ADD = 29,
    KVX_DMA_ERR_TX_VCHAN = 30,

    KVX_DMA_NUM_ERR
} KvxDmaError;


/*
 * Bundle decoding structures
 */
typedef enum BundleBranchOp {
    BUNDLE_BR_NOP = 0,
    BUNDLE_BR = 1,
    BUNDLE_BR_BEZ = 2,
    BUNDLE_BR_BNEZ = 3,
} BundleBranchOp;

typedef enum BundleDecounterOp {
    BUNDLE_DCNT_NOP = 0,
    BUNDLE_DCNT_DEC = 1,
    BUNDLE_DCNT_LOAD_REG = 2,
    BUNDLE_DCNT_LOAD_DCNT = 3,
} BundleDecounterOp;

typedef enum BundleWptrType {
    BUNDLE_WPTR_TYPE_NOC_ABS = 0,
    BUNDLE_WPTR_TYPE_NOC_REL = 1,
    BUNDLE_WPTR_TYPE_STORE_ADDR = 2,
    BUNDLE_WPTR_TYPE_ATOMIC_ADD_ADDR = 3,
} BundleWptrType;

typedef enum BundleWptrOp {
    BUNDLE_WPTR_NOP = 0,
    BUNDLE_WPTR_LOAD = 1,
    BUNDLE_WPTR_INC = 2,
    BUNDLE_WPTR_ADD = 3,
} BundleWptrOp;

typedef enum BundleCmdOp {
    BUNDLE_CMD_NOP = 0,
    BUNDLE_CMD_FENCE = 1,
    BUNDLE_CMD_SEND_RPTR = 2,
    BUNDLE_CMD_NOTIFY = 3,
    BUNDLE_CMD_SEND_REG = 4,
    BUNDLE_CMD_RESET_STROBE = 5,
    BUNDLE_CMD_RESERVED6 = 6,
    BUNDLE_CMD_RESERVED7 = 7,
} BundleCmdOp;

typedef enum BundleRptrOp {
    BUNDLE_RPTR_NOP = 0,
    BUNDLE_RPTR_LOAD = 1,
    BUNDLE_RPTR_INC = 2,
    BUNDLE_RPTR_ADD = 3,
} BundleRptrOp;

typedef struct DecodedBundle {
    unsigned int reg;
    unsigned int reg_size;
    unsigned int move_size;
    unsigned int dcnt;
    BundleWptrType wptr_type;
    uint8_t branch_addr;

    BundleBranchOp branch_op;
    BundleDecounterOp dcnt_op;
    BundleWptrOp wptr_op;
    BundleRptrOp rptr_op;
    BundleCmdOp cmd_op;

    bool load_strobe;
    bool reg_sign_ext;
    bool stop;
    bool send_eot;
    bool flush;
} DecodedBundle;


/*
 * MMIO read/write functions
 */
uint64_t kvx_dma_rx_channel_read(KvxDmaState *s, size_t id,
                                 hwaddr offset, unsigned int size);
void kvx_dma_rx_channel_write(KvxDmaState *s, size_t id, hwaddr offset,
                              uint64_t value, unsigned int size);

uint64_t kvx_dma_tx_thread_read(KvxDmaState *s, size_t id,
                                 hwaddr offset, unsigned int size);
void kvx_dma_tx_thread_write(KvxDmaState *s, size_t id, hwaddr offset,
                              uint64_t value, unsigned int size);

uint64_t kvx_dma_tx_pgrm_mem_read(KvxDmaState *s, size_t id,
                                  hwaddr offset, unsigned int size);
void kvx_dma_tx_pgrm_mem_write(KvxDmaState *s, size_t id, hwaddr offset,
                               uint64_t value, unsigned int size);

uint64_t kvx_dma_tx_pgrm_table_read(KvxDmaState *s, size_t id,
                                    hwaddr offset, unsigned int size);
void kvx_dma_tx_pgrm_table_write(KvxDmaState *s, size_t id, hwaddr offset,
                                 uint64_t value, unsigned int size);

uint64_t kvx_dma_noc_route_table_read(KvxDmaState *s, size_t id,
                                      hwaddr offset, unsigned int size);
void kvx_dma_noc_route_table_write(KvxDmaState *s, size_t id, hwaddr offset,
                                   uint64_t value, unsigned int size);

uint64_t kvx_dma_bw_limiter_table_read(KvxDmaState *s, size_t id,
                                       hwaddr offset, unsigned int size);
void kvx_dma_bw_limiter_table_write(KvxDmaState *s, size_t id, hwaddr offset,
                                    uint64_t value, unsigned int size);

uint64_t kvx_dma_tx_comp_queue_read(KvxDmaState *s, size_t id,
                                    hwaddr offset, unsigned int size);
void kvx_dma_tx_comp_queue_write(KvxDmaState *s, size_t id, hwaddr offset,
                                 uint64_t value, unsigned int size);

uint64_t kvx_dma_tx_job_queue_read(KvxDmaState *s, size_t id, hwaddr offset,
                                   unsigned int size);
void kvx_dma_tx_job_queue_write(KvxDmaState *s, size_t id, hwaddr offset,
                                uint64_t value, unsigned int size);

uint64_t kvx_dma_it_read(KvxDmaState *s, size_t id, hwaddr offset,
                         unsigned int size);
void kvx_dma_it_write(KvxDmaState *s, size_t id, hwaddr offset,
                      uint64_t value, unsigned int size);

uint64_t kvx_dma_errors_read(KvxDmaState *s, size_t id, hwaddr offset,
                             unsigned int size);
void kvx_dma_errors_write(KvxDmaState *s, size_t id, hwaddr offset,
                          uint64_t value, unsigned int size);


/*
 * Reset functions
 */
void kvx_dma_irq_errors_reset(KvxDmaState *s);
void kvx_dma_mem_reset(KvxDmaState *s);
void kvx_dma_tx_thread_reset(KvxDmaTxThread *thread);
void kvx_dma_tx_comp_queue_reset(KvxDmaTxCompQueue *queue);
void kvx_dma_tx_job_queue_reset(KvxDmaTxJobQueue *queue);
void kvx_dma_rx_channel_reset(KvxDmaRxChannel *chan);


/*
 * Error handling functions
 */
void kvx_dma_report_error(KvxDmaState *s, KvxDmaError err);
const char *kvx_dma_error_str(KvxDmaError err);

/*
 * Bundle related functions
 */
bool kvx_dma_bundle_decode(uint64_t bundle, DecodedBundle *decoded);
void kvx_dma_bundle_disas(const DecodedBundle *decoded, const char *sep,
                          GString *out);


/*
 * Tx job queue functions
 */
void kvx_dma_tx_job_queue_report_error(KvxDmaState *s, KvxDmaTxJobQueue *queue,
                                       KvxDmaError err);

static inline size_t kvx_dma_tx_job_queue_get_id(KvxDmaState *s,
                                                 KvxDmaTxJobQueue *queue)
{
    return (size_t)((queue - &s->tx_job_queue[0]) / sizeof(KvxDmaTxJobQueue));
}


/*
 * TX thread functions
 */
void kvx_dma_schedule_run_tx_threads(KvxDmaState *s);
void kvx_dma_run_tx_threads(KvxDmaState *s);
void kvx_dma_tx_thread_report_error(KvxDmaState *s, KvxDmaTxThread *thread,
                                    KvxDmaError err);


static inline size_t kvx_dma_tx_thread_get_id(KvxDmaState *s,
                                              KvxDmaTxThread *thread)
{
    return (size_t)((thread - &s->tx_thread[0]) / sizeof(KvxDmaTxThread));
}

static inline KvxDmaTxJobContext *kvx_dma_tx_thread_get_running_job(KvxDmaTxThread *thread)
{
    return &thread->job_pool[thread->running_job];
}

static inline KvxDmaTxJobContext *kvx_dma_tx_thread_get_cached_job(KvxDmaTxThread *thread)
{
    return &thread->job_pool[!thread->running_job];
}

static inline bool kvx_dma_tx_thread_has_cached_job(KvxDmaTxThread *thread)
{
    KvxDmaTxJobContext *job = kvx_dma_tx_thread_get_cached_job(thread);
    return job->valid;
}


/*
 * TX Completion queue functions
 */
bool kvx_dma_tx_comp_queue_notify(KvxDmaState *s, KvxDmaTxCompQueue *queue,
                                  KvxDmaTxThread *initiator);

static inline bool kvx_dma_tx_comp_queue_is_full(KvxDmaTxCompQueue *queue)
{
    if (queue->field_enabled == 0) {
        return false;
    }

    return queue->write_ptr > (queue->valid_read_ptr + (1 << queue->num_slots));
}

static inline size_t kvx_dma_tx_comp_queue_get_id(KvxDmaState *s,
                                                  KvxDmaTxCompQueue *queue)
{
    return (size_t)((queue - &s->tx_comp_queue[0]) / sizeof(KvxDmaTxCompQueue));
}
#endif