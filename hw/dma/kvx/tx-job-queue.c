
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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/dma/kvx-dma.h"

#include "trace.h"
#include "registers.h"
#include "internals.h"

/*
 * Returns the address of the next job descriptor of the queue, given by the
 * start address, the queue size, and the read pointer.
 */
static inline hwaddr get_next_job_desc_addr(KvxDmaTxJobQueue *queue)
{
    uint64_t ptr = queue->read_ptr & ((1 << queue->num_slots) - 1);

    return queue->start_addr + (ptr * TX_JOB_DESC_SIZE);
}

/*
 * Fetch the next job descriptor from memory.
 * Return false on load error.
 */
static bool fetch_job_desc(KvxDmaState *s, KvxDmaTxJobQueue *queue,
                           uint64_t *job_desc)
{
    hwaddr addr;
    MemTxResult res;

    addr = get_next_job_desc_addr(queue);
    res = address_space_read(&s->as, addr, MEMTXATTRS_UNSPECIFIED,
                             job_desc, TX_JOB_DESC_SIZE);

    return res == MEMTX_OK;
}

/*
 * Initialize a job context with data from a job descriptor
 */
static void fill_job_from_desc(KvxDmaTxJobContext *job, uint64_t *desc)
{
    uint64_t desc_ctrl = desc[R_TX_JOB_DESC_CTRL];

    memcpy(job->parameters, desc, sizeof(job->parameters));
    job->noc_route_id = FIELD_EX64(desc_ctrl, TX_JOB_DESC_CTRL, NOC_ROUTE_ID);
    job->pgrm_id = FIELD_EX64(desc_ctrl, TX_JOB_DESC_CTRL, PGRM_ID);
    job->comp_queue_id = FIELD_EX64(desc_ctrl, TX_JOB_DESC_CTRL, COMP_QUEUE_ID);
    job->rx_job_push_enabled = FIELD_EX64(desc_ctrl, TX_JOB_DESC_CTRL, RX_JOB_PUSH_EN);
    job->rx_job_queue_id = FIELD_EX64(desc_ctrl, TX_JOB_DESC_CTRL, RX_JOB_QUEUE_ID);
    job->fence_before = FIELD_EX64(desc_ctrl, TX_JOB_DESC_CTRL, FENCE_BEFORE);
    job->fence_after = FIELD_EX64(desc_ctrl, TX_JOB_DESC_CTRL, FENCE_AFTER);

    job->valid = true;
}

/*
 * Extract the next job from the queue and fill the thread cache with it.
 * The thread cache must be empty.
 *
 * @return true if a job has been successfully put into the thread cache, false
 *         otherwise.
 */
static bool run_queue(KvxDmaState *s, KvxDmaTxJobQueue *queue,
                      KvxDmaTxThread *thread)
{
    uint64_t job_desc[TX_JOB_DESC_NUM_ELTS];
    KvxDmaTxJobContext *job = NULL;

    if (!queue->running) {
        /* This queue is not running */
        return false;
    }

    if (queue->valid_write_ptr <= queue->read_ptr) {
        /* Queue empty */
        return false;
    }

    if (!fetch_job_desc(s, queue, job_desc)) {
        kvx_dma_tx_job_queue_report_error(s, queue,
                                          KVX_DMA_ERR_TX_JOB_QUEUE_ADDR);
        return false;
    }

    /* XXX not sure when this is done */
    queue->read_ptr++;

    job = kvx_dma_tx_thread_get_cached_job(thread);

    fill_job_from_desc(job, job_desc);

    return true;
}

static bool run_queues_for_thread(KvxDmaState *s, size_t thread_id)
{
    size_t i;
    KvxDmaTxThread *thread = &s->tx_thread[thread_id];

    for (i = 0; i < KVX_DMA_NUM_TX_JOB_QUEUE; i++) {
        size_t local_i = (thread->next_queue + i) % KVX_DMA_NUM_TX_JOB_QUEUE;
        KvxDmaTxJobQueue *queue = &s->tx_job_queue[local_i];

        if (queue->thread_id != thread_id) {
            /* This queue is not targeting this thread */
            continue;
        }

        if (run_queue(s, queue, thread)) {
            /* Cache successfully filled */
            thread->next_queue = (local_i + 1) % KVX_DMA_NUM_RX_JOB_QUEUE;
            return true;
        }
    }

    return false;
}

static void run_queues(KvxDmaState *s)
{
    size_t i;
    bool run_threads = false;

    for (i = 0; i < KVX_DMA_NUM_TX_THREAD; i++) {
        KvxDmaTxThread *thread = &s->tx_thread[i];

        if (kvx_dma_tx_thread_has_cached_job(thread)) {
            /* A job is already waiting in the cache */
            continue;
        }

        run_threads = run_threads || run_queues_for_thread(s, i);
    }

    if (run_threads) {
        kvx_dma_run_tx_threads(s);
    }
}

static inline void enable_queue(KvxDmaState *s, KvxDmaTxJobQueue *queue)
{
    if (queue->running) {
        return;
    }

    queue->running = true;
    run_queues(s);
}

void kvx_dma_tx_job_queue_report_error(KvxDmaState *s, KvxDmaTxJobQueue *queue,
                                       KvxDmaError err)
{
    /* Global DMA error to local TX job queue error mapping */
    static const uint64_t ERROR_MAPPING[KVX_DMA_NUM_ERR] = {
        [KVX_DMA_ERR_TX_PGRM_PERM] = R_TX_JOB_QUEUE_STATUS_PGRM_PERM_SHIFT,
        [KVX_DMA_ERR_TX_READ_ADDR] = R_TX_JOB_QUEUE_STATUS_PGRM_READ_ADDR_SHIFT,
        [KVX_DMA_ERR_TX_READ_DECC] = R_TX_JOB_QUEUE_STATUS_PGRM_READ_DECC_SHIFT,
        [KVX_DMA_ERR_TX_WRITE_ADDR] = R_TX_JOB_QUEUE_STATUS_PGRM_WRITE_ADDR_SHIFT,
        [KVX_DMA_ERR_TX_WRITE_DECC] = R_TX_JOB_QUEUE_STATUS_PGRM_WRITE_DECC_SHIFT,
        [KVX_DMA_ERR_TX_NOC_PERM] = R_TX_JOB_QUEUE_STATUS_NOC_TABLE_PERM_SHIFT,
        [KVX_DMA_ERR_TX_COM_PERM] = R_TX_JOB_QUEUE_STATUS_COMPLETION_QUEUE_PERM_SHIFT,
        [KVX_DMA_ERR_TX_COMP_QUEUE_ADDR] = R_TX_JOB_QUEUE_STATUS_COMPLETION_QUEUE_ADDR_SHIFT,
        [KVX_DMA_ERR_TX_COMP_QUEUE_DECC] = R_TX_JOB_QUEUE_STATUS_COMPLETION_QUEUE_DECC_SHIFT,
        [KVX_DMA_ERR_TX_BUNDLE] = R_TX_JOB_QUEUE_STATUS_BUNDLE_SHIFT,
        [KVX_DMA_ERR_TX_JOB_QUEUE_ADDR] = R_TX_JOB_QUEUE_STATUS_JOB_QUEUE_ADDR_SHIFT,
        [KVX_DMA_ERR_TX_JOB_QUEUE_DECC] = R_TX_JOB_QUEUE_STATUS_JOB_QUEUE_DECC_SHIFT,
        [KVX_DMA_ERR_TX_JOB_TO_RX_JOB_PUSH] = R_TX_JOB_QUEUE_STATUS_RX_JOB_QUEUE_SHIFT,
        [KVX_DMA_ERR_TX_AT_ADD] = R_TX_JOB_QUEUE_STATUS_ATOMIC_ADD_SHIFT,
        [KVX_DMA_ERR_TX_VCHAN] = R_TX_JOB_QUEUE_STATUS_VCHAN_SHIFT,
    };

    size_t id = kvx_dma_tx_job_queue_get_id(s, queue);

    queue->running = false;
    queue->errors |= (1 << ERROR_MAPPING[err]);
    s->tx_job_queue_err |= (1 << id);

    trace_kvx_dma_tx_job_queue_error(id, kvx_dma_error_str(err));

    kvx_dma_report_error(s, err);
}

uint64_t kvx_dma_tx_job_queue_read(KvxDmaState *s, size_t id, hwaddr offset,
                                   unsigned int size)
{
    uint64_t ret;
    KvxDmaTxJobQueue *queue;

    g_assert(id < KVX_DMA_NUM_TX_JOB_QUEUE);
    queue = &s->tx_job_queue[id];

    switch (offset) {
    case A_TX_JOB_QUEUE_START_ADDRESS:
        ret = queue->start_addr;
        break;

    case A_TX_JOB_QUEUE_SLOT_NB_LOG2:
        ret = queue->num_slots;
        break;

    case A_TX_JOB_QUEUE_WRITE_POINTER:
        ret = queue->write_ptr;
        break;

    case A_TX_JOB_QUEUE_LOAD_INCR_WRITE_POINTER:
        ret = queue->write_ptr++;
        break;

    case A_TX_JOB_QUEUE_VALID_WRITE_POINTER:
        ret = queue->valid_write_ptr;
        break;

    case A_TX_JOB_QUEUE_LOAD_INCR_VALID_WRITE_POINTER:
        ret = queue->valid_write_ptr++;
        run_queues(s);
        break;

    case A_TX_JOB_QUEUE_READ_POINTER:
        ret = queue->read_ptr;
        break;

    case A_TX_JOB_QUEUE_NOTIFICATION_ADDRESS:
        ret = queue->notify_addr;
        break;

    case A_TX_JOB_QUEUE_NOTIFICATION_ARG:
        ret = queue->notify_arg;
        break;

    case A_TX_JOB_QUEUE_STATUS:
        if (queue->errors) {
            ret = queue->errors | TX_JOB_QUEUE_STATUS_ERROR;
        } else {
            ret = queue->running;
        }
        break;

    case A_TX_JOB_QUEUE_ACTIVATE:
        /* read as 0 */
    default:
        /* n/i */
        ret = 0;
    }

    trace_kvx_dma_tx_job_queue_read(id, offset, ret);
    return ret;
}

void kvx_dma_tx_job_queue_write(KvxDmaState *s, size_t id, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    KvxDmaTxJobQueue *queue;

    g_assert(id < KVX_DMA_NUM_TX_JOB_QUEUE);
    queue = &s->tx_job_queue[id];

    switch (offset) {
    case A_TX_JOB_QUEUE_START_ADDRESS:
        queue->start_addr = value & ADDRESS_MASK;
        break;

    case A_TX_JOB_QUEUE_SLOT_NB_LOG2:
        queue->num_slots = value & 0x1f;
        break;

    case A_TX_JOB_QUEUE_WRITE_POINTER:
        queue->write_ptr = value;
        break;

    case A_TX_JOB_QUEUE_VALID_WRITE_POINTER:
        queue->valid_write_ptr = value;
        run_queues(s);
        break;

    case A_TX_JOB_QUEUE_READ_POINTER:
        queue->read_ptr = value;
        break;

    case A_TX_JOB_QUEUE_ACTIVATE:
        if (value == 1) {
            enable_queue(s, queue);
        }
        break;

    case A_TX_JOB_QUEUE_NOTIFICATION_ADDRESS:
        queue->notify_addr = value & ADDRESS_MASK;
        break;

    case A_TX_JOB_QUEUE_NOTIFICATION_ARG:
        queue->notify_arg = value;
        break;

    case A_TX_JOB_QUEUE_LOAD_INCR_WRITE_POINTER:
    case A_TX_JOB_QUEUE_LOAD_INCR_VALID_WRITE_POINTER:
        /* write ignored */
    default:
        /* n/i */
        break;
    }

    trace_kvx_dma_tx_job_queue_write(id, offset, value);
}

void kvx_dma_tx_job_queue_reset(KvxDmaTxJobQueue *queue)
{
    memset(queue, 0, sizeof(*queue));
}
