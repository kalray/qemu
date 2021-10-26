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
#include "hw/dma/kvx-dma.h"

#include "trace.h"
#include "registers.h"
#include "internals.h"

static inline void report_error(KvxDmaState *s, KvxDmaTxCompQueue *queue,
                                KvxDmaTxThread *initiator, KvxDmaError err)
{
    /* Global DMA error to local TX completion queue error mapping */
    static const uint64_t ERROR_MAPPING[KVX_DMA_NUM_ERR] = {
        [KVX_DMA_ERR_TX_COMP_QUEUE_ADDR] = R_TX_COMP_QUEUE_STATUS_MEM_ADDR_ERROR_SHIFT,
        [KVX_DMA_ERR_TX_COMP_QUEUE_DECC] = R_TX_COMP_QUEUE_STATUS_MEM_DECC_ERROR_SHIFT,
    };

    size_t id = kvx_dma_tx_comp_queue_get_id(s, queue);

    queue->running = false;
    queue->errors |= (1 << ERROR_MAPPING[err]);
    s->tx_comp_queue_err |= (1 << id);

    trace_kvx_dma_tx_comp_queue_error(id, kvx_dma_error_str(err));
    kvx_dma_report_error(s, err);

    /* propagate the error to the initiator thread */
    kvx_dma_tx_thread_report_error(s, initiator, err);
}

/*
 * Return the size of the job completion descriptor given the current
 * configuration of the queue.
 */
static inline uint64_t get_desc_write_size(KvxDmaTxCompQueue *queue)
{
    static const uint64_t WRITE_SIZE[] = {
        [0] = 0,
        [1] = 2 * sizeof(uint64_t),
        [2] = 10 * sizeof(uint64_t),
        [3] = 0, /* reserved */
    };

    g_assert(queue->field_enabled < ARRAY_SIZE(WRITE_SIZE));
    return WRITE_SIZE[queue->field_enabled];
}

/*
 * Return the address the next descriptor should be written at.
 */
static inline hwaddr get_next_desc_addr(KvxDmaTxCompQueue *queue)
{
    uint64_t write_size, offset;

    if (queue->static_mode) {
        return queue->start_addr;
    }

    write_size = get_desc_write_size(queue);
    offset = (queue->write_ptr * write_size) & ((1 << queue->num_slots) - 1);
    return queue->start_addr + offset;
}

static bool write_completion_desc(KvxDmaState *s, KvxDmaTxCompQueue *queue,
                                  KvxDmaTxThread *initiator)
{
    uint64_t desc[TX_JOB_DESC_NUM_ELTS];
    size_t param_copy_sz;
    hwaddr addr;
    MemTxResult res;
    KvxDmaTxJobContext *job;

    job = kvx_dma_tx_thread_get_running_job(initiator);

    /*
     * When field_enabled is 1, only copy the first 2 parameters. When it is 2,
     * copy the whole descriptor (the 8 params + the control part)
     */
    param_copy_sz = (queue->field_enabled == 2) ? 8 : 2;
    param_copy_sz *= sizeof(uint64_t);

    memcpy(desc, job->parameters, param_copy_sz);

    if (queue->field_enabled == 2) {
        uint64_t *ctrl = &desc[R_TX_JOB_DESC_CTRL];

        *ctrl = FIELD_DP64(0, TX_JOB_DESC_CTRL, COMP_QUEUE_ID,
                             job->comp_queue_id);
        *ctrl = FIELD_DP64(*ctrl, TX_JOB_DESC_CTRL, RX_JOB_PUSH_EN,
                             job->rx_job_push_enabled);
        *ctrl = FIELD_DP64(*ctrl, TX_JOB_DESC_CTRL, RX_JOB_QUEUE_ID,
                             job->rx_job_queue_id);
        *ctrl = FIELD_DP64(*ctrl, TX_JOB_DESC_CTRL, NOC_ROUTE_ID,
                             job->noc_route_id);
        *ctrl = FIELD_DP64(*ctrl, TX_JOB_DESC_CTRL, PGRM_ID,
                             job->pgrm_id);
        *ctrl = FIELD_DP64(*ctrl, TX_JOB_DESC_CTRL, FENCE_BEFORE,
                             job->fence_before);
        *ctrl = FIELD_DP64(*ctrl, TX_JOB_DESC_CTRL, FENCE_AFTER,
                             job->fence_after);
    }

    addr = get_next_desc_addr(queue);

    res = address_space_write(&s->as, addr, MEMTXATTRS_UNSPECIFIED,
                              desc, get_desc_write_size(queue));

    if (res != MEMTX_OK) {
        report_error(s, queue, initiator, KVX_DMA_ERR_TX_COMP_QUEUE_ADDR);
        return false;
    }

    return true;
}

static void notify_completion(KvxDmaState *s, KvxDmaTxCompQueue *queue,
                              KvxDmaTxThread *initiator)
{
    size_t id = kvx_dma_tx_comp_queue_get_id(s, queue);
    MemTxResult res;

    trace_kvx_dma_tx_comp_queue_notify(id, queue->notify_addr,
                                       queue->notify_arg);

    res = address_space_write(&s->as, queue->notify_addr,
                              MEMTXATTRS_UNSPECIFIED, &queue->notify_arg,
                              sizeof(queue->notify_arg));

    if (res != MEMTX_OK) {
        report_error(s, queue, initiator, KVX_DMA_ERR_TX_COMP_QUEUE_ADDR);
    }
}

static inline bool check_perm(KvxDmaTxCompQueue *queue,
                              KvxDmaTxThread *initiator)
{
    KvxDmaTxJobContext *job;

    if (!queue->running) {
        return false;
    }

    if (queue->global) {
        return true;
    }

    job = kvx_dma_tx_thread_get_running_job(initiator);

    if (queue->asn != job->asn) {
        return false;
    }

    return true;
}

/*
 * Write a job completion descriptor into the queue according to the
 * initiator's running job, and send a notify event. Return false in case of
 * invalid notify, when:
 *   - The queue is disabled
 *   - The running job is not allowed to use this queue (ASN mismatch)
 */
bool kvx_dma_tx_comp_queue_notify(KvxDmaState *s, KvxDmaTxCompQueue *queue,
                                  KvxDmaTxThread *initiator)
{
    bool success = true;

    if (!check_perm(queue, initiator)) {
        return false;
    }

    if (queue->field_enabled == 1 || queue->field_enabled == 2) {
        success = write_completion_desc(s, queue, initiator);
    }

    /*
     * The write pointer is incremented in all cases (even when no descriptor
     * are written to memory).
     */
    queue->write_ptr++;

    if (!success) {
        /*
         * Even though we have a failure, we return true here because the error
         * is already handled within the queue logic and forwarded to the thread.
         */
        return true;
    }

    notify_completion(s, queue, initiator);

    return true;
}

uint64_t kvx_dma_tx_comp_queue_read(KvxDmaState *s, size_t id, hwaddr offset,
                                    unsigned int size)
{
    uint64_t ret;
    KvxDmaTxCompQueue *queue;
    bool was_full;

    g_assert(id < KVX_DMA_NUM_TX_COMP_QUEUE);
    queue = &s->tx_comp_queue[id];

    was_full = kvx_dma_tx_comp_queue_is_full(queue);

    switch (offset) {
    case A_TX_COMP_QUEUE_MODE:
        ret = queue->static_mode;
        break;

    case A_TX_COMP_QUEUE_START_ADDRESS:
        ret = queue->start_addr;
        break;

    case A_TX_COMP_QUEUE_SLOT_NB_LOG2:
        ret = queue->num_slots;
        break;

    case A_TX_COMP_QUEUE_GLOBAL:
        ret = queue->global;
        break;

    case A_TX_COMP_QUEUE_ASN:
        ret = queue->asn;
        break;

    case A_TX_COMP_QUEUE_FIELD_ENABLE:
        ret = queue->field_enabled;
        break;

    case A_TX_COMP_QUEUE_WRITE_POINTER:
        ret = queue->write_ptr;
        break;

    case A_TX_COMP_QUEUE_READ_POINTER:
        ret = queue->read_ptr;
        break;

    case A_TX_COMP_QUEUE_LOAD_INCR_READ_POINTER:
        ret = queue->read_ptr++;
        break;

    case A_TX_COMP_QUEUE_VALID_READ_POINTER:
        ret = queue->valid_read_ptr;
        break;

    case A_TX_COMP_QUEUE_LOAD_INCR_VALID_READ_POINTER:
        ret = queue->valid_read_ptr++;
        break;

    case A_TX_COMP_QUEUE_NOTIFICATION_ADDRESS:
        ret = queue->notify_addr;
        break;

    case A_TX_COMP_QUEUE_NOTIFICATION_ARG:
        ret = queue->notify_arg;
        break;

    case A_TX_COMP_QUEUE_STATUS:
        ret = queue->running;
        break;

    case A_TX_COMP_QUEUE_ACTIVATE:
    case A_TX_COMP_QUEUE_STOP:
        /* Read as zero */
    case A_TX_COMP_QUEUE_STATUS_LAC:
    default:
        /* n/i */
        ret = 0;
    }

    trace_kvx_dma_tx_comp_queue_read(id, offset, ret);

    if (was_full && !kvx_dma_tx_comp_queue_is_full(queue)) {
        kvx_dma_run_tx_threads(s);
    }

    return ret;
}

void kvx_dma_tx_comp_queue_write(KvxDmaState *s, size_t id, hwaddr offset,
                                 uint64_t value, unsigned int size)
{
    KvxDmaTxCompQueue *queue;
    bool was_full;

    g_assert(id < KVX_DMA_NUM_TX_COMP_QUEUE);
    queue = &s->tx_comp_queue[id];

    trace_kvx_dma_tx_comp_queue_write(id, offset, value);

    was_full = kvx_dma_tx_comp_queue_is_full(queue);

    switch (offset) {
    case A_TX_COMP_QUEUE_ACTIVATE:
        queue->running = (value == 1);
        break;

    case A_TX_COMP_QUEUE_MODE:
        /* XXX: Can the mode be changed while the queue is running? */
        queue->static_mode = value & 0x1;
        break;

    case A_TX_COMP_QUEUE_START_ADDRESS:
        queue->start_addr = value & ADDRESS_MASK;
        break;

    case A_TX_COMP_QUEUE_SLOT_NB_LOG2:
        queue->num_slots = value & 0x1f;
        break;

    case A_TX_COMP_QUEUE_GLOBAL:
        queue->global = value & 0x1;
        break;

    case A_TX_COMP_QUEUE_ASN:
        queue->asn = value & 0x1ff;
        break;

    case A_TX_COMP_QUEUE_FIELD_ENABLE:
        queue->field_enabled = value & 0x3;
        break;

    case A_TX_COMP_QUEUE_WRITE_POINTER:
        queue->write_ptr = value;
        break;

    case A_TX_COMP_QUEUE_READ_POINTER:
        queue->read_ptr = value;
        break;

    case A_TX_COMP_QUEUE_VALID_READ_POINTER:
        queue->valid_read_ptr = value;
        break;

    case A_TX_COMP_QUEUE_NOTIFICATION_ADDRESS:
        queue->notify_addr = value;
        break;

    case A_TX_COMP_QUEUE_NOTIFICATION_ARG:
        queue->notify_arg = value;
        break;

    case A_TX_COMP_QUEUE_STOP:
        if (value & 0x1) {
            queue->running = false;
        }
        break;

    case A_TX_COMP_QUEUE_STATUS:
    case A_TX_COMP_QUEUE_STATUS_LAC:
    case A_TX_COMP_QUEUE_LOAD_INCR_READ_POINTER:
    case A_TX_COMP_QUEUE_LOAD_INCR_VALID_READ_POINTER:
        /* Write ignored */
    default:
        /* n/i */
        break;
    }

    if (was_full && !kvx_dma_tx_comp_queue_is_full(queue)) {
        kvx_dma_run_tx_threads(s);
    }
}

void kvx_dma_tx_comp_queue_reset(KvxDmaTxCompQueue *queue)
{
    memset(queue, 0, sizeof(*queue));
}
