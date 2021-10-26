
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

static inline void enable_queue(KvxDmaState *s, KvxDmaTxJobQueue *queue)
{
    if (queue->running) {
        return;
    }

    queue->running = true;
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
