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

uint64_t kvx_dma_tx_comp_queue_read(KvxDmaState *s, size_t id, hwaddr offset,
                                    unsigned int size)
{
    uint64_t ret;
    KvxDmaTxCompQueue *queue;

    g_assert(id < KVX_DMA_NUM_TX_COMP_QUEUE);
    queue = &s->tx_comp_queue[id];

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
    return ret;
}

void kvx_dma_tx_comp_queue_write(KvxDmaState *s, size_t id, hwaddr offset,
                                 uint64_t value, unsigned int size)
{
    KvxDmaTxCompQueue *queue;

    g_assert(id < KVX_DMA_NUM_TX_COMP_QUEUE);
    queue = &s->tx_comp_queue[id];

    trace_kvx_dma_tx_comp_queue_write(id, offset, value);

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
}

void kvx_dma_tx_comp_queue_reset(KvxDmaTxCompQueue *queue)
{
    memset(queue, 0, sizeof(*queue));
}
