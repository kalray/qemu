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

uint64_t kvx_dma_rx_channel_read(KvxDmaState *s, size_t id,
                                 hwaddr offset, unsigned int size)
{
    KvxDmaRxChannel *chan;
    uint64_t ret = 0;

    g_assert(id < KVX_DMA_NUM_RX_CHANNEL);
    chan = &s->rx_channel[id];

    switch(offset) {
    case A_RX_CHANNEL_BUFFER_START_ADDRESS:
        ret = chan->start_addr;
        break;

    case A_RX_CHANNEL_BUFFER_SIZE:
        ret = chan->size;
        break;

    case A_RX_CHANNEL_BUFFER_ENABLE:
        ret = chan->buf_enabled;
        break;

    case A_RX_CHANNEL_CURRENT_OFFSET:
    case A_RX_CHANNEL_JOB_QUEUE_CFG:
        break;

    case A_RX_CHANNEL_ACTIVATED:
        ret = chan->activated;
        break;

    case A_RX_CHANNEL_BYTE_CNT:
    case A_RX_CHANNEL_NOTIF_CNT:
    case A_RX_CHANNEL_CNT_CLEAR_MODE:
    case A_RX_CHANNEL_DMA_ERROR_STATUS:
    case A_RX_CHANNEL_DMA_ERROR_STATUS_LAC:
    case A_RX_CHANNEL_COMP_QUEUE_CFG:
    case A_RX_CHANNEL_COMP_QUEUE_MODE:
    case A_RX_CHANNEL_COMP_QUEUE_START_ADDRESS:
    case A_RX_CHANNEL_COMP_QUEUE_SLOT_NB_LOG2:
    case A_RX_CHANNEL_COMP_QUEUE_WRITE_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_LOAD_INCR_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_VALID_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_LOAD_INCR_VALID_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_NOTIFICATION_ADDRESS:
    case A_RX_CHANNEL_COMP_QUEUE_FULL_NOTIFICATION_ADDRESS:
    case A_RX_CHANNEL_COMP_QUEUE_NOTIFICATION_ARG:
    case A_RX_CHANNEL_COMP_QUEUE_ASN:
    default:
        break;
    }

    trace_kvx_dma_rx_channel_read(id, offset, ret);
    return ret;
}

void kvx_dma_rx_channel_write(KvxDmaState *s, size_t id, hwaddr offset,
                              uint64_t value, unsigned int size)
{
    KvxDmaRxChannel *chan;

    g_assert(id < KVX_DMA_NUM_RX_CHANNEL);
    chan = &s->rx_channel[id];

    switch(offset) {
    case A_RX_CHANNEL_BUFFER_START_ADDRESS:
        chan->start_addr = value & ADDRESS_MASK;
        break;

    case A_RX_CHANNEL_BUFFER_SIZE:
        chan->size = value & ADDRESS_MASK;
        break;

    case A_RX_CHANNEL_BUFFER_ENABLE:
        chan->buf_enabled = value & 0x1;
        break;

    case A_RX_CHANNEL_CURRENT_OFFSET:
    case A_RX_CHANNEL_JOB_QUEUE_CFG:
        break;

    case A_RX_CHANNEL_ACTIVATED:
        chan->activated = value & 0x1;
        break;

    case A_RX_CHANNEL_BYTE_CNT:
    case A_RX_CHANNEL_NOTIF_CNT:
    case A_RX_CHANNEL_CNT_CLEAR_MODE:
    case A_RX_CHANNEL_DMA_ERROR_STATUS:
    case A_RX_CHANNEL_DMA_ERROR_STATUS_LAC:
    case A_RX_CHANNEL_COMP_QUEUE_CFG:
    case A_RX_CHANNEL_COMP_QUEUE_MODE:
    case A_RX_CHANNEL_COMP_QUEUE_START_ADDRESS:
    case A_RX_CHANNEL_COMP_QUEUE_SLOT_NB_LOG2:
    case A_RX_CHANNEL_COMP_QUEUE_WRITE_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_LOAD_INCR_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_VALID_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_LOAD_INCR_VALID_READ_POINTER:
    case A_RX_CHANNEL_COMP_QUEUE_NOTIFICATION_ADDRESS:
    case A_RX_CHANNEL_COMP_QUEUE_FULL_NOTIFICATION_ADDRESS:
    case A_RX_CHANNEL_COMP_QUEUE_NOTIFICATION_ARG:
    case A_RX_CHANNEL_COMP_QUEUE_ASN:
    default:
        break;
    }

    trace_kvx_dma_rx_channel_write(id, offset, value);
}

void kvx_dma_rx_channel_reset(KvxDmaRxChannel *chan)
{
    memset(chan, 0, sizeof(*chan));
}
