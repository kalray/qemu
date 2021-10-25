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
#include "qemu/units.h"
#include "exec/address-spaces.h"
#include "hw/dma/kvx-dma.h"
#include "hw/irq.h"

#include "trace.h"
#include "registers.h"
#include "internals.h"

const char *kvx_dma_error_str(KvxDmaError err)
{
    static const char * const ERROR_STR[KVX_DMA_NUM_ERR] = {
        [KVX_DMA_ERR_RX_CLOSED_CHAN] = "RX closed channel error",
        [KVX_DMA_ERR_RX_WRITE_POINTER] = "RX write pointer error",
        [KVX_DMA_ERR_RX_BUFFER_SIZE] = "RX buffer size error",
        [KVX_DMA_ERR_RX_BUFFER_ADDR] = "RX buffer read/write error",
        [KVX_DMA_ERR_RX_BUFFER_DECC] = "RX buffer read/write ECC error",
        [KVX_DMA_ERR_RX_COMP_QUEUE_ADDR] = "RX completion queue write error",
        [KVX_DMA_ERR_RX_COMP_QUEUE_DECC] = "RX completion queue write ECC error",
        [KVX_DMA_ERR_RX_JOB_QUEUE_ADDR] = "RX job queue read error",
        [KVX_DMA_ERR_RX_JOB_QUEUE_DECC] = "RX job queue read ECC error",
        [KVX_DMA_ERR_RX_JOB_CACHE_EMPTY_ADDR] = "RX job queue job cache empty access error",
        [KVX_DMA_ERR_RX_JOB_CACHE_EMPTY_DECC] = "RX job queue job cache empty access ECC error",
        [KVX_DMA_ERR_RX_CHAN_JOB_CACHE] = "RX job cache error",
        [KVX_DMA_ERR_TX_BUNDLE] = "TX thread bundle error",
        [KVX_DMA_ERR_TX_PGRM_PERM] = "TX thread program permission error",
        [KVX_DMA_ERR_TX_NOC_PERM] = "TX NoC permission error",
        [KVX_DMA_ERR_TX_COM_PERM] = "TX completion queue permission error (disabled or ASN mismatch)",
        [KVX_DMA_ERR_TX_READ_ADDR] = "TX read error",
        [KVX_DMA_ERR_TX_READ_DECC] = "TX read ECC error",
        [KVX_DMA_ERR_TX_WRITE_ADDR] = "TX write error",
        [KVX_DMA_ERR_TX_WRITE_DECC] = "TX write ECC error",
        [KVX_DMA_ERR_TX_COMP_QUEUE_ADDR] = "TX completion queue write error",
        [KVX_DMA_ERR_TX_COMP_QUEUE_DECC] = "TX completion queue write ECC error",
        [KVX_DMA_ERR_TX_JOB_QUEUE_ADDR] = "TX job queue read error",
        [KVX_DMA_ERR_TX_JOB_QUEUE_DECC] = "TX job queue read ECC error",
        [KVX_DMA_ERR_TX_JOB_TO_RX_JOB_PUSH] = "TX to RX job push error",
        [KVX_DMA_ERR_TX_AT_ADD] = "TX atomic add error",
        [KVX_DMA_ERR_TX_VCHAN] = "TX virtual channel error",
    };

    return ERROR_STR[err];
}

static void update_irq(KvxDmaState *s)
{
    bool raise_irq;

    raise_irq = !!(s->irq_en | s->irq_vector);

    qemu_set_irq(s->irq, raise_irq);
}

void kvx_dma_report_error(KvxDmaState *s, KvxDmaError err)
{
    trace_kvx_dma_global_error(kvx_dma_error_str(err));

    s->irq_vector |= (1ull << err);
    update_irq(s);
}

uint64_t kvx_dma_it_read(KvxDmaState *s, size_t id, hwaddr offset,
                         unsigned int size)
{
    uint64_t ret = 0;

    switch(offset) {
        case A_DMA_IT_EN:
            ret = s->irq_en;
            break;

        case A_DMA_IT_VECTOR:
            ret = s->irq_vector;
            break;

        case A_DMA_IT_VECTOR_LAC:
            ret = s->irq_vector;
            s->irq_vector = 0;

            update_irq(s);
            break;

        default:
            break;
    }

    trace_kvx_dma_it_read(offset, ret);

    return ret;
}

void kvx_dma_it_write(KvxDmaState *s, size_t id, hwaddr offset,
                      uint64_t value, unsigned int size)
{
    trace_kvx_dma_it_write(offset, value);

    switch(offset) {
        case A_DMA_IT_EN:
            s->irq_en = value & DMA_IT_EN_WRITE_MASK;
            update_irq(s);
            break;

        case A_DMA_IT_VECTOR:
        case A_DMA_IT_VECTOR_LAC:
            /* Write ignored */
        default:
            break;
    }
}


uint64_t kvx_dma_errors_read(KvxDmaState *s, size_t id, hwaddr offset,
                             unsigned int size)
{
    uint64_t ret = 0;

    switch(offset) {
    case A_DMA_ERROR_RX_CHAN_ERROR_STATUS:
        ret = s->rx_channel_err;
        break;

    case A_DMA_ERROR_RX_CHAN_ERROR_STATUS_LAC:
        ret = s->rx_channel_err;
        s->rx_channel_err = 0;
        break;

    case A_DMA_ERROR_RX_JOB_ERROR_STATUS:
        ret = s->rx_job_queue_err;
        break;

    case A_DMA_ERROR_RX_JOB_ERROR_STATUS_LAC:
        ret = s->rx_job_queue_err;
        s->rx_job_queue_err = 0;
        break;

    case A_DMA_ERROR_TX_JOB_ERROR_STATUS:
        ret = s->tx_job_queue_err;
        break;

    case A_DMA_ERROR_TX_JOB_ERROR_STATUS_LAC:
        ret = s->tx_job_queue_err;
        s->tx_job_queue_err = 0;
        break;

    case A_DMA_ERROR_TX_THREAD_ERROR_STATUS:
        ret = s->tx_thread_err;
        break;

    case A_DMA_ERROR_TX_THREAD_ERROR_STATUS_LAC:
        ret = s->tx_thread_err;
        s->tx_thread_err = 0;
        break;

    case A_DMA_ERROR_TX_COMP_ERROR_STATUS:
        ret = s->tx_comp_queue_err;
        break;

    case A_DMA_ERROR_TX_COMP_ERROR_STATUS_LAC:
        ret = s->tx_comp_queue_err;
        s->tx_comp_queue_err = 0;
        break;

    default:
        break;
    }

    trace_kvx_dma_errors_read(offset, ret);

    return ret;
}

void kvx_dma_errors_write(KvxDmaState *s, size_t id, hwaddr offset,
                          uint64_t value, unsigned int size)
{

    trace_kvx_dma_errors_write(offset, value);

    switch(offset) {
    case A_DMA_ERROR_RX_CHAN_ERROR_STATUS:
    case A_DMA_ERROR_RX_CHAN_ERROR_STATUS_LAC:
    case A_DMA_ERROR_RX_JOB_ERROR_STATUS:
    case A_DMA_ERROR_RX_JOB_ERROR_STATUS_LAC:
    case A_DMA_ERROR_TX_JOB_ERROR_STATUS:
    case A_DMA_ERROR_TX_JOB_ERROR_STATUS_LAC:
    case A_DMA_ERROR_TX_THREAD_ERROR_STATUS:
    case A_DMA_ERROR_TX_THREAD_ERROR_STATUS_LAC:
    case A_DMA_ERROR_TX_COMP_ERROR_STATUS:
    case A_DMA_ERROR_TX_COMP_ERROR_STATUS_LAC:
        /* Write ignored */
    default:
        break;
    }
}

void kvx_dma_irq_errors_reset(KvxDmaState *s)
{
    s->rx_channel_err = 0;
    s->rx_job_queue_err = 0;
    s->tx_job_queue_err = 0;
    s->tx_thread_err = 0;
    s->tx_comp_queue_err = 0;
    s->irq_en = 0;
    s->irq_vector = 0;

    update_irq(s);
}
