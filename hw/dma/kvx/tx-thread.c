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
#include "qemu/bitops.h"
#include "hw/dma/kvx-dma.h"

#include "trace.h"
#include "registers.h"
#include "internals.h"

/*
 * Reset the internal micro-engine state so that it is ready to run a new job
 */
static inline void reset_micro_engine(KvxDmaTxThread *thread)
{
    thread->pc = 0;
    thread->strobe = 0xffff;
    thread->dcnt[0] = 0;
    thread->dcnt[1] = 0;
    thread->dcnt[2] = 0;
    thread->dcnt[3] = 0;
    thread->read_ptr = 0;
    thread->write_ptr = 0;
}

static void enable_thread(KvxDmaState *s, KvxDmaTxThread *thread)
{
    if (thread->running) {
        return;
    }

    reset_micro_engine(thread);
    thread->running = true;
}

uint64_t kvx_dma_tx_thread_read(KvxDmaState *s, size_t id,
                                hwaddr offset, unsigned int size)
{
    uint64_t ret = 0;
    KvxDmaTxThread *thread;
    KvxDmaTxJobContext *job;

    g_assert(id < KVX_DMA_NUM_TX_THREAD);

    thread = &s->tx_thread[id];
    job = kvx_dma_tx_thread_get_running_job(thread);

    switch (offset) {
    case A_TX_THREAD_PARAMETER ... A_TX_THREAD_PARAMETER + (7 * sizeof(uint64_t)):
        /* TX_THREAD_PARAMETER[0..7] registers */
        ret = job->parameters[offset / sizeof(uint64_t)];
        break;

    case A_TX_THREAD_PGRM_ID:
        ret = job->pgrm_id;
        break;

    case A_TX_THREAD_NOC_ROUTE_ID:
        ret = job->noc_route_id;
        break;

    case A_TX_THREAD_COMPLETION_QUEUE_ID:
        ret = FIELD_DP64(0, TX_THREAD_COMPLETION_QUEUE_ID,
                         QUEUE_ID, job->comp_queue_id);
        ret = FIELD_DP64(ret, TX_THREAD_COMPLETION_QUEUE_ID,
                         PUSH_EN, job->rx_job_push_enabled);
        ret = FIELD_DP64(ret, TX_THREAD_COMPLETION_QUEUE_ID,
                         RX_JOB_QUEUE_ID, job->rx_job_queue_id);
        break;

    case A_TX_THREAD_ACTIVATE:
        ret = thread->running;
        break;

    case A_TX_THREAD_FENCE_BEFORE:
        ret = job->fence_before;
        break;

    case A_TX_THREAD_FENCE_AFTER:
        ret = job->fence_after;
        break;

    case A_TX_THREAD_ERROR:
        ret = thread->errors;
        break;

    case A_TX_THREAD_ERROR_LAC:
        ret = thread->errors;
        thread->errors = 0;
        break;

    case A_TX_THREAD_ASN:
        ret = job->asn;
        break;

    default:
        break;
    }

    trace_kvx_dma_tx_thread_read(id, offset, ret);

    return ret;
}

void kvx_dma_tx_thread_write(KvxDmaState *s, size_t id, hwaddr offset,
                             uint64_t value, unsigned int size)
{
    KvxDmaTxThread *thread;
    KvxDmaTxJobContext *job;

    g_assert(id < KVX_DMA_NUM_TX_THREAD);

    thread = &s->tx_thread[id];
    job = kvx_dma_tx_thread_get_running_job(thread);

    trace_kvx_dma_tx_thread_write(id, offset, value);

    switch (offset) {
    case A_TX_THREAD_PARAMETER ... A_TX_THREAD_PARAMETER + (7 * sizeof(uint64_t)):
        /* TX_THREAD_PARAMETER[0..7] registers */
        job->parameters[offset / sizeof(uint64_t)] = value;
        break;

    case A_TX_THREAD_PGRM_ID:
        job->pgrm_id = value & R_TX_THREAD_PGRM_ID_PGRM_ID_MASK;
        break;

    case A_TX_THREAD_NOC_ROUTE_ID:
        job->noc_route_id = value & R_TX_THREAD_NOC_ROUTE_ID_ROUTE_ID_MASK;
        break;

    case A_TX_THREAD_COMPLETION_QUEUE_ID:
        job->comp_queue_id = FIELD_EX64(value,
                                        TX_THREAD_COMPLETION_QUEUE_ID,
                                        QUEUE_ID);
        job->rx_job_push_enabled = FIELD_EX64(value,
                                              TX_THREAD_COMPLETION_QUEUE_ID,
                                              PUSH_EN);
        job->rx_job_queue_id = FIELD_EX64(value,
                                          TX_THREAD_COMPLETION_QUEUE_ID,
                                          RX_JOB_QUEUE_ID);
        break;

    case A_TX_THREAD_ACTIVATE:
        if (value & 1) {
            enable_thread(s, thread);
        } else {
            thread->running = false;
        }
        break;

    case A_TX_THREAD_FENCE_BEFORE:
        job->fence_before = value & 0x1;
        break;

    case A_TX_THREAD_FENCE_AFTER:
        job->fence_after = value & 0x1;
        break;

    case A_TX_THREAD_ASN:
        job->asn = value & R_TX_THREAD_ASN_ASN_MASK;
        break;

    case A_TX_THREAD_ERROR:
    case A_TX_THREAD_ERROR_LAC:
        /* write ignored */
    default:
        break;
    }
}

void kvx_dma_tx_thread_reset(KvxDmaTxThread *thread)
{
    memset(thread, 0, sizeof(*thread));
}
