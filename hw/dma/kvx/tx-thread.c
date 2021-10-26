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

static void do_trace_bundle(size_t id,
                            const KvxDmaTxThread *thread,
                            const KvxDmaTxJobContext *job,
                            const DecodedBundle *decoded)
{
    g_autoptr(GString) disas = g_string_new(NULL);

    kvx_dma_bundle_disas(decoded, " | ", disas);
    trace_kvx_dma_tx_thread_disas_bundle(id, job->pgrm_id, thread->pc, disas->str);
}

void kvx_dma_tx_thread_report_error(KvxDmaState *s, KvxDmaTxThread *thread,
                                    KvxDmaError err)
{
    /* Global DMA error to local TX thread error mapping */
    static const uint64_t ERROR_MAPPING[KVX_DMA_NUM_ERR] = {
        [KVX_DMA_ERR_TX_PGRM_PERM] = R_TX_THREAD_ERROR_PGRM_PERM_SHIFT,
        [KVX_DMA_ERR_TX_READ_ADDR] = R_TX_THREAD_ERROR_PGRM_READ_ADDR_SHIFT,
        [KVX_DMA_ERR_TX_READ_DECC] = R_TX_THREAD_ERROR_PGRM_READ_DECC_SHIFT,
        [KVX_DMA_ERR_TX_WRITE_ADDR] = R_TX_THREAD_ERROR_PGRM_WRITE_ADDR_SHIFT,
        [KVX_DMA_ERR_TX_WRITE_DECC] = R_TX_THREAD_ERROR_PGRM_WRITE_DECC_SHIFT,
        [KVX_DMA_ERR_TX_NOC_PERM] = R_TX_THREAD_ERROR_NOC_TABLE_PERM_SHIFT,
        [KVX_DMA_ERR_TX_COM_PERM] = R_TX_THREAD_ERROR_COMPLETION_QUEUE_PERM_SHIFT,
        [KVX_DMA_ERR_TX_COMP_QUEUE_ADDR] = R_TX_THREAD_ERROR_COMPLETION_QUEUE_ADDR_SHIFT,
        [KVX_DMA_ERR_TX_COMP_QUEUE_DECC] = R_TX_THREAD_ERROR_COMPLETION_QUEUE_DECC_SHIFT,
        [KVX_DMA_ERR_TX_BUNDLE] = R_TX_THREAD_ERROR_BUNDLE_SHIFT,
        [KVX_DMA_ERR_TX_JOB_TO_RX_JOB_PUSH] = R_TX_THREAD_ERROR_RX_JOB_QUEUE_SHIFT,
        [KVX_DMA_ERR_TX_AT_ADD] = R_TX_THREAD_ERROR_ATOMIC_ADD_SHIFT,
        [KVX_DMA_ERR_TX_VCHAN] = R_TX_THREAD_ERROR_VCHAN_SHIFT,
    };

    size_t id = kvx_dma_tx_thread_get_id(s, thread);
    KvxDmaTxJobContext *job;

    job = kvx_dma_tx_thread_get_running_job(thread);
    thread->running = false;

    if (job->origin > -1) {
        /*
         * This job originates from a TX job queue. The error must be forwarded
         * to that queue.
         */
        KvxDmaTxJobQueue *queue;

        g_assert(job->origin < KVX_DMA_NUM_TX_JOB_QUEUE);
        queue = &s->tx_job_queue[job->origin];

        kvx_dma_tx_job_queue_report_error(s, queue, err);
    } else {
        /* Local error reporting */
        thread->errors |= (1 << ERROR_MAPPING[err]);
        s->tx_thread_err |= (1 << id);

        trace_kvx_dma_tx_thread_error(id, kvx_dma_error_str(err));

        kvx_dma_report_error(s, err);
    }
}

static inline uint64_t read_parameter(const KvxDmaTxJobContext *job,
                                      const DecodedBundle *decoded)
{
    size_t idx = decoded->reg * decoded->reg_size / 8;
    size_t offset = (decoded->reg * decoded->reg_size * 8) % 64;

    g_assert(idx < ARRAY_SIZE(job->parameters));

    return decoded->reg_sign_ext
        ? sextract64(job->parameters[idx], offset, decoded->reg_size * 8)
        : extract64(job->parameters[idx], offset, decoded->reg_size * 8);
}

static inline void do_trace_mem_access(KvxDmaState *s, KvxDmaTxThread *thread,
                                       const DecodedBundle *decoded,
                                       uint64_t *data, bool is_load)
{
    size_t thread_id = kvx_dma_tx_thread_get_id(s, thread);

    if (decoded->move_size == 16) {
        if (is_load) {
            trace_kvx_dma_tx_thread_load_mem128(thread_id, thread->read_ptr,
                                                data[1], data[0]);
        } else {
            trace_kvx_dma_tx_thread_store_mem128(thread_id, thread->write_ptr,
                                                 data[1], data[0],
                                                 thread->strobe);
        }
    } else {
        uint64_t data_mask = (1 << (decoded->move_size * 8)) - 1;

        if (is_load) {
            trace_kvx_dma_tx_thread_load_mem(thread_id, thread->read_ptr,
                                             data[0] & data_mask,
                                             decoded->move_size);
        } else {
            trace_kvx_dma_tx_thread_store_mem(thread_id, thread->write_ptr,
                                              data[0] & data_mask,
                                              decoded->move_size,
                                              thread->strobe);
        }
    }
}

static inline bool mem_send_data(KvxDmaState *s, KvxDmaTxThread *thread,
                                 const DecodedBundle *decoded,
                                 uint64_t *data)
{
    uint16_t strobe, size_mask;
    MemTxResult res = MEMTX_OK;

    size_mask = (1 << decoded->move_size) - 1;
    strobe = thread->strobe & size_mask;

    do_trace_mem_access(s, thread, decoded, data, false);

    if (strobe == size_mask) {
        res = address_space_write(&s->as, thread->write_ptr,
                                  MEMTXATTRS_UNSPECIFIED,
                                  data, decoded->move_size);
    } else {
        /* strobe is not full of ones */
        size_t addr = thread->write_ptr;
        uint8_t *pdata = (uint8_t *) data;

        while (strobe && (res == MEMTX_OK)) {
            size_t b = ctz32(strobe);

            strobe >>= b;
            addr += b;
            pdata += b;

            b = cto32(strobe);

            if (b) {
                res = address_space_write(&s->as, addr,
                                          MEMTXATTRS_UNSPECIFIED,
                                          pdata, b);
                strobe >>= b;
                addr += b;
                pdata += b;
            }
        }
    }

    if (res != MEMTX_OK) {
        kvx_dma_tx_thread_report_error(s, thread, KVX_DMA_ERR_TX_WRITE_ADDR);
        return false;
    }

    return true;
}

static inline bool send_data(KvxDmaState *s, KvxDmaTxThread *thread,
                             const DecodedBundle *decoded,
                             uint64_t *data)
{
    if (decoded->wptr_type != BUNDLE_WPTR_TYPE_STORE_ADDR) {
        qemu_log_mask(LOG_UNIMP, "kvx-dma: unimplemented write pointer type\n");
        return false;
    }

    return mem_send_data(s, thread, decoded, data);
}

static bool mem_recv_data(KvxDmaState *s, KvxDmaTxThread *thread,
                          const DecodedBundle *decoded,
                          uint64_t *data)
{
    MemTxResult res;

    res = address_space_read(&s->as, thread->read_ptr, MEMTXATTRS_UNSPECIFIED,
                             data, decoded->move_size);

    if (res != MEMTX_OK) {
        kvx_dma_tx_thread_report_error(s, thread, KVX_DMA_ERR_TX_READ_ADDR);
        return false;
    }

    do_trace_mem_access(s, thread, decoded, data, true);

    return true;
}

static void recv_and_send_data(KvxDmaState *s,
                               KvxDmaTxThread *thread,
                               const DecodedBundle *decoded)
{
    uint64_t data[2];

    g_assert(decoded->move_size <= sizeof(data));

    if (!mem_recv_data(s, thread, decoded, data)) {
        return;
    }

    mem_send_data(s, thread, decoded, data);
}

static void send_parameter(KvxDmaState *s,
                           KvxDmaTxThread *thread,
                           KvxDmaTxJobContext *job,
                           const DecodedBundle *decoded)
{
    uint64_t data[2];

    g_assert(decoded->move_size <= sizeof(data));

    data[0] = read_parameter(job, decoded);
    data[1] = 0;

    mem_send_data(s, thread, decoded, data);
}

/*
 * Simulate the bundle given as parameter. Return the next PC.
 */
static uint16_t exec_bundle(KvxDmaState *s, KvxDmaTxThread *thread,
                            DecodedBundle *decoded)
{
    KvxDmaTxJobContext *job;
    KvxDmaTxCompQueue *queue;
    uint16_t next_pc;

    job = kvx_dma_tx_thread_get_running_job(thread);
    next_pc = thread->pc + 1;

    /*
     * Note: the switches order matters as we don't want to clobber parts of
     * the thread state that can be read by other instructions in the same
     * bundle.
     */

    switch (decoded->branch_op) {
    case BUNDLE_BR_NOP:
        break;

    case BUNDLE_BR:
        next_pc = decoded->branch_addr;
        break;

    case BUNDLE_BR_BEZ:
        if (thread->dcnt[decoded->dcnt] == 0) {
            next_pc = decoded->branch_addr;
        }
        break;

    case BUNDLE_BR_BNEZ:
        if (thread->dcnt[decoded->dcnt] != 0) {
            next_pc = decoded->branch_addr;
        }
        break;
    }

    switch (decoded->cmd_op) {
    case BUNDLE_CMD_SEND_RPTR:
        recv_and_send_data(s, thread, decoded);
        break;

    case BUNDLE_CMD_NOTIFY:
        queue = &s->tx_comp_queue[job->comp_queue_id];
        if (!kvx_dma_tx_comp_queue_notify(s, queue, thread)) {
            kvx_dma_tx_thread_report_error(s, thread, KVX_DMA_ERR_TX_COM_PERM);
        }
        break;

    case BUNDLE_CMD_SEND_REG:
        send_parameter(s, thread, job, decoded);
        break;

    case BUNDLE_CMD_RESET_STROBE:
        thread->strobe = 0xffff;
        break;

    case BUNDLE_CMD_NOP:
    case BUNDLE_CMD_FENCE:
    case BUNDLE_CMD_RESERVED6:
    case BUNDLE_CMD_RESERVED7:
        break;
    }

    switch (decoded->dcnt_op) {
    case BUNDLE_DCNT_NOP:
        break;

    case BUNDLE_DCNT_DEC:
        thread->dcnt[decoded->dcnt]--;
        break;

    case BUNDLE_DCNT_LOAD_REG:
        thread->dcnt[decoded->dcnt] = read_parameter(job, decoded);
        break;

    case BUNDLE_DCNT_LOAD_DCNT:
        thread->dcnt[decoded->dcnt] = thread->dcnt[decoded->reg & 0x3];
        break;
    }

    switch (decoded->wptr_op) {
    case BUNDLE_WPTR_NOP:
        break;

    case BUNDLE_WPTR_LOAD:
        thread->write_ptr = read_parameter(job, decoded);
        break;

    case BUNDLE_WPTR_INC:
        thread->write_ptr += decoded->move_size;
        break;

    case BUNDLE_WPTR_ADD:
        thread->write_ptr += read_parameter(job, decoded);
        break;
    }

    switch (decoded->rptr_op) {
    case BUNDLE_RPTR_NOP:
        break;

    case BUNDLE_RPTR_LOAD:
        thread->read_ptr = read_parameter(job, decoded);
        break;

    case BUNDLE_RPTR_INC:
        thread->read_ptr += decoded->move_size;
        break;

    case BUNDLE_RPTR_ADD:
        thread->read_ptr += read_parameter(job, decoded);
        break;
    }

    if (decoded->stop) {
        thread->running = false;
    }

    return next_pc;
}

static inline void run_thread(KvxDmaState *s, size_t id,
                              KvxDmaTxThread *thread)
{
    uint16_t base_pc;
    uint64_t pgrm_desc;
    KvxDmaTxJobContext *job;
    KvxDmaTxCompQueue *queue;
    int run_budget = TX_THREAD_MAX_RUN_BUDGET;

    if (!thread->running) {
        return;
    }

    job = kvx_dma_tx_thread_get_running_job(thread);

    g_assert(job->comp_queue_id < KVX_DMA_NUM_TX_COMP_QUEUE);
    queue = &s->tx_comp_queue[job->comp_queue_id];
    if (kvx_dma_tx_comp_queue_is_full(queue)) {
        /*
         * The destination completion queue of the job in cache is currently
         * full. We must wait for some room before executing the job to ensure
         * it can write its notification at the end.
         */
        return;
    }

    g_assert(job->pgrm_id < KVX_DMA_TX_PGRM_TABLE_SIZE);
    pgrm_desc = s->tx_pgrm_table[job->pgrm_id];

    /* Job PC is relative to this address */
    base_pc = FIELD_EX64(pgrm_desc, TX_PGRM_TABLE, PM_START_ADDR);

    if (!FIELD_EX64(pgrm_desc, TX_PGRM_TABLE, TRANSFER_MODE)) {
        qemu_log_mask(LOG_UNIMP, "kvx-dma: NoC transfers not implemented\n");
        return;
    }

    while (thread->running && run_budget) {
        DecodedBundle decoded;
        uint64_t bundle;

        trace_kvx_dma_tx_thread_state(id, thread->pc, thread->strobe,
                                      thread->dcnt[0], thread->dcnt[1],
                                      thread->dcnt[2], thread->dcnt[3],
                                      thread->read_ptr, thread->write_ptr);

        g_assert(base_pc + thread->pc < KVX_DMA_TX_PGRM_MEM_SIZE);
        bundle = s->tx_pgrm_mem[base_pc + thread->pc];

        if (!kvx_dma_bundle_decode(bundle, &decoded)) {
            trace_kvx_dma_tx_thread_invalid_bundle(id, job->pgrm_id,
                                                   thread->pc, bundle);
            kvx_dma_tx_thread_report_error(s, thread, KVX_DMA_ERR_TX_BUNDLE);
            return;
        }

        if (trace_event_get_state_backends(TRACE_KVX_DMA_TX_THREAD_DISAS_BUNDLE)) {
            do_trace_bundle(id, thread, job, &decoded);
        }

        trace_kvx_dma_tx_thread_exec_bundle(id, job->pgrm_id, thread->pc);
        thread->pc = exec_bundle(s, thread, &decoded);

        run_budget--;
    }

    if (thread->running && !run_budget) {
        /*
         * The thread exhausted its budget. Schedule another run on the
         * iothread so the CPU thread can continue execution.
         */
        kvx_dma_schedule_run_tx_threads(s);
    }
}

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

    kvx_dma_run_tx_threads(s);
}

/*
 * Check if the given thread has a pending job in cache.
 * If it has one the thread can be started, swap the cached
 * and running job and mark the thread as running
 */
static inline void check_job_in_cache(KvxDmaState *s, KvxDmaTxThread *thread)
{
    KvxDmaTxJobContext *running_job;

    if (thread->running) {
        /* The thread is already running. Nothing to do */
        return;
    }

    if (!kvx_dma_tx_thread_has_cached_job(thread)) {
        /* No job in cache */
        return;
    }

    running_job = kvx_dma_tx_thread_get_running_job(thread);

    /*
     * Reset the running job (which is on the verge of becoming the cached job)
     * so that it becomes invalid and can be filled with a new job queue item.
     */
    running_job->valid = false;

    /* Swap jobs and run the thread */
    thread->running_job = !thread->running_job;
    enable_thread(s, thread);
}

void kvx_dma_run_tx_threads(KvxDmaState *s)
{
    size_t i;

    /* TODO: proper scheduling with QoS budget */
    for (i = 0; i < KVX_DMA_NUM_TX_THREAD; i++) {
        KvxDmaTxThread *thread = &s->tx_thread[i];

        check_job_in_cache(s, thread);
        run_thread(s, i, thread);
    }
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
