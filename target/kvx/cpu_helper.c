/*
 * QEMU Kalray kvx CPU
 *
 * Copyright (c) 2019-2020 GreenSocs SAS
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
#include "exec/exec-all.h"
#include "exec/address-spaces.h"
#include "cpu.h"
#include "internal.h"
#include "target/kvx/trace.h"

void QEMU_NORETURN kvx_raise_exception(CPUKVXState *env,
                                      uint32_t exception,
                                      uintptr_t pc)
{
    CPUState *cs = env_cpu(env);
    cs->exception_index = exception;
    cpu_loop_exit_restore(cs, pc);
}

static inline int kvx_irq_abs_prio_lvl(int pl, int il)
{
    /*
     * Absolute priority level of an IRQ:
     * the higher the most priority it is
     * See 5.1 of kv3 VLIW core architecture manual
     */
    return ((3 - pl) << 2) + il;
}

static uint64_t kvx_irq_get_pending_mask(CPUKVXState *env)
{
    uint64_t ilr, ile, mask;

    ilr = kvx_register_read_u64(env, REG_kv3_ILR);
    ile = kvx_register_read_u64(env, REG_kv3_ILE);

    mask = ilr & ile;

    return mask;
}

/*
 * @return a mask of IRQs owned by @pl
 */
static inline uint64_t get_irq_pl_mask(CPUKVXState *env, int pl)
{
    int i;
    uint64_t ret = 0;

    for (i = 0; i < 32; i++) {
        if (kvx_get_irq_pl(env, i) >= pl) {
            ret |= (1 << i);
        }
    }

    return ret;
}

static inline uint64_t refine_waitit_pending_irqs(CPUKVXState *env, uint64_t mask)
{
    uint64_t cur_pl_irqs, ret;
    int pl;
    bool wakeup = false;

    pl = kvx_get_current_pl(env);
    cur_pl_irqs = get_irq_pl_mask(env, pl) & mask;
    ret = ~cur_pl_irqs & mask;

    if (env->waitit_ctx.and_mask) {
        wakeup = (cur_pl_irqs & env->waitit_ctx.and_mask)
            == env->waitit_ctx.and_mask;
    }

    if (env->waitit_ctx.or_mask) {
        wakeup = wakeup || (cur_pl_irqs & env->waitit_ctx.or_mask);
    }

    if (wakeup) {
        /*
         * waitit conditions are fulfilled. Update the destination operand and
         * skip the waitit instruction. (so it effectively wakes up the CPU).
         */
        size_t reg = REG_kv3_R0 + env->waitit_ctx.dst_op;
        kvx_register_write_u64(env, reg, cur_pl_irqs);
        kvx_register_write_u64(env, REG_kv3_PC,
                               kvx_register_read_u64(env, REG_kv3_PC) + 4);
        env->sleep_state = KVX_RUNNING;

        /* Add the current PL IRQs to the pending mask */
        ret |= cur_pl_irqs;
    }

    return ret;
}

int kvx_irq_get_pending(CPUKVXState *env)
{
    int pl, il, cur_aipl, ie, irq = -1;
    uint64_t pending_mask = kvx_irq_get_pending_mask(env);

    if (!pending_mask) {
        return -1;
    }

    if (env->sleep_state == KVX_WAITIT) {
        pending_mask = refine_waitit_pending_irqs(env, pending_mask);
    }

    pl = kvx_get_current_pl(env);
    il = kvx_register_read_field(env, PS, IL);
    ie = kvx_register_read_field(env, PS, IE);
    cur_aipl = kvx_irq_abs_prio_lvl(pl, il);

    if (!ie) {
        /*
         * If PS.IE is unset, we do not take any interrupt in the current PL,
         * so raise IL so that this threshold is never met.
         */
        cur_aipl |= 0x3;
    }

    for (int i = 0; i < 32; i++) {
        if (pending_mask & (1u << i)) {
            int irq_aipl = kvx_irq_abs_prio_lvl(kvx_get_irq_pl(env, i),
                                                kvx_get_irq_ll(env, i));
            if (irq_aipl > cur_aipl) {
                irq = i;
                cur_aipl = irq_aipl;
            }
        }
    }

    trace_kvx_irq_get_pending(pl, il, ie, irq);

    return irq;
}

bool kvx_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        KVXCPU *cpu = KVX_CPU(cs);
        CPUKVXState *env = &cpu->env;
        int irq;

        irq = kvx_irq_get_pending(env);
        if (irq >= 0) {
            /* Acknowledge it */
            uint64_t es, ilr;

            trace_kvx_irq_ack(irq, kvx_get_irq_pl(env, irq), kvx_get_irq_ll(env, irq));

            /* Acknowledge the interrupt */
            ilr = kvx_register_read_u64(env, REG_kv3_ILR);
            ilr &= ~(((uint64_t) 1u) << irq);
            kvx_register_write_u64(env, REG_kv3_ILR, ilr);
            if (!ilr) {
                /* clear it flag if needed */
                cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
            }

            /* fill syndrome information */
            es = 0;
            es = KVX_FIELD_DP64(es, kv3_ES, ITN, irq);
            es = KVX_FIELD_DP64(es, kv3_ES, ITL, kvx_get_irq_ll(env, irq));
            /* TODO ES.ITI */
            env->excp_syndrome = es;

            /* Handle it */
            cs->exception_index = KVX_EXCP_INT;
            env->sleep_state = KVX_RUNNING;
            kvx_cpu_do_interrupt(cs);
            return true;
        }
    }
    return false;
}

static inline int kvx_get_syscall_target_pl(CPUKVXState *env)
{
    uint64_t syo = kvx_register_read_u64(env, REG_kv3_SYO);
    uint64_t sn = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, SN);

    return (syo >> ((sn >> 10) << 1)) & 0x3;
}

static inline int kvx_get_trap_target_pl(CPUKVXState *env,
                                         bool hw_trap_disabled)
{
    uint64_t htc;

    if (hw_trap_disabled) {
        return kvx_get_current_pl(env);
    }

    htc = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, HTC);

    switch (htc) {
        case TRAP_OPCODE:
            return kvx_register_read_field(env, HTO, OPC);
        case TRAP_PRIVILEGE:
            return env->excp_target_pl;
        case TRAP_DMISALIGN:
            return kvx_register_read_field(env, HTO, DMIS);
        case TRAP_PSYSERROR:
            return kvx_register_read_field(env, HTO, PSYS);
        case TRAP_DSYSERROR:
            return kvx_register_read_field(env, HTO, DSYS);
        case TRAP_PDECC:
        case TRAP_DDECC:
        case TRAP_PPAR:
        case TRAP_DPAR:
        case TRAP_TPAR:
            return kvx_register_read_field(env, HTO, DECCG);
        case TRAP_PSECC:
        case TRAP_DSECC:
            return kvx_register_read_field(env, HTO, SECCG);
        case TRAP_NOMAPPING:
            return kvx_register_read_field(env, HTO, NOMAP);
        case TRAP_PROTECTION:
            return kvx_register_read_field(env, HTO, PROT);
        case TRAP_WRITETOCLEAN:
            return kvx_register_read_field(env, HTO, W2CL);
        case TRAP_ATOMICTOCLEAN:
            return kvx_register_read_field(env, HTO, A2CL);
        case TRAP_VSFR:
            return kvx_register_read_field(env, HTO, VSFR);
        case TRAP_PL_OVERFLOW:
            return kvx_register_read_field(env, HTO, PLO);
        default:
            g_assert_not_reached();
    }
}

static inline int kvx_get_debug_target_pl(CPUKVXState *env)
{
    uint64_t dc;

    dc = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, DC);

    switch (dc) {
        uint64_t n;

    case DEBUG_BREAKPOINT:
        n = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, BN);
        return kvx_get_bp_pl(env, n);

    case DEBUG_WATCHPOINT:
        n = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, WN);
        return kvx_get_wp_pl(env, n);

    case DEBUG_STEP:
        return kvx_get_step_pl(env);

    case DEBUG_DSUBREAK:
    default:
        g_assert_not_reached();
    }
}

static int kvx_get_exception_target_pl(CPUKVXState *env, unsigned int excp,
                                       int pl, bool hw_trap_disabled)
{
    int target_pl;

    switch (excp) {
    case KVX_EXCP_DEBUG:
        target_pl = kvx_get_debug_target_pl(env);
        break;

    case KVX_EXCP_SYSCALL:
        target_pl = kvx_get_syscall_target_pl(env);
        break;

    case KVX_EXCP_INT:
        target_pl = kvx_get_irq_pl(env, KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, ITN));
        break;

    case KVX_EXCP_HW_TRAP:
        target_pl = kvx_get_trap_target_pl(env, hw_trap_disabled);
        break;

    case KVX_EXCP_DOUBLE_HW_TRAP:
    case KVX_EXCP_DOUBLE_INT:
    case KVX_EXCP_DOUBLE_SYSCALL:
        target_pl = kvx_register_read_field(env, HTO, DE);
        break;

    default:
        g_assert_not_reached();
    }

    /* The PL taking the trap cannot be less priviledged */
    return target_pl < pl ? target_pl : pl;
}

static inline uint64_t compute_exception_target_ps(CPUKVXState *env,
                                                   unsigned int excp,
                                                   uint64_t cur_ps,
                                                   int target_pl)
{
    int il = 0;

    if (excp == KVX_EXCP_INT || excp == KVX_EXCP_DOUBLE_INT) {
        il = kvx_get_irq_ll(env, KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, ITN));
    }

    cur_ps = KVX_FIELD_DP64(cur_ps, kv3_PS, PL, target_pl);
    cur_ps = KVX_FIELD_DP64(cur_ps, kv3_PS, ET, 1);
    cur_ps = KVX_FIELD_DP64(cur_ps, kv3_PS, IE, 0);
    cur_ps = KVX_FIELD_DP64(cur_ps, kv3_PS, IL, il);
    cur_ps = KVX_FIELD_DP64(cur_ps, kv3_PS, HLE, 0);
    cur_ps = KVX_FIELD_DP64(cur_ps, kv3_PS, HTD, 0);
    cur_ps = KVX_FIELD_DP64(cur_ps, kv3_PS, DAUS, 0);

    return cur_ps;
}

static inline uint64_t compute_exception_target_sps(CPUKVXState *env,
                                                    unsigned int excp,
                                                    uint64_t val,
                                                    int target_pl)
{
    val = KVX_FIELD_DP64(val, kv3_PS, PL, kvx_get_current_pl(env) - target_pl);

    /*
     * Handle stepping over syscall case, we need to check that:
     * + we are leaving step mode because of pl change
     * + we are taking a syscall
     */
    if (kvx_step_mode_enabled(env) &&
        target_pl <= kvx_get_step_pl(env) &&
        excp == KVX_EXCP_SYSCALL) {
        val = KVX_FIELD_DP64(val, kv3_PS, SMR, 1);
    }

    return val;
}

static inline uint64_t compute_exception_pc(CPUKVXState *env,
                                            unsigned int excp,
                                            int target_pl,
                                            bool hw_trap_disabled)
{
    uint64_t new_pc;

    if (hw_trap_disabled) {
        /* Skip the bundle that raised the exception */
        uint64_t bundle_size = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES_PL0, BS);
        return kvx_register_read_u64(env, REG_kv3_PC) + bundle_size * 4;
    }

    new_pc = kvx_register_read_u64(env, REG_kv3_EV_PLx(target_pl));

    switch (excp) {
    case KVX_EXCP_DEBUG:
        break;

    case KVX_EXCP_HW_TRAP:
    case KVX_EXCP_DOUBLE_SYSCALL:
    case KVX_EXCP_DOUBLE_INT:
    case KVX_EXCP_DOUBLE_HW_TRAP:
        new_pc += 0x40;
        break;

    case KVX_EXCP_INT:
        new_pc += 0x80;
        break;

    case KVX_EXCP_SYSCALL:
        new_pc += 0xc0;
        break;

    default:
        g_assert_not_reached();
    }

    return new_pc;
}


static inline uint64_t compute_exception_syndrome(CPUKVXState *env,
                                                  unsigned int excp,
                                                  int cur_pl,
                                                  int target_pl,
                                                  int prim_pl,
                                                  bool hw_trap_disabled)
{
    /*
     * cur_pl: PL before taking the exception
     * prim_pl: PL that would normally take the exception
     * target_pl: PL that will take the exception (after double exception case
     *            handling)
     * in 'normal' exception cases (no double exception) prim_pl == target_pl
     */
    uint64_t ret = env->excp_syndrome;

    ret |= (excp << KVX_FIELD_SHIFT(kv3_ES, EC))
        | (cur_pl << KVX_FIELD_SHIFT(kv3_ES, OAPL))
        | ((cur_pl - target_pl) << KVX_FIELD_SHIFT(kv3_ES, ORPL))
        | (prim_pl << KVX_FIELD_SHIFT(kv3_ES, PTAPL))
        | ((prim_pl - target_pl) << KVX_FIELD_SHIFT(kv3_ES, PTRPL))
        | ((uint64_t)hw_trap_disabled << KVX_FIELD_SHIFT(kv3_ES, PL0_DHT));

    /* For watchpoints, handle the UCA flag here */
    if ((excp == KVX_EXCP_DEBUG) &&
        (KVX_FIELD_EX64(ret, kv3_ES, DC) == DEBUG_WATCHPOINT)) {
        uint64_t ps = get_mmu_ps(env, false);

        if (!KVX_FIELD_EX64(ps, kv3_PS, DCE)) {
            ret = KVX_FIELD_DP64(ret, kv3_ES, UCA, 1);
        }
    }

    return ret;
}

static inline uint64_t compute_exception_address(CPUKVXState *env,
                                                 unsigned int excp)
{
    if (excp == KVX_EXCP_DEBUG) {
        uint64_t dc = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, DC);

        switch (dc) {
        case DEBUG_WATCHPOINT:
        case DEBUG_STEP:
            return env->excp_address;
        }
    } else if ((excp & ~KVX_EXCP_DOUBLE) == KVX_EXCP_HW_TRAP) {
        uint64_t ec = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, HTC);

        switch (ec) {
        case TRAP_DMISALIGN:
        case TRAP_PSYSERROR:
        case TRAP_DSYSERROR:
        case TRAP_NOMAPPING:
        case TRAP_PROTECTION:
        case TRAP_WRITETOCLEAN:
        case TRAP_ATOMICTOCLEAN:
        case TRAP_TPAR:
            return env->excp_address;
        }
    }

    return 0;
}

void kvx_cpu_do_interrupt(CPUState *cs)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;
    int cur_pl, target_pl, prim_pl;
    uint64_t cur_ps, new_ps;
    uint64_t cur_pc, new_pc;
    uint64_t cur_sps, new_sps;
    uint64_t es;
    unsigned int excp;
    bool hw_trap_disabled = false;

    excp = cs->exception_index;

    cur_pl = kvx_get_current_pl(env);
    cur_ps = kvx_register_read_u64(env, REG_kv3_PS);

    if (KVX_FIELD_EX64(cur_ps, kv3_PS, HTD) && (excp == KVX_EXCP_HW_TRAP)) {
        hw_trap_disabled = true;
    }

    prim_pl = kvx_get_exception_target_pl(env, excp, cur_pl, hw_trap_disabled);
    target_pl = prim_pl;

retry_excp:
    /* Note: previous calls should guarantee this */
    g_assert(cur_pl >= target_pl);

    if (KVX_FIELD_EX64(cur_ps, kv3_PS, ET) && cur_pl == target_pl) {
        /* we must not trigger debug exception on the same pl */
        g_assert(excp != KVX_EXCP_DEBUG);

        if (excp & KVX_EXCP_DOUBLE) {
            /* Reset the core */
            qemu_log_mask(LOG_GUEST_ERROR,
                          "core %d: horizontal exception with PS.ET=1, "
                          "stopping execution\n", cpu->cfg.pid);
            cpu_reset(cs);
            cs->halted = true;
            cs->exception_index = EXCP_HLT;
            env->sleep_state = KVX_RESETTING;

            cpu_loop_exit(cs);
        }

        /*
         * We now have a double exception. We need to compute the new target PL
         * and check if it can actually handle it.
         */
        excp |= KVX_EXCP_DOUBLE;
        target_pl = kvx_get_exception_target_pl(env, excp, cur_pl,
                                                hw_trap_disabled);
        goto retry_excp;
    }

    if (!hw_trap_disabled) {
        cur_sps = kvx_register_read_u64(env, REG_kv3_SPS_PLx(target_pl));

        /*
         * PS <- SPS_PL<j> except for the PL field and the FOE0/1 bits set by
         * hardware
         */
        new_ps = (cur_pl == target_pl) ? cur_ps : cur_sps;
        new_ps = compute_exception_target_ps(env, excp, new_ps, target_pl);

        /*
         * SPS_PL<j> <- PS (atomic with the preceding step, this is really a swap).
         */
        new_sps = compute_exception_target_sps(env, excp, cur_ps, target_pl);

        /* We do the real swap here so that previous call can still use the env */
        kvx_register_write_u64(env, REG_kv3_SPS_PLx(target_pl), new_sps);
        kvx_register_write_u64(env, REG_kv3_PS, new_ps);
    } else {
        new_ps = cur_ps;
        new_sps = cur_sps = 0;
    }

    /*
     * PC <- handler PC based on EV_PL<j> (see Section 2.3.28).
     */
    cur_pc = kvx_register_read_u64(env, REG_kv3_PC);
    new_pc = compute_exception_pc(env, excp, target_pl, hw_trap_disabled);

    kvx_register_write_u64(env, REG_kv3_PC, new_pc);

    if (!hw_trap_disabled) {
        if (excp == KVX_EXCP_SYSCALL) {
            /*
             * No need to check BS as a scall instruction is always alone in
             * its bundle.
             */
            cur_pc += 4;
        }

        /*
         * SPC_PL<j> <- PC (atomic with the preceding step, this is really a swap).
         */
        kvx_register_write_u64(env, REG_kv3_SPC_PLx(target_pl), cur_pc);
    }

    /*
     * ES_PL<j> <- filled with relevant information by hardware.
     */
    es = compute_exception_syndrome(env, excp, cur_pl, target_pl,
                                    prim_pl, hw_trap_disabled);
    kvx_register_write_u64(env, REG_kv3_ES_PLx(target_pl), es);

    /*
     * EA_PL<j> <- filled with the exception address, as usual.
     */
    kvx_register_write_u64(env, REG_kv3_EA_PLx(target_pl),
                           compute_exception_address(env, excp));

    kvx_update_cpu_state(env, cur_ps ^ new_ps, cur_sps ^ new_sps);
}

static inline void kvx_ipe_helper_transaction_begin(KVXCPU *cpu)
{
    KvxIpeHelper *ipe_helper;

    ipe_helper = cpu->ipe_helper;
    g_assert(ipe_helper);

    if (!ipe_helper->transaction_started) {
        /* Start a new transaction */
        ipe_helper->transaction_started = true;
        ipe_helper->loop = false;
        ipe_helper->initiator = cpu;
        return;
    }

    if (ipe_helper->initiator == cpu) {
        /* Loop detected */
        ipe_helper->loop = true;
    }
}

static inline void kvx_ipe_helper_transaction_end(KVXCPU *cpu)
{
    KvxIpeHelper *ipe_helper;

    ipe_helper = cpu->ipe_helper;
    g_assert(ipe_helper);
    g_assert(ipe_helper->transaction_started);

    if (ipe_helper->loop) {
        /*
         * If we entered the loop state, the initiator CPU should not carry on
         * with event forwarding. It must ends the transaction immediately
         * after loop detection.
         */
        g_assert(ipe_helper->initiator == cpu);
        ipe_helper->loop = false;
        return;
    }

    if (ipe_helper->initiator == cpu) {
        ipe_helper->transaction_started = false;
    }
}

static inline bool kvx_ipe_helper_loop_detected(KVXCPU *cpu)
{
    KvxIpeHelper *ipe_helper;

    ipe_helper = cpu->ipe_helper;
    g_assert(ipe_helper);
    g_assert(ipe_helper->transaction_started);

    return ipe_helper->loop;
}

static inline void send_one_ipe_event(CPUKVXState *env, qemu_irq irq)
{
    KVXCPU *cpu = env_archcpu(env);

    kvx_ipe_helper_transaction_begin(cpu);

    if (!kvx_ipe_helper_loop_detected(cpu)) {
        qemu_irq_pulse(irq);
    }

    kvx_ipe_helper_transaction_end(cpu);
}

static inline void send_ipe_events_on(CPUKVXState *env,
                                      qemu_irq *line, uint16_t mask)
{
    size_t i = 0;

    while (mask) {
        if (mask & 1) {
            send_one_ipe_event(env, line[i]);
        }

        mask >>= 1;
        i++;
    }
}

void kvx_send_ipe_events(CPUKVXState *env,
                         uint16_t fwd_mask, uint16_t bwd_mask)
{
    KVXCPU *cpu = env_archcpu(env);

    if (kvx_cpu_is_rm(cpu)) {
        /* RM core has no IPE connections */
        return;
    }

    trace_kvx_ipe_send_event(fwd_mask, bwd_mask);
    send_ipe_events_on(env, env->ipe_fwd_out, fwd_mask);
    send_ipe_events_on(env, env->ipe_bwd_out, bwd_mask);

}

void kvx_handle_ipe_event(CPUKVXState *env, bool forward, unsigned int idx)
{
    KVXCPU *cpu = env_archcpu(env);
    uint64_t *ipe = kvx_register_ptr_u64(env, REG_kv3_IPE);
    int ev_shift = forward ? idx : idx + KVX_NUM_IPE;

    g_assert(idx < KVX_NUM_IPE);

    trace_kvx_ipe_event(cpu->cfg.pid, forward ? 'f' : 'b', idx);

    /* register the event */
    *ipe |= 1ull << ev_shift;

    /* notify neighborhood if needed */
    if (*ipe & (1ull << (ev_shift + (KVX_NUM_IPE * 2)))) {
        qemu_irq *line = forward ? env->ipe_fwd_out : env->ipe_bwd_out;
        send_one_ipe_event(env, line[idx]);
    }

    /* wakeup the core if needed */
    if (env->sleep_state == KVX_SYNCGROUP) {
        uint32_t wait_mask = env->syncgroup_ctx.wait_mask;

        if ((*ipe & wait_mask) == wait_mask) {
            uint64_t pc;

            *ipe &= ~(uint64_t) wait_mask;

            kvx_send_ipe_events(env, env->syncgroup_ctx.notify_fwd,
                                env->syncgroup_ctx.notify_bwd);

            /* skip the syncgroup instruction */
            pc = kvx_register_read_u64(env, REG_kv3_PC);
            kvx_register_write_u64(env, REG_kv3_PC, pc + 4);

            env->sleep_state = KVX_RUNNING;
            qemu_cpu_kick(env_cpu(env));
        }
    }
}

static void tb_invalidate_bundle(CPUKVXState *env, target_ulong vaddr)
{
    int mmu_idx;
    TLBLookupInfo info;
    TLBLookupStatus ret;

    mmu_idx = kvx_cpu_mmu_index(env, true);

    ret = get_physical_addr(env, vaddr, MMU_INST_FETCH, mmu_idx, &info);

    if (ret == TLB_LOOKUP_SUCCESS) {
        tb_invalidate_phys_addr(&address_space_memory, info.paddr,
                                MEMTXATTRS_UNSPECIFIED);
    }
}

static void update_hardware_loop(CPUKVXState *env)
{

    if (!kvx_in_hardware_loop_ctx(env)) {
        return;
    }

    if (env->le_is_dirty) {
        uint64_t le = kvx_register_read_u64(env, REG_kv3_LE);

        tb_invalidate_bundle(env, le - 4);
        env->le_is_dirty = false;
    }
}

void kvx_update_cpu_state(CPUKVXState *env, uint64_t ps_mask, uint64_t sps_mask)
{
    uint64_t prev_ps, cur_ps;
    uint64_t prev_sps, cur_sps;
    uint64_t prev_mmu_ps, cur_mmu_ps;

    cur_ps = kvx_register_read_u64(env, REG_kv3_PS);
    prev_ps = cur_ps ^ ps_mask;

    cur_sps = kvx_register_read_aliased_u64(env, REG_kv3_SPS);
    prev_sps = cur_sps ^ sps_mask;

    prev_mmu_ps = KVX_FIELD_EX64(prev_ps, kv3_PS, DAUS)
        ? prev_sps
        : prev_ps;

    cur_mmu_ps = KVX_FIELD_EX64(cur_ps, kv3_PS, DAUS)
        ? cur_sps
        : cur_ps;

    if ((prev_mmu_ps ^ cur_mmu_ps) & KVX_FIELD_MASK(kv3_PS, V64)) {
        tlb_flush(env_cpu(env));
    }

    if (ps_mask & KVX_FIELD_MASK(kv3_PS, HLE)) {
        update_hardware_loop(env);
    }

    if (ps_mask & KVX_FIELD_MASK(kv3_PS, PL)) {
        kvx_update_breakpoints(env);
    }
}

void kvx_reg_write_shouldnothappen(KVXCPU *cpu, Register reg, uint64_t val)
{
    fprintf(stderr, "Writing register '%s' should not happen\n",
            REGISTERS[reg].name);
    abort();
}

uint64_t kvx_reg_read_shouldnothappen(KVXCPU *cpu, Register reg)
{
    fprintf(stderr, "Reading register '%s' should not happen\n",
            REGISTERS[reg].name);
    abort();
}

static inline bool apply_rpl_field(RegisterField field,
                                   uint64_t *val, uint64_t mask,
                                   int pl)
{
    const RegisterFieldDescr *descr = &REGISTERFIELDS[field];
    uint64_t field_mask = MAKE_64BIT_MASK(descr->offset, descr->width);
    uint64_t rpl;

    g_assert(descr->width == 2);

    /*
     * If the field is untouched, we have nothing to do.
     */
    if (!(mask & field_mask)) {
        return true;
    }

    /*
     * Compute the relative PL we will apply.
     * + if new_pl_bit0 is set we do +1
     * + if new_pl_bit1 is set we do +2
     * It is cumulative (we may do +3).
     * Note: a bit which is written as 0 (clear),
     * is equivalent not to write it (unset in mask).
     */
    rpl = (*val & mask & field_mask) >> descr->offset;
    rpl += pl;
    if (rpl > 3) {
        // PL overflow
        return false;
    }

    /* replace the new PL: it may change unmasked bit of the field */
    *val &= ~field_mask;
    *val += rpl << descr->offset;
    return true;
}

bool kvx_reg_apply_rpl_ps(CPUKVXState *env, Register reg, uint64_t *val,
                          uint64_t mask)
{
    int pl = kvx_get_current_pl(env);
    return apply_rpl_field(REGFIELD_kv3_PS_PL, val, mask, pl);
}

bool kvx_reg_apply_rpl_owners(CPUKVXState *env, Register reg, uint64_t *val,
                              uint64_t mask)
{
    const RegisterDescr *rdescr = &REGISTERS[reg];
    int pl = kvx_get_current_pl(env);
    bool res = true;

    /* all fields are PL fields we need to handle */
    for (int i = 0; i < rdescr->n_fields; i++) {
        res = res && apply_rpl_field(rdescr->fields[i], val, mask, pl);
    }

    return res;
}

void kvx_reg_write_ps(KVXCPU *cpu, Register reg, uint64_t val)
{
    CPUKVXState *env = &cpu->env;
    uint64_t cur;

    cur = kvx_register_read_u64(env, REG_kv3_PS);
    kvx_register_write_u64(env, REG_kv3_PS, val);

    kvx_update_cpu_state(env, cur ^ val, 0);
}

void kvx_reg_write_sps(KVXCPU *cpu, Register reg, uint64_t val)
{
    CPUKVXState *env = &cpu->env;
    int cur_pl = kvx_get_current_pl(env);
    int reg_pl = reg - REG_kv3_SPS_PL0;
    uint64_t cur = kvx_register_read_u64(env, reg);

    kvx_register_write_u64(env, reg, val);

    if (cur_pl == reg_pl) {
        kvx_update_cpu_state(env, 0, cur ^ val);
    }
}

void kvx_reg_write_csit(KVXCPU *cpu, Register reg, uint64_t val)
{
    kvx_register_write_u64(&cpu->env, reg, val);
}

void kvx_reg_write_lc_le(KVXCPU *cpu, Register reg, uint64_t val)
{
    kvx_register_write_u64(&cpu->env, reg, val);
    cpu->env.le_is_dirty = true;
    update_hardware_loop(&cpu->env);
}

void kvx_reg_write_mmc(KVXCPU *cpu, Register reg, uint64_t val)
{
    CPUKVXState *env = &cpu->env;
    uint64_t cur;

    cur = kvx_register_read_u64(env, REG_kv3_MMC);
    kvx_register_write_u64(env, REG_kv3_MMC, val);

    if ((val ^ cur) & KVX_FIELD_MASK(kv3_MMC, ASN)) {
        /* ASN changed, flush the QEMU TLB */
        tlb_flush(CPU(cpu));
    }
}

void kvx_reg_write_ilr(KVXCPU *cpu, Register reg, uint64_t val)
{
    CPUKVXState *env = &cpu->env;
    uint64_t old;

    old = kvx_register_read_u64(env, REG_kv3_ILR);
    kvx_register_write_u64(env, REG_kv3_ILR, val);

    if (old ^ val) {
        trace_kvx_irq_update(old, val);
        /* Value changed, recompute pending irqs */
        CPUState *cs = CPU(cpu);
        if (val) {
            cpu_interrupt(cs, CPU_INTERRUPT_HARD);
        } else {
            cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
        }
    }
}

void kvx_reg_write_tcr(KVXCPU *cpu, Register reg, uint64_t val)
{
    kvx_register_write_u64(&cpu->env, REG_kv3_TCR, val);
    kvx_cpu_update_timers(cpu);
}

void kvx_reg_write_txv_wdv(KVXCPU *cpu, Register reg, uint64_t val)
{
    switch (reg) {
    case REG_kv3_T0V:
        kvx_timer_write_value(&cpu->timer[0], val);
        return;
    case REG_kv3_T1V:
        kvx_timer_write_value(&cpu->timer[1], val);
        return;
    case REG_kv3_WDV:
        kvx_timer_write_value(&cpu->watchdog, val);
        return;
    default:
        g_assert_not_reached();
    }
}

uint64_t kvx_reg_read_tcr_txv_wdv(KVXCPU *cpu, Register reg)
{
    kvx_cpu_update_timers(cpu);
    return kvx_register_read_u64(&cpu->env, reg);
}

static inline int pmreg_index(Register reg)
{
    switch (reg) {
    case REG_kv3_PM0:
        return 0;
    case REG_kv3_PM1:
        return 1;
    case REG_kv3_PM2:
        return 2;
    case REG_kv3_PM3:
        return 3;
    default:
        g_assert_not_reached();
    }
}

uint64_t kvx_reg_read_pmcnt(KVXCPU *cpu, Register reg)
{
    int mode, n = pmreg_index(reg);
    uint64_t pmc, cnt;
    int64_t now;

    pmc = kvx_register_read_u64(&cpu->env, REG_kv3_PMC);
    switch (n) {
        case 0:
            mode = KVX_FIELD_EX64(pmc, kv3_PMC, PM0C);
            break;
        case 1:
            mode = KVX_FIELD_EX64(pmc, kv3_PMC, PM1C);
            break;
        case 2:
            mode = KVX_FIELD_EX64(pmc, kv3_PMC, PM2C);
            break;
        case 3:
            mode = KVX_FIELD_EX64(pmc, kv3_PMC, PM3C);
            break;
        default:
            g_assert_not_reached();
    }

    cnt = kvx_register_read_u64(&cpu->env, reg);
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * basic implementation aproximating some counters using the virtual
     * clock
     */
    switch (mode) {
    case 0: /* PCC Processor Cycle Count */
    case 2: /* EBE Executed Bundle event */
    case 3: /* ENIE Executed N Instructions event */
        cnt += (now - cpu->pm_base[n]) / kvx_cpu_clock_period_ns();
        break;
    case 48: /* SE Stop Event */
    case 49: /* RE Reset Event */
    default:
        break;
    }

    return cnt;
}

void kvx_reg_write_pmcnt(KVXCPU *cpu, Register reg, uint64_t val)
{
    int n = pmreg_index(reg);

    kvx_register_write_u64(&cpu->env, reg, val);
    cpu->pm_base[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

void kvx_reg_write_pmctl(KVXCPU *cpu, Register reg, uint64_t val)
{
    uint64_t pmc = kvx_register_read_u64(&cpu->env, REG_kv3_PMC);

    /* update counters */
    for (int n = 0; n < 4; n++) {
        uint64_t cnt;
        int old, new;
        Register reg;
        switch (n) {
            case 0:
                old = KVX_FIELD_EX64(pmc, kv3_PMC, PM0C);
                new = KVX_FIELD_EX64(val, kv3_PMC, PM0C);
                reg = REG_kv3_PM0;
                break;
            case 1:
                old = KVX_FIELD_EX64(pmc, kv3_PMC, PM1C);
                new = KVX_FIELD_EX64(val, kv3_PMC, PM1C);
                reg = REG_kv3_PM1;
                break;
            case 2:
                old = KVX_FIELD_EX64(pmc, kv3_PMC, PM2C);
                new = KVX_FIELD_EX64(val, kv3_PMC, PM2C);
                reg = REG_kv3_PM2;
                break;
            case 3:
                old = KVX_FIELD_EX64(pmc, kv3_PMC, PM3C);
                new = KVX_FIELD_EX64(val, kv3_PMC, PM3C);
                reg = REG_kv3_PM3;
                break;
            default:
                g_assert_not_reached();
        }
        if (old == new) {
            continue;
        }
        /* get the current counter value */
        if (new == 49) { /* Reset Event */
            cnt = 0;
        } else {
            cnt = kvx_reg_read_pmcnt(cpu, reg);
        }
        /* update the counter value */
        kvx_reg_write_pmcnt(cpu, reg, cnt);
    }

    /* write the new control value */
    kvx_register_write_u64(&cpu->env, REG_kv3_PMC, val);
}

void kvx_reg_write_simple(KVXCPU *cpu, Register reg, uint64_t val)
{
    kvx_register_write_u64(&cpu->env, reg, val);
}

void kvx_register_write(KVXCPU *cpu, Register reg, uint64_t val)
{
    const RegisterDescr *rdescr;
    int pl = kvx_get_current_pl(&cpu->env);
    const RegisterInfo *info;
    reg = kvx_register_alias_pl(reg, pl);
    info = kvx_register_info(reg);

    /* first do any masking */
    rdescr = &REGISTERS[reg];
    if (rdescr->reg_width < 64) {
        val &= (1llu << rdescr->reg_width) - 1;
    }
    val &= rdescr->mask;

    /* handle address sign extension and alignment */
    if (info->addr_store_mask) {
        uint64_t mask = info->addr_store_mask;

        val <<= 64 - KVX_ADDRESS_LENGTH;
        val = (int64_t) val >> (64 - KVX_ADDRESS_LENGTH);

        if (info->v64_aware) {
            mask &= 0xffffffffull;
        }

        val &= info->addr_store_mask;
    }

    /*
     * At this point, the iothread mutex must be owned if the register
     * access requires it.
     * For calls from TCG, it is done in the helper.
     */
    if (info->write) {
        info->write(cpu, reg, val);
    } else {
        kvx_register_write_u64(&cpu->env, reg, val);
    }
}

uint64_t kvx_register_read(KVXCPU *cpu, Register reg)
{
    int pl = kvx_get_current_pl(&cpu->env);
    const RegisterInfo *info;
    reg = kvx_register_alias_pl(reg, pl);
    info = kvx_register_info(reg);

    /*
     * At this point, the iothread mutex must be owned if the register
     * access requires it.
     * For calls from TCG, it is done in the helper.
     */
    if (info->read) {
        return info->read(cpu, reg);
    } else {
        return kvx_register_read_u64(&cpu->env, reg);
    }
}
