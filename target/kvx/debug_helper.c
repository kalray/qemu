/*
 * QEMU Kalray kvx CPU
 *
 * Copyright (c) 2020 GreenSocs SAS
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
#include "exec/helper-proto.h"
#include "cpu.h"
#include "internal.h"
#include "target/kvx/trace.h"

static inline bool kvx_get_bp_en(CPUKVXState *env, int n)
{
    return n ? kvx_register_read_field(env, DC, BE1)
             : kvx_register_read_field(env, DC, BE0);
}

static inline bool kvx_get_wp_en(CPUKVXState *env, int n)
{
    return n ? kvx_register_read_field(env, DC, WE1)
             : kvx_register_read_field(env, DC, WE0);
}

static inline void compute_xp_addr_len(int range, uint64_t *addr,
                                       uint64_t *len)
{
    /* range is in [0;63] so we may have up to a 2**63 length only */
    *len = (1ull << range);
    *addr &= ~(*len - 1);
}

static inline void kvx_get_bp_addr_len(CPUKVXState *env, int n,
                                       uint64_t *addr, uint64_t *len)
{
    int range = n ? kvx_register_read_field(env, DC, BR1)
                  : kvx_register_read_field(env, DC, BR0);
    *addr = n ? kvx_register_read_u64(env, REG_kv3_DBA1)
              : kvx_register_read_u64(env, REG_kv3_DBA0);
    compute_xp_addr_len(range, addr, len);
}

static inline void kvx_get_wp_addr_len(CPUKVXState *env, int n,
                                       uint64_t *addr, uint64_t *len)
{
    int range = n ? kvx_register_read_field(env, DC, WR1)
                  : kvx_register_read_field(env, DC, WR0);
    *addr = n ? kvx_register_read_u64(env, REG_kv3_DWA1)
              : kvx_register_read_u64(env, REG_kv3_DWA0);
    compute_xp_addr_len(range, addr, len);
}

static void update_breakpoint(CPUKVXState *env, int n)
{
    bool enable = kvx_get_bp_en(env, n);

    /*
     * Note: QEMU does not support ranged breakpoint so we cannot uses
     * cpu_breakpoint_insert and cpu_breakpoint_remove_by_ref
     */

    /* Remove the breakpoint if it changes */
    if (env->breakpoint[n].enabled || enable) {
        tb_flush(env_cpu(env));
        env->breakpoint[n].enabled = false;
    }

    /* Add the new breakpoint */
    if (enable) {
        uint64_t addr, len;
        kvx_get_bp_addr_len(env, n, &addr, &len);
        env->breakpoint[n].addr_low = addr;
        env->breakpoint[n].addr_high = addr + len - 1;
        env->breakpoint[n].enabled = true;
    }
}

void kvx_update_breakpoints(CPUKVXState *env)
{
    update_breakpoint(env, 0);
    update_breakpoint(env, 1);
}

static void update_watchpoint(CPUKVXState *env, int n)
{
    KVXCPU *cpu = KVX_CPU(env_cpu(env));
    bool enable = kvx_get_wp_en(env, n);

    if (env->watchpoint[n]) {
        cpu_watchpoint_remove_by_ref(CPU(cpu), env->watchpoint[n]);
        env->watchpoint[n] = NULL;
    }

    /* Add the new watchpoint */
    if (enable) {
        int flags = BP_CPU | BP_STOP_BEFORE_ACCESS | BP_MEM_WRITE;
        uint64_t addr, len;
        kvx_get_wp_addr_len(env, n, &addr, &len);
        cpu_watchpoint_insert(CPU(cpu), addr, len, flags, &env->watchpoint[n]);
    }
}

void kvx_reg_write_dc(KVXCPU *cpu, Register reg, uint64_t val)
{
    uint64_t old = kvx_register_read_u64(&cpu->env, REG_kv3_DC);
    kvx_register_write_u64(&cpu->env, REG_kv3_DC, val);

    if ((old ^ val) & (KVX_FIELD_MASK(kv3_DC, BE0) | KVX_FIELD_MASK(kv3_DC, BR0))) {
        update_breakpoint(&cpu->env, 0);
    }
    if ((old ^ val) & (KVX_FIELD_MASK(kv3_DC, BE1) | KVX_FIELD_MASK(kv3_DC, BR1))) {
        update_breakpoint(&cpu->env, 1);
    }
    if ((old ^ val) & (KVX_FIELD_MASK(kv3_DC, WE0) | KVX_FIELD_MASK(kv3_DC, WR0))) {
        update_watchpoint(&cpu->env, 0);
    }
    if ((old ^ val) & (KVX_FIELD_MASK(kv3_DC, WE1) | KVX_FIELD_MASK(kv3_DC, WR1))) {
        update_watchpoint(&cpu->env, 1);
    }
}

void kvx_reg_write_dba(KVXCPU *cpu, Register reg, uint64_t val)
{
    kvx_register_write_u64(&cpu->env, reg, val);
    switch (reg) {
    case REG_kv3_DBA0:
        update_breakpoint(&cpu->env, 0);
        return;
    case REG_kv3_DBA1:
        update_breakpoint(&cpu->env, 1);
        return;
    default:
        g_assert_not_reached();
    }
}

void kvx_reg_write_dwa(KVXCPU *cpu, Register reg, uint64_t val)
{
    kvx_register_write_u64(&cpu->env, reg, val);
    switch (reg) {
    case REG_kv3_DWA0:
        update_watchpoint(&cpu->env, 0);
        return;
    case REG_kv3_DWA1:
        update_watchpoint(&cpu->env, 1);
        return;
    default:
        g_assert_not_reached();
    }
}
static int check_dbg_bp(CPUKVXState *env, int n)
{
    const struct CPUKVXBreakpoint *bp = &env->breakpoint[n];
    uint64_t pc = kvx_register_read_u64(env, REG_kv3_PC);
    int bp_pl, cur_pl = kvx_get_current_pl(env);

    switch (n) {
    case 0:
        bp_pl = kvx_register_read_field(env, DO, B0);
        break;
    case 1:
        bp_pl = kvx_register_read_field(env, DO, B1);
        break;
    default:
        g_assert_not_reached();
    }

    if ((cur_pl > bp_pl) && bp->enabled &&
            pc >= bp->addr_low && pc <= bp->addr_high) {
        return bp_pl;
    }

    return -1;
}

static void QEMU_NORETURN raise_debug(CPUKVXState *env, unsigned cause, uint64_t syndrome)
{
    env->excp_syndrome = KVX_FIELD_DP64(syndrome, kv3_ES, DC, cause);
    HELPER(raise_exception)(env, KVX_EXCP_DEBUG);
}

void HELPER(raise_debug_step)(CPUKVXState *env)
{
    uint64_t syndrome = env->excp_syndrome;
    syndrome &= KVX_FIELD_MASK(kv3_ES, SFRI) |
                KVX_FIELD_MASK(kv3_ES, GPRP) |
                KVX_FIELD_MASK(kv3_ES, SFRP) |
                KVX_FIELD_MASK(kv3_ES, DHT) |
                KVX_FIELD_MASK(kv3_ES, RWX) |
                KVX_FIELD_MASK(kv3_ES, NTA) |
                KVX_FIELD_MASK(kv3_ES, UCA) |
                KVX_FIELD_MASK(kv3_ES, AS) |
                KVX_FIELD_MASK(kv3_ES, BS) |
                KVX_FIELD_MASK(kv3_ES, DRI) |
                KVX_FIELD_MASK(kv3_ES, PIC);
    /*
     * EC, *PL and DC fields will be filled later by
     * the helper or exception function.
     */
    raise_debug(env, DEBUG_STEP, syndrome);
}

void HELPER(check_raise_debug_breakpoint)(CPUKVXState *env, uint64_t syndrome)
{
    int bp_n = -1, bp_pl = 4; /* > valid PL */

    for (int i = 0; i < 2; i++) {
        int pl = check_dbg_bp(env, i);
        if (pl >= 0 && pl < bp_pl) {
            bp_n = i;
            bp_pl = pl;
        }
    }

    if (bp_n >= 0) {
        /* we've hit a breakpoint */
        uint64_t es = KVX_FIELD_DP64(syndrome, kv3_ES, BN, bp_n);
        raise_debug(env, DEBUG_BREAKPOINT, es);
    }
}

void HELPER(check_step_mode_ready)(CPUKVXState *env)
{
    if (kvx_register_read_field(env, PS, SMR)) {
        /*
         * If we are here, it is probably because we just rfe from a
         * scall handler.
         * Clear SMR and setup a 4bytes bundle syndrome (like scall).
         */
        uint64_t es = KVX_FIELD_DP64(0, kv3_ES, BS, 1);
        kvx_register_write_field(env, PS, SMR, 0);
        env->excp_address = 0;
        raise_debug(env, DEBUG_STEP, es);
    }
}

void kvx_debug_excp_handler(CPUState *cs)
{
    /*
     * Called by core code when a watchpoint or breakpoint fires;
     * need to check which one and raise the appropriate exception.
     *
     * Note: we do not use CPUBreakpoint for HW breakpoint because we
     * need the range support. So we do not end up here for this case.
     */
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;
    CPUWatchpoint *wp_hit = cs->watchpoint_hit;

    if (wp_hit) {
        if (wp_hit->flags & BP_CPU) {
            uint64_t es;
            int wp_n = wp_hit == env->watchpoint[0] ? 0 : 1;

            cs->watchpoint_hit = NULL;

            es = env->excp_syndrome;
            es = KVX_FIELD_DP64(es, kv3_ES, WN, wp_n);

            env->excp_address = wp_hit->hitaddr;
            raise_debug(env, DEBUG_WATCHPOINT, es);
        }
    } else {
        /*
         * (1) GDB breakpoints should be handled first.
         * (2) Do not raise a CPU exception if no CPU breakpoint has fired,
         * since singlestep is also done by generating a debug internal
         * exception.
         */
        /*
         * Note: we do not have CPU breakpoint since we do not use
         * CPUBreakpoint to implement them. We can just return.
         */
        return;
    }
}

static bool kvx_wp_matches(CPUKVXState *env, int n, uint64_t addr, uint64_t len)
{
    uint64_t wp_addr, wp_len, wp_high, high;

    if (!kvx_get_wp_en(env, n)) {
        return false;
    }

    if (kvx_get_wp_pl(env, n) >= kvx_get_current_pl(env)) {
        return false;
    }

    kvx_get_wp_addr_len(env, n, &wp_addr, &wp_len);

    wp_high = wp_addr + wp_len - 1;
    high = addr + len - 1;
    if (addr > wp_high || high < wp_addr) {
        return false;
    }

    return true;
}

vaddr kvx_adjust_watchpoint_address(CPUState *cs, vaddr addr, int len)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;

    /*
     * Save addr & len in env so that we can access it in
     * kvx_debug_check_watchpoint below.
     */
    env->watchpoint_hit_access_addr = addr;
    env->watchpoint_hit_access_len = len;
    return addr;
}

bool kvx_debug_check_watchpoint(CPUState *cs, CPUWatchpoint *wp)
{
    /*
     * Called by core code when a BP_CPU watchpoint fires.
     * Core has already checked that wp address matches.
     * We have to tell whether the watchpoint really hits.
     *
     * watchpoint_hit_access_addr&len are the access info
     * stored by kvx_adjust_watchpoint_address above.
     */
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;
    vaddr addr = env->watchpoint_hit_access_addr;
    vaddr len  = env->watchpoint_hit_access_len;
    int wp_n = wp == env->watchpoint[0] ? 0 : 1;

    /* We check more things than core does (PL in particular) */
    if (!kvx_wp_matches(env, wp_n, addr, len)) {
        return false;
    }

    if (kvx_wp_matches(env, 1-wp_n, addr, len)) {
        int wp_pl = kvx_get_wp_pl(env, wp_n);
        int wp_pl_other = kvx_get_wp_pl(env, 1-wp_n);
        /* Least privileged watchpoint hits first */
        if (wp_pl < wp_pl_other) {
            return false;
        } else if (wp_pl == wp_pl_other && wp_n == 1) {
            /* WP0 has priority over WP1 */
            return false;
        }
    }

    return true;
}
