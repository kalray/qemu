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

#ifndef KVX_INTERNAL_H
#define KVX_INTERNAL_H

#include "sysemu/cpu-timers.h"

/* we use 20us period (50KHz frequency) in case we are not in icount mode. */
#define KVX_CPU_DEFAULT_CLOCK_PERIOD_NS 20000

typedef enum {
    TLB_LOOKUP_SUCCESS = 0,
    TLB_LOOKUP_FAIL
} TLBLookupStatus;

typedef struct TLBLookupInfo {
    target_ulong paddr;
    int page_size;
    int prot;
    int cache_policy;
    MemTxAttrs attrs;
    int fault;
} TLBLookupInfo;

TLBLookupStatus get_physical_addr(CPUKVXState *env, target_ulong vaddr,
                                  MMUAccessType access_type, int mmu_idx,
                                  TLBLookupInfo *info);

void fill_tlb_excp_syndrome(CPUKVXState *env, target_ulong fault_addr,
                            MMUAccessType access_type, int fault);

void kvx_do_semihosting(CPUKVXState *env, uint64_t scall);
int kvx_irq_get_pending(CPUKVXState *env);
void kvx_handle_ipe_event(CPUKVXState *env, bool forward, unsigned int idx);
void kvx_send_ipe_events(CPUKVXState *env, uint16_t fwd_mask, uint16_t bwd_mask);
void kvx_update_cpu_state(CPUKVXState *env, uint64_t ps_mask, uint64_t sps_mask);
void kvx_update_breakpoints(CPUKVXState *env);
void kvx_debug_excp_handler(CPUState *cs);
vaddr kvx_adjust_watchpoint_address(CPUState *cs, vaddr addr, int len);
bool kvx_debug_check_watchpoint(CPUState *cs, CPUWatchpoint *wp);

/*
 * When PS.DAUS is set, most of the MMU related configuration bits are
 * taken from SPS instead of PS for data side accesses.
 */
static inline uint64_t get_mmu_ps(CPUKVXState *env, bool ifetch)
{
    uint64_t ps = kvx_register_read_u64(env, REG_kv3_PS);

    if ((!ifetch) && KVX_FIELD_EX64(ps, kv3_PS, DAUS)) {
        return kvx_register_read_aliased_u64(env, REG_kv3_SPS);
    }

    return ps;
}

static inline unsigned kvx_cpu_clock_period_ns(void)
{
    return icount_enabled() ? icount_to_ns(1) : KVX_CPU_DEFAULT_CLOCK_PERIOD_NS;
}

#define PACK_FIELD(packed_, total_, syndrome_, field_)               \
    do {                                                             \
        (packed_) |= (((syndrome_) & KVX_FIELD_MASK(kv3_ES, field_)) \
                      >> KVX_FIELD_SHIFT(kv3_ES, field_)             \
                      << (total_));                                  \
        (total_) += KVX_FIELD_LENGTH(kv3_ES, field_);                \
    } while (0)


/*
 * Pack a syndrome info into a target_ulong to fit into TB instruction extra
 * info. Pack only the necessary bits so that the sleb128 compression performs
 * better.
 */
static inline target_ulong insn_syndrome_pack(uint64_t syndrome)
{
    int total = 0;
    target_ulong ret = 0;

    /* This function should be used for memory access instructions only */
    g_assert(KVX_FIELD_EX64(syndrome, kv3_ES, AS));

    PACK_FIELD(ret, total, syndrome, BS);
    PACK_FIELD(ret, total, syndrome, AS);
    PACK_FIELD(ret, total, syndrome, UCA);
    PACK_FIELD(ret, total, syndrome, NTA);
    PACK_FIELD(ret, total, syndrome, RWX);
    PACK_FIELD(ret, total, syndrome, DRI);

    return ret;
}

#undef PACK_FIELD

#define UNPACK_FIELD(packed_, field_)                               \
    ({                                                              \
        uint64_t v = ((packed_)                                     \
            & MAKE_64BIT_MASK(0, KVX_FIELD_LENGTH(kv3_ES, field_))) \
            << KVX_FIELD_SHIFT(kv3_ES, field_);                     \
        (packed_) >>= KVX_FIELD_LENGTH(kv3_ES, field_);             \
        v;                                                          \
     })

/*
 * Unpack a syndrome info from a TB instruction extra info into the expected
 * ES_PLx register layout.
 */
static inline uint64_t insn_syndrome_unpack(target_ulong packed_syndrome)
{
    uint64_t syndrome = 0;

    /*
     * A null syndrome means the corresponding instruction is not a memory
     * access. So it should not raise an MMU fault on the data side.
     */
    g_assert(packed_syndrome != 0);

    syndrome |= UNPACK_FIELD(packed_syndrome, BS);
    syndrome |= UNPACK_FIELD(packed_syndrome, AS);
    syndrome |= UNPACK_FIELD(packed_syndrome, UCA);
    syndrome |= UNPACK_FIELD(packed_syndrome, NTA);
    syndrome |= UNPACK_FIELD(packed_syndrome, RWX);
    syndrome |= UNPACK_FIELD(packed_syndrome, DRI);

    return syndrome;
}

#undef UNPACK_FIELD

static inline int kvx_get_current_iapl(CPUKVXState *env)
{
    int lvl;
    lvl = (3 - kvx_register_read_field(env, PS, PL)) * 4;
    lvl += kvx_register_read_field(env, PS, IL);
    return lvl;
}

static inline int kvx_get_irq_apl(CPUKVXState *env, int irq)
{
    int ito, ill;
    irq *= 2;
    ito = (kvx_register_read_u64(env, REG_kv3_ITO) >> irq) & 0x3;
    ill = (kvx_register_read_u64(env, REG_kv3_ILL) >> irq) & 0x3;
    return ((3 - ito) * 4) + ill;
}

static inline int kvx_get_irq_pl(CPUKVXState *env, int irq)
{
    return (kvx_register_read_u64(env, REG_kv3_ITO) >> (irq*2)) & 0x3;
}

static inline int kvx_get_irq_ll(CPUKVXState *env, int irq)
{
    return (kvx_register_read_u64(env, REG_kv3_ILL) >> (irq*2)) & 0x3;
}

static inline int kvx_get_bp_pl(CPUKVXState *env, int n)
{
    return n ? kvx_register_read_field(env, DO, B1)
             : kvx_register_read_field(env, DO, B0);
}

static inline int kvx_get_wp_pl(CPUKVXState *env, int n)
{
    return n ? kvx_register_read_field(env, DO, W1)
             : kvx_register_read_field(env, DO, W0);
}

#endif
