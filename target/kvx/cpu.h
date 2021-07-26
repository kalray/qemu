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

#ifndef KVX_CPU_H
#define KVX_CPU_H

#include "qemu-common.h"
#include "hw/core/cpu.h"
#include "exec/cpu-defs.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "sysemu/cpus.h"

#include "gen/cpu-models.h"

#define TYPE_KVX_CPU "kvx-cpu"

#define KVX_CPU_TYPE_SUFFIX "-" TYPE_KVX_CPU
#define KVX_CPU_TYPE_NAME(name) (name KVX_CPU_TYPE_SUFFIX)
#define CPU_RESOLVING_TYPE TYPE_KVX_CPU

#define TYPE_KVX_CPU_KV3_V1 KVX_CPU_TYPE_NAME("kv3-1")

typedef struct CPUKVXState CPUKVXState;

#include "cpu_bits.h"

typedef struct KVXTLBEntry {
    uint64_t low;
    uint64_t high;
} KVXTLBEntry;

typedef enum KVXSleepState {
    KVX_RUNNING = 0,
    KVX_RESETTING,
    KVX_WAITIT,
    KVX_SYNCGROUP,
    KVX_AWAIT,
    KVX_SLEEP,
    KVX_STOP
} KVXSleepState;

struct CPUKVXState {
#include "gen/storages.inc.h"

    /* TLBs */
    KVXTLBEntry ltlb[LTLB_WAYS];
    KVXTLBEntry jtlb[JTLB_SETS][JTLB_WAYS];

    /* Used by helpers that need to return more than 64 bits */
    uint64_t scratch[4];

    /* LE was written since last call to update_hardware_loop */
    bool le_is_dirty;

    /*
     * Filled when raising an exception. Used by kvx_cpu_do_interrupt to build
     * the exception syndrome.
     */
    uint64_t excp_syndrome;
    uint64_t excp_address;
    int excp_target_pl;

    /*
     * CPU HW breakpoints
     * Note: We cannot uses standard breakpoint because they do not
     * support ranges.
     */
    struct CPUKVXBreakpoint {
        /* true if bp is enabled and pl is enough */
        bool enabled;
        /* first and last address */
        uint64_t addr_low;
        uint64_t addr_high;
    } breakpoint[2];
    /* CPU HW watchpoints */
    struct CPUWatchpoint *watchpoint[2];
    /* Used during watchpoint processing */
    vaddr watchpoint_hit_access_addr;
    vaddr watchpoint_hit_access_len;

    /* Inter process event lines */
    qemu_irq ipe_fwd_out[KVX_NUM_IPE];
    qemu_irq ipe_bwd_out[KVX_NUM_IPE];

    KVXSleepState sleep_state;

    struct {
        uint32_t or_mask;
        uint32_t and_mask;
        uint32_t dst_op;
    } waitit_ctx;

    struct {
        uint32_t wait_mask;
        uint16_t notify_fwd;
        uint16_t notify_bwd;
    } syncgroup_ctx;
};

typedef CPUKVXState CPUArchState;

typedef enum RegisterAccessType {
    REG_ACCESS_NONE = 0,
    REG_ACCESS_GET,
    REG_ACCESS_SET,
    REG_ACCESS_WFX,
} RegisterAccessType;

typedef enum RegisterReadErrorType {
    REG_READ_ERROR_READ = 0, // 0 value means we do a normal read
    REG_READ_ERROR_READ0,
    REG_READ_ERROR_TRAP_PRIVILEGE,
} RegisterReadErrorType;

typedef enum RegisterWriteErrorType {
    REG_WRITE_ERROR_WRITE = 0, // 0 value means we do a normal write
    REG_WRITE_ERROR_TRAP_PRIVILEGE,
} RegisterWriteErrorType;

#include "gen/registers.inc.h"

typedef struct KVXCPU KVXCPU;

#include "reg_info.h"

static inline int kvx_get_current_pl(CPUKVXState *env);

/*
 * Tell if get/set/wfx operation are supported on the given register.
 */
static inline bool kvx_register_support_access(Register reg, RegisterAccessType access)
{
    const RegisterDescr *rdescr = &REGISTERS[reg];
    switch (access) {
    case REG_ACCESS_GET:
        return rdescr->raccess == REG_ACCESS_GET;
    case REG_ACCESS_SET:
        return rdescr->waccess == REG_ACCESS_SET ||
               rdescr->waccess == REG_ACCESS_WFX;
    case REG_ACCESS_WFX:
        return rdescr->waccess == REG_ACCESS_WFX;
    default:
        g_assert_not_reached();
    }
}

static inline uint64_t *kvx_register_ptr_u64(CPUArchState *env, Register reg)
{
    uint8_t *ptr;
    const RegisterDescr *rdescr = &REGISTERS[reg];

    g_assert(rdescr->reg_width <= 64);

    ptr = ((uint8_t*) &env->storages) + rdescr->offset;

    return (uint64_t *) ptr;
}

static inline uint64_t kvx_register_read_u64(CPUArchState *env, Register reg)
{
    return *kvx_register_ptr_u64(env, reg);
}

static inline uint64_t kvx_register_read_aliased_u64(CPUArchState *env,
                                                     Register reg)
{
    int pl = kvx_get_current_pl(env);
    Register aliased = kvx_register_alias_pl(reg, pl);
    return kvx_register_read_u64(env, aliased);
}

static inline void kvx_register_write_u64(CPUArchState *env, Register reg,
                                         uint64_t val)
{
    *kvx_register_ptr_u64(env, reg) = val;
}

static inline uint64_t kvx_regfield_read(CPUArchState *env,
                                         RegisterField field)
{
    const RegisterFieldDescr *rfdescr = &REGISTERFIELDS[field];
    uint64_t v = kvx_register_read_u64(env, rfdescr->reg);
    return extract64(v, rfdescr->offset, rfdescr->width);
}

#define KVX_FIELD_MASK(reg_, field_) \
    (REGISTERFIELDS[REGFIELD_ ## reg_ ## _ ## field_].mask)
#define KVX_FIELD_LENGTH(reg_, field_) \
    (REGISTERFIELDS[REGFIELD_ ## reg_ ## _ ## field_].width)
#define KVX_FIELD_SHIFT(reg_, field_) \
    (REGISTERFIELDS[REGFIELD_ ## reg_ ## _ ## field_].offset)

#define KVX_FIELD_EX64(val_, reg_, field_)    \
    extract64((val_),                         \
              KVX_FIELD_SHIFT(reg_, field_),  \
              KVX_FIELD_LENGTH(reg_, field_))

#define KVX_FIELD_DP64(sto_, reg_, field_, val_) \
    deposit64((sto_),                            \
              KVX_FIELD_SHIFT(reg_, field_),     \
              KVX_FIELD_LENGTH(reg_, field_),    \
              (val_))

#define kvx_register_read_field(env_, reg_, field_) KVX_FIELD_EX64( \
            kvx_register_read_u64(env_, REG_kv3_##reg_), \
            kv3_##reg_, field_)

#define kvx_register_write_field(env_, reg_, field_, val_) \
            (*kvx_register_ptr_u64(env_, REG_kv3_##reg_) = KVX_FIELD_DP64( \
            kvx_register_read_u64(env_, REG_kv3_##reg_), \
            kv3_##reg_, field_, val_))

static inline int kvx_get_current_pl(CPUKVXState *env)
{
    return kvx_register_read_field(env, PS, PL);
}

/**
 * KVXTimer:
 * used in KVXCPU.
 */
typedef void (*KVXTimerCallback)(KVXCPU* cpu);
typedef struct KVXTimer KVXTimer;
struct KVXTimer {
    KVXCPU* cpu;
    QEMUTimer *qemu_timer;
    qemu_irq irq;
    /* timer's period in ns (constant) */
    uint32_t period_ns;
    /* timestamp of the last timer update */
    int64_t timestamp;
    /* flag telling if timer is counting or not */
    bool enabled;
    /* flag telling if irq/watchdog is enabled */
    bool ev_enabled;
    /* pointers to the timer registers in the cpu's env */
    uint64_t *reg_value;
    uint64_t *reg_reload;
    /* bitmasks in the TCR register for the timer */
    uint64_t tcr_enable_bit;
    uint64_t tcr_status_bit;
    uint64_t tcr_interrupt_bit;
    uint64_t tcr_idle_bit;
    uint64_t tcr_watchdog_bit;
};

/*
 * Common object shared between the PEs. It tracks when an IPE event forwarding
 * begins to avoid entering event infinite loop between the cores.
 *
 * TODO: make this object thread safe when adding MTTCG support
 */
#define TYPE_KVX_IPE_HELPER "kvx-ipe-helper"
#define KVX_IPE_HELPER(obj) \
    OBJECT_CHECK(KvxIpeHelper, (obj), \
                 TYPE_KVX_IPE_HELPER)

typedef struct KvxIpeHelper {
    /*< private >*/
    Object parent;

    /*< public >*/
    bool transaction_started;
    bool loop;
    KVXCPU *initiator;
} KvxIpeHelper;

#define KVX_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(KVXCPUClass, (klass), TYPE_KVX_CPU)
#define KVX_CPU(obj) \
    OBJECT_CHECK(KVXCPU, (obj), TYPE_KVX_CPU)
#define KVX_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(KVXCPUClass, (obj), TYPE_KVX_CPU)

/**
 * KVXCPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 *
 * A KVX CPU model.
 */
typedef struct KVXCPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/
    DeviceRealize parent_realize;
    DeviceReset parent_reset;
} KVXCPUClass;

/**
 * KVXCPU:
 * @env: #CPUKVXState
 *
 * A KVX CPU.
 */
struct KVXCPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    ProcessorModel model;

    CPUNegativeOffsetState neg;
    CPUKVXState env;

    struct {
        uint8_t pid;
    } cfg;

    /* watchdog & timers */
    struct KVXTimer timer[2];
    struct KVXTimer watchdog;

    /* GPIO/IRQ output of the watchdog timer */
    qemu_irq watchdog_interrupt;

    /* performance monitor counters base */
    int64_t pm_base[4];

    KvxIpeHelper *ipe_helper;
};

typedef KVXCPU ArchCPU;

/**
 * @return true if cpu is the RM core of the cluster
 */
static inline bool kvx_cpu_is_rm(KVXCPU *cpu)
{
    return cpu->cfg.pid == 16;
}

/**
 * kvx_register_read/set:
 * getter and setter handling side effects.
 */
uint64_t kvx_register_read(KVXCPU *cpu, Register reg);
void kvx_register_write(KVXCPU *cpu, Register reg, uint64_t val);

/**
 * kvx_timer_init/term:
 * Initialize/Terminate the timer structure.
 * Init takes some arguments:
 * + value/reload: where are stored the value and reload register
 * + tcr_enable_bit : mask in TCR reg to check if enabled
 * + tcr_status_bit : mask in TCR reg to set on underflow
 * + tcr_interrupt_bit : mask in TCR reg to check if interrupt is enabled
 * + tcr_idle_bit: mask in TCR reg to check if counting is stopped when cpu
 *                 is idle
 * + tcr_watchdog_bit: mask in TCR reg to check if watchdog is enabled
 */
void kvx_timer_init(KVXTimer *timer, KVXCPU *cpu, qemu_irq irq,
                    uint64_t *reg_value, uint64_t *reg_reload,
                    uint64_t tcr_enable_bit,
                    uint64_t tcr_status_bit,
                    uint64_t tcr_interrupt_bit,
                    uint64_t tcr_idle_bit,
                    uint64_t tcr_watchdog_bit);

/**
 * kvx_timer_reset:
 * Reset the timer.
 * Note: should be paired with clearing the timer registers.
 */
void kvx_timer_reset(KVXTimer *timer);


/**
 * kvx_timer_write/read_value:
 * To read or write the value register.
 */
void kvx_timer_write_value(KVXTimer *timer, uint64_t value);

/**
 * kvx_timer_update:
 * Update the timer state.
 * This should be called:
 * + before reading any timer/watchdog related registers.
 * + after writing TCR or changing cpu idle state
 */
void kvx_timer_update(KVXTimer *timer);

/**
 * kvx_cpu_update_timers:
 * Update all timers. should be done when modifying TCR and sleep states.
 */
void kvx_cpu_update_timers(KVXCPU *cpu);

int kvx_cpu_mmu_index(CPUKVXState *env, bool ifetch);
#define cpu_mmu_index kvx_cpu_mmu_index

void kvx_cpu_list(void);
#define cpu_list kvx_cpu_list

void QEMU_NORETURN kvx_raise_exception(CPUKVXState *env,
                                      uint32_t exception,
                                      uintptr_t pc);
void kvx_cpu_do_interrupt(CPUState *cs);
bool kvx_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
int kvx_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n);
int kvx_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n);
void kvx_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                MMUAccessType access_type, int mmu_idx,
                                uintptr_t retaddr);
bool kvx_cpu_tlb_fill(CPUState *cs, target_ulong vaddr, int size,
                     MMUAccessType access_type, int mmu_idx,
                     bool probe, uintptr_t retaddr);
hwaddr kvx_cpu_get_phys_page_debug(CPUState *cs, vaddr addr);
void kvx_translate_init(void);

void kvx_cpu_trigger_watchdog(KVXCPU *cpu);

static inline bool kvx_hardware_loop_enabled(CPUKVXState *env)
{
    return kvx_register_read_field(env, PS, HLE);
}

static inline bool kvx_in_hardware_loop_ctx(CPUKVXState *env)
{
    uint64_t reg_lc, reg_le;

    reg_lc = kvx_register_read_u64(env, REG_kv3_LC);
    reg_le = kvx_register_read_u64(env, REG_kv3_LE);

    return (kvx_hardware_loop_enabled(env))
        && (reg_lc != 1)
        && (reg_le != 0);
}

static inline bool kvx_arith_irq_enabled(CPUKVXState *env)
{
    uint64_t csit = kvx_register_read_u64(env, REG_kv3_CSIT);

    return (csit & KVX_CSIT_IRQ_MASK)
        && !KVX_FIELD_EX64(csit, kv3_CSIT, AEIR);
}

static inline int kvx_get_step_pl(CPUKVXState *env)
{
    return kvx_register_read_field(env, PSO, SME);
}

static inline bool kvx_step_mode_enabled(CPUKVXState *env)
{
    int pl = kvx_get_current_pl(env);
    int step_pl = kvx_get_step_pl(env);
    int sme = kvx_register_read_field(env, PS, SME);

    return sme && (pl > step_pl);
}

/* TB state fields */
/* MMU index field and sub-fields */
FIELD(TB_STATE, MMU_IDX, 0, 4)
FIELD(TB_STATE, MMU_IDX_MMUP, 0, 1)
FIELD(TB_STATE, MMU_IDX_VS, 1, 2)
FIELD(TB_STATE, MMU_IDX_V64, 3, 1)

/* PS.HLE == 1 */
FIELD(TB_STATE, HLE, 4, 1)

/* So we don't forget to update MMU_IDX when adding sub-fields */
QEMU_BUILD_BUG_ON((int)R_TB_STATE_MMU_IDX_LENGTH != (int)R_TB_STATE_HLE_SHIFT);

/* In hardware loop context */
FIELD(TB_STATE, HL_CTX, 5, 1)

/* PS.DAUS == 1 */
FIELD(TB_STATE, DAUS, 6, 1)

/* Wakeup status (WS.WUx) */
FIELD(TB_STATE, WU, 7, 3)

/* Arithmetic IRQs may be raised */
FIELD(TB_STATE, ARITH_IRQ, 10, 1)

/* Step Mode Enabled */
FIELD(TB_STATE, SME, 11, 1)

/* Data Cache Enabled */
FIELD(TB_STATE, DCE, 12, 1)

static inline void cpu_get_tb_cpu_state(CPUKVXState *env, target_ulong *pc,
                                        target_ulong *cs_base, uint32_t *flags)
{

    *pc = kvx_register_read_u64(env, REG_kv3_PC);
    *cs_base = 0;

    *flags = cpu_mmu_index(env, false);
    *flags = FIELD_DP64(*flags, TB_STATE, HLE, kvx_hardware_loop_enabled(env));
    *flags = FIELD_DP64(*flags, TB_STATE, HL_CTX, kvx_in_hardware_loop_ctx(env));

    *flags = FIELD_DP64(*flags, TB_STATE, DAUS,
                        kvx_register_read_field(env, PS, DAUS));

    *flags = FIELD_DP64(*flags, TB_STATE, WU,
                        kvx_register_read_u64(env, REG_kv3_WS) & 0x7);

    *flags = FIELD_DP64(*flags, TB_STATE, ARITH_IRQ,
                        kvx_arith_irq_enabled(env));

    *flags = FIELD_DP64(*flags, TB_STATE, SME,
                        kvx_step_mode_enabled(env));

    *flags = FIELD_DP64(*flags, TB_STATE, DCE,
                        kvx_register_read_field(env, PS, DCE));
}

#include "exec/cpu-all.h"

#endif

