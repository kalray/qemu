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
#include "qemu/qemu-print.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qemu/main-loop.h"
#include "trace.h"
#include "internal.h"

static const VMStateDescription vmstate_kvx_cpu = {
    .name = "cpu",
    .unmigratable = 1,
};

static void kvx_cpu_irq_request(void *opaque, int irq, int level)
{
    KVXCPU *cpu = opaque;
    CPUKVXState *env = &cpu->env;
    CPUState *cs = CPU(cpu);
    bool locked = false;
    uint64_t *ilr, old;

    g_assert(irq >= 0 && irq < 32);

    if (!level) {
        return;
    }

    /* Make sure locking works even if BQL is already held by the caller */
    if (!qemu_mutex_iothread_locked()) {
        locked = true;
        qemu_mutex_lock_iothread();
    }

    ilr = kvx_register_ptr_u64(env, REG_kv3_ILR);
    old = *ilr;
    /*
     * According to MPPA cluster doc 2.7.2 each core has:
     * + 32 edge-triggered interrupt inputs
     */
    *ilr |= ((uint64_t) 0x1) << irq;

    /* WS.WU0 and WS.WU1 are set unconditionally */
    kvx_register_write_field(env, WS, WU0, true);
    kvx_register_write_field(env, WS, WU1, true);

    trace_kvx_irq_request(cpu->cfg.pid, irq, old, *ilr);
    cpu_interrupt(cs, CPU_INTERRUPT_HARD);

    if (locked) {
        qemu_mutex_unlock_iothread();
    }
}

static void kvx_cpu_ipe_fwd_event(void *opaque, int idx, int lvl)
{
    KVXCPU *cpu = opaque;
    CPUKVXState *env = &cpu->env;

    if (!lvl) {
        return;
    }

    kvx_handle_ipe_event(env, true, idx);
}

static void kvx_cpu_ipe_bwd_event(void *opaque, int idx, int lvl)
{
    KVXCPU *cpu = opaque;
    CPUKVXState *env = &cpu->env;

    if (!lvl) {
        return;
    }

    kvx_handle_ipe_event(env, false, idx);
}

static void kvx_cpu_irqs_init(KVXCPU *cpu)
{
    CPUKVXState *env = &cpu->env;
    qdev_init_gpio_in(DEVICE(cpu), kvx_cpu_irq_request, KVX_NUM_IRQ);

    qdev_init_gpio_in_named(DEVICE(cpu), kvx_cpu_ipe_fwd_event,
                            "ipe-fwd-in", KVX_NUM_IPE);
    qdev_init_gpio_in_named(DEVICE(cpu), kvx_cpu_ipe_bwd_event,
                            "ipe-bwd-in", KVX_NUM_IPE);
    qdev_init_gpio_out_named(DEVICE(cpu), env->ipe_fwd_out,
                             "ipe-fwd-out", KVX_NUM_IPE);
    qdev_init_gpio_out_named(DEVICE(cpu), env->ipe_bwd_out,
                             "ipe-bwd-out", KVX_NUM_IPE);
}

void kvx_cpu_update_timers(KVXCPU *cpu)
{
    kvx_timer_update(&cpu->watchdog);
    kvx_timer_update(&cpu->timer[0]);
    kvx_timer_update(&cpu->timer[1]);
}

void kvx_cpu_trigger_watchdog(KVXCPU *cpu)
{
    CPUState *cs = CPU(cpu);
    CPUKVXState *env = &cpu->env;

    qemu_irq_pulse(cpu->watchdog_interrupt);

    qemu_log_mask(LOG_GUEST_ERROR,
                  "core %d: watchdog trigger, "
                  "stopping execution\n", cpu->cfg.pid);
    cpu_reset(cs);
    cs->halted = true;
    cs->exception_index = EXCP_HLT;
    env->sleep_state = KVX_RESETTING;
}

static void kvx_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    KVXCPU *cpu = KVX_CPU(dev);
    Error *local_err = NULL;
    KVXCPUClass *mcc = KVX_CPU_GET_CLASS(dev);
    CPUKVXState *env = &cpu->env;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    kvx_cpu_irqs_init(cpu);
    kvx_timer_init(&cpu->timer[0], cpu,
                   qdev_get_gpio_in(dev, KVX_IRQ_TIMER0),
                   kvx_register_ptr_u64(env, REG_kv3_T0V),
                   kvx_register_ptr_u64(env, REG_kv3_T0R),
                   KVX_FIELD_DP64(0, kv3_TCR, T0CE, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, T0ST, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, T0IE, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, T0SI, 1),
                   0x0 /* not a  watchdog */);
    kvx_timer_init(&cpu->timer[1], cpu,
                   qdev_get_gpio_in(dev, KVX_IRQ_TIMER1),
                   kvx_register_ptr_u64(env, REG_kv3_T1V),
                   kvx_register_ptr_u64(env, REG_kv3_T1R),
                   KVX_FIELD_DP64(0, kv3_TCR, T1CE, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, T1ST, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, T1IE, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, T1SI, 1),
                   0x0 /* not a  watchdog */);
    kvx_timer_init(&cpu->watchdog, cpu,
                   qdev_get_gpio_in(dev, KVX_IRQ_WATCHDOG),
                   kvx_register_ptr_u64(env, REG_kv3_WDV),
                   kvx_register_ptr_u64(env, REG_kv3_WDR),
                   KVX_FIELD_DP64(0, kv3_TCR, WCE, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, WUS, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, WIE, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, WSI, 1),
                   KVX_FIELD_DP64(0, kv3_TCR, WUI, 1));
    cpu_reset(cs);

    mcc->parent_realize(dev, errp);
}

static void kvx_cpu_reset(DeviceState *dev)
{
    CPUState *cs = CPU(dev);
    KVXCPU *cpu = KVX_CPU(cs);
    KVXCPUClass *mcc = KVX_CPU_GET_CLASS(cpu);
    CPUKVXState *env = &cpu->env;
    size_t i;

    mcc->parent_reset(dev);
    kvx_timer_reset(&cpu->watchdog);
    kvx_timer_reset(&cpu->timer[0]);
    kvx_timer_reset(&cpu->timer[1]);

    memset(&env->storages, 0, sizeof(env->storages));

    for (i = 0; i < ARRAY_SIZE(REGISTERS); i++) {
        uint64_t reset = REGISTERS[i].reset;

        if (reset) {
            kvx_register_write_u64(env, i, reset);
        }
    }

    kvx_register_write_field(env, PCR, PID, cpu->cfg.pid);

    memset(env->prev_le, 0, sizeof(env->prev_le));
}

static void kvx_cpu_init(Object *obj)
{
    KVXCPU *cpu = KVX_CPU(obj);

    cpu_set_cpustate_pointers(cpu);

    qdev_init_gpio_out_named(DEVICE(cpu), &cpu->watchdog_interrupt,
                             "watchdog-interrupt", 1);
}

static void kv3_v1_cpu_init(Object *obj)
{
}

/*
 * @return true if the core is in one of the three sleep states (await, sleep,
 * stop), and the corresponding bit in WS is set.
 * Note that the WAITIT case is handled in kvx_cpu_exec_interrupt.
 */
static bool kvx_must_wakeup(CPUKVXState *env)
{
    uint8_t ws;
    int idle_lvl;

    if (env->sleep_state < KVX_AWAIT) {
        return false;
    }

    ws = kvx_register_read_u64(env, REG_kv3_WS);
    idle_lvl = env->sleep_state - KVX_AWAIT;

    return ws & (1 << idle_lvl);
}

static bool kvx_cpu_has_work(CPUState *cs)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;

    return (env->sleep_state == KVX_RUNNING)
        || kvx_must_wakeup(env)
        || ((env->sleep_state == KVX_WAITIT)
            && (kvx_irq_get_pending(env) != -1));
}

static void kvx_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;

#include "gen/cpu-dump-state.inc.c"
}

static void kvx_cpu_set_pc(CPUState *cs, vaddr value)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;

    kvx_register_write_u64(env, REG_kv3_PC, value);
}

static void kvx_cpu_synchronize_from_tb(CPUState *cs, const TranslationBlock *tb)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;

    kvx_register_write_u64(env, REG_kv3_PC, tb->pc);
}

static void kvx_cpu_disas_set_info(CPUState *s, disassemble_info *info)
{
    info->print_insn = print_insn_kvx;
    info->arch = bfd_arch_kvx;
    info->mach = bfd_mach_kv3_1;
}

static ObjectClass *kvx_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;
    char **cpuname;

    cpuname = g_strsplit(cpu_model, ",", 1);
    typename = g_strdup_printf(KVX_CPU_TYPE_NAME("%s"), cpuname[0]);
    oc = object_class_by_name(typename);
    g_strfreev(cpuname);
    g_free(typename);
    if (!oc || !object_class_dynamic_cast(oc, TYPE_KVX_CPU) ||
        object_class_is_abstract(oc)) {
        return NULL;
    }
    return oc;
}

static void kvx_cpu_list_entry(gpointer data, gpointer user_data)
{
    ObjectClass *oc = data;
    const char *typename = object_class_get_name(oc);
    char *name;

    name = g_strndup(typename, strlen(typename) - strlen(KVX_CPU_TYPE_SUFFIX));
    qemu_printf("  %s\n", name);
    g_free(name);
}

void kvx_cpu_list(void)
{
    GSList *list;

    list = object_class_get_list_sorted(TYPE_KVX_CPU, false);
    qemu_printf("Available CPUs:\n");
    g_slist_foreach(list, kvx_cpu_list_entry, NULL);
    g_slist_free(list);
}

static gchar *kv3_gdb_arch_name(CPUState *cs)
{
    return g_strdup("kvx:kv3-1:64");
}

static Property kvx_cpu_properties[] = {
    DEFINE_PROP_UINT8("pid", KVXCPU, cfg.pid, 0),
    DEFINE_PROP_LINK("ipe-helper", KVXCPU, ipe_helper,
                     TYPE_KVX_IPE_HELPER, KvxIpeHelper *),
    DEFINE_PROP_END_OF_LIST(),
};

#ifndef CONFIG_USER_ONLY
#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps kvx_sysemu_ops = {
    .get_phys_page_debug = kvx_cpu_get_phys_page_debug,
};
#endif

#include "hw/core/tcg-cpu-ops.h"

#ifdef CONFIG_TCG
static struct TCGCPUOps kvx_tcg_ops = {
    .initialize = kvx_translate_init,
    .synchronize_from_tb = kvx_cpu_synchronize_from_tb,
    .cpu_exec_interrupt = kvx_cpu_exec_interrupt,
    .tlb_fill = kvx_cpu_tlb_fill,
    .debug_excp_handler = kvx_debug_excp_handler,

#ifndef CONFIG_USER_ONLY
    .do_interrupt = kvx_cpu_do_interrupt,
    .do_unaligned_access = kvx_cpu_do_unaligned_access,
    .adjust_watchpoint_address = kvx_adjust_watchpoint_address,
    .debug_check_watchpoint = kvx_debug_check_watchpoint,
#endif
};
#endif

static void kvx_cpu_class_init(ObjectClass *c, void *data)
{
    KVXCPUClass *mcc = KVX_CPU_CLASS(c);
    CPUClass *cc = CPU_CLASS(c);
    DeviceClass *dc = DEVICE_CLASS(c);

    device_class_set_parent_realize(dc, kvx_cpu_realize,
                                    &mcc->parent_realize);

    device_class_set_parent_reset(dc, kvx_cpu_reset, &mcc->parent_reset);

    cc->class_by_name = kvx_cpu_class_by_name;
    cc->has_work = kvx_cpu_has_work;
    cc->dump_state = kvx_cpu_dump_state;
    cc->set_pc = kvx_cpu_set_pc;
    cc->gdb_read_register = kvx_cpu_gdb_read_register;
    cc->gdb_write_register = kvx_cpu_gdb_write_register;
    cc->gdb_num_core_regs = 400;
    cc->gdb_core_xml_file = "kv3-core.xml";
    cc->gdb_arch_name = kv3_gdb_arch_name;
    cc->disas_set_info = kvx_cpu_disas_set_info;

#ifdef CONFIG_TCG
    cc->tcg_ops = &kvx_tcg_ops;
#endif

#ifndef CONFIG_USER_ONLY
    cc->sysemu_ops = &kvx_sysemu_ops;
    dc->vmsd = &vmstate_kvx_cpu;
#endif

    device_class_set_props(dc, kvx_cpu_properties);
}

static void kvx_ipe_helper_init(Object *obj)
{
    KvxIpeHelper *s = KVX_IPE_HELPER(obj);

    s->transaction_started = false;
}

#define DEFINE_CPU(type_name, init_fn) \
    {                                  \
        .name = type_name,             \
        .parent = TYPE_KVX_CPU,        \
        .instance_init = init_fn       \
    }

static const TypeInfo kvx_cpu_type_infos[] = {
    {
        .name = TYPE_KVX_IPE_HELPER,
        .parent = TYPE_OBJECT,
        .instance_init = kvx_ipe_helper_init,
    },
    {
        .name = TYPE_KVX_CPU,
        .parent = TYPE_CPU,
        .instance_size = sizeof(KVXCPU),
        .instance_init = kvx_cpu_init,
        .abstract = true,
        .class_size = sizeof(KVXCPUClass),
        .class_init = kvx_cpu_class_init,
    },
    DEFINE_CPU(TYPE_KVX_CPU_KV3_V1, kv3_v1_cpu_init),
};

DEFINE_TYPES(kvx_cpu_type_infos)
