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
#include "qemu/qemu-print.h"
#include "cpu.h"
#include "internal.h"

static void kvx_timer_callback(void *opaque)
{
    struct KVXTimer *timer = opaque;
    kvx_timer_update(timer);
}

void kvx_timer_init(KVXTimer *timer, KVXCPU *cpu, qemu_irq irq,
                    uint64_t *reg_value, uint64_t *reg_reload,
                    uint64_t tcr_enable_bit,
                    uint64_t tcr_status_bit,
                    uint64_t tcr_interrupt_bit,
                    uint64_t tcr_idle_bit,
                    uint64_t tcr_watchdog_bit)
{
    timer->cpu = cpu;
    timer->irq = irq;
    timer->qemu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, kvx_timer_callback, timer);
    timer->period_ns = kvx_cpu_clock_period_ns();
    timer->enabled = false;
    timer->ev_enabled = false;
    timer->reg_value = reg_value;
    timer->reg_reload = reg_reload;
    timer->tcr_enable_bit = tcr_enable_bit;
    timer->tcr_status_bit = tcr_status_bit;
    timer->tcr_interrupt_bit = tcr_interrupt_bit;
    timer->tcr_idle_bit = tcr_idle_bit;
    timer->tcr_watchdog_bit = tcr_watchdog_bit;
}

static bool kvx_timer_update_value(KVXTimer *timer)
{
    int64_t now;
    uint64_t elapsed;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = ((uint64_t)(now - timer->timestamp)) / timer->period_ns;
    timer->timestamp += elapsed * timer->period_ns;

    if (elapsed <= *timer->reg_value) {
        /* timer has not yet underflowed */
        *timer->reg_value -= elapsed;
        return false;
    } else {
        /* timer has underflowed */
        uint64_t period = *timer->reg_reload + 1;
        /* reload occurs 1 tick after timer reached 0 */
        elapsed -= *timer->reg_value + 1u;
        if (period) {
            *timer->reg_value = *timer->reg_reload - (elapsed % period);
            /* note: there is no flag if we missed an underflow */
        } else {
            /* period is 2**64 */
            *timer->reg_value = *timer->reg_reload - elapsed;
        }
        return true;
    }
}

static void kvx_timer_rearm(KVXTimer *timer)
{
    const uint64_t max_val = (INT64_MAX / timer->period_ns) - 1;
    int64_t deadline;
    if (*timer->reg_value > max_val) {
        /* timer is scheduled in more than ~300 years ... (it'll never happen) */
        timer_del(timer->qemu_timer);
    } else {
        deadline = (*timer->reg_value + 1) * timer->period_ns;
        deadline += timer->timestamp;
        timer_mod_ns(timer->qemu_timer, deadline);
    }
}

void kvx_timer_update(KVXTimer *timer)
{
    KVXCPU *cpu = timer->cpu;
    uint64_t tcr = kvx_register_read_u64(&cpu->env, REG_kv3_TCR);
    bool enable, underflow = false, ev_enable;

    enable = tcr & timer->tcr_enable_bit;
    // TODO: take idle states and timer->tcr_idle_bit in account to compute 'enable'
    ev_enable = enable &&
        tcr & (timer->tcr_interrupt_bit | timer->tcr_watchdog_bit);

    if (timer->enabled) {
        /* first update the counter value */
        underflow = kvx_timer_update_value(timer);
    } else if (enable) {
        timer->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }

    /* update the qemu_timer if needed */
    if (ev_enable && (!timer->ev_enabled || underflow)) {
        /* we need to start/restart the qemu_timer */
        kvx_timer_rearm(timer);
    } else if (!ev_enable && timer->ev_enabled) {
        /* we do not need the qemu_timer any more */
        timer_del(timer->qemu_timer);
    }

    timer->enabled = enable;
    timer->ev_enabled = ev_enable;

    /* last, update TCR and trigger cpu's side effects */
    if (underflow) {
        /*
         * Note: we keep the old value in 'tcr' to have the previous
         * status bit and check for watchdog
         */
        kvx_register_write_u64(&timer->cpu->env, REG_kv3_TCR,
                               tcr | timer->tcr_status_bit);

        if (tcr & timer->tcr_interrupt_bit) {
            qemu_irq_pulse(timer->irq);
        }

        if ((tcr & timer->tcr_status_bit) && (tcr & timer->tcr_watchdog_bit)) {
            kvx_cpu_trigger_watchdog(cpu);
        }
    }
}

void kvx_timer_write_value(KVXTimer *timer, uint64_t value)
{
    *timer->reg_value = value;
    timer->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (timer->ev_enabled) {
        kvx_timer_rearm(timer);
    }
}

void kvx_timer_reset(KVXTimer *timer)
{
    timer->enabled = false;
    timer->ev_enabled = false;
    timer_del(timer->qemu_timer);
}
