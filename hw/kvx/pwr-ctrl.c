/*
 * Kalray KVX MPPA cluster power controller
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

/*
 * Note: this model has been written without any documentation. It was
 *       developped by retro-engineering the corresponding Linux kernel driver.
 *       It is incomplete and probably wrong in some places.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/registerfields.h"
#include "hw/kvx/pwr-ctrl.h"
#include "cpu.h"
#include "trace.h"

REG64(VEC, 0x1000)
REG64(VEC_WUP_SET, 0x1010)
REG64(VEC_WUP_CLEAR, 0x1020)

REG64(RESET_PC, 0x2000)

REG64(GLOBAL, 0x4040)
REG64(GLOBAL_SET, 0x4050)
    FIELD(GLOBAL_SET, PE_EN, 1, 1)

static inline void kvx_pwr_ctrl_update_cpus(KvxPwrCtrlState *s)
{
    uint16_t wup = s->reg_vec;

    CPUState *cpu;

    /* XXX maybe? */
    if (!FIELD_EX64(s->reg_global, GLOBAL_SET, PE_EN)) {
        return;
    }

    CPU_FOREACH(cpu) {
        KVXCPU *kvx_cpu = KVX_CPU(cpu);
        CPUKVXState *env = &kvx_cpu->env;
        int pid;

        if (env->sleep_state != KVX_RESETTING) {
            continue;
        }

        pid = kvx_cpu->cfg.pid;

        if ((1 << pid) & wup) {
            trace_kvx_pwr_ctrl_wakeup_cpu(kvx_cpu->cfg.pid, s->reg_reset_pc);
            cpu_set_pc(cpu, s->reg_reset_pc);
            env->sleep_state = KVX_RUNNING;
            qemu_cpu_kick(cpu);
        }
    }
}

static uint64_t kvx_pwr_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    KvxPwrCtrlState *s = KVX_PWR_CTRL(opaque);
    uint64_t ret;

    switch (offset) {
    case A_VEC:
        ret = s->reg_vec;
        break;

    case A_GLOBAL:
        ret = s->reg_global;
        break;

    case A_RESET_PC:
        ret = s->reg_reset_pc;
        break;

    default:
        ret = 0;
    }

    trace_kvx_pwr_ctrl_read(offset, ret);

    return ret;
}

static void kvx_pwr_ctrl_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    KvxPwrCtrlState *s = KVX_PWR_CTRL(opaque);

    trace_kvx_pwr_ctrl_write(offset, value);

    switch (offset) {
    case A_VEC_WUP_SET:
        s->reg_vec |= value & 0xffff;
        kvx_pwr_ctrl_update_cpus(s);
        break;

    case A_VEC_WUP_CLEAR:
        s->reg_vec &= ~(value & 0xffff);
        break;

    case A_GLOBAL_SET:
        s->reg_global |= value;
        break;

    case A_RESET_PC:
        s->reg_reset_pc = value & ~((4ull * KiB) - 1);
        break;
    }
}

static const MemoryRegionOps kvx_pwr_ctrl_ops = {
    .read = kvx_pwr_ctrl_read,
    .write = kvx_pwr_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
};

static void kvx_pwr_ctrl_reset(DeviceState *dev)
{
    KvxPwrCtrlState *s = KVX_PWR_CTRL(dev);

    s->reg_reset_pc = 0;
    s->reg_global = 0;
    s->reg_vec = 0;
}

static void kvx_pwr_ctrl_init(Object *obj)
{
    KvxPwrCtrlState *s = KVX_PWR_CTRL(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_pwr_ctrl_ops, s,
                          TYPE_KVX_PWR_CTRL, KVX_PWR_CTRL_MMIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void kvx_pwr_ctrl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_pwr_ctrl_reset;
}

static TypeInfo kvx_pwr_ctrl_info = {
    .name          = TYPE_KVX_PWR_CTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxPwrCtrlState),
    .class_init    = kvx_pwr_ctrl_class_init,
    .instance_init = kvx_pwr_ctrl_init,
};

static void kvx_pwr_ctrl_register_types(void)
{
    type_register_static(&kvx_pwr_ctrl_info);
}

type_init(kvx_pwr_ctrl_register_types)
