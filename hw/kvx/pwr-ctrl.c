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

/* 0x40 between each pe_control_reg */
REG64(PE_CONTROL, 0x0000)
REG64(PE_CONTROL_SET, 0x0010)
REG64(PE_CONTROL_CLEAR, 0x0020)
    FIELD(PE_CONTROL, WAKEUP, 0, 1)
    FIELD(PE_CONTROL, RESET_ON_WAKEUP, 1, 1)
    FIELD(PE_CONTROL, WD_ACK, 10, 1)
#define PE_CONTROL_MASK (R_PE_CONTROL_WAKEUP_MASK \
                           | R_PE_CONTROL_RESET_ON_WAKEUP_MASK \
                           | R_PE_CONTROL_WD_ACK_MASK)

REG64(PE_STATUS, 0x0400)

REG64(VEC_WUP, 0x1000)
REG64(VEC_WUP_SET, 0x1010)
REG64(VEC_WUP_CLEAR, 0x1020)

REG64(VEC_RESET_ON_WAKEUP, 0x1040)
REG64(VEC_RESET_ON_WAKEUP_SET, 0x1050)
REG64(VEC_RESET_ON_WAKEUP_CLEAR, 0x1060)

REG64(RESET_PC, 0x2000)

REG64(GLOBAL_CONF, 0x4040)
REG64(GLOBAL_CONF_SET, 0x4050)
REG64(GLOBAL_CONF_CLEAR, 0x4060)
    FIELD(GLOBAL_CONF, USER_EN, 0, 1)
    FIELD(GLOBAL_CONF, PE_EN, 1, 1)
    FIELD(GLOBAL_CONF, L2_CACHE_EN, 2, 1)
    FIELD(GLOBAL_CONF, L2_CACHE_CONFIG, 3, 1)
    FIELD(GLOBAL_CONF, SECURE_MST_WEAK, 8, 24)
    FIELD(GLOBAL_CONF, DSU_ECC_BYPASS, 33, 1)
    FIELD(GLOBAL_CONF, SMEM_ECC_BYPASS, 34, 1)
    FIELD(GLOBAL_CONF, SMEM_BURST_LEN, 35, 3)
    FIELD(GLOBAL_CONF, AXI_BURST_DIS, 38, 1)
    FIELD(GLOBAL_CONF, LOCK, 63, 1)
#define GLOBAL_CONF_MASK (R_GLOBAL_CONF_USER_EN_MASK \
                           | R_GLOBAL_CONF_PE_EN_MASK \
                           | R_GLOBAL_CONF_L2_CACHE_EN_MASK \
                           | R_GLOBAL_CONF_L2_CACHE_CONFIG_MASK \
                           | R_GLOBAL_CONF_SECURE_MST_WEAK_MASK \
                           | R_GLOBAL_CONF_DSU_ECC_BYPASS_MASK \
                           | R_GLOBAL_CONF_SMEM_ECC_BYPASS_MASK \
                           | R_GLOBAL_CONF_SMEM_BURST_LEN_MASK \
                           | R_GLOBAL_CONF_AXI_BURST_DIS_MASK \
                           | R_GLOBAL_CONF_LOCK_MASK )

REG64(RM_CONTROL, 0x40C0)
REG64(RM_CONTROL_SET, 0x40D0)
REG64(RM_CONTROL_CLEAR, 0x40E0)

static inline void kvx_pwr_ctrl_update_cpus(KvxPwrCtrlState *s)
{
    uint16_t wup = s->reg_vector_proc_control_wup;

    CPUState *cpu;

    /* XXX maybe? */
    if (!FIELD_EX64(s->reg_global_config, GLOBAL_CONF, PE_EN)) {
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

    int pe_id = 0;
    hwaddr offset_decoded = offset;

    if (offset < A_PE_STATUS) {
        pe_id = offset / 0x40;
        offset_decoded = offset - pe_id * 0x40;
    }

    switch (offset_decoded) {
    case A_PE_CONTROL:
        ret = s->reg_pe_control[pe_id];
        break;

    case A_VEC_WUP:
        ret = s->reg_vector_proc_control_wup;
        break;

    case A_VEC_RESET_ON_WAKEUP:
        ret = s->reg_vector_proc_control_reset_on_wakeup;
        break;

    case A_GLOBAL_CONF:
        ret = s->reg_global_config;
        break;

    case A_RESET_PC:
        ret = s->reg_reset_pc;
        break;

    case A_RM_CONTROL:
        ret = s->reg_rm_control;
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

    int pe_id = 0;
    hwaddr offset_decoded = offset;

    if (offset < A_PE_STATUS) {
        pe_id = offset / 0x40;
        offset_decoded =  offset - (pe_id * 0x40);
    }

    switch (offset_decoded) {
    case A_PE_CONTROL:
       s->reg_pe_control[pe_id] = (value & PE_CONTROL_MASK);
        break;
    case A_PE_CONTROL_SET:
       s->reg_pe_control[pe_id] |= (value & PE_CONTROL_MASK);
        break;
    case A_PE_CONTROL_CLEAR:
       s->reg_pe_control[pe_id] &= ~(value & PE_CONTROL_MASK);
        break;

    case A_VEC_WUP:
        s->reg_vector_proc_control_wup = (value & 0xffff);
        kvx_pwr_ctrl_update_cpus(s);
        break;
    case A_VEC_WUP_SET:
        s->reg_vector_proc_control_wup |= (value & 0xffff);
        kvx_pwr_ctrl_update_cpus(s);
        break;
    case A_VEC_WUP_CLEAR:
        s->reg_vector_proc_control_wup &= ~(value & 0xffff);
        break;

    case A_VEC_RESET_ON_WAKEUP:
        s->reg_vector_proc_control_wup = (value & 0xffff);
        break;
    case A_VEC_RESET_ON_WAKEUP_SET:
        s->reg_vector_proc_control_wup |= (value & 0xffff);
        break;
    case A_VEC_RESET_ON_WAKEUP_CLEAR:
        s->reg_vector_proc_control_wup &= ~(value & 0xffff);
        break;

    case A_RESET_PC:
        s->reg_reset_pc = value & ~((4ull * KiB) - 1);
        break;


    /* TODO side effect on L2 cache */
    case A_GLOBAL_CONF:
        s->reg_global_config = (value & GLOBAL_CONF_MASK);
        break;
    case A_GLOBAL_CONF_SET:
        s->reg_global_config |= (value & GLOBAL_CONF_MASK);
        break;
    case A_GLOBAL_CONF_CLEAR:
        s->reg_global_config &= ~(value & GLOBAL_CONF_MASK);
        break;

    case A_RM_CONTROL:
        s->reg_rm_control = (value & 0x0403);
        break;
    case A_RM_CONTROL_SET:
        s->reg_rm_control |= (value & 0x0403);
        break;
    case A_RM_CONTROL_CLEAR:
        s->reg_rm_control &= ~(value & 0x0403);
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

    for (int num_pe = 0; num_pe < 16; num_pe++) {
        s->reg_pe_control[num_pe] = 0;
    }
    s->reg_vector_proc_control_wup = 0;
    s->reg_vector_proc_control_reset_on_wakeup = 0;
    s->reg_reset_pc = 0;
    s->reg_global_config = 0x00000008ffffff00;
    s->reg_rm_control = 0;

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
