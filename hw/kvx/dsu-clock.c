/*
 * Kalray KVX MPPA cluster DSU clock
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
#include "qemu/log.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/kvx/dsu-clock.h"
#include "cpu.h"
#include "trace.h"

REG64(CLOCK, 0x0)

static uint64_t kvx_dsu_clock_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    KvxDsuClockState *s = KVX_DSU_CLOCK(opaque);

    /*
     * The clock rate is 1MHz. Let's return a tick value from the virtual clock
     * in us.
     */
    uint64_t val = s->initial_val
        + qemu_clock_get_us(QEMU_CLOCK_VIRTUAL)
        - s->reset_val;

    return val;
}

static void kvx_dsu_clock_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    /* assume read only */
}

static const MemoryRegionOps kvx_dsu_clock_ops = {
    .read = kvx_dsu_clock_read,
    .write = kvx_dsu_clock_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        /* assume unaligned accesses are prohibited */
        .unaligned = false,
        .min_access_size = 8,
        .max_access_size = 8,
    },
    .impl = {
        .unaligned = false,
        .min_access_size = 8,
        .max_access_size = 8,
    },
};

static void kvx_dsu_clock_reset(DeviceState *dev)
{
    KvxDsuClockState *s = KVX_DSU_CLOCK(dev);

    s->reset_val = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
}

static void kvx_dsu_clock_init(Object *obj)
{
    KvxDsuClockState *s = KVX_DSU_CLOCK(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_dsu_clock_ops, s,
                          TYPE_KVX_DSU_CLOCK, KVX_DSU_CLOCK_MMIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static Property kvx_dsu_clock_properties[] = {
    DEFINE_PROP_UINT64("initial-value", KvxDsuClockState, initial_val, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void kvx_dsu_clock_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_dsu_clock_reset;
    device_class_set_props(dc, kvx_dsu_clock_properties);
}

static TypeInfo kvx_dsu_clock_info = {
    .name          = TYPE_KVX_DSU_CLOCK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxDsuClockState),
    .instance_init = kvx_dsu_clock_init,
    .class_init    = kvx_dsu_clock_class_init,
};

static void kvx_dsu_clock_register_types(void)
{
    type_register_static(&kvx_dsu_clock_info);
}

type_init(kvx_dsu_clock_register_types)
