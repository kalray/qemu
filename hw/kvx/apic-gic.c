/*
 * Kalray KVX MPPA cluster APIC GIC
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
#include "hw/kvx/apic-gic.h"
#include "cpu.h"
#include "trace.h"

REG8(ENABLE, 0)
    FIELD(ENABLE, IRQ, 0, 1)

REG64(LAC, 0x120)

/*
 * @return true if the given @irq_in is enabled for the given @irq_out output
 * line.
 */
static bool irq_in_is_enabled(const KvxApicGicState *s,
                              size_t irq_out, size_t irq_in)
{
    size_t idx = irq_out >> 6;
    size_t bit = irq_out & 0x3f;

    return extract64(s->irq_in_enabled[irq_in][idx], bit, 1);
}

static void kvx_apic_gic_irq_in_update(void *opaque, int irq_in, int level)
{
    size_t irq_out;
    size_t lac_bit;
    size_t lac_idx;
    KvxApicGicState *s = KVX_APIC_GIC(opaque);

    trace_kvx_apic_gic_irq_in_update(irq_in, level);

    if (!level) {
        return;
    }

    lac_idx = irq_in_to_lac_idx_bit(irq_in, &lac_bit);

    for (irq_out = 0; irq_out < KVX_APIC_GIC_NUM_IRQ_OUT; irq_out++) {
        if (irq_in_is_enabled(s, irq_out, irq_in)) {
            /*
             * Assume the corresponding LAC bit is not set when the input IRQ
             * is disabled for a given output line.
             */
            uint64_t *lac = &s->irq_in_set[irq_out][lac_idx];
            *lac = deposit64(*lac, lac_bit, true, 1);
            qemu_irq_raise(s->irq_out[irq_out]);
        }
    }
}

static inline uint64_t reg_enable_read(KvxApicGicState *s, size_t group,
                                       size_t line, size_t irq_in)
{
    size_t irq_out;

    g_assert(irq_in < KVX_APIC_GIC_NUM_IRQ_IN);

    irq_out = group_line_to_irq_out_idx(group, line);
    return irq_in_is_enabled(s, irq_out, irq_in);
}

static inline uint64_t reg_lac_read(KvxApicGicState *s, size_t group,
                                    size_t line, size_t lac_idx)
{
    uint64_t ret;
    size_t irq_out, i;
    bool has_active_irqs = true;

    g_assert(lac_idx < KVX_APIC_GIC_NUM_REG_LAC);

    irq_out = group_line_to_irq_out_idx(group, line);

    ret = s->irq_in_set[irq_out][lac_idx];

    /* Acknowledge those 64 irqs */
    s->irq_in_set[irq_out][lac_idx] = 0;

    /* Check for other active IRQs */
    for (i = 0; i < ARRAY_SIZE(s->irq_in_set[irq_out]); i++) {
        has_active_irqs |= s->irq_in_set[irq_out][i];
    }

    /* When no more active IRQs, lower the output line */
    if (!has_active_irqs) {
        qemu_irq_lower(s->irq_out[irq_out]);
    }

    return ret;
}

static inline uint64_t kvx_apic_gic_group_read(KvxApicGicState *s, size_t group,
                                               size_t line, hwaddr offset,
                                               unsigned size)
{
    uint64_t ret = 0;

    if (offset < A_LAC) {
        size_t i;

        if (offset > KVX_APIC_GIC_NUM_IRQ_IN) {
            /* Nothing here? */
            return 0;
        }

        for (i = 0; i < size; i++) {
            uint64_t enable;
            enable = reg_enable_read(s, group, line, offset + i);
            ret |= enable << (i * 8);
        }
    } else if (offset < A_LAC + KVX_APIC_GIC_NUM_REG_LAC * sizeof(uint64_t)) {
        /* assume that only 64 bits reads are allowed */
        if (size != 8) {
            return 0;
        }

        ret = reg_lac_read(s, group, line,
                           (offset - A_LAC) / sizeof(uint64_t));
    }

    trace_kvx_apic_gic_read(group, line, offset, ret, size);

    return ret;
}

static uint64_t kvx_apic_gic_read(void *opaque, hwaddr offset, unsigned size)
{
    KvxApicGicState *s = KVX_APIC_GIC(opaque);
    size_t group, line;

    group = offset / KVX_APIC_GIC_GROUP_SIZE;
    offset %= KVX_APIC_GIC_GROUP_SIZE;

    line = offset / KVX_APIC_GIC_LINE_SIZE;
    offset %= KVX_APIC_GIC_LINE_SIZE;

    if (group > KVX_APIC_GIC_NUM_GROUP) {
        /* are there more registers? */
        return 0;
    }

    return kvx_apic_gic_group_read(s, group, line, offset, size);
}

static inline void reg_enable_write(KvxApicGicState *s, size_t group,
                                    size_t line, size_t irq_in, uint8_t value)
{
    bool enabled = FIELD_EX8(value, ENABLE, IRQ);
    size_t idx, bit;
    uint64_t *val;

    g_assert(irq_in < KVX_APIC_GIC_NUM_IRQ_IN);

    bit = group_line_to_irq_out_idx(group, line);
    idx = bit >> 6;
    bit = bit & 0x3f;

    val = &s->irq_in_enabled[irq_in][idx];

    *val = deposit64(*val, bit, 1, enabled);

    /* TODO: update */
}

static inline void kvx_apic_gic_group_write(KvxApicGicState *s, size_t group,
                                            size_t line, hwaddr offset,
                                            uint64_t value, unsigned size)
{
    trace_kvx_apic_gic_write(group, line, offset, value, size);

    if (offset < A_LAC) {
        size_t i;

        if (offset > KVX_APIC_GIC_NUM_IRQ_IN) {
            /* Nothing here? */
            return;
        }

        for (i = 0; i < size; i++) {
            reg_enable_write(s, group, line, offset + i, value >> (8 * i));
        }
    } else if (offset < A_LAC + KVX_APIC_GIC_NUM_REG_LAC * sizeof(uint64_t)) {
        /* LAC read only? */
    }
}

static void kvx_apic_gic_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    KvxApicGicState *s = KVX_APIC_GIC(opaque);
    size_t group, line;

    group = offset / KVX_APIC_GIC_GROUP_SIZE;
    offset %= KVX_APIC_GIC_GROUP_SIZE;

    line = offset / KVX_APIC_GIC_LINE_SIZE;
    offset %= KVX_APIC_GIC_LINE_SIZE;

    if (group > KVX_APIC_GIC_NUM_GROUP) {
        /* are there more registers? */
        return;
    }

    kvx_apic_gic_group_write(s, group, line, offset, value, size);
}

static const MemoryRegionOps kvx_apic_gic_ops = {
    .read = kvx_apic_gic_read,
    .write = kvx_apic_gic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        /* assume unaligned accesses are prohibited */
        .unaligned = false,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    }
};

static void kvx_apic_gic_reset(DeviceState *dev)
{
    KvxApicGicState *s = KVX_APIC_GIC(dev);

    memset(s->irq_in_enabled, 0, sizeof(s->irq_in_enabled));
    memset(s->irq_in_set, 0, sizeof(s->irq_in_set));
}

static void kvx_apic_gic_init(Object *obj)
{
    KvxApicGicState *s = KVX_APIC_GIC(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_apic_gic_ops, s,
                          TYPE_KVX_APIC_GIC, KVX_APIC_GIC_MMIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    qdev_init_gpio_in(DEVICE(obj), kvx_apic_gic_irq_in_update, KVX_APIC_GIC_NUM_IRQ_IN);
    qdev_init_gpio_out(DEVICE(obj), s->irq_out, KVX_APIC_GIC_NUM_IRQ_OUT);
}

static void kvx_apic_gic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_apic_gic_reset;
}

static TypeInfo kvx_apic_gic_info = {
    .name          = TYPE_KVX_APIC_GIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxApicGicState),
    .class_init    = kvx_apic_gic_class_init,
    .instance_init = kvx_apic_gic_init,
};

static void kvx_apic_gic_register_types(void)
{
    type_register_static(&kvx_apic_gic_info);
}

type_init(kvx_apic_gic_register_types)
