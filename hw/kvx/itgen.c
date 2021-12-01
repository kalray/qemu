/*
 * Kalray KVX MPPA cluster itgen IRQ controller
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
#include "qemu/units.h"
#include "qemu/log.h"
#include "exec/address-spaces.h"
#include "hw/registerfields.h"
#include "hw/kvx/itgen.h"
#include "cpu.h"
#include "trace.h"

REG32(CFG, 0x0)
    FIELD(CFG, MAILBOX, 0, 7)
    FIELD(CFG, CLUSTER, 8, 3)
    FIELD(CFG, BIT_SELECT, 18, 6)

REG32(ENABLE, 0x8)
    FIELD(ENABLE, IRQ, 0, 1)

REG32(PARAM, 0x1100)

/* This address is hardcoded into the controller */
const uint64_t MAILBOX_ADDR = 0x00a00000;
#define MB_ADDR_CLUSTER_SHIFT	24
#define MB_ADDR_MAILBOX_SHIFT	9

static inline uint64_t decode_mailbox_addr(const KvxItgenIrq *irq)
{
    uint64_t ret = MAILBOX_ADDR;

    ret |= FIELD_EX64(irq->cfg, CFG, MAILBOX) << MB_ADDR_MAILBOX_SHIFT;
    ret |= (FIELD_EX64(irq->cfg, CFG, CLUSTER) + 1) << MB_ADDR_CLUSTER_SHIFT;

    return ret;
}

static void kvx_itgen_irq_update(void *opaque, int irq_idx, int level)
{
    KvxItgenState *s = KVX_ITGEN(opaque);
    KvxItgenIrq *irq;

    /* Assuming edge triggered IRQs */
    if (!level) {
        return;
    }

    irq = &s->irqs[irq_idx];

    if (irq->enable) {
        uint64_t addr = decode_mailbox_addr(irq);
        uint64_t val = 1ull << FIELD_EX64(irq->cfg, CFG, BIT_SELECT);

        trace_kvx_itgen_send_msi(addr, val);
        address_space_write(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, &val, sizeof(val));
    }
}

static inline uint32_t kvx_itgen_irq_read(KvxItgenIrq *irq, size_t offset)
{
    switch (offset) {
    case A_CFG:
        return irq->cfg;

    case A_ENABLE:
        return irq->enable;

    default:
        return 0;
    }
}

static uint64_t kvx_itgen_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    KvxItgenState *s = KVX_ITGEN(opaque);
    uint64_t ret;
    size_t irq_idx;

    switch (offset) {
    case A_PARAM:
        ret = KVX_ITGEN_NUM_IRQ_IN;
        break;

    default:
        irq_idx = offset / KVX_ITGEN_IRQ_SIZE;
        ret = kvx_itgen_irq_read(&s->irqs[irq_idx],
                                 offset % KVX_ITGEN_IRQ_SIZE);
    }

    trace_kvx_itgen_read(offset, ret);

    return ret;
}

static inline void kvx_itgen_irq_write(KvxItgenIrq *irq, size_t offset,
                                       uint32_t value)
{
    switch (offset) {
    case A_CFG:
        irq->cfg = value;
        break;

    case A_ENABLE:
        irq->enable = value;
        break;

    default:
        break;
    }
}

static void kvx_itgen_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    KvxItgenState *s = KVX_ITGEN(opaque);
    size_t irq_idx;

    trace_kvx_itgen_write(offset, value);

    switch (offset) {
    case A_PARAM:
        /* read only? */
        return;

    default:
        irq_idx = offset / KVX_ITGEN_IRQ_SIZE;
        kvx_itgen_irq_write(&s->irqs[irq_idx],
                            offset % KVX_ITGEN_IRQ_SIZE, value);
    }

}

static const MemoryRegionOps kvx_itgen_ops = {
    .read = kvx_itgen_read,
    .write = kvx_itgen_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        /* assume unaligned accesses are prohibited */
        .unaligned = false,
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void kvx_itgen_reset(DeviceState *dev)
{
    KvxItgenState *s = KVX_ITGEN(dev);

    memset(s->irqs, 0, sizeof(s->irqs));
}

static void kvx_itgen_init(Object *obj)
{
    KvxItgenState *s = KVX_ITGEN(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_itgen_ops, s,
                          TYPE_KVX_ITGEN, KVX_ITGEN_MMIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    qdev_init_gpio_in(DEVICE(obj), kvx_itgen_irq_update, KVX_ITGEN_NUM_IRQ_IN);
}

static void kvx_itgen_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_itgen_reset;
}

static TypeInfo kvx_itgen_info = {
    .name          = TYPE_KVX_ITGEN,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxItgenState),
    .class_init    = kvx_itgen_class_init,
    .instance_init = kvx_itgen_init,
};

static void kvx_itgen_register_types(void)
{
    type_register_static(&kvx_itgen_info);
}

type_init(kvx_itgen_register_types)
