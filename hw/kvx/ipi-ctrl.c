/*
 * Kalray KVX MPPA cluster IPI controller
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
#include "hw/kvx/ipi-ctrl.h"
#include "cpu.h"
#include "trace.h"

REG32(INTERRUPT, 0x0)
REG32(MASK, 0x20)

static void kvx_ipi_send_irq(KvxIpiCtrlState *s, uint32_t mask)
{
    size_t i = 0;

    mask &= (1 << KVX_IPI_CTRL_NUM_IRQ) - 1;
    mask &= ~s->reg_mask;

    while (mask) {
        if (mask & 0x1) {
            trace_kvx_ipi_ctrl_send_irq(i);
            qemu_irq_pulse(s->irq_out[i]);
        }

        i++;
        mask >>= 1;
    }
}

static uint64_t kvx_ipi_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    KvxIpiCtrlState *s = KVX_IPI_CTRL(opaque);
    uint64_t ret;

    switch (offset) {
    case A_MASK:
        ret = s->reg_mask;
        break;

    case A_INTERRUPT:
        /* Read as zero? */
        ret = 0;
        break;

    default:
        ret = 0;
    }

    trace_kvx_ipi_ctrl_read(offset, ret);

    return ret;
}

static void kvx_ipi_ctrl_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    KvxIpiCtrlState *s = KVX_IPI_CTRL(opaque);

    trace_kvx_ipi_ctrl_write(offset, value);

    switch (offset) {
    case A_MASK:
        s->reg_mask = value;
        break;

    case A_INTERRUPT:
        kvx_ipi_send_irq(s, value);
        break;
    }
}

static const MemoryRegionOps kvx_ipi_ctrl_ops = {
    .read = kvx_ipi_ctrl_read,
    .write = kvx_ipi_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void kvx_ipi_ctrl_reset(DeviceState *dev)
{
    KvxIpiCtrlState *s = KVX_IPI_CTRL(dev);

    s->reg_mask = 0;
}

static void kvx_ipi_ctrl_init(Object *obj)
{
    KvxIpiCtrlState *s = KVX_IPI_CTRL(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_ipi_ctrl_ops, s,
                          TYPE_KVX_IPI_CTRL, KVX_IPI_CTRL_MMIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq_out, KVX_IPI_CTRL_NUM_IRQ);
}

static void kvx_ipi_ctrl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_ipi_ctrl_reset;
}

static TypeInfo kvx_ipi_ctrl_info = {
    .name          = TYPE_KVX_IPI_CTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxIpiCtrlState),
    .class_init    = kvx_ipi_ctrl_class_init,
    .instance_init = kvx_ipi_ctrl_init,
};

static void kvx_ipi_ctrl_register_types(void)
{
    type_register_static(&kvx_ipi_ctrl_info);
}

type_init(kvx_ipi_ctrl_register_types)
