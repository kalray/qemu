/*
 * Kalray KVX MPPA cluster DMA
 *
 * Copyright (c) 2021 Kalray Inc.
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
#include "qemu/main-loop.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/dma/kvx-dma.h"

#include "trace.h"
#include "registers.h"

static uint64_t kvx_dma_read(void *opaque, hwaddr offset, unsigned size)
{
    uint64_t ret = 0;

    trace_kvx_dma_read(offset, ret);
    return ret;
}

static void kvx_dma_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned int size)
{
    trace_kvx_dma_write(offset, value);
}

static const MemoryRegionOps kvx_dma_ops = {
    .read = kvx_dma_read,
    .write = kvx_dma_write,
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

static void kvx_dma_reset(DeviceState *dev)
{
}

static void kvx_dma_realize(DeviceState *dev, Error **errp)
{
    KvxDmaState *s = KVX_DMA(dev);

    address_space_init(&s->as, s->root_mr, "kvx-dma");
}

static void kvx_dma_init(Object *obj)
{
    KvxDmaState *s = KVX_DMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_dma_ops, s,
                          TYPE_KVX_DMA, KVX_DMA_MMIO_LEN);

    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property kvx_dma_properties[] = {
    DEFINE_PROP_LINK("root-mr", KvxDmaState, root_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST()
};

static void kvx_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_dma_reset;
    dc->realize = kvx_dma_realize;
    device_class_set_props(dc, kvx_dma_properties);
}

static TypeInfo kvx_dma_info = {
    .name          = TYPE_KVX_DMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxDmaState),
    .class_init    = kvx_dma_class_init,
    .instance_init = kvx_dma_init,
};

static void kvx_dma_register_types(void)
{
    type_register_static(&kvx_dma_info);
}

type_init(kvx_dma_register_types)
