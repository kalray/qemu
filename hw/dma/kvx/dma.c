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
#include "internals.h"

typedef uint64_t (*KvxDmaRegGroupReadFn)(KvxDmaState *, size_t id,
                                         hwaddr off, unsigned int size);

typedef void (*KvxDmaRegGroupWriteFn)(KvxDmaState *, size_t id,
                                      hwaddr off, uint64_t value,
                                      unsigned int size);

typedef struct KvxDmaRegGroupAccess {
    KvxDmaRegGroupReadFn read;
    KvxDmaRegGroupWriteFn write;
} KvxDmaRegGroupAccess;

static uint64_t unimp_group_read(KvxDmaState *s, size_t id, hwaddr offset,
                                 unsigned int size)
{
    return 0;
}

static void unimp_group_write(KvxDmaState *s, size_t id, hwaddr offset,
                              uint64_t value, unsigned int size)
{
}

static const KvxDmaRegGroupAccess KVX_DMA_GROUP_ACCESS[] = {
    [GROUP_RX_CHANNEL] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_RX_JOB_QUEUE] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_RX_JOB_CACHE] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_RX_DIAG] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_RX_MUX] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_RX_MONITORING] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_TX_THREAD] = { .read = unimp_group_read, .write = unimp_group_write },

    [GROUP_DMA_IT] = {
        .read = kvx_dma_it_read,
        .write = kvx_dma_it_write,
    },

    [GROUP_DMA_ERROR] = {
        .read = kvx_dma_errors_read,
        .write = kvx_dma_errors_write,
    },

    [GROUP_TX_PGRM_MEM] = {
        .read = kvx_dma_tx_pgrm_mem_read,
        .write = kvx_dma_tx_pgrm_mem_write,
    },

    [GROUP_TX_PGRM_TABLE] = {
        .read = kvx_dma_tx_pgrm_table_read,
        .write = kvx_dma_tx_pgrm_table_write,
    },

    [GROUP_NOC_ROUTE_TABLE] = {
        .read = kvx_dma_noc_route_table_read,
        .write = kvx_dma_noc_route_table_write,
    },

    [GROUP_BW_LIMITER_TABLE] = {
        .read = kvx_dma_bw_limiter_table_read,
        .write = kvx_dma_bw_limiter_table_write,
    },

    [GROUP_TX_MONITORING] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_TX_NOC_TEST] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_TX_JOB_QUEUE] = { .read = unimp_group_read, .write = unimp_group_write },
    [GROUP_TX_COMP_QUEUE] = { .read = unimp_group_read, .write = unimp_group_write },
};

static inline KvxDmaRegGroup decode_mmio_offset(hwaddr *offset, size_t *id)
{
    KvxDmaRegGroup group;
    uint64_t group_start;
    uint64_t group_sz;

    for (group = 0; group < GROUP_END; group++) {
        group_start = KVX_DMA_GROUP_MMIO_START[group + 1];

        if (*offset < group_start) {
            break;
        }
    }

    g_assert(group < GROUP_END);

    group_start = KVX_DMA_GROUP_MMIO_START[group];
    group_sz = kvx_dma_group_mmio_size(group);

    *offset -= group_start;
    *id = *offset / group_sz;
    *offset = *offset % group_sz;

    return group;
}

static uint64_t kvx_dma_read(void *opaque, hwaddr offset, unsigned size)
{
    KvxDmaState *s = KVX_DMA(opaque);
    KvxDmaRegGroup group;
    size_t id;
    uint64_t ret;
    hwaddr sub_offset = offset;

    group = decode_mmio_offset(&sub_offset, &id);

    ret = KVX_DMA_GROUP_ACCESS[group].read(s, id, sub_offset, size);

    trace_kvx_dma_read(offset, ret);
    return ret;
}

static void kvx_dma_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned int size)
{
    KvxDmaState *s = KVX_DMA(opaque);
    KvxDmaRegGroup group;
    size_t id;
    hwaddr sub_offset = offset;

    group = decode_mmio_offset(&sub_offset, &id);

    trace_kvx_dma_write(offset, value);

    KVX_DMA_GROUP_ACCESS[group].write(s, id, sub_offset, value, size);
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
    KvxDmaState *s = KVX_DMA(dev);

    kvx_dma_irq_errors_reset(s);
    kvx_dma_mem_reset(s);
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
