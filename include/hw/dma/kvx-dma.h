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

#ifndef HW_KVX_DMA_H
#define HW_KVX_DMA_H

#include "hw/sysbus.h"

#define TYPE_KVX_DMA "kvx-dma"
#define KVX_DMA(obj) \
    OBJECT_CHECK(KvxDmaState, (obj), TYPE_KVX_DMA)

#define KVX_DMA_MMIO_LEN 0x100000

typedef struct KvxDmaState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    MemoryRegion iomem;

    MemoryRegion *root_mr;
    AddressSpace as;

    qemu_irq irq;
} KvxDmaState;

#endif
