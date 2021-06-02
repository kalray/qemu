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

#ifndef HW_KVX_ITGEN_H
#define HW_KVX_ITGEN_H

#include "hw/sysbus.h"

#define TYPE_KVX_ITGEN "kvx-itgen"
#define KVX_ITGEN(obj) \
    OBJECT_CHECK(KvxItgenState, (obj), TYPE_KVX_ITGEN)

#define KVX_ITGEN_NUM_IRQ_IN 272

#define KVX_ITGEN_IRQ_SIZE 0x10

/* matches the device tree */
#define KVX_ITGEN_MMIO_LEN \
    (KVX_ITGEN_NUM_IRQ_IN * KVX_ITGEN_IRQ_SIZE) + 4

typedef struct KvxItgenIrq {
    uint32_t cfg;
    uint32_t enable;
} KvxItgenIrq;

typedef struct KvxItgenState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    KvxItgenIrq irqs[KVX_ITGEN_NUM_IRQ_IN];

    MemoryRegion iomem;
} KvxItgenState;

#endif
