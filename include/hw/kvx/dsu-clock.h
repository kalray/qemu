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

#ifndef HW_KVX_DSU_CLOCK_H
#define HW_KVX_DSU_CLOCK_H

#include "hw/sysbus.h"

#define TYPE_KVX_DSU_CLOCK "kvx-dsu-clock"
#define KVX_DSU_CLOCK(obj) \
    OBJECT_CHECK(KvxDsuClockState, (obj), TYPE_KVX_DSU_CLOCK)

/*
 * according to the device tree. But this thing is probably part of a bigger
 * peripheral.
 */
#define KVX_DSU_CLOCK_MMIO_LEN 8

typedef struct KvxDsuClockState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    uint64_t reset_val;
    MemoryRegion iomem;
} KvxDsuClockState;

#endif
