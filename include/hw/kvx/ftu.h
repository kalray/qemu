/*
 * Kalray KVX MPPA FTU
 *
 * Copyright (c) 2021 Kalray
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

#ifndef HW_KVX_FTU_H
#define HW_KVX_FTU_H

#include "hw/sysbus.h"

#define TYPE_KVX_FTU "kvx-ftu"
#define KVX_FTU(obj) \
    OBJECT_CHECK(KvxFtuState, (obj), TYPE_KVX_FTU)

#define KVX_FTU_MMIO_LEN 0x418

struct KvxCoolidgeClusterState;

typedef struct clusterCtrl {
    bool rm_runing;

    bool rm_wup;
    bool reset;
    bool clock_enable;
    bool scrambling;
} clusterCtrl;

typedef struct KvxFtuState {
    /*< private >*/
    SysBusDevice parent;
    struct KvxCoolidgeClusterState *clusters; /* array with the 5 clusters */

    /*< public >*/
    clusterCtrl cluster_control[5];

    uint32_t rm_reset_pc[5];

    MemoryRegion iomem;

} KvxFtuState;

#endif
