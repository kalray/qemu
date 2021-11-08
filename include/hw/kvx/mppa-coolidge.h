/*
 * Kalray KVX MPPA cluster
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

#ifndef HW_KVX_MPPA_COOLIDGE_H
#define HW_KVX_MPPA_COOLIDGE_H

#include "hw/char/serial.h"
#include "hw/kvx/itgen.h"
#include "hw/kvx/ftu.h"
#include "hw/kvx/coolidge-cluster.h"


#define TYPE_MPPA_COOLIDGE_MACHINE MACHINE_TYPE_NAME("mppa-coolidge")
#define MPPA_COOLIDGE(obj) \
    OBJECT_CHECK(MppaCoolidgeMachineState, (obj), TYPE_MPPA_COOLIDGE_MACHINE)

#define MPPA_COOLIDGE_NUM_CLUSTER 5

#define MPPA_COOLIDGE_NUM_CPU (MPPA_CLUSTER_NUM_CPUS * MPPA_COOLIDGE_NUM_CLUSTER)

extern const PeriphEntry mppa_cluster_periphs[];

typedef struct MppaCoolidgeMachineState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    bool gen_mppa_argarea;
    bool gen_dtb;
    uint64_t frequency;
    uint64_t initial_dsu_clock;

    KvxCoolidgeClusterState cluster[MPPA_COOLIDGE_NUM_CLUSTER];

    SerialMM uart[6];
    KvxItgenState itgen0;
    KvxItgenState itgen1;
    KvxFtuState ftu;

    MemoryRegion ddr;
    MemoryRegion ddr_32bits_alias;

} MppaCoolidgeMachineState;

#endif