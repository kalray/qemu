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

#ifndef HW_KVX_COOLIDGE_CLUSTER_H
#define HW_KVX_COOLIDGE_CLUSTER_H

#include "hw/sysbus.h"

#include "target/kvx/cpu.h"
#include "hw/kvx/pwr-ctrl.h"
#include "hw/kvx/ipi-ctrl.h"
#include "hw/kvx/apic-mailbox.h"
#include "hw/kvx/apic-gic.h"
#include "hw/kvx/dsu-clock.h"
#include "hw/dma/kvx-dma.h"
#include "hw/kvx/boot.h"

#include "hw/kvx/machine-descr.h"

#define TYPE_KVX_CLUSTER "kvx-cluster"
#define KVX_CLUSTER(obj) \
    OBJECT_CHECK(KvxCoolidgeClusterState, (obj), TYPE_KVX_CLUSTER)

#define MPPA_CLUSTER_NUM_PE_CORES 16
#define MPPA_CLUSTER_NUM_CPUS (MPPA_CLUSTER_NUM_PE_CORES + 1)

#define MPPA_CLUSTER_LINUX_BOOT_MAGIC 0x31564752414e494cull

#define MPPA_CLUSTER_MMIO_LEN (16 * MiB)

/*
 * ClusterOS has a dedicated ELF section for the DTB to be stored into. Its
 * size is very limited.
 */
#define FDT_MAX_SIZE 0x8000

typedef struct KvxCoolidgeClusterState {
    /*< private >*/
    SysBusDevice parent;

    int fdt_size;
    uint64_t frequency;
    uint64_t initial_dsu_clock;
    bool gen_mppa_argarea;
    bool gen_dtb;

    /*< public >*/
    KVXCPU rm_core;
    KVXCPU pe_cores[MPPA_CLUSTER_NUM_PE_CORES];
    KvxIpeHelper ipe_helper;

    KvxApicMailboxState apic_mailbox;
    KvxApicGicState apic_gic;
    KvxPwrCtrlState pwr_ctrl;
    KvxDsuClockState dsu_clock;
    KvxIpiCtrlState ipi_ctrl;
    KvxDmaState dma;

    MemoryRegion smem;

    MemoryRegion cluster_region;
    MemoryRegion local_alias;
    MemoryRegion local_root_region;
    MemoryRegion system_memory_alias;

    KvxBootInfo boot_info;
} KvxCoolidgeClusterState;

enum {
    MPPA_CLUSTER_ROOT,
    MPPA_CLUSTER_BUS_AXI,
    MPPA_CLUSTER_RM_CORE,
    MPPA_CLUSTER_PE_CORES,

    /* Leave room for the 16 cores */
    MPPA_CLUSTER_SMEM = MPPA_CLUSTER_PE_CORES + MPPA_CLUSTER_NUM_PE_CORES,
    MPPA_CLUSTER_APIC_MAILBOX,
    MPPA_CLUSTER_APIC_GIC,
    MPPA_CLUSTER_PWR_CTRL,
    MPPA_CLUSTER_DSU_CLOCK, /* probably part of a bigger peripheral */
    MPPA_CLUSTER_IPI_CTRL,
    MPPA_CLUSTER_UART0,
    MPPA_CLUSTER_UART1,
    MPPA_CLUSTER_UART2,
    MPPA_CLUSTER_UART3,
    MPPA_CLUSTER_UART4,
    MPPA_CLUSTER_UART5,
    MPPA_CLUSTER_ITGEN0,
    MPPA_CLUSTER_ITGEN1,
    MPPA_CLUSTER_DMA,
    MPPA_CLUSTER_DDR,
    MPPA_CLUSTER_DDR_32BITS_ALIAS,
    MPPA_CLUSTER_L2_CTRL,
    MPPA_CLUSTER_FTU,

    MPPA_CLUSTER_CORE_INTC,
    MPPA_CLUSTER_CORE_TIMER,
    MPPA_CLUSTER_CORE_WATCHDOG,

    FIXED_CLOCK_CORE,
    FIXED_CLOCK_REF,
};

static inline CPUState *kvx_coolidge_cluster_get_cpu(KvxCoolidgeClusterState *cluster, int pid)
{
    g_assert(pid <= 16);
    return (pid == 16) ? CPU(&cluster->rm_core) : CPU(&cluster->pe_cores[pid]);
}

#endif
