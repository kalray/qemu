/*
 * Kalray KVX MPPA cluster
 *
 * Copyright (c) 2019-2020 GreenSocs SAS
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
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "exec/address-spaces.h"
#include "target/kvx/cpu.h"
#include "hw/qdev-properties.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "hw/char/serial.h"

#include "hw/kvx/boot.h"
#include "hw/kvx/pwr-ctrl.h"
#include "hw/kvx/ipi-ctrl.h"
#include "hw/kvx/apic-gic.h"
#include "hw/kvx/apic-mailbox.h"
#include "hw/kvx/itgen.h"
#include "hw/kvx/dsu-clock.h"

#define TYPE_MPPA_CLUSTER_MACHINE MACHINE_TYPE_NAME("mppa-cluster")
#define MPPA_CLUSTER(obj) \
    OBJECT_CHECK(MppaClusterMachineState, (obj), TYPE_MPPA_CLUSTER_MACHINE)

#define MPPA_CLUSTER_NUM_PE_CORES 16
#define MPPA_CLUSTER_NUM_CPUS   (MPPA_CLUSTER_NUM_PE_CORES + 1)

#define MPPA_CLUSTER_SMEM_SIZE  (4 * MiB)
#define MPPA_CLUSTER_DDR_32BITS_ALIAS_SIZE (2 * GiB)

#define MPPA_CLUSTER_LINUX_BOOT_MAGIC 0x31564752414e494cull
#define MPPA_CLUSTER_DTB_LOAD_ADDR (0x101000000ull)

typedef struct MppaClusterMachineState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    KVXCPU rm_core;
    KVXCPU pe_cores[MPPA_CLUSTER_NUM_PE_CORES];
    KvxIpeHelper ipe_helper;

    KvxApicMailboxState apic_mailbox;
    KvxApicGicState apic_gic;
    KvxPwrCtrlState pwr_ctrl;
    KvxDsuClockState dsu_clock;
    KvxIpiCtrlState ipi_ctrl;
    SerialMM uart[6];
    KvxItgenState itgen0;
    KvxItgenState itgen1;

    MemoryRegion cluster_region;
    MemoryRegion cluster_alias;

    MemoryRegion smem;
    MemoryRegion ddr;
    MemoryRegion ddr_32bits_alias;

    KvxBootInfo boot_info;
} MppaClusterMachineState;

enum {
    MPPA_CLUSTER_SMEM,
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
    MPPA_CLUSTER_DDR,
    MPPA_CLUSTER_DDR_32BITS_ALIAS,
};

typedef struct MemmapEntry MemmapEntry;

struct MemmapEntry {
    hwaddr base;
    hwaddr size;
};

static const uint64_t mppa_cluster_memmap[] = {
    [MPPA_CLUSTER_SMEM]             = 0x0,
    [MPPA_CLUSTER_APIC_MAILBOX]     = 0xa00000,
    [MPPA_CLUSTER_APIC_GIC]         = 0xa20000,
    [MPPA_CLUSTER_PWR_CTRL]         = 0xa40000,
    [MPPA_CLUSTER_DSU_CLOCK]        = 0xa44180,
    [MPPA_CLUSTER_IPI_CTRL]         = 0xad0000,
    [MPPA_CLUSTER_UART0]            = 0x20210000,
    [MPPA_CLUSTER_UART1]            = 0x20211000,
    [MPPA_CLUSTER_UART2]            = 0x20212000,
    [MPPA_CLUSTER_UART3]            = 0x20213000,
    [MPPA_CLUSTER_UART4]            = 0x20214000,
    [MPPA_CLUSTER_UART5]            = 0x20215000,
    [MPPA_CLUSTER_ITGEN0]           = 0x27000000,
    [MPPA_CLUSTER_ITGEN1]           = 0x27010000,
    [MPPA_CLUSTER_DDR_32BITS_ALIAS] = 0x40000000,
    [MPPA_CLUSTER_DDR]              = 0x100000000,
};

static const int mppa_cluster_irqmap[] = {
    [MPPA_CLUSTER_UART0]            = 41,
    [MPPA_CLUSTER_UART1]            = 42,
    [MPPA_CLUSTER_UART2]            = 43,
    [MPPA_CLUSTER_UART3]            = 44,
    [MPPA_CLUSTER_UART4]            = 45,
    [MPPA_CLUSTER_UART5]            = 46,
};

static void mppa_cluster_rm_reset(void *opaque)
{
    CPUState *cpu = CPU(opaque);
    MppaClusterMachineState *s = MPPA_CLUSTER(qdev_get_machine());

    cpu_reset(cpu);

    if (s->boot_info.kernel_loaded) {
        cpu_set_pc(cpu, s->boot_info.kernel_entry);
    }

    if (s->boot_info.dtb_loaded) {
        CPUKVXState *env = &KVX_CPU(cpu)->env;
        kvx_register_write_u64(env, REG_kv3_R0, MPPA_CLUSTER_LINUX_BOOT_MAGIC);
        kvx_register_write_u64(env, REG_kv3_R1, MPPA_CLUSTER_DTB_LOAD_ADDR);
    }
}

static void mppa_cluster_pe_reset(void *opaque)
{
    CPUState *cpu = CPU(opaque);
    cpu_reset(cpu);

    /* PEs start in resetting state */
    cpu->halted = true;
    KVX_CPU(cpu)->env.sleep_state = KVX_RESETTING;
}

static inline void core_init(MppaClusterMachineState *s, KVXCPU *core,
                             bool is_rm)
{
    const char *name = is_rm ? "rm" : "pe[*]";
    QEMUResetHandler *core_reset_handler = is_rm
        ? mppa_cluster_rm_reset
        : mppa_cluster_pe_reset;

    object_initialize_child(OBJECT(s), name, core, MACHINE(s)->cpu_type);

    if (!is_rm) {
        object_property_set_link(OBJECT(core), "ipe-helper",
                                 OBJECT(&s->ipe_helper), &error_abort);
    }

    qemu_register_reset(core_reset_handler, core);
}

static inline void core_realize(KVXCPU *core, int pid)
{
    Object *obj = OBJECT(core);

    object_property_set_uint(obj, "pid", pid, &error_abort);
    object_property_set_bool(obj, "realized", true, &error_abort);
}

static inline void connect_pe_ipe(KVXCPU *cpus, int pid)
{
    DeviceState *prev, *cur, *next;
    int idx;

    cur = DEVICE(&cpus[pid]);
    prev = DEVICE(&cpus[pid ? pid - 1 : 15]);
    next = DEVICE(&cpus[(pid + 1) % 16]);

    for (idx = 0; idx < KVX_NUM_IPE; idx++) {
        qemu_irq fwd_in = qdev_get_gpio_in_named(next, "ipe-fwd-in", idx);
        qemu_irq bwd_in = qdev_get_gpio_in_named(prev, "ipe-bwd-in", idx);

        qdev_connect_gpio_out_named(cur, "ipe-fwd-out", idx, fwd_in);
        qdev_connect_gpio_out_named(cur, "ipe-bwd-out", idx, bwd_in);
    }
}

static inline void connect_apic_gic_to_core(DeviceState *gic,
                                            DeviceState *core, int pid)
{
    int i;

    /*
     * APIC GIC output line (pid * 4 + i) connects
     * to core's IRQ number 4 + i
     */
    for (i = 0; i < 4; i++) {
        qemu_irq in;
        int out_idx = pid * 4 + i;

        in = qdev_get_gpio_in(core, 4 + i);
        qdev_connect_gpio_out(gic, out_idx, in);
    }
}

static inline void connect_apic_gic_irqs(MppaClusterMachineState *s)
{
    size_t i;
    DeviceState *gic = DEVICE(&s->apic_gic);

    /* PE cores */
    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        connect_apic_gic_to_core(gic, DEVICE(&s->pe_cores[i]), i);
    }

    /* RM core */
    connect_apic_gic_to_core(gic, DEVICE(&s->rm_core), 16);
}

static inline void connect_apic_mailbox_irqs(MppaClusterMachineState *s)
{
    size_t i;
    DeviceState *mb = DEVICE(&s->apic_mailbox);
    DeviceState *gic = DEVICE(&s->apic_gic);

    for (i = 0; i < KVX_APIC_MAILBOX_NUM_IRQ_OUT; i++) {
        qemu_irq in = qdev_get_gpio_in(gic, i);
        qdev_connect_gpio_out(mb, i, in);
    }
}

static inline void devices_init(MppaClusterMachineState *s)
{
    int i;
    uint64_t ram_size = MACHINE(s)->ram_size;
    MemoryRegion *system_memory = get_system_memory();

    /* IPE helper */
    object_initialize_child(OBJECT(s), "ipe-helper", &s->ipe_helper,
                            TYPE_KVX_IPE_HELPER);

    /* KVX cores */
    core_init(s, &s->rm_core, true);

    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        core_init(s, &s->pe_cores[i], false);
    }

    /* Cluster memory region */
    memory_region_init(&s->cluster_region, NULL, "mppa-cluster", 16 * MiB);
    memory_region_init_alias(&s->cluster_alias, NULL, "mppa-cluster.alias0",
                             &s->cluster_region, 0, 16 * MiB);
    memory_region_add_subregion(system_memory, 0, &s->cluster_region);
    memory_region_add_subregion(system_memory, 16 * MiB, &s->cluster_alias);

    /* APIC Mailbox */
    object_initialize_child(OBJECT(s), "apic-mailbox", &s->apic_mailbox,
                            TYPE_KVX_APIC_MAILBOX);
    memory_region_add_subregion(&s->cluster_region,
                                mppa_cluster_memmap[MPPA_CLUSTER_APIC_MAILBOX],
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->apic_mailbox), 0));

    /* APIC GIC */
    object_initialize_child(OBJECT(s), "apic-gic", &s->apic_gic,
                            TYPE_KVX_APIC_GIC);
    memory_region_add_subregion(&s->cluster_region,
                                mppa_cluster_memmap[MPPA_CLUSTER_APIC_GIC],
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->apic_gic), 0));

    /* Power controller */
    object_initialize_child(OBJECT(s), "pwr-ctrl", &s->pwr_ctrl,
                            TYPE_KVX_PWR_CTRL);
    memory_region_add_subregion(&s->cluster_region,
                                mppa_cluster_memmap[MPPA_CLUSTER_PWR_CTRL],
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->pwr_ctrl), 0));

    /* DSU clock */
    object_initialize_child(OBJECT(s), "dsu-clock", &s->dsu_clock,
                            TYPE_KVX_DSU_CLOCK);
    memory_region_add_subregion(&s->cluster_region,
                                mppa_cluster_memmap[MPPA_CLUSTER_DSU_CLOCK],
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dsu_clock), 0));

    /* IPI controller */
    object_initialize_child(OBJECT(s), "ipi-ctrl", &s->ipi_ctrl,
                            TYPE_KVX_IPI_CTRL);
    memory_region_add_subregion(&s->cluster_region,
                                mppa_cluster_memmap[MPPA_CLUSTER_IPI_CTRL],
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->ipi_ctrl), 0));

    /* UARTs */
    for (i = 0; i < ARRAY_SIZE(s->uart); i++) {
        object_initialize_child(OBJECT(s), "uart[*]", &s->uart[i],
                                TYPE_SERIAL_MM);
    }

    /* itgen IRQ controllers */
    object_initialize_child(OBJECT(s), "itgen0", &s->itgen0, TYPE_KVX_ITGEN);
    memory_region_add_subregion(get_system_memory(),
                                mppa_cluster_memmap[MPPA_CLUSTER_ITGEN0],
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->itgen0), 0));

    object_initialize_child(OBJECT(s), "itgen1", &s->itgen1, TYPE_KVX_ITGEN);
    memory_region_add_subregion(get_system_memory(),
                                mppa_cluster_memmap[MPPA_CLUSTER_ITGEN1],
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->itgen1), 0));

    /* SMEM */
    memory_region_init_ram(&s->smem, NULL, "mppa-cluster.smem",
                           MPPA_CLUSTER_SMEM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(&s->cluster_region,
                                mppa_cluster_memmap[MPPA_CLUSTER_SMEM],
                                &s->smem);

    /* DDR */
    memory_region_init_ram(&s->ddr, NULL, "mppa-cluster.ddr",
                           ram_size,
                           &error_fatal);
    memory_region_add_subregion(system_memory,
                                mppa_cluster_memmap[MPPA_CLUSTER_DDR],
                                &s->ddr);

    memory_region_init_alias(&s->ddr_32bits_alias, NULL,
                             "mppa-cluster.ddr-32bits-alias",
                             &s->ddr, 0,
                             MPPA_CLUSTER_DDR_32BITS_ALIAS_SIZE);
    memory_region_add_subregion(system_memory,
                                mppa_cluster_memmap[MPPA_CLUSTER_DDR_32BITS_ALIAS],
                                &s->ddr_32bits_alias);
}

static inline void devices_realize(MppaClusterMachineState *s)
{
    int i;

    /* KVX cores */
    core_realize(&s->rm_core, 16);

    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        core_realize(&s->pe_cores[i], i);
    }

    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        connect_pe_ipe(s->pe_cores, i);
    }

    /* APIC Mailbox */
    sysbus_realize(SYS_BUS_DEVICE(&s->apic_mailbox), &error_abort);

    connect_apic_mailbox_irqs(s);

    /* APIC GIC */
    sysbus_realize(SYS_BUS_DEVICE(&s->apic_gic), &error_abort);

    connect_apic_gic_irqs(s);

    /* Power controller */
    sysbus_realize(SYS_BUS_DEVICE(&s->pwr_ctrl), &error_abort);

    /* DSU clock */
    sysbus_realize(SYS_BUS_DEVICE(&s->dsu_clock), &error_abort);

    /* IPI controller */
    sysbus_realize(SYS_BUS_DEVICE(&s->ipi_ctrl), &error_abort);

    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        qemu_irq ipi = qdev_get_gpio_in(DEVICE(&s->pe_cores[i]), KVX_IRQ_IPI);
        qdev_connect_gpio_out(DEVICE(&s->ipi_ctrl), i, ipi);
    }

    /* UARTs */
    for (i = 0; i < ARRAY_SIZE(s->uart); i++) {
        uint64_t addr = mppa_cluster_memmap[MPPA_CLUSTER_UART0 + i];
        int irq_num = mppa_cluster_irqmap[MPPA_CLUSTER_UART0 + i];
        DeviceState *dev = DEVICE(&s->uart[i]);
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->uart[i]);

        qdev_prop_set_uint8(dev, "regshift", 2);
        qdev_prop_set_uint32(dev, "baudbase", 100000000);
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_abort);

        memory_region_add_subregion(get_system_memory(), addr,
                                    sysbus_mmio_get_region(sbd, 0));

        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(DEVICE(&s->itgen0), irq_num));
    }

    /* itgen IRQ controllers */
    sysbus_realize(SYS_BUS_DEVICE(&s->itgen0), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->itgen1), &error_abort);
}

static void boot_cluster(MppaClusterMachineState *s)
{
    kvx_boot_fill_info_from_machine(&s->boot_info, MACHINE(s));

    s->boot_info.ddr_base = mppa_cluster_memmap[MPPA_CLUSTER_DDR];
    s->boot_info.ddr_size = MACHINE(s)->ram_size;
    s->boot_info.dtb_load_addr = MPPA_CLUSTER_DTB_LOAD_ADDR;

    kvx_boot(&s->boot_info);
}

static void mppa_cluster_init(MachineState *machine)
{
    MppaClusterMachineState *s = MPPA_CLUSTER(machine);

    devices_init(s);
    devices_realize(s);
    boot_cluster(s);
}

static void mppa_cluster_machine_class_init(ObjectClass *klass, void *data)
{
    MachineClass *mc = MACHINE_CLASS(klass);

    mc->desc = "Kalray MPPA cluster";
    mc->init = mppa_cluster_init;
    mc->max_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->default_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->max_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->min_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->default_cpu_type = TYPE_KVX_CPU_KV3;
    mc->is_default = true;
}

static const TypeInfo mppa_cluster_machine_type_info[] = {
    {
        .name = TYPE_MPPA_CLUSTER_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(MppaClusterMachineState),
        .class_init = mppa_cluster_machine_class_init,
    },
};

DEFINE_TYPES(mppa_cluster_machine_type_info)
