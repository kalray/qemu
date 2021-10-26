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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "hw/qdev-properties.h"
#include "sysemu/reset.h"
#include "sysemu/device_tree.h"
#include "hw/boards.h"

#include "hw/kvx/coolidge-cluster.h"
#include "hw/kvx/mppa-cluster.h"

static uint64_t get_core_clk_freq(MachineState *m)
{
    return MPPA_CLUSTER(m)->frequency;
}

static uint64_t get_ddr_size(MachineState *m)
{
    return m->ram_size;
}

static void fdt_node_uart(MachineState *m, void *fdt, const char *node,
                          size_t id, int idx)
{
    qemu_fdt_setprop_cell(fdt, node, "reg-io-width", 4);
    qemu_fdt_setprop_cell(fdt, node, "reg-shift", 2);
}

static void fdt_node_cpu(MachineState *m, void *fdt, const char *node,
                         size_t id, int idx)
{
    if (idx == 0) {
        /* Add clock and power ctrl information on CPU 0 */
        qemu_fdt_setprop_cell(fdt, node, "clocks",
                              periph_fdt_get_phandle(FIXED_CLOCK_CORE));
        qemu_fdt_setprop_cell(fdt, node, "power-controller",
                              periph_fdt_get_phandle(MPPA_CLUSTER_PWR_CTRL));
    }
}

static void fdt_node_l2_cache(MachineState *m, void *fdt, const char *node,
                              size_t id, int idx)
{
    qemu_fdt_setprop(fdt, node, "kalray,is-qemu", NULL, 0);
    qemu_fdt_setprop_string(fdt, node, "status", "disabled");
}

const PeriphEntry mppa_cluster_periphs[] = {
    [MPPA_CLUSTER_ROOT] = {
        .fdt = {
            .node = "/",
            .compatible = "kalray,coolidge",
        },

        .bus = {
            .valid = true,
            .fdt_size_cells = 2,
            .fdt_address_cells = 2,
        },
    },

    [MPPA_CLUSTER_BUS_AXI] = {
        .fdt = {
            .node = "/axi",
            .compatible = "simple-bus",
        },

        .bus = {
            .valid = true,
            .fdt_size_cells = 2,
            .fdt_address_cells = 2,
        },
    },

    [FIXED_CLOCK_CORE] = {
        .fdt = {
            .node = "/clocks/core_clk",
            .compatible = "fixed-clock",
        },

        .fixed_clock = {
            .valid = true,
            .freq_cb = get_core_clk_freq,
        },
    },

    [FIXED_CLOCK_REF] = {
        .fdt = {
            .node = "/clocks/ref_clk",
            .compatible = "fixed-clock",
        },

        .fixed_clock = {
            .valid = true,
            .freq = 100000000,
        },
    },

    [MPPA_CLUSTER_PE_CORES] = {
        .count = 16,

        .fdt = {
            .node = "/cpus/cpu",
            .compatible = "kalray,kvx-pe",
            .cb = fdt_node_cpu,
        },
    },

    [MPPA_CLUSTER_SMEM] = {
        .fdt = {
            .node = "/memory",
            .compatible = "memory",
        },

        .mmio_map = {
            .valid = true,
            .base = 0x0,
            .size = 4 * MiB,
        },
    },

    [MPPA_CLUSTER_DDR] = {
        .fdt = {
            .node = "/memory",
            .compatible = "memory",
        },

        .mmio_map = {
            .valid = true,
            .base = 0x100000000ull,
            .get_size_cb = get_ddr_size,
        },
    },

    [MPPA_CLUSTER_DDR_32BITS_ALIAS] = {
        .mmio_map = {
            .valid = true,
            .base = 0x40000000,
            .size = 2 * GiB,
        },
    },

    [MPPA_CLUSTER_UART0] = {
        .fdt = {
            .node = "/axi/uart0",
            .compatible = "snps,dw-apb-uart",
            .cb = fdt_node_uart,
        },

        .mmio_map = {
            .valid = true,
            .base = 0x20210000,
            .size = 0x100,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_ITGEN0,
            .mapping = IRQS( IRQ_FLAG(41, 0x4) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_REF),
        },
    },

    [MPPA_CLUSTER_UART1] = {
        .fdt = {
            .node = "/axi/uart1",
            .compatible = "snps,dw-apb-uart",
            .cb = fdt_node_uart,
        },

        .mmio_map = {
            .valid = true,
            .base = 0x20211000,
            .size = 0x100,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_ITGEN0,
            .mapping = IRQS( IRQ_FLAG(42, 0x4) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_REF),
        },
    },

    [MPPA_CLUSTER_UART2] = {
        .fdt = {
            .node = "/axi/uart2",
            .compatible = "snps,dw-apb-uart",
            .cb = fdt_node_uart,
        },

        .mmio_map = {
            .valid = true,
            .base = 0x20212000,
            .size = 0x100,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_ITGEN0,
            .mapping = IRQS( IRQ_FLAG(43, 0x4) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_REF),
        },
    },

    [MPPA_CLUSTER_UART3] = {
        .fdt = {
            .node = "/axi/uart3",
            .compatible = "snps,dw-apb-uart",
            .cb = fdt_node_uart,
        },

        .mmio_map = {
            .valid = true,
            .base = 0x20213000,
            .size = 0x100,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_ITGEN0,
            .mapping = IRQS( IRQ_FLAG(44, 0x4) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_REF),
        },
    },

    [MPPA_CLUSTER_UART4] = {
        .fdt = {
            .node = "/axi/uart4",
            .compatible = "snps,dw-apb-uart",
            .cb = fdt_node_uart,
        },

        .mmio_map = {
            .valid = true,
            .base = 0x20214000,
            .size = 0x100,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_ITGEN0,
            .mapping = IRQS( IRQ_FLAG(45, 0x4) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_REF),
        },
    },

    [MPPA_CLUSTER_UART5] = {
        .fdt = {
            .node = "/axi/uart5",
            .compatible = "snps,dw-apb-uart",
            .cb = fdt_node_uart,
        },

        .mmio_map = {
            .valid = true,
            .base = 0x20215000,
            .size = 0x100,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_ITGEN0,
            .mapping = IRQS( IRQ_FLAG(46, 0x4) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_REF),
        },
    },

    [MPPA_CLUSTER_ITGEN0] = {
        .fdt = {
            .node = "/axi/itgen_soc_periph0",
            .compatible = "kalray,kvx-itgen",
        },

        .mmio_map = {
            .valid = true,
            .base = 0x27000000,
            .size = 0x1104,
        },

        .irq_ctrl = {
            .valid = true,
            .fdt_interrupt_cells = 2,
        },

        .msi_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_APIC_MAILBOX,
        },
    },

    [MPPA_CLUSTER_ITGEN1] = {
        .fdt = {
            .node = "/axi/itgen_soc_periph1",
            .compatible = "kalray,kvx-itgen",
        },

        .mmio_map = {
            .valid = true,
            .base = 0x27010000,
            .size = 0x1104,
        },

        .irq_ctrl = {
            .valid = true,
            .fdt_interrupt_cells = 2,
        },

        .msi_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_APIC_MAILBOX,
        },
    },

    [MPPA_CLUSTER_DMA] = {
        .mmio_map = {
            .valid = true,
            .base = 0x800000,
            .size = 0x100000,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_APIC_GIC,
            .mapping = IRQS( IRQ(129) ),
        },
    },

    [MPPA_CLUSTER_APIC_MAILBOX] = {
        .fdt = {
            .node = "/apic_mailbox",
            .compatible = "kalray,kvx-apic-mailbox",
        },

        .mmio_map = {
            .valid = true,
            .base = 0xa00000,
            .size = 0x10000,
        },

        .msi_ctrl = {
            .valid = true,
        },

        .irq_ctrl = {
            .valid = true,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_APIC_GIC,
            .mapping = IRQS( IRQ(0), IRQ(1), IRQ(2), IRQ(3), IRQ(4), IRQ(5),
                             IRQ(6), IRQ(7), IRQ(8), IRQ(9), IRQ(10), IRQ(11),
                             IRQ(12), IRQ(13), IRQ(14), IRQ(15), IRQ(16), IRQ(17),
                             IRQ(18), IRQ(19), IRQ(20), IRQ(21), IRQ(22), IRQ(23),
                             IRQ(24), IRQ(25), IRQ(26), IRQ(27), IRQ(28), IRQ(29),
                             IRQ(30), IRQ(31), IRQ(32), IRQ(33), IRQ(34), IRQ(35),
                             IRQ(36), IRQ(37), IRQ(38), IRQ(39), IRQ(40), IRQ(41),
                             IRQ(42), IRQ(43), IRQ(44), IRQ(45), IRQ(46), IRQ(47),
                             IRQ(48), IRQ(49), IRQ(50), IRQ(51), IRQ(52), IRQ(53),
                             IRQ(54), IRQ(55), IRQ(56), IRQ(57), IRQ(58), IRQ(59),
                             IRQ(60), IRQ(61), IRQ(62), IRQ(63), IRQ(64), IRQ(65),
                             IRQ(66), IRQ(67), IRQ(68), IRQ(69), IRQ(70), IRQ(71),
                             IRQ(72), IRQ(73), IRQ(74), IRQ(75), IRQ(76), IRQ(77),
                             IRQ(78), IRQ(79), IRQ(80), IRQ(81), IRQ(82), IRQ(83),
                             IRQ(84), IRQ(85), IRQ(86), IRQ(87), IRQ(88), IRQ(89),
                             IRQ(90), IRQ(91), IRQ(92), IRQ(93), IRQ(94), IRQ(95),
                             IRQ(96), IRQ(97), IRQ(98), IRQ(99), IRQ(100), IRQ(101),
                             IRQ(102), IRQ(103), IRQ(104), IRQ(105), IRQ(106), IRQ(107),
                             IRQ(108), IRQ(109), IRQ(110), IRQ(111), IRQ(112), IRQ(113),
                             IRQ(114), IRQ(115), IRQ(116), IRQ(117), IRQ(118), IRQ(119),
                             IRQ(120) ),
        },
    },

    [MPPA_CLUSTER_APIC_GIC] = {
        .fdt = {
            .node = "/apic_gic",
            .compatible = "kalray,kvx-apic-gic",
        },

        .mmio_map = {
            .valid = true,
            .base = 0xa20000,
            .size = 0x12000,
        },

        .irq_ctrl = {
            .valid = true,
            .fdt_interrupt_cells = 1,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_CORE_INTC,
            .mapping = IRQS( IRQ(4), IRQ(5), IRQ(6), IRQ(7) ),
        },
    },

    [MPPA_CLUSTER_PWR_CTRL] = {
        .fdt = {
            .node = "/pwr_ctrl",
            .compatible = "kalray,kvx-pwr-ctrl",
        },

        .mmio_map = {
            .valid = true,
            .base = 0xa40000,
            .size = 0x4188,
        },
    },

    [MPPA_CLUSTER_FTU] = {
        .fdt = {
            .node = "/ftu",
            .compatible = "kalray,kvx-syscon"
        },

        .mmio_map = {
            .valid = true,
            .base = 0x10181000,
            .size = 0x418,
        },
    },

    [MPPA_CLUSTER_DSU_CLOCK] = {
        .fdt = {
            .node = "/dsu_clock",
            .compatible = "kalray,kvx-dsu-clock",
        },

        .mmio_map = {
            .valid = true,
            .base = 0xa44180,
            .size = 0x8,
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS( FIXED_CLOCK_CORE ),
        },
    },

    [MPPA_CLUSTER_IPI_CTRL] = {
        .fdt = {
            .node = "/ipi_ctrl",
            .compatible = "kalray,kvx-ipi-ctrl",
        },

        .mmio_map = {
            .valid = true,
            .base = 0xad0000,
            .size = 0x1000,
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_CORE_INTC,
            .mapping = IRQS( IRQ(24) ),
        },
    },

    [MPPA_CLUSTER_L2_CTRL] = {
        .fdt = {
            .node = "/l2_cache",
            .compatible = "kalray,kvx-l2-cache",
            .cb = fdt_node_l2_cache,
        },
    },

    [MPPA_CLUSTER_CORE_INTC] = {
        .fdt = {
            .node = "/core_intc",
            .compatible = "kalray,kvx-core-intc",
        },

        .irq_ctrl = {
            .valid = true,
            .fdt_interrupt_cells = 1,
        },
    },

    [MPPA_CLUSTER_CORE_TIMER] = {
        .fdt = {
            .node = "/core_timer",
            .compatible = "kalray,kvx-core-timer",
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_CORE_INTC,
            .mapping = IRQS( IRQ(0) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_CORE),
        },
    },

    [MPPA_CLUSTER_CORE_WATCHDOG] = {
        .fdt = {
            .node = "/core_watchdog",
            .compatible = "kalray,kvx-core-watchdog",
        },

        .irq_map = {
            .valid = true,
            .parent = MPPA_CLUSTER_CORE_INTC,
            .mapping = IRQS( IRQ(2) ),
        },

        .clock_map = {
            .valid = true,
            .clocks = CLOCKS(FIXED_CLOCK_CORE),
        },
    },
};

static inline void core_realize(KvxCoolidgeClusterState *s,
                                KVXCPU *core, int pid)
{
    Object *obj = OBJECT(core);

    if (pid < 16) {
        object_property_set_link(OBJECT(core), "ipe-helper",
                                 OBJECT(&s->ipe_helper), &error_abort);
    }

    object_property_set_link(OBJECT(core), "memory",
                             OBJECT(&s->local_root_region), &error_abort);

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

static inline void connect_apic_gic_to_core(DeviceState *gic, DeviceState *core,
                                            int pid)
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

static inline void connect_apic_gic_irqs(KvxCoolidgeClusterState *s)
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

static inline void connect_apic_mailbox_irqs(KvxCoolidgeClusterState *s)
{
    size_t i;
    DeviceState *mb = DEVICE(&s->apic_mailbox);
    DeviceState *gic = DEVICE(&s->apic_gic);

    for (i = 0; i < KVX_APIC_MAILBOX_NUM_IRQ_OUT; i++) {
        qemu_irq in = qdev_get_gpio_in(gic, i);
        qdev_connect_gpio_out(mb, i, in);
    }
}


static void mppa_cluster_rm_reset(void *opaque)
{
    KvxCoolidgeClusterState *s = KVX_CLUSTER(opaque);
    CPUState *cpu = CPU(&s->rm_core);
    CPUKVXState *env = &KVX_CPU(cpu)->env;

    cpu_reset(cpu);

    if (s->boot_info.kernel_loaded) {
        cpu_set_pc(cpu, s->boot_info.kernel_entry);
        kvx_register_write_field(env, PS, V64, 1);
    }

    if (s->boot_info.fdt) {
        kvx_register_write_u64(env, REG_kv3_R0, MPPA_CLUSTER_LINUX_BOOT_MAGIC);
        kvx_register_write_u64(env, REG_kv3_R1, s->boot_info.dtb_load_addr);
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

static inline void core_init(KvxCoolidgeClusterState *s, KVXCPU *core,
                             bool is_rm)
{
    const char *name = is_rm ? "rm" : "pe[*]";
    QEMUResetHandler *core_reset_handler =
        is_rm ? mppa_cluster_rm_reset : mppa_cluster_pe_reset;

    object_initialize_child(OBJECT(s), name, core, TYPE_KVX_CPU_KV3_V1);

    if (is_rm) {
        qemu_register_reset(core_reset_handler, s);
    } else {
        qemu_register_reset(core_reset_handler, core);
    }
}

static void create_fdt(KvxCoolidgeClusterState *s)
{
    void *fdt;
    MachineState *m = MACHINE(qdev_get_machine());

    if (m->dtb) {
        /* A DTB file has been provided by the user */
        int size;

        m->fdt = load_device_tree(m->dtb, &size);

        if (!m->fdt) {
            error_report("could not open dtb file %s\n", m->dtb);
            exit(1);
        }

        s->fdt_size = size;
        return;
    }

    if (!s->gen_dtb) {
        return;
    }

    m->fdt = fdt = create_device_tree_with_size(FDT_MAX_SIZE);
    s->fdt_size = FDT_MAX_SIZE;

    machine_populate_fdt(m, mppa_cluster_periphs,
                         ARRAY_SIZE(mppa_cluster_periphs));

    qemu_fdt_setprop_string(fdt, "/", "model",
                            "Kalray Coolidge processor (QEMU)");

    const char *const compatible[] = { "kalray,coolidge", "kalray,iss" };
    qemu_fdt_setprop_string_array(fdt, "/", "compatible", (char **)compatible,
                                  ARRAY_SIZE(compatible));

    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path",
                            "/axi/uart0@20210000");
}

static void boot_cluster(KvxCoolidgeClusterState *cluster)
{
    MachineState *s = MACHINE(qdev_get_machine());
    cluster->boot_info.kernel_filename = MACHINE(s)->kernel_filename;
    cluster->boot_info.kernel_cmdline = MACHINE(s)->kernel_cmdline;
    cluster->boot_info.fdt = MACHINE(s)->fdt;
    cluster->boot_info.fdt_size = cluster->fdt_size;
    cluster->boot_info.ddr_base =
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_DDR);
    cluster->boot_info.ddr_size = MACHINE(s)->ram_size;
    cluster->boot_info.gen_mppa_argarea = cluster->gen_mppa_argarea;
    cluster->boot_info.frequency = cluster->frequency;
    cluster->boot_info.as = cpu_get_address_space(CPU(&cluster->rm_core), 0);

    kvx_boot(&cluster->boot_info);
}

static void coolidge_cluster_realize(DeviceState *dev, Error **errp)
{
    int i;

    KvxCoolidgeClusterState *s = KVX_CLUSTER(dev);

    /* KVX cores */
    core_realize(s, &s->rm_core, 16);

    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        core_realize(s, &s->pe_cores[i], i);
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
    qdev_prop_set_uint64(DEVICE(&s->dsu_clock), "initial-value",
                         s->initial_dsu_clock);
    sysbus_realize(SYS_BUS_DEVICE(&s->dsu_clock), &error_abort);

    /* DMA */
    object_property_set_link(OBJECT(&s->dma), "root-mr",
                             OBJECT(&s->local_root_region), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->dma), &error_abort);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->dma), 0,
                       qdev_get_gpio_in(DEVICE(&s->apic_gic),
                                        periph_irq_mapping_idx(mppa_cluster_periphs,
                                                               MPPA_CLUSTER_DMA, 0)));


    /* IPI controller */
    sysbus_realize(SYS_BUS_DEVICE(&s->ipi_ctrl), &error_abort);

    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        qemu_irq ipi = qdev_get_gpio_in(DEVICE(&s->pe_cores[i]), KVX_IRQ_IPI);
        qdev_connect_gpio_out(DEVICE(&s->ipi_ctrl), i, ipi);
    }

    create_fdt(s);
    boot_cluster(s);
}

static void coolidge_cluster_instance_init(Object *obj)
{
    int i;

    KvxCoolidgeClusterState *s = KVX_CLUSTER(obj);

    /* Cluster memory region */
    memory_region_init(&s->cluster_region, obj, "cluster",
                       MPPA_CLUSTER_MMIO_LEN);
    memory_region_init_alias(&s->local_alias, obj, "cluster.local-alias",
                             &s->cluster_region, 0, MPPA_CLUSTER_MMIO_LEN);

    memory_region_init(&s->local_root_region, NULL, "local-root-region",
                       UINT64_MAX);

    memory_region_init_alias(&s->system_memory_alias, obj,
                             "cluster.system_memory_alias",
                             get_system_memory(), 0, UINT64_MAX);

    memory_region_add_subregion_overlap(&s->local_root_region, 0,
                                        &s->system_memory_alias, -1);

    memory_region_add_subregion(&s->local_root_region, 0, &s->local_alias);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->cluster_region);

    /* IPE helper */
    object_initialize_child(OBJECT(s), "ipe-helper", &s->ipe_helper,
                            TYPE_KVX_IPE_HELPER);

    /* KVX cores */
    core_init(s, &s->rm_core, true);

    for (i = 0; i < MPPA_CLUSTER_NUM_PE_CORES; i++) {
        core_init(s, &s->pe_cores[i], false);
    }

    /* APIC Mailbox */
    object_initialize_child(OBJECT(s), "apic-mailbox", &s->apic_mailbox,
                            TYPE_KVX_APIC_MAILBOX);
    memory_region_add_subregion(
        &s->cluster_region,
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_APIC_MAILBOX),
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->apic_mailbox), 0));

    /* APIC GIC */
    object_initialize_child(OBJECT(s), "apic-gic", &s->apic_gic,
                            TYPE_KVX_APIC_GIC);
    memory_region_add_subregion(
        &s->cluster_region,
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_APIC_GIC),
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->apic_gic), 0));

    /* Power controller */
    object_initialize_child(OBJECT(s), "pwr-ctrl", &s->pwr_ctrl,
                            TYPE_KVX_PWR_CTRL);
    memory_region_add_subregion(
        &s->cluster_region,
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_PWR_CTRL),
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->pwr_ctrl), 0));

    /* DSU clock */
    object_initialize_child(OBJECT(s), "dsu-clock", &s->dsu_clock,
                            TYPE_KVX_DSU_CLOCK);
    memory_region_add_subregion(
        &s->cluster_region,
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_DSU_CLOCK),
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dsu_clock), 0));

    /* IPI controller */
    object_initialize_child(OBJECT(s), "ipi-ctrl", &s->ipi_ctrl,
                            TYPE_KVX_IPI_CTRL);
    memory_region_add_subregion(
        &s->cluster_region,
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_IPI_CTRL),
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->ipi_ctrl), 0));

    /* DMA */
    object_initialize_child(OBJECT(s), "dma", &s->dma, TYPE_KVX_DMA);
    memory_region_add_subregion(
        &s->cluster_region,
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_DMA),
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dma), 0));

    /* SMEM */
    memory_region_init_ram(
        &s->smem, NULL, "mppa-cluster.smem",
        periph_mmio_size(mppa_cluster_periphs, MPPA_CLUSTER_SMEM),
        &error_fatal);
    memory_region_add_subregion(
        &s->cluster_region,
        periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_SMEM), &s->smem);
}

static void coolidge_cluster_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = coolidge_cluster_realize;
}

static const TypeInfo coolidge_cluster_info = {
    .name = TYPE_KVX_CLUSTER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxCoolidgeClusterState),
    .class_init = coolidge_cluster_class_init,
    .instance_init = coolidge_cluster_instance_init,
};

static void coolidge_cluster_register_types(void)
{
    type_register_static(&coolidge_cluster_info);
}

type_init(coolidge_cluster_register_types);
