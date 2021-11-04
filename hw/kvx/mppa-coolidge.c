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
#include "hw/qdev-properties.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"

#include "hw/kvx/coolidge-cluster.h"
#include "hw/kvx/mppa-coolidge.h"

static inline void devices_init(MppaCoolidgeMachineState *s)
{
    int i;
    uint64_t ram_size = MACHINE(s)->ram_size;
    MemoryRegion *system_memory = get_system_memory();

    /* Cluster memory region */
    object_initialize_child(OBJECT(s), "cluster[*]", &s->cluster, TYPE_KVX_CLUSTER);
    memory_region_add_subregion(get_system_memory(),
                                MPPA_CLUSTER_MMIO_LEN,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->cluster), 0));
    s->cluster.gen_mppa_argarea = s->gen_mppa_argarea;
    s->cluster.gen_dtb = s->gen_dtb;
    s->cluster.frequency = s->frequency;
    s->cluster.initial_dsu_clock = s->initial_dsu_clock;

    /* UARTs */
    for (i = 0; i < ARRAY_SIZE(s->uart); i++) {
        object_initialize_child(OBJECT(s), "uart[*]", &s->uart[i],
                                TYPE_SERIAL_MM);
    }

    /* itgen IRQ controllers */
    object_initialize_child(OBJECT(s), "itgen0", &s->itgen0, TYPE_KVX_ITGEN);
    memory_region_add_subregion(get_system_memory(),
                                periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_ITGEN0),
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->itgen0), 0));

    object_initialize_child(OBJECT(s), "itgen1", &s->itgen1, TYPE_KVX_ITGEN);
    memory_region_add_subregion(get_system_memory(),
                                periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_ITGEN1),
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->itgen1), 0));

    /* Ftu */
    object_initialize_child(OBJECT(s), "ftu", &s->ftu, TYPE_KVX_FTU);
    memory_region_add_subregion(get_system_memory(),
                                periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_FTU),
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->ftu), 0));

    /* DDR */
    memory_region_init_ram(&s->ddr, NULL, "mppa-cluster.ddr",
                           ram_size,
                           &error_fatal);
    memory_region_add_subregion(system_memory,
                                periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_DDR),
                                &s->ddr);

    memory_region_init_alias(&s->ddr_32bits_alias, NULL,
                             "mppa-cluster.ddr-32bits-alias",
                             &s->ddr, 0,
                             periph_mmio_size(mppa_cluster_periphs, MPPA_CLUSTER_DDR_32BITS_ALIAS));

    memory_region_add_subregion(system_memory,
                                periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_DDR_32BITS_ALIAS),
                                &s->ddr_32bits_alias);
}

static inline void devices_realize(MppaCoolidgeMachineState *s)
{
    int i;

    sysbus_realize(SYS_BUS_DEVICE(&s->cluster), &error_abort);

    /* UARTs */
    for (i = 0; i < ARRAY_SIZE(s->uart); i++) {
        uint64_t addr = periph_mmio_base(mppa_cluster_periphs, MPPA_CLUSTER_UART0 + i);
        int irq_num = periph_irq_mapping_idx(mppa_cluster_periphs, MPPA_CLUSTER_UART0 + i, 0);
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

    /* FTU */
    sysbus_realize(SYS_BUS_DEVICE(&s->ftu), &error_abort);
}

static void mppa_cluster_init(MachineState *machine)
{
    MppaCoolidgeMachineState *s = MPPA_COOLIDGE(machine);

    devices_init(s);
    devices_realize(s);
}

static void mppa_cluster_machine_class_init(ObjectClass *klass, void *data)
{
    MachineClass *mc = MACHINE_CLASS(klass);

    mc->desc = "Kalray MPPA";
    mc->init = mppa_cluster_init;
    mc->max_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->default_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->max_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->min_cpus = MPPA_CLUSTER_NUM_CPUS;
    mc->default_cpu_type = TYPE_KVX_CPU_KV3_V1;
    mc->is_default = true;
}

static bool mppa_cluster_get_gen_argarea(Object *obj, Error **errp)
{
    MppaCoolidgeMachineState *s = MPPA_COOLIDGE(obj);

    return s->gen_mppa_argarea;
}

static void mppa_cluster_set_gen_argarea(Object *obj, bool value, Error **errp)
{
    MppaCoolidgeMachineState *s = MPPA_COOLIDGE(obj);

    s->gen_mppa_argarea = value;
}

static bool mppa_cluster_get_gen_dtb(Object *obj, Error **errp)
{
    MppaCoolidgeMachineState *s = MPPA_COOLIDGE(obj);

    return s->gen_dtb;
}

static void mppa_cluster_set_gen_dtb(Object *obj, bool value, Error **errp)
{
    MppaCoolidgeMachineState *s = MPPA_COOLIDGE(obj);

    s->gen_dtb = value;
}

static void mppa_cluster_instance_init(Object *obj)
{
    MppaCoolidgeMachineState *s = MPPA_COOLIDGE(obj);

    s->gen_mppa_argarea = true;
    object_property_add_bool(obj, "generate-mppa-argarea",
                             mppa_cluster_get_gen_argarea,
                             mppa_cluster_set_gen_argarea);
    object_property_set_description(obj, "generate-mppa-argarea",
                                    "Set on/off to enable/disable generation "
                                    "of the .mppa_arg section when one is "
                                    "detected in the loaded kernel "
                                    "(default is on)");

    s->gen_dtb = true;
    object_property_add_bool(obj, "generate-dtb",
                             mppa_cluster_get_gen_dtb,
                             mppa_cluster_set_gen_dtb);
    object_property_set_description(obj, "generate-dtb",
                                    "Set on/off to enable/disable generation "
                                    "of the device tree"
                                    "(default is on)");

    /* Default to 1MHz */
    s->frequency = 1000000;
    object_property_add_uint64_ptr(obj, "frequency", &s->frequency,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "frequency",
                                    "Cluster frequency in Hz (default is 1MHz)");

    s->initial_dsu_clock = 0;
    object_property_add_uint64_ptr(obj, "initial-dsu-clock", &s->initial_dsu_clock,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "initial-dsu-clock",
                                    "Initial value of the DSU clock  (default is 0)");
}

static const TypeInfo mppa_coolidge_machine_type_info[] = {
    {
        .name = TYPE_MPPA_COOLIDGE_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(MppaCoolidgeMachineState),
        .class_init = mppa_cluster_machine_class_init,
        .instance_init = mppa_cluster_instance_init,
    },
};

DEFINE_TYPES(mppa_coolidge_machine_type_info)
