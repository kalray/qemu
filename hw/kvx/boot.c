/*
 * Kalray KVX MPPA boot helpers
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

#include <libfdt.h>

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/error-report.h"
#include "qemu/datadir.h"
#include "qapi/error.h"
#include "elf.h"
#include "hw/loader.h"
#include "hw/boards.h"
#include "sysemu/device_tree.h"

#include "hw/kvx/boot.h"

static void fdt_set_memory_node(KvxBootInfo *info, void *fdt)
{
    char *nodename;
    uint32_t acells, scells;

    acells = qemu_fdt_getprop_cell(fdt, "/", "#address-cells",
                                   NULL, &error_fatal);
    scells = qemu_fdt_getprop_cell(fdt, "/", "#size-cells",
                                   NULL, &error_fatal);

    nodename = g_strdup_printf("/memory@%" PRIx64, info->ddr_base);

    qemu_fdt_setprop_sized_cells(fdt, nodename, "reg",
                                 acells, info->ddr_base,
                                 scells, info->ddr_size);
    g_free(nodename);
}

static void fdt_set_bootargs(KvxBootInfo *info, void *fdt)
{
    if (!info->kernel_cmdline || !*info->kernel_cmdline) {
        return;
    }

    if (fdt_path_offset(fdt, "/chosen") < 0) {
        qemu_fdt_add_subnode(fdt, "/chosen");
    }

    qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                            info->kernel_cmdline);
}

static void load_and_setup_dtb(KvxBootInfo *info)
{
    char *filename;
    void *fdt;
    int size;

    if (!info->dtb_filename) {
        return;
    }

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, info->dtb_filename);
    if (!filename) {
        error_report("could not open dtb file %s\n", info->dtb_filename);
        exit(1);
    }

    fdt = load_device_tree(filename, &size);
    if (!fdt) {
        error_report("could not open dtb file %s\n", filename);
        g_free(filename);
        exit(1);
    }

    g_free(filename);

    fdt_set_memory_node(info, fdt);
    fdt_set_bootargs(info, fdt);

    rom_add_blob_fixed("dtb", fdt, size, info->dtb_load_addr);
    g_free(fdt);

    info->dtb_loaded = true;
}

static void load_kernel(KvxBootInfo *info)
{
    if (info->kernel_filename) {
        uint64_t kernel_entry, kernel_high;

        if (load_elf(info->kernel_filename, NULL, NULL, NULL,
                     &kernel_entry, NULL, &kernel_high, NULL,
                     0, EM_KVX, 1, 0) <= 0) {
            error_report("could not load kernel '%s'", info->kernel_filename);
            exit(1);
        }

        info->kernel_loaded = true;
        info->kernel_entry = kernel_entry;
    }
}

void kvx_boot(KvxBootInfo *info)
{
    load_kernel(info);
    load_and_setup_dtb(info);
}

void kvx_boot_fill_info_from_machine(KvxBootInfo *info, MachineState *machine)
{
    info->kernel_filename = machine->kernel_filename;
    info->kernel_cmdline = machine->kernel_cmdline;
    info->dtb_filename = machine->dtb;
}
