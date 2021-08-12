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
#include "qemu/units.h"
#include "qemu-common.h"
#include "qemu/error-report.h"
#include "qemu/datadir.h"
#include "qapi/error.h"
#include "elf.h"
#include "hw/loader.h"
#include "hw/boards.h"
#include "sysemu/device_tree.h"
#include "sysemu/reset.h"

#include "hw/kvx/boot.h"

#define MPPA_ARGAREA_MAGIC 0xdeadbeef

typedef enum MppaArgAreaSpawnType {
    MPPA_UNKNOWN_SPAWN = 0,
    MPPA_RUNNER_SPAWN,	      /* ISS or JTAG */
    MPPA_MPPA_SPAWN,	      /* mppa_spawn */
    MPPA_PCI_SPAWN,	          /* PCI */
    MPPA_RPROC_LINUX_SPAWN,   /* Boot by rproc Linux */
    MPPA_RPROC_PCI_SPAWN,	  /* Boot by rproc PCIe */
    MPPA_RPROC_COS_SPAWN,	  /* Boot by rproc ClusterOS */
} MppaArgAreaSpawnType;

typedef struct MppaArgArea32 {
    uint32_t magic_value;
    uint32_t spawn_type; /* MppaArgAreaSpawnType */
    uint32_t spawner_id; /* Cluster ID of spawner */
    uint32_t remote_console_enable;
    uint32_t frequency; /* frequency of the cluster */
    uint32_t argc;
    uint32_t envc;
    uint32_t argv_ptr;
    uint32_t envp_ptr;
} MppaArgArea32 __attribute__((aligned(8)));

typedef struct MppaArgArea64 {
    uint32_t magic_value;
    uint32_t spawn_type; /* MppaArgAreaSpawnType */
    uint32_t spawner_id; /* Cluster ID of spawner */
    uint32_t remote_console_enable;
    uint32_t frequency; /* frequency of the cluster */
    uint32_t argc;
    uint32_t envc;
    uint64_t argv_ptr;
    uint64_t envp_ptr;
} MppaArgArea64 __attribute__((aligned(8)));

static void fdt_set_memory_node(KvxBootInfo *info)
{
    char *nodename;
    uint32_t acells, scells;

    acells = qemu_fdt_getprop_cell(info->fdt, "/", "#address-cells",
                                   NULL, &error_fatal);
    scells = qemu_fdt_getprop_cell(info->fdt, "/", "#size-cells",
                                   NULL, &error_fatal);

    nodename = g_strdup_printf("/memory@%" PRIx64, info->ddr_base);

    qemu_fdt_setprop_sized_cells(info->fdt, nodename, "reg",
                                 acells, info->ddr_base,
                                 scells, info->ddr_size);
    g_free(nodename);
}

static void fdt_set_bootargs(KvxBootInfo *info)
{
    if (!info->kernel_cmdline || !*info->kernel_cmdline) {
        return;
    }

    if (fdt_path_offset(info->fdt, "/chosen") < 0) {
        qemu_fdt_add_subnode(info->fdt, "/chosen");
    }

    qemu_fdt_setprop_string(info->fdt, "/chosen", "bootargs",
                            info->kernel_cmdline);
}

static void setup_and_load_dtb(KvxBootInfo *info)
{
    if (info->fdt == NULL) {
        return;
    }

    fdt_set_memory_node(info);
    fdt_set_bootargs(info);

    if (!info->found_dtb_start) {
        info->dtb_load_addr = info->kernel_loaded
            ? ROUND_UP(info->kernel_high, 4 * MiB)
            : info->ddr_base;

        /*
         * Here we're loading the DTB outside of the ELF load area. We can use
         * rom_add_blob_fixed.
         */
        rom_add_blob_fixed("dtb", info->fdt, info->fdt_size,
                           info->dtb_load_addr);
    } else {
        if (info->fdt_size > info->dtb_max_size) {
            error_report("The DTB is too big to fit into the .dtb section\n");
        } else {
            void *fdt = rom_ptr(info->dtb_load_addr, info->dtb_max_size);

            /*
             * We found __dtb_start and __dtb_size. rom_ptr() should really
             * not return NULL.
             */
            g_assert(fdt != NULL);

            memcpy(fdt, info->fdt, info->fdt_size);
        }
    }

    qemu_fdt_dumpdtb(info->fdt, info->fdt_size);
}

static void do_bootloader_reset(void *opaque)
{
    KvxBootInfo *info = (KvxBootInfo *) opaque;
    AddressSpace *as = &address_space_memory;

    address_space_write_rom(as, info->mppa_argarea_start_addr,
                            MEMTXATTRS_UNSPECIFIED,
                            info->mppa_argarea_buf, info->mppa_argarea_size);
}

static inline size_t get_num_args(const KvxBootInfo *info)
{
    size_t arg_count = 0;
    const char *in = info->kernel_cmdline;

    if (!in || !*in) {
        return 0;
    }

    while (*in) {
        if (*in == ' ') {
            arg_count++;
        }
        in++;
    }
    arg_count++;

    return arg_count;
}

#define cpu_to_le_ptr(a) (info->kernel_is_64bits \
                          ? cpu_to_le64(a)       \
                          : cpu_to_le32(a))

#define argv_push(a) do {              \
    if (info->kernel_is_64bits) {      \
        ((uint64_t *)argv)[i++] = (a); \
    } else {                           \
        ((uint32_t *)argv)[i++] = (a); \
    }                                  \
} while (0)

static void parse_and_copy_args(KvxBootInfo *info, char *argv, char *args,
                                uint64_t args_addend)
{
    size_t i = 0;
    const char *in = info->kernel_cmdline;
    char *out = args, *arg_start;

    strcpy(out, info->kernel_filename);
    argv_push(cpu_to_le_ptr(((uint64_t) args) + args_addend));
    out += strlen(info->kernel_filename) + 1;

    if (!in || !*in) {
        return;
    }

    arg_start = out;

    /* copy arguments, change spaces to nul char, setup argv pointers */
    while (*in) {
        if (*in == ' ') {
            *out = '\0';
            argv_push(cpu_to_le_ptr(((uint64_t) arg_start) + args_addend));
            arg_start = out + 1;
        } else {
            *out = *in;
        }

        in++;
        out++;
    }

    *out = '\0';
    argv_push(cpu_to_le_ptr(((uint64_t) arg_start) + args_addend));
}

#define set_descr_field(field, val) do { \
    if (info->kernel_is_64bits) {        \
        descr.p64->field = (val);        \
    } else {                             \
        descr.p32->field = (val);        \
    }                                    \
} while (0)

static void setup_argarea(KvxBootInfo *info)
{
    union {
        MppaArgArea32 *p32;
        MppaArgArea64 *p64;
    } descr;

    size_t num_args, descr_size, argv_size;
    size_t arg0_size, args_size, buf_size;
    uint64_t addend;
    char *buf;

    num_args = get_num_args(info);

    descr_size = info->kernel_is_64bits
        ? sizeof(MppaArgArea64)
        : sizeof(MppaArgArea32);

    /* num_args + 2 -> one for argv[0], and one for the NULL termination */
    argv_size = (num_args + 2) * (info->kernel_is_64bits
                                  ? sizeof(uint64_t)
                                  : sizeof(uint32_t));

    arg0_size = strlen(info->kernel_filename) + 1;
    args_size = strlen(info->kernel_cmdline) + 1;

    buf_size = descr_size + argv_size + arg0_size + args_size;

    buf = g_malloc0(buf_size);
    addend = info->mppa_argarea_start_addr - ((uint64_t) buf);

    parse_and_copy_args(info,
                        buf + descr_size,
                        buf + descr_size + argv_size,
                        addend);

    descr.p64 = (MppaArgArea64 *) buf;
    set_descr_field(magic_value, cpu_to_le32(MPPA_ARGAREA_MAGIC));
    set_descr_field(spawn_type, cpu_to_le32(MPPA_RUNNER_SPAWN));
    set_descr_field(spawner_id, cpu_to_le32(0));
    set_descr_field(remote_console_enable, cpu_to_le32(0));
    set_descr_field(frequency, cpu_to_le32(info->frequency));
    set_descr_field(argc, cpu_to_le32(num_args + 1));
    set_descr_field(envc, cpu_to_le32(0));
    set_descr_field(argv_ptr, cpu_to_le_ptr(((uint64_t) buf) + descr_size + addend));
    set_descr_field(envp_ptr, cpu_to_le_ptr((uint64_t) NULL));

    info->mppa_argarea_buf = buf;
    info->mppa_argarea_size = buf_size;

    qemu_register_reset(do_bootloader_reset, info);
}

#undef cpu_to_le_ptr
#undef argv_push
#undef set_descr_field

static bool mppa_argarea_start_found;
static uint64_t mppa_argarea_start_addr;

static bool dtb_start_found;
static uint64_t dtb_start_addr;

static bool dtb_size_found;
static uint64_t dtb_size_val;

static void find_cos_syms(const char *st_name, int st_info,
                          uint64_t st_value, uint64_t st_size)
{
    if (!strcmp(st_name, "MPPA_ARGAREA_START")) {
        mppa_argarea_start_found = true;
        mppa_argarea_start_addr = st_value;
    }

    if (!strcmp(st_name, "__dtb_start")) {
        dtb_start_found = true;
        dtb_start_addr = st_value;
    }

    if (!strcmp(st_name, "__dtb_size")) {
        dtb_size_found = true;
        dtb_size_val = st_value;
    }
}

static void load_kernel(KvxBootInfo *info)
{
    if (info->kernel_filename) {
        uint64_t kernel_entry, kernel_high;
        Error *err = NULL;

        mppa_argarea_start_found = false;
        dtb_start_found = false;
        dtb_size_found = false;

        load_elf_hdr(info->kernel_filename, NULL,
                     &info->kernel_is_64bits, &err);

        if (err) {
            error_free(err);
            return;
        }

        if (load_elf_ram_sym(info->kernel_filename, NULL, NULL, NULL,
                             &kernel_entry, NULL, &kernel_high, NULL,
                             0, EM_KVX, 1, 0, NULL, true,
                             find_cos_syms) <= 0) {
            return;
        }

        info->kernel_loaded = true;
        info->kernel_entry = kernel_entry;
        info->kernel_high = kernel_high;

        if (info->gen_mppa_argarea && mppa_argarea_start_found) {
            info->mppa_argarea_start_addr = mppa_argarea_start_addr;
            setup_argarea(info);

            if (dtb_start_found && dtb_size_found) {
                info->found_dtb_start = true;
                info->dtb_load_addr = dtb_start_addr;
                info->dtb_max_size = dtb_size_val;
            }
        }
    }
}

void kvx_boot(KvxBootInfo *info)
{
    load_kernel(info);
    setup_and_load_dtb(info);
}