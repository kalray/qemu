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

#ifndef HW_KVX_BOOT_H
#define HW_KVX_BOOT_H

typedef struct KvxBootInfo {
    /* Bootloader params */
    const char *kernel_filename;
    const char *kernel_cmdline;
    void *fdt;
    int fdt_size;
    uint64_t ddr_base;
    uint64_t ddr_size;
    bool gen_mppa_argarea;

    /* Resulting info when bootloading is done */
    bool kernel_loaded;
    bool kernel_is_64bits;
    uint64_t kernel_entry;
    uint64_t kernel_high;
    bool found_dtb_start;   /* Found __dtb_start symbol in kernel */
    uint64_t dtb_load_addr;
    uint64_t dtb_max_size;  /* set with __dtb_size symbol value when found */

    uint64_t mppa_argarea_start_addr;
    char * mppa_argarea_buf;
    size_t mppa_argarea_size;
} KvxBootInfo;

void kvx_boot(KvxBootInfo *info);
void kvx_boot_fill_info_from_machine(KvxBootInfo *info, MachineState *machine);

#endif
