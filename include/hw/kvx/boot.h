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
    const char *dtb_filename;
    uint64_t dtb_load_addr;
    uint64_t ddr_base;
    uint64_t ddr_size;

    /* Resulting info when bootloading is done */
    bool kernel_loaded;
    uint64_t kernel_entry;
    bool dtb_loaded;
} KvxBootInfo;

void kvx_boot(KvxBootInfo *info);
void kvx_boot_fill_info_from_machine(KvxBootInfo *info, MachineState *machine);

#endif
