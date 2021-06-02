/*
 * KVX cpu parameters for qemu.
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

#ifndef KVX_CPU_PARAM_H
#define KVX_CPU_PARAM_H

#define TARGET_PAGE_BITS 12 /* 4 KiB pages */
#define TARGET_LONG_BITS 64

#define TARGET_PHYS_ADDR_SPACE_BITS 40
#define TARGET_VIRT_ADDR_SPACE_BITS 41

#define NB_MMU_MODES 16

/* The extra word is used to store instruction syndrome */
#define TARGET_INSN_START_EXTRA_WORDS 1

#endif
