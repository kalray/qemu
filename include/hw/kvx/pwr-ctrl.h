/*
 * Kalray KVX MPPA cluster power controller
 *
 * Copyright (c) 2020 GreenSocs SAS
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

#ifndef HW_KVX_PWR_CTRL_H
#define HW_KVX_PWR_CTRL_H

#include "hw/sysbus.h"

#define TYPE_KVX_PWR_CTRL "kvx-pwr-ctrl"
#define KVX_PWR_CTRL(obj) \
    OBJECT_CHECK(KvxPwrCtrlState, (obj), TYPE_KVX_PWR_CTRL)

/* according to the device tree */
#define KVX_PWR_CTRL_MMIO_LEN 0x4188

typedef struct KvxPwrCtrlState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    uint64_t reg_pe_control[16];
    uint64_t reg_vector_proc_control_wup;
    uint64_t reg_vector_proc_control_reset_on_wakeup;
    uint64_t reg_reset_pc;
    uint64_t reg_global_config;
    uint64_t reg_rm_control;

    MemoryRegion iomem;
} KvxPwrCtrlState;

#endif
