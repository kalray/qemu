/*
 * Kalray KVX MPPA cluster IPI controller
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

#ifndef HW_KVX_IPI_CTRL_H
#define HW_KVX_IPI_CTRL_H

#include "hw/sysbus.h"

#define TYPE_KVX_IPI_CTRL "kvx-ipi-ctrl"
#define KVX_IPI_CTRL(obj) \
    OBJECT_CHECK(KvxIpiCtrlState, (obj), TYPE_KVX_IPI_CTRL)

/* according to the device tree */
#define KVX_IPI_CTRL_MMIO_LEN 0x1000

#define KVX_IPI_CTRL_NUM_IRQ 17

typedef struct KvxIpiCtrlState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    uint32_t reg_mask;

    MemoryRegion iomem;

    qemu_irq irq_out[KVX_IPI_CTRL_NUM_IRQ];
} KvxIpiCtrlState;

#endif
