/*
 * Kalray KVX MPPA cluster APIC GIC
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

#ifndef HW_KVX_APIC_GIC_H
#define HW_KVX_APIC_GIC_H

#include "hw/sysbus.h"

#define TYPE_KVX_APIC_GIC "kvx-apic-gic"
#define KVX_APIC_GIC(obj) \
    OBJECT_CHECK(KvxApicGicState, (obj), TYPE_KVX_APIC_GIC)

/* according to the device tree */
#define KVX_APIC_GIC_MMIO_LEN 0x12000

#define KVX_APIC_GIC_NUM_IRQ_IN 157

/* 16 PEs, RM, DMA */
#define KVX_APIC_GIC_NUM_GROUP  18
#define KVX_APIC_GIC_NUM_LINE   4
#define KVX_APIC_GIC_NUM_IRQ_OUT \
    (KVX_APIC_GIC_NUM_GROUP * KVX_APIC_GIC_NUM_LINE)

#define KVX_APIC_GIC_LINE_SIZE 0x400
#define KVX_APIC_GIC_GROUP_SIZE \
    (KVX_APIC_GIC_LINE_SIZE * KVX_APIC_GIC_NUM_LINE)
#define KVX_APIC_GIC_NUM_REG_LAC 3

/* GIC enable register definitions */
#define KVX_GIC_ENABLE_OFFSET     0x0
#define KVX_GIC_ENABLE_ELEM_SIZE  0x1
#define KVX_GIC_INPUT_IT_COUNT 0x9D
#define KVX_GIC_ELEM_SIZE 0x400

/* GIC status lac register definitions */
#define KVX_GIC_STATUS_LAC_OFFSET     0x120
#define KVX_GIC_STATUS_LAC_ELEM_SIZE  0x8
#define KVX_GIC_STATUS_LAC_ARRAY_SIZE 0x3

typedef struct KvxApicGicState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/

    /*
     * Second dimension: One bit -> one input irq. Bit set when the corresponding IRQ
     * has triggered (assuming edge triggered IRQs).
     */
    uint64_t irq_in_set[KVX_APIC_GIC_NUM_IRQ_OUT][KVX_APIC_GIC_NUM_REG_LAC];

    /*
     * Second dimension: on which output line this input IRQ (first dimension)
     * is enabled.
     * 2 * 64 bits == 128 is enough for 18 * 4 == 72 output lines.
     */
    uint64_t irq_in_enabled[KVX_APIC_GIC_NUM_IRQ_IN][2];

    MemoryRegion iomem;

    qemu_irq irq_out[KVX_APIC_GIC_NUM_IRQ_OUT];
} KvxApicGicState;

/*
 * Convert a (group, line) pair to the corresponding output IRQ index. This
 * index is used to address the irq_in_set first array's dimension, and the bit
 * number in irq_in_enabled[one_in_irq].
 */
static inline size_t group_line_to_irq_out_idx(size_t group, size_t line)
{
    return (group * KVX_APIC_GIC_NUM_LINE) + line;
}

/*
 * Convert an `irq_in` to to corresponding (index, bit) pair in the
 * irq_in_set[one_irq_out] array.
 *
 * @param[in] irq_in: the input IRQ index to convert
 * @param[out] bit: a pointer to a variable where the bit number is written
 *
 * @return the index
 */
static inline size_t irq_in_to_lac_idx_bit(size_t irq_in, size_t *bit)
{
    size_t idx;

    g_assert(irq_in < KVX_APIC_GIC_NUM_IRQ_IN);

    idx = irq_in >> 6;
    *bit = irq_in & 0x3f;

    return idx;
}

#endif
