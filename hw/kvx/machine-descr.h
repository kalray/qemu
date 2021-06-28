/*
 * Structural QEMU machine description
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

#ifndef HW_KVX_MACHINE_DESCR_H
#define HW_KVX_MACHINE_DESCR_H

#include "exec/hwaddr.h"

typedef struct PeriphEntry PeriphEntry;

typedef uint64_t (*GetMMIOMapSizeCb)(MachineState *);
typedef uint64_t (*GetFixedClockFreqCb)(MachineState *);
typedef void (*FdtNodeCb)(MachineState *, void *fdt,
                          const char *node, size_t id, int idx);

typedef struct FdtNode {
    const char *node;
    const char *compatible;
    FdtNodeCb cb;
} FdtNode;

typedef struct BusEntry {
    bool valid;
    int fdt_size_cells;
    int fdt_address_cells;
} BusEntry;

typedef struct MMIOMapEntry {
    bool valid;
    hwaddr base;
    hwaddr size;
    GetMMIOMapSizeCb get_size_cb;
} MMIOMapEntry;

typedef struct IRQControllerEntry {
    bool valid;
    int fdt_interrupt_cells;
} IRQControllerEntry;

#define IRQ(idx) { idx, 0 }
#define IRQ_FLAG(idx, flag) { idx, flag }
#define IRQ_END IRQ_FLAG(-1, -1)

#define IRQS(...) \
    (IRQMapping []) { __VA_ARGS__, IRQ_END }

typedef struct IRQMapping {
    uint32_t idx;
    uint32_t flags;
} IRQMapping;

typedef struct IRQMapEntry {
    bool valid;
    size_t parent;
    IRQMapping *mapping;
} IRQMapEntry;

typedef struct MSIControllerEntry {
    bool valid;
} MSIControllerEntry;

typedef struct MSIMapEntry {
    bool valid;
    size_t parent;
} MSIMapEntry;

typedef struct FixedClockEntry {
    bool valid;
    uint64_t freq;
    GetFixedClockFreqCb freq_cb;
} FixedClockEntry;

#define CLOCK_END (-1)
#define CLOCKS(...) \
    (size_t []) { __VA_ARGS__, CLOCK_END }

typedef struct ClockMapEntry {
    bool valid;
    size_t *clocks;
} ClockMapEntry;

struct PeriphEntry {
    size_t count;

    FdtNode fdt;
    BusEntry bus;
    MMIOMapEntry mmio_map;
    IRQControllerEntry irq_ctrl;
    IRQMapEntry irq_map;
    MSIControllerEntry msi_ctrl;
    MSIMapEntry msi_map;
    FixedClockEntry fixed_clock;
    ClockMapEntry clock_map;
};

void machine_populate_fdt(MachineState *m, const PeriphEntry *periphs,
                          size_t len);

static inline int periph_fdt_get_phandle(size_t periph)
{
    return periph + 1;
}

static inline int periph_fdt_get_phandle_idx(size_t periph, int idx)
{
    return periph + idx + 1;
}

static inline uint64_t periph_mmio_base(const PeriphEntry *periphs, size_t idx)
{
    const MMIOMapEntry *mmio = &periphs[idx].mmio_map;

    g_assert(mmio->valid);

    return mmio->base;
}

static inline uint64_t periph_mmio_size(const PeriphEntry *periphs, size_t idx)
{
    const MMIOMapEntry *mmio = &periphs[idx].mmio_map;

    g_assert(mmio->valid);

    return mmio->size;
}

static inline uint64_t periph_irq_mapping_idx(const PeriphEntry *periphs,
                                              size_t idx, size_t irq_idx)
{
    const IRQMapEntry *irq = &periphs[idx].irq_map;

    g_assert(irq->valid);

    return irq->mapping[irq_idx].idx;
}

#endif
