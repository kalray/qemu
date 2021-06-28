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

#include "qemu/osdep.h"
#include "hw/boards.h"
#include "sysemu/device_tree.h"
#include "machine-descr.h"

#include <libfdt.h>

static void create_fdt_subnode_rec(void *fdt, const char *node)
{
    char *copy = g_strdup(node);
    char *cur = copy;
    char *end;

    do {
        int offset;

        end = strchr(cur, '/');

        if (end) {
            *end = '\0';
        }

        offset = fdt_path_offset(fdt, copy);

        if (offset < 0) {
            qemu_fdt_add_subnode(fdt, copy);
        }

        if (end) {
            cur = end + 1;
            *end = '/';
        }
    } while (end);

    g_free(copy);
}

static void create_fdt_node(MachineState *m, const PeriphEntry *periphs,
                            size_t len, int id)
{
    void *fdt = m->fdt;
    const PeriphEntry *p = &periphs[id];
    char *node;
    bool is_cpu;
    int i, count;

    if (p->fdt.compatible == NULL) {
        return;
    }

    is_cpu = (strstr(p->fdt.node, "/cpus") == p->fdt.node);

    count = (p->count > 1) ? p->count : 1;

    for (i = count - 1; i >= 0; i--) {
        if (is_cpu) {
            node = g_strdup_printf("%s@%d", p->fdt.node, i);
        } else if (p->mmio_map.valid) {
            node = g_strdup_printf("%s@%" PRIx64, p->fdt.node, p->mmio_map.base);
        } else {
            node = g_strdup_printf("%s", p->fdt.node);
        }

        create_fdt_subnode_rec(fdt, node);

        if (!strcmp(p->fdt.compatible, "memory")) {
            qemu_fdt_setprop_string(fdt, node, "device_type", p->fdt.compatible);
        } else {
            qemu_fdt_setprop_string(fdt, node, "compatible", p->fdt.compatible);
        }

        if (p->bus.valid) {
            qemu_fdt_setprop_cell(fdt, node, "#size-cells",
                                   p->bus.fdt_size_cells);
            qemu_fdt_setprop_cell(fdt, node, "#address-cells",
                                   p->bus.fdt_address_cells);

            if (strcmp(node, "/")) {
                /* TODO: for now, we only support 1:1 mapping with the parent */
                qemu_fdt_setprop(fdt, node, "ranges", NULL, 0);
            }
        }

        if (is_cpu) {
            qemu_fdt_setprop_cells(fdt, node, "reg", i);
        } else if (p->mmio_map.valid) {
            uint64_t size = (p->mmio_map.get_size_cb)
                ? p->mmio_map.get_size_cb(m)
                : p->mmio_map.size;

            /* TODO: look for parent #size-cells and #address-cells */
            qemu_fdt_setprop_cells(fdt, node, "reg",
                                   p->mmio_map.base >> 32, p->mmio_map.base,
                                   size >> 32, size);
        }

        if (p->irq_ctrl.valid) {
            qemu_fdt_setprop(fdt, node, "interrupt-controller", NULL, 0);
            qemu_fdt_setprop_cell(fdt, node, "#interrupt-cells",
                                  p->irq_ctrl.fdt_interrupt_cells);
        }

        if (p->irq_map.valid) {
            const IRQMapEntry *irq = &p->irq_map;
            int parent_irq_cells;
            const IRQMapping *map;
            GArray *irq_cell;

            g_assert(irq->parent < len);
            g_assert(periphs[irq->parent].irq_ctrl.valid);
            parent_irq_cells = periphs[irq->parent].irq_ctrl.fdt_interrupt_cells;
            g_assert(parent_irq_cells > 0);
            g_assert(parent_irq_cells <= 2);

            qemu_fdt_setprop_cell(fdt, node, "interrupt-parent",
                                  periph_fdt_get_phandle(irq->parent));

            irq_cell = g_array_new(false, false, sizeof(uint32_t));
            map = p->irq_map.mapping;

            while (map->idx != -1) {
                uint32_t val;

                val = cpu_to_be32(map->idx);
                g_array_append_val(irq_cell, val);
                if (parent_irq_cells == 2) {
                    val = cpu_to_be32(map->flags);
                    g_array_append_val(irq_cell, val);
                }

                map++;
            }

            qemu_fdt_setprop(fdt, node, "interrupts",
                             irq_cell->data, irq_cell->len * sizeof(uint32_t));

            g_array_free(irq_cell, true);
        }

        if (p->msi_ctrl.valid) {
            qemu_fdt_setprop(fdt, node, "msi-controller", NULL, 0);
        }

        if (p->msi_map.valid) {
            qemu_fdt_setprop_cell(fdt, node, "msi-parent",
                                  periph_fdt_get_phandle(p->msi_map.parent));
        }

        if (p->fixed_clock.valid) {
            uint32_t freq = p->fixed_clock.freq_cb
                ? p->fixed_clock.freq_cb(m)
                : p->fixed_clock.freq;

            qemu_fdt_setprop_cell(fdt, node, "#clock-cells", 0);
            qemu_fdt_setprop_cell(fdt, node, "clock-frequency", freq);
        }

        if (p->clock_map.valid) {
            GArray *clocks_node = g_array_new(false, false, sizeof(uint32_t));
            const size_t *clock = p->clock_map.clocks;

            while (*clock != CLOCK_END) {
                uint32_t val = cpu_to_be32(periph_fdt_get_phandle(*clock));
                g_array_append_val(clocks_node, val);
                clock++;
            }

            qemu_fdt_setprop(fdt, node, "clocks",
                             clocks_node->data,
                             clocks_node->len * sizeof(uint32_t));

            g_array_free(clocks_node, true);
        }

        qemu_fdt_setprop_cell(fdt, node, "phandle", periph_fdt_get_phandle_idx(id, i));

        if (p->fdt.cb) {
            p->fdt.cb(m, fdt, node, id, i);
        }

        g_free(node);
    }
}

void machine_populate_fdt(MachineState *m, const PeriphEntry *periphs, size_t len)
{
    void *fdt = m->fdt;
    int i;

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0x0);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 0x1);

    for (i = len - 1; i >= 0; i--) {
        create_fdt_node(m, periphs, len, i);
    }
}
