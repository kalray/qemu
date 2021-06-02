/*
 * Kalray KVX MPPA cluster APIC Mailbox
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

#ifndef HW_KVX_APIC_MAILBOX_H
#define HW_KVX_APIC_MAILBOX_H

#include "hw/sysbus.h"

#define TYPE_KVX_APIC_MAILBOX "kvx-apic-mailbox"
#define KVX_APIC_MAILBOX(obj) \
    OBJECT_CHECK(KvxApicMailboxState, (obj), TYPE_KVX_APIC_MAILBOX)

#define KVX_APIC_MAILBOX_NUM_IRQ_OUT 121

#define KVX_APIC_MAILBOX_MB_SIZE 0x200
#define KVX_APIC_MAILBOX_NUM_MB KVX_APIC_MAILBOX_NUM_IRQ_OUT

/* matches the device tree */
#define KVX_APIC_MAILBOX_MMIO_LEN \
    (KVX_APIC_MAILBOX_NUM_MB * KVX_APIC_MAILBOX_MB_SIZE)

typedef struct KvxApicMailbox {
    uint64_t value;
    uint64_t mask;
    uint64_t func;
} KvxApicMailbox;

typedef struct KvxApicMailboxState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    KvxApicMailbox mailboxes[KVX_APIC_MAILBOX_NUM_MB];

    MemoryRegion iomem;

    qemu_irq irq_out[KVX_APIC_MAILBOX_NUM_IRQ_OUT];
} KvxApicMailboxState;

#endif
