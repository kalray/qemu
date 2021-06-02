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

/*
 * Note: this model has been written without any documentation. It was
 *       developped by retro-engineering the corresponding Linux kernel driver.
 *       It is incomplete and probably wrong in some places.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "hw/registerfields.h"
#include "hw/kvx/apic-mailbox.h"
#include "cpu.h"
#include "trace.h"

REG64(VALUE, 0x0)
REG64(LAC, 0x8)
REG64(MASK, 0x10)
REG64(FUNC, 0x18)
    FIELD(FUNC, MODE, 0, 2)
    FIELD(FUNC, TRIGGER, 8, 3)

typedef enum KvxApicMailboxWriteMode {
    MB_MODE_WRITE = 0,
    MB_MODE_OR = 1,
    MB_MODE_ADD = 2, /* XXX not implemented */
} KvxApicMailboxWriteMode;

static inline KvxApicMailboxWriteMode get_write_mode(const KvxApicMailbox *mb)
{
    return FIELD_EX64(mb->func, FUNC, MODE);
}

static inline void do_value_write(KvxApicMailbox *mb, uint64_t value)
{
    KvxApicMailboxWriteMode mode;

    mode = get_write_mode(mb);

    switch (mode) {
    case MB_MODE_WRITE:
        break;
    case MB_MODE_OR:
        value |= mb->value;
        break;
    case MB_MODE_ADD:
        qemu_log_mask(LOG_UNIMP, "kvx-apic-mailbox: unimplemented ADD mode\n");
        break;
    }

    mb->value = value;
}

static uint64_t kvx_apic_mailbox_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    KvxApicMailboxState *s = KVX_APIC_MAILBOX(opaque);
    KvxApicMailbox *mb;
    size_t mb_idx;
    uint64_t ret;

    mb_idx = offset / KVX_APIC_MAILBOX_MB_SIZE;
    offset %= KVX_APIC_MAILBOX_MB_SIZE;
    g_assert(mb_idx < KVX_APIC_MAILBOX_NUM_MB);
    mb = &s->mailboxes[mb_idx];

    switch (offset) {
    case A_VALUE:
        ret = mb->value;
        break;

    case A_LAC:
        ret = mb->value;
        mb->value = 0;
        break;

    case A_MASK:
        ret = mb->mask;
        break;

    case A_FUNC:
        ret = mb->func;
        break;

    default:
        ret = 0;
    }

    trace_kvx_apic_mailbox_read(mb_idx, offset, ret);

    return ret;
}

static void mailbox_write_value(KvxApicMailboxState *s, size_t mb_idx,
                                uint64_t value)
{
    KvxApicMailbox *mb;

    mb = &s->mailboxes[mb_idx];

    if (value & mb->mask) {
        trace_kvx_apic_mailbox_irq(mb_idx);
        qemu_irq_pulse(s->irq_out[mb_idx]);
    }

    do_value_write(mb, value);
}

static void kvx_apic_mailbox_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    KvxApicMailboxState *s = KVX_APIC_MAILBOX(opaque);
    KvxApicMailbox *mb;
    size_t mb_idx;

    mb_idx = offset / KVX_APIC_MAILBOX_MB_SIZE;
    offset %= KVX_APIC_MAILBOX_MB_SIZE;
    g_assert(mb_idx < KVX_APIC_MAILBOX_NUM_MB);
    mb = &s->mailboxes[mb_idx];

    trace_kvx_apic_mailbox_write(mb_idx, offset, value);

    switch (offset) {
    case A_VALUE:
        mailbox_write_value(s, mb_idx, value);
        break;

    case A_LAC:
        do_value_write(mb, value);
        break;

    case A_MASK:
        mb->mask = value;
        break;

    case A_FUNC:
        mb->func = value;
        break;

    default:
        break;
    }
}

static const MemoryRegionOps kvx_apic_mailbox_ops = {
    .read = kvx_apic_mailbox_read,
    .write = kvx_apic_mailbox_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        /* assume unaligned accesses are prohibited */
        .unaligned = false,
        .min_access_size = 8,
        .max_access_size = 8,
    },
    .impl = {
        .unaligned = false,
        .min_access_size = 8,
        .max_access_size = 8,
    },
};

static void kvx_apic_mailbox_reset(DeviceState *dev)
{
    KvxApicMailboxState *s = KVX_APIC_MAILBOX(dev);

    memset(s->mailboxes, 0, sizeof(s->mailboxes));
}

static void kvx_apic_mailbox_init(Object *obj)
{
    KvxApicMailboxState *s = KVX_APIC_MAILBOX(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_apic_mailbox_ops, s,
                          TYPE_KVX_APIC_MAILBOX, KVX_APIC_MAILBOX_MMIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq_out, KVX_APIC_MAILBOX_NUM_IRQ_OUT);
}

static void kvx_apic_mailbox_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_apic_mailbox_reset;
}

static TypeInfo kvx_apic_mailbox_info = {
    .name          = TYPE_KVX_APIC_MAILBOX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxApicMailboxState),
    .class_init    = kvx_apic_mailbox_class_init,
    .instance_init = kvx_apic_mailbox_init,
};

static void kvx_apic_mailbox_register_types(void)
{
    type_register_static(&kvx_apic_mailbox_info);
}

type_init(kvx_apic_mailbox_register_types)
