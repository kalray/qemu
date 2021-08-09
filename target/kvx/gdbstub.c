/*
 * QEMU Kalray kvx CPU
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
#include "exec/gdbstub.h"
#include "cpu.h"
#include "gen/gdbstub.inc.c"
#include "internal.h"

int kvx_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    KVXCPU *cpu = KVX_CPU(cs);
    int res = 0;
    Register reg;

    if (n < 0 || n >= sizeof(GDB_REGISTERS) / sizeof(GDB_REGISTERS[0])) {
        return 0;
    }
    reg = GDB_REGISTERS[n];

    switch (REGISTERS[reg].reg_width) {
    case 32:
        res = gdb_get_reg32(mem_buf, kvx_register_read(cpu, reg));
        break;
    case 64:
        if (reg == REG_kv3_ILR) {
            kvx_sync_ilr(&cpu->env);
        }
        res = gdb_get_reg64(mem_buf, kvx_register_read(cpu, reg));
        break;
    default:
        g_assert_not_reached();
        break;
    }

    return res;
}

int kvx_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    KVXCPU *cpu = KVX_CPU(cs);
    int res = 0;

    switch(n) {
        uint64_t tmp;
    case GDB_REG_kv3_r0 ... GDB_REG_kv3_r63:
        tmp = ldq_p(mem_buf);
        kvx_register_write(cpu, GDB_REGISTERS[n], tmp);
        res = 8;
        break;
    }

    return res;
}
