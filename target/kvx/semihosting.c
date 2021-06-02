/*
 * QEMU Kalray kvx CPU
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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "exec/gdbstub.h"
#include "semihosting/semihost.h"
#include "semihosting/console.h"
#include "cpu.h"
#include "internal.h"

enum Syscall {
    SCALL_trace_notify = 4055,
    SCALL_is_simu      = 4056,
    SCALL_notify_spawn = 4057,
    SCALL_inbyte       = 4058,
    SCALL_poll         = 4059,
    SCALL_close        = 4062,
    SCALL_fstat        = 4063,
    SCALL_load_elf     = 4072,
    SCALL_mkfifo       = 4073,
    SCALL_link         = 4074,
    SCALL_unlink       = 4075,
    SCALL_lseek        = 4076,
    SCALL_open         = 4077,
    SCALL_read         = 4078,
    SCALL_stat         = 4079,
    SCALL_gettimeofday = 4080,
    SCALL_write        = 4081,
    SCALL_chmod        = 4082,
    SCALL_isatty       = 4083,
    SCALL_dup          = 4084,
    SCALL_dup2         = 4085,
    SCALL_fcntl        = 4086,
    SCALL_mkdir        = 4087,
    SCALL_rmdir        = 4088,
    SCALL_access       = 4089,
    SCALL_chdir        = 4090,
    SCALL_getdents     = 4091,
    SCALL_readtimer    = 4093,
    SCALL_printf       = 4094,
    SCALL_exit         = 4095,
};

#define ARG(i) arg ## i = kvx_register_read_u64(env, REG_kv3_R ## i)

#define ARG_1 ARG(0)
#define ARG_2 ARG_1; ARG(1)
#define ARG_3 ARG_2; ARG(2)
#define ARGS(i) do { ARG_ ## i; } while (0)

#define RET(val) kvx_register_write_u64(env, REG_kv3_R0, val)

void kvx_do_semihosting(CPUKVXState *env, uint64_t scall)
{
    uint64_t arg0, arg1;

    switch (scall) {
    case SCALL_printf:
        ARGS(2);
        RET(arg1);
        while (arg1--) {
            qemu_semihosting_console_outc(env, arg0++);
        }
        break;

    case SCALL_exit:
        ARGS(1);
        gdb_exit(arg0);
        exit(arg0);
        break;

    default:
        break;
    }
}
