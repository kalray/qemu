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
#include "trace.h"

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

typedef enum TargetFdFlags {
    FD_FLAGS_RDONLY   = 0x001,
    FD_FLAGS_WRONLY   = 0x002,
    FD_FLAGS_RDWR     = 0x004,
    FD_FLAGS_APPEND   = 0x008,
    FD_FLAGS_CREAT    = 0x010,
    FD_FLAGS_TRUNC    = 0x020,
    FD_FLAGS_EXCL     = 0x040,
    FD_FLAGS_SYNC     = 0x080,
    FD_FLAGS_NDELAY   = 0x100,
    FD_FLAGS_NONBLOCK = 0x200,
    FD_FLAGS_NOCTTY   = 0x400,
} TargetFdFlags;

static int fd_flags_from_target(TargetFdFlags f)
{
    int ret = 0;

    if (f & FD_FLAGS_RDONLY) {
        ret |= O_RDONLY;
    }

    if (f & FD_FLAGS_WRONLY) {
        ret |= O_WRONLY;
    }

    if (f & FD_FLAGS_RDWR) {
        ret |= O_RDWR;
    }

    if (f & FD_FLAGS_APPEND) {
        ret |= O_APPEND;
    }

    if (f & FD_FLAGS_CREAT) {
        ret |= O_CREAT;
    }

    if (f & FD_FLAGS_TRUNC) {
        ret |= O_TRUNC;
    }

    if (f & FD_FLAGS_EXCL) {
        ret |= O_EXCL;
    }

    if (f & FD_FLAGS_SYNC) {
        ret |= O_SYNC;
    }

    if (f & FD_FLAGS_NDELAY) {
        ret |= O_NDELAY;
    }

    if (f & FD_FLAGS_NONBLOCK) {
        ret |= O_NONBLOCK;
    }

    if (f & FD_FLAGS_NOCTTY) {
        ret |= O_NOCTTY;
    }

    return ret;
}

static TargetFdFlags fd_flags_to_target(int f)
{
    int ret = 0;

    if (f & O_RDONLY) {
        ret |= FD_FLAGS_RDONLY;
    }

    if (f & O_WRONLY) {
        ret |= FD_FLAGS_WRONLY;
    }

    if (f & O_RDWR) {
        ret |= FD_FLAGS_RDWR;
    }

    if (f & O_APPEND) {
        ret |= FD_FLAGS_APPEND;
    }

    if (f & O_CREAT) {
        ret |= FD_FLAGS_CREAT;
    }

    if (f & O_TRUNC) {
        ret |= FD_FLAGS_TRUNC;
    }

    if (f & O_EXCL) {
        ret |= FD_FLAGS_EXCL;
    }

    if (f & O_SYNC) {
        ret |= FD_FLAGS_SYNC;
    }

    if (f & O_NDELAY) {
        ret |= FD_FLAGS_NDELAY;
    }

    if (f & O_NONBLOCK) {
        ret |= FD_FLAGS_NONBLOCK;
    }

    if (f & O_NOCTTY) {
        ret |= FD_FLAGS_NOCTTY;
    }

    return ret;
}

static inline uint64_t cpu_to_le(uint64_t v, size_t sz)
{
    switch (sz) {
    case 1:
        return v;
    case 2:
        return cpu_to_le16(v);
    case 4:
        return cpu_to_le32(v);
    case 8:
        return cpu_to_le64(v);
    default:
        g_assert_not_reached();
    }
}

#define CPU_TO_LE(v) (cpu_to_le((v), sizeof(v)))

static inline void stat_to_target(const struct stat *in, uint64_t out[13])
{
    out[0] = CPU_TO_LE(in->st_dev);
    out[1] = CPU_TO_LE(in->st_ino);
    out[2] = CPU_TO_LE(in->st_mode);
    out[3] = CPU_TO_LE(in->st_nlink);
    out[4] = CPU_TO_LE(in->st_uid);
    out[5] = CPU_TO_LE(in->st_gid);
    out[6] = CPU_TO_LE(in->st_rdev);
    out[7] = CPU_TO_LE(in->st_size);
    out[8] = CPU_TO_LE(in->st_blksize);
    out[9] = CPU_TO_LE(in->st_blocks);
    out[10] = CPU_TO_LE(in->st_atim.tv_sec);
    out[11] = CPU_TO_LE(in->st_mtim.tv_sec);
    out[12] = CPU_TO_LE(in->st_ctim.tv_sec);
}

static char *target_read_str(CPUKVXState *cpu, target_ulong addr)
{
    const size_t CHUNK_SZ = 4096;

    char *ret = NULL;
    size_t sz = 0, cur = 0;

    do {
        int err;

        if (sz <= cur) {
            sz += CHUNK_SZ;
            ret = g_realloc(ret, sz);
        }

        err = cpu_memory_rw_debug(env_cpu(cpu), addr + cur,
                                  ret + cur, 1, false);
        cur++;

        if (err) {
            g_free(ret);
            return NULL;
        }

    } while(ret[cur - 1] != '\0');

    return ret;
}

static int do_open(CPUKVXState *cpu, target_ulong path_addr,
                   TargetFdFlags target_flags, mode_t mode)
{
    char *buf = target_read_str(cpu, path_addr);
    ssize_t ret;
    int flags;

    if (buf == NULL) {
        return -EFAULT;
    }

    flags = fd_flags_from_target(target_flags);
    ret = open(buf, flags, mode);

    g_free(buf);
    return ret;
}

static ssize_t do_read(CPUKVXState *cpu, int fd,
                       target_ulong addr, size_t count)
{
    uint8_t *buf = g_malloc(count);
    ssize_t ret;

    ret = read(fd, buf, count);

    if (ret < 0) {
        ret = -errno;
        goto out;
    }

    if (ret > 0) {
        if (cpu_memory_rw_debug(env_cpu(cpu), addr, buf, ret, true) == -1) {
            ret = -EFAULT;
        }
    }

out:
    g_free(buf);
    return ret;
}

static ssize_t do_write(CPUKVXState *cpu, int fd,
                       target_ulong addr, size_t count)
{
    uint8_t *buf = g_malloc(count);
    ssize_t ret;

    if (cpu_memory_rw_debug(env_cpu(cpu), addr, buf, count, false) == -1) {
        return -EFAULT;
    }

    ret = write(fd, buf, count);

    g_free(buf);
    return ret;
}

static int do_fstat(CPUKVXState *cpu, int fd, target_ulong statbuf_addr)
{
    uint64_t target_statbuf[13];

    struct stat statbuf;
    int ret;

    ret = fstat(fd, &statbuf);
    stat_to_target(&statbuf, target_statbuf);

    if (cpu_memory_rw_debug(env_cpu(cpu), statbuf_addr, target_statbuf,
                            sizeof(target_statbuf), true) == -1) {
        return -EFAULT;
    }

    return ret;
}

static int do_access(CPUKVXState *cpu, target_ulong path_addr, int mode)
{
    char *path = target_read_str(cpu, path_addr);
    int ret;

    if (path == NULL) {
        return -EFAULT;
    }

    ret = access(path, mode);

    g_free(path);

    if (ret == -1) {
        return -errno;
    }

    return ret;
}

static int do_chdir(CPUKVXState *cpu, target_ulong path_addr)
{
    char *path = target_read_str(cpu, path_addr);
    int ret;

    if (path == NULL) {
        return -EFAULT;
    }

    ret = chdir(path);

    g_free(path);

    if (ret == -1) {
        return -errno;
    }

    return ret;
}

static int do_chmod(CPUKVXState *cpu, target_ulong path_addr, mode_t mode)
{
    char *path = target_read_str(cpu, path_addr);
    int ret;

    if (path == NULL) {
        return -EFAULT;
    }

    ret = chmod(path, mode);

    g_free(path);

    if (ret == -1) {
        return -errno;
    }

    return ret;
}

static int do_fcntl(int fd, int command, uint64_t arg)
{
    int flags = 0;
    int ret;

    if (command == F_SETFL) {
        flags = fd_flags_from_target(arg);
    }

    ret = fcntl(fd, command, flags);

    if (command == F_GETFL) {
        ret = fd_flags_to_target(ret);
    }

    if (ret == -1) {
        return -errno;
    }

    return ret;
}

static int do_link(CPUKVXState *cpu, target_ulong old_addr, target_ulong new_addr)
{
    char *old, *new;
    int ret;

    old = target_read_str(cpu, old_addr);
    if (old == NULL) {
        ret = -EFAULT;
        goto old_err;
    }

    new = target_read_str(cpu, new_addr);
    if (new == NULL) {
        ret = -EFAULT;
        goto new_err;
    }

    ret = link(old, new);

    if (ret == -1) {
        ret = -errno;
    }

    g_free(new);

new_err:
    g_free(old);

old_err:
    return ret;
}

static int do_mkdir(CPUKVXState *cpu, target_ulong path_addr, mode_t mode)
{
    char *path = target_read_str(cpu, path_addr);
    int ret;

    if (path == NULL) {
        return -EFAULT;
    }

    ret = mkdir(path, mode);

    if (ret == -1) {
        ret = -errno;
    }

    return ret;
}

static int do_mkfifo(CPUKVXState *cpu, target_ulong path_addr, mode_t mode)
{
    char *path = target_read_str(cpu, path_addr);
    int ret;

    if (path == NULL) {
        return -EFAULT;
    }

    ret = mkfifo(path, mode);

    if (ret == -1) {
        ret = -errno;
    }

    return ret;
}

static int do_rmdir(CPUKVXState *cpu, target_ulong path_addr)
{
    char *path = target_read_str(cpu, path_addr);
    int ret;

    if (path == NULL) {
        return -EFAULT;
    }

    ret = rmdir(path);

    if (ret == -1) {
        ret = -errno;
    }

    return ret;
}

static int do_stat(CPUKVXState *cpu, target_ulong path_addr,
                   target_ulong statbuf_addr)
{
    char *path;
    uint64_t target_statbuf[13];
    struct stat statbuf;
    int ret;

    path = target_read_str(cpu, path_addr);

    if (path == NULL) {
        return -EFAULT;
    }

    ret = stat(path, &statbuf);
    stat_to_target(&statbuf, target_statbuf);

    g_free(path);

    if (cpu_memory_rw_debug(env_cpu(cpu), statbuf_addr, target_statbuf,
                            sizeof(target_statbuf), true) == -1) {
        return -EFAULT;
    }

    return ret;
}

#define ARG(i) arg ## i = le64_to_cpu(kvx_register_read_u64(env, REG_kv3_R ## i))

#define ARG_1 ARG(0)
#define ARG_2 ARG_1; ARG(1)
#define ARG_3 ARG_2; ARG(2)
#define ARGS(i) do { ARG_ ## i; } while (0)

#define RET(val) kvx_register_write_u64(env, REG_kv3_R0, cpu_to_le64(val))
#define RET_ERRNO(val) do {                                           \
    uint64_t val_ = (val);                                            \
    if (val_ == -1) {                                                 \
        kvx_register_write_u64(env, REG_kv3_R0, cpu_to_le64(-errno)); \
    } else {                                                          \
        kvx_register_write_u64(env, REG_kv3_R0, cpu_to_le64(val_));   \
    }                                                                 \
} while (0)


void kvx_do_semihosting(CPUKVXState *env, uint64_t scall)
{
    uint64_t arg0, arg1, arg2;

    trace_kvx_semihosting_syscall(scall);

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

    case SCALL_open:
        ARGS(3);
        RET(do_open(env, arg0, arg1, arg2));
        break;

    case SCALL_close:
        ARGS(1);

        if (arg0 < 3) {
            /* Avoid having the guest closing stdin, stdout and stderr */
            RET(0);
            break;
        }

        RET(close(arg0));
        break;

    case SCALL_read:
        ARGS(3);
        RET(do_read(env, arg0, arg1, arg2));
        break;

    case SCALL_write:
        ARGS(3);
        RET(do_write(env, arg0, arg1, arg2));
        break;

    case SCALL_fstat:
        ARGS(2);
        RET(do_fstat(env, arg0, arg1));
        break;

    case SCALL_lseek:
        ARGS(3);
        RET_ERRNO(lseek(arg0, arg1, arg2));
        break;

    case SCALL_access:
        ARGS(2);
        RET(do_access(env, arg0, arg1));
        break;

    case SCALL_chdir:
        ARGS(1);
        RET(do_chdir(env, arg0));
        break;

    case SCALL_chmod:
        ARGS(2);
        RET(do_chmod(env, arg0, arg1));
        break;

    case SCALL_dup:
        ARGS(1);
        RET_ERRNO(dup(arg0));
        break;

    case SCALL_dup2:
        ARGS(2);
        RET_ERRNO(dup2(arg0, arg1));
        break;

    case SCALL_fcntl:
        ARGS(3);
        RET(do_fcntl(arg0, arg1, arg2));
        break;

    case SCALL_isatty:
        ARGS(1);
        RET_ERRNO(isatty(arg0));
        break;

    case SCALL_link:
        ARGS(2);
        RET(do_link(env, arg0, arg1));
        break;

    case SCALL_mkdir:
        ARGS(2);
        RET(do_mkdir(env, arg0, arg1));
        break;

    case SCALL_mkfifo:
        ARGS(2);
        RET(do_mkfifo(env, arg0, arg1));
        break;

    case SCALL_rmdir:
        ARGS(1);
        RET(do_rmdir(env, arg0));
        break;

    case SCALL_stat:
        ARGS(2);
        RET(do_stat(env, arg0, arg1));
        break;

    default:
        trace_kvx_semihosting_unimp_syscall(scall);
        break;
    }
}
