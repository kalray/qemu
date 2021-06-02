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

#ifndef KVX_CPU_BITS_H
#define KVX_CPU_BITS_H

#include "qemu/units.h"
#include "qemu/bitops.h"

#define KVX_ADDRESS_LENGTH TARGET_VIRT_ADDR_SPACE_BITS
#define FIRST_SEMIHOSTED_SYSCALL 2049

#define KVX_NUM_IRQ 32
#define KVX_NUM_IPE 16

#define KVX_IRQ_TIMER0     0
#define KVX_IRQ_TIMER1     1
#define KVX_IRQ_WATCHDOG   2
#define KVX_IRQ_ARITHMETIC 13
#define KVX_IRQ_IPI        24

/* Bits in CS and CSIT corresponding to an arithmetic IRQ */
#define KVX_CSIT_IRQ_MASK  0x00003e3f

typedef enum KvxCsitAecValue {
    CSIT_AEC_IC = 0,
    CSIT_AEC_IO = 1,
    CSIT_AEC_DZ = 2,
    CSIT_AEC_OV = 3,
    CSIT_AEC_UN = 4,
    CSIT_AEC_IN = 5,
} KvxCsitAecValue;

/* Exception classes */
#define KVX_EXCP_DEBUG           0
#define KVX_EXCP_HW_TRAP         1
#define KVX_EXCP_INT             2
#define KVX_EXCP_SYSCALL         3
#define KVX_EXCP_WATCHDOG        8
#define KVX_EXCP_DOUBLE          8
#define KVX_EXCP_DOUBLE_HW_TRAP  9
#define KVX_EXCP_DOUBLE_INT      10
#define KVX_EXCP_DOUBLE_SYSCALL  11

/* Debug Exception Causes */
#define DEBUG_BREAKPOINT 0
#define DEBUG_WATCHPOINT 1
#define DEBUG_STEP       2
#define DEBUG_DSUBREAK   3

/* HW Trap Exception Causes */
#define TRAP_RESERVED          0
#define TRAP_OPCODE            1
#define TRAP_PRIVILEGE         2
#define TRAP_DMISALIGN         3
#define TRAP_PSYSERROR         4
#define TRAP_DSYSERROR         5
#define TRAP_PDECC             6
#define TRAP_DDECC             7
#define TRAP_PPAR              8
#define TRAP_DPAR              9
#define TRAP_PSECC             10
#define TRAP_DSECC             11
#define TRAP_NOMAPPING         12
#define TRAP_PROTECTION        13
#define TRAP_WRITETOCLEAN      14
#define TRAP_ATOMICTOCLEAN     15
#define TRAP_TPAR              16
#define TRAP_DOUBLE_EXCEPTION  17
#define TRAP_VSFR              18
#define TRAP_PL_OVERFLOW       19

#define TRAP_OPCODE_OR_VSFR(sfr_idx) ((sfr_idx) < 256 ? TRAP_OPCODE : TRAP_VSFR)

/* Exception Syndrome SFRI values */
#define ES_SFRI_INVALID 0
#define ES_SFRI_GET     2
#define ES_SFRI_IGET    3
#define ES_SFRI_SET     4
#define ES_SFRI_WFXL    5
#define ES_SFRI_WFXM    6
#define ES_SFRI_RSWAP   7

/* Exception Syndrome PIC values */
#define ES_PIC_NONE      0
#define ES_PIC_TLBWRITE  1
#define ES_PIC_TLBREAD   2
#define ES_PIC_TLBPROBE  3
#define ES_PIC_RFE       4
#define ES_PIC_STOP      5
#define ES_PIC_SYNCGROUP 6

/*
 * -------------------
 * ----- TLB/MMU -----
 * -------------------
 */

/* Size of the TLBs */
#define LTLB_WAYS 16
#define JTLB_WAYS 4
#define JTLB_SETS 64

/* TLB entry status */
#define TLB_ES_INVALID      0
#define TLB_ES_PRESENT      1
#define TLB_ES_MODIFIED     2
#define TLB_ES_A_MODIFIED   3

/* LTB entry cache policy */
#define CP_D_DEVICE_I_UNCACHED      0
#define CP_D_UNCACHED_I_UNCACHED    1
#define CP_D_WRITE_THROUGH_I_CACHED 2
#define CP_D_UNCACHED_I_CACHED      3

/*
 * Declare those as defines to avoid GCC version < 8
 * "initializer element is not constant" compilation error.
 */
#define TLB_PS_4KB_SHIFT   12
#define TLB_PS_64KB_SHIFT  16
#define TLB_PS_2MB_SHIFT   21
#define TLB_PS_512MB_SHIFT 29

typedef enum TLBPageSize {
    TLB_PS_4KB = 0,
    TLB_PS_64KB = 1,
    TLB_PS_2MB = 2,
    TLB_PS_512MB = 3,
} TLBPageSize;

static const int MMU_PAGE_SHIFT[] = {
    [TLB_PS_4KB]   = TLB_PS_4KB_SHIFT,
    [TLB_PS_64KB]  = TLB_PS_64KB_SHIFT,
    [TLB_PS_2MB]   = TLB_PS_2MB_SHIFT,
    [TLB_PS_512MB] = TLB_PS_512MB_SHIFT,
};

static const int MMU_PAGE_SIZE[] = {
    [TLB_PS_4KB]   = 1 << TLB_PS_4KB_SHIFT,
    [TLB_PS_64KB]  = 1 << TLB_PS_64KB_SHIFT,
    [TLB_PS_2MB]   = 1 << TLB_PS_2MB_SHIFT,
    [TLB_PS_512MB] = 1 << TLB_PS_512MB_SHIFT,
};

static const target_ulong MMU_PAGE_SIZE_MASK[] = {
    [TLB_PS_4KB]   = ~MAKE_64BIT_MASK(0, TLB_PS_4KB_SHIFT),
    [TLB_PS_64KB]  = ~MAKE_64BIT_MASK(0, TLB_PS_64KB_SHIFT),
    [TLB_PS_2MB]   = ~MAKE_64BIT_MASK(0, TLB_PS_2MB_SHIFT),
    [TLB_PS_512MB] = ~MAKE_64BIT_MASK(0, TLB_PS_512MB_SHIFT),
};

#define PA_NA   0
#define PA_R    (PROT_READ)
#define PA_W    (PROT_WRITE)
#define PA_X    (PROT_EXEC)
#define PA_RW   (PA_R | PA_W)
#define PA_RX   (PA_R | PA_X)
#define PA_RWX  (PA_R | PA_W | PA_X)

static const int * const MMU_PROTECTION_ATTRS[2] = {
    /* MMUP == 0 */
    [0] = (const int []) {
        PA_NA, PA_NA, PA_NA, PA_NA, PA_NA, PA_R, PA_R, PA_R,
        PA_R, PA_RW, PA_RW, PA_RX, PA_RX, PA_RWX,

        /* Invalid values treated as no access */
        PA_NA, PA_NA
    },

    /* MMUP == 1 */
    [1] = (const int []) {
        PA_NA, PA_R, PA_RW, PA_RX, PA_RWX, PA_R, PA_RW, PA_RX,
        PA_RWX, PA_RW, PA_RWX, PA_RX, PA_RWX, PA_RWX,

        /* Invalid values treated as no access */
        PA_NA, PA_NA
    }
};

#undef PA_RWX
#undef PA_RX
#undef PA_RW
#undef PA_X
#undef PA_W
#undef PA_R
#undef PA_NA

typedef enum ExceptionSyndromeRWXValue {
    ES_RWX_NOT_MEM_TRAP = 0,
    ES_RWX_FETCH_SIDE = 1,
    ES_RWX_DATA_SIDE_WRITE = 2,
    ES_RWX_DATA_SIDE_READ = 4,
    ES_RWX_DATA_SIDE_ATOMIC = 6,
} ExceptionSyndromeRWXValue;
#endif
