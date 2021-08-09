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

#ifndef KVX_REG_INFO
#define KVX_REG_INFO

typedef struct RegisterInfo {
    /* Access does io-related things (and so need iothread mutex) */
    bool io;

    /* Register is an alias depending on PL */
    bool alias_pl;

    /* Access to this register requires a minimum privileged level */
    bool check_pl;

    /* We need to re-evaluate pending irqs after a write */
    bool it;

    /*
     * if true the register is only 32 bits when PS.V64 is cleared. Only
     * checked if addr_store_mask is not 0.
     */
    bool v64_aware;

    /* Required privileged level (significant when check_pl is true) */
    int required_pl;

    /*
     * Additionnal mask to apply on store. When not zero, the register is
     * assumed to contain an address. On store, the value is sign extended to
     * KVX_ADDRESS_LENGTH bits and the mask is applied.
     */
    uint64_t addr_store_mask;

    /*
     * Read and write functions (used for any read/write, including those
     * which come from gdbstub)
     */
    uint64_t (*read)(KVXCPU*, Register);
    void (*write)(KVXCPU*, Register, uint64_t);

    /*
     * For register with PL field(s) that use relative PL writing.
     * Takes:
     *  - a pointer to the new value (after any wfx computation) to be
     *    written
     *  - the mask telling the modified part.
     * The value is modified according to the current PL (unmasked
     * bit may be modified too).
     * Returns true if the relative PL fits, false if it overflows.
     */
    bool (*apply_rpl)(CPUKVXState *, Register, uint64_t *, uint64_t);
} RegisterInfo;

void kvx_reg_write_simple(KVXCPU *cpu, Register reg, uint64_t val);

void kvx_reg_write_csit(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_lc_le(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_ps(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_sps(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_mmc(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_ilr(KVXCPU *cpu, Register reg, uint64_t val);
uint64_t kvx_reg_read_ilr(KVXCPU *cpu, Register reg);

void kvx_reg_write_shouldnothappen(KVXCPU *cpu, Register reg, uint64_t val);
uint64_t kvx_reg_read_shouldnothappen(KVXCPU *cpu, Register reg);

bool kvx_reg_apply_rpl_ps(CPUKVXState *env, Register reg, uint64_t *val,
                          uint64_t mask);
bool kvx_reg_apply_rpl_owners(CPUKVXState *env, Register reg, uint64_t *val,
                              uint64_t mask);

void kvx_reg_write_tcr(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_txv_wdv(KVXCPU *cpu, Register reg, uint64_t val);
uint64_t kvx_reg_read_tcr_txv_wdv(KVXCPU *cpu, Register reg);

void kvx_reg_write_dc(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_dba(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_dwa(KVXCPU *cpu, Register reg, uint64_t val);

#define STORE_MASK_EA 0xffffffffffffffffull
#define STORE_MASK_PC 0xfffffffffffffffcull
#define STORE_MASK_EV 0xffffffffffffff00ull

void kvx_reg_write_pmctl(KVXCPU *cpu, Register reg, uint64_t val);
void kvx_reg_write_pmcnt(KVXCPU *cpu, Register reg, uint64_t val);
uint64_t kvx_reg_read_pmcnt(KVXCPU *cpu, Register reg);

static const RegisterInfo REG_INFO[] = {
    /* Program Counter */
    [REG_kv3_PC] = {
        .addr_store_mask = STORE_MASK_PC,
        .v64_aware = true,
    },

    /* Processing Status */
    [REG_kv3_PS] = {
        .it = true,
        .write = kvx_reg_write_ps,
        .apply_rpl = kvx_reg_apply_rpl_ps,
    },

    /* Return Address */
    [REG_kv3_RA] = {
        .addr_store_mask = STORE_MASK_PC,
    },

    /* Compute status interrupt */
    [REG_kv3_CSIT] = {
        .write = kvx_reg_write_csit,
    },

    /* Arithmetic Exception Saved PC */
    [REG_kv3_AESPC] = {
        .addr_store_mask = STORE_MASK_PC,
    },

    /* Hardware Loop registers (start, end) */
    [REG_kv3_LS] = {
        .addr_store_mask = STORE_MASK_PC,
    },
    [REG_kv3_LE] = {
        .addr_store_mask = STORE_MASK_PC,
        .write = kvx_reg_write_lc_le,
    },

    /* Performance counters registers */
    [REG_kv3_PMC] = {
        .io = true,
        .write = kvx_reg_write_pmctl,
    },
    [REG_kv3_PM0] = {
        .io = true,
        .read = kvx_reg_read_pmcnt,
        .write = kvx_reg_write_pmcnt,
    },
    [REG_kv3_PM1] = {
        .io = true,
        .read = kvx_reg_read_pmcnt,
        .write = kvx_reg_write_pmcnt,
    },
    [REG_kv3_PM2] = {
        .io = true,
        .read = kvx_reg_read_pmcnt,
        .write = kvx_reg_write_pmcnt,
    },
    [REG_kv3_PM3] = {
        .io = true,
        .read = kvx_reg_read_pmcnt,
        .write = kvx_reg_write_pmcnt,
    },

    /* Timer registers */
    [REG_kv3_TCR] = {
        .io = true,
        .read = kvx_reg_read_tcr_txv_wdv,
        .write = kvx_reg_write_tcr,
    },
    [REG_kv3_T0V] = {
        .io = true,
        .read = kvx_reg_read_tcr_txv_wdv,
        .write = kvx_reg_write_txv_wdv,
    },
    [REG_kv3_T1V] = {
        .io = true,
        .read = kvx_reg_read_tcr_txv_wdv,
        .write = kvx_reg_write_txv_wdv,
    },
    [REG_kv3_WDV] = {
        .io = true,
        .read = kvx_reg_read_tcr_txv_wdv,
        .write = kvx_reg_write_txv_wdv,
    },

    /* Interrupt Line Enable, Level, Request */
    [REG_kv3_ILE] = {
        .it = true,
    },
    [REG_kv3_ILL] = {
        .it = true,
    },
    [REG_kv3_ILR] = {
        .io = true, /* this register is protected by mutex_iothread */
        .it = true,
        .write = kvx_reg_write_ilr,
        .read = kvx_reg_read_ilr,
    },

    /* Memory Translation register */
    [REG_kv3_MMC] = {
        .write = kvx_reg_write_mmc,
    },

    /* Debug Control */
    [REG_kv3_DC] = {
        .write = kvx_reg_write_dc,
    },

    /* Debug Breakpoint Address 0 & 1 */
    [REG_kv3_DBA0] = {
        .write = kvx_reg_write_dba,
    },
    [REG_kv3_DBA1] = {
        .write = kvx_reg_write_dba,
    },

    /* Debug Watchpoint Address 0 & 1 */
    [REG_kv3_DWA0] = {
        .write = kvx_reg_write_dwa,
    },
    [REG_kv3_DWA1] = {
        .write = kvx_reg_write_dwa,
    },

    /* Write aliases to read-only Owners registers */
    [REG_kv3_SYOW] = {
        .apply_rpl = kvx_reg_apply_rpl_owners,
    },
    [REG_kv3_HTOW] = {
        .apply_rpl = kvx_reg_apply_rpl_owners,
    },
    [REG_kv3_ITOW] = {
        .it = true,
        .apply_rpl = kvx_reg_apply_rpl_owners,
    },
    [REG_kv3_DOW] = {
        .apply_rpl = kvx_reg_apply_rpl_owners,
    },
    [REG_kv3_MOW] = {
        .apply_rpl = kvx_reg_apply_rpl_owners,
    },
    [REG_kv3_PSOW] = {
        .apply_rpl = kvx_reg_apply_rpl_owners,
        /* dummy write handler to force ending tb and recompute tb_state */
        .write = kvx_reg_write_simple,
    },

    /*
     * Alias registers
     */
    [REG_kv3_SPC] = {
        .alias_pl = true,
        .read = kvx_reg_read_shouldnothappen,
        .write = kvx_reg_write_shouldnothappen,
    },
    [REG_kv3_SPS] = {
        .alias_pl = true,
        .read = kvx_reg_read_shouldnothappen,
        .write = kvx_reg_write_shouldnothappen,
    },
    [REG_kv3_EA] = {
        .alias_pl = true,
        .read = kvx_reg_read_shouldnothappen,
        .write = kvx_reg_write_shouldnothappen,
    },
    [REG_kv3_EV] = {
        .alias_pl = true,
        .read = kvx_reg_read_shouldnothappen,
        .write = kvx_reg_write_shouldnothappen,
    },
    [REG_kv3_SR] = {
        .alias_pl = true,
        .read = kvx_reg_read_shouldnothappen,
        .write = kvx_reg_write_shouldnothappen,
    },
    [REG_kv3_ES] = {
        .alias_pl = true,
        .read = kvx_reg_read_shouldnothappen,
        .write = kvx_reg_write_shouldnothappen,
    },

    /* Shadow Program counter */
    [REG_kv3_SPC_PL0] = {
        .check_pl = true,
        .required_pl = 0,
    },
    [REG_kv3_SPC_PL1] = {
        .check_pl = true,
        .required_pl = 1,
    },
    [REG_kv3_SPC_PL2] = {
        .check_pl = true,
        .required_pl = 2,
    },

    /* Shadow Processing Status */
    [REG_kv3_SPS_PL0] = {
        .check_pl = true,
        .required_pl = 0,
        .write = kvx_reg_write_sps,
    },
    [REG_kv3_SPS_PL1] = {
        .check_pl = true,
        .required_pl = 1,
        .write = kvx_reg_write_sps,
    },
    [REG_kv3_SPS_PL2] = {
        .check_pl = true,
        .required_pl = 2,
        .write = kvx_reg_write_sps,
    },
    [REG_kv3_SPS_PL3] = {
        .write = kvx_reg_write_sps,
    },

    /* Exception Vector */
    [REG_kv3_EV_PL0] = {
        .addr_store_mask = STORE_MASK_EV,
        .check_pl = true,
        .required_pl = 0,
    },
    [REG_kv3_EV_PL1] = {
        .addr_store_mask = STORE_MASK_EV,
        .check_pl = true,
        .required_pl = 1,
    },
    [REG_kv3_EV_PL2] = {
        .addr_store_mask = STORE_MASK_EV,
        .check_pl = true,
        .required_pl = 2,
    },
    [REG_kv3_EV_PL3] = {
        .addr_store_mask = STORE_MASK_EV,
    },

    /* Exception Address */
    [REG_kv3_EA_PL0] = {
        .addr_store_mask = STORE_MASK_EA,
        .check_pl = true,
        .required_pl = 0,
    },
    [REG_kv3_EA_PL1] = {
        .addr_store_mask = STORE_MASK_EA,
        .check_pl = true,
        .required_pl = 1,
    },
    [REG_kv3_EA_PL2] = {
        .addr_store_mask = STORE_MASK_EA,
        .check_pl = true,
        .required_pl = 2,
    },
    [REG_kv3_EA_PL3] = {
        .addr_store_mask = STORE_MASK_EA,
    },

    /* System-Reserved */
    [REG_kv3_SR_PL0] = {
        .check_pl = true,
        .required_pl = 0,
    },
    [REG_kv3_SR_PL1] = {
        .check_pl = true,
        .required_pl = 1,
    },
    [REG_kv3_SR_PL2] = {
        .check_pl = true,
        .required_pl = 2,
    },

    /* Exception Syndrome */
    [REG_kv3_ES_PL0] = {
        .check_pl = true,
        .required_pl = 0,
    },
    [REG_kv3_ES_PL1] = {
        .check_pl = true,
        .required_pl = 1,
    },
    [REG_kv3_ES_PL2] = {
        .check_pl = true,
        .required_pl = 2,
    },
};

static inline const RegisterInfo *kvx_register_info(Register reg)
{
    static const RegisterInfo default_regi = {
        .io = false,
        .alias_pl = false,
        .read = NULL,
        .write = NULL,
        .addr_store_mask = 0,
        .v64_aware = false,
        .check_pl = false,
    };

    if (reg < (sizeof(REG_INFO) / sizeof(RegisterInfo))) {
        return &REG_INFO[reg];
    }

    return &default_regi;
}

static inline Register kvx_register_alias_pl(Register reg, int pl)
{
    if (!kvx_register_info(reg)->alias_pl) {
        return reg;
    }
#define RETURN_ALIAS_REG(reg_, pl_) { \
    g_assert(REG_kv3_ ## reg_ ## _PL1 == (REG_kv3_ ## reg_ ## _PL0 + 1)); \
    g_assert(REG_kv3_ ## reg_ ## _PL2 == (REG_kv3_ ## reg_ ## _PL0 + 2)); \
    g_assert(REG_kv3_ ## reg_ ## _PL3 == (REG_kv3_ ## reg_ ## _PL0 + 3)); \
    return REG_kv3_##reg_##_PL0 + pl_; \
}
    switch (reg) {
    case REG_kv3_EV:
        RETURN_ALIAS_REG(EV, pl)
    case REG_kv3_EA:
        RETURN_ALIAS_REG(EA, pl)
    case REG_kv3_ES:
        RETURN_ALIAS_REG(ES, pl)
    case REG_kv3_SR:
        RETURN_ALIAS_REG(SR, pl)
    case REG_kv3_SPS:
        RETURN_ALIAS_REG(SPS, pl)
    case REG_kv3_SPC:
        RETURN_ALIAS_REG(SPC, pl)
    default:
        g_assert_not_reached();
    }
#undef RETURN_ALIAS_REG
}

#define REG_kv3_EV_PLx(pl) kvx_register_alias_pl(REG_kv3_EV, pl)
#define REG_kv3_EA_PLx(pl) kvx_register_alias_pl(REG_kv3_EA, pl)
#define REG_kv3_ES_PLx(pl) kvx_register_alias_pl(REG_kv3_ES, pl)
#define REG_kv3_SR_PLx(pl) kvx_register_alias_pl(REG_kv3_SR, pl)
#define REG_kv3_SPS_PLx(pl) kvx_register_alias_pl(REG_kv3_SPS, pl)
#define REG_kv3_SPC_PLx(pl) kvx_register_alias_pl(REG_kv3_SPC, pl)

#define kv3_PS_FOE01_MASK (KVX_FIELD_MASK(kv3_PS, ET) | \
                           KVX_FIELD_MASK(kv3_PS, IE) | \
                           KVX_FIELD_MASK(kv3_PS, HLE) )

#endif
