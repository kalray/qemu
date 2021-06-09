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
#include "qemu/log.h"
#include "cpu.h"
#include "qemu/main-loop.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "internal.h"

void HELPER(set_excp_address)(CPUKVXState *env, uint64_t value)
{
    env->excp_address = value;
}

void HELPER(update_excp_syndrome)(CPUKVXState *env, uint64_t value,
                                  uint64_t mask)
{
    env->excp_syndrome = (env->excp_syndrome & ~mask) | (value & mask);
}

void HELPER(raise_exception)(CPUKVXState *env, uint32_t exception)
{
    kvx_raise_exception(env, exception, 0);
}

void HELPER(raise_trap)(CPUKVXState *env, uint32_t trap, uint64_t syndrome)
{
    env->excp_syndrome = KVX_FIELD_DP64(syndrome, kv3_ES, HTC, trap);

    HELPER(raise_exception)(env, KVX_EXCP_HW_TRAP);
}

void HELPER(raise_syscall)(CPUKVXState *env, uint64_t scall_num)
{
    env->excp_syndrome = KVX_FIELD_DP64(0, kv3_ES, SN, scall_num);

    HELPER(raise_exception)(env, KVX_EXCP_SYSCALL);
}

void HELPER(raise_syscall_semi)(CPUKVXState *env, uint64_t scall_num,
                                uint64_t pc)
{
    if (scall_num >= FIRST_SEMIHOSTED_SYSCALL) {
        kvx_do_semihosting(env, scall_num);
        kvx_register_write_u64(env, REG_kv3_PC, pc + 4);
        return;
    }

    kvx_register_write_u64(env, REG_kv3_PC, pc);
    HELPER(raise_syscall)(env, scall_num);
}

static void raise_privilege_trap(CPUKVXState *env, int trap_pl, uint64_t syndrome)
{
    env->excp_target_pl = trap_pl;
    HELPER(raise_trap)(env, TRAP_PRIVILEGE, syndrome);
}

void HELPER(check_arith_irq)(CPUKVXState *env, uint64_t cs_mask, uint64_t pc)
{
    static const int AEC_MAPPING[] = {
        [0] = CSIT_AEC_IC,
        [1] = CSIT_AEC_IO,
        [2] = CSIT_AEC_DZ,
        [3] = CSIT_AEC_OV,
        [4] = CSIT_AEC_UN,
        [5] = CSIT_AEC_IN,
        [9] = CSIT_AEC_IO,
        [10] = CSIT_AEC_DZ,
        [11] = CSIT_AEC_OV,
        [12] = CSIT_AEC_UN,
        [13] = CSIT_AEC_IN,
    };
    CPUState *cpu = env_cpu(env);
    uint64_t irq_bits, cs, csit;
    int aec = INT_MAX, i = 0;

    cs = kvx_register_read_u64(env, REG_kv3_CS);
    csit = kvx_register_read_u64(env, REG_kv3_CSIT);

    irq_bits = cs_mask & cs & csit;

    if (!irq_bits) {
        return;
    }

    /* Look for the smallest AEC value among the triggered arithmetic interrupts */
    while (irq_bits) {
        g_assert(i < ARRAY_SIZE(AEC_MAPPING));
        if (irq_bits & 1) {
            aec = (aec < AEC_MAPPING[i]) ? aec : AEC_MAPPING[i];
        }

        irq_bits >>= 1;
        i++;
    }

    csit = KVX_FIELD_DP64(csit, kv3_CSIT, AEIR, 1);
    csit = KVX_FIELD_DP64(csit, kv3_CSIT, AEC, aec);

    /*
     * Always consider PC is saved into AESPC. This is not always true on real
     * hardware.
     */
    csit = KVX_FIELD_DP64(csit, kv3_CSIT, SPCV, 1);

    kvx_register_write_u64(env, REG_kv3_CSIT, csit);
    kvx_register_write_u64(env, REG_kv3_AESPC, pc);

    qemu_irq_pulse(qdev_get_gpio_in(DEVICE(cpu), KVX_IRQ_ARITHMETIC));
}

static uint64_t test_owner(CPUKVXState *env, uint64_t syndrome, int owner_pl)
{
    int cur_pl = kvx_get_current_pl(env);
    if (cur_pl > owner_pl) {
        /* requested pl is not met */
        raise_privilege_trap(env, owner_pl, syndrome);
        return 0;
    }

    return 1;
}

uint64_t HELPER(test_stop_owner)(CPUKVXState *env, uint64_t syndrome)
{
    int owner_pl = kvx_register_read_field(env, MO, STOP);
    return test_owner(env, syndrome, owner_pl);
}

uint64_t HELPER(test_syncgroup_owner)(CPUKVXState *env, uint64_t syndrome)
{
    int owner_pl = kvx_register_read_field(env, MO, SYNC);
    return test_owner(env, syndrome, owner_pl);
}

uint64_t HELPER(test_mmi_owner)(CPUKVXState *env, uint64_t syndrome)
{
    int owner_pl = kvx_register_read_field(env, MO, MMI);
    return test_owner(env, syndrome, owner_pl);
}

uint64_t HELPER(test_rfe_owner)(CPUKVXState *env, uint64_t syndrome)
{
    int owner_pl = kvx_register_read_field(env, MO, RFE);
    return test_owner(env, syndrome, owner_pl);
}

void HELPER(rfe)(CPUKVXState *env, uint64_t syndrome)
{
    int cur_pl, target_pl;
    uint64_t cur_ps, new_ps;
    uint64_t cur_sps, new_sps;
    uint64_t new_pc;

    cur_pl = kvx_get_current_pl(env);
    cur_ps = kvx_register_read_u64(env, REG_kv3_PS);
    cur_sps = kvx_register_read_u64(env, REG_kv3_SPS_PLx(cur_pl));

    target_pl = cur_pl + KVX_FIELD_EX64(cur_sps, kv3_PS, PL);
    if (target_pl > 3) {
        HELPER(raise_trap)(env, TRAP_PL_OVERFLOW, syndrome);
        return;
    }

    /* PS <- SPS_PL<i> */
    new_ps = KVX_FIELD_DP64(cur_sps, kv3_PS, PL, target_pl);
    kvx_register_write_u64(env, REG_kv3_PS, new_ps);

    /*
     * SPS_PL<i> <- PS except for the FOE0/1 bits, SPS_PL<i>.IL and
     * SPS_PL<i>.PL fields that are left unchanged (back-up of the remnant
     * state of PL<i>, atomic with the preceding step, this is really a swap).
     */
    if (cur_pl != target_pl) {
        uint64_t mask = kv3_PS_FOE01_MASK | KVX_FIELD_MASK(kv3_PS, IL) |
                        KVX_FIELD_MASK(kv3_PS, PL);
        new_sps = cur_sps & mask;
        new_sps |= cur_ps & ~mask;
        kvx_register_write_u64(env, REG_kv3_SPS_PLx(cur_pl), new_sps);
    } else {
        new_sps = cur_sps;
    }

    /* PC <- SPC_PL<i> */
    new_pc = kvx_register_read_u64(env, REG_kv3_SPC_PLx(cur_pl));
    kvx_register_write_u64(env, REG_kv3_PC, new_pc);

    kvx_update_cpu_state(env, cur_ps ^ new_ps, cur_sps ^ new_sps);
}

void HELPER(idle)(CPUKVXState *env, uint64_t lvl)
{
    CPUState *cs = env_cpu(env);

    /* We already checked that the corresponding WS.WUx bit is cleared */
    env->sleep_state = KVX_AWAIT + lvl;
    cs->halted = true;
    cs->exception_index = EXCP_HLT;

    cpu_loop_exit(cs);
}

void HELPER(waitit)(CPUKVXState *env, uint64_t or_mask, uint64_t and_mask,
                    uint64_t operand)
{
    CPUState *cs = env_cpu(env);

    if (or_mask || and_mask) {
        env->waitit_ctx.or_mask = or_mask;
        env->waitit_ctx.and_mask = and_mask;
        env->waitit_ctx.dst_op = operand;

        env->sleep_state = KVX_WAITIT;
        cs->halted = true;
        cs->exception_index = EXCP_HLT;
    }

    cpu_loop_exit(cs);
}

void HELPER(syncgroup)(CPUKVXState *env, uint64_t clr_fwd, uint64_t clr_bwd,
                       uint64_t notify_fwd, uint64_t notify_bwd)
{
    CPUState *cs = env_cpu(env);
    uint64_t pc;

    if (clr_fwd || clr_bwd) {
        uint32_t wait_mask = (clr_bwd << KVX_NUM_IPE) | clr_fwd;
        uint64_t *ipe = kvx_register_ptr_u64(env, REG_kv3_IPE);

        if ((*ipe & wait_mask) == wait_mask) {
            /* events are already there. no need to go to sleep mode */
            /* clear the events */
            *ipe &= ~(uint64_t) wait_mask;
        } else {
            /* we must wait for some events to be set */
            env->syncgroup_ctx.wait_mask = wait_mask;
            env->syncgroup_ctx.notify_fwd = notify_fwd;
            env->syncgroup_ctx.notify_bwd = notify_bwd;

            env->sleep_state = KVX_SYNCGROUP;
            cs->halted = true;
            cs->exception_index = EXCP_HLT;

            cpu_loop_exit(cs);
        }
    }

    /* forward the events */
    kvx_send_ipe_events(env, notify_fwd, notify_bwd);

    /* skip the syncgroup instruction */
    pc = kvx_register_read_u64(env, REG_kv3_PC);
    kvx_register_write_u64(env, REG_kv3_PC, pc + 4);

    cpu_loop_exit(cs);
}

static uint64_t do_load(CPUKVXState *env, uint64_t address, MemOp op)
{
    uint64_t ret;
    int size = memop_size(op);

    switch (size) {
    case 1:
        ret = cpu_ldub_data(env, address);
        break;

    case 2:
        ret = cpu_lduw_data(env, address);
        break;

    case 4:
        ret = cpu_ldl_data(env, address);
        break;

    case 8:
        return cpu_ldq_data(env, address);

    default:
        g_assert_not_reached();
    }

    if (op & MO_SIGN) {
        size = 64 - (size * 8);
        ret = (((int64_t) ret) << size) >> size;
    }

    return ret;
}

static inline bool check_page(CPUKVXState *env,
                              vaddr addr, int size,
                              int *cp, uintptr_t retaddr)
{
    TLBLookupStatus mmu_res;
    TLBLookupInfo info;
    int mmu_idx = cpu_mmu_index(env, false);
    bool raise_fault = false;

    /*
     * Do a TLB lookup to know the page status. If the lookup fails, the
     * speculative load should write 0 to the destination register, unless SNE
     * or SPE are set.
     */
    mmu_res = get_physical_addr(env, addr, MMU_DATA_LOAD, mmu_idx, &info);

    if (mmu_res == TLB_LOOKUP_SUCCESS) {
        if (info.attrs.target_tlb_bit1) {
            /*
             * This page is mapped with the DEVICE data cache policy. In this
             * case speculative accesses do not reach memory and 0 is returned
             * to the core.
             */
            return false;
        }

        if (-(addr | (info.page_size - 1)) < size) {
            /*
             * The access spans over two pages. We have to check the second
             * page for valid mapping and matching cache policy.
             */
            int page1_cp;
            vaddr next_page = (addr & ~(info.page_size - 1)) + info.page_size;
            bool ret = check_page(env, next_page, size, &page1_cp, retaddr);

            if (!ret) {
                return false;
            }

            if (page1_cp != info.cache_policy) {
                return false;
            }
        }

        if (cp) {
            *cp = info.cache_policy;
        }

        /* do the load */
        return true;
    }

    if (kvx_register_read_field(env, MMC, SNE)
        && (info.fault == TRAP_NOMAPPING)) {
        raise_fault = true;
    }

    if (kvx_register_read_field(env, MMC, SPE)
        && (info.fault == TRAP_PROTECTION)) {
        raise_fault = true;
    }

    if (raise_fault) {
        size = MAX(size, 8);
        probe_access(env, addr, size, MMU_DATA_LOAD, mmu_idx, retaddr);
        g_assert_not_reached();
    }

    return false;
}

uint64_t HELPER(load_speculative)(CPUKVXState *env,
                                  uint64_t addr, uint64_t m_op)
{
    MemOp mem_op = m_op;

    if (!check_page(env, addr, memop_size(mem_op), NULL, GETPC())) {
        return 0;
    }

    return do_load(env, addr, mem_op);
}

void HELPER(load_speculative_multiple)(CPUKVXState *env,
                                       uint64_t addr, uint64_t size)
{
    int i;

    if (!check_page(env, addr, size, NULL, GETPC())) {
        memset(env->scratch, 0, sizeof(env->scratch));
        return;
    }

    /* Do the load */
    for (i = 0; i < size / 8; i++) {
        const MemOp mem_op = MO_TE | size_memop(8);
        env->scratch[i] = do_load(env, addr + (i * 8), mem_op);
    }
}

uint64_t HELPER(read_register)(CPUKVXState *env, uint64_t reg_enum)
{
    Register reg = reg_enum;
    KVXCPU *cpu = env_archcpu(env);
    const RegisterInfo *info = kvx_register_info(reg);
    uint64_t val;

    if (info->io) {
        qemu_mutex_lock_iothread();
    }

    val = kvx_register_read(cpu, reg);

    if (info->io) {
        qemu_mutex_unlock_iothread();
    }

    return val;
}

uint64_t HELPER(read_regfile_indirect)(CPUKVXState *env, uint64_t regfile_enum,
                                       uint64_t reg_idx)
{
    const RegisterFile *regfile = &REGFILE_MAPPING[regfile_enum];
    Register reg;

    g_assert(reg_idx < regfile->size);
    reg = regfile->registers[reg_idx];

    /*
     * in case of igetm PC has been flushed to its register
     * when checking access rights
     */
    return HELPER(read_register)(env, reg);
}

void HELPER(write_register)(CPUKVXState *env, uint64_t reg_enum,
                            uint64_t value)
{
    KVXCPU *cpu = env_archcpu(env);
    Register reg = reg_enum;
    const RegisterInfo *info = kvx_register_info(reg);

    /* TCG must not write PC this way */
    g_assert(reg != REG_kv3_PC);

    if (info->io) {
        qemu_mutex_lock_iothread();
    }

    kvx_register_write(cpu, reg, value);

    if (info->io) {
        qemu_mutex_unlock_iothread();
    }
}

uint64_t HELPER(apply_relative_pl)(CPUKVXState *env, uint32_t sfr_idx,
                                   uint64_t value, uint64_t mask,
                                   uint64_t syndrome)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr_idx];
    const RegisterInfo *info = kvx_register_info(reg);

    if (!info->apply_rpl(env, reg, &value, mask)) {
        HELPER(raise_trap)(env, TRAP_PL_OVERFLOW, syndrome);
    }
    return value;
}

static bool field_is_per_bit_pl(RegisterField field)
{
    switch (field) {
        case REGFIELD_kv3_PS_PL:
        case REGFIELD_kv3_PS_IL:
        case REGFIELD_kv3_PS_VS:
        case REGFIELD_kv3_SPS_PL:
        case REGFIELD_kv3_SPS_IL:
        case REGFIELD_kv3_SPS_VS:
        case REGFIELD_kv3_SPS_PL0_PL:
        case REGFIELD_kv3_SPS_PL0_IL:
        case REGFIELD_kv3_SPS_PL0_VS:
        case REGFIELD_kv3_SPS_PL1_PL:
        case REGFIELD_kv3_SPS_PL1_IL:
        case REGFIELD_kv3_SPS_PL1_VS:
        case REGFIELD_kv3_SPS_PL2_PL:
        case REGFIELD_kv3_SPS_PL2_IL:
        case REGFIELD_kv3_SPS_PL2_VS:
        case REGFIELD_kv3_SPS_PL3_PL:
        case REGFIELD_kv3_SPS_PL3_IL:
        case REGFIELD_kv3_SPS_PL3_VS:
            return true;
        default:
            return false;
    }
}

static int get_owner_pl(CPUKVXState *env, const RegisterFieldDescr *descr)
{
    int owner_pl;
    // the true owner is the most privileged level in all 'owners' field

    g_assert(REGISTERFIELDS[descr->owners[0]].width == 2);
    owner_pl = kvx_regfield_read(env, descr->owners[0]);

    for (int i = 1; i < descr->n_owners; i++) {
        int tmp = kvx_regfield_read(env, descr->owners[i]);
        g_assert(REGISTERFIELDS[descr->owners[i]].width == 2);
        if (tmp < owner_pl) {
            owner_pl = tmp;
        }
    }

    return owner_pl;
}

/*
 * Check if it's OK to read a field.
 *
 * mask: inout param, tells which bits of the register are read.
 *       If the field need to be masked-out for reading, bits will
 *       be be cleared in the mask.
 *
 * pl: tells what is the current PL level.
 *
 * Return -1 if the access is granted or the required PL level if the access
 *        does an unpriviledge access.
 */
static int check_field_read(CPUKVXState *env, RegisterField field,
                            int pl, uint64_t *mask)
{
    const RegisterFieldDescr *rfdescr = &REGISTERFIELDS[field];
    uint64_t field_mask = MAKE_64BIT_MASK(rfdescr->offset, rfdescr->width);
    int owner_pl;

    if (!rfdescr->n_owners || !(field_mask & *mask) ||
        rfdescr->rerror == REG_READ_ERROR_READ) {
        return -1;
    }

    if (field_is_per_bit_pl(field)) {
        //per_bit_pl field are in READ mode
        g_assert_not_reached();
    } else {
        owner_pl = get_owner_pl(env, rfdescr);
    }

    if (pl > owner_pl) {
        /* current PL is not good enough */
        switch(rfdescr->rerror) {
            case REG_READ_ERROR_READ0:
                *mask &= ~field_mask;
                return -1;
            case REG_READ_ERROR_TRAP_PRIVILEGE:
                return owner_pl;
            default:
                g_assert_not_reached();
        }
    }

    return -1;
}

/*
 * Check if it's OK to write a field.
 *
 * mask: tells which bits of the register are written.
 *
 * pl: tells what is the current PL level.
 *
 * Return -1 if the access is granted or the required PL level if the access
 *        does an unpriviledge access.
 */
static int check_field_write(CPUKVXState *env, RegisterField field,
                             int pl, uint64_t mask)
{
    const RegisterFieldDescr *rfdescr = &REGISTERFIELDS[field];
    uint64_t field_mask = MAKE_64BIT_MASK(rfdescr->offset, rfdescr->width);
    int owner_pl;

    if (!rfdescr->n_owners || !(field_mask & mask) ||
        rfdescr->werror == REG_WRITE_ERROR_WRITE) {
        return -1;
    }

    g_assert(rfdescr->werror == REG_WRITE_ERROR_TRAP_PRIVILEGE);

    if (field_is_per_bit_pl(field)) {
        int trap_pl = -1;
        //each bit of the field has its own PL owner
        g_assert(rfdescr->n_owners == rfdescr->width);

        for (int i = 0; i < rfdescr->width; i++) {
            uint64_t bit = 1ull << (rfdescr->offset + i);
            int bit_owner_pl;

            g_assert(REGISTERFIELDS[rfdescr->owners[i]].width == 2);
            bit_owner_pl = kvx_regfield_read(env, rfdescr->owners[i]);
            if ((bit & mask) && bit_owner_pl < pl && bit_owner_pl > trap_pl) {
                // we take the least privileged level
                trap_pl = bit_owner_pl;
            }
        }
        return trap_pl;
    } else {
        owner_pl = get_owner_pl(env, rfdescr);
    }

    return owner_pl < pl ? owner_pl : -1;
}

/*
 * Check if it's OK to read a register.
 *
 * mask: to get the mask-out to apply on read, should be all-ones at entry
 *
 * Return -1 if the access is granted or the PL trap level.
 */
static int check_register_read(CPUKVXState *env, Register reg, int pl,
                               uint64_t *mask)
{
    const RegisterDescr *rdescr = &REGISTERS[reg];
    int trap_pl = -1;

    for (int f = 0; f < rdescr->n_fields; f++) {
        int tmp = check_field_read(env, rdescr->fields[f], pl, mask);
        // in case there is different PL owner, we return the least
        // priviledge (ie: higher PL number)
        if (tmp > trap_pl) {
            trap_pl = tmp;
        }
    }

    return trap_pl;
}

/*
 * Check if it's OK to write a register.
 *
 * mask: the bitmask part of the register to be written.
 *       For SET/RSWAP it's an all-ones mask
 *       For WFXL/WFXM it's a partial mask
 *
 * Return -1 if the access is granted or the PL trap level.
 */
static int check_register_write(CPUKVXState *env, Register reg, int pl,
                                uint64_t mask)
{
    const RegisterDescr *rdescr = &REGISTERS[reg];
    int trap_pl = -1;

    for (int f = 0; f < rdescr->n_fields; f++) {
        int tmp = check_field_write(env, rdescr->fields[f], pl, mask);
        // in case there is different PL owner, we return the least
        // priviledge (ie: higher PL number)
        if (tmp > trap_pl) {
            trap_pl = tmp;
        }
    }

    return trap_pl;
}

/*
 * This function is used for register which are aliased on PL: xxx_PLy
 */
static int kvx_reg_check_pl(RegisterAccessType access,
                            int pl, uint64_t *mask, int required_pl)
{
    if (pl <= required_pl) {
        return -1;
    }
    switch(access) {
        /* pl is not right */
    case REG_ACCESS_GET:
        /* RAZ */
        *mask = 0;
        return -1;
    case REG_ACCESS_SET:
    case REG_ACCESS_WFX:
        /* trap privilege */
        return required_pl;
    default:
        g_assert_not_reached();
    }
}

/*
 * common function to do dynamic access check. It has no side effects.
 *
 * reg: register
 * access: type of access (SET, GET or WFX)
 * mask: optional pointer to a bitmask
 *       if is left to NULL, the mask is considered to be all ones
 *
 * if WFX mask should be initialized to the part of the register being changed
 * if SET/GET mask should be left NULL or initialized to all-ones
 *
 * if GET the mask is modified any read-as-zero bit masked-out
 *
 * return -1 if access is granted or the PL handling the privilege trap
 */
static int check_access(CPUKVXState *env, Register reg,
                        RegisterAccessType access,
                        uint64_t *mask)
{
    int trap_pl = -1, cur_pl = kvx_get_current_pl(env);
    const RegisterInfo *info;
    uint64_t ones = ~0ull;

    if (!mask) {
        mask = &ones;
    }

    reg = kvx_register_alias_pl(reg, cur_pl);
    info = kvx_register_info(reg);

    /* check using custom handler first */
    if (info->check_pl) {
        trap_pl = kvx_reg_check_pl(access, cur_pl, mask, info->required_pl);
    }

    /* then default fields check */
    if (trap_pl < 0) {
        switch (access) {
        case REG_ACCESS_GET:
            trap_pl = check_register_read(env, reg, cur_pl, mask);
            break;
        case REG_ACCESS_SET:
        case REG_ACCESS_WFX:
            trap_pl = check_register_write(env, reg, cur_pl, *mask);
            break;
        case REG_ACCESS_NONE:
            g_assert_not_reached();
        }
    }

    return trap_pl;
}

uint64_t HELPER(iget_check_access)(CPUKVXState *env, uint32_t sfr,
                                   uint64_t syndrome)
{
    int trap_pl;
    Register reg;

    g_assert(sfr < REGFILE_MAPPING[REGFILE_kv3_SFR].size);
    reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr];

    syndrome = KVX_FIELD_DP64(syndrome, kv3_ES, SFRP, sfr);

    /* first check if this kind of access is legal on the register */
    if (!kvx_register_support_access(reg, REG_ACCESS_GET)) {
        uint32_t opcode = TRAP_OPCODE_OR_VSFR(sfr);
        HELPER(raise_trap)(env, opcode, syndrome);
        return 0;
    }

    trap_pl = check_access(env, reg, REG_ACCESS_GET, NULL);

    /* trap if access not granted */
    if (trap_pl >= 0) {
        raise_privilege_trap(env, trap_pl, syndrome);
        return 0;
    }

    /* also update excp_syndrome in case we are in step mode */
    env->excp_syndrome = KVX_FIELD_DP64(env->excp_syndrome, kv3_ES, SFRP, sfr);

    return 1;
}

uint64_t HELPER(iget_apply)(CPUKVXState *env, uint32_t sfr, uint64_t value)
{
    int trap_pl;
    Register reg;
    uint64_t mask = ~0ull;

    g_assert(sfr < REGFILE_MAPPING[REGFILE_kv3_SFR].size);
    reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr];

    trap_pl = check_access(env, reg, REG_ACCESS_GET, &mask);
    g_assert(trap_pl == -1);

    return value & mask;
}

uint64_t HELPER(set_check_access)(CPUKVXState *env, uint32_t sfr,
                                  uint64_t syndrome)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr];
    int trap_pl = check_access(env, reg, REG_ACCESS_SET, NULL);

    /* trap if access not granted */
    if (trap_pl >= 0) {
        raise_privilege_trap(env, trap_pl, syndrome);
        return 0;
    }

    return 1;
}

uint64_t HELPER(wfxl_check_access)(CPUKVXState *env, uint32_t sfr,
                                   uint64_t setclear, uint64_t syndrome)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr];
    uint64_t mask = ((setclear >> 32) | setclear) & 0xffffffff;
    int trap_pl = check_access(env, reg, REG_ACCESS_WFX, &mask);

    /* trap if access not granted */
    if (trap_pl >= 0) {
        raise_privilege_trap(env, trap_pl, syndrome);
        return 0;
    }

    return 1;
}

uint64_t HELPER(wfxm_check_access)(CPUKVXState *env, uint32_t sfr,
                                   uint64_t setclear, uint64_t syndrome)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr];
    uint64_t mask = (((setclear >> 32) | setclear) & 0xffffffff) << 32;
    int trap_pl = check_access(env, reg, REG_ACCESS_WFX, &mask);

    /* trap if access not granted */
    if (trap_pl >= 0) {
        raise_privilege_trap(env, trap_pl, syndrome);
        return 0;
    }

    return 1;
}

void HELPER(check_atomic_access)(CPUKVXState *env, uint64_t addr,
                                 uint64_t size, uint64_t syndrome)
{
    int mmu_idx = cpu_mmu_index(env, false);
    unsigned int index;

    if (addr & (size - 1)) {
        /*
         * Atomic accesses must be aligned. We check here instead of in the
         * cpu_do_unaligned_access helper so we don't have to differentiate
         * between DEVICE cache policy and atomic accesses.
         */
        env->excp_syndrome = syndrome;
        fill_tlb_excp_syndrome(env, addr, MMU_DATA_LOAD, TRAP_DMISALIGN);
        kvx_raise_exception(env, KVX_EXCP_HW_TRAP, 0);
    }

    /*
     * Ensure the page is set in the TLB. This call will take care of
     * NOMAPPING, PROTECTION and WRITETOCLEAN traps. If we return, we know for
     * sure the TLB entry is valid.
     */
    probe_access(env, addr, size, MMU_DATA_STORE,
                 cpu_mmu_index(env, false), GETPC());

    index = tlb_index(env, mmu_idx, addr);

    if (env_tlb(env)->d[mmu_idx].iotlb[index].attrs.target_tlb_bit0) {
        /* The page is _not_ A_MODIFIED. We must raise an ATOMICTOCLEAN trap. */
        env->excp_syndrome = syndrome;
        fill_tlb_excp_syndrome(env, addr, MMU_DATA_LOAD, TRAP_ATOMICTOCLEAN);
        kvx_raise_exception(env, KVX_EXCP_HW_TRAP, 0);
    }
}
