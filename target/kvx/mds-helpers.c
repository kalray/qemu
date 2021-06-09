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
#include "semihosting/semihost.h"

#include "translate.h"
#include "gen/mds-helpers.h"

/* Helper macros to expand bigints into helper parameters */
#define _BI_EXPAND_2(_arg) (_arg)->val[0], (_arg)->val[1]
#define _BI_EXPAND_4(_arg) _BI_EXPAND_2(_arg), (_arg)->val[2], (_arg)->val[3]
#define _BI_EXPAND(arg, sz) _BI_EXPAND_##sz(arg)

#define BI_EXPAND(arg, sz) _BI_EXPAND(arg, sz)

/* Helper macro to load helper result from scratch values in CPU env */
#define _LOAD_HELPER_RESULT_2(_ret) do { \
    tcg_gen_ld_i64((_ret)->val[0], cpu_env, offsetof(CPUKVXState, scratch[0])); \
    tcg_gen_ld_i64((_ret)->val[1], cpu_env, offsetof(CPUKVXState, scratch[1])); \
} while (0)

#define _LOAD_HELPER_RESULT_4(_ret) do { \
    _LOAD_HELPER_RESULT_2(_ret); \
    tcg_gen_ld_i64((_ret)->val[2], cpu_env, offsetof(CPUKVXState, scratch[2])); \
    tcg_gen_ld_i64((_ret)->val[3], cpu_env, offsetof(CPUKVXState, scratch[3])); \
} while (0)

#define _LOAD_HELPER_RESULT(ret, sz) _LOAD_HELPER_RESULT_##sz(ret)

#define LOAD_HELPER_RESULT(ret, sz) _LOAD_HELPER_RESULT(ret, sz)

/* MMU/TLB helpers */
void gen_test_mmi_owner(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret)
{
    TCGv_i64 syndrome;

    switch (ctx->cur_opcode->insn) {
    case kv3_TLBWRITE:
        gen_fill_syndrome_pic(ctx, ES_PIC_TLBWRITE);
        break;
    case kv3_TLBREAD:
        gen_fill_syndrome_pic(ctx, ES_PIC_TLBREAD);
        break;
    case kv3_TLBPROBE:
        gen_fill_syndrome_pic(ctx, ES_PIC_TLBPROBE);
        break;
    default:
        g_assert_not_reached();
    }
    gen_save_pc(ctx, ctx->bundle.pc); // in case we trap

    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_test_rfe_owner(ret, cpu_env, syndrome);
    tcg_temp_free_i64(syndrome);
}

void gen_effect_readtlb(DisasContext *ctx)
{
    gen_helper_tlb_read(cpu_env);
}

void gen_effect_writetlb(DisasContext *ctx)
{
    gen_helper_tlb_write(cpu_env);
}

void gen_effect_probetlb(DisasContext *ctx)
{
    gen_helper_tlb_probe(cpu_env);
}

void gen_effect_invaldtlb(DisasContext *ctx)
{
    /* No architectural effect */
}

void gen_effect_invalitlb(DisasContext *ctx)
{
    /* No architectural effect */
}

void gen_test_wfxl_check_access(DisasContext *ctx, MDSTypeBinding ret_type,
                                TCGv_i64 ret, uint64_t sfr_idx,
                                TCGv_i64 setclear_val)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr_idx];
    const RegisterInfo *info = kvx_register_info(reg);
    gen_fill_syndrome_sfr(ctx, ES_SFRI_WFXL, sfr_idx, 0);

    /* first check if this kind of access is legal on the register */
    if (!kvx_register_support_access(reg, REG_ACCESS_WFX)) {
        uint32_t opcode = TRAP_OPCODE_OR_VSFR(sfr_idx);
        gen_raise_trap(ctx, opcode);
        tcg_gen_movi_i64(ret, 0);
        return;
    }

    /* generate a call to the dynamic checker only if needed */
    if (REGISTERS[reg].werror || info->check_pl) {
        TCGv_i32 sfr = tcg_const_i32(sfr_idx);
        TCGv_i64 syndrome = tcg_const_i64(get_syndrome(ctx));
        gen_save_pc(ctx, ctx->bundle.pc); // in case we trap
        gen_helper_wfxl_check_access(ret, cpu_env, sfr, setclear_val,
                                     syndrome);
        tcg_temp_free_i32(sfr);
        tcg_temp_free_i64(syndrome);
    } else {
        tcg_gen_movi_i64(ret, 1);
    }
}

void gen_test_wfxm_check_access(DisasContext *ctx, MDSTypeBinding ret_type,
                                TCGv_i64 ret, uint64_t sfr_idx,
                                TCGv_i64 setclear_val)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr_idx];
    const RegisterInfo *info = kvx_register_info(reg);

    gen_fill_syndrome_sfr(ctx, ES_SFRI_WFXM, sfr_idx, 0);

    /* first check if this kind of access is legal on the register */
    if (!kvx_register_support_access(reg, REG_ACCESS_WFX)) {
        uint32_t opcode = TRAP_OPCODE_OR_VSFR(sfr_idx);
        gen_raise_trap(ctx, opcode);
        tcg_gen_movi_i64(ret, 0);
        return;
    }

    /* generate a call to the dynamic checker only if needed */
    if (REGISTERS[reg].werror || info->check_pl) {
        TCGv_i32 sfr = tcg_const_i32(sfr_idx);
        TCGv_i64 syndrome = tcg_const_i64(get_syndrome(ctx));
        gen_save_pc(ctx, ctx->bundle.pc); // in case we trap
        gen_helper_wfxm_check_access(ret, cpu_env, sfr, setclear_val,
                                     syndrome);
        tcg_temp_free_i32(sfr);
        tcg_temp_free_i64(syndrome);
    } else {
        tcg_gen_movi_i64(ret, 1);
    }
}

static void gen_apply_wfx_relative_pl(DisasContext *ctx, TCGv_i64 ret,
                                      uint64_t sfr_idx, TCGv_i64 mask)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr_idx];
    const RegisterInfo *info = kvx_register_info(reg);

    if (info->apply_rpl) {
        TCGv_i32 sfr = tcg_const_i32(sfr_idx);
        TCGv_i64 syndrome;

        gen_fill_syndrome_sfr(ctx, ES_SFRI_WFXM, sfr_idx, 0);
        syndrome = tcg_const_i64(get_syndrome(ctx));

        /* no need to save PC, it has been done in test_access previously */
        gen_helper_apply_relative_pl(ret, cpu_env, sfr, ret, mask, syndrome);
        tcg_temp_free_i64(syndrome);
        tcg_temp_free_i32(sfr);
    }
}

void gen_apply_wfxl(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, uint64_t sfr_idx, TCGv_i64 setclear_val)
{
    TCGv_i64 clear_mask, set_mask, modified_mask;

    gen_load_operand(ctx, ret, REGCLASS_kv3_systemReg, sfr_idx);

    modified_mask = tcg_temp_new_i64();
    clear_mask = tcg_temp_new_i64();
    tcg_gen_andi_i64(modified_mask, setclear_val, 0xffffffff);
    tcg_gen_not_i64(clear_mask, modified_mask);
    tcg_gen_and_i64(ret, ret, clear_mask);
    tcg_temp_free_i64(clear_mask);

    set_mask = tcg_temp_new_i64();
    tcg_gen_shri_i64(set_mask, setclear_val, 32);
    tcg_gen_or_i64(modified_mask, modified_mask, set_mask);
    tcg_gen_or_i64(ret, ret, set_mask);
    tcg_temp_free_i64(set_mask);

    gen_apply_wfx_relative_pl(ctx, ret, sfr_idx, modified_mask);
    tcg_temp_free_i64(modified_mask);
}

void gen_apply_wfxm(DisasContext *ctx, MDSTypeBinding ret_type,
                    TCGv_i64 ret, uint64_t sfr_idx, TCGv_i64 setclear_val)
{
    TCGv_i64 clear_mask, set_mask, modified_mask;

    gen_load_operand(ctx, ret, REGCLASS_kv3_systemReg, sfr_idx);

    modified_mask = tcg_temp_new_i64();
    clear_mask = tcg_temp_new_i64();
    tcg_gen_shli_i64(modified_mask, setclear_val, 32);
    tcg_gen_not_i64(clear_mask, modified_mask);
    tcg_gen_and_i64(ret, ret, clear_mask);
    tcg_temp_free_i64(clear_mask);

    set_mask = tcg_temp_new_i64();
    tcg_gen_andi_i64(set_mask, setclear_val, 0xffffffffull << 32);
    tcg_gen_or_i64(modified_mask, modified_mask, set_mask);
    tcg_gen_or_i64(ret, ret, set_mask);
    tcg_temp_free_i64(set_mask);

    gen_apply_wfx_relative_pl(ctx, ret, sfr_idx, modified_mask);
    tcg_temp_free_i64(modified_mask);
}

void gen_test_get_check_access(DisasContext *ctx, MDSTypeBinding ret_type,
                               TCGv_i64 ret, TCGv_i64 sfr_idx,
                               uint64_t gpr_idx)
{
    TCGv_i32 sfr_idx32 = tcg_temp_new_i32();
    TCGv_i64 syndrome;
    uint32_t sfri;

    tcg_gen_extrl_i64_i32(sfr_idx32, sfr_idx);
    switch (ctx->cur_opcode->insn) {
        case kv3_GET:
            sfri = ES_SFRI_GET;
            break;
        case kv3_IGET:
            sfri = ES_SFRI_IGET;
            break;
        case kv3_RSWAP:
            sfri = ES_SFRI_RSWAP;
            break;
        default:
            g_assert_not_reached();
    }
    /* sfr_reg will be filled by the helper into the syndrome */
    gen_fill_syndrome_sfr(ctx, sfri, 0, gpr_idx);
    syndrome = tcg_const_i64(get_syndrome(ctx));

    gen_save_pc(ctx, ctx->bundle.pc); /* in case we trap */
    gen_helper_iget_check_access(ret, cpu_env, sfr_idx32, syndrome);
    tcg_temp_free_i32(sfr_idx32);
    tcg_temp_free_i64(syndrome);
}

void gen_apply_get(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret,
                   TCGv_i64 sfr_idx, TCGv_i64 value)
{
    /*
     * At this point no exception can occur, but we may need to maskout
     * some fields.
     */
    TCGv_i32 sfr_idx32 = tcg_temp_new_i32();
    tcg_gen_extrl_i64_i32(sfr_idx32, sfr_idx);
    gen_helper_iget_apply(ret, cpu_env, sfr_idx32, value);
    tcg_temp_free_i32(sfr_idx32);
}

void gen_test_set_check_access(DisasContext *ctx, MDSTypeBinding ret_type,
                               TCGv_i64 ret, uint64_t sfr_idx, TCGv_i64 value,
                               uint64_t gpr_idx)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr_idx];
    const RegisterInfo *info = kvx_register_info(reg);
    uint32_t sfri;

    /* we need this info later to trap on PL overflow */
    ctx->set_gpr_idx = gpr_idx;

    switch (ctx->cur_opcode->insn) {
        case kv3_SET:
            sfri = ES_SFRI_SET;
            break;
        case kv3_RSWAP:
            sfri = ES_SFRI_RSWAP;
            break;
        default:
            g_assert_not_reached();
    }

    gen_fill_syndrome_sfr(ctx, sfri, sfr_idx, gpr_idx);

    /* first check if this kind of access is legal on the register */
    if (!kvx_register_support_access(reg, REG_ACCESS_SET)) {
        uint32_t opcode = TRAP_OPCODE_OR_VSFR(sfr_idx);
        gen_raise_trap(ctx, opcode);
        tcg_gen_movi_i64(ret, 0);
        return;
    }

    /* generate a call to the dynamic checker only if needed */
    if (REGISTERS[reg].werror || info->check_pl) {
        TCGv_i32 sfr = tcg_const_i32(sfr_idx);
        TCGv_i64 syndrome = tcg_const_i64(get_syndrome(ctx));
        gen_save_pc(ctx, ctx->bundle.pc); /* in case we trap */
        gen_helper_set_check_access(ret, cpu_env, sfr, syndrome);
        tcg_temp_free_i32(sfr);
        tcg_temp_free_i64(syndrome);
    } else {
        tcg_gen_movi_i64(ret, 1);
    }
}

void gen_test_scalarcond(DisasContext *ctx, MDSTypeBinding ret_type,
                                TCGv_i64 ret, uint64_t arg_cond, TCGv_i64 val)
{
    static const TCGCond COND_MAPPING[] = {
        [SCALARCOND_DNEZ] = TCG_COND_NE,
        [SCALARCOND_DEQZ] = TCG_COND_EQ,
        [SCALARCOND_DLTZ] = TCG_COND_LT,
        [SCALARCOND_DGEZ] = TCG_COND_GE,
        [SCALARCOND_DLEZ] = TCG_COND_LE,
        [SCALARCOND_DGTZ] = TCG_COND_GT,
    };

    Modifier_kv3_scalarcond cond = arg_cond;
    int64_t imm = 0;

    if (cond == SCALARCOND_ODD) {
        imm = 1;
    }

    if (cond == SCALARCOND_ODD || cond == SCALARCOND_EVEN) {
        tcg_gen_andi_i64(val, val, 1);
        cond = SCALARCOND_DEQZ;
    }

    if (cond >= SCALARCOND_WNEZ) {
        /* 32 bits test */
        tcg_gen_ext32s_i64(val, val);
        cond -= SCALARCOND_WNEZ;
    }

    tcg_gen_setcondi_i64(COND_MAPPING[cond], ret, val, imm);
}

void gen_test_simplecond_32(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, uint64_t arg_cond, TCGv_i64 val)
{
    tcg_gen_shli_i64(val, val, 64 - 32);
    tcg_gen_sari_i64(val, val, 64 - 32);
    gen_test_scalarcond(ctx, ret_type, ret, arg_cond, val);
}

void gen_test_simplecond_16(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, uint64_t arg_cond, TCGv_i64 val)
{
    tcg_gen_shli_i64(val, val, 64 - 16);
    tcg_gen_sari_i64(val, val, 64 - 16);
    gen_test_scalarcond(ctx, ret_type, ret, arg_cond, val);
}


/*
 * ---------------------
 * -- Memory accesses --
 * ---------------------
 */
static inline MemOp compute_memop(int64_t size, MDSTypeBinding ret_type)
{
    MemOp mem_op = MO_TE;

    g_assert(size <= 8);
    mem_op |= size_memop(size);

    if (ret_type.type == MDS_SIGNED && size < 8) {
        mem_op |= MO_SIGN;
    }

    return mem_op;
}

static void gen_mem_load_regular(DisasContext *ctx, MDSTypeBinding ret_type,
                                 TCGv_i64 ret, TCGv_i64 address, int64_t size)
{
    tcg_gen_qemu_ld_i64(ret, address, ctx->mem_index,
                        compute_memop(size, ret_type));
}

static void gen_mem_load_speculative(DisasContext *ctx,
                                     MDSTypeBinding ret_type, TCGv_i64 ret,
                                     TCGv_i64 address, int64_t size)
{
    TCGv_i64 mem_op = tcg_const_i64(compute_memop(size, ret_type));
    gen_helper_load_speculative(ret, cpu_env, address, mem_op);
    tcg_temp_free_i64(mem_op);
}

static void gen_mem_load_speculative_bigint(DisasContext *ctx,
                                            MDSTypeBinding ret_type,
                                            MDSTCGBigInt *ret,
                                            TCGv_i64 address, int64_t sz)
{
    TCGv_i64 size = tcg_const_i64(sz);
    TCGv_i64 val;

    gen_helper_load_speculative_multiple(cpu_env, address, size);
    tcg_temp_free_i64(size);

    MDS_BIGINT_FOREACH(ret, val) {
        size_t i = mds_tcg_bigint_get_cur_idx(ret);
        tcg_gen_ld_i64(val, cpu_env, offsetof(CPUKVXState, scratch[i]));
    }
}

static void gen_mem_load(DisasContext *ctx, MDSTypeBinding ret_type,
                         TCGv_i64 ret, TCGv_i64 address, int64_t size,
                         bool is_speculative)
{
    if (is_speculative) {
        gen_mem_load_speculative(ctx, ret_type, ret, address, size);
    } else {
        gen_mem_load_regular(ctx, ret_type, ret, address, size);
    }
}

static void gen_mem_store(DisasContext *ctx, TCGv_i64 address, int64_t size,
                          TCGv_i64 data)
{
    switch (size) {
    case 1:
        tcg_gen_qemu_st8(data, address, ctx->mem_index);
        break;
    case 2:
        tcg_gen_qemu_st16(data, address, ctx->mem_index);
        break;
    case 4:
        tcg_gen_qemu_st32(data, address, ctx->mem_index);
        break;
    case 8:
        tcg_gen_qemu_st64(data, address, ctx->mem_index);
        break;
    default:
        g_assert_not_reached();
    }
}

void gen_apply_MEM_load(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 address, int64_t size,
                        uint64_t variant, uint64_t op)
{
    bool is_speculative = (variant == VARIANT_S || variant == VARIANT_US);

    gen_mem_load(ctx, ret_type, ret, address, size, is_speculative);
    gen_fill_syndrome_mem_access(ctx, address, size, variant, op,
                                 ES_RWX_DATA_SIDE_READ);
}

void gen_apply_MEM_load_bigint_tcg64_s64_u64_u64(DisasContext *ctx,
                                                 MDSTypeBinding ret_type,
                                                 MDSTCGBigInt *ret,
                                                 TCGv_i64 address, int64_t size,
                                                 uint64_t variant, uint64_t op)
{
    TCGv_i64 val;
    bool is_speculative = (variant == VARIANT_S || variant == VARIANT_US);

    g_assert(mds_tcg_bigint_get_size(ret) == size * 8);

    if (is_speculative) {
        gen_mem_load_speculative_bigint(ctx, ret_type, ret, address, size);
    } else {
        MDS_BIGINT_FOREACH(ret, val) {
            gen_mem_load_regular(ctx, ret_type, val, address, 8);
            tcg_gen_addi_i64(address, address, 8);
        }
    }

    gen_fill_syndrome_mem_access(ctx, address, size, variant, op,
                                 ES_RWX_DATA_SIDE_READ);
}

void gen_effect_MEM_store(DisasContext *ctx, TCGv_i64 address,
                          int64_t size, TCGv_i64 data, uint64_t op)
{
    gen_mem_store(ctx, address, size, data);
    gen_fill_syndrome_mem_access(ctx, address, size, 0, op,
                                 ES_RWX_DATA_SIDE_WRITE);
}

void gen_effect_MEM_store_tcg64_s64_bigint_u64(DisasContext *ctx, TCGv_i64 address,
                                               int64_t size, MDSTCGBigInt *data,
                                               uint64_t op)
{
    TCGv_i64 val;

    g_assert(mds_tcg_bigint_get_size(data) == size * 8);

    MDS_BIGINT_FOREACH(data, val) {
        gen_mem_store(ctx, address, 8, val);
        tcg_gen_addi_i64(address, address, 8);
    }

    gen_fill_syndrome_mem_access(ctx, address, size, 0, op,
                                 ES_RWX_DATA_SIDE_WRITE);
}

void gen_apply_MEM_cas(DisasContext *ctx,
                       MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 address, int64_t size, TCGv_i64 update,
                       TCGv_i64 expect, uint64_t boolcas, uint64_t dri)
{
    /* TODO: Implement this as an atomic operation */
    TCGv_i64 size_tcg, syndrome, tmp;

    gen_fill_syndrome_mem_access(ctx, address, size, 0, dri,
                                 ES_RWX_DATA_SIDE_ATOMIC);

    size_tcg = tcg_const_i64(size);
    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_check_atomic_access(cpu_env, address, size_tcg, syndrome);
    tcg_temp_free_i64(syndrome);
    tcg_temp_free_i64(size_tcg);

    tmp = tcg_temp_new_i64();
    ret_type.type = MDS_UNSIGNED;
    gen_mem_load(ctx, ret_type, tmp, address, size, false);

    tcg_gen_setcond_i64(TCG_COND_EQ, ret, tmp, expect);
    tcg_gen_movcond_i64(TCG_COND_EQ, tmp, tmp, expect, update, tmp);
    gen_mem_store(ctx, address, size, tmp);

    tcg_temp_free_i64(tmp);
}

void gen_apply_MEM_cas_tcg64_tcg64_s64_tcg64_bigint_s64_u64(DisasContext *ctx,
                                                            MDSTypeBinding ret_type, TCGv_i64 ret,
                                                            TCGv_i64 address, int64_t size, TCGv_i64 update,
                                                            MDSTCGBigInt * expect, int64_t boolcas, uint64_t dri)
{
    gen_apply_MEM_cas(ctx, ret_type, ret, address, size,
                      update, expect->val[0], boolcas, dri);
}

void gen_apply_MEM_cas_tcg64_tcg64_s64_tcg64_bigint_u64_u64(DisasContext *ctx,
                                                            MDSTypeBinding ret_type, TCGv_i64 ret,
                                                            TCGv_i64 address, int64_t size, TCGv_i64 update,
                                                            MDSTCGBigInt * expect, uint64_t boolcas, uint64_t dri)
{
    gen_apply_MEM_cas(ctx, ret_type, ret, address, size,
                      update, expect->val[0], boolcas, dri);
}

void gen_apply_MEM_faa(DisasContext *ctx,
                       MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 address, int64_t size,
                       TCGv_i64 addend, uint64_t reg)
{
    /* TODO: Implement this as an atomic operation */
    TCGv_i64 size_tcg, syndrome, tmp;

    gen_fill_syndrome_mem_access(ctx, address, size, 0, reg,
                                 ES_RWX_DATA_SIDE_ATOMIC);

    size_tcg = tcg_const_i64(size);
    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_check_atomic_access(cpu_env, address, size_tcg, syndrome);
    tcg_temp_free_i64(syndrome);
    tcg_temp_free_i64(size_tcg);

    gen_mem_load(ctx, ret_type, ret, address, size, false);

    tmp = tcg_temp_new_i64();
    tcg_gen_add_i64(tmp, ret, addend);
    gen_mem_store(ctx, address, size, tmp);

    tcg_temp_free_i64(tmp);
}

void gen_apply_MEM_swap(DisasContext *ctx,
                        MDSTypeBinding ret_type, TCGv_i64 ret,
                        TCGv_i64 address, int64_t size, int64_t update, uint64_t reg)
{
    /* TODO: Implement this as an atomic operation */
    TCGv_i64 size_tcg, syndrome, tmp;

    gen_fill_syndrome_mem_access(ctx, address, size, 0, reg,
                                 ES_RWX_DATA_SIDE_ATOMIC);

    size_tcg = tcg_const_i64(size);
    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_check_atomic_access(cpu_env, address, size_tcg, syndrome);
    tcg_temp_free_i64(syndrome);
    tcg_temp_free_i64(size_tcg);

    gen_mem_load(ctx, ret_type, ret, address, size, false);

    tmp = tcg_const_i64(update);
    gen_mem_store(ctx, address, size, tmp);

    tcg_temp_free_i64(tmp);
}

void gen_effect_MEM_dzerol(DisasContext *ctx, TCGv_i64 address, int64_t arg1)
{
    int i;
    TCGv_i64 zero = tcg_const_i64(0);

    /*
     * Start with a byte access at the given address. If the access raises an
     * MMU fault, the ea_plx register will be correct that way.
     */
    gen_mem_store(ctx, address, 1, zero);

    /* Align address on 64 bytes boundary */
    tcg_gen_andi_i64(address, address, ~((1 << 6) - 1));

    for (i = 0; i < 8; i++) {
        gen_mem_store(ctx, address, 8, zero);
        tcg_gen_addi_i64(address, address, 8);
    }

    tcg_temp_free_i64(zero);

    gen_fill_syndrome_mem_access(ctx, address, 8 * 8, 0, 0,
                                 ES_RWX_DATA_SIDE_WRITE);
}

void gen_test_comparison_64(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1,
                            TCGv_i64 arg2)
{
    static const TCGCond COND_MAPPING[] = {
        [COMPARISON_NE] = TCG_COND_NE,
        [COMPARISON_EQ] = TCG_COND_EQ,
        [COMPARISON_LT] = TCG_COND_LT,
        [COMPARISON_GE] = TCG_COND_GE,
        [COMPARISON_LE] = TCG_COND_LE,
        [COMPARISON_GT] = TCG_COND_GT,
        [COMPARISON_LTU] = TCG_COND_LTU,
        [COMPARISON_GEU] = TCG_COND_GEU,
        [COMPARISON_LEU] = TCG_COND_LEU,
        [COMPARISON_GTU] = TCG_COND_GTU,
        [COMPARISON_ALL] = TCG_COND_EQ,
        [COMPARISON_NALL] = TCG_COND_NE,
        [COMPARISON_ANY] = TCG_COND_NE,
        [COMPARISON_NONE] = TCG_COND_EQ,
    };

    Modifier_kv3_comparison comp = arg0;

    if (comp > COMPARISON_GTU) {
        tcg_gen_and_i64(arg1, arg1, arg2);
    }

    if (comp > COMPARISON_NALL) {
        tcg_gen_setcondi_i64(COND_MAPPING[comp], ret, arg1, 0);
    } else {
        tcg_gen_setcond_i64(COND_MAPPING[comp], ret, arg1, arg2);
    }
}

void gen_test_comparison_32(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1,
                            TCGv_i64 arg2)
{
    if (arg0 < COMPARISON_LTU) {
        tcg_gen_shli_i64(arg1, arg1, 64 - 32);
        tcg_gen_sari_i64(arg1, arg1, 64 - 32);

        tcg_gen_shli_i64(arg2, arg2, 64 - 32);
        tcg_gen_sari_i64(arg2, arg2, 64 - 32);
    } else {
        tcg_gen_andi_i64(arg1, arg1, (1ull << 32) - 1);
        tcg_gen_andi_i64(arg2, arg2, (1ull << 32) - 1);
    }

    gen_test_comparison_64(ctx, ret_type, ret, arg0, arg1, arg2);
}

void gen_test_comparison_16(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1,
                            TCGv_i64 arg2)
{
    if (arg0 < COMPARISON_LTU) {
        tcg_gen_shli_i64(arg1, arg1, 64 - 16);
        tcg_gen_sari_i64(arg1, arg1, 64 - 16);

        tcg_gen_shli_i64(arg2, arg2, 64 - 16);
        tcg_gen_sari_i64(arg2, arg2, 64 - 16);
    } else {
        tcg_gen_andi_i64(arg1, arg1, (1ull << 16) - 1);
        tcg_gen_andi_i64(arg2, arg2, (1ull << 16) - 1);
    }

    gen_test_comparison_64(ctx, ret_type, ret, arg0, arg1, arg2);
}

void gen_apply_CLZ(DisasContext *ctx,
                   MDSTypeBinding ret_type, TCGv_i64 ret,
                   int64_t size, TCGv_i64 arg)
{
    TCGv_i32 a0;

    switch (size) {
    case 32:
        a0 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a0, arg);

        tcg_gen_clzi_i32(a0, a0, size);

        tcg_gen_extu_i32_i64(ret, a0);
        tcg_temp_free_i32(a0);
        break;

    case 64:
        tcg_gen_clzi_i64(ret, arg, size);
        break;

    default:
        g_assert_not_reached();
    }
}

void gen_apply_CLS(DisasContext *ctx,
                   MDSTypeBinding ret_type, TCGv_i64 ret,
                   int64_t size, TCGv_i64 arg)
{
    TCGv_i32 a0;

    switch (size) {
    case 32:
        a0 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a0, arg);

        tcg_gen_clrsb_i32(a0, a0);

        tcg_gen_extu_i32_i64(ret, a0);
        tcg_temp_free_i32(a0);
        break;

    case 64:
        tcg_gen_clrsb_i64(ret, arg);
        break;

    default:
        g_assert_not_reached();
    }
}

void gen_apply_CBS(DisasContext *ctx,
                   MDSTypeBinding ret_type, TCGv_i64 ret,
                   int64_t size, TCGv_i64 arg)
{
    TCGv_i32 a0;

    switch (size) {
    case 32:
        a0 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a0, arg);

        tcg_gen_ctpop_i32(a0, a0);

        tcg_gen_extu_i32_i64(ret, a0);
        tcg_temp_free_i32(a0);
        break;

    case 64:
        tcg_gen_ctpop_i64(ret, arg);
        break;

    default:
        g_assert_not_reached();
    }
}

void gen_apply_CTZ(DisasContext *ctx,
                   MDSTypeBinding ret_type, TCGv_i64 ret,
                   int64_t size, TCGv_i64 arg)
{
    TCGv_i32 a0;

    switch (size) {
    case 32:
        a0 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a0, arg);

        tcg_gen_ctzi_i32(a0, a0, size);

        tcg_gen_extu_i32_i64(ret, a0);
        tcg_temp_free_i32(a0);
        break;

    case 64:
        tcg_gen_ctzi_i64(ret, arg, size);
        break;

    default:
        g_assert_not_reached();
    }
}

void gen_apply_ROL(DisasContext *ctx,
                   MDSTypeBinding ret_type, TCGv_i64 ret,
                   int64_t size, TCGv_i64 arg0, TCGv_i64 arg1)
{
    TCGv_i32 a0, a1;

    switch (size) {
    case 32:
        a0 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a0, arg0);

        a1 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a1, arg1);

        tcg_gen_rotl_i32(a0, a0, a1);
        tcg_temp_free_i32(a1);

        tcg_gen_extu_i32_i64(ret, a0);
        tcg_temp_free_i32(a0);
        break;

    case 64:
        tcg_gen_rotl_i64(ret, arg0, arg1);
        break;

    default:
        g_assert_not_reached();
    }
}

void gen_apply_ROR(DisasContext *ctx,
                   MDSTypeBinding ret_type, TCGv_i64 ret,
                   int64_t size, TCGv_i64 arg0, TCGv_i64 arg1)
{
    TCGv_i32 a0, a1;

    switch (size) {
    case 32:
        a0 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a0, arg0);

        a1 = tcg_temp_new_i32();
        tcg_gen_extrl_i64_i32(a1, arg1);

        tcg_gen_rotr_i32(a0, a0, a1);
        tcg_temp_free_i32(a1);

        tcg_gen_extu_i32_i64(ret, a0);
        tcg_temp_free_i32(a0);
        break;

    case 64:
        tcg_gen_rotr_i64(ret, arg0, arg1);
        break;

    default:
        g_assert_not_reached();
    }
}

void gen_apply_SWAP(DisasContext *ctx,
                    MDSTypeBinding ret_type, TCGv_i64 ret,
                    int64_t arg0, TCGv_i64 arg1)
{
    uint64_t mask;

    switch (arg0) {
    case 1:
        mask = 0x5555555555555555ull;
        break;

    case 2:
        mask = 0x3333333333333333ull;
        break;

    case 4:
        mask = 0x0f0f0f0f0f0f0f0full;
        break;

    case 8:
        mask = 0x00ff00ff00ff00ffull;
        break;

    case 16:
        mask = 0x0000ffff0000ffffull;
        break;

    case 32:
        mask = 0x00000000ffffffffull;
        break;

    default:
        g_assert_not_reached();
    }


    tcg_gen_andi_i64(ret, arg1, mask);
    tcg_gen_shli_i64(ret, ret, arg0);

    tcg_gen_andi_i64(arg1, arg1, ~mask);
    tcg_gen_shri_i64(arg1, arg1, arg0);

    tcg_gen_or_i64(ret, ret, arg1);
}


void gen_apply_insert_64_bigint_bigint_tcg64_u64(DisasContext *ctx,
                                                 MDSTypeBinding ret_type,
                                                 MDSTCGBigInt *ret,
                                                 MDSTCGBigInt *arg0,
                                                 TCGv_i64 arg1,
                                                 uint64_t column)
{
    TCGv_i64 src, dest;

    g_assert((column * 64) < mds_tcg_bigint_get_size(arg0));
    g_assert(mds_tcg_bigint_get_size(ret) == mds_tcg_bigint_get_size(arg0));

    MDS_BIGINT_FOREACH_2(ret, dest, arg0, src) {
        size_t idx = mds_tcg_bigint_get_cur_idx(ret);

        if (column == idx) {
            tcg_gen_mov_i64(dest, arg1);
        } else {
            tcg_gen_mov_i64(dest, src);
        }
    }
}

void gen_apply_insert_64_bigint_bigint_bigint_u64(DisasContext *ctx,
                                                  MDSTypeBinding ret_type,
                                                  MDSTCGBigInt * ret,
                                                  MDSTCGBigInt * arg0,
                                                  MDSTCGBigInt * arg1,
                                                  uint64_t column)
{
    gen_apply_insert_64_bigint_bigint_tcg64_u64(ctx, ret_type, ret,
                                                arg0, arg1->val[0], column);
}

void gen_apply__BMM_8(DisasContext *ctx,
                      MDSTypeBinding ret_type, TCGv_i64 ret,
                      TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_bmm_8(ret, arg0, arg1);
}

void gen_apply__BMT_8(DisasContext *ctx,
                      MDSTypeBinding ret_type, TCGv_i64 ret,
                      TCGv_i64 arg0)
{
    gen_helper_bmt_8(ret, arg0);
}

void gen_apply_clm_64_128_bigint_tcg64_tcg64(DisasContext *ctx,
                                             MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                             TCGv_i64 arg0, TCGv_i64 arg1)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_clm_64_128(cpu_env, arg0, arg1);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_clm_64_128_bigint_tcg64_s64(DisasContext *ctx,
                                           MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                           TCGv_i64 arg0, int64_t arg1)
{
    TCGv_i64 a1 = tcg_const_i64(arg1);

    gen_apply_clm_64_128_bigint_tcg64_tcg64(ctx, ret_type, ret, arg0, a1);
    tcg_temp_free_i64(a1);
}

void gen_apply_gcm_bb_64_128_bigint_tcg64_tcg64(DisasContext *ctx,
                                                MDSTypeBinding ret_type,
                                                MDSTCGBigInt *ret,
                                                TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_gcm_bb_64_128(cpu_env, arg0, arg1);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_gcm_bt_64_128_bigint_tcg64_tcg64(DisasContext *ctx,
                                                MDSTypeBinding ret_type,
                                                MDSTCGBigInt *ret,
                                                TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_gcm_bt_64_128(cpu_env, arg0, arg1);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_gcm_tt_64_128_bigint_tcg64_tcg64(DisasContext *ctx,
                                                MDSTypeBinding ret_type,
                                                MDSTCGBigInt *ret,
                                                TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_gcm_tt_64_128(cpu_env, arg0, arg1);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_crc32_be_u32(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_crc32_be_u32(ret, arg0, arg1);
}

void gen_apply_reflect_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_reflect_32(ret, arg0);
}

void gen_apply_transpose_64_4x4_0_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx,
                                                                     MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                                     MDSTCGBigInt *arg0, MDSTCGBigInt *arg1,
                                                                     MDSTCGBigInt *arg2, MDSTCGBigInt *arg3)
{
    tcg_gen_mov_i64(ret->val[0], arg0->val[0]);
    tcg_gen_mov_i64(ret->val[1], arg1->val[0]);
    tcg_gen_mov_i64(ret->val[2], arg2->val[0]);
    tcg_gen_mov_i64(ret->val[3], arg3->val[0]);
}

void gen_apply_transpose_64_4x4_1_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx,
                                                                     MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                                     MDSTCGBigInt *arg0, MDSTCGBigInt *arg1,
                                                                     MDSTCGBigInt *arg2, MDSTCGBigInt *arg3)
{
    tcg_gen_mov_i64(ret->val[0], arg0->val[1]);
    tcg_gen_mov_i64(ret->val[1], arg1->val[1]);
    tcg_gen_mov_i64(ret->val[2], arg2->val[1]);
    tcg_gen_mov_i64(ret->val[3], arg3->val[1]);
}

void gen_apply_transpose_64_4x4_2_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx,
                                                                     MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                                     MDSTCGBigInt *arg0, MDSTCGBigInt *arg1,
                                                                     MDSTCGBigInt *arg2, MDSTCGBigInt *arg3)
{
    tcg_gen_mov_i64(ret->val[0], arg0->val[2]);
    tcg_gen_mov_i64(ret->val[1], arg1->val[2]);
    tcg_gen_mov_i64(ret->val[2], arg2->val[2]);
    tcg_gen_mov_i64(ret->val[3], arg3->val[2]);
}

void gen_apply_transpose_64_4x4_3_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx,
                                                                     MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                                     MDSTCGBigInt *arg0, MDSTCGBigInt *arg1,
                                                                     MDSTCGBigInt *arg2, MDSTCGBigInt *arg3)
{
    tcg_gen_mov_i64(ret->val[0], arg0->val[3]);
    tcg_gen_mov_i64(ret->val[1], arg1->val[3]);
    tcg_gen_mov_i64(ret->val[2], arg2->val[3]);
    tcg_gen_mov_i64(ret->val[3], arg3->val[3]);
}

void gen_apply_dot8_8_32(DisasContext *ctx, MDSTypeBinding ret_type,
                         TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot8_8_32(ret, arg0, arg1);
}

void gen_apply_join_32_x4_bigint_tcg64_tcg64_tcg64_tcg64(DisasContext *ctx,
                                                         MDSTypeBinding ret_type,
                                                         MDSTCGBigInt *ret,
                                                         TCGv_i64 arg0, TCGv_i64 arg1,
                                                         TCGv_i64 arg2, TCGv_i64 arg3)
{
    tcg_gen_mov_i64(ret->val[0], arg0);
    tcg_gen_shli_i64(arg1, arg1, 32);
    tcg_gen_or_i64(ret->val[0], ret->val[0], arg1);

    tcg_gen_mov_i64(ret->val[1], arg2);
    tcg_gen_shli_i64(arg3, arg3, 32);
    tcg_gen_or_i64(ret->val[1], ret->val[1], arg3);
}

void gen_apply_join_32_x8_bigint_tcg64_tcg64_tcg64_tcg64_tcg64_tcg64_tcg64_tcg64(DisasContext *ctx,
                                                                                 MDSTypeBinding ret_type,
                                                                                 MDSTCGBigInt * ret,
                                                                                 TCGv_i64 arg0, TCGv_i64 arg1,
                                                                                 TCGv_i64 arg2, TCGv_i64 arg3,
                                                                                 TCGv_i64 arg4, TCGv_i64 arg5,
                                                                                 TCGv_i64 arg6, TCGv_i64 arg7)
{
    tcg_gen_andi_i64(ret->val[0], arg0, 0xffffffff);
    tcg_gen_shli_i64(arg1, arg1, 32);
    tcg_gen_or_i64(ret->val[0], ret->val[0], arg1);

    tcg_gen_andi_i64(ret->val[1], arg2, 0xffffffff);
    tcg_gen_shli_i64(arg3, arg3, 32);
    tcg_gen_or_i64(ret->val[1], ret->val[1], arg3);

    tcg_gen_andi_i64(ret->val[2], arg4, 0xffffffff);
    tcg_gen_shli_i64(arg5, arg5, 32);
    tcg_gen_or_i64(ret->val[2], ret->val[2], arg5);

    tcg_gen_andi_i64(ret->val[3], arg6, 0xffffffff);
    tcg_gen_shli_i64(arg7, arg7, 32);
    tcg_gen_or_i64(ret->val[3], ret->val[3], arg7);
}

void gen_apply_join_64_x4_bigint_tcg64_tcg64_tcg64_tcg64(DisasContext *ctx,
                                                         MDSTypeBinding ret_type, MDSTCGBigInt * ret,
                                                         TCGv_i64 arg0, TCGv_i64 arg1,
                                                         TCGv_i64 arg2, TCGv_i64 arg3)
{
    tcg_gen_mov_i64(ret->val[0], arg0);
    tcg_gen_mov_i64(ret->val[1], arg1);
    tcg_gen_mov_i64(ret->val[2], arg2);
    tcg_gen_mov_i64(ret->val[3], arg3);
}

void gen_apply_add_32_32_x8_bigint_bigint_bigint(DisasContext *ctx,
                                                 MDSTypeBinding ret_type,
                                                 MDSTCGBigInt *ret,
                                                 MDSTCGBigInt *arg0,
                                                 MDSTCGBigInt *arg1)
{
    TCGv_i64 dst, a0, a1, tmp;

    tmp = tcg_temp_new_i64();

    MDS_BIGINT_FOREACH_3(ret, dst, arg0, a0, arg1, a1) {
        tcg_gen_add_i64(dst, a0, a1);
        tcg_gen_andi_i64(dst, dst, 0x00000000ffffffff);
        tcg_gen_andi_i64(a0, a0, 0xffffffff00000000);
        tcg_gen_andi_i64(a1, a1, 0xffffffff00000000);
        tcg_gen_add_i64(tmp, a0, a1);
        tcg_gen_or_i64(dst, dst, tmp);
    }

    tcg_temp_free_i64(tmp);
}

void gen_apply_add_64_64_x4_bigint_bigint_bigint(DisasContext *ctx,
                                                 MDSTypeBinding ret_type,
                                                 MDSTCGBigInt *ret,
                                                 MDSTCGBigInt *arg0,
                                                 MDSTCGBigInt *arg1)
{
    TCGv_i64 dst, a0, a1;

    MDS_BIGINT_FOREACH_3(ret, dst, arg0, a0, arg1, a1) {
        tcg_gen_add_i64(dst, a0, a1);
    }
}

void gen_apply_dot8u_8_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot8u_8_32(ret, arg0, arg1);
}

void gen_apply_dot8su_8_32(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot8su_8_32(ret, arg0, arg1);
}

void gen_apply_dot8us_8_32(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot8us_8_32(ret, arg0, arg1);
}

void gen_apply_sxb_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret,
                      TCGv_i64 arg0)
{
    gen_helper_sxb_x4(ret, arg0);
}

void gen_apply_dot4_16_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot4_16_64(ret, arg0, arg1);
}

void gen_apply_zxb_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret,
                      TCGv_i64 arg0)
{
    gen_helper_zxb_x4(ret, arg0);
}

void gen_apply_dot4u_16_64(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot4u_16_64(ret, arg0, arg1);
}

void gen_apply_dot4su_16_64(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot4su_16_64(ret, arg0, arg1);
}

void gen_apply_dot4us_16_64(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_dot4us_16_64(ret, arg0, arg1);
}

void gen_apply_round_64_x4_bigint_u64_tcg64_bigint(DisasContext *ctx,
                                                   MDSTypeBinding ret_type,
                                                   MDSTCGBigInt * ret,
                                                   uint64_t imm0, TCGv_i64 arg1,
                                                   MDSTCGBigInt * arg2)
{
    TCGv_i64 arg0 = tcg_const_i64(imm0);
    gen_helper_round_64_x4(cpu_env, arg0, arg1, BI_EXPAND(arg2, 4));
    tcg_temp_free_i64(arg0);

    LOAD_HELPER_RESULT(ret, 4);
}

void gen_apply_round_32_x8_bigint_u64_tcg64_bigint(DisasContext *ctx,
                                                   MDSTypeBinding ret_type, MDSTCGBigInt * ret,
                                                   uint64_t imm0, TCGv_i64 arg1, MDSTCGBigInt * arg2)
{
    TCGv_i64 arg0 = tcg_const_i64(imm0);

    gen_helper_round_32_x8(cpu_env, arg0, arg1, BI_EXPAND(arg2, 4));

    tcg_temp_free_i64(arg0);

    LOAD_HELPER_RESULT(ret, 4);
}

void gen_apply_satu_64_16_x4_tcg64_bigint(DisasContext *ctx,
                                          MDSTypeBinding ret_type, TCGv_i64 ret,
                                          MDSTCGBigInt * arg0)
{
    TCGv_i64 a0, min, max;

    tcg_gen_movi_i64(ret, 0);

    min = tcg_const_i64(0);
    max = tcg_const_i64(0xffff);

    MDS_BIGINT_FOREACH_REVERSE(arg0, a0) {
        tcg_gen_smin_i64(a0, a0, max);
        tcg_gen_smax_i64(a0, a0, min);
        tcg_gen_shli_i64(ret, ret, 16);
        tcg_gen_or_i64(ret, ret, a0);
    }

    tcg_temp_free_i64(max);
    tcg_temp_free_i64(min);
}

void gen_apply_sat_64_16_x4_tcg64_bigint(DisasContext *ctx,
                                         MDSTypeBinding ret_type, TCGv_i64 ret,
                                         MDSTCGBigInt * arg0)
{
    TCGv_i64 a0, min, max;

    tcg_gen_movi_i64(ret, 0);

    min = tcg_const_i64(-0x8000);
    max = tcg_const_i64(0x7fff);

    MDS_BIGINT_FOREACH_REVERSE(arg0, a0) {
        tcg_gen_smin_i64(a0, a0, max);
        tcg_gen_smax_i64(a0, a0, min);
        tcg_gen_andi_i64(a0, a0, 0xffff);
        tcg_gen_shli_i64(ret, ret, 16);
        tcg_gen_or_i64(ret, ret, a0);
    }

    tcg_temp_free_i64(max);
    tcg_temp_free_i64(min);
}

void gen_apply_satu_32_8_x8_tcg64_bigint(DisasContext *ctx,
                                         MDSTypeBinding ret_type, TCGv_i64 ret,
                                         MDSTCGBigInt * arg0)
{
    gen_helper_satu_32_8_x8(ret, BI_EXPAND(arg0, 4));
}

void gen_apply_sat_32_8_x8_tcg64_bigint(DisasContext *ctx,
                                        MDSTypeBinding ret_type, TCGv_i64 ret,
                                        MDSTCGBigInt * arg0)
{
    gen_helper_sat_32_8_x8(ret, BI_EXPAND(arg0, 4));
}


/*
 * Floating point helpers
 */

void gen_apply_fsrec_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_fsrec_64(ret, arg0);
}

void gen_apply_fdivbyzero(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    gen_helper_fdivbyzero(ret);
}

void gen_apply_finexact(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    gen_helper_finexact(ret);
}

void gen_apply_finvalid(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    gen_helper_finvalid(ret);
}

void gen_apply_foverflow(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    gen_helper_foverflow(ret);
}

void gen_apply_funderflow(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    gen_helper_funderflow(ret);
}

void gen_apply_fnarrow_64_32_x2_tcg64_tcg64_bigint(DisasContext *ctx,
                                                   MDSTypeBinding ret_type, TCGv_i64 ret,
                                                   TCGv_i64 rm, MDSTCGBigInt *arg0)
{
    g_assert(mds_tcg_bigint_get_size(arg0) == 128);

    gen_helper_fnarrow_64_32_x2(ret, rm, arg0->val[0], arg0->val[1]);
}

void gen_apply_fnarrow_32_16_x4_tcg64_tcg64_bigint(DisasContext *ctx,
                                                   MDSTypeBinding ret_type, TCGv_i64 ret,
                                                   TCGv_i64 rm, MDSTCGBigInt *arg0)
{
    g_assert(mds_tcg_bigint_get_size(arg0) == 128);

    gen_helper_fnarrow_32_16_x4(ret, rm, arg0->val[0], arg0->val[1]);
}

void gen_apply_fsrsr_64(DisasContext *ctx,
                        MDSTypeBinding ret_type, TCGv_i64 ret,
                        TCGv_i64 arg0)
{
    gen_helper_fsrsr_64(ret, arg0);
}

void gen_apply_fwiden_32_64(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_fwiden_32_64(ret, arg0);
}

void gen_apply_fnarrow_64_32(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fnarrow_64_32(ret, arg0, arg1);
}

void gen_apply_frec_32(DisasContext *ctx,
                       MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_frec_32(ret, arg0, arg1);
}

void gen_apply_frsq_32(DisasContext *ctx,
                       MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_frsq_32(ret, arg0, arg1);
}

void gen_apply_fsrec_32(DisasContext *ctx,
                        MDSTypeBinding ret_type, TCGv_i64 ret,
                        TCGv_i64 arg0)
{
    gen_helper_fsrec_32(ret, arg0);
}

void gen_apply_fsrsr_32(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_fsrsr_32(ret, arg0);
}

void gen_apply_fwiden_16_32(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_fwiden_16_32(ret, arg0);
}

void gen_apply_fnarrow_32_16(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fnarrow_32_16(ret, arg0, arg1);
}

void gen_apply_fsrec_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_fsrec_32_x2(ret, arg0);
}

void gen_apply_fsrsr_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_fsrsr_32_x2(ret, arg0);
}

void gen_apply_fwiden_16_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                               TCGv_i64 ret, TCGv_i64 arg0)
{
    gen_helper_fwiden_16_32_x2(ret, arg0);
}

void gen_apply_fsdiv_64(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fsdiv_64(ret, arg0, arg1);
}

void gen_apply_fcdiv_64(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fcdiv_64(ret, arg0, arg1);
}

void gen_apply_fsdiv_32(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fsdiv_32(ret, arg0, arg1);
}

void gen_apply_fcdiv_32(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fcdiv_32(ret, arg0, arg1);
}

void gen_apply_fsdiv_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fsdiv_32_x2(ret, arg0, arg1);
}

void gen_apply_fcdiv_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fcdiv_32_x2(ret, arg0, arg1);
}

void gen_apply_fmin_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmin_64(ret, arg0, arg1);
}

void gen_apply_fmax_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmax_64(ret, arg0, arg1);
}

void gen_apply_fmin_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmin_32(ret, arg0, arg1);
}

void gen_apply_fmax_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret,
                       TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmax_32(ret, arg0, arg1);
}

void gen_apply_fmin_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmin_32_x2(ret, arg0, arg1);
}

void gen_apply_fmax_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmax_32_x2(ret, arg0, arg1);
}

void gen_apply_fmin_16_x4(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmin_16_x4(ret, arg0, arg1);
}

void gen_apply_fmax_16_x4(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    gen_helper_fmax_16_x4(ret, arg0, arg1);
}

void gen_apply_float_64(DisasContext *ctx,
                        MDSTypeBinding ret_type, TCGv_i64 ret,
                        TCGv_i64 rm, TCGv_i64 arg1, uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_float_64(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_float_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                           uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_float_32_x2(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_float_32(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                        uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_float_32(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_floatu_64(DisasContext *ctx, MDSTypeBinding ret_type,
                         TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                         uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_floatu_64(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_floatu_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                            uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_floatu_32_x2(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_floatu_32(DisasContext *ctx, MDSTypeBinding ret_type,
                         TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                         uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_floatu_32(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fixed_64(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                        uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_fixed_64(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fixed_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                           uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_fixed_32_x2(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fixed_32(DisasContext *ctx, MDSTypeBinding ret_type,
                        TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                        uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_fixed_32(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fixedu_64(DisasContext *ctx, MDSTypeBinding ret_type,
                         TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                         uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_fixedu_64(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fixedu_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                            uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_fixedu_32_x2(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fixedu_32(DisasContext *ctx, MDSTypeBinding ret_type,
                         TCGv_i64 ret, TCGv_i64 rm, TCGv_i64 arg1,
                         uint64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_helper_fixedu_32(ret, rm, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_ffma_64_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffma_64_64(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffma_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffma_32_32_x2(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffma_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffma_16_16_x4(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffms_64_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffms_64_64(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffms_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffms_32_32_x2(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffms_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffms_16_16_x4(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_fmul_64_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2)
{
    gen_helper_fmul_64_64(ret, arg0, arg1, arg2);
}

void gen_apply_fmul_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2)
{
    gen_helper_fmul_32_32_x2(ret, arg0, arg1, arg2);
}

void gen_apply_fmul_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2)
{
    gen_helper_fmul_16_16_x4(ret, arg0, arg1, arg2);
}

void gen_apply_fmul_32_64_x2_bigint_tcg64_tcg64_tcg64(DisasContext *ctx,
                                                      MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                      TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmul_32_64_x2(cpu_env, arg0, arg1, arg2);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmul_32_64_x2_bigint_tcg64_tcg64_s64(DisasContext *ctx,
                                                    MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                    TCGv_i64 arg0, TCGv_i64 arg1, int64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_apply_fmul_32_64_x2_bigint_tcg64_tcg64_tcg64(ctx, ret_type, ret, arg0, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fmul_16_32_x4_bigint_tcg64_tcg64_tcg64(DisasContext *ctx,
                                                      MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                      TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmul_16_32_x4(cpu_env, arg0, arg1, arg2);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmul_16_32_x4_bigint_tcg64_tcg64_s64(DisasContext *ctx,
                                                    MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                    TCGv_i64 arg0, TCGv_i64 arg1, int64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_apply_fmul_16_32_x4_bigint_tcg64_tcg64_tcg64(ctx, ret_type, ret, arg0, arg1, arg2);

    tcg_temp_free_i64(arg2);
}


void gen_apply_fmul_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                        MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                        TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmul_32_32_x4(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmul_16_16_x8_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                        MDSTypeBinding ret_type, MDSTCGBigInt * ret,
                                                        TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmul_16_16_x8(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmulc_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                           TCGv_i64 arg2)
{
    gen_helper_fmulc_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_fmulc_32_64_bigint_tcg64_tcg64_tcg64(DisasContext *ctx,
                                                    MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                    TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmulc_32_64(cpu_env, arg0, arg1, arg2);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmulc_32_64_bigint_tcg64_tcg64_s64(DisasContext *ctx,
                                                  MDSTypeBinding ret_type,
                                                  MDSTCGBigInt *ret,
                                                  TCGv_i64 arg0, TCGv_i64 arg1,
                                                  int64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_apply_fmulc_32_64_bigint_tcg64_tcg64_tcg64(ctx, ret_type, ret, arg0, arg1, arg2);

    tcg_temp_free_i64(arg2);
}


void gen_apply_fmulcc_32_32(DisasContext *ctx,
                            MDSTypeBinding ret_type, TCGv_i64 ret,
                            TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    gen_helper_fmulcc_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_fmulcc_32_64_bigint_tcg64_tcg64_tcg64(DisasContext *ctx,
                                                     MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                     TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmulcc_32_64(cpu_env, arg0, arg1, arg2);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmulcc_32_64_bigint_tcg64_tcg64_s64(DisasContext *ctx,
                                                   MDSTypeBinding ret_type,
                                                   MDSTCGBigInt *ret,
                                                   TCGv_i64 arg0, TCGv_i64 arg1,
                                                   int64_t imm2)
{
    TCGv_i64 arg2 = tcg_const_i64(imm2);

    gen_apply_fmulcc_32_64_bigint_tcg64_tcg64_tcg64(ctx, ret_type, ret, arg0, arg1, arg2);

    tcg_temp_free_i64(arg2);
}

void gen_apply_fadd_64_64(DisasContext *ctx,
                          MDSTypeBinding ret_type, TCGv_i64 ret,
                          TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    gen_helper_fadd_64_64(ret, arg0, arg1, arg2);
}

void gen_apply_fadd_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2)
{
    gen_helper_fadd_32_32_x2(ret, arg0, arg1, arg2);
}

void gen_apply_fadd_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2)
{
    gen_helper_fadd_16_16_x4(ret, arg0, arg1, arg2);
}

void gen_apply_fadd_16_16_x8_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type,
                                                        MDSTCGBigInt * ret, TCGv_i64 arg0,
                                                        MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fadd_16_16_x8(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffma_16_16_x8_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx,
                                                               MDSTypeBinding ret_type,
                                                               MDSTCGBigInt * ret,
                                                               TCGv_i64 arg0,
                                                               MDSTCGBigInt * arg1,
                                                               MDSTCGBigInt * arg2,
                                                               MDSTCGBigInt * arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    tcg_gen_st_i64(arg3->val[0], cpu_env, offsetof(CPUKVXState, scratch[0]));
    tcg_gen_st_i64(arg3->val[1], cpu_env, offsetof(CPUKVXState, scratch[1]));

    gen_helper_ffma_16_16_x8(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffma_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx,
                                                               MDSTypeBinding ret_type,
                                                               MDSTCGBigInt * ret,
                                                               TCGv_i64 arg0,
                                                               MDSTCGBigInt * arg1,
                                                               MDSTCGBigInt * arg2,
                                                               MDSTCGBigInt * arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    tcg_gen_st_i64(arg3->val[0], cpu_env, offsetof(CPUKVXState, scratch[0]));
    tcg_gen_st_i64(arg3->val[1], cpu_env, offsetof(CPUKVXState, scratch[1]));

    gen_helper_ffma_32_32_x4(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffms_16_16_x8_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx,
                                                               MDSTypeBinding ret_type,
                                                               MDSTCGBigInt * ret,
                                                               TCGv_i64 arg0,
                                                               MDSTCGBigInt * arg1,
                                                               MDSTCGBigInt * arg2,
                                                               MDSTCGBigInt * arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    tcg_gen_st_i64(arg3->val[0], cpu_env, offsetof(CPUKVXState, scratch[0]));
    tcg_gen_st_i64(arg3->val[1], cpu_env, offsetof(CPUKVXState, scratch[1]));

    gen_helper_ffms_16_16_x8(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffms_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx,
                                                               MDSTypeBinding ret_type,
                                                               MDSTCGBigInt * ret,
                                                               TCGv_i64 arg0,
                                                               MDSTCGBigInt * arg1,
                                                               MDSTCGBigInt * arg2,
                                                               MDSTCGBigInt * arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    tcg_gen_st_i64(arg3->val[0], cpu_env, offsetof(CPUKVXState, scratch[0]));
    tcg_gen_st_i64(arg3->val[1], cpu_env, offsetof(CPUKVXState, scratch[1]));

    gen_helper_ffms_32_32_x4(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fadd_64_64_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                        MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                        TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fadd_64_64_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fadd_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                        MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                        TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fadd_32_32_x4(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_faddcc_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                            TCGv_i64 arg2)
{
    gen_helper_faddcc_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_faddcc_64_64_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                       MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                       TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_faddcc_64_64(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_faddcc_32_32_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                          MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                          TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_faddcc_32_32_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fsbf_64_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2)
{
    gen_helper_fsbf_64_64(ret, arg0, arg1, arg2);
}

void gen_apply_fsbf_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2)
{
    gen_helper_fsbf_32_32_x2(ret, arg0, arg1, arg2);
}

void gen_apply_fsbf_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type,
                             TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                             TCGv_i64 arg2)
{
    gen_helper_fsbf_16_16_x4(ret, arg0, arg1, arg2);
}

void gen_apply_fsbf_16_16_x8_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                        MDSTypeBinding ret_type, MDSTCGBigInt * ret,
                                                        TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fsbf_16_16_x8(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fsbf_64_64_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                        MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                        TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fsbf_64_64_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fsbf_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                        MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                        TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fsbf_32_32_x4(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fsbfcc_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                            TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                            TCGv_i64 arg2)
{
    gen_helper_fsbfcc_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_fsbfcc_64_64_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                       MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                       TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fsbfcc_64_64(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fsbfcc_32_32_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                          MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                          TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fsbfcc_32_32_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffma_16_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffma_16_32(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffma_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffma_32_32(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffma_32_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffma_32_64(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffma_32_64_x2_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx,
                                                             MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                             TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_ffma_32_64_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), arg2, arg3);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffma_32_64_x2_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx,
                                                           MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                           TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                           TCGv_i64 arg2, int64_t imm3)
{
    TCGv_i64 arg3 = tcg_const_i64(imm3);

    gen_apply_ffma_32_64_x2_bigint_tcg64_bigint_tcg64_tcg64(ctx, ret_type, ret, arg0,
                                                            arg1, arg2, arg3);

    tcg_temp_free_i64(arg3);
}


void gen_apply_ffma_16_32_x4_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx,
                                                             MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                             TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_ffma_16_32_x4(cpu_env, arg0, BI_EXPAND(arg1, 2), arg2, arg3);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffma_16_32_x4_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx,
                                                           MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                           TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                           TCGv_i64 arg2, int64_t imm3)
{
    TCGv_i64 arg3 = tcg_const_i64(imm3);

    gen_apply_ffma_16_32_x4_bigint_tcg64_bigint_tcg64_tcg64(ctx, ret_type, ret, arg0,
                                                            arg1, arg2, arg3);

    tcg_temp_free_i64(arg3);
}

void gen_apply_ffms_16_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffms_16_32(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffms_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffms_32_32(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffms_32_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_ffms_32_64(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_ffms_32_64_x2_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx,
                                                             MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                             TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_ffms_32_64_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), arg2, arg3);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffms_32_64_x2_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx,
                                                           MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                           TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                           TCGv_i64 arg2, int64_t imm3)
{
    TCGv_i64 arg3 = tcg_const_i64(imm3);

    gen_apply_ffms_32_64_x2_bigint_tcg64_bigint_tcg64_tcg64(ctx, ret_type, ret,
                                                            arg0, arg1, arg2, arg3);

    tcg_temp_free_i64(arg3);
}

void gen_apply_ffms_16_32_x4_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx,
                                                             MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                             TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                             TCGv_i64 arg2, TCGv_i64 arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_ffms_16_32_x4(cpu_env, arg0, BI_EXPAND(arg1, 2), arg2, arg3);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_ffms_16_32_x4_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx,
                                                           MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                           TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                           TCGv_i64 arg2, int64_t imm3)
{
    TCGv_i64 arg3 = tcg_const_i64(imm3);

    gen_apply_ffms_16_32_x4_bigint_tcg64_bigint_tcg64_tcg64(ctx, ret_type, ret,
                                                            arg0, arg1, arg2, arg3);

    tcg_temp_free_i64(arg3);
}

void gen_apply_fadd_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2)
{
    gen_helper_fadd_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_fsbf_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2)
{
    gen_helper_fsbf_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_fmul_16_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2)
{
    gen_helper_fmul_16_32(ret, arg0, arg1, arg2);
}

void gen_apply_fmul_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2)
{
    gen_helper_fmul_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_fmul_32_64(DisasContext *ctx, MDSTypeBinding ret_type,
                          TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                          TCGv_i64 arg2)
{
    gen_helper_fmul_32_64(ret, arg0, arg1, arg2);
}

void gen_apply_fdot2_32_32(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                           TCGv_i64 arg2)
{
    gen_helper_fdot2_32_32(ret, arg0, arg1, arg2);
}

void gen_apply_fdot2_32_64(DisasContext *ctx, MDSTypeBinding ret_type,
                           TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                           TCGv_i64 arg2)
{
    gen_helper_fdot2_32_64(ret, arg0, arg1, arg2);
}

void gen_apply_fdot2_32_32_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                         MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                         TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fdot2_32_32_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fdot2_32_64_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx,
                                                         MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                         TCGv_i64 arg0, MDSTCGBigInt *arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fdot2_32_64_x2(cpu_env, arg0, BI_EXPAND(arg1, 2), BI_EXPAND(arg2, 2));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmm2awq_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx,
                                                       MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                       TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                       TCGv_i64 arg2, TCGv_i64 arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmm2awq(cpu_env, arg0, BI_EXPAND(arg1, 2), arg2, arg3);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmm2swq_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx,
                                                       MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                       TCGv_i64 arg0, MDSTCGBigInt *arg1,
                                                       TCGv_i64 arg2, TCGv_i64 arg3)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmm2swq(cpu_env, arg0, BI_EXPAND(arg1, 2), arg2, arg3);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fmm2wq_bigint_tcg64_tcg64_tcg64(DisasContext *ctx,
                                               MDSTypeBinding ret_type,
                                               MDSTCGBigInt *ret, TCGv_i64 arg0,
                                               TCGv_i64 arg1, TCGv_i64 arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fmm2wq(cpu_env, arg0, arg1, arg2);

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fdot4add_16_32_reset(DisasContext *ctx, MDSTypeBinding ret_type,
                                    TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1,
                                    TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_fdot4add_16_32_reset(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_fdot4add_16_32_noreset(DisasContext *ctx,
                                      MDSTypeBinding ret_type, TCGv_i64 ret,
                                      TCGv_i64 arg0, TCGv_i64 arg1,
                                      TCGv_i64 arg2, TCGv_i64 arg3)
{
    gen_helper_fdot4add_16_32_noreset(ret, arg0, arg1, arg2, arg3);
}

void gen_apply_fpow2scale_32_32_x8_bigint_tcg64_tcg64_bigint(DisasContext *ctx,
                                                             MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                             TCGv_i64 arg0, TCGv_i64 arg1, MDSTCGBigInt *arg2)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 256);

    gen_helper_fpow2scale_32_32_x8(cpu_env, arg0, arg1, BI_EXPAND(arg2, 4));

    LOAD_HELPER_RESULT(ret, 4);
}

void gen_apply_frelu_32_32_x8_noreset_bigint_bigint(DisasContext *ctx,
                                                    MDSTypeBinding ret_type,
                                                    MDSTCGBigInt *ret,
                                                    MDSTCGBigInt *arg0)
{
    g_assert(mds_tcg_bigint_get_size(ret) == 256);

    gen_helper_frelu_32_32_x8(cpu_env, BI_EXPAND(arg0, 4));

    LOAD_HELPER_RESULT(ret, 4);
}

void gen_apply_fnarrow_32_16_x8_part0_bigint_tcg64_bigint(DisasContext *ctx,
                                                          MDSTypeBinding ret_type, MDSTCGBigInt *ret,
                                                          TCGv_i64 arg0, MDSTCGBigInt *arg1)
{
    g_assert(mds_tcg_bigint_get_size(arg1) == 256);
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fnarrow_32_16_x8_part0(cpu_env, arg0, BI_EXPAND(arg1, 4));

    LOAD_HELPER_RESULT(ret, 2);
}

void gen_apply_fnarrow_32_16_x8_part1_bigint_tcg64_bigint(DisasContext *ctx,
                                                          MDSTypeBinding ret_type, MDSTCGBigInt * ret,
                                                          TCGv_i64 arg0, MDSTCGBigInt *arg1)
{
    g_assert(mds_tcg_bigint_get_size(arg1) == 256);
    g_assert(mds_tcg_bigint_get_size(ret) == 128);

    gen_helper_fnarrow_32_16_x8_part1(cpu_env, arg0, BI_EXPAND(arg1, 4));

    LOAD_HELPER_RESULT(ret, 2);
}


void gen_test_floatcomp_16(DisasContext *ctx,
                           MDSTypeBinding ret_type, TCGv_i64 ret,
                           uint64_t imm0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    TCGv_i64 arg0 = tcg_const_i64(imm0);

    gen_helper_floatcomp_16(ret, arg0, arg1, arg2);

    tcg_temp_free_i64(arg0);
}

void gen_test_floatcomp_32(DisasContext *ctx,
                           MDSTypeBinding ret_type, TCGv_i64 ret,
                           uint64_t imm0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    TCGv_i64 arg0 = tcg_const_i64(imm0);

    gen_helper_floatcomp_32(ret, arg0, arg1, arg2);

    tcg_temp_free_i64(arg0);
}

void gen_test_floatcomp_64(DisasContext *ctx,
                           MDSTypeBinding ret_type, TCGv_i64 ret,
                           uint64_t imm0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    TCGv_i64 arg0 = tcg_const_i64(imm0);

    gen_helper_floatcomp_64(ret, arg0, arg1, arg2);

    tcg_temp_free_i64(arg0);
}

void gen_throw_OPCODE(DisasContext *ctx)
{
    gen_raise_trap(ctx, TRAP_OPCODE);
}

void gen_test_rfe_owner(DisasContext *ctx,
                        MDSTypeBinding ret_type, TCGv_i64 ret)
{
    TCGv_i64 syndrome;

    gen_fill_syndrome_pic(ctx, ES_PIC_RFE);
    gen_save_pc(ctx, ctx->bundle.pc); // in case we trap

    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_test_rfe_owner(ret, cpu_env, syndrome);
    tcg_temp_free_i64(syndrome);
}

void gen_effect_rfe(DisasContext *ctx)
{
    TCGv_i64 syndrome;

    // we already saved PC
    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_rfe(cpu_env, syndrome);
    tcg_temp_free_i64(syndrome);

    end_tb(ctx, DISAS_EXIT);
}

void gen_effect_syscall(DisasContext *ctx, TCGv_i64 num)
{
    if (semihosting_enabled()) {
        gen_raise_syscall_semi(ctx, num);
    } else {
        gen_raise_syscall(ctx, num);
    }

    end_tb(ctx, DISAS_EXIT);
}

void gen_apply_waitit(DisasContext *ctx,
                      MDSTypeBinding ret_type, TCGv_i64 ret,
                      TCGv_i64 or_mask, TCGv_i64 and_mask)
{
    /*
     * The waitit instruction generated implementation is overriden by a
     * manually written one. See translate-insn.override.c. This MDS helper
     * should not be called.
     */
    g_assert_not_reached();
}

void gen_test_stop_owner(DisasContext *ctx,
                         MDSTypeBinding ret_type, TCGv_i64 ret)
{
    TCGv_i64 syndrome;

    gen_fill_syndrome_pic(ctx, ES_PIC_STOP);
    gen_save_pc(ctx, ctx->bundle.pc); // in case we trap

    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_test_stop_owner(ret, cpu_env, syndrome);
    tcg_temp_free_i64(syndrome);
}

void gen_effect_idle(DisasContext *ctx, int64_t lvl)
{
    TCGv_i64 lvl_tcg;

    if ((1 << lvl) & ctx->wu) {
        /* nop when the corresponding WS.WUx bit is set */
        return;
    }

    gen_save_pc(ctx, ctx->next_bundle_pc);

    lvl_tcg = tcg_const_i64(lvl);
    gen_helper_idle(cpu_env, lvl_tcg);
    tcg_temp_free_i64(lvl_tcg);

    end_tb(ctx, DISAS_NORETURN);
}

void gen_test_syncgroup_owner(DisasContext *ctx,
                              MDSTypeBinding ret_type, TCGv_i64 ret)
{
    TCGv_i64 syndrome;

    gen_fill_syndrome_pic(ctx, ES_PIC_SYNCGROUP);
    gen_save_pc(ctx, ctx->bundle.pc); // in case we trap

    syndrome = tcg_const_i64(get_syndrome(ctx));
    gen_helper_test_stop_owner(ret, cpu_env, syndrome);
    tcg_temp_free_i64(syndrome);
}

void gen_effect_syncgroup(DisasContext *ctx,
                          TCGv_i64 clr_fwd, TCGv_i64 clr_bwd,
                          TCGv_i64 notify_fwd, TCGv_i64 notify_bwd)
{
    /*
     * Set PC to the current bundle address. In case of a wakeup event due to
     * e.g. a more priviledge IRQ, this instruction will get re-executed and
     * the core will go back to sleep. The KVX_SYNCGROUP wake up logic will
     * take care of incrementing PC when the real wake up condition is
     * fulfilled.
     */
    gen_save_pc(ctx, ctx->bundle.pc);
    gen_helper_syncgroup(cpu_env, clr_fwd, clr_bwd, notify_fwd, notify_bwd);
    end_tb(ctx, DISAS_NORETURN);
}

void gen_effect_barrier(DisasContext *ctx) {}
void gen_effect_MEM_dtouchl(DisasContext *ctx, TCGv_i64 arg0, uint64_t arg1) {}
void gen_effect_MEM_dinvall(DisasContext *ctx, TCGv_i64 arg0, uint64_t arg1) {}
void gen_effect_MEM_iinvals(DisasContext *ctx, TCGv_i64 arg0, uint64_t arg1) {}
void gen_effect_MEM_dinval(DisasContext *ctx) {}
void gen_effect_MEM_iinval(DisasContext *ctx) {}
void gen_effect_MEM_fence(DisasContext *ctx) {}

void gen_apply_recvv_bigint_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, uint64_t arg0) {}
void gen_effect_break(DisasContext *ctx, uint64_t arg0) {}
void gen_effect_sendv_bigint_u64(DisasContext *ctx, MDSTCGBigInt * arg0, uint64_t arg1) {}
