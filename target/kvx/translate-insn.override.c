#include "translate.h"
#include "gen/translate-insn.h"
#include "exec/helper-proto.h"

static void trans_add_with_carry(DisasContext *ctx,
                                 struct ops_kv3_registerW_kv3_registerZ_kv3_registerY *fmt,
                                 bool initial,
                                 bool complement_z)
{
    TCGv_i64 a0, a1, carry_in, carry_out, counter;

    a0 = tcg_temp_new_i64();
    gen_load_operand(ctx, a0, REGCLASS_kv3_singleReg, fmt->kv3_registerZ);

    a1 = tcg_temp_new_i64();
    gen_load_operand(ctx, a1, REGCLASS_kv3_singleReg, fmt->kv3_registerY);

    carry_in = tcg_temp_new_i64();
    gen_load_storage(ctx, carry_in, STORAGE_kv3_CS, 0, 1);

    if (complement_z) {
        tcg_gen_not_i64(a0, a0);
    }

    tcg_gen_add_i64(a0, a0, a1);

    carry_out = tcg_temp_new_i64();
    tcg_gen_setcond_i64(TCG_COND_LTU, carry_out, a0, a1);
    tcg_temp_free_i64(a1);

    if (complement_z && initial) {
        tcg_gen_addi_i64(a0, a0, 1);

        tcg_gen_setcondi_i64(TCG_COND_LTU, carry_in, a0, 1);
        tcg_gen_or_i64(carry_out, carry_out, carry_in);

    } else if (!initial) {
        tcg_gen_add_i64(a0, a0, carry_in);

        tcg_gen_setcond_i64(TCG_COND_LTU, carry_in, a0, carry_in);
        tcg_gen_or_i64(carry_out, carry_out, carry_in);
    }

    tcg_temp_free_i64(carry_in);

    gen_store_operand(ctx, a0, REGCLASS_kv3_singleReg, fmt->kv3_registerW);
    tcg_temp_free_i64(a0);

    gen_store_storage(ctx, carry_out, STORAGE_kv3_CS, 0, 1);

    counter = tcg_temp_new_i64();
    gen_load_storage(ctx, counter, STORAGE_kv3_CS, 32, 16);

    tcg_gen_add_i64(counter, counter, carry_out);
    tcg_temp_free_i64(carry_out);

    gen_store_storage(ctx, counter, STORAGE_kv3_CS, 32, 16);
    tcg_temp_free_i64(counter);
}


/*
 * Overridden because the MDS description reads the 65th bit of the result as
 * the carry. This is not supported by the generator that works on 64 bits
 * values in this case.
 */
bool trans_v1_ADDCD_registerW_registerZ_registerY_simple(DisasContext *ctx,
                                                         struct ops_kv3_registerW_kv3_registerZ_kv3_registerY *fmt)
{
    trans_add_with_carry(ctx, fmt, false, false);
    return true;
}

/*
 * Overridden for the same reason as ADDCD
 */
bool trans_v1_ADDCD_I_registerW_registerZ_registerY_simple(DisasContext *ctx,
                                                          struct ops_kv3_registerW_kv3_registerZ_kv3_registerY *fmt)
{
    trans_add_with_carry(ctx, fmt, true, false);
    return true;
}

/*
 * Overridden for the same reason as ADDCD
 */
bool trans_v1_SBFCD_registerW_registerZ_registerY_simple(DisasContext *ctx,
                                                         struct ops_kv3_registerW_kv3_registerZ_kv3_registerY *fmt)
{
    trans_add_with_carry(ctx, fmt, false, true);
    return true;
}

/*
 * Overridden for the same reason as ADDCD
 */
bool trans_v1_SBFCD_I_registerW_registerZ_registerY_simple(DisasContext *ctx,
                                                          struct ops_kv3_registerW_kv3_registerZ_kv3_registerY *fmt)
{
    trans_add_with_carry(ctx, fmt, true, true);
    return true;
}

/*
 * ABBD instruction
 *
 * Overridden because the subtraction must be done on full precision.
 * Problems arise when the subtraction overflows on 64 bits. The full precision
 * result is negative, but the truncated one become positive. The _ABS operator
 * then returns the same value, but should return the 2's complement value on
 * 64 bits.
 *
 * Example:
 *   make $r1 = 0x8000000000000000  # (INT64_MIN)
 *   make $r2 = 0x7fffffffffffffff  # (INT64_MAX)
 *   ;;
 *
 *   abbd $r0 = $r1, $r2
 *   ;;
 *
 *   # correct result: $r0 == 0xffffffffffffffff
 *   # wrong result:   $r0 == 0x1
 *
 *   $r0 should contain UINT64_MAX, which is also "INT65_MAX" == "abs(INT65_MIN + 1)"
 */

uint64_t HELPER(abdd)(int64_t a0, int64_t a1)
{
    int64_t res = a0 - a1;
    uint64_t ov;

    uint64_t sa0 = ((uint64_t)a0) >> 63;
    uint64_t sa1 = ((uint64_t)-a1) >> 63;
    uint64_t sres = ((uint64_t) res) >> 63;

    ov = (sa0 == sa1) && (sa0 != sres);

    return (ov != sres) ? -res : res;
}

bool trans_v1_ABDD_registerW_registerZ_registerY_simple(DisasContext *ctx,
                                                        struct ops_kv3_registerW_kv3_registerZ_kv3_registerY *fmt)
{
    TCGv_i64 a0, a1;

    a0 = tcg_temp_new_i64();
    gen_load_operand(ctx, a0, REGCLASS_kv3_singleReg, fmt->kv3_registerY);

    a1 = tcg_temp_new_i64();
    gen_load_operand(ctx, a1, REGCLASS_kv3_singleReg, fmt->kv3_registerZ);

    gen_helper_abdd(a0, a0, a1);
    tcg_temp_free_i64(a1);

    gen_store_operand(ctx, a0, REGCLASS_kv3_singleReg, fmt->kv3_registerW);
    tcg_temp_free_i64(a0);

    return true;
}

bool trans_v1_ABDD_registerW_registerZ_signed10_simple(DisasContext *ctx,
                                                       struct ops_kv3_registerW_kv3_registerZ_kv3_signed10 *fmt)
{
    TCGv_i64 a0, a1;

    a0 = tcg_const_i64(fmt->kv3_signed10);

    a1 = tcg_temp_new_i64();
    gen_load_operand(ctx, a1, REGCLASS_kv3_singleReg, fmt->kv3_registerZ);

    gen_helper_abdd(a0, a0, a1);
    tcg_temp_free_i64(a1);

    gen_store_operand(ctx, a0, REGCLASS_kv3_singleReg, fmt->kv3_registerW);
    tcg_temp_free_i64(a0);

    return true;
}

bool trans_v1_ABDD_registerW_registerZ_upper27_lower10_double(DisasContext *ctx,
                                                              struct ops_kv3_registerW_kv3_registerZ_kv3_upper27_lower10 *fmt)
{
    TCGv_i64 a0, a1;

    a0 = tcg_const_i64(fmt->kv3_upper27_lower10);

    a1 = tcg_temp_new_i64();
    gen_load_operand(ctx, a1, REGCLASS_kv3_singleReg, fmt->kv3_registerZ);

    gen_helper_abdd(a0, a0, a1);
    tcg_temp_free_i64(a1);

    gen_store_operand(ctx, a0, REGCLASS_kv3_singleReg, fmt->kv3_registerW);
    tcg_temp_free_i64(a0);

    return true;
}

bool trans_v1_ABDD_registerW_registerZ_extend27_upper27_lower10_triple(DisasContext *ctx,
                                                                       struct ops_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10 *fmt)
{
    TCGv_i64 a0, a1;

    a0 = tcg_const_i64(fmt->kv3_extend27_upper27_lower10);

    a1 = tcg_temp_new_i64();
    gen_load_operand(ctx, a1, REGCLASS_kv3_singleReg, fmt->kv3_registerZ);

    gen_helper_abdd(a0, a0, a1);
    tcg_temp_free_i64(a1);

    gen_store_operand(ctx, a0, REGCLASS_kv3_singleReg, fmt->kv3_registerW);
    tcg_temp_free_i64(a0);

    return true;
}

/*
 * Overridden because PS.HLE is part of the TB flags. When HL are disabled, we
 * can simply generate an OPCODE trap and nothing else.
 * The generated code generates a if...else construct which does not work
 * because the trap is conditional, but unconditionally triggers
 * a end_tb(ctx, DISAS_NORETURN).
 */
bool trans_v1_LOOPDO_registerZ_pcrel17_simple(DisasContext *ctx,
                                              struct ops_kv3_registerZ_kv3_pcrel17 *fmt)
{
    int64_t pcrel;
    TCGv_i64 tmp;

    if (!ctx->hardware_loop_enabled) {
        gen_raise_trap(ctx, TRAP_OPCODE);
        end_tb(ctx, DISAS_NORETURN);
        return true;
    }

    pcrel = fmt->kv3_pcrel17;
    pcrel <<= 0x2f;
    pcrel >>= 0x2f;

    tmp = tcg_temp_new_i64();

    gen_load_storage(ctx, tmp, STORAGE_kv3_NPC, 0, 1);
    gen_store_regfile(ctx, tmp, REGFILE_kv3_SFR, 7, 1); /* LS */

    gen_load_storage(ctx, tmp, STORAGE_kv3_PC, 0, 1);
    tcg_gen_addi_i64(tmp, tmp, pcrel);
    gen_store_regfile(ctx, tmp, REGFILE_kv3_SFR, 8, 1); /* LE */

    gen_load_operand(ctx, tmp, REGCLASS_kv3_singleReg, fmt->kv3_registerZ);
    gen_store_regfile(ctx, tmp, REGFILE_kv3_SFR, 9, 1); /* LC */

    tcg_temp_free_i64(tmp);

    return true;
}

bool trans_v2_LOOPDO_registerZ_pcrel17_simple(DisasContext *ctx,
                                              struct ops_kv3_registerZ_kv3_pcrel17 *fmt)
{
    return trans_v1_LOOPDO_registerZ_pcrel17_simple(ctx, fmt);
}

/*
 * Overridden because we need to know the waitit operand. It needs to be stored
 * in the CPU state to be written when the CPU wakes up from IRQ.
 */
bool trans_v1_WAITIT_registerZ_simple(DisasContext *ctx, struct ops_kv3_registerZ *fmt)
{
    TCGv_i64 and_mask;
    TCGv_i64 or_mask;
    TCGv_i64 operand;

    /*
     * Set PC to current bundle address. The bundle will get re-executed if
     * the CPU wakes up to handle an IRQ owned by a more priviledged PL.
     */
    gen_save_pc(ctx, ctx->bundle.pc);

    and_mask = tcg_temp_new_i64();
    gen_load_operand(ctx, and_mask, REGCLASS_kv3_singleReg, fmt->kv3_registerZ);

    or_mask = tcg_temp_new_i64();
    tcg_gen_mov_i64(or_mask, and_mask);

    tcg_gen_andi_i64(or_mask, or_mask, ((int64_t)0xffffffff));
    tcg_gen_shri_i64(and_mask, and_mask, ((int64_t)0x20));
    tcg_gen_andi_i64(and_mask, and_mask, ((int64_t)0xffffffff));

    operand = tcg_const_i64(fmt->kv3_registerZ);

    gen_helper_waitit(cpu_env, or_mask, and_mask, operand);
    end_tb(ctx, DISAS_NORETURN);

    tcg_temp_free_i64(operand);
    tcg_temp_free_i64(or_mask);
    tcg_temp_free_i64(and_mask);

    return true;
}

bool trans_v2_WAITIT_registerZ_simple(DisasContext *ctx, struct ops_kv3_registerZ *fmt)
{
    return trans_v1_WAITIT_registerZ_simple(ctx, fmt);
}
