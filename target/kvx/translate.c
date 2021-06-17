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
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#include "exec/log.h"

#include "translate.h"
#include "gen/decode.h"

static void kvx_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = cs->env_ptr;

    ctx->model = cpu->model;
    ctx->mem_index = ctx->base.tb->flags & R_TB_STATE_MMU_IDX_MASK;
    ctx->bundle.len = 0;
    ctx->hardware_loop_enabled = kvx_hardware_loop_enabled(env);
    ctx->gen_hardware_loop = kvx_in_hardware_loop_ctx(env);
    ctx->v64 = FIELD_EX64(ctx->base.tb->flags, TB_STATE, V64);
    ctx->wu = FIELD_EX64(ctx->base.tb->flags, TB_STATE, WU);
    ctx->arith_irq_enabled = FIELD_EX64(ctx->base.tb->flags, TB_STATE, ARITH_IRQ);
    ctx->cs_mask = 0;
    ctx->jump_cond_flag_valid = false;

    if (ctx->gen_hardware_loop) {
        ctx->hardware_loop_pc = kvx_register_read_u64(env, REG_kv3_LE);
    }

    ctx->step_mode_enabled = FIELD_EX64(ctx->base.tb->flags, TB_STATE, SME);
    /*
     * In single step mode, we only put one instruction per tb.
     * Then we generate a debug step exception.
     */
    if (ctx->step_mode_enabled) {
        ctx->base.max_insns = 1;
    }

    ctx->data_cache_enabled = FIELD_EX64(ctx->base.tb->flags, TB_STATE, DCE);

    store_buffer_reset(ctx);
}

static void kvx_tr_tb_start(DisasContextBase *db, CPUState *cpu)
{
}

static void kvx_tr_insn_start(DisasContextBase *dcbase, CPUState *cpu)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(ctx->base.pc_next, 0);
    ctx->start_op = tcg_last_op();
}

static inline void gen_hardware_loop(DisasContext *ctx)
{
    if (!ctx->gen_hardware_loop) {
        return;
    }

    if (ctx->next_bundle_pc != ctx->hardware_loop_pc) {
        return;
    }

    TCGv_i64 hw_loop_pc = tcg_const_i64(ctx->hardware_loop_pc);

    if (ctx->jump_cond_flag_valid) {
        gen_helper_check_hw_loop(cpu_env, hw_loop_pc, ctx->jump_cond_flag);
        tcg_temp_free_i64(ctx->jump_cond_flag);
        ctx->jump_cond_flag_valid = false;
    } else {
        TCGv_i64 tcg_skip_jmp;
        bool skip_jmp;

        skip_jmp = (ctx->base.is_jmp == DISAS_JUMP) || (ctx->base.is_jmp == DISAS_EXIT);
        tcg_skip_jmp = tcg_const_i64(skip_jmp);
        gen_helper_check_hw_loop(cpu_env, hw_loop_pc, tcg_skip_jmp);
        tcg_temp_free_i64(tcg_skip_jmp);
    }

    end_tb(ctx, DISAS_EXIT);

    tcg_temp_free_i64(hw_loop_pc);
}

static void check_arithmetic_irq(DisasContext *ctx)
{
    uint64_t cs_mask;
    TCGv_i64 pc, cs_mask_tcg;

    if (!ctx->arith_irq_enabled) {
        return;
    }

    cs_mask = ctx->cs_mask & KVX_CSIT_IRQ_MASK;

    if (!cs_mask) {
        return;
    }

    cs_mask_tcg = tcg_const_i64(cs_mask);
    pc = tcg_const_i64(ctx->bundle.pc);

    gen_helper_check_arith_irq(cpu_env, cs_mask_tcg, pc);

    tcg_temp_free_i64(pc);
    tcg_temp_free_i64(cs_mask_tcg);

    end_tb(ctx, DISAS_UPDATE);
}

typedef bool (*DecodeFunc)(DisasContext *ctx, const uint32_t *buf);
typedef struct DecodeFuncs {
    DecodeFunc simple;
    DecodeFunc double_;
    DecodeFunc triple;
} DecodeFuncs;

static const DecodeFuncs DECODE_FUNCS[] = {
    [CPU_MODEL_v1] = {
        .simple = decode_v1_simple,
        .double_ = decode_v1_double,
        .triple = decode_v1_triple,
    },

    [CPU_MODEL_v2] = {
        .simple = decode_v2_simple,
        .double_ = decode_v2_double,
        .triple = decode_v2_triple,
    },
};

static void decode_opcode(DisasContext *ctx, Opcode *opcode)
{
    const DecodeFuncs *decode = &DECODE_FUNCS[ctx->model];

    switch (opcode->len) {
    default:
        if (decode->triple(ctx, opcode->val)) {
            return;
        }

        /* fall through */
    case 2:
        if (decode->double_(ctx, opcode->val)) {
            return;
        }

        /* fall through */
    case 1:
        if (decode->simple(ctx, opcode->val)) {
            return;
        }
    }

    gen_raise_trap(ctx, TRAP_OPCODE);
}

static void pack_and_set_insn_syndrome(DisasContext *ctx)
{
    uint64_t s = get_syndrome(ctx);

    if (!KVX_FIELD_EX64(s, kv3_ES, AS)) {
        /*
         * This is not a memory access. Packing is done for memory accesses
         * only.
         */
        return;
    }

    tcg_set_insn_start_param(ctx->start_op, 1, insn_syndrome_pack(s));
}

static void translate_bundle(DisasContext *ctx)
{
    Bundle *bundle = &ctx->bundle;
    BundleUnit unit;

    for (unit = 0; unit < NUM_UNITS; unit++) {
        Opcode *opcode = &bundle->decoded[unit];
        if (opcode->len) {
            ctx->cur_opcode = opcode;

            decode_opcode(ctx, opcode);

            pack_and_set_insn_syndrome(ctx);

            /*
             * We advance by one syllable, even if the opcode has extended
             * immediates.
             */
            ctx->base.pc_next += 4;
        }
    }
}

typedef enum BundleDecodeState {
    DEC_NONE = -1,
    /* Stay aligned with BundleUnit */
    DEC_BCU = 0, DEC_TCA, DEC_ALU0, DEC_ALU1, DEC_MAU, DEC_LSU, DEC_EXT
} BundleDecodeState;

static const BundleUnit BUNDLE_SLOT_STEERING[] = {
    [0x0] = UNIT_BCU,
    [0x1] = UNIT_LSU,
    [0x2] = UNIT_MAU,
    [0x3] = UNIT_ALU0,
};

static const BundleUnit BUNDLE_SLOT_EXNUM[] = {
    [0x0] = UNIT_ALU0,
    [0x1] = UNIT_ALU1,
    [0x2] = UNIT_MAU,
    [0x3] = UNIT_LSU,
};

static inline BundleUnit extract_steering(uint32_t opcode)
{
    uint32_t steering = extract32(opcode, 29, 2);
    return BUNDLE_SLOT_STEERING[steering];
}

static inline BundleUnit extract_exnum(uint32_t opcode)
{
    uint32_t exnum = extract32(opcode, 27, 2);
    return BUNDLE_SLOT_EXNUM[exnum];
}

static inline bool opcode_is_tca(uint32_t opcode) {
    uint32_t tcacode = extract32(opcode, 24, 3);
    uint32_t exnum = extract32(opcode, 27, 2);

    return exnum == 0 && tcacode > 1;
}

static inline BundleDecodeState get_next_state(BundleUnit steering,
                                               BundleDecodeState prev,
                                               bool is_tca)
{
    BundleDecodeState state = steering;

    if (state <= prev) {
        if (prev == DEC_BCU && state == DEC_BCU && is_tca) {
            /*
             * TCA opcode uses the BCU steering and comes just after the BCU
             * opcode.
             */
            return DEC_TCA;
        }

        if (state == DEC_BCU) {
            /*
             * We went over the instruction syllables. Now come the
             * immediate extensions (which have the same steering value as
             * the BCU instructions).
             */
            return DEC_EXT;
        }

        if (state == DEC_ALU0 && prev < DEC_LSU) {
            /*
             * ALU instructions can go on ALU0, ALU1 or the tiny ALUs of MAU and
             * LSU. XXX This would require further checks to see if the opcode
             * can actually go on the requested ALU.
             */
            return prev + 1;
        }

        /* The other cases lead to an invalid bundle */
        return DEC_NONE;
    }

    return state;
}

static inline BundleUnit get_next_unit(BundleDecodeState state, uint32_t opcode)
{
    if (state == DEC_EXT) {
        return extract_exnum(opcode);
    }

    return state;
}


static inline void push_syllable(DisasContext *ctx, BundleUnit unit,
                                 uint32_t word, bool ext)
{
    Bundle *bundle = &ctx->bundle;
    Opcode *opcode = &bundle->decoded[unit];

    if (ext && !opcode->len) {
        /*
         * If the unit has not been allocated by an instruction syllable,
         * simply skip the extended ones.
         */
        return;
    }

    opcode->val[opcode->len++] = word;
}

static inline void reset_opcodes(DisasContext *ctx)
{
    Bundle *bundle = &ctx->bundle;
    BundleUnit i;

    for (i = 0; i < NUM_UNITS; i++) {
        bundle->decoded[i].len = 0;
    }
}

static inline bool decode_bundle(DisasContext *ctx)
{
    size_t cur;
    BundleDecodeState prev_state = DEC_NONE;
    Bundle *bundle = &ctx->bundle;

    reset_opcodes(ctx);

    for (cur = 0; cur < bundle->len; cur++) {
        uint32_t opcode = bundle->buf[cur];
        BundleUnit steering = extract_steering(opcode);
        bool is_tca = opcode_is_tca(opcode);
        BundleUnit unit;
        BundleDecodeState state = get_next_state(steering, prev_state, is_tca);

        if (state == DEC_NONE) {
            goto invalid_bundle;
        }

        unit = get_next_unit(state, opcode);
        push_syllable(ctx, unit, opcode, state == DEC_EXT);

        prev_state = state;
    }

    return true;

invalid_bundle:
    gen_raise_trap_opcode(ctx);
    return false;
}

#define OPCODE_PARALLEL_MASK 0x80000000

static bool read_next_bundle(DisasContext *ctx, CPUState *cpu)
{
    CPUKVXState *env = cpu->env_ptr;
    Bundle *bundle = &ctx->bundle;
    bool ret = true;

    uint32_t opcode;
    target_ulong pc_next = ctx->base.pc_next;

    /* Start a new bundle */
    bundle->pc = pc_next;
    bundle->len = 0;

    do {
        if (bundle->len == BUNDLE_MAX_SIZE) {
            /* oversized bundle */
            gen_raise_trap_opcode(ctx);
            ret = false;
            break;
        }

        opcode = translator_ldl(env, pc_next);
        bundle->buf[bundle->len++] = opcode;

        pc_next += 4;
    } while (opcode & OPCODE_PARALLEL_MASK);

    ctx->next_bundle_pc = bundle->pc + bundle->len * 4;
    gen_init_syndrome(ctx);
    return ret;
}

static inline bool check_hw_breakpoint(CPUKVXState *env, uint64_t pc, int n)
{
    const struct CPUKVXBreakpoint *bp = &env->breakpoint[n];

    return bp->enabled && pc >= bp->addr_low && pc <= bp->addr_high;
}

static void gen_check_hw_breakpoints(DisasContext *ctx, CPUState *cpu)
{
    CPUKVXState *env = cpu->env_ptr;
    target_ulong pc = ctx->base.pc_next;
    TCGv_i64 es;

    es = tcg_const_i64(get_syndrome(ctx));

    for (int i = 0; i < 2; i++) {
        if (unlikely(check_hw_breakpoint(env, pc, i))) {

            /* PL is checked in the helper */
            gen_save_pc(ctx, ctx->base.pc_next);
            gen_helper_check_raise_debug_breakpoint(cpu_env, es);

            /*
             * End the tb after this insn anyway as we will likely
             * not execute it because of the breakpoint
             */
            ctx->base.is_jmp = DISAS_TOO_MANY;
            break;
        }
    }

    tcg_temp_free_i64(es);
    return;
}

static inline void gen_check_step_mode_ready(DisasContext *ctx, CPUState *cpu)
{
    gen_save_pc(ctx, ctx->base.pc_next);
    gen_helper_check_step_mode_ready(cpu_env);
}

static void kvx_tr_translate_insn(DisasContextBase *dcbase, CPUState *cpu)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    if (unlikely(ctx->step_mode_enabled)) {
        /* if SMR is set, we trap before doing anything else */
        gen_check_step_mode_ready(ctx, cpu);
    }

    if (!read_next_bundle(ctx, cpu)) {
        ctx->base.pc_next = ctx->next_bundle_pc;
        return;
    }

    /* check for breakpoint after fetch */
    gen_check_hw_breakpoints(ctx, cpu);

    if (!decode_bundle(ctx)) {
        ctx->base.pc_next = ctx->next_bundle_pc;
        return;
    }

    translate_bundle(ctx);

    store_buffer_flush(ctx);

    check_arithmetic_irq(ctx);

    gen_hardware_loop(ctx);

    ctx->base.pc_next = ctx->next_bundle_pc;

    if (ctx->base.is_jmp == DISAS_NEXT) {
        target_ulong start_page = ctx->base.pc_first & TARGET_PAGE_MASK;
        target_ulong cur_page = ctx->base.pc_next & TARGET_PAGE_MASK;
        target_ulong next_page = (ctx->base.pc_next + BUNDLE_MAX_SIZE - 1) & TARGET_PAGE_MASK;

        if ((tb_cflags(ctx->base.tb) & CF_USE_ICOUNT)
            && ((start_page != cur_page) || (cur_page != next_page))) {
            /*
             * When using icount, we do not want to cross a page boundary in
             * the middle of a TB. We allow it only if the crossing instruction
             * is the first of the block.
             */
            ctx->base.is_jmp = DISAS_TOO_MANY;
        } else if ((ctx->base.pc_next - ctx->base.pc_first)
                   >= (TARGET_PAGE_MASK - BUNDLE_MAX_SIZE)) {
            /* Otherwise, we limit the size of a TB to about the size
             * of a page */
            ctx->base.is_jmp = DISAS_TOO_MANY;
        }
    }

    translator_loop_temp_check(dcbase);
}

static void kvx_tr_tb_stop(DisasContextBase *dcbase, CPUState *cpu)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    store_buffer_flush(ctx);

    if (unlikely(is_singlestepping(ctx))) {
        switch (ctx->base.is_jmp) {
        case DISAS_TOO_MANY:
        case DISAS_UPDATE:
            gen_save_pc(ctx, ctx->base.pc_next);
            /* fall through */
        case DISAS_EXIT:
        case DISAS_JUMP:
            gen_raise_step_exception(ctx);
            break;
        case DISAS_NORETURN:
            break;
        default:
            g_assert_not_reached();
        }
        return;
    }

    switch (ctx->base.is_jmp) {
    case DISAS_TOO_MANY:
        gen_goto_tb(ctx, 0, ctx->base.pc_next);
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        break;
    case DISAS_UPDATE:
        gen_save_pc(ctx, ctx->base.pc_next);
        /* fall through */
    case DISAS_EXIT:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static void kvx_tr_disas_log(const DisasContextBase *dcbase, CPUState *cpu)
{
    qemu_log("IN: %s\n", lookup_symbol(dcbase->pc_first));
    log_target_disas(cpu, dcbase->pc_first, dcbase->tb->size);
}


static const TranslatorOps kvx_tr_ops = {
    .init_disas_context = kvx_tr_init_disas_context,
    .tb_start           = kvx_tr_tb_start,
    .insn_start         = kvx_tr_insn_start,
    .translate_insn     = kvx_tr_translate_insn,
    .tb_stop            = kvx_tr_tb_stop,
    .disas_log          = kvx_tr_disas_log,
};

void gen_intermediate_code(CPUState *cs, TranslationBlock *tb, int max_insns)
{
    DisasContext ctx;

    translator_loop(&kvx_tr_ops, &ctx.base, cs, tb, max_insns);
}

void kvx_translate_init(void)
{
}

void restore_state_to_opc(CPUKVXState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    kvx_register_write_u64(env, REG_kv3_PC, data[0]);
    env->excp_syndrome = insn_syndrome_unpack(data[1]);
}
