#ifndef _TRANSLATE_H
#define _TRANSLATE_H

#include "qemu/osdep.h"
#include "exec/translator.h"
#include "tcg/tcg-op.h"
#include "exec/gen-icount.h"

#include "internal.h"
#include "mds-big-int.h"
#include "gen/instructions.h"
#include "gen/modifiers.h"

#define BUNDLE_MAX_SIZE 8
#define OPCODE_MAX_SIZE 3
#define ST_BUF_MAX_SIZE 16

/*
 * For BCU instructions which want to exit to the main loop to
 * immediately handle side-effects such as IOs.
 */
#define DISAS_EXIT   DISAS_TARGET_0
#define DISAS_UPDATE DISAS_TARGET_1
#define DISAS_JUMP   DISAS_TARGET_2

typedef enum BundleUnit {
    UNIT_BCU = 0, UNIT_TCA, UNIT_ALU0, UNIT_ALU1, UNIT_MAU, UNIT_LSU,
    NUM_UNITS
} BundleUnit;

typedef enum MDSType {
    MDS_SIGNED,
    MDS_UNSIGNED,
    MDS_BOOL,
} MDSType;

typedef struct MDSTypeBinding {
    MDSType type;
    size_t size;
} MDSTypeBinding;

typedef struct StorageAccessInfo {
    size_t data_size;
    size_t access_size;
    size_t access_count;
    size_t offset;
    size_t sub_offset;

    uint64_t write_mask;

    bool is_sysreg;
    Register reg;
} StorageAccessInfo;

typedef enum StoreBufferEntryValType {
    VAL_TYPE_TCG,
    VAL_TYPE_IMM
} StoreBufferEntryValType;

/*
 * StoreBufferEntry
 *
 * @type  The type of values in this entry
 * @val   Array of values to store
 * @count Number of values
 * @info  Information used during the actual store.
 * @cond  True if the store is conditional, i.e. it may be skipped during
 *        execution. In this case we rely on the @valid flag to check whether
 *        we should do the final store or not.
 * @valid True if the store is valid at run time.
 */
#define STORE_BUFFER_ENTRY_MAX_VAL MDS_TCG_BIGINT_MAX_SZ
typedef struct StoreBufferEntry {
    StoreBufferEntryValType type;
    union {
        TCGv_i64 tcg[STORE_BUFFER_ENTRY_MAX_VAL];
        uint64_t imm[STORE_BUFFER_ENTRY_MAX_VAL];
    } val;
    size_t count;

    StorageAccessInfo *info;

    bool cond;
    TCGv_i64 valid;
} StoreBufferEntry;

/*
 * StoreBufferStore
 *
 * A store at a fixed address. Multiple StoreBufferEntry can map to a single
 * StoreBufferStore
 *
 * @info    Information used during the actual store.
 * @entries The concurrent entries that map to this store. They must be ORed
 *          together to get the final result.
 * @count   The number of entries.
 */
#define STORE_BUFFER_STORE_MAX_ENTRIES NUM_UNITS
typedef struct StoreBufferStore {
    StorageAccessInfo info;

    StoreBufferEntry *entries[STORE_BUFFER_STORE_MAX_ENTRIES];
    size_t count;
} StoreBufferStore;

typedef struct StoreBuffer {
    StoreBufferEntry entries[ST_BUF_MAX_SIZE];
    size_t num_entry;

    StoreBufferStore stores[ST_BUF_MAX_SIZE];
    size_t num_store;
} StoreBuffer;

typedef struct Opcode {
    uint32_t val[OPCODE_MAX_SIZE];
    size_t len;
    InsnFormat_kv3 format;
    Insn_kv3 insn;
} Opcode;

typedef struct Bundle {
    target_ulong pc;

    uint32_t buf[BUNDLE_MAX_SIZE];
    size_t len;
    uint64_t syndrome;

    Opcode decoded[NUM_UNITS];
} Bundle;

typedef struct DisasContext {
    DisasContextBase base;

    int mem_index;

    Bundle bundle;
    Opcode *cur_opcode;
    TCGOp *start_op;
    target_ulong next_bundle_pc;

    StoreBuffer store_buffer;

    bool hardware_loop_enabled;
    bool gen_hardware_loop;
    target_ulong hardware_loop_pc;

    bool v64;
    uint8_t wu;
    bool data_cache_enabled;

    unsigned set_gpr_idx;

    bool arith_irq_enabled;
    uint64_t cs_mask;

    bool step_mode_enabled;
} DisasContext;


static inline void gen_store_register_no_delay(DisasContext *ctx, TCGv_i64 src, Register reg);
static inline void gen_load_register(DisasContext *ctx, TCGv_i64 dest, Register reg);

/*
 * Set ctx->base.is_jmp value. Since we are targeting a VLIW architecture,
 * multiple instructions may want to set is_jmp in the same bundle. This
 * function takes care of the priority of is_jmp values.
 */
static inline void end_tb(DisasContext *ctx, int is_jmp)
{
    static const int IS_JMP_ORDER[] = {
        [DISAS_NEXT] = 0,
        [DISAS_JUMP] = 1,
        [DISAS_UPDATE] = 2,
        [DISAS_EXIT] = 3,
        [DISAS_NORETURN] = 4,
    };

    if (is_jmp == DISAS_JUMP && ctx->base.is_jmp == DISAS_UPDATE) {
        ctx->base.is_jmp = DISAS_EXIT;
    }

    if (is_jmp == DISAS_UPDATE && ctx->base.is_jmp == DISAS_JUMP) {
        ctx->base.is_jmp = DISAS_EXIT;
    }

    if (IS_JMP_ORDER[is_jmp] > IS_JMP_ORDER[ctx->base.is_jmp]) {
        ctx->base.is_jmp = is_jmp;
    }
}

static inline void gen_raise_exception(DisasContext *ctx, uint32_t excp)
{
    TCGv_i32 exception = tcg_const_i32(excp);

    gen_helper_raise_exception(cpu_env, exception);
    tcg_temp_free_i32(exception);

    end_tb(ctx, DISAS_NORETURN);
}


static inline void gen_save_pc(DisasContext *ctx, target_ulong dest)
{
    TCGv_i64 pc = tcg_const_i64(dest);
    gen_store_register_no_delay(ctx, pc, REG_kv3_PC);
    tcg_temp_free_i64(pc);
}

static inline uint64_t get_syndrome(DisasContext *ctx)
{
    return ctx->bundle.syndrome;
}

static inline void gen_update_excp_syndrome(DisasContext *ctx, uint64_t mask)
{
    TCGv_i64 tcg_value = tcg_const_i64(ctx->bundle.syndrome);
    TCGv_i64 tcg_mask = tcg_const_i64(mask);
    gen_helper_update_excp_syndrome(cpu_env, tcg_value, tcg_mask);
    tcg_temp_free_i64(tcg_mask);
    tcg_temp_free_i64(tcg_value);
}

static inline void gen_set_excp_address(DisasContext *ctx, TCGv_i64 address)
{
    if (unlikely(ctx->step_mode_enabled)) {
        gen_helper_set_excp_address(cpu_env, address);
    }
}

static inline void gen_init_syndrome(DisasContext *ctx)
{
    ctx->bundle.syndrome = KVX_FIELD_DP64(0, kv3_ES, BS, ctx->bundle.len);
    if (unlikely(ctx->step_mode_enabled)) {
        TCGv_i64 addr = tcg_const_i64(0);
        gen_update_excp_syndrome(ctx, -1ull);
        gen_set_excp_address(ctx, addr);
        tcg_temp_free_i64(addr);
    }
}

#define FILL_SYNDROME(ctx_, synmask_, field_, val_)                        \
    do {                                                                   \
        (ctx_)->bundle.syndrome =                                          \
            KVX_FIELD_DP64((ctx_)->bundle.syndrome, kv3_ES, field_, val_); \
        synmask_ |= KVX_FIELD_MASK(kv3_ES, field_);                        \
    } while (0)

static inline void gen_fill_syndrome_sfr(DisasContext *ctx, uint32_t sfri,
                                         uint32_t sfrp, uint32_t gprp)
{
    uint64_t mask = 0;
    FILL_SYNDROME(ctx, mask, SFRT, 1);
    FILL_SYNDROME(ctx, mask, SFRI, sfri);
    FILL_SYNDROME(ctx, mask, SFRP, sfrp);
    FILL_SYNDROME(ctx, mask, GPRP, gprp);
    if (unlikely(ctx->step_mode_enabled)) {
        gen_update_excp_syndrome(ctx, mask);
    }
}

static inline void gen_fill_syndrome_pic(DisasContext *ctx, uint32_t pic)
{
    uint64_t mask = 0;
    FILL_SYNDROME(ctx, mask, PIC, pic);
    if (unlikely(ctx->step_mode_enabled)) {
        gen_update_excp_syndrome(ctx, mask);
    }
}

static inline uint64_t fix_syndrome_dri_field(Opcode *opcode, int64_t size,
                                              uint64_t op)
{
    /*
     * The op value given by the MDS behaviour needs to be fixed in the case of
     * memory accesses that read or write a general purpose register tuple
     * greater than 8 bytes. In this case we have the index of the _tuple_, but
     * we need the index of the _first register in the tuple_.
     */

    if (opcode->insn == kv3_LQ || opcode->insn == kv3_SQ
        || opcode->insn == kv3_LO || opcode->insn == kv3_SO
        || opcode->insn == kv3_SV) {
        return op * size / 8;
    }

    /* The same applies for the LV but not the column (lv.cx) version */
    if ((opcode->insn == kv3_LV)
        && ((opcode->format == kv3_LSU_LVBO)
         || (opcode->format == kv3_LSU_LVBO_X)
         || (opcode->format == kv3_LSU_LVBO_Y)
         || (opcode->format == kv3_LSU_LVPB)
         || (opcode->format == kv3_LSU_LVPB_O)
         || (opcode->format == kv3_LSU_LVPB_Y)
         || (opcode->format == kv3_LSU_LVBI))) {
        return op * size / 8;
    }

    /*
     * For ACSWAP[WD], the access size is 4 or 8 bytes but we are indeed writing
     * a 128 bits register pair.
     */
    if (opcode->insn == kv3_ACSWAPW || opcode->insn == kv3_ACSWAPD) {
        return op * 2;
    }

    return op;
}

static inline void gen_fill_syndrome_mem_access(DisasContext *ctx,
                                                TCGv_i64 address,
                                                int64_t size,
                                                Modifier_kv3_variant variant,
                                                uint64_t dri,
                                                ExceptionSyndromeRWXValue rwx)
{
    uint64_t mask = 0;
    dri = fix_syndrome_dri_field(ctx->cur_opcode, size, dri);
    FILL_SYNDROME(ctx, mask, AS, (size == 64) ? 0x3f : size);
    FILL_SYNDROME(ctx, mask, UCA, (!ctx->data_cache_enabled || variant == VARIANT_U || variant == VARIANT_US));
    FILL_SYNDROME(ctx, mask, NTA, (variant == VARIANT_S || variant == VARIANT_US));
    FILL_SYNDROME(ctx, mask, RWX, rwx);
    FILL_SYNDROME(ctx, mask, DRI, dri);
    if (unlikely(ctx->step_mode_enabled)) {
        gen_update_excp_syndrome(ctx, mask);
        gen_helper_set_excp_address(cpu_env, address);
    }
}

#undef FILL_SYNDROME

static inline void gen_raise_trap_with_syndrome(DisasContext *ctx,
                                                unsigned int trap_idx,
                                                uint64_t syndrome)
{
    TCGv_i32 trap;
    TCGv_i64 syndrome_tcg;

    gen_save_pc(ctx, ctx->bundle.pc);

    trap = tcg_const_i32(trap_idx);
    syndrome_tcg = tcg_const_i64(syndrome);
    gen_helper_raise_trap(cpu_env, trap, syndrome_tcg);

    tcg_temp_free_i64(syndrome_tcg);
    tcg_temp_free_i32(trap);

    end_tb(ctx, DISAS_NORETURN);
}

static inline void gen_raise_trap(DisasContext *ctx, unsigned int trap_idx)
{
    gen_raise_trap_with_syndrome(ctx, trap_idx, get_syndrome(ctx));
}

static inline void gen_raise_trap_opcode(DisasContext *ctx)
{
    uint64_t syndrome = KVX_FIELD_DP64(0, kv3_ES, BS, ctx->bundle.len);
    gen_raise_trap_with_syndrome(ctx, TRAP_OPCODE, syndrome);
}

static inline void gen_raise_syscall(DisasContext *ctx, TCGv_i64 scall_num)
{
    gen_save_pc(ctx, ctx->bundle.pc);
    gen_helper_raise_syscall(cpu_env, scall_num);
}

static inline void gen_raise_syscall_semi(DisasContext *ctx, TCGv_i64 scall_num)
{
    TCGv_i64 pc = tcg_const_i64(ctx->bundle.pc);
    gen_helper_raise_syscall_semi(cpu_env, scall_num, pc);
    tcg_temp_free_i64(pc);
}

static inline bool is_singlestepping(DisasContext *ctx)
{
    return ctx->base.singlestep_enabled || ctx->step_mode_enabled;
}

static inline void gen_raise_step_exception(DisasContext *ctx)
{
    if (ctx->step_mode_enabled) {
        gen_helper_raise_debug_step(cpu_env);
    } else {
        gen_raise_exception(ctx, EXCP_DEBUG);
    }
}

static inline void gen_goto_tb(DisasContext *ctx, int n, target_ulong dest)
{
    if (unlikely(is_singlestepping(ctx))) {
        gen_save_pc(ctx, dest);
        gen_raise_step_exception(ctx);
    } else if (translator_use_goto_tb(&ctx->base, dest)) {
        tcg_gen_goto_tb(n);
        gen_save_pc(ctx, dest);
        tcg_gen_exit_tb(ctx->base.tb, n);
    } else {
        gen_save_pc(ctx, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
}

static inline void store_buffer_reset(DisasContext *ctx)
{
    ctx->store_buffer.num_entry = 0;
    ctx->store_buffer.num_store = 0;
}

static inline StoreBufferEntry* store_buffer_new_entry(DisasContext *ctx)
{
    StoreBufferEntry *entry;

    g_assert(ctx->store_buffer.num_entry < ST_BUF_MAX_SIZE);

    entry = &ctx->store_buffer.entries[ctx->store_buffer.num_entry];
    entry->count = 0;
    entry->cond = false;

    ctx->store_buffer.num_entry++;

    return entry;
}

static inline StoreBufferEntry* store_buffer_new_cond_entry(DisasContext *ctx)
{
    StoreBufferEntry *entry;

    entry = store_buffer_new_entry(ctx);
    entry->cond = true;
    entry->valid = tcg_const_local_i64(0);

    return entry;
}

static inline bool access_info_eq(const StorageAccessInfo *i0,
                                  const StorageAccessInfo *i1)
{
    return i0->data_size == i1->data_size
        && i0->offset == i1->offset
        && i0->sub_offset == i1->sub_offset;
}

static inline bool access_maps_to_register(const StorageAccessInfo *info, Register reg)
{
    return (info->offset == REGISTERS[reg].offset)
        && (info->data_size == (REGISTERS[reg].reg_width));
}

static inline bool access_maps_to_storage(const StorageAccessInfo *info, Storage sto_id)
{
    const StorageDescr *sto = &STORAGES[sto_id];
    size_t size = sto->width * sto->count / 8;

    return (info->offset >= sto->offset)
        && ((info->offset + info->data_size / 8) <= (sto->offset + size));
}

/*
 * Raw load/store functions
 * ========================
 */

/* Apply load value fixup operations contained in info */
static inline void gen_load_fixup_value(TCGv_i64 val,
                                        const StorageAccessInfo *info)
{
    if (info->sub_offset) {
        tcg_gen_shri_i64(val, val, info->sub_offset);
    }

    if (info->data_size < 64) {
        tcg_gen_andi_i64(val, val, (1ull << info->data_size) - 1);
    }
}

static bool gen_load_helper(DisasContext *ctx, TCGv_i64 dest,
                            const StorageAccessInfo *info)
{
    const RegisterInfo *regi = kvx_register_info(info->reg);

    if (regi->io || regi->alias_pl || regi->read) {
        TCGv_i64 index = tcg_const_i64(info->reg);
        if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
            gen_io_start();
        }

        end_tb(ctx, DISAS_UPDATE);

        gen_helper_read_register(dest, cpu_env, index);
        tcg_temp_free_i64(index);
        return true;
    }

    return false;
}

static inline void gen_load(DisasContext *ctx, TCGv_i64 dest,
                            const StorageAccessInfo *info)
{
    if (info->is_sysreg && gen_load_helper(ctx, dest, info)) {
        g_assert(info->reg != REG_kv3_PC);
        return;
    }

    if (access_maps_to_register(info, REG_kv3_PC)) {
        /* XXX This case can be optimised to compute jumps at translate time */
        tcg_gen_movi_i64(dest, ctx->bundle.pc);
        return;
    }

    if (access_maps_to_storage(info, STORAGE_kv3_NPC)) {
        /* XXX This case can be optimised to compute jumps at translate time */
        tcg_gen_movi_i64(dest, ctx->next_bundle_pc);
        return;
    }

    switch (info->access_size) {
    case 8:
        tcg_gen_ld8u_i64(dest, cpu_env, info->offset);
        break;

    case 16:
        tcg_gen_ld16u_i64(dest, cpu_env, info->offset);
        break;

    case 32:
        tcg_gen_ld32u_i64(dest, cpu_env, info->offset);
        break;

    case 64:
        /* TODO: add support for non-aligned 64 bits accesses */
        g_assert(info->access_count == 1);

        tcg_gen_ld_i64(dest, cpu_env, info->offset);
        break;

    default:
        g_assert_not_reached();
    }

    gen_load_fixup_value(dest, info);
}

static inline void gen_store_with_size(TCGv_i64 val, const StorageAccessInfo *info,
                                       void (*load_fct)(TCGv_i64, TCGv_ptr, tcg_target_long),
                                       void (*store_fct)(TCGv_i64, TCGv_ptr, tcg_target_long))
{
    if (info->data_size < info->access_size) {
        TCGv_i64 prev = tcg_temp_new_i64();

        load_fct(prev, cpu_env, info->offset);
        tcg_gen_andi_i64(prev, prev, ~(((1 << info->data_size) - 1) << info->sub_offset));

        tcg_gen_or_i64(val, prev, val);
        tcg_temp_free_i64(prev);
    }

    store_fct(val, cpu_env, info->offset);
}

/* Apply store value fixup operations contained in info */
static inline void gen_store_fixup_value(TCGv_i64 val,
                                         const StorageAccessInfo *info)
{
    if (info->data_size < 64) {
        tcg_gen_andi_i64(val, val, (1ull << info->data_size) - 1);
    }

    if (info->sub_offset) {
        tcg_gen_shli_i64(val, val, info->sub_offset);
    }

    if (info->write_mask != UINT64_MAX) {
        tcg_gen_andi_i64(val, val, info->write_mask);
    }
}

static inline bool gen_store_helper(DisasContext *ctx, TCGv_i64 val,
                                    const StorageAccessInfo *info)
{
    const RegisterInfo *regi = kvx_register_info(info->reg);

    if (regi->io || regi->alias_pl || regi->write || regi->it) {
        TCGv_i64 index = tcg_const_i64(info->reg);
        if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
            gen_io_start();
        }

        end_tb(ctx, DISAS_UPDATE);

        gen_helper_write_register(cpu_env, index, val);
        tcg_temp_free_i64(index);
        return true;
    }

    if (regi->addr_store_mask) {
        /* addresses are sign-extended and aligned */
        uint64_t mask = regi->addr_store_mask;

        tcg_gen_shli_i64(val, val, 64 - KVX_ADDRESS_LENGTH);
        tcg_gen_sari_i64(val, val, 64 - KVX_ADDRESS_LENGTH);

        if (regi->v64_aware && !ctx->v64) {
            mask &= 0xffffffffull;
        }

        tcg_gen_andi_i64(val, val, mask);
    }

    return false;
}

static inline void gen_store(DisasContext *ctx, TCGv_i64 val,
                             const StorageAccessInfo *info)
{
    if (info->is_sysreg && gen_store_helper(ctx, val, info)) {
        g_assert(info->reg != REG_kv3_PC);
        return;
    }

    gen_store_fixup_value(val, info);

    switch (info->access_size) {
    case 8:
        gen_store_with_size(val, info, tcg_gen_ld8u_i64, tcg_gen_st8_i64);
        break;

    case 16:
        gen_store_with_size(val, info, tcg_gen_ld16u_i64, tcg_gen_st16_i64);
        break;

    case 32:
        gen_store_with_size(val, info, tcg_gen_ld32u_i64, tcg_gen_st32_i64);
        break;

    case 64:
        gen_store_with_size(val, info, tcg_gen_ld_i64, tcg_gen_st_i64);
        break;

    default:
        g_assert_not_reached();
    }
}

/*
 * Store buffer functions
 * ======================
 */

/*
 * Return the store matching the info given as parameter or NULL if no store
 * matches the info.
 */
static inline StoreBufferStore *store_buffer_find_store(StoreBuffer *buffer,
                                                        StorageAccessInfo *info)
{
    size_t i;

    for (i = 0; i < buffer->num_store; i++) {
        StoreBufferStore *store = &buffer->stores[i];
        if (access_info_eq(info, &store->info)) {
            return store;
        }
    }

    return NULL;
}

static inline void store_buffer_entry_set_info(DisasContext *ctx,
                                               StoreBufferEntry *entry,
                                               StorageAccessInfo *info)
{
    StoreBufferStore *store;
    StoreBuffer *buf = &ctx->store_buffer;

    store = store_buffer_find_store(buf, info);

    if (store == NULL) {
        /* Allocate a new store */
        store = &buf->stores[buf->num_store++];
        store->info = *info;
        store->count = 0;
    }

    store->entries[store->count++] = entry;
    entry->info = &store->info;
}

static inline void store_buffer_entry_add_val_common(StoreBufferEntry *entry,
                                                     StoreBufferEntryValType type)
{
    size_t i = entry->count;

    g_assert(i * 8 < entry->info->data_size);

    if (i == 0) {
        entry->type = type;
    }

    g_assert(entry->type == type);

    if (entry->cond && i == 0) {
        tcg_gen_movi_i64(entry->valid, 1);
    }
}

static inline void store_buffer_entry_add_val_tcg(StoreBufferEntry *entry,
                                                  TCGv_i64 val)
{
    size_t i = entry->count;

    store_buffer_entry_add_val_common(entry, VAL_TYPE_TCG);
    entry->val.tcg[i] = tcg_temp_local_new();
    tcg_gen_mov_i64(entry->val.tcg[i], val);

    entry->count++;
}

static inline void store_buffer_entry_add_val_imm(StoreBufferEntry *entry,
                                                  uint64_t val)
{
    size_t i = entry->count;

    store_buffer_entry_add_val_common(entry, VAL_TYPE_IMM);
    entry->val.imm[i] = val;
    entry->count++;
}

static inline void store_buffer_push_tcg(DisasContext *ctx, TCGv_i64 val,
                                         StorageAccessInfo *info)
{
    StoreBufferEntry *entry;

    entry = store_buffer_new_entry(ctx);
    store_buffer_entry_set_info(ctx, entry, info);
    store_buffer_entry_add_val_tcg(entry, val);
}

static inline void store_buffer_push_imm(DisasContext *ctx, uint64_t val,
                                         StorageAccessInfo *info)
{
    StoreBufferEntry *entry;

    entry = store_buffer_new_entry(ctx);
    store_buffer_entry_set_info(ctx, entry, info);
    store_buffer_entry_add_val_imm(entry, val);
}



/*
 * Store to PC its next value if a conditional jump is not taken. Usually,
 * it is the next bundle PC, but can be the hardware loop start PC value if we
 * are in an hardware loop context and lc matches the next bundle address.
 */
static inline void gen_store_next_default_pc(DisasContext *ctx)
{
    if (ctx->gen_hardware_loop &&
        ctx->next_bundle_pc == ctx->hardware_loop_pc) {
        TCGv_i64 ls;

        ls = tcg_temp_new_i64();
        gen_load_register(ctx, ls, REG_kv3_LS);
        gen_store_register_no_delay(ctx, ls, REG_kv3_PC);
        tcg_temp_free_i64(ls);
    } else {
        gen_save_pc(ctx, ctx->next_bundle_pc);
    }
}

static inline bool store_buffer_do_pc_imm_entry(DisasContext *ctx,
                                                StoreBufferEntry *entry)
{
    TCGLabel *false_cond;
    const bool is_cond = entry->cond;

    /* PC is 64 bits */
    g_assert(entry->count == 1);

    if (ctx->base.is_jmp != DISAS_NEXT) {
        /*
         * Another instruction requested that we exit the TB. Don't use
         * gen_goto_tb then.
         */
        return false;
    }

    if (ctx->gen_hardware_loop) {
        /*
         * For sake of simplicity, don't use gen_goto_tb in this case. This
         * would theoretically work if the store is not conditional but
         * requires refactoring of the DISAS_NORETURN and gen_goto_tb
         * generation logic.
         */
        return false;
    }

    if (is_cond) {
        false_cond = gen_new_label();
        tcg_gen_brcondi_i64(TCG_COND_EQ, entry->valid, 0, false_cond);
        tcg_temp_free_i64(entry->valid);
    }

    gen_goto_tb(ctx, 0, entry->val.imm[0]);

    if (is_cond) {
        gen_set_label(false_cond);
        gen_goto_tb(ctx, 1, ctx->next_bundle_pc);
    }

    end_tb(ctx, DISAS_NORETURN);
    return true;
}

static inline void store_buffer_do_entry(DisasContext *ctx, StoreBufferEntry *entry)
{
    TCGLabel *skip = NULL, *end;
    StorageAccessInfo info = *entry->info;
    size_t i;
    bool is_pc = access_maps_to_register(entry->info, REG_kv3_PC);
    bool is_imm = entry->type == VAL_TYPE_IMM;

    if (is_imm) {
        /* Only PC can be stored as an immediate value (for TB chaining) */
        g_assert(is_pc);

        if (store_buffer_do_pc_imm_entry(ctx, entry)) {
            return;
        }

        /* Chaining is not possible. Fallback to classic store */
    }

    if (entry->cond) {
        skip = gen_new_label();
        tcg_gen_brcondi_i64(TCG_COND_EQ, entry->valid, 0, skip);
        tcg_temp_free_i64(entry->valid);
    }

    info.access_size = MIN(info.access_size, 64);

    for (i = 0; i < entry->count; i++) {
        TCGv_i64 val;

        val = entry->type == VAL_TYPE_TCG
            ? entry->val.tcg[i]
            : tcg_const_i64(entry->val.imm[i]);

        gen_store(ctx, val, &info);

        info.offset += 8;
        tcg_temp_free_i64(val);
    }

    if (entry->cond) {
        if (is_pc) {
            end = gen_new_label();
            tcg_gen_br(end);
            gen_set_label(skip);
            gen_store_next_default_pc(ctx);
            gen_set_label(end);
        } else {
            gen_set_label(skip);
        }
    }

    if (is_pc) {
        end_tb(ctx, DISAS_JUMP);
    }
}

/*
 * We can merge entries in two different ways:
 *   - MERGE_LAST: keep the most recent one
 *   - MERGE_OR: OR the values together
 * MERGE_OR is used for the CS storage, for which flags can be written by
 * multiple instructions in the same bundle.
 */
typedef enum MergeMode {
    MERGE_LAST,
    MERGE_OR,
} MergeMode;

static inline MergeMode store_buffer_get_merge_mode(const StorageAccessInfo *info)
{
    const StorageDescr *cs = &STORAGES[STORAGE_kv3_CS];

    if (info->offset >= cs->offset
        && info->offset < cs->offset + (cs->width * cs->count / 8)) {
        return MERGE_OR;
    }

    return MERGE_LAST;
}

/*
 * Merge two store buffer entries. Only dst is valid after the merge and
 * represents the merge result of src and dst.
 */
static inline void store_buffer_merge_entries(StoreBufferEntry *dst,
                                              StoreBufferEntry *src)
{
    size_t i;
    MergeMode mode = store_buffer_get_merge_mode(dst->info);
    TCGv_i64 zero = tcg_const_i64(0);
    bool merge_cond;

    g_assert(src->count == dst->count);
    g_assert(src->type == VAL_TYPE_TCG);

    if (src->cond) {
        for (i = 0; i < src->count; i++) {
            tcg_gen_movcond_i64(TCG_COND_NE, src->val.tcg[i],
                                src->valid, zero, src->val.tcg[i], zero);
        }
    }

    if (dst->cond) {
        for (i = 0; i < src->count; i++) {
            tcg_gen_movcond_i64(TCG_COND_NE, dst->val.tcg[i],
                                dst->valid, zero, dst->val.tcg[i], src->val.tcg[i]);
        }
    }

    for (i = 0; i < src->count; i++) {
        if (mode == MERGE_OR) {
            tcg_gen_or_i64(dst->val.tcg[i], dst->val.tcg[i], src->val.tcg[i]);
        }
        tcg_temp_free_i64(src->val.tcg[i]);
    }

    merge_cond = src->cond && dst->cond;

    if (merge_cond) {
        tcg_gen_or_i64(dst->valid, dst->valid, src->valid);
        tcg_temp_free_i64(src->valid);
    } else if (src->cond) {
        tcg_temp_free_i64(src->valid);
    } else if (dst->cond) {
        tcg_temp_free_i64(dst->valid);
    }

    dst->cond = merge_cond;

    tcg_temp_free_i64(zero);
}

static inline void store_buffer_do_store(DisasContext *ctx, StoreBufferStore *store)
{
    size_t i;

    for (i = 0; i < store->count - 1; i++) {
        /* Merge this entry with the next one */
        store_buffer_merge_entries(store->entries[i + 1],
                                   store->entries[i]);
    }

    /* Last entry, do the actual store */
    store_buffer_do_entry(ctx, store->entries[store->count - 1]);
}

static inline void store_buffer_flush(DisasContext *ctx)
{
    size_t i;
    ssize_t imm_pc_idx = -1;

    for (i = 0; i < ctx->store_buffer.num_store; i++) {
        StoreBufferStore *store = &ctx->store_buffer.stores[i];

        if (store->entries[0]->type == VAL_TYPE_IMM) {
            /* Do not proceed immediate PC store now. Defer it to the end */
            imm_pc_idx = i;
            continue;
        }

        store_buffer_do_store(ctx, store);
    }

    if (imm_pc_idx != -1) {
        /*
         * Proceed the immediate PC store if it exists. Doing it as the last
         * entry ensures we flushed all the store buffer before calling
         * gen_goto_tb.
         */
        store_buffer_do_store(ctx, &ctx->store_buffer.stores[imm_pc_idx]);
    }

    store_buffer_reset(ctx);
}


/*
 * Access info filling functions
 * =============================
 */
static inline void compute_access_size(StorageAccessInfo *info)
{
    info->access_size = MAX(pow2ceil(info->data_size), 8);

    if (info->sub_offset + info->data_size > info->access_size) {
        info->access_size *= 2;
    }

    if (info->access_size > 64) {
        info->access_count = info->access_size / 64;
        info->access_size = 64;
    } else {
        info->access_count = 1;
    }
}

static inline void get_register_access_info_common(Register reg_id,
                                                   size_t len,
                                                   StorageAccessInfo *ret)
{
    const RegisterDescr *reg = &REGISTERS[reg_id];

    ret->data_size = reg->reg_width * len;
    ret->offset = reg->offset;
    ret->sub_offset = 0;
    ret->write_mask = reg->mask;
    ret->is_sysreg = (reg->regfile == REGFILE_kv3_SFR);
    ret->reg = reg_id;

    /* Access to SFR regs must be on a single register */
    g_assert(!ret->is_sysreg || len == 1);

    compute_access_size(ret);
}

static inline void get_register_access_info(Register reg_id,
                                            StorageAccessInfo *ret)
{
    get_register_access_info_common(reg_id, 1, ret);
}

static inline void get_storage_access_info(Storage sto, size_t addr, size_t len,
                                           StorageAccessInfo *ret)
{
    const StorageDescr *sdescr = &STORAGES[sto];

    switch (sto) {
    case STORAGE_kv3_PC:
        get_register_access_info(REG_kv3_PC, ret);
        return;
    case STORAGE_kv3_SRS:
        /*
         * Access to SRS should go through regfile/regclass in order
         * to decide if we need to use an helper.
         */
        g_assert_not_reached();
        break;
    default:
        break;
    }

    /*
     * We assume access is on a pure memory register
     * on existing fields only. Any other access
     * must be handled above or detected in load/store
     * code after (NPC is).
     */
    ret->write_mask = UINT64_MAX;
    ret->is_sysreg = false;
    ret->reg = -1;

    ret->data_size = sdescr->width * len;
    ret->offset = sdescr->offset + ((addr * sdescr->width) / 8);
    ret->sub_offset = (addr * sdescr->width) % 8;

    /* We do not support >64 bits unaligned accesses */
    g_assert(ret->data_size <= 64 || ret->sub_offset == 0);

    compute_access_size(ret);
}

static inline void get_regfile_access_info(RegFile regfile,
                                           uint64_t offset, size_t len,
                                           StorageAccessInfo *ret)
{
    Register reg_id = REGFILE_MAPPING[regfile].registers[offset];
    get_register_access_info_common(reg_id, len, ret);
}

static inline void get_regclass_access_info(RegClass reg_class, uint64_t offset,
                                            StorageAccessInfo *ret)
{
    RegFile regfile = REGCLASS_MAPPING[reg_class];
    get_regfile_access_info(regfile, offset, 1, ret);
}



/*
 * Storage access
 * ==============
 */
static inline void gen_load_storage(DisasContext *ctx, TCGv_i64 dest,
                                    Storage storage, size_t addr, size_t len)
{
    StorageAccessInfo info;

    get_storage_access_info(storage, addr, len, &info);
    gen_load(ctx, dest, &info);
}

static inline uint64_t gen_load_storage_imm(DisasContext *ctx, Storage storage,
                                            size_t addr, size_t len)
{
    /* This function should be used for PC and NPC only */
    switch (storage) {
    case STORAGE_kv3_PC:
        g_assert(addr == 0);
        g_assert(len == 1);
        return ctx->bundle.pc;

    case STORAGE_kv3_NPC:
        g_assert(addr == 0);
        g_assert(len == 1);
        return ctx->next_bundle_pc;

    default:
        g_assert_not_reached();
    }
}

static inline StorageAccessInfo gen_store_storage_common(DisasContext *ctx,
                                                         Storage *storage,
                                                         size_t addr, size_t len)
{
    StorageAccessInfo info;

    if (*storage == STORAGE_kv3_NPC) {
        /*
         * Store directly in PC, so we don't need to move NPC to PC at the end
         * of the TB. Store to storage NPC will never occur.
         */
        *storage = STORAGE_kv3_PC;
    }

    if (*storage == STORAGE_kv3_CS) {
        /*
         * Track CS stores to detect possible arithmetic interrupt conditions.
         */
        ctx->cs_mask |= MAKE_64BIT_MASK(addr, len);
    }

    get_storage_access_info(*storage, addr, len, &info);

    return info;
}

static inline void gen_store_storage(DisasContext *ctx, TCGv_i64 src,
                                     Storage storage, size_t addr, size_t len)
{
    StorageAccessInfo info = gen_store_storage_common(ctx, &storage, addr, len);
    store_buffer_push_tcg(ctx, src, &info);
}

static inline void gen_store_storage_cond(DisasContext *ctx,
                                          StoreBufferEntry *entry,
                                          TCGv_i64 src, Storage storage,
                                          size_t addr, size_t len)
{
    StorageAccessInfo info = gen_store_storage_common(ctx, &storage, addr, len);

    store_buffer_entry_set_info(ctx, entry, &info);
    store_buffer_entry_add_val_tcg(entry, src);
}

static inline void gen_store_storage_imm(DisasContext *ctx,
                                         uint64_t val, Storage storage,
                                         size_t addr, size_t len)
{
    StorageAccessInfo info = gen_store_storage_common(ctx, &storage, addr, len);
    store_buffer_push_imm(ctx, val, &info);
}

static inline void gen_store_storage_cond_imm(DisasContext *ctx,
                                              StoreBufferEntry *entry,
                                              uint64_t val, Storage storage,
                                              size_t addr, size_t len)
{
    StorageAccessInfo info = gen_store_storage_common(ctx, &storage, addr, len);

    store_buffer_entry_set_info(ctx, entry, &info);
    store_buffer_entry_add_val_imm(entry, val);
}

/*
 * Register access
 * ===============
 */
static inline void gen_load_register(DisasContext *ctx, TCGv_i64 dest, Register reg)
{
    StorageAccessInfo info;

    get_register_access_info(reg, &info);
    gen_load(ctx, dest, &info);
}

static inline void gen_store_register_no_delay(DisasContext *ctx, TCGv_i64 src, Register reg)
{
    StorageAccessInfo info;

    get_register_access_info(reg, &info);
    gen_store(ctx, src, &info);
}


/*
 * RegFile access
 * ==============
 */
static inline void gen_load_regfile(DisasContext *ctx, TCGv_i64 dest,
                                    RegFile regfile, size_t addr, size_t len)
{
    StorageAccessInfo info;

    get_regfile_access_info(regfile, addr, len, &info);
    gen_load(ctx, dest, &info);
}

static inline void gen_load_regfile_indirect(DisasContext *ctx, TCGv_i64 dest,
                                             RegFile regfile, TCGv_i64 addr, size_t len)
{
    TCGv_i64 regfile_id;

    g_assert(len == 1);

    /*
     * Not all SFR registers require the following. But it does not harm
     * and the helper knows how to access simple registers.
     */
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }

    regfile_id = tcg_const_i64(regfile);
    gen_helper_read_regfile_indirect(dest, cpu_env, regfile_id, addr);
    tcg_temp_free_i64(regfile_id);

    end_tb(ctx, DISAS_UPDATE);
}

static inline void gen_store_regfile(DisasContext *ctx, TCGv_i64 src,
                                     RegFile regfile, size_t addr, size_t len)
{
    StorageAccessInfo info;

    get_regfile_access_info(regfile, addr, len, &info);

    store_buffer_push_tcg(ctx, src, &info);
}

static inline void gen_store_regfile_cond(DisasContext *ctx,
                                          StoreBufferEntry *entry,
                                          TCGv_i64 src, RegFile regfile,
                                          size_t addr, size_t len)
{
    StorageAccessInfo info;

    get_regfile_access_info(regfile, addr, len, &info);

    store_buffer_entry_set_info(ctx, entry, &info);
    store_buffer_entry_add_val_tcg(entry, src);
}


/*
 * RegClass (operands) access
 * ==========================
 */
static inline void gen_load_operand(DisasContext *ctx, TCGv_i64 dest,
                                    RegClass regclass, uint64_t offset)
{
    RegFile regfile = REGCLASS_MAPPING[regclass];

    gen_load_regfile(ctx, dest, regfile, offset, 1);
}

static inline void gen_load_operand_bigint(DisasContext *ctx, MDSTCGBigInt *dest,
                                           RegClass regclass, uint64_t offset,
                                           size_t sub_offset)
{
    StorageAccessInfo info;
    TCGv_i64 val;

    get_regclass_access_info(regclass, offset, &info);

    g_assert(info.data_size >= mds_tcg_bigint_get_size(dest));

    /* Clamp the access size to the big int size since accesses can be sliced */
    info.data_size = mds_tcg_bigint_get_size(dest);
    info.offset += sub_offset / 8;

    /* Split the access into 64 bits pieces */
    MDS_BIGINT_FOREACH(dest, val) {
        tcg_gen_ld_i64(val, cpu_env, info.offset);
        info.offset += 8;
    }
}

static inline void gen_apply_set_relative_pl(DisasContext *ctx,
                                             uint64_t sfr_idx, TCGv_i64 value)
{
    Register reg = REGFILE_MAPPING[REGFILE_kv3_SFR].registers[sfr_idx];
    const RegisterInfo *info = kvx_register_info(reg);
    unsigned sfri;

    /*
     * we only do that for SET and RSWAP instruction, WFXL and WFXM
     * writes are caught in the 'apply' mds-handler.
     */
    switch (ctx->cur_opcode->insn) {
    case kv3_SET:
        sfri = ES_SFRI_SET;
        break;
    case kv3_RSWAP:
        sfri = ES_SFRI_RSWAP;
        break;
    default:
        return;
    }

    if (info->apply_rpl) {
        TCGv_i64 mask = tcg_const_i64(~0ull);
        TCGv_i32 sfr = tcg_const_i32(sfr_idx);
        TCGv_i64 syndrome;

        gen_fill_syndrome_sfr(ctx, sfri, sfr_idx, ctx->set_gpr_idx);
        syndrome = tcg_const_i64(get_syndrome(ctx));

        /* no need to save PC, it has been done in test_access previously */
        gen_helper_apply_relative_pl(value, cpu_env, sfr, value, mask,
                                     syndrome);
        tcg_temp_free_i32(sfr);
        tcg_temp_free_i64(mask);
        tcg_temp_free_i64(syndrome);
    }
}

static inline void gen_store_operand(DisasContext *ctx, TCGv_i64 src,
                                     RegClass regclass, uint64_t reg_idx)
{
    RegFile regfile = REGCLASS_MAPPING[regclass];
    StorageAccessInfo info;

    get_regfile_access_info(regfile, reg_idx, 1, &info);
    if (info.is_sysreg) {
        gen_apply_set_relative_pl(ctx, reg_idx, src);
    }

    store_buffer_push_tcg(ctx, src, &info);
}

static inline void gen_store_operand_cond(DisasContext *ctx,
                                          StoreBufferEntry *entry,
                                          TCGv_i64 src, RegClass regclass,
                                          uint64_t reg_idx)
{
    RegFile regfile = REGCLASS_MAPPING[regclass];
    StorageAccessInfo info;

    get_regfile_access_info(regfile, reg_idx, 1, &info);
    if (info.is_sysreg) {
        gen_apply_set_relative_pl(ctx, reg_idx, src);
    }

    store_buffer_entry_set_info(ctx, entry, &info);
    store_buffer_entry_add_val_tcg(entry, src);
}

static inline void gen_store_bigint_with_info(DisasContext *ctx,
                                              MDSTCGBigInt *src,
                                              StorageAccessInfo *info,
                                              StoreBufferEntry *entry)
{
    TCGv_i64 val;

    g_assert(info->data_size >= mds_tcg_bigint_get_size(src));

    /* Clamp the access size to the big int size since accesses can be sliced */
    info->data_size = mds_tcg_bigint_get_size(src);

    store_buffer_entry_set_info(ctx, entry, info);

    /* Split the access into 64 bits pieces */
    MDS_BIGINT_FOREACH(src, val) {
        store_buffer_entry_add_val_tcg(entry, val);
    }
}

static inline void gen_store_operand_bigint(DisasContext *ctx,
                                            MDSTCGBigInt *src,
                                            RegClass regclass, uint64_t offset,
                                            size_t sub_offset)
{
    StorageAccessInfo info;
    StoreBufferEntry *entry;

    get_regclass_access_info(regclass, offset, &info);

    info.offset += sub_offset / 8;

    entry = store_buffer_new_entry(ctx);

    gen_store_bigint_with_info(ctx, src, &info, entry);
}

static inline void gen_store_operand_bigint_cond(DisasContext *ctx,
                                                 StoreBufferEntry *entry,
                                                 MDSTCGBigInt *src,
                                                 RegClass regclass, uint64_t offset,
                                                 size_t sub_offset)
{
    StorageAccessInfo info;

    get_regclass_access_info(regclass, offset, &info);
    info.offset += sub_offset / 8;

    gen_store_bigint_with_info(ctx, src, &info, entry);
}


#endif
