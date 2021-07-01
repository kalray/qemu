#include "qemu/osdep.h"
#include "qemu/log.h"
#include "mds-helpers.h"

#include "../translate.h"

__attribute__((weak)) void gen_effect_idle(DisasContext *ctx, int64_t arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_stop_owner(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_barrier(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_mmi_owner(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_readtlb(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_probetlb(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_writetlb(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_invaldtlb(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_invalitlb(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_throw_OPCODE(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_rfe_owner(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_rfe(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_syscall(DisasContext *ctx, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_wfxl_check_access(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_wfxl(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_wfxm_check_access(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_wfxm(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_get_check_access(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, uint64_t arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_get(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_set_check_access(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_waitit(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_syncgroup_owner(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_syncgroup(DisasContext *ctx, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_scalarcond(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_MEM_load(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, int64_t arg1, uint64_t arg2, uint64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_MEM_load_bigint_tcg64_s64_u64_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, int64_t arg1, uint64_t arg2, uint64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_insert_64_bigint_bigint_tcg64_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_insert_64_bigint_bigint_bigint_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, MDSTCGBigInt * arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_store(DisasContext *ctx, TCGv_i64 arg0, int64_t arg1, TCGv_i64 arg2, uint64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_store_tcg64_s64_bigint_u64(DisasContext *ctx, TCGv_i64 arg0, int64_t arg1, MDSTCGBigInt * arg2, uint64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_MEM_cas(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, int64_t arg1, TCGv_i64 arg2, TCGv_i64 arg3, uint64_t arg4, uint64_t arg5)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_MEM_cas_tcg64_tcg64_s64_tcg64_bigint_s64_u64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, int64_t arg1, TCGv_i64 arg2, MDSTCGBigInt * arg3, int64_t arg4, uint64_t arg5)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_MEM_swap(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, int64_t arg1, int64_t arg2, uint64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_MEM_faa(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, int64_t arg1, TCGv_i64 arg2, uint64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_dtouchl(DisasContext *ctx, TCGv_i64 arg0, uint64_t arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_dinvall(DisasContext *ctx, TCGv_i64 arg0, uint64_t arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_dzerol(DisasContext *ctx, TCGv_i64 arg0, int64_t arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_iinvals(DisasContext *ctx, TCGv_i64 arg0, uint64_t arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_dinval(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_iinval(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_MEM_fence(DisasContext *ctx)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply__BMM_8(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply__BMT_8(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsrec_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fdivbyzero(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_finexact(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_finvalid(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_foverflow(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsrsr_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fwiden_32_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fnarrow_64_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_funderflow(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_frec_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_frsq_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsrec_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsrsr_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fwiden_16_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fnarrow_32_16(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsrec_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsrsr_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fwiden_16_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fnarrow_64_32_x2_tcg64_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fnarrow_32_16_x4_tcg64_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ROL(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, int64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ROR(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, int64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_comparison_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_comparison_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_comparison_16(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_simplecond_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_CLZ(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, int64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_CLS(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, int64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_CBS(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, int64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_CTZ(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, int64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_simplecond_16(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_floatcomp_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_floatcomp_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_test_floatcomp_16(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, uint64_t arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsdiv_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fcdiv_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsdiv_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fcdiv_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsdiv_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fcdiv_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmin_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmax_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmin_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmax_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmin_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmax_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmin_16_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmax_16_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_float_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_float_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_float_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_floatu_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_floatu_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_floatu_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fixed_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fixed_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fixed_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fixedu_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fixedu_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fixedu_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, uint64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_clm_64_128_bigint_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, int64_t arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_clm_64_128_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_gcm_bb_64_128_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_gcm_bt_64_128_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_gcm_tt_64_128_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_SWAP(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, int64_t arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_crc32_be_u32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_reflect_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_64_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_64_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_64_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_32_64_x2_bigint_tcg64_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, int64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_32_64_x2_bigint_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_16_32_x4_bigint_tcg64_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, int64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_16_32_x4_bigint_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmulc_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmulc_32_64_bigint_tcg64_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, int64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmulc_32_64_bigint_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmulcc_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmulcc_32_64_bigint_tcg64_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, int64_t arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmulcc_32_64_bigint_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fadd_64_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fadd_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fadd_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fadd_64_64_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fadd_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_faddcc_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_faddcc_64_64_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_faddcc_32_32_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbf_64_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbf_32_32_x2(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbf_16_16_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbf_64_64_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbf_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbfcc_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbfcc_64_64_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbfcc_32_32_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_16_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_32_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_32_64_x2_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, int64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_32_64_x2_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_16_32_x4_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, int64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_16_32_x4_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_16_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_32_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_32_64_x2_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, int64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_32_64_x2_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_16_32_x4_bigint_tcg64_bigint_tcg64_s64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, int64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_16_32_x4_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fadd_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbf_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_16_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_32_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fdot2_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fdot2_32_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fdot2_32_32_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fdot2_32_64_x2_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmm2awq_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmm2swq_bigint_tcg64_bigint_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmm2wq_bigint_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_transpose_64_4x4_0_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_transpose_64_4x4_1_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_transpose_64_4x4_2_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_transpose_64_4x4_3_bigint_bigint_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot8_8_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_join_32_x8_bigint_tcg64_tcg64_tcg64_tcg64_tcg64_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3, TCGv_i64 arg4, TCGv_i64 arg5, TCGv_i64 arg6, TCGv_i64 arg7)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_add_32_32_x8_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, MDSTCGBigInt * arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot8u_8_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot8su_8_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot8us_8_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_sxb_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot4_16_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_join_64_x4_bigint_tcg64_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_add_64_64_x4_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0, MDSTCGBigInt * arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_zxb_x4(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot4u_16_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot4su_16_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_dot4us_16_64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fdot4add_16_32_reset(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fdot4add_16_32_noreset(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_join_32_x4_bigint_tcg64_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_round_64_x4_bigint_u64_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, uint64_t arg0, TCGv_i64 arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_satu_64_16_x4_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, MDSTCGBigInt * arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_sat_64_16_x4_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, MDSTCGBigInt * arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_round_32_x8_bigint_u64_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, uint64_t arg0, TCGv_i64 arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_satu_32_8_x8_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, MDSTCGBigInt * arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_sat_32_8_x8_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, MDSTCGBigInt * arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fpow2scale_32_32_x8_bigint_tcg64_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_frelu_32_32_x8_noreset_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, MDSTCGBigInt * arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fnarrow_32_16_x8_reset_bigint_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fnarrow_32_16_x8_noreset_bigint_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_break(DisasContext *ctx, uint64_t arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_MEM_cas_tcg64_tcg64_s64_tcg64_bigint_u64_u64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, int64_t arg1, TCGv_i64 arg2, MDSTCGBigInt * arg3, uint64_t arg4, uint64_t arg5)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffma_16_16_x8_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffms_16_16_x8_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmul_16_16_x8_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fconj_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fconj_32_32_x2_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, MDSTCGBigInt * arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmulc_32_32_x2_tcg64_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdma_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdma_32_32_x2_tcg64_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdma_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdms_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdms_32_32_x2_tcg64_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdms_32_32_x4_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmda_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmda_32_32_x2_tcg64_tcg64_bigint_bigint_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmda_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmsa_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmsa_32_32_x2_tcg64_tcg64_bigint_bigint_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmsa_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmds_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmds_32_32_x2_tcg64_tcg64_bigint_bigint_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmds_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmas_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmas_32_32_x2_tcg64_tcg64_bigint_bigint_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffdmas_32_32_x4_bigint_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmm212_32_32_bigint_tcg64_tcg64_tcg64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmm222_32_32_bigint_tcg64_bigint_bigint_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, uint64_t arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmma212_32_32_bigint_tcg64_tcg64_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmma222_32_32_bigint_tcg64_bigint_bigint_bigint_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3, uint64_t arg4)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmms212_32_32_bigint_tcg64_tcg64_tcg64_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fmms222_32_32_bigint_tcg64_bigint_bigint_bigint_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3, uint64_t arg4)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffmac_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffmac_32_32_x2_tcg64_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffmsc_32_32(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, TCGv_i64 arg1, TCGv_i64 arg2, TCGv_i64 arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_ffmsc_32_32_x2_tcg64_tcg64_bigint_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, TCGv_i64 ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2, MDSTCGBigInt * arg3)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fadd_16_16_x8_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_fsbf_16_16_x8_bigint_tcg64_bigint_bigint(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, TCGv_i64 arg0, MDSTCGBigInt * arg1, MDSTCGBigInt * arg2)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_effect_sendv_bigint_u64(DisasContext *ctx, MDSTCGBigInt * arg0, uint64_t arg1)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
__attribute__((weak)) void gen_apply_recvv_bigint_u64(DisasContext *ctx, MDSTypeBinding ret_type, MDSTCGBigInt * ret, uint64_t arg0)
{
    qemu_log("Warning: %s: not implemented\n", __func__);
}
