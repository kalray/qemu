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
#include "exec/helper-proto.h"

#include "lao-wrapper.h"

uint64_t HELPER(bmm_8)(uint64_t arg0, uint64_t arg1)
{
    return lao_bmm_8(arg0, arg1);
}

uint64_t HELPER(bmt_8)(uint64_t arg0)
{
    return lao_bmt_8(arg0);
}

void HELPER(clm_64_128)(CPUKVXState *env, uint64_t arg0, uint64_t arg1)
{
    lao_clm_64_128(&env->scratch[0], &env->scratch[1], arg0, arg1);
}

void HELPER(gcm_bb_64_128)(CPUKVXState *env, uint64_t arg0, uint64_t arg1)
{
    lao_gcm_bb_64_128(&env->scratch[0], &env->scratch[1], arg0, arg1);
}

void HELPER(gcm_bt_64_128)(CPUKVXState *env, uint64_t arg0, uint64_t arg1)
{
    lao_gcm_bt_64_128(&env->scratch[0], &env->scratch[1], arg0, arg1);
}

void HELPER(gcm_tt_64_128)(CPUKVXState *env, uint64_t arg0, uint64_t arg1)
{
    lao_gcm_tt_64_128(&env->scratch[0], &env->scratch[1], arg0, arg1);
}


/*
 * FPU related helpers
 */

uint64_t HELPER(fnarrow_64_32_x2)(uint64_t rm, uint64_t arg0_l, uint64_t arg0_h)
{
    return lao_fnarrow_64_32_x2(rm, arg0_l, arg0_h);
}

uint64_t HELPER(fsrec_64)(uint64_t arg0)
{
    return lao_fsrec_64(arg0);
}

uint64_t HELPER(fdivbyzero)(void)
{
    return lao_fdivbyzero();
}

uint64_t HELPER(finexact)(void)
{
    return lao_finexact();
}

uint64_t HELPER(finvalid)(void)
{
    return lao_finvalid();
}

uint64_t HELPER(foverflow)(void)
{
    return lao_foverflow();
}

uint64_t HELPER(funderflow)(void)
{
    return lao_funderflow();
}

uint64_t HELPER(crc32_be_u32)(uint64_t arg0, uint64_t arg1)
{
    return lao_crc32_be_u32(arg0, arg1);
}

uint64_t HELPER(reflect_32)(uint64_t arg0)
{
    return lao_reflect_32(arg0);
}

uint64_t HELPER(dot8_8_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot8_8_32(arg0, arg1);
}

uint64_t HELPER(dot8u_8_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot8u_8_32(arg0, arg1);
}

uint64_t HELPER(dot8su_8_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot8su_8_32(arg0, arg1);
}

uint64_t HELPER(dot8us_8_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot8us_8_32(arg0, arg1);
}

uint64_t HELPER(sxb_x4)(uint64_t arg0)
{
    return lao_sxb_x4(arg0);
}

uint64_t HELPER(dot4_16_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot4_16_64(arg0, arg1);
}

uint64_t HELPER(zxb_x4)(uint64_t arg0)
{
    return lao_zxb_x4(arg0);
}

uint64_t HELPER(dot4u_16_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot4u_16_64(arg0, arg1);
}

uint64_t HELPER(dot4su_16_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot4su_16_64(arg0, arg1);
}

uint64_t HELPER(dot4us_16_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_dot4us_16_64(arg0, arg1);
}

void HELPER(round_64_x4)(CPUKVXState *env, uint64_t arg0, uint64_t arg1,
                         uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5)
{
    lao_round_64_x4(&env->scratch[0], &env->scratch[1],
                    &env->scratch[2], &env->scratch[3],
                    arg0, arg1, arg2, arg3, arg4, arg5);
}

void HELPER(round_32_x8)(CPUKVXState *env, uint64_t arg0, uint64_t arg1,
                         uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5)
{
    lao_round_32_x8(&env->scratch[0], &env->scratch[1],
                    &env->scratch[2], &env->scratch[3],
                    arg0, arg1, arg2, arg3, arg4, arg5);
}

uint64_t HELPER(satu_32_8_x8)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_satu_32_8_x8(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(sat_32_8_x8)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_sat_32_8_x8(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(fnarrow_32_16_x4)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fnarrow_32_16_x4(arg0, arg1, arg2);
}

uint64_t HELPER(fsrsr_64)(uint64_t arg0)
{
    return lao_fsrsr_64(arg0);
}

uint64_t HELPER(fwiden_32_64)(uint64_t arg0)
{
    return lao_fwiden_32_64(arg0);
}

uint64_t HELPER(fnarrow_64_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_fnarrow_64_32(arg0, arg1);
}

uint64_t HELPER(frec_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_frec_32(arg0, arg1);
}

uint64_t HELPER(frsq_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_frsq_32(arg0, arg1);
}

uint64_t HELPER(fsrec_32)(uint64_t arg0)
{
    return lao_fsrec_32(arg0);
}

uint64_t HELPER(fsrsr_32)(uint64_t arg0)
{
    return lao_fsrsr_32(arg0);
}

uint64_t HELPER(fwiden_16_32)(uint64_t arg0)
{
    return lao_fwiden_16_32(arg0);
}

uint64_t HELPER(fnarrow_32_16)(uint64_t arg0, uint64_t arg1)
{
    return lao_fnarrow_32_16(arg0, arg1);
}

uint64_t HELPER(fsrec_32_x2)(uint64_t arg0)
{
    return lao_fsrec_32_x2(arg0);
}

uint64_t HELPER(fsrsr_32_x2)(uint64_t arg0)
{
    return lao_fsrsr_32_x2(arg0);
}

uint64_t HELPER(fwiden_16_32_x2)(uint64_t arg0)
{
    return lao_fwiden_16_32_x2(arg0);
}

uint64_t HELPER(fsdiv_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_fsdiv_64(arg0, arg1);
}

uint64_t HELPER(fcdiv_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_fcdiv_64(arg0, arg1);
}

uint64_t HELPER(fsdiv_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_fsdiv_32(arg0, arg1);
}

uint64_t HELPER(fcdiv_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_fcdiv_32(arg0, arg1);
}

uint64_t HELPER(fsdiv_32_x2)(uint64_t arg0, uint64_t arg1)
{
    return lao_fsdiv_32_x2(arg0, arg1);
}

uint64_t HELPER(fcdiv_32_x2)(uint64_t arg0, uint64_t arg1)
{
    return lao_fcdiv_32_x2(arg0, arg1);
}

uint64_t HELPER(fmin_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmin_64(arg0, arg1);
}

uint64_t HELPER(fmax_64)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmax_64(arg0, arg1);
}

uint64_t HELPER(fmin_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmin_32(arg0, arg1);
}

uint64_t HELPER(fmax_32)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmax_32(arg0, arg1);
}

uint64_t HELPER(fmin_32_x2)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmin_32_x2(arg0, arg1);
}

uint64_t HELPER(fmax_32_x2)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmax_32_x2(arg0, arg1);
}

uint64_t HELPER(fmin_16_x4)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmin_16_x4(arg0, arg1);
}

uint64_t HELPER(fmax_16_x4)(uint64_t arg0, uint64_t arg1)
{
    return lao_fmax_16_x4(arg0, arg1);
}

uint64_t HELPER(float_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_float_64(arg0, arg1, arg2);
}

uint64_t HELPER(float_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_float_32_x2(arg0, arg1, arg2);
}

uint64_t HELPER(float_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_float_32(arg0, arg1, arg2);
}

uint64_t HELPER(floatu_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_floatu_64(arg0, arg1, arg2);
}

uint64_t HELPER(floatu_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_floatu_32_x2(arg0, arg1, arg2);
}

uint64_t HELPER(floatu_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_floatu_32(arg0, arg1, arg2);
}

uint64_t HELPER(fixed_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fixed_64(arg0, arg1, arg2);
}

uint64_t HELPER(fixed_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fixed_32_x2(arg0, arg1, arg2);
}

uint64_t HELPER(fixed_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fixed_32(arg0, arg1, arg2);
}

uint64_t HELPER(fixedu_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fixedu_64(arg0, arg1, arg2);
}

uint64_t HELPER(fixedu_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fixedu_32_x2(arg0, arg1, arg2);
}

uint64_t HELPER(fixedu_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fixedu_32(arg0, arg1, arg2);
}

uint64_t HELPER(ffma_64_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffma_64_64(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffma_32_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffma_32_32_x2(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffma_16_16_x4)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffma_16_16_x4(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffms_64_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffms_64_64(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffms_32_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffms_32_32_x2(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffms_16_16_x4)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffms_16_16_x4(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(fmul_64_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmul_64_64(arg0, arg1, arg2);
}

uint64_t HELPER(fmul_32_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmul_32_32_x2(arg0, arg1, arg2);
}

uint64_t HELPER(fmul_16_16_x4)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmul_16_16_x4(arg0, arg1, arg2);
}

void HELPER(fmul_32_64_x2)(CPUKVXState *env, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    lao_fmul_32_64_x2(&env->scratch[0], &env->scratch[1], arg0, arg1, arg2);
}

void HELPER(fmul_16_32_x4)(CPUKVXState *env, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    lao_fmul_16_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1, arg2);
}

uint64_t HELPER(fmulc_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmulc_32_32(arg0, arg1, arg2);
}

void HELPER(fmulc_32_64)(CPUKVXState *env, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    lao_fmulc_32_64(&env->scratch[0], &env->scratch[1], arg0, arg1, arg2);
}

uint64_t HELPER(fmulcc_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmulcc_32_32(arg0, arg1, arg2);
}

void HELPER(fmulcc_32_64)(CPUKVXState *env, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    lao_fmulcc_32_64(&env->scratch[0], &env->scratch[1], arg0, arg1, arg2);
}

uint64_t HELPER(fadd_64_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fadd_64_64(arg0, arg1, arg2);
}

uint64_t HELPER(fadd_32_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fadd_32_32_x2(arg0, arg1, arg2);
}

uint64_t HELPER(fadd_16_16_x4)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fadd_16_16_x4(arg0, arg1, arg2);
}

uint64_t HELPER(faddcc_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_faddcc_32_32(arg0, arg1, arg2);
}

uint64_t HELPER(fsbf_64_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fsbf_64_64(arg0, arg1, arg2);
}

uint64_t HELPER(fsbf_32_32_x2)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fsbf_32_32_x2(arg0, arg1, arg2);
}

uint64_t HELPER(fsbf_16_16_x4)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fsbf_16_16_x4(arg0, arg1, arg2);
}

uint64_t HELPER(fsbfcc_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fsbfcc_32_32(arg0, arg1, arg2);
}

uint64_t HELPER(ffma_16_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffma_16_32(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffma_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffma_32_32(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffma_32_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffma_32_64(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffms_16_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffms_16_32(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffms_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffms_32_32(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(ffms_32_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_ffms_32_64(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(fadd_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fadd_32_32(arg0, arg1, arg2);
}

uint64_t HELPER(fsbf_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fsbf_32_32(arg0, arg1, arg2);
}

uint64_t HELPER(fmul_16_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmul_16_32(arg0, arg1, arg2);
}

uint64_t HELPER(fmul_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmul_32_32(arg0, arg1, arg2);
}

uint64_t HELPER(fmul_32_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fmul_32_64(arg0, arg1, arg2);
}

uint64_t HELPER(fdot2_32_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fdot2_32_32(arg0, arg1, arg2);
}

uint64_t HELPER(fdot2_32_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_fdot2_32_64(arg0, arg1, arg2);
}

void HELPER(fmm2wq)(CPUKVXState *env, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    lao_fmm2wq(&env->scratch[0], &env->scratch[1], arg0, arg1, arg2);
}

uint64_t HELPER(fdot4add_16_32_reset)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_fdot4add_16_32_reset(arg0, arg1, arg2, arg3);
}

uint64_t HELPER(fdot4add_16_32_noreset)(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return lao_fdot4add_16_32_noreset(arg0, arg1, arg2, arg3);
}

void HELPER(fpow2scale_32_32_x8)(CPUKVXState *env, uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5)
{
    lao_fpow2scale_32_32_x8(&env->scratch[0], &env->scratch[1],
                            &env->scratch[2], &env->scratch[3],
                            arg0, arg1, arg2, arg3, arg4, arg5);
}

void HELPER(frelu_32_32_x8)(CPUKVXState *env, uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    lao_frelu_32_32_x8(&env->scratch[0], &env->scratch[1],
                       &env->scratch[2], &env->scratch[3],
                       arg0, arg1, arg2, arg3);
}

void HELPER(fnarrow_32_16_x8_reset)(CPUKVXState *env, uint64_t arg0,
                                        uint64_t arg1_0, uint64_t arg1_1,
                                        uint64_t arg1_2, uint64_t arg1_3)
{
    lao_fnarrow_32_16_x8_reset(&env->scratch[0], &env->scratch[1],
                               &env->scratch[2], &env->scratch[3],
                               arg0, arg1_0, arg1_1, arg1_2, arg1_3);
}

void HELPER(fnarrow_32_16_x8_noreset)(CPUKVXState *env, uint64_t arg0,
                                        uint64_t arg1_0, uint64_t arg1_1,
                                        uint64_t arg1_2, uint64_t arg1_3)
{
    lao_fnarrow_32_16_x8_noreset(&env->scratch[0], &env->scratch[1],
                               &env->scratch[2], &env->scratch[3],
                               arg0, arg1_0, arg1_1, arg1_2, arg1_3);
}

void HELPER(fmul_16_16_x8)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fmul_16_16_x8(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fmul_32_32_x4)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fmul_32_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fadd_16_16_x8)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fadd_64_64_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fadd_64_64_x2)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fadd_64_64_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fadd_32_32_x4)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fadd_32_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(faddcc_32_32_x2)(CPUKVXState *env, uint64_t arg0,
                             uint64_t arg1_l, uint64_t arg1_h,
                             uint64_t arg2_l, uint64_t arg2_h)
{
    lao_faddcc_32_32_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(faddcc_64_64)(CPUKVXState *env, uint64_t arg0,
                             uint64_t arg1_l, uint64_t arg1_h,
                             uint64_t arg2_l, uint64_t arg2_h)
{
    lao_faddcc_64_64(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fsbf_16_16_x8)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fsbf_16_16_x8(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fsbf_64_64_x2)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fsbf_64_64_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fsbf_32_32_x4)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fsbf_32_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fsbfcc_32_32_x2)(CPUKVXState *env, uint64_t arg0,
                             uint64_t arg1_l, uint64_t arg1_h,
                             uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fsbfcc_32_32_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fsbfcc_64_64)(CPUKVXState *env, uint64_t arg0,
                             uint64_t arg1_l, uint64_t arg1_h,
                             uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fsbfcc_64_64(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(ffma_32_64_x2)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    lao_ffma_32_64_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3);
}

void HELPER(ffma_16_32_x4)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    lao_ffma_16_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3);
}

void HELPER(ffma_16_16_x8)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    uint64_t arg4 = env->scratch[0];
    uint64_t arg5 = env->scratch[1];

    lao_ffma_16_16_x8(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3, arg4, arg5);
}

void HELPER(ffma_32_32_x4)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    uint64_t arg4 = env->scratch[0];
    uint64_t arg5 = env->scratch[1];

    lao_ffma_32_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3, arg4, arg5);
}

void HELPER(ffms_16_16_x8)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    uint64_t arg4 = env->scratch[0];
    uint64_t arg5 = env->scratch[1];

    lao_ffms_16_16_x8(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3, arg4, arg5);
}

void HELPER(ffms_32_32_x4)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    uint64_t arg4 = env->scratch[0];
    uint64_t arg5 = env->scratch[1];

    lao_ffms_32_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3, arg4, arg5);
}

void HELPER(ffms_32_64_x2)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    lao_ffms_32_64_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3);
}

void HELPER(ffms_16_32_x4)(CPUKVXState *env, uint64_t arg0,
                           uint64_t arg1_l, uint64_t arg1_h,
                           uint64_t arg2, uint64_t arg3)
{
    lao_ffms_16_32_x4(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3);
}

void HELPER(fdot2_32_32_x2)(CPUKVXState *env, uint64_t arg0,
                            uint64_t arg1_l, uint64_t arg1_h,
                            uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fdot2_32_32_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fdot2_32_64_x2)(CPUKVXState *env, uint64_t arg0,
                            uint64_t arg1_l, uint64_t arg1_h,
                            uint64_t arg2_l, uint64_t arg2_h)
{
    lao_fdot2_32_64_x2(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2_l, arg2_h);
}

void HELPER(fmm2awq)(CPUKVXState *env, uint64_t arg0,
                     uint64_t arg1_l, uint64_t arg1_h,
                     uint64_t arg2, uint64_t arg3)
{
    lao_fmm2awq(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3);
}

void HELPER(fmm2swq)(CPUKVXState *env, uint64_t arg0,
                     uint64_t arg1_l, uint64_t arg1_h,
                     uint64_t arg2, uint64_t arg3)
{
    lao_fmm2swq(&env->scratch[0], &env->scratch[1], arg0, arg1_l, arg1_h, arg2, arg3);
}

uint64_t HELPER(floatcomp_16)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_floatcomp_16(arg0, arg1, arg2);
}

uint64_t HELPER(floatcomp_32)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_floatcomp_32(arg0, arg1, arg2);
}

uint64_t HELPER(floatcomp_64)(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return lao_floatcomp_64(arg0, arg1, arg2);
}
