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

#ifndef _LAO_WRAPPER_H
#define _LAO_WRAPPER_H

#include <stdint.h>

uint64_t lao_bmm_8(uint64_t arg0, uint64_t arg1);
uint64_t lao_bmt_8(uint64_t arg0);

void lao_clm_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1);
void lao_gcm_bb_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1);
void lao_gcm_bt_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1);
void lao_gcm_tt_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1);

uint64_t lao_fnarrow_64_32_x2(uint64_t rm, uint64_t arg0_l, uint64_t arg0_h);
uint64_t lao_fsrec_64(uint64_t arg0);
uint64_t lao_fdivbyzero(void);
uint64_t lao_finexact(void);
uint64_t lao_finvalid(void);
uint64_t lao_foverflow(void);
uint64_t lao_funderflow(void);

uint64_t lao_crc32_be_u32(uint64_t arg0, uint64_t arg1);
uint64_t lao_reflect_32(uint64_t arg0);
uint64_t lao_dot8_8_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_dot8u_8_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_dot8su_8_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_dot8us_8_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_sxb_x4(uint64_t arg0);
uint64_t lao_dot4_16_64(uint64_t arg0, uint64_t arg1);
uint64_t lao_zxb_x4(uint64_t arg0);
uint64_t lao_dot4u_16_64(uint64_t arg0, uint64_t arg1);
uint64_t lao_dot4su_16_64(uint64_t arg0, uint64_t arg1);
uint64_t lao_dot4us_16_64(uint64_t arg0, uint64_t arg1);
void lao_round_64_x4(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                     uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5);
void lao_round_32_x8(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                     uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5);
uint64_t lao_satu_32_8_x8(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_sat_32_8_x8(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_fnarrow_32_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fsrsr_64(uint64_t arg0);
uint64_t lao_fwiden_32_64(uint64_t arg0);
uint64_t lao_fnarrow_64_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_frec_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_frsq_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_fsrec_32(uint64_t arg0);
uint64_t lao_fsrsr_32(uint64_t arg0);
uint64_t lao_fwiden_16_32(uint64_t arg0);
uint64_t lao_fnarrow_32_16(uint64_t arg0, uint64_t arg1);
uint64_t lao_fsrec_32_x2(uint64_t arg0);
uint64_t lao_fsrsr_32_x2(uint64_t arg0);
uint64_t lao_fwiden_16_32_x2(uint64_t arg0);
uint64_t lao_fsdiv_64(uint64_t arg0, uint64_t arg1);
uint64_t lao_fcdiv_64(uint64_t arg0, uint64_t arg1);
uint64_t lao_fsdiv_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_fcdiv_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_fsdiv_32_x2(uint64_t arg0, uint64_t arg1);
uint64_t lao_fcdiv_32_x2(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmin_64(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmax_64(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmin_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmax_32(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmin_32_x2(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmax_32_x2(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmin_16_x4(uint64_t arg0, uint64_t arg1);
uint64_t lao_fmax_16_x4(uint64_t arg0, uint64_t arg1);
uint64_t lao_float_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_float_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_float_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_floatu_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_floatu_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_floatu_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fixed_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fixed_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fixed_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fixedu_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fixedu_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fixedu_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_ffma_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffma_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffma_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffms_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffms_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffms_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_fmul_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fmul_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fmul_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2);
void lao_fmul_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2);
void lao_fmul_16_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fmulc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
void lao_fmulc_32_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fmulcc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
void lao_fmulcc_32_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fadd_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fadd_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fadd_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_faddcc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fsbf_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fsbf_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fsbf_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fsbfcc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_ffma_16_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffma_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffma_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffms_16_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffms_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_ffms_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_fadd_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fsbf_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fmul_16_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fmul_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fmul_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fdot2_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fdot2_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
void lao_fmm2wq(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_fdot4add_16_32_reset(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
uint64_t lao_fdot4add_16_32_noreset(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3);
void lao_fpow2scale_32_32_x8(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3, uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5);
void lao_frelu_32_32_x8(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                        uint64_t arg0_0, uint64_t arg0_1, uint64_t arg0_2, uint64_t arg0_3);
void lao_fnarrow_32_16_x8_reset(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                                uint64_t arg0, uint64_t arg1_0, uint64_t arg1_1, uint64_t arg1_2, uint64_t arg1_3);
void lao_fnarrow_32_16_x8_noreset(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                                uint64_t arg0, uint64_t arg1_0, uint64_t arg1_1, uint64_t arg1_2, uint64_t arg1_3);

void lao_fmul_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fmul_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fadd_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fadd_64_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fadd_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_faddcc_32_32_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_faddcc_64_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fsbf_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fsbf_64_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fsbf_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fsbfcc_32_32_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fsbfcc_64_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_ffma_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3);
void lao_ffma_16_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3);
void lao_ffma_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h);
void lao_ffma_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h);
void lao_ffms_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h);
void lao_ffms_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h);
void lao_ffms_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3);
void lao_ffms_16_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3);
void lao_fdot2_32_32_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fdot2_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h);
void lao_fmm2awq(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3);
void lao_fmm2swq(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3);

uint64_t lao_floatcomp_16(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_floatcomp_32(uint64_t arg0, uint64_t arg1, uint64_t arg2);
uint64_t lao_floatcomp_64(uint64_t arg0, uint64_t arg1, uint64_t arg2);
#endif
