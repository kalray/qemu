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

#include <MDT/MDT_.h>

#include "lao-wrapper.h"

uint64_t lao_bmm_8(uint64_t arg0, uint64_t arg1)
{
    return uint64_bmm8(arg0, arg1);
}

uint64_t lao_bmt_8(uint64_t arg0)
{
    return uint64_bmt8(arg0);
}

void lao_clm_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1)
{
    Int256_ ret;

    ret = Behavior_clm_64_128(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1));

    *res_l = ret.dwords[0];
    *res_h = ret.dwords[1];
}

void lao_gcm_bb_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1)
{
    Int256_ ret;

    ret = Behavior_gcm_bb_64_128(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1));

    *res_l = ret.dwords[0];
    *res_h = ret.dwords[1];
}

void lao_gcm_bt_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1)
{
    Int256_ ret;

    ret = Behavior_gcm_bt_64_128(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1));

    *res_l = ret.dwords[0];
    *res_h = ret.dwords[1];
}

void lao_gcm_tt_64_128(uint64_t *res_l, uint64_t *res_h, uint64_t arg0, uint64_t arg1)
{
    Int256_ ret;

    ret = Behavior_gcm_tt_64_128(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1));

    *res_l = ret.dwords[0];
    *res_h = ret.dwords[1];
}

uint64_t lao_fnarrow_64_32_x2(uint64_t rm, uint64_t arg0_l, uint64_t arg0_h)
{
    Int256_ arg0_256;

    arg0_256 = Int256_fromUInt64(arg0_l);
    arg0_256.dwords[1] = arg0_h;

    return Int256_toUInt64(Behavior_fnarrow_64_32_x2(NULL,
                                                     Int256_fromUInt64(rm), arg0_256));
}

uint64_t lao_fsrec_64(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fsrec_64(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fdivbyzero(void)
{
    return Int256_toUInt64(Behavior_fdivbyzero(NULL));
}

uint64_t lao_finexact(void)
{
    return Int256_toUInt64(Behavior_finexact(NULL));
}

uint64_t lao_finvalid(void)
{
    return Int256_toUInt64(Behavior_finvalid(NULL));
}

uint64_t lao_foverflow(void)
{
    return Int256_toUInt64(Behavior_foverflow(NULL));
}

uint64_t lao_funderflow(void)
{
    return Int256_toUInt64(Behavior_funderflow(NULL));
}

uint64_t lao_crc32_be_u32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_crc32_be_u32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_reflect_32(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_reflect_32(NULL,Int256_fromUInt64(arg0)));
}

uint64_t lao_dot8_8_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot8_8_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_dot8u_8_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot8u_8_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_dot8su_8_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot8su_8_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_dot8us_8_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot8us_8_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_sxb_x4(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_sxb_x4(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_dot4_16_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot4_16_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_zxb_x4(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_zxb_x4(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_dot4u_16_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot4u_16_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_dot4su_16_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot4su_16_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_dot4us_16_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_dot4us_16_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

void lao_round_64_x4(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                     uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5)
{
    Int256_ ret;
    Int256_ in = Int256_fromUInt64(arg2);
    in.dwords[1] = arg3;
    in.dwords[2] = arg4;
    in.dwords[3] = arg5;

    ret = Behavior_round_64_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), in);

    *ret_0 = ret.dwords[0];
    *ret_1 = ret.dwords[1];
    *ret_2 = ret.dwords[2];
    *ret_3 = ret.dwords[3];
}

void lao_round_32_x8(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                     uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5)
{
    Int256_ ret;
    Int256_ in = Int256_fromUInt64(arg2);
    in.dwords[1] = arg3;
    in.dwords[2] = arg4;
    in.dwords[3] = arg5;

    ret = Behavior_round_32_x8(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), in);

    *ret_0 = ret.dwords[0];
    *ret_1 = ret.dwords[1];
    *ret_2 = ret.dwords[2];
    *ret_3 = ret.dwords[3];
}

uint64_t lao_satu_32_8_x8(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    Int256_ in = Int256_fromUInt64(arg0);
    in.dwords[1] = arg1;
    in.dwords[2] = arg2;
    in.dwords[3] = arg3;

    return Int256_toUInt64(Behavior_satu_32_8_x8(NULL, in));
}

uint64_t lao_sat_32_8_x8(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    Int256_ in = Int256_fromUInt64(arg0);
    in.dwords[1] = arg1;
    in.dwords[2] = arg2;
    in.dwords[3] = arg3;

    return Int256_toUInt64(Behavior_sat_32_8_x8(NULL, in));
}

uint64_t lao_fnarrow_32_16_x4(uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h)
{
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    arg1.dwords[1] = arg1_h;

    return Int256_toUInt64(Behavior_fnarrow_32_16_x4(NULL, Int256_fromUInt64(arg0), arg1));
}

uint64_t lao_fsrsr_64(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fsrsr_64(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fwiden_32_64(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fwiden_32_64(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fnarrow_64_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fnarrow_64_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_frec_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_frec_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_frsq_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_frsq_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fsrec_32(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fsrec_32(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fsrsr_32(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fsrsr_32(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fwiden_16_32(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fwiden_16_32(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fnarrow_32_16(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fnarrow_32_16(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fsrec_32_x2(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fsrec_32_x2(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fsrsr_32_x2(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fsrsr_32_x2(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fwiden_16_32_x2(uint64_t arg0)
{
    return Int256_toUInt64(Behavior_fwiden_16_32_x2(NULL, Int256_fromUInt64(arg0)));
}

uint64_t lao_fsdiv_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fsdiv_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fcdiv_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fcdiv_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fsdiv_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fsdiv_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fcdiv_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fcdiv_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fsdiv_32_x2(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fsdiv_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fcdiv_32_x2(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fcdiv_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmin_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmin_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmax_64(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmax_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmin_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmin_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmax_32(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmax_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmin_32_x2(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmin_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmax_32_x2(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmax_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmin_16_x4(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmin_16_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_fmax_16_x4(uint64_t arg0, uint64_t arg1)
{
    return Int256_toUInt64(Behavior_fmax_16_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1)));
}

uint64_t lao_float_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_float_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_float_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_float_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_float_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_float_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_floatu_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_floatu_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_floatu_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_floatu_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_floatu_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_floatu_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fixed_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fixed_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fixed_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fixed_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fixed_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fixed_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fixedu_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fixedu_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fixedu_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fixedu_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fixedu_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fixedu_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_ffma_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffma_64_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffma_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffma_32_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffma_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffma_16_16_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffms_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffms_64_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffms_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffms_32_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffms_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffms_16_16_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_fmul_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmul_64_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fmul_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmul_32_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fmul_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmul_16_16_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

void lao_fmul_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    Int256_ ret;

    ret = Behavior_fmul_32_64_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fmul_16_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    Int256_ ret;

    ret = Behavior_fmul_16_32_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

uint64_t lao_fmulc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmulc_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

void lao_fmulc_32_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    Int256_ ret;

    ret = Behavior_fmulc_32_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

uint64_t lao_fmulcc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmulcc_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

void lao_fmulcc_32_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    Int256_ ret;

    ret = Behavior_fmulcc_32_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

uint64_t lao_fadd_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fadd_64_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fadd_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fadd_32_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fadd_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fadd_16_16_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_faddcc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_faddcc_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fsbf_64_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fsbf_64_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fsbf_32_32_x2(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fsbf_32_32_x2(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fsbf_16_16_x4(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fsbf_16_16_x4(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fsbfcc_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fsbfcc_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_ffma_16_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffma_16_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffma_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffma_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffma_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffma_32_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffms_16_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffms_16_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffms_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffms_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_ffms_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_ffms_32_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_fadd_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fadd_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fsbf_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fsbf_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fmul_16_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmul_16_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fmul_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmul_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fmul_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fmul_32_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fdot2_32_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fdot2_32_32(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

uint64_t lao_fdot2_32_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Int256_toUInt64(Behavior_fdot2_32_64(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2)));
}

void lao_fmm2wq(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    Int256_ ret;

    ret = Behavior_fmm2wq(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

uint64_t lao_fdot4add_16_32_reset(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_fdot4add_16_32_reset(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

uint64_t lao_fdot4add_16_32_noreset(uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    return Int256_toUInt64(Behavior_fdot4add_16_32_noreset(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), Int256_fromUInt64(arg2), Int256_fromUInt64(arg3)));
}

void lao_fpow2scale_32_32_x8(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                             uint64_t arg0, uint64_t arg1,
                             uint64_t arg2_0, uint64_t arg2_1, uint64_t arg2_2, uint64_t arg2_3)
{
    Int256_ ret;
    Int256_ arg2 = Int256_fromUInt64(arg2_0);

    arg2.dwords[1] = arg2_1;
    arg2.dwords[2] = arg2_2;
    arg2.dwords[3] = arg2_3;

    ret = Behavior_fpow2scale_32_32_x8(NULL, Int256_fromUInt64(arg0), Int256_fromUInt64(arg1), arg2);

    *ret_0 = ret.dwords[0];
    *ret_1 = ret.dwords[1];
    *ret_2 = ret.dwords[2];
    *ret_3 = ret.dwords[3];
}

void lao_frelu_32_32_x8(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                        uint64_t arg0_0, uint64_t arg0_1, uint64_t arg0_2, uint64_t arg0_3)
{
    Int256_ ret;
    Int256_ arg0 = Int256_fromUInt64(arg0_0);

    arg0.dwords[1] = arg0_1;
    arg0.dwords[2] = arg0_2;
    arg0.dwords[3] = arg0_3;

    ret = Behavior_frelu_32_32_x8_noreset(NULL, arg0);

    *ret_0 = ret.dwords[0];
    *ret_1 = ret.dwords[1];
    *ret_2 = ret.dwords[2];
    *ret_3 = ret.dwords[3];
}

void lao_fnarrow_32_16_x8_part0(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                                uint64_t arg0, uint64_t arg1_0, uint64_t arg1_1, uint64_t arg1_2, uint64_t arg1_3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_0);

    arg1.dwords[1] = arg1_1;
    arg1.dwords[2] = arg1_2;
    arg1.dwords[3] = arg1_3;

    ret = Behavior_fnarrow_32_16_x8_part0(NULL, Int256_fromUInt64(arg0), arg1);

    *ret_0 = ret.dwords[0];
    *ret_1 = ret.dwords[1];
    *ret_2 = ret.dwords[2];
    *ret_3 = ret.dwords[3];
}

void lao_fnarrow_32_16_x8_part1(uint64_t *ret_0, uint64_t *ret_1, uint64_t *ret_2, uint64_t *ret_3,
                                uint64_t arg0, uint64_t arg1_0, uint64_t arg1_1, uint64_t arg1_2, uint64_t arg1_3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_0);

    arg1.dwords[1] = arg1_1;
    arg1.dwords[2] = arg1_2;
    arg1.dwords[3] = arg1_3;

    ret = Behavior_fnarrow_32_16_x8_part1(NULL, Int256_fromUInt64(arg0), arg1);

    *ret_0 = ret.dwords[0];
    *ret_1 = ret.dwords[1];
    *ret_2 = ret.dwords[2];
    *ret_3 = ret.dwords[3];
}


void lao_fmul_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fmul_16_16_x8(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fmul_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fmul_32_32_x4(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fadd_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fadd_16_16_x8(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fadd_64_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fadd_64_64_x2(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fadd_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fadd_32_32_x4(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_faddcc_32_32_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_faddcc_32_32_x2(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_faddcc_64_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_faddcc_64_64(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fsbf_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fsbf_16_16_x8(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fsbf_64_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fsbf_64_64_x2(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fsbf_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fsbf_32_32_x4(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fsbfcc_32_32_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fsbfcc_32_32_x2(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fsbfcc_64_64(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fsbfcc_64_64(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffma_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);

    arg1.dwords[1] = arg1_h;

    ret = Behavior_ffma_32_64_x2(NULL, Int256_fromUInt64(arg0), arg1, Int256_fromUInt64(arg2), Int256_fromUInt64(arg3));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffma_16_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);

    arg1.dwords[1] = arg1_h;

    ret = Behavior_ffma_16_32_x4(NULL, Int256_fromUInt64(arg0), arg1, Int256_fromUInt64(arg2), Int256_fromUInt64(arg3));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffma_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);
    Int256_ arg3 = Int256_fromUInt64(arg3_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;
    arg3.dwords[1] = arg3_h;

    ret = Behavior_ffma_16_16_x8(NULL, Int256_fromUInt64(arg0), arg1, arg2, arg3);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffma_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);
    Int256_ arg3 = Int256_fromUInt64(arg3_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;
    arg3.dwords[1] = arg3_h;

    ret = Behavior_ffma_32_32_x4(NULL, Int256_fromUInt64(arg0), arg1, arg2, arg3);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffms_16_16_x8(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);
    Int256_ arg3 = Int256_fromUInt64(arg3_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;
    arg3.dwords[1] = arg3_h;

    ret = Behavior_ffms_16_16_x8(NULL, Int256_fromUInt64(arg0), arg1, arg2, arg3);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffms_32_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h, uint64_t arg3_l, uint64_t arg3_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);
    Int256_ arg3 = Int256_fromUInt64(arg3_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;
    arg3.dwords[1] = arg3_h;

    ret = Behavior_ffms_32_32_x4(NULL, Int256_fromUInt64(arg0), arg1, arg2, arg3);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffms_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);

    arg1.dwords[1] = arg1_h;

    ret = Behavior_ffms_32_64_x2(NULL, Int256_fromUInt64(arg0), arg1, Int256_fromUInt64(arg2), Int256_fromUInt64(arg3));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_ffms_16_32_x4(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);

    arg1.dwords[1] = arg1_h;

    ret = Behavior_ffms_16_32_x4(NULL, Int256_fromUInt64(arg0), arg1, Int256_fromUInt64(arg2), Int256_fromUInt64(arg3));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fdot2_32_32_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fdot2_32_32_x2(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fdot2_32_64_x2(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2_l, uint64_t arg2_h)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);
    Int256_ arg2 = Int256_fromUInt64(arg2_l);

    arg1.dwords[1] = arg1_h;
    arg2.dwords[1] = arg2_h;

    ret = Behavior_fdot2_32_64_x2(NULL, Int256_fromUInt64(arg0), arg1, arg2);

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fmm2awq(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);

    arg1.dwords[1] = arg1_h;

    ret = Behavior_fmm2awq(NULL, Int256_fromUInt64(arg0), arg1, Int256_fromUInt64(arg2), Int256_fromUInt64(arg3));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

void lao_fmm2swq(uint64_t *ret_l, uint64_t *ret_h, uint64_t arg0, uint64_t arg1_l, uint64_t arg1_h, uint64_t arg2, uint64_t arg3)
{
    Int256_ ret;
    Int256_ arg1 = Int256_fromUInt64(arg1_l);

    arg1.dwords[1] = arg1_h;

    ret = Behavior_fmm2swq(NULL, Int256_fromUInt64(arg0), arg1, Int256_fromUInt64(arg2), Int256_fromUInt64(arg3));

    *ret_l = ret.dwords[0];
    *ret_h = ret.dwords[1];
}

uint64_t lao_floatcomp_16(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Behavior_floatcomp_16(NULL,
                                 Int256_fromUInt64(arg0),
                                 Int256_fromUInt64(arg1),
                                 Int256_fromUInt64(arg2));
}

uint64_t lao_floatcomp_32(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Behavior_floatcomp_32(NULL,
                                 Int256_fromUInt64(arg0),
                                 Int256_fromUInt64(arg1),
                                 Int256_fromUInt64(arg2));
}

uint64_t lao_floatcomp_64(uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    return Behavior_floatcomp_64(NULL,
                                 Int256_fromUInt64(arg0),
                                 Int256_fromUInt64(arg1),
                                 Int256_fromUInt64(arg2));
}
