#ifndef _MDS_TCG_BIG_INT_H
#define _MDS_TCG_BIG_INT_H

#define MDS_TCG_BIGINT_MAX_SZ 16  /* 1024 bits */

#include "tcg/tcg-op.h"

typedef struct MDSTCGBigInt {
    size_t size;
    bool local;
    TCGv_i64 val[MDS_TCG_BIGINT_MAX_SZ];

    /* Iterator for the FOREACH macro */
    ssize_t i;
} MDSTCGBigInt;

#define MDS_BIGINT_FOREACH(_b, _val) \
    for ((_b)->i = 0, _val = (_b)->val[0]; \
         (_b)->i < (_b)->size; \
         (_b)->i++, _val = (_b)->val[(_b)->i])

#define MDS_BIGINT_FOREACH_REVERSE(_b, _val) \
    for ((_b)->i = (_b)->size - 1, _val = (_b)->val[(_b)->i]; \
         (_b)->i >= 0; \
         (_b)->i--, _val = (_b)->val[(_b)->i])

#define MDS_BIGINT_FOREACH_2(_b0, _val0, _b1, _val1) \
    for ((_b0)->i = 0, _val0 = (_b0)->val[0], _val1 = (_b1)->val[0]; \
         (_b0)->i < (_b0)->size; \
         (_b0)->i++, _val0 = (_b0)->val[(_b0)->i], _val1 = (_b1)->val[(_b0)->i])

#define MDS_BIGINT_FOREACH_3(_b0, _val0, _b1, _val1, _b2, _val2) \
    for ((_b0)->i = 0, _val0 = (_b0)->val[0], _val1 = (_b1)->val[0], _val2 = (_b2)->val[0]; \
         (_b0)->i < (_b0)->size; \
         (_b0)->i++, _val0 = (_b0)->val[(_b0)->i], _val1 = (_b1)->val[(_b0)->i], _val2 = (_b2)->val[(_b0)->i])


static inline void mds_tcg_bigint_resize(MDSTCGBigInt *v, size_t new_size)
{
    size_t min, max, i;

    new_size /= 64;

    g_assert(new_size < MDS_TCG_BIGINT_MAX_SZ);

    min = MIN(v->size, new_size);
    max = MAX(v->size, new_size);

    for (i = min; i < max; i++) {
        if (v->size < new_size) {
            v->val[i] = v->local ? tcg_temp_local_new_i64() : tcg_temp_new_i64();
        } else {
            tcg_temp_free(v->val[i]);
        }
    }

    v->size = new_size;
}

static inline void mds_tcg_bigint_new(MDSTCGBigInt *v, size_t size)
{
    v->size = 0;
    v->local = false;
    mds_tcg_bigint_resize(v, size);
}

static inline void mds_tcg_bigint_local_new(MDSTCGBigInt *v, size_t size)
{
    v->size = 0;
    v->local = true;
    mds_tcg_bigint_resize(v, size);
}

static inline void mds_tcg_bigint_free(MDSTCGBigInt *v)
{
    mds_tcg_bigint_resize(v, 0);
}

static inline size_t mds_tcg_bigint_get_size(MDSTCGBigInt *v)
{
    return v->size * 64;
}

static inline size_t mds_tcg_bigint_get_cur_idx(MDSTCGBigInt *v)
{
    return v->i;
}

static inline void mds_tcg_bigint_to_i64(TCGv_i64 ret, MDSTCGBigInt *v)
{
    g_assert(v->size > 0);

    tcg_gen_mov_i64(ret, v->val[0]);
}

/* Signed arithmetic saturation */
static inline void mds_tcg_bigint_to_i64_sat(TCGv_i64 ret, MDSTCGBigInt *v,
                                             size_t sat_size)
{
    size_t i;
    TCGv_i64 sign, upper, sat, zero, tmp;

    g_assert(sat_size <= 64);
    g_assert(v->size > 1);

    /* Signed saturation: the effective saturation is done on sat_size - 1 */
    sat_size--;

    sign = tcg_temp_new_i64();
    tcg_gen_sari_i64(sign, v->val[v->size - 1], 63);

    upper = tcg_temp_new_i64();
    tcg_gen_xor_i64(upper, v->val[1], sign);

    tmp = tcg_temp_new_i64();
    for (i = 2; i < v->size; i++) {
        tcg_gen_xor_i64(tmp, v->val[i], sign);
        tcg_gen_or_i64(upper, upper, tmp);
    }
    tcg_temp_free_i64(tmp);

    sat = tcg_const_i64((1ull << sat_size) - 1);

    zero = tcg_const_i64(0);
    tcg_gen_movcond_i64(TCG_COND_NE, ret, upper, zero, sat, v->val[0]);
    tcg_temp_free_i64(zero);
    tcg_temp_free_i64(upper);

    tcg_gen_xor_i64(ret, ret, sign);
    tcg_gen_umin_i64(ret, ret, sat);
    tcg_gen_xor_i64(ret, ret, sign);

    tcg_temp_free_i64(sat);

    tcg_temp_free_i64(sign);
}

static inline void mds_tcg_bigint_to_i64_satu(TCGv_i64 ret, MDSTCGBigInt *v,
                                              size_t sat_size)
{
    size_t i;
    TCGv_i64 sign, upper;

    g_assert(sat_size <= 64);


    /* Check for non-zero bits in upper (>=64th bits) parts */
    upper = tcg_const_i64(0);
    for (i = 1; i < v->size; i++) {
        tcg_gen_or_i64(upper, upper, v->val[i]);
    }

    /* Set upper to UINT64_MAX if some bits are set, 0 otherwise */
    tcg_gen_movcond_i64(TCG_COND_NE, upper,
                        upper, tcg_constant_i64(0),
                        tcg_constant_i64(UINT64_MAX), tcg_constant_i64(0));


    /* ret <- absolute saturated value on 64 bits */
    tcg_gen_or_i64(ret, v->val[0], upper);
    tcg_temp_free_i64(upper);

    /* ret <- absolute saturated value on sat_size bits */
    tcg_gen_umin_i64(ret, ret, tcg_constant_i64((1ull << sat_size) - 1));

    /* invert ret if v is negative */
    sign = tcg_temp_new_i64();
    tcg_gen_sari_i64(sign, v->val[v->size - 1], 63);
    tcg_gen_shri_i64(sign, sign, 64 - sat_size);
    tcg_gen_xor_i64(ret, ret, sign);
    tcg_temp_free_i64(sign);
}

static inline void mds_tcg_i64_to_bigint_u(MDSTCGBigInt *ret, TCGv_i64 v, size_t ret_size)
{
    size_t i;

    mds_tcg_bigint_resize(ret, ret_size);

    tcg_gen_mov_i64(ret->val[0], v);

    for (i = 1; i < ret->size; i++) {
        tcg_gen_movi_i64(ret->val[i], 0);
    }
}

static inline void mds_tcg_imm64_to_bigint_u(MDSTCGBigInt *ret, uint64_t v, size_t ret_size)
{
    size_t i;

    mds_tcg_bigint_resize(ret, ret_size);

    tcg_gen_movi_i64(ret->val[0], v);

    for (i = 1; i < ret->size; i++) {
        tcg_gen_movi_i64(ret->val[i], 0);
    }
}

static inline void mds_tcg_i64_to_bigint_s(MDSTCGBigInt *ret, TCGv_i64 v, size_t ret_size)
{
    size_t i;
    TCGv_i64 sign;

    mds_tcg_bigint_resize(ret, ret_size);

    tcg_gen_mov_i64(ret->val[0], v);

    sign = tcg_temp_new_i64();
    tcg_gen_sari_i64(sign, ret->val[0], 63);

    for (i = 1; i < ret->size; i++) {
        tcg_gen_mov_i64(ret->val[i], sign);
    }

    tcg_temp_free_i64(sign);
}

static inline void mds_tcg_imm64_to_bigint_s(MDSTCGBigInt *ret, uint64_t v, size_t ret_size)
{
    size_t i;
    int64_t sign;

    mds_tcg_bigint_resize(ret, ret_size);

    tcg_gen_movi_i64(ret->val[0], v);

    sign = ((int64_t) v) >> 63;

    for (i = 1; i < ret->size; i++) {
        tcg_gen_movi_i64(ret->val[i], sign);
    }
}

typedef void (*BinaryTCGOp)(TCGv_i64 ret, TCGv_i64 a0, TCGv_i64 a1);
typedef void (*BinaryTCGOpI)(TCGv_i64 ret, TCGv_i64 a0, int64_t a1);

static inline void mds_tcg_bigint_assert_same_size_2(MDSTCGBigInt *a0,
                                                     MDSTCGBigInt *a1)
{
    g_assert(a0->size == a1->size);
}

static inline void mds_tcg_bigint_assert_same_size_3(MDSTCGBigInt *a0,
                                                     MDSTCGBigInt *a1,
                                                     MDSTCGBigInt *a2)
{
    g_assert(a0->size == a1->size);
    g_assert(a0->size == a2->size);
}

static inline void mds_tcg_gen_mov_bigint(MDSTCGBigInt *dst, MDSTCGBigInt *src)
{
    TCGv_i64 d, s;

    mds_tcg_bigint_assert_same_size_2(dst, src);

    MDS_BIGINT_FOREACH_2(dst, d, src, s) {
        tcg_gen_mov_i64(d, s);
    }
}

static inline void mds_tcg_gen_bigint_bitwise_op(BinaryTCGOp op, MDSTCGBigInt *ret,
                                                 MDSTCGBigInt *a0, MDSTCGBigInt *a1)
{
    TCGv_i64 dst, src0, src1;

    mds_tcg_bigint_assert_same_size_3(ret, a0, a1);

    MDS_BIGINT_FOREACH_3(ret, dst, a0, src0, a1, src1) {
        op(dst, src0, src1);
    }
}

static inline void mds_tcg_gen_bigint_bitwise_op_i(BinaryTCGOpI op, MDSTCGBigInt *ret,
                                                   MDSTCGBigInt *a0, int64_t a1)
{
    TCGv_i64 dst, src0;

    mds_tcg_bigint_assert_same_size_2(ret, a0);

    MDS_BIGINT_FOREACH_2(ret, dst, a0, src0) {
        size_t i = mds_tcg_bigint_get_cur_idx(ret);

        if (!i) {
            op(dst, src0, a1);
        } else {
            op(dst, src0, 0);
        }
    }
}

static inline void mds_tcg_gen_add_bigint(MDSTCGBigInt *ret,
                                          MDSTCGBigInt *a0, MDSTCGBigInt *a1)
{
    TCGv_i64 dst, src0, src1, carry;
    MDSTCGBigInt tmp;

    mds_tcg_bigint_assert_same_size_3(ret, a0, a1);

    mds_tcg_bigint_new(&tmp, mds_tcg_bigint_get_size(ret));
    carry = tcg_const_i64(0);

    MDS_BIGINT_FOREACH_3(&tmp, dst, a0, src0, a1, src1) {
        tcg_gen_add_i64(dst, src0, src1);
        tcg_gen_add_i64(dst, dst, carry);
        tcg_gen_setcond_i64(TCG_COND_LTU, carry, dst, src0);
    }

    tcg_temp_free_i64(carry);

    mds_tcg_gen_mov_bigint(ret, &tmp);
    mds_tcg_bigint_free(&tmp);
}

static inline void mds_tcg_gen_addi_bigint(MDSTCGBigInt *ret,
                                           MDSTCGBigInt *a0, uint64_t a1)
{
    mds_tcg_bigint_assert_same_size_2(ret, a0);

    mds_tcg_imm64_to_bigint_u(ret, a1, mds_tcg_bigint_get_size(ret));
    mds_tcg_gen_add_bigint(ret, ret, a0);
}

static inline void mds_tcg_gen_sub_bigint(MDSTCGBigInt *ret,
                                          MDSTCGBigInt *a0, MDSTCGBigInt *a1)
{
    TCGv_i64 dst, src0, src1, carry;
    MDSTCGBigInt tmp;

    mds_tcg_bigint_assert_same_size_3(ret, a0, a1);

    mds_tcg_bigint_new(&tmp, mds_tcg_bigint_get_size(ret));
    carry = tcg_const_i64(0);

    MDS_BIGINT_FOREACH_3(&tmp, dst, a0, src0, a1, src1) {
        tcg_gen_sub_i64(dst, src0, src1);
        tcg_gen_sub_i64(dst, dst, carry);
        tcg_gen_setcond_i64(TCG_COND_LTU, carry, src0, src1);
    }

    tcg_temp_free_i64(carry);

    mds_tcg_gen_mov_bigint(ret, &tmp);
    mds_tcg_bigint_free(&tmp);
}

static inline void mds_tcg_gen_subfi_bigint(MDSTCGBigInt *ret,
                                            uint64_t a0, MDSTCGBigInt *a1)
{
    mds_tcg_bigint_assert_same_size_2(ret, a1);

    mds_tcg_imm64_to_bigint_u(ret, a0, mds_tcg_bigint_get_size(ret));
    mds_tcg_gen_sub_bigint(ret, ret, a1);
}

/*
 * XXX We only support 64 * 64 -> 128 bits multiplications
 */
static inline void mds_tcg_gen_mulu_bigint(MDSTCGBigInt *ret,
                                           MDSTCGBigInt *a0, MDSTCGBigInt *a1)
{
    mds_tcg_bigint_assert_same_size_3(ret, a0, a1);

    if (mds_tcg_bigint_get_size(ret) == 128) {
        tcg_gen_mulu2_i64(ret->val[0], ret->val[1], a0->val[0], a1->val[0]);
    } else {
        /* Not implemented */
        g_assert_not_reached();
    }
}

/*
 * XXX We only support 64 * 64 -> 128 bits multiplications
 */
static inline void mds_tcg_gen_muls_bigint(MDSTCGBigInt *ret,
                                           MDSTCGBigInt *a0, MDSTCGBigInt *a1)
{
    mds_tcg_bigint_assert_same_size_3(ret, a0, a1);

    if (mds_tcg_bigint_get_size(ret) == 128) {
        TCGv_i64 s0, s1;
        MDSTCGBigInt bs;

        /* invert the first operand if it's negative */
        s0 = tcg_temp_new_i64();
        tcg_gen_sari_i64(s0, a0->val[1], 63);
        tcg_gen_xor_i64(a0->val[0], a0->val[0], s0);
        tcg_gen_shri_i64(s0, s0, 63);
        tcg_gen_add_i64(a0->val[0], a0->val[0], s0);

        /* invert the second operand if it's negative */
        s1 = tcg_temp_new_i64();
        tcg_gen_sari_i64(s1, a1->val[1], 63);
        tcg_gen_xor_i64(a1->val[0], a1->val[0], s1);
        tcg_gen_shri_i64(s1, s1, 63);
        tcg_gen_add_i64(a1->val[0], a1->val[0], s1);

        /* compute the sign of the result */
        tcg_gen_xor_i64(s0, s0, s1);
        tcg_temp_free_i64(s1);

        /* an unsigned multiplication is performed */
        tcg_gen_mulu2_i64(ret->val[0], ret->val[1], a0->val[0], a1->val[0]);

        /* invert the result if it should be negative */
        tcg_gen_shli_i64(s0, s0, 63);
        tcg_gen_sari_i64(s0, s0, 63);

        tcg_gen_xor_i64(ret->val[0], ret->val[0], s0);
        tcg_gen_xor_i64(ret->val[1], ret->val[1], s0);

        tcg_gen_shri_i64(s0, s0, 63);
        mds_tcg_bigint_new(&bs, 128);
        mds_tcg_i64_to_bigint_u(&bs, s0, 128);
        tcg_temp_free(s0);

        mds_tcg_gen_add_bigint(ret, ret, &bs);
        mds_tcg_bigint_free(&bs);
    } else {
        /* Not implemented */
        g_assert_not_reached();
    }
}

/*
 * Step 0: shift by shift_amount / 64
 */
static inline void mds_tcg_gen_shift_left_bigint_step0(MDSTCGBigInt *ret,
                                                       MDSTCGBigInt *src,
                                                       MDSTCGBigInt *shift_amount)
{
    TCGv_i64 dst, tcg_i, shift;

    /* shift = shift_amount / 64 */
    shift = tcg_temp_new_i64();
    tcg_gen_shri_i64(shift, shift_amount->val[0], 6);

    MDS_BIGINT_FOREACH_REVERSE(ret, dst) {
        size_t idx = mds_tcg_bigint_get_cur_idx(ret);
        tcg_gen_movi_i64(dst, 0);

        for (size_t i = 0; i <= idx; i++) {
            /* ret[idx] = (i == shift) ? src[idx - i] : ret[idx] */
            tcg_i = tcg_const_i64(i);
            tcg_gen_movcond_i64(TCG_COND_EQ, dst, shift, tcg_i,
                                src->val[idx - i], dst);
            tcg_temp_free_i64(tcg_i);
        }
    }

    tcg_temp_free_i64(shift);
}

/*
 * Step 1: shift by shift_amount % 64
 */
static inline void mds_tcg_gen_shift_left_bigint_step1(MDSTCGBigInt *ret,
                                                       MDSTCGBigInt *src,
                                                       MDSTCGBigInt *shift_amount)
{
    TCGv_i64 dst, tmp, shift, nshift, mask;

    shift = tcg_temp_new_i64();
    tcg_gen_andi_i64(shift, shift_amount->val[0], 63);

    nshift = tcg_temp_new_i64();
    tcg_gen_subfi_i64(nshift, 64, shift);

    /* To handle the corner case where shift == 0 */
    mask = tcg_temp_new_i64();
    tcg_gen_setcondi_i64(TCG_COND_NE, mask, nshift, 64);
    tcg_gen_shli_i64(mask, mask, 63);
    tcg_gen_sari_i64(mask, mask, 63);

    tmp = tcg_temp_new_i64();

    MDS_BIGINT_FOREACH_REVERSE(ret, dst) {
        size_t idx = mds_tcg_bigint_get_cur_idx(ret);

        tcg_gen_shl_i64(dst, src->val[idx], shift);

        if (idx) {
            tcg_gen_shr_i64(tmp, src->val[idx - 1], nshift);
            tcg_gen_and_i64(tmp, tmp, mask); /* if shift == 0, we ignore this value */
            tcg_gen_or_i64(dst, dst, tmp);
            tcg_gen_discard_i64(tmp);
        }
    }

    tcg_temp_free_i64(tmp);
    tcg_temp_free_i64(mask);
    tcg_temp_free_i64(nshift);
    tcg_temp_free_i64(shift);
}

static inline void mds_tcg_gen_shl_bigint(MDSTCGBigInt *ret,
                                          MDSTCGBigInt *a0, MDSTCGBigInt *shift)
{
    MDSTCGBigInt tmp;

    mds_tcg_bigint_new(&tmp, mds_tcg_bigint_get_size(ret));

    mds_tcg_gen_shift_left_bigint_step0(&tmp, a0, shift);
    mds_tcg_gen_shift_left_bigint_step1(ret, &tmp, shift);

    mds_tcg_bigint_free(&tmp);
}

/*
 * Step 0: shift by shift_amount / 64
 */
static inline void mds_tcg_gen_shift_right_bigint_step0(MDSTCGBigInt *ret,
                                                        MDSTCGBigInt *src,
                                                        MDSTCGBigInt *shift_amount,
                                                        TCGv_i64 sign)
{
    TCGv_i64 dst, tcg_i, shift;

    /* shift = shift_amount / 64 */
    shift = tcg_temp_new_i64();
    tcg_gen_shri_i64(shift, shift_amount->val[0], 6);

    MDS_BIGINT_FOREACH(ret, dst) {
        size_t idx = mds_tcg_bigint_get_cur_idx(ret);
        tcg_gen_mov_i64(dst, sign);

        for (size_t i = 0; i < ret->size - idx; i++) {
            /* ret[idx] = (i == shift) ? src[idx + i] : ret[idx] */
            tcg_i = tcg_const_i64(i);
            tcg_gen_movcond_i64(TCG_COND_EQ, dst, shift, tcg_i,
                                src->val[idx + i], dst);
            tcg_temp_free_i64(tcg_i);
        }
    }

    tcg_temp_free_i64(shift);
}

/*
 * Step 1: shift by shift_amount % 64
 */
static inline void mds_tcg_gen_shift_right_bigint_step1(MDSTCGBigInt *ret,
                                                        MDSTCGBigInt *src,
                                                        MDSTCGBigInt *shift_amount,
                                                        bool arithmetic)
{
    TCGv_i64 dst, tmp, shift, nshift, mask;

    shift = tcg_temp_new_i64();
    tcg_gen_andi_i64(shift, shift_amount->val[0], 63);

    nshift = tcg_temp_new_i64();
    tcg_gen_subfi_i64(nshift, 64, shift);

    /* To handle the corner case where shift == 0 */
    mask = tcg_temp_new_i64();
    tcg_gen_setcondi_i64(TCG_COND_NE, mask, nshift, 64);
    tcg_gen_shli_i64(mask, mask, 63);
    tcg_gen_sari_i64(mask, mask, 63);

    tmp = tcg_temp_new_i64();

    MDS_BIGINT_FOREACH(ret, dst) {
        size_t idx = mds_tcg_bigint_get_cur_idx(ret);

        if (arithmetic && (idx == ret->size - 1)) {
            tcg_gen_sar_i64(dst, src->val[idx], shift);
        } else {
            tcg_gen_shr_i64(dst, src->val[idx], shift);
        }

        if (idx < ret->size - 1) {
            tcg_gen_shl_i64(tmp, src->val[idx + 1], nshift);
            tcg_gen_and_i64(tmp, tmp, mask); /* if shift == 0, we ignore this value */
            tcg_gen_or_i64(dst, dst, tmp);
            tcg_gen_discard_i64(tmp);
        }
    }

    tcg_temp_free_i64(tmp);
    tcg_temp_free_i64(mask);
    tcg_temp_free_i64(nshift);
    tcg_temp_free_i64(shift);
}

static inline void mds_tcg_gen_shr_bigint(MDSTCGBigInt *ret,
                                          MDSTCGBigInt *a0, MDSTCGBigInt *shift)
{
    MDSTCGBigInt tmp;
    TCGv_i64 zero;

    mds_tcg_bigint_new(&tmp, mds_tcg_bigint_get_size(ret));

    zero = tcg_const_i64(0);
    mds_tcg_gen_shift_right_bigint_step0(&tmp, a0, shift, zero);
    tcg_temp_free_i64(zero);

    mds_tcg_gen_shift_right_bigint_step1(ret, &tmp, shift, false);

    mds_tcg_bigint_free(&tmp);
}

static inline void mds_tcg_gen_sar_bigint(MDSTCGBigInt *ret,
                                          MDSTCGBigInt *a0, MDSTCGBigInt *shift)
{
    MDSTCGBigInt tmp;
    TCGv_i64 sign;

    mds_tcg_bigint_new(&tmp, mds_tcg_bigint_get_size(ret));

    sign = tcg_temp_new_i64();
    tcg_gen_sari_i64(sign, a0->val[a0->size - 1], 63);
    mds_tcg_gen_shift_right_bigint_step0(&tmp, a0, shift, sign);
    tcg_temp_free_i64(sign);

    mds_tcg_gen_shift_right_bigint_step1(ret, &tmp, shift, true);

    mds_tcg_bigint_free(&tmp);
}

static inline void mds_tcg_gen_shli_bigint(MDSTCGBigInt *ret,
                                           MDSTCGBigInt *a0, int64_t shift)
{
    TCGv_i64 dst, tmp;

    int64_t shift0 = shift / 64;

    int64_t shift1 = shift % 64;
    int64_t nshift1 = 64 - shift1;

    g_assert(shift0 < ret->size);

    /* Step 0 */
    MDS_BIGINT_FOREACH_REVERSE(ret, dst) {
        ssize_t i = mds_tcg_bigint_get_cur_idx(ret) - shift0;

        if (i >= 0) {
            tcg_gen_mov_i64(dst, a0->val[i]);
        } else {
            tcg_gen_movi_i64(dst, 0);
        }
    }

    if (!shift1) {
        return;
    }

    /* Step 1 */
    tmp = tcg_temp_new_i64();

    MDS_BIGINT_FOREACH_REVERSE(ret, dst) {
        size_t i = mds_tcg_bigint_get_cur_idx(ret);
        tcg_gen_shli_i64(dst, dst, shift1);

        if (i) {
            tcg_gen_shri_i64(tmp, a0->val[i-1], nshift1);
            tcg_gen_or_i64(dst, dst, tmp);
            tcg_gen_discard_i64(tmp);
        }
    }

    tcg_temp_free_i64(tmp);
}

static inline void mds_tcg_gen_shri_bigint(MDSTCGBigInt *ret,
                                           MDSTCGBigInt *a0, int64_t shift)
{
    TCGv_i64 dst, tmp;

    int64_t shift0 = shift / 64;

    int64_t shift1 = shift % 64;
    int64_t nshift1 = 64 - shift1;

    g_assert(shift0 < ret->size);

    /* Step 0 */
    MDS_BIGINT_FOREACH(ret, dst) {
        size_t i = mds_tcg_bigint_get_cur_idx(ret) + shift0;

        if (i < a0->size) {
            tcg_gen_mov_i64(dst, a0->val[i]);
        } else {
            tcg_gen_movi_i64(dst, 0);
        }
    }

    if (!shift1) {
        return;
    }

    /* Step 1 */
    tmp = tcg_temp_new_i64();

    MDS_BIGINT_FOREACH(ret, dst) {
        size_t i = mds_tcg_bigint_get_cur_idx(ret);
        tcg_gen_shri_i64(dst, dst, shift1);

        if (i < ret->size - 1) {
            tcg_gen_shli_i64(tmp, a0->val[i + 1], nshift1);
            tcg_gen_or_i64(dst, dst, tmp);
            tcg_gen_discard_i64(tmp);
        }
    }

    tcg_temp_free_i64(tmp);
}

static inline void mds_tcg_gen_sari_bigint(MDSTCGBigInt *ret,
                                           MDSTCGBigInt *a0, int64_t shift)
{
    TCGv_i64 dst, sign, tmp;

    int64_t shift0 = shift / 64;

    int64_t shift1 = shift % 64;
    int64_t nshift1 = 64 - shift1;

    g_assert(shift0 < ret->size);

    sign = tcg_temp_new_i64();
    tcg_gen_sari_i64(sign, a0->val[a0->size - 1], 63);

    MDS_BIGINT_FOREACH(ret, dst) {
        size_t i = mds_tcg_bigint_get_cur_idx(ret) + shift0;

        if (i < a0->size) {
            tcg_gen_mov_i64(dst, a0->val[i]);
        } else {
            tcg_gen_mov_i64(dst, sign);
        }
    }
    tcg_temp_free_i64(sign);

    if (!shift1) {
        return;
    }

    /* Step 1 */
    tmp = tcg_temp_new_i64();

    MDS_BIGINT_FOREACH(ret, dst) {
        size_t i = mds_tcg_bigint_get_cur_idx(ret);

        if (i == ret->size - 1) {
            tcg_gen_sari_i64(dst, dst, shift1);
        } else {
            tcg_gen_shri_i64(dst, dst, shift1);
        }

        if (i < ret->size - 1) {
            tcg_gen_shli_i64(tmp, a0->val[i + 1], nshift1);
            tcg_gen_or_i64(dst, dst, tmp);
            tcg_gen_discard_i64(tmp);
        }
    }

    tcg_temp_free_i64(tmp);
}

#endif
