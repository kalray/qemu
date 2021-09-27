/*
 * kv3/decode.c
  *  (c) Copyright 2010-2018 Kalray SA.
 * Automatically generated from the Machine Description System (MDS).
 */
#include "decode.h"
#include "translate-insn.h"
#include "instructions.h"
static void decode_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerBo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerCe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_byteshift = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerN_kv3_registerBe(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerN_kv3_registerBe *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerBe = decoded;
    }
}

static void decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_rounding = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 15) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerBo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerCe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_byteshift = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerU = decoded;
    }
}

static void decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerE(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerE *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerE = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerA1_kv3_registerBp_kv3_registerC_kv3_registerD(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA1_kv3_registerBp_kv3_registerC_kv3_registerD *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 2;
        decoded += 1;
        a->kv3_registerA1 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerBp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerC = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerD = decoded;
    }
}

static void decode_kv3_signed10_kv3_registerZ_kv3_registerT(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_signed10_kv3_registerZ_kv3_registerT *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerT = decoded;
    }
}

static void decode_kv3_registerA2_kv3_registerBp_kv3_registerC_kv3_registerD(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA2_kv3_registerBp_kv3_registerC_kv3_registerD *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 2;
        decoded += 2;
        a->kv3_registerA2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerBp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerC = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerD = decoded;
    }
}

static void decode_kv3_speculate_kv3_registerA_kv3_signed10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_speculate_kv3_registerA_kv3_signed10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_roundint_kv3_saturate_kv3_registerAz_kv3_registerBq(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_roundint_kv3_saturate_kv3_registerAz_kv3_registerBq *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_roundint = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_saturate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 2;
        decoded += 2;
        a->kv3_registerAz = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
}

static void decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerV(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerV *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerV = decoded;
    }
}

static void decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerV(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerV *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerV = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_extend27_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_extend27_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerT = decoded;
    }
}

static void decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerU(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerU *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerU = decoded;
    }
}

static void decode_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerBe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerCo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_rounding = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 15) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_unsigned6 = decoded;
    }
}

static void decode_kv3_systemT2_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_systemT2_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 9) - 1);
        decoded = field;
        a->kv3_systemT2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerN_kv3_registerBo(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerN_kv3_registerBo *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerBo = decoded;
    }
}

static void decode_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerBe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerCo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_byteshift = decoded;
    }
}

static void decode_kv3_registerZ_kv3_systemS2(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerZ_kv3_systemS2 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 9) - 1);
        decoded = field;
        a->kv3_systemS2 = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_extend27_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_extend27_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_systemT3_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_systemT3_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 9) - 1);
        decoded = field;
        a->kv3_systemT3 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_roundint_kv3_saturate_kv3_registerAx_kv3_registerBq(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_roundint_kv3_saturate_kv3_registerAx_kv3_registerBq *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_roundint = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_saturate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 2;
        a->kv3_registerAx = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
}

static void decode_kv3_variant_kv3_registerM_kv3_signed10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerM_kv3_signed10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerA_kv3_registerBo(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA_kv3_registerBo *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerBo = decoded;
    }
}

static void decode_kv3_signed10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_signed10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerE(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerE *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerE = decoded;
    }
}

static void decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
}

static void decode_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_variant_kv3_registerN_kv3_extend27_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerN_kv3_extend27_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerW_kv3_extend27_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_extend27_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_roundint_kv3_saturate_kv3_registerAy_kv3_registerBq(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_roundint_kv3_saturate_kv3_registerAy_kv3_registerBq *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_roundint = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_saturate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 2;
        decoded += 1;
        a->kv3_registerAy = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
}

static void decode_kv3_speculate_kv3_scaling_kv3_registerA_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_speculate_kv3_scaling_kv3_registerA_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_column_kv3_speculate_kv3_registerAq_kv3_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_column_kv3_speculate_kv3_registerAq_kv3_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_column = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_comparison = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
}

static void decode_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_column = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerA3_kv3_registerBp_kv3_registerC_kv3_registerD(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA3_kv3_registerBp_kv3_registerC_kv3_registerD *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 2;
        decoded += 3;
        a->kv3_registerA3 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerBp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerC = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerD = decoded;
    }
}

static void decode_kv3_signed10_kv3_registerZ_kv3_registerU(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_signed10_kv3_registerZ_kv3_registerU *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerU = decoded;
    }
}

static void decode_kv3_variant_kv3_registerN_kv3_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerN_kv3_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerU = decoded;
    }
}

static void decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_roundint_kv3_saturate_kv3_registerAt_kv3_registerBq(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_roundint_kv3_saturate_kv3_registerAt_kv3_registerBq *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_roundint = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_saturate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 2;
        decoded += 3;
        a->kv3_registerAt = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
}

static void decode_kv3_silent2_kv3_registerW_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_silent2_kv3_registerW_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 11) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_variant_kv3_registerM_kv3_extend27_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerM_kv3_extend27_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_floatcomp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_registerAh_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerAh_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerAh = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerZ_kv3_systemS4(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerZ_kv3_systemS4 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 9) - 1);
        decoded = field;
        a->kv3_systemS4 = decoded;
    }
}

static void decode_kv3_registerAl_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerAl_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerAl = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerC = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerD = decoded;
    }
}

static void decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_comparison = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 5;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_upper27_lower5 = decoded;
    }
}

static void decode_kv3_column_kv3_speculate_kv3_scaling_kv3_registerAq_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_column_kv3_speculate_kv3_scaling_kv3_registerAq_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_column = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
}

static void decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_floatcomp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 5;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_upper27_lower5 = decoded;
    }
}

static void decode_kv3_registerA_kv3_registerBe(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA_kv3_registerBe *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerBe = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerM_kv3_registerP_kv3_registerO(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerM_kv3_registerP_kv3_registerO *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 1) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerP = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerO = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerT = decoded;
    }
}

static void decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_signed10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_signed10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_scalarcond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
}

static void decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_scalarcond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
}

static void decode_kv3_xrounding_kv3_silent2_kv3_registerA_kv3_registerBp(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_xrounding_kv3_silent2_kv3_registerA_kv3_registerBp *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_xrounding = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 11) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerBp = decoded;
    }
}

static void decode_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerBo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerCe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_roundint_kv3_saturate_kv3_registerAh_kv3_registerBq(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_roundint_kv3_saturate_kv3_registerAh_kv3_registerBq *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_roundint = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_saturate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerAh = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        field <<= 4;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_stopbit2_stopbit4 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_startbit = decoded;
    }
}

static void decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_extend27_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_extend27_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_scalarcond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ_kv3_signed10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ_kv3_signed10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_silent2_kv3_registerW_kv3_registerP(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_silent2_kv3_registerW_kv3_registerP *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 11) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 1) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerP = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ_kv3_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
}

static void decode_kv3_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerAp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerBp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerC = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerD = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerV(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerV *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerV = decoded;
    }
}

static void decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerT = decoded;
    }
}

static void decode_kv3_roundint_kv3_saturate_kv3_registerAl_kv3_registerBq(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_roundint_kv3_saturate_kv3_registerAl_kv3_registerBq *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_roundint = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_saturate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerAl = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
}

static void decode_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerBe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerCo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_byteshift = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_speculate_kv3_registerA_kv3_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_speculate_kv3_registerA_kv3_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerE(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerE *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerE = decoded;
    }
}

static void decode_kv3_registerW_kv3_signed10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_signed10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerE(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerE *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerE = decoded;
    }
}

static void decode_kv3_extend27_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_extend27_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerM_kv3_registerZ_kv3_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerE(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerE *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerE = decoded;
    }
}

static void decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerT = decoded;
    }
}

static void decode_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerAq_kv3_registerBq(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerAq_kv3_registerBq *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 14) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerBq = decoded;
    }
}

static void decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerU = decoded;
    }
}

static void decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerV(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerV *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerV = decoded;
    }
}

static void decode_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerT(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerT *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerT = decoded;
    }
}

static void decode_kv3_registerM_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerM_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerV(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerV *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerV = decoded;
    }
}

static void decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_comparison = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
}

static void decode_kv3_registerZ_kv3_pcrel17(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerZ_kv3_pcrel17 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 17) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 17;
        decoded >>= sizeof(decoded)*8 - 17;
        decoded <<= 2;
        a->kv3_pcrel17 = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_scalarcond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerU(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerU *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerU = decoded;
    }
}

static void decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 11) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_splat32 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 5;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_upper27_lower5 = decoded;
    }
}

static void decode_kv3_variant_kv3_registerN_kv3_signed10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerN_kv3_signed10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerU = decoded;
    }
}

static void decode_kv3_variant_kv3_scaling_kv3_registerM_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_scaling_kv3_registerM_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_simdcond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_registerN_kv3_registerQ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerN_kv3_registerQ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 2) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerQ = decoded;
    }
}

static void decode_kv3_sysnumber(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_sysnumber *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 12) - 1);
        decoded = field;
        a->kv3_sysnumber = decoded;
    }
}

static void decode_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_extend27_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_extend27_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_column = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_extend27_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_extend27_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_xrounding_kv3_silent2_kv3_rectify_kv3_registerA_kv3_registerB(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_xrounding_kv3_silent2_kv3_rectify_kv3_registerA_kv3_registerB *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_xrounding = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 11) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_rectify = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerB = decoded;
    }
}

static void decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_comparison = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_variant_kv3_registerM_kv3_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_registerM_kv3_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 37;
        decoded >>= sizeof(decoded)*8 - 37;
        a->kv3_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        a->kv3_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_column = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 5;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_upper27_lower5 = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ_kv3_unsigned6 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_unsigned6 = decoded;
    }
}

static void decode_kv3_pcrel27(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_pcrel27 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 27;
        decoded >>= sizeof(decoded)*8 - 27;
        decoded <<= 2;
        a->kv3_pcrel27 = decoded;
    }
}

static void decode_kv3_registerM_kv3_registerZ_kv3_signed10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerM_kv3_registerZ_kv3_signed10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerV(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerV *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerV = decoded;
    }
}

static void decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 54;
        decoded >>= sizeof(decoded)*8 - 54;
        a->kv3_extend27_offset27 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerA0_kv3_registerBp_kv3_registerC_kv3_registerD(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA0_kv3_registerBp_kv3_registerC_kv3_registerD *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 2;
        a->kv3_registerA0 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerBp = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerC = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerD = decoded;
    }
}

static void decode_kv3_registerW_kv3_extend6_upper27_lower10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_extend6_upper27_lower10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 43;
        decoded >>= sizeof(decoded)*8 - 43;
        a->kv3_extend6_upper27_lower10 = decoded;
    }
}

static void decode_kv3_column_kv3_speculate_kv3_registerAq_kv3_signed10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_column_kv3_speculate_kv3_registerAq_kv3_signed10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_column = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerW_kv3_signed16(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_signed16 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 16) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 16;
        decoded >>= sizeof(decoded)*8 - 16;
        a->kv3_signed16 = decoded;
    }
}

static void decode_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerBe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerCo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerE(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerE *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerE = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_signed10(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_signed10 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_comparison = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
}

static void decode_kv3_signed10_kv3_registerZ_kv3_registerE(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_signed10_kv3_registerZ_kv3_registerE *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerE = decoded;
    }
}

static void decode_kv3_signed10_kv3_registerZ_kv3_registerV(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_signed10_kv3_registerZ_kv3_registerV *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 10;
        decoded >>= sizeof(decoded)*8 - 10;
        a->kv3_signed10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerV = decoded;
    }
}

static void decode_kv3_column_kv3_speculate_kv3_registerAq_kv3_extend27_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_column_kv3_speculate_kv3_registerAq_kv3_extend27_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_column = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerAq = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_branchcond_kv3_registerZ_kv3_pcrel17(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_branchcond_kv3_registerZ_kv3_pcrel17 *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 23) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_branchcond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        int64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 17) - 1);
        decoded = field;
        decoded <<= sizeof(decoded)*8 - 17;
        decoded >>= sizeof(decoded)*8 - 17;
        decoded <<= 2;
        a->kv3_pcrel17 = decoded;
    }
}

static void decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_rounding2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 11) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_speculate_kv3_registerA_kv3_extend27_upper27_lower10_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_speculate_kv3_registerA_kv3_extend27_upper27_lower10_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_speculate = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[2] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 27;
        field |= (WORDS[1] >> 0) & (((uint32_t)1 << 27) - 1);
        field <<= 10;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 10) - 1);
        decoded = field;
        a->kv3_extend27_upper27_lower10 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_rounding = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 15) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 1) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerP = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerO = decoded;
    }
}

static void decode_kv3_variant_kv3_scaling_kv3_registerN_kv3_registerY_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_variant_kv3_scaling_kv3_registerN_kv3_registerY_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 24) & (((uint32_t)1 << 2) - 1);
        decoded = field;
        a->kv3_variant = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_scaling = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 20) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_registerN = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerA = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 13) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        decoded += 1;
        a->kv3_registerBo = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 7) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        decoded <<= 1;
        a->kv3_registerCe = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 4) - 1);
        decoded = field;
        a->kv3_lsucond = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerT = decoded;
    }
}

static void decode_kv3_registerW_kv3_registerZ(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_registerW_kv3_registerZ *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
}

static void decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 12) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_rounding = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 15) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 19) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerM = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 0) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerZ = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 6) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerY = decoded;
    }
}

static void decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP(DisasContext *ctx, const uint32_t *codeWords, struct ops_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP *a)
{
        const uint32_t *restrict WORDS = codeWords;
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 8) & (((uint32_t)1 << 3) - 1);
        decoded = field;
        a->kv3_rounding2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 11) & (((uint32_t)1 << 1) - 1);
        decoded = field;
        a->kv3_silent2 = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 18) & (((uint32_t)1 << 6) - 1);
        decoded = field;
        a->kv3_registerW = decoded;
    }
    {
        uint64_t field = 0;
        uint64_t decoded = 0;
        field |= (WORDS[0] >> 1) & (((uint32_t)1 << 5) - 1);
        decoded = field;
        a->kv3_registerP = decoded;
    }
}

union decode_formats {
    struct ops_empty empty;
    struct ops_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ;
    struct ops_kv3_registerN_kv3_registerBe kv3_registerN_kv3_registerBe;
    struct ops_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY;
    struct ops_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift;
    struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU;
    struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerE kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerE;
    struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ;
    struct ops_kv3_registerA1_kv3_registerBp_kv3_registerC_kv3_registerD kv3_registerA1_kv3_registerBp_kv3_registerC_kv3_registerD;
    struct ops_kv3_signed10_kv3_registerZ_kv3_registerT kv3_signed10_kv3_registerZ_kv3_registerT;
    struct ops_kv3_registerA2_kv3_registerBp_kv3_registerC_kv3_registerD kv3_registerA2_kv3_registerBp_kv3_registerC_kv3_registerD;
    struct ops_kv3_speculate_kv3_registerA_kv3_signed10_kv3_registerZ kv3_speculate_kv3_registerA_kv3_signed10_kv3_registerZ;
    struct ops_kv3_roundint_kv3_saturate_kv3_registerAz_kv3_registerBq kv3_roundint_kv3_saturate_kv3_registerAz_kv3_registerBq;
    struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerV kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerV;
    struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerV kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerV;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_extend27_offset27_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_extend27_offset27_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT;
    struct ops_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ;
    struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerU kv3_upper27_lower10_kv3_registerZ_kv3_registerU;
    struct ops_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_registerZ kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_registerZ;
    struct ops_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6 kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6;
    struct ops_kv3_systemT2_kv3_registerZ kv3_systemT2_kv3_registerZ;
    struct ops_kv3_registerN_kv3_registerBo kv3_registerN_kv3_registerBo;
    struct ops_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift;
    struct ops_kv3_registerZ_kv3_systemS2 kv3_registerZ_kv3_systemS2;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_extend27_offset27_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_extend27_offset27_kv3_registerZ;
    struct ops_kv3_systemT3_kv3_registerZ kv3_systemT3_kv3_registerZ;
    struct ops_kv3_roundint_kv3_saturate_kv3_registerAx_kv3_registerBq kv3_roundint_kv3_saturate_kv3_registerAx_kv3_registerBq;
    struct ops_kv3_variant_kv3_registerM_kv3_signed10_kv3_registerZ kv3_variant_kv3_registerM_kv3_signed10_kv3_registerZ;
    struct ops_kv3_registerA_kv3_registerBo kv3_registerA_kv3_registerBo;
    struct ops_kv3_signed10_kv3_registerZ kv3_signed10_kv3_registerZ;
    struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerE kv3_upper27_lower10_kv3_registerZ_kv3_registerE;
    struct ops_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10 kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10;
    struct ops_kv3_registerW_kv3_upper27_lower10_kv3_registerZ kv3_registerW_kv3_upper27_lower10_kv3_registerZ;
    struct ops_kv3_variant_kv3_registerN_kv3_extend27_upper27_lower10_kv3_registerZ kv3_variant_kv3_registerN_kv3_extend27_upper27_lower10_kv3_registerZ;
    struct ops_kv3_registerW_kv3_extend27_upper27_lower10 kv3_registerW_kv3_extend27_upper27_lower10;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ;
    struct ops_kv3_roundint_kv3_saturate_kv3_registerAy_kv3_registerBq kv3_roundint_kv3_saturate_kv3_registerAy_kv3_registerBq;
    struct ops_kv3_speculate_kv3_scaling_kv3_registerA_kv3_registerY_kv3_registerZ kv3_speculate_kv3_scaling_kv3_registerA_kv3_registerY_kv3_registerZ;
    struct ops_kv3_column_kv3_speculate_kv3_registerAq_kv3_upper27_lower10_kv3_registerZ kv3_column_kv3_speculate_kv3_registerAq_kv3_upper27_lower10_kv3_registerZ;
    struct ops_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_offset27_kv3_registerZ kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_offset27_kv3_registerZ;
    struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower10 kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower10;
    struct ops_kv3_systemT4_kv3_registerZ kv3_systemT4_kv3_registerZ;
    struct ops_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_offset27_kv3_registerZ kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_offset27_kv3_registerZ;
    struct ops_kv3_registerA3_kv3_registerBp_kv3_registerC_kv3_registerD kv3_registerA3_kv3_registerBp_kv3_registerC_kv3_registerD;
    struct ops_kv3_signed10_kv3_registerZ_kv3_registerU kv3_signed10_kv3_registerZ_kv3_registerU;
    struct ops_kv3_variant_kv3_registerN_kv3_upper27_lower10_kv3_registerZ kv3_variant_kv3_registerN_kv3_upper27_lower10_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU;
    struct ops_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ;
    struct ops_kv3_roundint_kv3_saturate_kv3_registerAt_kv3_registerBq kv3_roundint_kv3_saturate_kv3_registerAt_kv3_registerBq;
    struct ops_kv3_silent2_kv3_registerW_kv3_registerZ kv3_silent2_kv3_registerW_kv3_registerZ;
    struct ops_kv3_variant_kv3_registerM_kv3_extend27_upper27_lower10_kv3_registerZ kv3_variant_kv3_registerM_kv3_extend27_upper27_lower10_kv3_registerZ;
    struct ops_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY;
    struct ops_kv3_registerAh_kv3_registerZ_kv3_registerY kv3_registerAh_kv3_registerZ_kv3_registerY;
    struct ops_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ;
    struct ops_kv3_registerZ_kv3_systemS4 kv3_registerZ_kv3_systemS4;
    struct ops_kv3_registerAl_kv3_registerZ_kv3_registerY kv3_registerAl_kv3_registerZ_kv3_registerY;
    struct ops_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD;
    struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower5;
    struct ops_kv3_column_kv3_speculate_kv3_scaling_kv3_registerAq_kv3_registerY_kv3_registerZ kv3_column_kv3_speculate_kv3_scaling_kv3_registerAq_kv3_registerY_kv3_registerZ;
    struct ops_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ;
    struct ops_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10 kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10;
    struct ops_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5;
    struct ops_kv3_registerA_kv3_registerBe kv3_registerA_kv3_registerBe;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_registerZ;
    struct ops_kv3_registerM_kv3_registerP_kv3_registerO kv3_registerM_kv3_registerP_kv3_registerO;
    struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT;
    struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_signed10 kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_signed10;
    struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_upper27_lower10 kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_upper27_lower10;
    struct ops_kv3_xrounding_kv3_silent2_kv3_registerA_kv3_registerBp kv3_xrounding_kv3_silent2_kv3_registerA_kv3_registerBp;
    struct ops_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_registerZ kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_registerZ;
    struct ops_kv3_roundint_kv3_saturate_kv3_registerAh_kv3_registerBq kv3_roundint_kv3_saturate_kv3_registerAh_kv3_registerBq;
    struct ops_kv3_systemRA_kv3_registerZ kv3_systemRA_kv3_registerZ;
    struct ops_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit;
    struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_extend27_upper27_lower10 kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_extend27_upper27_lower10;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_offset27_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_offset27_kv3_registerZ;
    struct ops_kv3_registerW_kv3_registerZ_kv3_signed10 kv3_registerW_kv3_registerZ_kv3_signed10;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_registerZ;
    struct ops_kv3_silent2_kv3_registerW_kv3_registerP kv3_silent2_kv3_registerW_kv3_registerP;
    struct ops_kv3_registerW_kv3_registerZ_kv3_upper27_lower10 kv3_registerW_kv3_registerZ_kv3_upper27_lower10;
    struct ops_kv3_upper27_lower10_kv3_registerZ kv3_upper27_lower10_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ;
    struct ops_kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerV kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerV;
    struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT;
    struct ops_kv3_roundint_kv3_saturate_kv3_registerAl_kv3_registerBq kv3_roundint_kv3_saturate_kv3_registerAl_kv3_registerBq;
    struct ops_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ;
    struct ops_kv3_speculate_kv3_registerA_kv3_upper27_lower10_kv3_registerZ kv3_speculate_kv3_registerA_kv3_upper27_lower10_kv3_registerZ;
    struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerE kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerE;
    struct ops_kv3_registerW_kv3_signed10_kv3_registerZ kv3_registerW_kv3_signed10_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerE kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerE;
    struct ops_kv3_extend27_upper27_lower10_kv3_registerZ kv3_extend27_upper27_lower10_kv3_registerZ;
    struct ops_kv3_registerM_kv3_registerZ_kv3_upper27_lower10 kv3_registerM_kv3_registerZ_kv3_upper27_lower10;
    struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerE kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerE;
    struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT;
    struct ops_kv3_registerY_kv3_registerZ kv3_registerY_kv3_registerZ;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_offset27_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_offset27_kv3_registerZ;
    struct ops_kv3_registerAq_kv3_registerBq kv3_registerAq_kv3_registerBq;
    struct ops_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU;
    struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerV kv3_upper27_lower10_kv3_registerZ_kv3_registerV;
    struct ops_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_registerZ kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_registerZ;
    struct ops_kv3_upper27_lower10_kv3_registerZ_kv3_registerT kv3_upper27_lower10_kv3_registerZ_kv3_registerT;
    struct ops_kv3_registerM_kv3_registerZ_kv3_registerY kv3_registerM_kv3_registerZ_kv3_registerY;
    struct ops_kv3_registerZ_kv3_systemS3 kv3_registerZ_kv3_systemS3;
    struct ops_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerV kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerV;
    struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10 kv3_comparison_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10;
    struct ops_kv3_registerZ_kv3_pcrel17 kv3_registerZ_kv3_pcrel17;
    struct ops_kv3_registerW_kv3_registerZ_kv3_registerY kv3_registerW_kv3_registerZ_kv3_registerY;
    struct ops_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_registerY kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_registerY;
    struct ops_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerU kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerU;
    struct ops_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5;
    struct ops_kv3_variant_kv3_registerN_kv3_signed10_kv3_registerZ kv3_variant_kv3_registerN_kv3_signed10_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU;
    struct ops_kv3_variant_kv3_scaling_kv3_registerM_kv3_registerY_kv3_registerZ kv3_variant_kv3_scaling_kv3_registerM_kv3_registerY_kv3_registerZ;
    struct ops_kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY;
    struct ops_kv3_registerN_kv3_registerQ kv3_registerN_kv3_registerQ;
    struct ops_kv3_sysnumber kv3_sysnumber;
    struct ops_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_extend27_offset27_kv3_registerZ kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_extend27_offset27_kv3_registerZ;
    struct ops_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_extend27_offset27_kv3_registerZ kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_extend27_offset27_kv3_registerZ;
    struct ops_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ;
    struct ops_kv3_xrounding_kv3_silent2_kv3_rectify_kv3_registerA_kv3_registerB kv3_xrounding_kv3_silent2_kv3_rectify_kv3_registerA_kv3_registerB;
    struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY;
    struct ops_kv3_variant_kv3_registerM_kv3_upper27_lower10_kv3_registerZ kv3_variant_kv3_registerM_kv3_upper27_lower10_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ;
    struct ops_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_registerZ kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_registerZ;
    struct ops_kv3_registerW_kv3_registerZ_kv3_upper27_lower5 kv3_registerW_kv3_registerZ_kv3_upper27_lower5;
    struct ops_kv3_registerW_kv3_registerZ_kv3_unsigned6 kv3_registerW_kv3_registerZ_kv3_unsigned6;
    struct ops_kv3_pcrel27 kv3_pcrel27;
    struct ops_kv3_registerM_kv3_registerZ_kv3_signed10 kv3_registerM_kv3_registerZ_kv3_signed10;
    struct ops_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerV kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerV;
    struct ops_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ;
    struct ops_kv3_registerA0_kv3_registerBp_kv3_registerC_kv3_registerD kv3_registerA0_kv3_registerBp_kv3_registerC_kv3_registerD;
    struct ops_kv3_registerW_kv3_extend6_upper27_lower10 kv3_registerW_kv3_extend6_upper27_lower10;
    struct ops_kv3_column_kv3_speculate_kv3_registerAq_kv3_signed10_kv3_registerZ kv3_column_kv3_speculate_kv3_registerAq_kv3_signed10_kv3_registerZ;
    struct ops_kv3_registerW_kv3_signed16 kv3_registerW_kv3_signed16;
    struct ops_kv3_registerZ_kv3_systemAlone kv3_registerZ_kv3_systemAlone;
    struct ops_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_registerZ kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_registerZ;
    struct ops_kv3_registerZ kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerE kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerE;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ kv3_lsucond_kv3_registerY_kv3_registerZ;
    struct ops_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_signed10 kv3_comparison_kv3_registerW_kv3_registerZ_kv3_signed10;
    struct ops_kv3_systemAlone_kv3_registerZ kv3_systemAlone_kv3_registerZ;
    struct ops_kv3_signed10_kv3_registerZ_kv3_registerE kv3_signed10_kv3_registerZ_kv3_registerE;
    struct ops_kv3_signed10_kv3_registerZ_kv3_registerV kv3_signed10_kv3_registerZ_kv3_registerV;
    struct ops_kv3_column_kv3_speculate_kv3_registerAq_kv3_extend27_upper27_lower10_kv3_registerZ kv3_column_kv3_speculate_kv3_registerAq_kv3_extend27_upper27_lower10_kv3_registerZ;
    struct ops_kv3_branchcond_kv3_registerZ_kv3_pcrel17 kv3_branchcond_kv3_registerZ_kv3_pcrel17;
    struct ops_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ;
    struct ops_kv3_speculate_kv3_registerA_kv3_extend27_upper27_lower10_kv3_registerZ kv3_speculate_kv3_registerA_kv3_extend27_upper27_lower10_kv3_registerZ;
    struct ops_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO;
    struct ops_kv3_variant_kv3_scaling_kv3_registerN_kv3_registerY_kv3_registerZ kv3_variant_kv3_scaling_kv3_registerN_kv3_registerY_kv3_registerZ;
    struct ops_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_registerZ kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_registerZ;
    struct ops_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT;
    struct ops_kv3_registerW_kv3_registerZ kv3_registerW_kv3_registerZ;
    struct ops_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY;
    struct ops_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP;
};

bool decode_v1_double(DisasContext *ctx, const uint32_t *buffer)
{
  const uint32_t *codeWords = (const uint32_t *)buffer;
  union decode_formats u;
  uint32_t codeWord_0 = codeWords[0];
  uint32_t codeWord_1 = codeWords[1];
  switch ((codeWord_0 >> 28) & 0x0000000f) {
  case 0x0000000a:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBO_X;
          ctx->cur_opcode->insn = kv3_LBZ;
          return trans_v1_LBZ_variant_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBO_X;
          ctx->cur_opcode->insn = kv3_LBS;
          return trans_v1_LBS_variant_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBO_X;
          ctx->cur_opcode->insn = kv3_LHZ;
          return trans_v1_LHZ_variant_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBO_X;
          ctx->cur_opcode->insn = kv3_LHS;
          return trans_v1_LHS_variant_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 25) & 0x00000007) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_speculate_kv3_registerA_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_speculate_kv3_registerA_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVBO_X;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_speculate_registerA_upper27_lower10_registerZ_double(ctx, &u.kv3_speculate_kv3_registerA_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_column_kv3_speculate_kv3_registerAq_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_column_kv3_speculate_kv3_registerAq_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVSBO_X;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_column_speculate_registerAq_upper27_lower10_registerZ_double(ctx, &u.kv3_column_kv3_speculate_kv3_registerAq_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBO_X;
            ctx->cur_opcode->insn = kv3_SB;
            return trans_v1_SB_upper27_lower10_registerZ_registerT_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
          case 0x00000001:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBO_X;
            ctx->cur_opcode->insn = kv3_SH;
            return trans_v1_SH_upper27_lower10_registerZ_registerT_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBO_X;
            ctx->cur_opcode->insn = kv3_SW;
            return trans_v1_SW_upper27_lower10_registerZ_registerT_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
          case 0x00000001:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBO_X;
            ctx->cur_opcode->insn = kv3_SD;
            return trans_v1_SD_upper27_lower10_registerZ_registerT_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerU);
              ctx->cur_opcode->format = kv3_LSU_SPBO_X;
              ctx->cur_opcode->insn = kv3_SQ;
              return trans_v1_SQ_upper27_lower10_registerZ_registerU_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerU);
            case 0x00000001:
              switch ((codeWord_0 >> 19) & 0x00000001) {
              case 0x00000000:
                decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerV(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerV);
                ctx->cur_opcode->format = kv3_LSU_SQBO_X;
                ctx->cur_opcode->insn = kv3_SO;
                return trans_v1_SO_upper27_lower10_registerZ_registerV_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerV);
              case 0x00000001:
                switch ((codeWord_0 >> 20) & 0x0000000f) {
                case 0x00000000:
                  decode_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FZBO_X;
                  ctx->cur_opcode->insn = kv3_DZEROL;
                  return trans_v1_DZEROL_upper27_lower10_registerZ_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ);
                default:
                  break;
                }
                return false;
                break;
              }
              return false;
              break;
            }
            return false;
          case 0x00000001:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerE(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerE);
            ctx->cur_opcode->format = kv3_LSU_SVBO_X;
            ctx->cur_opcode->insn = kv3_SV;
            return trans_v1_SV_upper27_lower10_registerZ_registerE_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerE);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASBO_X;
            ctx->cur_opcode->insn = kv3_ALCLRW;
            return trans_v1_ALCLRW_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          case 0x00000001:
            decode_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASBO_X;
            ctx->cur_opcode->insn = kv3_ALCLRD;
            return trans_v1_ALCLRD_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AABO_X;
            ctx->cur_opcode->insn = kv3_ALADDW;
            return trans_v1_ALADDW_upper27_lower10_registerZ_registerT_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
          case 0x00000001:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AABO_X;
            ctx->cur_opcode->insn = kv3_ALADDD;
            return trans_v1_ALADDD_upper27_lower10_registerZ_registerT_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB_O;
          ctx->cur_opcode->insn = kv3_LBZ;
          return trans_v1_LBZ_variant_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB_O;
          ctx->cur_opcode->insn = kv3_LBS;
          return trans_v1_LBS_variant_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB_O;
          ctx->cur_opcode->insn = kv3_LHZ;
          return trans_v1_LHZ_variant_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB_O;
          ctx->cur_opcode->insn = kv3_LHS;
          return trans_v1_LHS_variant_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 25) & 0x00000007) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVPB_O;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_speculate_lsucond_registerY_registerA_offset27_registerZ_double(ctx, &u.kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVSPB_O;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_column_speculate_lsucond_registerY_registerAq_offset27_registerZ_double(ctx, &u.kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB_O;
            ctx->cur_opcode->insn = kv3_SB;
            return trans_v1_SB_lsucond_registerY_offset27_registerZ_registerT_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
          case 0x00000001:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB_O;
            ctx->cur_opcode->insn = kv3_SH;
            return trans_v1_SH_lsucond_registerY_offset27_registerZ_registerT_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB_O;
            ctx->cur_opcode->insn = kv3_SW;
            return trans_v1_SW_lsucond_registerY_offset27_registerZ_registerT_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
          case 0x00000001:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB_O;
            ctx->cur_opcode->insn = kv3_SD;
            return trans_v1_SD_lsucond_registerY_offset27_registerZ_registerT_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU);
              ctx->cur_opcode->format = kv3_LSU_SPPB_O;
              ctx->cur_opcode->insn = kv3_SQ;
              return trans_v1_SQ_lsucond_registerY_offset27_registerZ_registerU_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU);
            case 0x00000001:
              switch ((codeWord_0 >> 19) & 0x00000001) {
              case 0x00000000:
                decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerV(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerV);
                ctx->cur_opcode->format = kv3_LSU_SQPB_O;
                ctx->cur_opcode->insn = kv3_SO;
                return trans_v1_SO_lsucond_registerY_offset27_registerZ_registerV_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerV);
              case 0x00000001:
                switch ((codeWord_0 >> 20) & 0x0000000f) {
                case 0x00000000:
                  decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FZCB_O;
                  ctx->cur_opcode->insn = kv3_DZEROL;
                  return trans_v1_DZEROL_lsucond_registerY_offset27_registerZ_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
                default:
                  break;
                }
                return false;
                break;
              }
              return false;
              break;
            }
            return false;
          case 0x00000001:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerE(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerE);
            ctx->cur_opcode->format = kv3_LSU_SVPB_O;
            ctx->cur_opcode->insn = kv3_SV;
            return trans_v1_SV_lsucond_registerY_offset27_registerZ_registerE_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerE);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASPB_O;
            ctx->cur_opcode->insn = kv3_ALCLRW;
            return trans_v1_ALCLRW_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          case 0x00000001:
            decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASPB_O;
            ctx->cur_opcode->insn = kv3_ALCLRD;
            return trans_v1_ALCLRD_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x00000001) {
          case 0x00000000:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AAPB_O;
            ctx->cur_opcode->insn = kv3_ALADDW;
            return trans_v1_ALADDW_lsucond_registerY_offset27_registerZ_registerT_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
          case 0x00000001:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AAPB_O;
            ctx->cur_opcode->insn = kv3_ALADDD;
            return trans_v1_ALADDD_lsucond_registerY_offset27_registerZ_registerT_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x0000000b:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBO_X;
          ctx->cur_opcode->insn = kv3_LWZ;
          return trans_v1_LWZ_variant_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBO_X;
          ctx->cur_opcode->insn = kv3_LWS;
          return trans_v1_LWS_variant_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBO_X;
          ctx->cur_opcode->insn = kv3_LD;
          return trans_v1_LD_variant_registerW_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerW_kv3_upper27_lower10_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerM_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerM_kv3_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LPBO_X;
            ctx->cur_opcode->insn = kv3_LQ;
            return trans_v1_LQ_variant_registerM_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerM_kv3_upper27_lower10_kv3_registerZ);
          case 0x00000001:
            switch ((codeWord_0 >> 19) & 0x00000001) {
            case 0x00000000:
              decode_kv3_variant_kv3_registerN_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerN_kv3_upper27_lower10_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_LQBO_X;
              ctx->cur_opcode->insn = kv3_LO;
              return trans_v1_LO_variant_registerN_upper27_lower10_registerZ_double(ctx, &u.kv3_variant_kv3_registerN_kv3_upper27_lower10_kv3_registerZ);
            case 0x00000001:
              switch ((codeWord_0 >> 20) & 0x0000000f) {
              case 0x00000000:
                decode_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXBO_X;
                ctx->cur_opcode->insn = kv3_DTOUCHL;
                return trans_v1_DTOUCHL_upper27_lower10_registerZ_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ);
              case 0x00000001:
                decode_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXBO_X;
                ctx->cur_opcode->insn = kv3_DINVALL;
                return trans_v1_DINVALL_upper27_lower10_registerZ_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ);
              case 0x00000005:
                decode_kv3_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXBO_X;
                ctx->cur_opcode->insn = kv3_IINVALS;
                return trans_v1_IINVALS_upper27_lower10_registerZ_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ);
              default:
                break;
              }
              return false;
              break;
            }
            return false;
            break;
          }
          return false;
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x0000000e:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APBO_X;
            ctx->cur_opcode->insn = kv3_ACSWAPW;
            return trans_v1_ACSWAPW_upper27_lower10_registerZ_registerU_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerU);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_upper27_lower10_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APBO_X;
            ctx->cur_opcode->insn = kv3_ACSWAPD;
            return trans_v1_ACSWAPD_upper27_lower10_registerZ_registerU_double(ctx, &u.kv3_upper27_lower10_kv3_registerZ_kv3_registerU);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB_O;
          ctx->cur_opcode->insn = kv3_LWZ;
          return trans_v1_LWZ_variant_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB_O;
          ctx->cur_opcode->insn = kv3_LWS;
          return trans_v1_LWS_variant_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB_O;
          ctx->cur_opcode->insn = kv3_LD;
          return trans_v1_LD_variant_lsucond_registerY_registerW_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_offset27_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LPPB_O;
            ctx->cur_opcode->insn = kv3_LQ;
            return trans_v1_LQ_variant_lsucond_registerY_registerM_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_offset27_kv3_registerZ);
          case 0x00000001:
            switch ((codeWord_0 >> 19) & 0x00000001) {
            case 0x00000000:
              decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_offset27_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_LPPB1_O;
              ctx->cur_opcode->insn = kv3_LO;
              return trans_v1_LO_variant_lsucond_registerY_registerN_offset27_registerZ_double(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_offset27_kv3_registerZ);
            case 0x00000001:
              switch ((codeWord_0 >> 20) & 0x0000000f) {
              case 0x00000000:
                decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXCB_O;
                ctx->cur_opcode->insn = kv3_DTOUCHL;
                return trans_v1_DTOUCHL_lsucond_registerY_offset27_registerZ_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
              case 0x00000001:
                decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXCB_O;
                ctx->cur_opcode->insn = kv3_DINVALL;
                return trans_v1_DINVALL_lsucond_registerY_offset27_registerZ_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
              case 0x00000005:
                decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXCB_O;
                ctx->cur_opcode->insn = kv3_IINVALS;
                return trans_v1_IINVALS_lsucond_registerY_offset27_registerZ_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ);
              default:
                break;
              }
              return false;
              break;
            }
            return false;
            break;
          }
          return false;
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x0000000e:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APPB_O;
            ctx->cur_opcode->insn = kv3_ACSWAPW;
            return trans_v1_ACSWAPW_lsucond_registerY_offset27_registerZ_registerU_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APPB_O;
            ctx->cur_opcode->insn = kv3_ACSWAPD;
            return trans_v1_ACSWAPD_lsucond_registerY_offset27_registerZ_registerU_double(ctx, &u.kv3_lsucond_kv3_registerY_kv3_offset27_kv3_registerZ_kv3_registerU);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x0000000c:
    switch ((codeWord_0 >> 24) & 0x0000000f) {
    case 0x00000000:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DDDI0_X;
          ctx->cur_opcode->insn = kv3_MADDD;
          return trans_v1_MADDD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMAD;
          return trans_v1_FFMAD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FADDD;
          return trans_v1_FADDD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQQDI_X;
            ctx->cur_opcode->insn = kv3_FFMAWDP;
            return trans_v1_FFMAWDP_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQDI_X;
            ctx->cur_opcode->insn = kv3_FMULWDP;
            return trans_v1_FMULWDP_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DDDI0_X;
          ctx->cur_opcode->insn = kv3_MADDWP;
          return trans_v1_MADDWP_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMAWD;
          return trans_v1_FFMAWD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FADDWP;
          return trans_v1_FADDWP_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQQDI_X;
            ctx->cur_opcode->insn = kv3_FFMAHWQ;
            return trans_v1_FFMAHWQ_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQDI_X;
            ctx->cur_opcode->insn = kv3_FMULHWQ;
            return trans_v1_FMULHWQ_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DDDI0_X;
          ctx->cur_opcode->insn = kv3_MADDHQ;
          return trans_v1_MADDHQ_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMAWP;
          return trans_v1_FFMAWP_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FADDHQ;
          return trans_v1_FADDHQ_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQQDI_X;
            ctx->cur_opcode->insn = kv3_FFMSWDP;
            return trans_v1_FFMSWDP_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQDI_X;
            ctx->cur_opcode->insn = kv3_FMULWDC;
            return trans_v1_FMULWDC_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMAHQ;
          return trans_v1_FFMAHQ_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FADDCWC;
          return trans_v1_FADDCWC_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQQDI_X;
            ctx->cur_opcode->insn = kv3_FFMSHWQ;
            return trans_v1_FFMSHWQ_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FQDI_X;
            ctx->cur_opcode->insn = kv3_FMULCWDC;
            return trans_v1_FMULCWDC_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000004:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DDI_X;
          ctx->cur_opcode->insn = kv3_MULD;
          return trans_v1_MULD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMSD;
          return trans_v1_FFMSD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FSBFD;
          return trans_v1_FSBFD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000005:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DDI_X;
          ctx->cur_opcode->insn = kv3_MULWP;
          return trans_v1_MULWP_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMSWD;
          return trans_v1_FFMSWD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FSBFWP;
          return trans_v1_FSBFWP_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000006:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DDI_X;
          ctx->cur_opcode->insn = kv3_MULHQ;
          return trans_v1_MULHQ_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMSWP;
          return trans_v1_FFMSWP_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FSBFHQ;
          return trans_v1_FSBFHQ_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000007:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DDI_X;
          ctx->cur_opcode->insn = kv3_MULWC;
          return trans_v1_MULWC_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDDI_X;
          ctx->cur_opcode->insn = kv3_FFMSHQ;
          return trans_v1_FFMSHQ_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI1_X;
          ctx->cur_opcode->insn = kv3_FSBFCWC;
          return trans_v1_FSBFCWC_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000008:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QQDI_X;
            ctx->cur_opcode->insn = kv3_MADDDT;
            return trans_v1_MADDDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QDI_X;
            ctx->cur_opcode->insn = kv3_MULDT;
            return trans_v1_MULDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FMULD;
          return trans_v1_FMULD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSSI_X;
          ctx->cur_opcode->insn = kv3_FFMAHW;
          return trans_v1_FFMAHW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000009:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QQDI_X;
            ctx->cur_opcode->insn = kv3_MADDUDT;
            return trans_v1_MADDUDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QDI_X;
            ctx->cur_opcode->insn = kv3_MULUDT;
            return trans_v1_MULUDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FMULWD;
          return trans_v1_FMULWD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSSI_X;
          ctx->cur_opcode->insn = kv3_FFMAW;
          return trans_v1_FFMAW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000a:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QQDI_X;
            ctx->cur_opcode->insn = kv3_MADDSUDT;
            return trans_v1_MADDSUDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QDI_X;
            ctx->cur_opcode->insn = kv3_MULSUDT;
            return trans_v1_MULSUDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FMULWP;
          return trans_v1_FMULWP_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSSI_X;
          ctx->cur_opcode->insn = kv3_FFMSHW;
          return trans_v1_FFMSHW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000b:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QQDI_X;
            ctx->cur_opcode->insn = kv3_MADDUZDT;
            return trans_v1_MADDUZDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_QDI_X;
            ctx->cur_opcode->insn = kv3_CMULDT;
            return trans_v1_CMULDT_registerM_registerZ_upper27_lower10_double(ctx, &u.kv3_registerM_kv3_registerZ_kv3_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FMULHQ;
          return trans_v1_FMULHQ_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSSI_X;
          ctx->cur_opcode->insn = kv3_FFMSW;
          return trans_v1_FFMSW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000c:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DPI_X;
          ctx->cur_opcode->insn = kv3_DOT2WD;
          return trans_v1_DOT2WD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FDOT2W;
          return trans_v1_FDOT2W_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSI_X;
          ctx->cur_opcode->insn = kv3_FADDW;
          return trans_v1_FADDW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000d:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DPI_X;
          ctx->cur_opcode->insn = kv3_DOT2UWD;
          return trans_v1_DOT2UWD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FDOT2WD;
          return trans_v1_FDOT2WD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSI_X;
          ctx->cur_opcode->insn = kv3_FSBFW;
          return trans_v1_FSBFW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000e:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DPI_X;
          ctx->cur_opcode->insn = kv3_DOT2SUWD;
          return trans_v1_DOT2SUWD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FMULWC;
          return trans_v1_FMULWC_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSI_X;
          ctx->cur_opcode->insn = kv3_FMULW;
          return trans_v1_FMULW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000f:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_DPI_X;
          ctx->cur_opcode->insn = kv3_DOT2W;
          return trans_v1_DOT2W_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FDDI_X;
          ctx->cur_opcode->insn = kv3_FMULCWC;
          return trans_v1_FMULCWC_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_MAU_FSSI_X;
          ctx->cur_opcode->insn = kv3_FMULHW;
          return trans_v1_FMULHW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x0000000d:
    switch ((codeWord_0 >> 12) & 0x0000003f) {
    case 0x00000002:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x00000008:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_CRC_X;
          ctx->cur_opcode->insn = kv3_CRCBELMW;
          return trans_v1_CRCBELMW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000009:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_CRC_X;
          ctx->cur_opcode->insn = kv3_CRCBELLW;
          return trans_v1_CRCBELLW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x0000000a:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_CRC_X;
          ctx->cur_opcode->insn = kv3_CRCLELMW;
          return trans_v1_CRCLELMW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x0000000b:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_CRC_X;
          ctx->cur_opcode->insn = kv3_CRCLELLW;
          return trans_v1_CRCLELLW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MADDWD;
          return trans_v1_MADDWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MADDUWD;
          return trans_v1_MADDUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MADDSUWD;
          return trans_v1_MADDSUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MADDW;
          return trans_v1_MADDW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MSBFWD;
          return trans_v1_MSBFWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MSBFUWD;
          return trans_v1_MSBFUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MSBFSUWD;
          return trans_v1_MSBFSUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSSS_X;
          ctx->cur_opcode->insn = kv3_MSBFW;
          return trans_v1_MSBFW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000008:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSS_X;
          ctx->cur_opcode->insn = kv3_MULWD;
          return trans_v1_MULWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x00000009:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSS_X;
          ctx->cur_opcode->insn = kv3_MULUWD;
          return trans_v1_MULUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x0000000a:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSS_X;
          ctx->cur_opcode->insn = kv3_MULSUWD;
          return trans_v1_MULSUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      case 0x0000000b:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          ctx->cur_opcode->format = kv3_MAU_SSS_X;
          ctx->cur_opcode->insn = kv3_MULW;
          return trans_v1_MULW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    default:
      break;
    }
    return false;
  case 0x0000000e:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_extend6_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_extend6_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WI_X;
          ctx->cur_opcode->insn = kv3_MAKE;
          return trans_v1_MAKE_registerW_extend6_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_extend6_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_ADDD;
          return trans_v1_ADDD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_MIND;
          return trans_v1_MIND_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_MAXD;
          return trans_v1_MAXD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_ABDD;
          return trans_v1_ABDD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_SBFD;
          return trans_v1_SBFD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_MINUD;
          return trans_v1_MINUD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_MAXUD;
          return trans_v1_MAXUD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000008:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_ANDD;
          return trans_v1_ANDD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000009:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_NANDD;
          return trans_v1_NANDD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000a:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_ORD;
          return trans_v1_ORD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000b:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_NORD;
          return trans_v1_NORD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000c:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_XORD;
          return trans_v1_XORD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000d:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_NXORD;
          return trans_v1_NXORD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000e:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_ANDND;
          return trans_v1_ANDND_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRI_X;
          ctx->cur_opcode->insn = kv3_ORND;
          return trans_v1_ORND_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_1 >> 29) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIS_X;
          ctx->cur_opcode->insn = kv3_ADDSD;
          return trans_v1_ADDSD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIS_X;
          ctx->cur_opcode->insn = kv3_SBFSD;
          return trans_v1_SBFSD_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_CWRI_X;
          ctx->cur_opcode->insn = kv3_COMPD;
          return trans_v1_COMPD_comparison_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_1 >> 29) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_BWRI_X;
          ctx->cur_opcode->insn = kv3_SBMM8;
          return trans_v1_SBMM8_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_BWRI_X;
          ctx->cur_opcode->insn = kv3_SBMMT8;
          return trans_v1_SBMMT8_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_upper27_lower10(ctx, codeWords, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_MWRI_X;
          ctx->cur_opcode->insn = kv3_CMOVED;
          return trans_v1_CMOVED_scalarcond_registerZ_registerW_upper27_lower10_double(ctx, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_upper27_lower10);
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    default:
      break;
    }
    return false;
  case 0x0000000f:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_extend6_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_extend6_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WIPC_X;
          ctx->cur_opcode->insn = kv3_PCREL;
          return trans_v1_PCREL_registerW_extend6_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_extend6_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_ADDW;
          return trans_v1_ADDW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_MINW;
          return trans_v1_MINW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_MAXW;
          return trans_v1_MAXW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_ABDW;
          return trans_v1_ABDW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_SBFW;
          return trans_v1_SBFW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_MINUW;
          return trans_v1_MINUW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_MAXUW;
          return trans_v1_MAXUW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000008:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_ANDW;
          return trans_v1_ANDW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x00000009:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_NANDW;
          return trans_v1_NANDW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000a:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_ORW;
          return trans_v1_ORW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000b:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_NORW;
          return trans_v1_NORW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000c:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_XORW;
          return trans_v1_XORW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000d:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_NXORW;
          return trans_v1_NXORW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000e:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_ANDNW;
          return trans_v1_ANDNW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
          ctx->cur_opcode->format = kv3_ALU_WRIW_X;
          ctx->cur_opcode->insn = kv3_ORNW;
          return trans_v1_ORNW_registerW_registerZ_upper27_lower10_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower10);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 12) & 0x0000000f) {
      case 0x00000002:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000001:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
            ctx->cur_opcode->insn = kv3_ADDWP;
            return trans_v1_ADDWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
            ctx->cur_opcode->insn = kv3_MINWP;
            return trans_v1_MINWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
            ctx->cur_opcode->insn = kv3_MAXWP;
            return trans_v1_MAXWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_1 >> 0) & 0x07ffffff) {
            case 0x00000000:
              switch ((codeWord_0 >> 6) & 0x0000003f) {
              case 0x00000000:
                decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
                ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
                ctx->cur_opcode->insn = kv3_ABSWP;
                return trans_v1_ABSWP_registerW_registerZ_double(ctx, &u.kv3_registerW_kv3_registerZ);
              default:
                decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
                ctx->cur_opcode->insn = kv3_ABDWP;
                return trans_v1_ABDWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                break;
              }
              return false;
            default:
              decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
              ctx->cur_opcode->insn = kv3_ABDWP;
              return trans_v1_ABDWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_1 >> 0) & 0x07ffffff) {
            case 0x00000000:
              switch ((codeWord_0 >> 6) & 0x0000003f) {
              case 0x00000000:
                decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
                ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
                ctx->cur_opcode->insn = kv3_NEGWP;
                return trans_v1_NEGWP_registerW_registerZ_double(ctx, &u.kv3_registerW_kv3_registerZ);
              default:
                decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
                ctx->cur_opcode->insn = kv3_SBFWP;
                return trans_v1_SBFWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                break;
              }
              return false;
            default:
              decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
              ctx->cur_opcode->insn = kv3_SBFWP;
              return trans_v1_SBFWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
            ctx->cur_opcode->insn = kv3_MINUWP;
            return trans_v1_MINUWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
            ctx->cur_opcode->insn = kv3_MAXUWP;
            return trans_v1_MAXUWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
            ctx->cur_opcode->insn = kv3_ADDCWC;
            return trans_v1_ADDCWC_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP0_X;
            ctx->cur_opcode->insn = kv3_SBFCWC;
            return trans_v1_SBFCWC_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000001:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
            ctx->cur_opcode->insn = kv3_ADDHQ;
            return trans_v1_ADDHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
            ctx->cur_opcode->insn = kv3_MINHQ;
            return trans_v1_MINHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
            ctx->cur_opcode->insn = kv3_MAXHQ;
            return trans_v1_MAXHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_1 >> 0) & 0x07ffffff) {
            case 0x00000000:
              switch ((codeWord_0 >> 6) & 0x0000003f) {
              case 0x00000000:
                decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
                ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
                ctx->cur_opcode->insn = kv3_ABSHQ;
                return trans_v1_ABSHQ_registerW_registerZ_double(ctx, &u.kv3_registerW_kv3_registerZ);
              default:
                decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
                ctx->cur_opcode->insn = kv3_ABDHQ;
                return trans_v1_ABDHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                break;
              }
              return false;
            default:
              decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
              ctx->cur_opcode->insn = kv3_ABDHQ;
              return trans_v1_ABDHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_1 >> 0) & 0x07ffffff) {
            case 0x00000000:
              switch ((codeWord_0 >> 6) & 0x0000003f) {
              case 0x00000000:
                decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
                ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
                ctx->cur_opcode->insn = kv3_NEGHQ;
                return trans_v1_NEGHQ_registerW_registerZ_double(ctx, &u.kv3_registerW_kv3_registerZ);
              default:
                decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
                ctx->cur_opcode->insn = kv3_SBFHQ;
                return trans_v1_SBFHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
                break;
              }
              return false;
            default:
              decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
              ctx->cur_opcode->insn = kv3_SBFHQ;
              return trans_v1_SBFHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
            ctx->cur_opcode->insn = kv3_MINUHQ;
            return trans_v1_MINUHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
            ctx->cur_opcode->insn = kv3_MAXUHQ;
            return trans_v1_MAXUHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
            ctx->cur_opcode->insn = kv3_ADDCHCP;
            return trans_v1_ADDCHCP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ0_X;
            ctx->cur_opcode->insn = kv3_SBFCHCP;
            return trans_v1_SBFCHCP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX2D;
            return trans_v1_ADDX2D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX2D;
            return trans_v1_SBFX2D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX4D;
            return trans_v1_ADDX4D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX4D;
            return trans_v1_SBFX4D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX8D;
            return trans_v1_ADDX8D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX8D;
            return trans_v1_SBFX8D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX16D;
            return trans_v1_ADDX16D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX16D;
            return trans_v1_SBFX16D_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000008:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_ADDWD;
            return trans_v1_ADDWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000009:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_SBFWD;
            return trans_v1_SBFWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000a:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_ADDUWD;
            return trans_v1_ADDUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000b:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_SBFUWD;
            return trans_v1_SBFUWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_LANDD;
            return trans_v1_LANDD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_LNANDD;
            return trans_v1_LNANDD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000e:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_LORD;
            return trans_v1_LORD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRR1_X;
            ctx->cur_opcode->insn = kv3_LNORD;
            return trans_v1_LNORD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX2W;
            return trans_v1_ADDX2W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX2W;
            return trans_v1_SBFX2W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX4W;
            return trans_v1_ADDX4W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX4W;
            return trans_v1_SBFX4W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX8W;
            return trans_v1_ADDX8W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX8W;
            return trans_v1_SBFX8W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_ADDX16W;
            return trans_v1_ADDX16W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_SBFX16W;
            return trans_v1_SBFX16W_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000008:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_AVGW;
            return trans_v1_AVGW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000009:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_AVGUW;
            return trans_v1_AVGUW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000a:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_AVGRW;
            return trans_v1_AVGRW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000b:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_AVGRUW;
            return trans_v1_AVGRUW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_LANDW;
            return trans_v1_LANDW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_LNANDW;
            return trans_v1_LNANDW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000e:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_LORW;
            return trans_v1_LORW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR1_X;
            ctx->cur_opcode->insn = kv3_LNORW;
            return trans_v1_LNORW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_ADDX2WP;
            return trans_v1_ADDX2WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_SBFX2WP;
            return trans_v1_SBFX2WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_ADDX4WP;
            return trans_v1_ADDX4WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_SBFX4WP;
            return trans_v1_SBFX4WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_ADDX8WP;
            return trans_v1_ADDX8WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_SBFX8WP;
            return trans_v1_SBFX8WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_ADDX16WP;
            return trans_v1_ADDX16WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_SBFX16WP;
            return trans_v1_SBFX16WP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000008:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_AVGWP;
            return trans_v1_AVGWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000009:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_AVGUWP;
            return trans_v1_AVGUWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000a:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_AVGRWP;
            return trans_v1_AVGRWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000b:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_AVGRUWP;
            return trans_v1_AVGRUWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_LANDWP;
            return trans_v1_LANDWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_LNANDWP;
            return trans_v1_LNANDWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000e:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_LORWP;
            return trans_v1_LORWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRWP1_X;
            ctx->cur_opcode->insn = kv3_LNORWP;
            return trans_v1_LNORWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_ADDX2HQ;
            return trans_v1_ADDX2HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_SBFX2HQ;
            return trans_v1_SBFX2HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_ADDX4HQ;
            return trans_v1_ADDX4HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_SBFX4HQ;
            return trans_v1_SBFX4HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_ADDX8HQ;
            return trans_v1_ADDX8HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_SBFX8HQ;
            return trans_v1_SBFX8HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_ADDX16HQ;
            return trans_v1_ADDX16HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_SBFX16HQ;
            return trans_v1_SBFX16HQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000008:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_AVGHQ;
            return trans_v1_AVGHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000009:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_AVGUHQ;
            return trans_v1_AVGUHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000a:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_AVGRHQ;
            return trans_v1_AVGRHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000b:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_AVGRUHQ;
            return trans_v1_AVGRUHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_LANDHQ;
            return trans_v1_LANDHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_LNANDHQ;
            return trans_v1_LNANDHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000e:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_LORHQ;
            return trans_v1_LORHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_WRRHQ1_X;
            ctx->cur_opcode->insn = kv3_LNORHQ;
            return trans_v1_LNORHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000008:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX2WD;
            return trans_v1_ADDX2WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX2WD;
            return trans_v1_SBFX2WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX4WD;
            return trans_v1_ADDX4WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX4WD;
            return trans_v1_SBFX4WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX8WD;
            return trans_v1_ADDX8WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX8WD;
            return trans_v1_SBFX8WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX16WD;
            return trans_v1_ADDX16WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX16WD;
            return trans_v1_SBFX16WD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000008:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX2UWD;
            return trans_v1_ADDX2UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x00000009:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX2UWD;
            return trans_v1_SBFX2UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000a:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX4UWD;
            return trans_v1_ADDX4UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000b:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX4UWD;
            return trans_v1_SBFX4UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX8UWD;
            return trans_v1_ADDX8UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX8UWD;
            return trans_v1_SBFX8UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000e:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_ADDX16UWD;
            return trans_v1_ADDX16UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DWRR2_X;
            ctx->cur_opcode->insn = kv3_SBFX16UWD;
            return trans_v1_SBFX16UWD_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x0000000b:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x0000000f) {
          case 0x0000000e:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_AWRRW_X;
            ctx->cur_opcode->insn = kv3_ADDSW;
            return trans_v1_ADDSW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          case 0x0000000f:
            decode_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_AWRRW_X;
            ctx->cur_opcode->insn = kv3_SBFSW;
            return trans_v1_SBFSW_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_DCWRR_X;
            ctx->cur_opcode->insn = kv3_COMPW;
            return trans_v1_COMPW_comparison_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x0000000e:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_AWRRWP_X;
            ctx->cur_opcode->insn = kv3_ADDSWP;
            return trans_v1_ADDSWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_AWRRWP_X;
            ctx->cur_opcode->insn = kv3_SBFSWP;
            return trans_v1_SBFSWP_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_AWRRHQ_X;
            ctx->cur_opcode->insn = kv3_ADDSHQ;
            return trans_v1_ADDSHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_1 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_AWRRHQ_X;
            ctx->cur_opcode->insn = kv3_SBFSHQ;
            return trans_v1_SBFSHQ_splat32_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_splat32_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 12) & 0x0000000f) {
      case 0x00000000:
        switch ((codeWord_1 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 27) & 0x00000001) {
          case 0x00000000:
            decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_FCWRRS_X;
            ctx->cur_opcode->insn = kv3_FCOMPW;
            return trans_v1_FCOMPW_floatcomp_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
          case 0x00000001:
            decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5(ctx, codeWords, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            ctx->cur_opcode->format = kv3_ALU_FCWRRS_X;
            ctx->cur_opcode->insn = kv3_FCOMPD;
            return trans_v1_FCOMPD_floatcomp_registerW_registerZ_upper27_lower5_double(ctx, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_upper27_lower5);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    default:
      break;
    }
    return false;
  default:
    break;
  }
  return false;
}

bool decode_v1_simple(DisasContext *ctx, const uint32_t *buffer)
{
  const uint32_t *codeWords = (const uint32_t *)buffer;
  union decode_formats u;
  uint32_t codeWord_0 = codeWords[0];
  switch ((codeWord_0 >> 28) & 0x00000007) {
  case 0x00000000:
    switch ((codeWord_0 >> 27) & 0x00000001) {
    case 0x00000000:
      switch ((codeWord_0 >> 24) & 0x00000007) {
      case 0x00000000:
        switch ((codeWord_0 >> 18) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 6) & 0x00000fff) {
          case 0x00000000:
            switch ((codeWord_0 >> 20) & 0x0000000f) {
            case 0x00000000:
              ctx->cur_opcode->format = kv3_BCU_NOB;
              ctx->cur_opcode->insn = kv3_ERROP;
              return trans_v1_ERROP_simple(ctx, &u.empty);
            default:
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            switch ((codeWord_0 >> 12) & 0x00000001) {
            case 0x00000001:
              switch ((codeWord_0 >> 0) & 0x0000003f) {
              case 0x00000000:
                switch ((codeWord_0 >> 7) & 0x0000001f) {
                case 0x0000001f:
                  decode_kv3_registerN_kv3_registerBe(ctx, codeWords, &u.kv3_registerN_kv3_registerBe);
                  ctx->cur_opcode->format = kv3_BCU_AOEOI;
                  ctx->cur_opcode->insn = kv3_MOVEFO;
                  return trans_v1_MOVEFO_registerN_registerBe_simple(ctx, &u.kv3_registerN_kv3_registerBe);
                default:
                  decode_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift(ctx, codeWords, &u.kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift);
                  ctx->cur_opcode->format = kv3_BCU_AOEOI;
                  ctx->cur_opcode->insn = kv3_ALIGNO;
                  return trans_v1_ALIGNO_registerN_registerBe_registerCo_byteshift_simple(ctx, &u.kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift);
                  break;
                }
                return false;
              default:
                decode_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift(ctx, codeWords, &u.kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift);
                ctx->cur_opcode->format = kv3_BCU_AOEOI;
                ctx->cur_opcode->insn = kv3_ALIGNO;
                return trans_v1_ALIGNO_registerN_registerBe_registerCo_byteshift_simple(ctx, &u.kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_byteshift);
                break;
              }
              return false;
            default:
              break;
            }
            return false;
          case 0x00000001:
            switch ((codeWord_0 >> 12) & 0x00000001) {
            case 0x00000001:
              decode_kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_registerZ(ctx, codeWords, &u.kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_registerZ);
              ctx->cur_opcode->format = kv3_BCU_AOEOR;
              ctx->cur_opcode->insn = kv3_ALIGNO;
              return trans_v1_ALIGNO_registerN_registerBe_registerCo_registerZ_simple(ctx, &u.kv3_registerN_kv3_registerBe_kv3_registerCo_kv3_registerZ);
            default:
              break;
            }
            return false;
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            switch ((codeWord_0 >> 12) & 0x00000001) {
            case 0x00000001:
              switch ((codeWord_0 >> 0) & 0x0000003f) {
              case 0x00000000:
                switch ((codeWord_0 >> 7) & 0x0000001f) {
                case 0x0000001f:
                  decode_kv3_registerN_kv3_registerBo(ctx, codeWords, &u.kv3_registerN_kv3_registerBo);
                  ctx->cur_opcode->format = kv3_BCU_AOOEI;
                  ctx->cur_opcode->insn = kv3_MOVEFO;
                  return trans_v1_MOVEFO_registerN_registerBo_simple(ctx, &u.kv3_registerN_kv3_registerBo);
                default:
                  decode_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift(ctx, codeWords, &u.kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift);
                  ctx->cur_opcode->format = kv3_BCU_AOOEI;
                  ctx->cur_opcode->insn = kv3_ALIGNO;
                  return trans_v1_ALIGNO_registerN_registerBo_registerCe_byteshift_simple(ctx, &u.kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift);
                  break;
                }
                return false;
              default:
                decode_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift(ctx, codeWords, &u.kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift);
                ctx->cur_opcode->format = kv3_BCU_AOOEI;
                ctx->cur_opcode->insn = kv3_ALIGNO;
                return trans_v1_ALIGNO_registerN_registerBo_registerCe_byteshift_simple(ctx, &u.kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_byteshift);
                break;
              }
              return false;
            default:
              break;
            }
            return false;
          case 0x00000001:
            switch ((codeWord_0 >> 12) & 0x00000001) {
            case 0x00000001:
              decode_kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_registerZ(ctx, codeWords, &u.kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_registerZ);
              ctx->cur_opcode->format = kv3_BCU_AOOER;
              ctx->cur_opcode->insn = kv3_ALIGNO;
              return trans_v1_ALIGNO_registerN_registerBo_registerCe_registerZ_simple(ctx, &u.kv3_registerN_kv3_registerBo_kv3_registerCe_kv3_registerZ);
            default:
              break;
            }
            return false;
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 6) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 12) & 0x00000001) {
          case 0x00000000:
            switch ((codeWord_0 >> 0) & 0x0000003f) {
            case 0x00000000:
              switch ((codeWord_0 >> 7) & 0x0000001f) {
              case 0x0000001f:
                decode_kv3_registerA_kv3_registerBe(ctx, codeWords, &u.kv3_registerA_kv3_registerBe);
                ctx->cur_opcode->format = kv3_BCU_AVEOI;
                ctx->cur_opcode->insn = kv3_COPYV;
                return trans_v1_COPYV_registerA_registerBe_simple(ctx, &u.kv3_registerA_kv3_registerBe);
              default:
                decode_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift(ctx, codeWords, &u.kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift);
                ctx->cur_opcode->format = kv3_BCU_AVEOI;
                ctx->cur_opcode->insn = kv3_ALIGNV;
                return trans_v1_ALIGNV_registerA_registerBe_registerCo_byteshift_simple(ctx, &u.kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift);
                break;
              }
              return false;
            default:
              decode_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift(ctx, codeWords, &u.kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift);
              ctx->cur_opcode->format = kv3_BCU_AVEOI;
              ctx->cur_opcode->insn = kv3_ALIGNV;
              return trans_v1_ALIGNV_registerA_registerBe_registerCo_byteshift_simple(ctx, &u.kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_byteshift);
              break;
            }
            return false;
          case 0x00000001:
            switch ((codeWord_0 >> 0) & 0x0000003f) {
            case 0x00000000:
              switch ((codeWord_0 >> 7) & 0x0000001f) {
              case 0x0000001f:
                decode_kv3_registerA_kv3_registerBo(ctx, codeWords, &u.kv3_registerA_kv3_registerBo);
                ctx->cur_opcode->format = kv3_BCU_AVOEI;
                ctx->cur_opcode->insn = kv3_COPYV;
                return trans_v1_COPYV_registerA_registerBo_simple(ctx, &u.kv3_registerA_kv3_registerBo);
              default:
                decode_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift(ctx, codeWords, &u.kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift);
                ctx->cur_opcode->format = kv3_BCU_AVOEI;
                ctx->cur_opcode->insn = kv3_ALIGNV;
                return trans_v1_ALIGNV_registerA_registerBo_registerCe_byteshift_simple(ctx, &u.kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift);
                break;
              }
              return false;
            default:
              decode_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift(ctx, codeWords, &u.kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift);
              ctx->cur_opcode->format = kv3_BCU_AVOEI;
              ctx->cur_opcode->insn = kv3_ALIGNV;
              return trans_v1_ALIGNV_registerA_registerBo_registerCe_byteshift_simple(ctx, &u.kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_byteshift);
              break;
            }
            return false;
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 12) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_registerZ(ctx, codeWords, &u.kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_registerZ);
            ctx->cur_opcode->format = kv3_BCU_AVEOR;
            ctx->cur_opcode->insn = kv3_ALIGNV;
            return trans_v1_ALIGNV_registerA_registerBe_registerCo_registerZ_simple(ctx, &u.kv3_registerA_kv3_registerBe_kv3_registerCo_kv3_registerZ);
          case 0x00000001:
            decode_kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_registerZ(ctx, codeWords, &u.kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_registerZ);
            ctx->cur_opcode->format = kv3_BCU_AVOER;
            ctx->cur_opcode->insn = kv3_ALIGNV;
            return trans_v1_ALIGNV_registerA_registerBo_registerCe_registerZ_simple(ctx, &u.kv3_registerA_kv3_registerBo_kv3_registerCe_kv3_registerZ);
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 12) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMABW0;
            ctx->cur_opcode->insn = kv3_MMA484BW;
            return trans_v1_MMA484BW_registerAp_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMABW2;
            ctx->cur_opcode->insn = kv3_MMA484SUBW;
            return trans_v1_MMA484SUBW_registerAp_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMABW1;
            ctx->cur_opcode->insn = kv3_MMA484UBW;
            return trans_v1_MMA484UBW_registerAp_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMABW3;
            ctx->cur_opcode->insn = kv3_MMA484USBW;
            return trans_v1_MMA484USBW_registerAp_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerAp_kv3_registerBp_kv3_registerC_kv3_registerD);
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 12) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerA0_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerA0_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_FMMAHW0;
            ctx->cur_opcode->insn = kv3_FMMA242HW0;
            return trans_v1_FMMA242HW0_registerA0_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerA0_kv3_registerBp_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerA2_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerA2_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_FMMAHW2;
            ctx->cur_opcode->insn = kv3_FMMA242HW2;
            return trans_v1_FMMA242HW2_registerA2_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerA2_kv3_registerBp_kv3_registerC_kv3_registerD);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerA1_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerA1_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_FMMAHW1;
            ctx->cur_opcode->insn = kv3_FMMA242HW1;
            return trans_v1_FMMA242HW1_registerA1_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerA1_kv3_registerBp_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerA3_kv3_registerBp_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerA3_kv3_registerBp_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_FMMAHW3;
            ctx->cur_opcode->insn = kv3_FMMA242HW3;
            return trans_v1_FMMA242HW3_registerA3_registerBp_registerC_registerD_simple(ctx, &u.kv3_registerA3_kv3_registerBp_kv3_registerC_kv3_registerD);
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_0 >> 12) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD0;
            ctx->cur_opcode->insn = kv3_MMA444HBD0;
            return trans_v1_MMA444HBD0_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD1;
            ctx->cur_opcode->insn = kv3_MMA444HBD1;
            return trans_v1_MMA444HBD1_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000002:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHD;
            ctx->cur_opcode->insn = kv3_MMA444HD;
            return trans_v1_MMA444HD_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000003:
            decode_kv3_registerAq_kv3_registerBq(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq);
            ctx->cur_opcode->format = kv3_TCA_MTD;
            ctx->cur_opcode->insn = kv3_MT44D;
            return trans_v1_MT44D_registerAq_registerBq_simple(ctx, &u.kv3_registerAq_kv3_registerBq);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD0;
            ctx->cur_opcode->insn = kv3_MMA444UHBD0;
            return trans_v1_MMA444UHBD0_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD1;
            ctx->cur_opcode->insn = kv3_MMA444UHBD1;
            return trans_v1_MMA444UHBD1_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000002:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHD;
            ctx->cur_opcode->insn = kv3_MMA444UHD;
            return trans_v1_MMA444UHD_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_0 >> 18) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD0;
            ctx->cur_opcode->insn = kv3_MMA444SUHBD0;
            return trans_v1_MMA444SUHBD0_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD1;
            ctx->cur_opcode->insn = kv3_MMA444SUHBD1;
            return trans_v1_MMA444SUHBD1_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000002:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHD;
            ctx->cur_opcode->insn = kv3_MMA444SUHD;
            return trans_v1_MMA444SUHD_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_0 >> 18) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD0;
            ctx->cur_opcode->insn = kv3_MMA444USHBD0;
            return trans_v1_MMA444USHBD0_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000001:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHBD1;
            ctx->cur_opcode->insn = kv3_MMA444USHBD1;
            return trans_v1_MMA444USHBD1_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          case 0x00000002:
            decode_kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD(ctx, codeWords, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
            ctx->cur_opcode->format = kv3_TCA_MMAHD;
            ctx->cur_opcode->insn = kv3_MMA444USHD;
            return trans_v1_MMA444USHD_registerAq_registerBq_registerC_registerD_simple(ctx, &u.kv3_registerAq_kv3_registerBq_kv3_registerC_kv3_registerD);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 12) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            decode_kv3_roundint_kv3_saturate_kv3_registerAl_kv3_registerBq(ctx, codeWords, &u.kv3_roundint_kv3_saturate_kv3_registerAl_kv3_registerBq);
            ctx->cur_opcode->format = kv3_TCA_CDHVL;
            ctx->cur_opcode->insn = kv3_CONVDHV0;
            return trans_v1_CONVDHV0_roundint_saturate_registerAl_registerBq_simple(ctx, &u.kv3_roundint_kv3_saturate_kv3_registerAl_kv3_registerBq);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            decode_kv3_roundint_kv3_saturate_kv3_registerAh_kv3_registerBq(ctx, codeWords, &u.kv3_roundint_kv3_saturate_kv3_registerAh_kv3_registerBq);
            ctx->cur_opcode->format = kv3_TCA_CDHVH;
            ctx->cur_opcode->insn = kv3_CONVDHV1;
            return trans_v1_CONVDHV1_roundint_saturate_registerAh_registerBq_simple(ctx, &u.kv3_roundint_kv3_saturate_kv3_registerAh_kv3_registerBq);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_0 >> 12) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            decode_kv3_roundint_kv3_saturate_kv3_registerAx_kv3_registerBq(ctx, codeWords, &u.kv3_roundint_kv3_saturate_kv3_registerAx_kv3_registerBq);
            ctx->cur_opcode->format = kv3_TCA_CWBV0;
            ctx->cur_opcode->insn = kv3_CONVWBV0;
            return trans_v1_CONVWBV0_roundint_saturate_registerAx_registerBq_simple(ctx, &u.kv3_roundint_kv3_saturate_kv3_registerAx_kv3_registerBq);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            decode_kv3_roundint_kv3_saturate_kv3_registerAy_kv3_registerBq(ctx, codeWords, &u.kv3_roundint_kv3_saturate_kv3_registerAy_kv3_registerBq);
            ctx->cur_opcode->format = kv3_TCA_CWBV1;
            ctx->cur_opcode->insn = kv3_CONVWBV1;
            return trans_v1_CONVWBV1_roundint_saturate_registerAy_registerBq_simple(ctx, &u.kv3_roundint_kv3_saturate_kv3_registerAy_kv3_registerBq);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            decode_kv3_roundint_kv3_saturate_kv3_registerAz_kv3_registerBq(ctx, codeWords, &u.kv3_roundint_kv3_saturate_kv3_registerAz_kv3_registerBq);
            ctx->cur_opcode->format = kv3_TCA_CWBV2;
            ctx->cur_opcode->insn = kv3_CONVWBV2;
            return trans_v1_CONVWBV2_roundint_saturate_registerAz_registerBq_simple(ctx, &u.kv3_roundint_kv3_saturate_kv3_registerAz_kv3_registerBq);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_0 >> 6) & 0x00000001) {
          case 0x00000000:
            decode_kv3_roundint_kv3_saturate_kv3_registerAt_kv3_registerBq(ctx, codeWords, &u.kv3_roundint_kv3_saturate_kv3_registerAt_kv3_registerBq);
            ctx->cur_opcode->format = kv3_TCA_CWBV3;
            ctx->cur_opcode->insn = kv3_CONVWBV3;
            return trans_v1_CONVWBV3_roundint_saturate_registerAt_registerBq_simple(ctx, &u.kv3_roundint_kv3_saturate_kv3_registerAt_kv3_registerBq);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_0 >> 6) & 0x00000001) {
        case 0x00000000:
          decode_kv3_xrounding_kv3_silent2_kv3_rectify_kv3_registerA_kv3_registerB(ctx, codeWords, &u.kv3_xrounding_kv3_silent2_kv3_rectify_kv3_registerA_kv3_registerB);
          ctx->cur_opcode->format = kv3_TCA_FSWV;
          ctx->cur_opcode->insn = kv3_FSCALEWV;
          return trans_v1_FSCALEWV_xrounding_silent2_rectify_registerA_registerB_simple(ctx, &u.kv3_xrounding_kv3_silent2_kv3_rectify_kv3_registerA_kv3_registerB);
        case 0x00000001:
          switch ((codeWord_0 >> 12) & 0x00000001) {
          case 0x00000000:
            decode_kv3_xrounding_kv3_silent2_kv3_registerA_kv3_registerBp(ctx, codeWords, &u.kv3_xrounding_kv3_silent2_kv3_registerA_kv3_registerBp);
            ctx->cur_opcode->format = kv3_TCA_FNWHV;
            ctx->cur_opcode->insn = kv3_FNARROWWHV;
            return trans_v1_FNARROWWHV_xrounding_silent2_registerA_registerBp_simple(ctx, &u.kv3_xrounding_kv3_silent2_kv3_registerA_kv3_registerBp);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 23) & 0x0000000f) {
      case 0x0000000e:
        decode_kv3_registerZ_kv3_pcrel17(ctx, codeWords, &u.kv3_registerZ_kv3_pcrel17);
        ctx->cur_opcode->format = kv3_BCU_HLS;
        ctx->cur_opcode->insn = kv3_LOOPDO;
        return trans_v1_LOOPDO_registerZ_pcrel17_simple(ctx, &u.kv3_registerZ_kv3_pcrel17);
      case 0x0000000f:
        switch ((codeWord_0 >> 18) & 0x0000001f) {
        case 0x00000000:
          ctx->cur_opcode->format = kv3_BCU_TLB;
          ctx->cur_opcode->insn = kv3_TLBREAD;
          return trans_v1_TLBREAD_simple(ctx, &u.empty);
        case 0x00000001:
          ctx->cur_opcode->format = kv3_BCU_TLB;
          ctx->cur_opcode->insn = kv3_TLBPROBE;
          return trans_v1_TLBPROBE_simple(ctx, &u.empty);
        case 0x00000002:
          ctx->cur_opcode->format = kv3_BCU_TLB;
          ctx->cur_opcode->insn = kv3_TLBWRITE;
          return trans_v1_TLBWRITE_simple(ctx, &u.empty);
        case 0x00000003:
          ctx->cur_opcode->format = kv3_BCU_TLB;
          ctx->cur_opcode->insn = kv3_TLBDINVAL;
          return trans_v1_TLBDINVAL_simple(ctx, &u.empty);
        case 0x00000004:
          ctx->cur_opcode->format = kv3_BCU_TLB;
          ctx->cur_opcode->insn = kv3_TLBIINVAL;
          return trans_v1_TLBIINVAL_simple(ctx, &u.empty);
        case 0x00000008:
          ctx->cur_opcode->format = kv3_BCU_IPC;
          ctx->cur_opcode->insn = kv3_AWAIT;
          return trans_v1_AWAIT_simple(ctx, &u.empty);
        case 0x00000009:
          ctx->cur_opcode->format = kv3_BCU_IPC;
          ctx->cur_opcode->insn = kv3_SLEEP;
          return trans_v1_SLEEP_simple(ctx, &u.empty);
        case 0x0000000a:
          ctx->cur_opcode->format = kv3_BCU_IPC;
          ctx->cur_opcode->insn = kv3_STOP;
          return trans_v1_STOP_simple(ctx, &u.empty);
        case 0x0000000b:
          ctx->cur_opcode->format = kv3_BCU_IPC;
          ctx->cur_opcode->insn = kv3_BARRIER;
          return trans_v1_BARRIER_simple(ctx, &u.empty);
        case 0x0000000c:
          decode_kv3_registerZ(ctx, codeWords, &u.kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_PGI;
          ctx->cur_opcode->insn = kv3_WAITIT;
          return trans_v1_WAITIT_registerZ_simple(ctx, &u.kv3_registerZ);
        case 0x0000000d:
          decode_kv3_registerZ(ctx, codeWords, &u.kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_PGI;
          ctx->cur_opcode->insn = kv3_SYNCGROUP;
          return trans_v1_SYNCGROUP_registerZ_simple(ctx, &u.kv3_registerZ);
        case 0x0000000e:
          decode_kv3_systemT2_kv3_registerZ(ctx, codeWords, &u.kv3_systemT2_kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_WFX;
          ctx->cur_opcode->insn = kv3_WFXL;
          return trans_v1_WFXL_systemT2_registerZ_simple(ctx, &u.kv3_systemT2_kv3_registerZ);
        case 0x0000000f:
          decode_kv3_systemT2_kv3_registerZ(ctx, codeWords, &u.kv3_systemT2_kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_WFX;
          ctx->cur_opcode->insn = kv3_WFXM;
          return trans_v1_WFXM_systemT2_registerZ_simple(ctx, &u.kv3_systemT2_kv3_registerZ);
        case 0x00000010:
          decode_kv3_systemT3_kv3_registerZ(ctx, codeWords, &u.kv3_systemT3_kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_SET;
          ctx->cur_opcode->insn = kv3_SET;
          return trans_v1_SET_systemT3_registerZ_simple(ctx, &u.kv3_systemT3_kv3_registerZ);
        case 0x00000011:
          decode_kv3_registerZ_kv3_systemS2(ctx, codeWords, &u.kv3_registerZ_kv3_systemS2);
          ctx->cur_opcode->format = kv3_BCU_GSR;
          ctx->cur_opcode->insn = kv3_GET;
          return trans_v1_GET_registerZ_systemS2_simple(ctx, &u.kv3_registerZ_kv3_systemS2);
        case 0x00000012:
          decode_kv3_registerZ_kv3_systemS4(ctx, codeWords, &u.kv3_registerZ_kv3_systemS4);
          ctx->cur_opcode->format = kv3_BCU_RSWAP;
          ctx->cur_opcode->insn = kv3_RSWAP;
          return trans_v1_RSWAP_registerZ_systemS4_simple(ctx, &u.kv3_registerZ_kv3_systemS4);
        case 0x00000013:
          decode_kv3_registerZ(ctx, codeWords, &u.kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_IGSR;
          ctx->cur_opcode->insn = kv3_IGET;
          return trans_v1_IGET_registerZ_simple(ctx, &u.kv3_registerZ);
        case 0x00000014:
          ctx->cur_opcode->format = kv3_BCU_RTS;
          ctx->cur_opcode->insn = kv3_RET;
          return trans_v1_RET_simple(ctx, &u.empty);
        case 0x00000015:
          ctx->cur_opcode->format = kv3_BCU_RTS;
          ctx->cur_opcode->insn = kv3_RFE;
          return trans_v1_RFE_simple(ctx, &u.empty);
        case 0x00000016:
          decode_kv3_registerZ(ctx, codeWords, &u.kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_IBC;
          ctx->cur_opcode->insn = kv3_IGOTO;
          return trans_v1_IGOTO_registerZ_simple(ctx, &u.kv3_registerZ);
        case 0x00000017:
          decode_kv3_registerZ(ctx, codeWords, &u.kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_IBC;
          ctx->cur_opcode->insn = kv3_ICALL;
          return trans_v1_ICALL_registerZ_simple(ctx, &u.kv3_registerZ);
        case 0x00000018:
          decode_kv3_sysnumber(ctx, codeWords, &u.kv3_sysnumber);
          ctx->cur_opcode->format = kv3_BCU_SCI;
          ctx->cur_opcode->insn = kv3_SCALL;
          return trans_v1_SCALL_sysnumber_simple(ctx, &u.kv3_sysnumber);
        case 0x00000019:
          decode_kv3_registerZ(ctx, codeWords, &u.kv3_registerZ);
          ctx->cur_opcode->format = kv3_BCU_SCR;
          ctx->cur_opcode->insn = kv3_SCALL;
          return trans_v1_SCALL_registerZ_simple(ctx, &u.kv3_registerZ);
        default:
          decode_kv3_branchcond_kv3_registerZ_kv3_pcrel17(ctx, codeWords, &u.kv3_branchcond_kv3_registerZ_kv3_pcrel17);
          ctx->cur_opcode->format = kv3_BCU_CB;
          ctx->cur_opcode->insn = kv3_CB;
          return trans_v1_CB_branchcond_registerZ_pcrel17_simple(ctx, &u.kv3_branchcond_kv3_registerZ_kv3_pcrel17);
          break;
        }
        return false;
      default:
        decode_kv3_branchcond_kv3_registerZ_kv3_pcrel17(ctx, codeWords, &u.kv3_branchcond_kv3_registerZ_kv3_pcrel17);
        ctx->cur_opcode->format = kv3_BCU_CB;
        ctx->cur_opcode->insn = kv3_CB;
        return trans_v1_CB_branchcond_registerZ_pcrel17_simple(ctx, &u.kv3_branchcond_kv3_registerZ_kv3_pcrel17);
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x00000001:
    switch ((codeWord_0 >> 27) & 0x00000001) {
    case 0x00000000:
      decode_kv3_pcrel27(ctx, codeWords, &u.kv3_pcrel27);
      ctx->cur_opcode->format = kv3_BCU_UB;
      ctx->cur_opcode->insn = kv3_GOTO;
      return trans_v1_GOTO_pcrel27_simple(ctx, &u.kv3_pcrel27);
    case 0x00000001:
      decode_kv3_pcrel27(ctx, codeWords, &u.kv3_pcrel27);
      ctx->cur_opcode->format = kv3_BCU_UB;
      ctx->cur_opcode->insn = kv3_CALL;
      return trans_v1_CALL_pcrel27_simple(ctx, &u.kv3_pcrel27);
      break;
    }
    return false;
  case 0x00000002:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LSBO;
        ctx->cur_opcode->insn = kv3_LBZ;
        return trans_v1_LBZ_variant_registerW_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
      case 0x00000001:
        decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LSBO;
        ctx->cur_opcode->insn = kv3_LBS;
        return trans_v1_LBS_variant_registerW_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
      case 0x00000002:
        decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LSBO;
        ctx->cur_opcode->insn = kv3_LHZ;
        return trans_v1_LHZ_variant_registerW_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
      case 0x00000003:
        decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LSBO;
        ctx->cur_opcode->insn = kv3_LHS;
        return trans_v1_LHS_variant_registerW_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 25) & 0x00000007) {
      case 0x00000000:
        decode_kv3_speculate_kv3_registerA_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_speculate_kv3_registerA_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LVBO;
        ctx->cur_opcode->insn = kv3_LV;
        return trans_v1_LV_speculate_registerA_signed10_registerZ_simple(ctx, &u.kv3_speculate_kv3_registerA_kv3_signed10_kv3_registerZ);
      case 0x00000001:
        decode_kv3_column_kv3_speculate_kv3_registerAq_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_column_kv3_speculate_kv3_registerAq_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LVSBO;
        ctx->cur_opcode->insn = kv3_LV;
        return trans_v1_LV_column_speculate_registerAq_signed10_registerZ_simple(ctx, &u.kv3_column_kv3_speculate_kv3_registerAq_kv3_signed10_kv3_registerZ);
      case 0x00000002:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          decode_kv3_signed10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          ctx->cur_opcode->format = kv3_LSU_SSBO;
          ctx->cur_opcode->insn = kv3_SB;
          return trans_v1_SB_signed10_registerZ_registerT_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
        case 0x00000001:
          decode_kv3_signed10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          ctx->cur_opcode->format = kv3_LSU_SSBO;
          ctx->cur_opcode->insn = kv3_SH;
          return trans_v1_SH_signed10_registerZ_registerT_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          decode_kv3_signed10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          ctx->cur_opcode->format = kv3_LSU_SSBO;
          ctx->cur_opcode->insn = kv3_SW;
          return trans_v1_SW_signed10_registerZ_registerT_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
        case 0x00000001:
          decode_kv3_signed10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          ctx->cur_opcode->format = kv3_LSU_SSBO;
          ctx->cur_opcode->insn = kv3_SD;
          return trans_v1_SD_signed10_registerZ_registerT_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_signed10_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_SPBO;
            ctx->cur_opcode->insn = kv3_SQ;
            return trans_v1_SQ_signed10_registerZ_registerU_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerU);
          case 0x00000001:
            switch ((codeWord_0 >> 19) & 0x00000001) {
            case 0x00000000:
              decode_kv3_signed10_kv3_registerZ_kv3_registerV(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerV);
              ctx->cur_opcode->format = kv3_LSU_SQBO;
              ctx->cur_opcode->insn = kv3_SO;
              return trans_v1_SO_signed10_registerZ_registerV_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerV);
            case 0x00000001:
              switch ((codeWord_0 >> 20) & 0x0000000f) {
              case 0x00000000:
                decode_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_signed10_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FZBO;
                ctx->cur_opcode->insn = kv3_DZEROL;
                return trans_v1_DZEROL_signed10_registerZ_simple(ctx, &u.kv3_signed10_kv3_registerZ);
              default:
                break;
              }
              return false;
              break;
            }
            return false;
            break;
          }
          return false;
        case 0x00000001:
          decode_kv3_signed10_kv3_registerZ_kv3_registerE(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerE);
          ctx->cur_opcode->format = kv3_LSU_SVBO;
          ctx->cur_opcode->insn = kv3_SV;
          return trans_v1_SV_signed10_registerZ_registerE_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerE);
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_signed10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_ASBO;
          ctx->cur_opcode->insn = kv3_ALCLRW;
          return trans_v1_ALCLRW_registerW_signed10_registerZ_simple(ctx, &u.kv3_registerW_kv3_signed10_kv3_registerZ);
        case 0x00000001:
          decode_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_signed10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_ASBO;
          ctx->cur_opcode->insn = kv3_ALCLRD;
          return trans_v1_ALCLRD_registerW_signed10_registerZ_simple(ctx, &u.kv3_registerW_kv3_signed10_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          decode_kv3_signed10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          ctx->cur_opcode->format = kv3_LSU_AABO;
          ctx->cur_opcode->insn = kv3_ALADDW;
          return trans_v1_ALADDW_signed10_registerZ_registerT_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
        case 0x00000001:
          decode_kv3_signed10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          ctx->cur_opcode->format = kv3_LSU_AABO;
          ctx->cur_opcode->insn = kv3_ALADDD;
          return trans_v1_ALADDD_signed10_registerZ_registerT_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerT);
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBI;
          ctx->cur_opcode->insn = kv3_LBZ;
          return trans_v1_LBZ_variant_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB;
          ctx->cur_opcode->insn = kv3_LBZ;
          return trans_v1_LBZ_variant_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBI;
          ctx->cur_opcode->insn = kv3_LBS;
          return trans_v1_LBS_variant_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB;
          ctx->cur_opcode->insn = kv3_LBS;
          return trans_v1_LBS_variant_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBI;
          ctx->cur_opcode->insn = kv3_LHZ;
          return trans_v1_LHZ_variant_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB;
          ctx->cur_opcode->insn = kv3_LHZ;
          return trans_v1_LHZ_variant_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBI;
          ctx->cur_opcode->insn = kv3_LHS;
          return trans_v1_LHS_variant_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB;
          ctx->cur_opcode->insn = kv3_LHS;
          return trans_v1_LHS_variant_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 25) & 0x00000007) {
      case 0x00000000:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_speculate_kv3_scaling_kv3_registerA_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_speculate_kv3_scaling_kv3_registerA_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVBI;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_speculate_scaling_registerA_registerY_registerZ_simple(ctx, &u.kv3_speculate_kv3_scaling_kv3_registerA_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_registerZ(ctx, codeWords, &u.kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVPB;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_speculate_lsucond_registerY_registerA_registerZ_simple(ctx, &u.kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_column_kv3_speculate_kv3_scaling_kv3_registerAq_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_column_kv3_speculate_kv3_scaling_kv3_registerAq_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVSBI;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_column_speculate_scaling_registerAq_registerY_registerZ_simple(ctx, &u.kv3_column_kv3_speculate_kv3_scaling_kv3_registerAq_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_registerZ(ctx, codeWords, &u.kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LVSPB;
          ctx->cur_opcode->insn = kv3_LV;
          return trans_v1_LV_column_speculate_lsucond_registerY_registerAq_registerZ_simple(ctx, &u.kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBI;
            ctx->cur_opcode->insn = kv3_SB;
            return trans_v1_SB_scaling_registerY_registerZ_registerT_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB;
            ctx->cur_opcode->insn = kv3_SB;
            return trans_v1_SB_lsucond_registerY_registerZ_registerT_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBI;
            ctx->cur_opcode->insn = kv3_SH;
            return trans_v1_SH_scaling_registerY_registerZ_registerT_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB;
            ctx->cur_opcode->insn = kv3_SH;
            return trans_v1_SH_lsucond_registerY_registerZ_registerT_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBI;
            ctx->cur_opcode->insn = kv3_SW;
            return trans_v1_SW_scaling_registerY_registerZ_registerT_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB;
            ctx->cur_opcode->insn = kv3_SW;
            return trans_v1_SW_lsucond_registerY_registerZ_registerT_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSBI;
            ctx->cur_opcode->insn = kv3_SD;
            return trans_v1_SD_scaling_registerY_registerZ_registerT_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_SSPB;
            ctx->cur_opcode->insn = kv3_SD;
            return trans_v1_SD_lsucond_registerY_registerZ_registerT_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            switch ((codeWord_0 >> 13) & 0x00000007) {
            case 0x00000007:
              decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU);
              ctx->cur_opcode->format = kv3_LSU_SPBI;
              ctx->cur_opcode->insn = kv3_SQ;
              return trans_v1_SQ_scaling_registerY_registerZ_registerU_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU);
            default:
              decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU);
              ctx->cur_opcode->format = kv3_LSU_SPPB;
              ctx->cur_opcode->insn = kv3_SQ;
              return trans_v1_SQ_lsucond_registerY_registerZ_registerU_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU);
              break;
            }
            return false;
          case 0x00000001:
            switch ((codeWord_0 >> 19) & 0x00000001) {
            case 0x00000000:
              switch ((codeWord_0 >> 13) & 0x00000007) {
              case 0x00000007:
                decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerV(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerV);
                ctx->cur_opcode->format = kv3_LSU_SQBI;
                ctx->cur_opcode->insn = kv3_SO;
                return trans_v1_SO_scaling_registerY_registerZ_registerV_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerV);
              default:
                decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerV(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerV);
                ctx->cur_opcode->format = kv3_LSU_SQPB;
                ctx->cur_opcode->insn = kv3_SO;
                return trans_v1_SO_lsucond_registerY_registerZ_registerV_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerV);
                break;
              }
              return false;
            case 0x00000001:
              switch ((codeWord_0 >> 20) & 0x0000000f) {
              case 0x00000000:
                switch ((codeWord_0 >> 12) & 0x0000000f) {
                case 0x0000000e:
                  decode_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_registerY_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FZBI;
                  ctx->cur_opcode->insn = kv3_DZEROL;
                  return trans_v1_DZEROL_registerY_registerZ_simple(ctx, &u.kv3_registerY_kv3_registerZ);
                default:
                  decode_kv3_lsucond_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FZCB;
                  ctx->cur_opcode->insn = kv3_DZEROL;
                  return trans_v1_DZEROL_lsucond_registerY_registerZ_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                  break;
                }
                return false;
              default:
                break;
              }
              return false;
              break;
            }
            return false;
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerE(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerE);
            ctx->cur_opcode->format = kv3_LSU_SVBI;
            ctx->cur_opcode->insn = kv3_SV;
            return trans_v1_SV_scaling_registerY_registerZ_registerE_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerE);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerE(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerE);
            ctx->cur_opcode->format = kv3_LSU_SVPB;
            ctx->cur_opcode->insn = kv3_SV;
            return trans_v1_SV_lsucond_registerY_registerZ_registerE_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerE);
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASBI;
            ctx->cur_opcode->insn = kv3_ALCLRW;
            return trans_v1_ALCLRW_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASPB;
            ctx->cur_opcode->insn = kv3_ALCLRW;
            return trans_v1_ALCLRW_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASBI;
            ctx->cur_opcode->insn = kv3_ALCLRD;
            return trans_v1_ALCLRD_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_ASPB;
            ctx->cur_opcode->insn = kv3_ALCLRD;
            return trans_v1_ALCLRD_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_0 >> 24) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AABI;
            ctx->cur_opcode->insn = kv3_ALADDW;
            return trans_v1_ALADDW_scaling_registerY_registerZ_registerT_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AAPB;
            ctx->cur_opcode->insn = kv3_ALADDW;
            return trans_v1_ALADDW_lsucond_registerY_registerZ_registerT_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AABI;
            ctx->cur_opcode->insn = kv3_ALADDD;
            return trans_v1_ALADDD_scaling_registerY_registerZ_registerT_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerT);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            ctx->cur_opcode->format = kv3_LSU_AAPB;
            ctx->cur_opcode->insn = kv3_ALADDD;
            return trans_v1_ALADDD_lsucond_registerY_registerZ_registerT_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerT);
            break;
          }
          return false;
          break;
        }
        return false;
      default:
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x00000003:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LSBO;
        ctx->cur_opcode->insn = kv3_LWZ;
        return trans_v1_LWZ_variant_registerW_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
      case 0x00000001:
        decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LSBO;
        ctx->cur_opcode->insn = kv3_LWS;
        return trans_v1_LWS_variant_registerW_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
      case 0x00000002:
        decode_kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
        ctx->cur_opcode->format = kv3_LSU_LSBO;
        ctx->cur_opcode->insn = kv3_LD;
        return trans_v1_LD_variant_registerW_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerW_kv3_signed10_kv3_registerZ);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_variant_kv3_registerM_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerM_kv3_signed10_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LPBO;
          ctx->cur_opcode->insn = kv3_LQ;
          return trans_v1_LQ_variant_registerM_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerM_kv3_signed10_kv3_registerZ);
        case 0x00000001:
          switch ((codeWord_0 >> 19) & 0x00000001) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerN_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerN_kv3_signed10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LQBO;
            ctx->cur_opcode->insn = kv3_LO;
            return trans_v1_LO_variant_registerN_signed10_registerZ_simple(ctx, &u.kv3_variant_kv3_registerN_kv3_signed10_kv3_registerZ);
          case 0x00000001:
            switch ((codeWord_0 >> 20) & 0x0000000f) {
            case 0x00000000:
              decode_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_signed10_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_FXBO;
              ctx->cur_opcode->insn = kv3_DTOUCHL;
              return trans_v1_DTOUCHL_signed10_registerZ_simple(ctx, &u.kv3_signed10_kv3_registerZ);
            case 0x00000001:
              decode_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_signed10_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_FXBO;
              ctx->cur_opcode->insn = kv3_DINVALL;
              return trans_v1_DINVALL_signed10_registerZ_simple(ctx, &u.kv3_signed10_kv3_registerZ);
            case 0x00000005:
              decode_kv3_signed10_kv3_registerZ(ctx, codeWords, &u.kv3_signed10_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_FXBO;
              ctx->cur_opcode->insn = kv3_IINVALS;
              return trans_v1_IINVALS_signed10_registerZ_simple(ctx, &u.kv3_signed10_kv3_registerZ);
            default:
              break;
            }
            return false;
            break;
          }
          return false;
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x0000000e:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_signed10_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerU);
          ctx->cur_opcode->format = kv3_LSU_APBO;
          ctx->cur_opcode->insn = kv3_ACSWAPW;
          return trans_v1_ACSWAPW_signed10_registerZ_registerU_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerU);
        case 0x00000001:
          switch ((codeWord_0 >> 19) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerN_kv3_registerQ(ctx, codeWords, &u.kv3_registerN_kv3_registerQ);
            ctx->cur_opcode->format = kv3_LSU_COPYO;
            ctx->cur_opcode->insn = kv3_COPYO;
            return trans_v1_COPYO_registerN_registerQ_simple(ctx, &u.kv3_registerN_kv3_registerQ);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_signed10_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_signed10_kv3_registerZ_kv3_registerU);
          ctx->cur_opcode->format = kv3_LSU_APBO;
          ctx->cur_opcode->insn = kv3_ACSWAPD;
          return trans_v1_ACSWAPD_signed10_registerZ_registerU_simple(ctx, &u.kv3_signed10_kv3_registerZ_kv3_registerU);
        case 0x00000001:
          switch ((codeWord_0 >> 19) & 0x0000001f) {
          case 0x00000011:
            ctx->cur_opcode->format = kv3_LSU_MCC;
            ctx->cur_opcode->insn = kv3_DINVAL;
            return trans_v1_DINVAL_simple(ctx, &u.empty);
          case 0x00000013:
            ctx->cur_opcode->format = kv3_LSU_MCC;
            ctx->cur_opcode->insn = kv3_IINVAL;
            return trans_v1_IINVAL_simple(ctx, &u.empty);
          case 0x00000019:
            ctx->cur_opcode->format = kv3_LSU_MCC;
            ctx->cur_opcode->insn = kv3_FENCE;
            return trans_v1_FENCE_simple(ctx, &u.empty);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBI;
          ctx->cur_opcode->insn = kv3_LWZ;
          return trans_v1_LWZ_variant_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB;
          ctx->cur_opcode->insn = kv3_LWZ;
          return trans_v1_LWZ_variant_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBI;
          ctx->cur_opcode->insn = kv3_LWS;
          return trans_v1_LWS_variant_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB;
          ctx->cur_opcode->insn = kv3_LWS;
          return trans_v1_LWS_variant_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 13) & 0x00000007) {
        case 0x00000007:
          decode_kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSBI;
          ctx->cur_opcode->insn = kv3_LD;
          return trans_v1_LD_variant_scaling_registerW_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerW_kv3_registerY_kv3_registerZ);
        default:
          decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_LSU_LSPB;
          ctx->cur_opcode->insn = kv3_LD;
          return trans_v1_LD_variant_lsucond_registerY_registerW_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_registerZ);
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_variant_kv3_scaling_kv3_registerM_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerM_kv3_registerY_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LPBI;
            ctx->cur_opcode->insn = kv3_LQ;
            return trans_v1_LQ_variant_scaling_registerM_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerM_kv3_registerY_kv3_registerZ);
          default:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LPPB;
            ctx->cur_opcode->insn = kv3_LQ;
            return trans_v1_LQ_variant_lsucond_registerY_registerM_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_registerZ);
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_0 >> 19) & 0x00000001) {
          case 0x00000000:
            switch ((codeWord_0 >> 13) & 0x00000007) {
            case 0x00000007:
              decode_kv3_variant_kv3_scaling_kv3_registerN_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_scaling_kv3_registerN_kv3_registerY_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_LPBI1;
              ctx->cur_opcode->insn = kv3_LO;
              return trans_v1_LO_variant_scaling_registerN_registerY_registerZ_simple(ctx, &u.kv3_variant_kv3_scaling_kv3_registerN_kv3_registerY_kv3_registerZ);
            default:
              decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_LPPB1;
              ctx->cur_opcode->insn = kv3_LO;
              return trans_v1_LO_variant_lsucond_registerY_registerN_registerZ_simple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_registerZ);
              break;
            }
            return false;
          case 0x00000001:
            switch ((codeWord_0 >> 20) & 0x0000000f) {
            case 0x00000000:
              switch ((codeWord_0 >> 12) & 0x0000000f) {
              case 0x0000000e:
                decode_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_registerY_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXBI;
                ctx->cur_opcode->insn = kv3_DTOUCHL;
                return trans_v1_DTOUCHL_registerY_registerZ_simple(ctx, &u.kv3_registerY_kv3_registerZ);
              default:
                decode_kv3_lsucond_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXCB;
                ctx->cur_opcode->insn = kv3_DTOUCHL;
                return trans_v1_DTOUCHL_lsucond_registerY_registerZ_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                break;
              }
              return false;
            case 0x00000001:
              switch ((codeWord_0 >> 12) & 0x0000000f) {
              case 0x0000000e:
                decode_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_registerY_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXBI;
                ctx->cur_opcode->insn = kv3_DINVALL;
                return trans_v1_DINVALL_registerY_registerZ_simple(ctx, &u.kv3_registerY_kv3_registerZ);
              default:
                decode_kv3_lsucond_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXCB;
                ctx->cur_opcode->insn = kv3_DINVALL;
                return trans_v1_DINVALL_lsucond_registerY_registerZ_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                break;
              }
              return false;
            case 0x00000005:
              switch ((codeWord_0 >> 12) & 0x0000000f) {
              case 0x0000000e:
                decode_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_registerY_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXBI;
                ctx->cur_opcode->insn = kv3_IINVALS;
                return trans_v1_IINVALS_registerY_registerZ_simple(ctx, &u.kv3_registerY_kv3_registerZ);
              default:
                decode_kv3_lsucond_kv3_registerY_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_FXCB;
                ctx->cur_opcode->insn = kv3_IINVALS;
                return trans_v1_IINVALS_lsucond_registerY_registerZ_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ);
                break;
              }
              return false;
            default:
              break;
            }
            return false;
            break;
          }
          return false;
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x0000000e:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APBI;
            ctx->cur_opcode->insn = kv3_ACSWAPW;
            return trans_v1_ACSWAPW_scaling_registerY_registerZ_registerU_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APPB;
            ctx->cur_opcode->insn = kv3_ACSWAPW;
            return trans_v1_ACSWAPW_lsucond_registerY_registerZ_registerU_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          switch ((codeWord_0 >> 13) & 0x00000007) {
          case 0x00000007:
            decode_kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APBI;
            ctx->cur_opcode->insn = kv3_ACSWAPD;
            return trans_v1_ACSWAPD_scaling_registerY_registerZ_registerU_simple(ctx, &u.kv3_scaling_kv3_registerY_kv3_registerZ_kv3_registerU);
          default:
            decode_kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU);
            ctx->cur_opcode->format = kv3_LSU_APPB;
            ctx->cur_opcode->insn = kv3_ACSWAPD;
            return trans_v1_ACSWAPD_lsucond_registerY_registerZ_registerU_simple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerZ_kv3_registerU);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x00000004:
    switch ((codeWord_0 >> 24) & 0x0000000f) {
    case 0x00000000:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DDDI0;
        ctx->cur_opcode->insn = kv3_MADDD;
        return trans_v1_MADDD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMAD;
        return trans_v1_FFMAD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FADDD;
        return trans_v1_FADDD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQQDI;
          ctx->cur_opcode->insn = kv3_FFMAWDP;
          return trans_v1_FFMAWDP_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQDI;
          ctx->cur_opcode->insn = kv3_FMULWDP;
          return trans_v1_FMULWDP_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DDDI0;
        ctx->cur_opcode->insn = kv3_MADDWP;
        return trans_v1_MADDWP_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMAWD;
        return trans_v1_FFMAWD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FADDWP;
        return trans_v1_FADDWP_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQQDI;
          ctx->cur_opcode->insn = kv3_FFMAHWQ;
          return trans_v1_FFMAHWQ_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQDI;
          ctx->cur_opcode->insn = kv3_FMULHWQ;
          return trans_v1_FMULHWQ_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DDDI0;
        ctx->cur_opcode->insn = kv3_MADDHQ;
        return trans_v1_MADDHQ_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMAWP;
        return trans_v1_FFMAWP_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FADDHQ;
        return trans_v1_FADDHQ_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQQDI;
          ctx->cur_opcode->insn = kv3_FFMSWDP;
          return trans_v1_FFMSWDP_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQDI;
          ctx->cur_opcode->insn = kv3_FMULWDC;
          return trans_v1_FMULWDC_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMAHQ;
        return trans_v1_FFMAHQ_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FADDCWC;
        return trans_v1_FADDCWC_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQQDI;
          ctx->cur_opcode->insn = kv3_FFMSHWQ;
          return trans_v1_FFMSHWQ_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_FQDI;
          ctx->cur_opcode->insn = kv3_FMULCWDC;
          return trans_v1_FMULCWDC_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000004:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DDI;
        ctx->cur_opcode->insn = kv3_MULD;
        return trans_v1_MULD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMSD;
        return trans_v1_FFMSD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FSBFD;
        return trans_v1_FSBFD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FLOATD;
        return trans_v1_FLOATD_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x00000005:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DDI;
        ctx->cur_opcode->insn = kv3_MULWP;
        return trans_v1_MULWP_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMSWD;
        return trans_v1_FFMSWD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FSBFWP;
        return trans_v1_FSBFWP_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FLOATUD;
        return trans_v1_FLOATUD_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x00000006:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DDI;
        ctx->cur_opcode->insn = kv3_MULHQ;
        return trans_v1_MULHQ_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMSWP;
        return trans_v1_FFMSWP_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FSBFHQ;
        return trans_v1_FSBFHQ_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FIXEDD;
        return trans_v1_FIXEDD_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x00000007:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DDI;
        ctx->cur_opcode->insn = kv3_MULWC;
        return trans_v1_MULWC_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDDI;
        ctx->cur_opcode->insn = kv3_FFMSHQ;
        return trans_v1_FFMSHQ_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI1;
        ctx->cur_opcode->insn = kv3_FSBFCWC;
        return trans_v1_FSBFCWC_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FIXEDUD;
        return trans_v1_FIXEDUD_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x00000008:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QQDI;
          ctx->cur_opcode->insn = kv3_MADDDT;
          return trans_v1_MADDDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QDI;
          ctx->cur_opcode->insn = kv3_MULDT;
          return trans_v1_MULDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FMULD;
        return trans_v1_FMULD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSSI;
        ctx->cur_opcode->insn = kv3_FFMAHW;
        return trans_v1_FFMAHW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FLOATW;
        return trans_v1_FLOATW_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x00000009:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QQDI;
          ctx->cur_opcode->insn = kv3_MADDUDT;
          return trans_v1_MADDUDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QDI;
          ctx->cur_opcode->insn = kv3_MULUDT;
          return trans_v1_MULUDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FMULWD;
        return trans_v1_FMULWD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSSI;
        ctx->cur_opcode->insn = kv3_FFMAW;
        return trans_v1_FFMAW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FLOATUW;
        return trans_v1_FLOATUW_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x0000000a:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QQDI;
          ctx->cur_opcode->insn = kv3_MADDSUDT;
          return trans_v1_MADDSUDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QDI;
          ctx->cur_opcode->insn = kv3_MULSUDT;
          return trans_v1_MULSUDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FMULWP;
        return trans_v1_FMULWP_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSSI;
        ctx->cur_opcode->insn = kv3_FFMSHW;
        return trans_v1_FFMSHW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FIXEDW;
        return trans_v1_FIXEDW_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x0000000b:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QQDI;
          ctx->cur_opcode->insn = kv3_MADDUZDT;
          return trans_v1_MADDUZDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_MAU_QDI;
          ctx->cur_opcode->insn = kv3_CMULDT;
          return trans_v1_CMULDT_registerM_registerZ_signed10_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FMULHQ;
        return trans_v1_FMULHQ_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSSI;
        ctx->cur_opcode->insn = kv3_FFMSW;
        return trans_v1_FFMSW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FIXEDUW;
        return trans_v1_FIXEDUW_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x0000000c:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DPI;
        ctx->cur_opcode->insn = kv3_DOT2WD;
        return trans_v1_DOT2WD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FDOT2W;
        return trans_v1_FDOT2W_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSI;
        ctx->cur_opcode->insn = kv3_FADDW;
        return trans_v1_FADDW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FLOATWP;
        return trans_v1_FLOATWP_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x0000000d:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DPI;
        ctx->cur_opcode->insn = kv3_DOT2UWD;
        return trans_v1_DOT2UWD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FDOT2WD;
        return trans_v1_FDOT2WD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSI;
        ctx->cur_opcode->insn = kv3_FSBFW;
        return trans_v1_FSBFW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FLOATUWP;
        return trans_v1_FLOATUWP_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x0000000e:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DPI;
        ctx->cur_opcode->insn = kv3_DOT2SUWD;
        return trans_v1_DOT2SUWD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FMULWC;
        return trans_v1_FMULWC_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSI;
        ctx->cur_opcode->insn = kv3_FMULW;
        return trans_v1_FMULW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FIXEDWP;
        return trans_v1_FIXEDWP_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
    case 0x0000000f:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_DPI;
        ctx->cur_opcode->insn = kv3_DOT2W;
        return trans_v1_DOT2W_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FDDI;
        ctx->cur_opcode->insn = kv3_FMULCWC;
        return trans_v1_FMULCWC_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_MAU_FSSI;
        ctx->cur_opcode->insn = kv3_FMULHW;
        return trans_v1_FMULHW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        ctx->cur_opcode->format = kv3_MAU_FDD;
        ctx->cur_opcode->insn = kv3_FIXEDUWP;
        return trans_v1_FIXEDUWP_rounding_silent_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_unsigned6);
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x00000005:
    switch ((codeWord_0 >> 24) & 0x0000000f) {
    case 0x00000000:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDDD0;
          ctx->cur_opcode->insn = kv3_MADDD;
          return trans_v1_MADDD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD0;
          ctx->cur_opcode->insn = kv3_MULWDP;
          return trans_v1_MULWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_QQQ;
          ctx->cur_opcode->insn = kv3_DOT2WDP;
          return trans_v1_DOT2WDP_registerM_registerP_registerO_simple(ctx, &u.kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MADDWD;
          return trans_v1_MADDWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD3;
          ctx->cur_opcode->insn = kv3_MADDHWQ;
          return trans_v1_MADDHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMAD;
        return trans_v1_FFMAD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FADDD;
        return trans_v1_FADDD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQQDD;
          ctx->cur_opcode->insn = kv3_FFMAWDP;
          return trans_v1_FFMAWDP_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQDD;
          ctx->cur_opcode->insn = kv3_FMULWDP;
          return trans_v1_FMULWDP_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDDD0;
          ctx->cur_opcode->insn = kv3_MADDWP;
          return trans_v1_MADDWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD0;
          ctx->cur_opcode->insn = kv3_MULUWDP;
          return trans_v1_MULUWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_QQQ;
          ctx->cur_opcode->insn = kv3_DOT2UWDP;
          return trans_v1_DOT2UWDP_registerM_registerP_registerO_simple(ctx, &u.kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MADDUWD;
          return trans_v1_MADDUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD3;
          ctx->cur_opcode->insn = kv3_MADDUHWQ;
          return trans_v1_MADDUHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMAWD;
        return trans_v1_FFMAWD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FADDWP;
        return trans_v1_FADDWP_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQQDD;
          ctx->cur_opcode->insn = kv3_FFMAHWQ;
          return trans_v1_FFMAHWQ_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQDD;
          ctx->cur_opcode->insn = kv3_FMULHWQ;
          return trans_v1_FMULHWQ_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDDD0;
          ctx->cur_opcode->insn = kv3_MADDHQ;
          return trans_v1_MADDHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD0;
          ctx->cur_opcode->insn = kv3_MULSUWDP;
          return trans_v1_MULSUWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_QQQ;
          ctx->cur_opcode->insn = kv3_DOT2SUWDP;
          return trans_v1_DOT2SUWDP_registerM_registerP_registerO_simple(ctx, &u.kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MADDSUWD;
          return trans_v1_MADDSUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD3;
          ctx->cur_opcode->insn = kv3_MADDSUHWQ;
          return trans_v1_MADDSUHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMAWP;
        return trans_v1_FFMAWP_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FADDHQ;
        return trans_v1_FADDHQ_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQQDD;
          ctx->cur_opcode->insn = kv3_FFMSWDP;
          return trans_v1_FFMSWDP_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQDD;
          ctx->cur_opcode->insn = kv3_FMULWDC;
          return trans_v1_FMULWDC_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000001:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD0;
          ctx->cur_opcode->insn = kv3_MM212W;
          return trans_v1_MM212W_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_QQQ;
          ctx->cur_opcode->insn = kv3_DOT2WZP;
          return trans_v1_DOT2WZP_registerM_registerP_registerO_simple(ctx, &u.kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MADDW;
          return trans_v1_MADDW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMAHQ;
        return trans_v1_FFMAHQ_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FADDCWC;
        return trans_v1_FADDCWC_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQQDD;
          ctx->cur_opcode->insn = kv3_FFMSHWQ;
          return trans_v1_FFMSHWQ_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQDD;
          ctx->cur_opcode->insn = kv3_FMULCWDC;
          return trans_v1_FMULCWDC_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000004:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDDD0;
          ctx->cur_opcode->insn = kv3_MSBFD;
          return trans_v1_MSBFD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDD0;
          ctx->cur_opcode->insn = kv3_MULD;
          return trans_v1_MULD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_QQQ;
          ctx->cur_opcode->insn = kv3_MULWQ;
          return trans_v1_MULWQ_registerM_registerP_registerO_simple(ctx, &u.kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MSBFWD;
          return trans_v1_MSBFWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD3;
          ctx->cur_opcode->insn = kv3_MSBFHWQ;
          return trans_v1_MSBFHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMSD;
        return trans_v1_FFMSD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FSBFD;
        return trans_v1_FSBFD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQQDD;
          ctx->cur_opcode->insn = kv3_FMMA212W;
          return trans_v1_FMMA212W_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQDD;
          ctx->cur_opcode->insn = kv3_FMM212W;
          return trans_v1_FMM212W_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000005:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDDD0;
          ctx->cur_opcode->insn = kv3_MSBFWP;
          return trans_v1_MSBFWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDD0;
          ctx->cur_opcode->insn = kv3_MULWP;
          return trans_v1_MULWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDD1;
          ctx->cur_opcode->insn = kv3_MULCWC;
          return trans_v1_MULCWC_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MSBFUWD;
          return trans_v1_MSBFUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD3;
          ctx->cur_opcode->insn = kv3_MSBFUHWQ;
          return trans_v1_MSBFUHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMSWD;
        return trans_v1_FFMSWD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FSBFWP;
        return trans_v1_FSBFWP_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      default:
        break;
      }
      return false;
    case 0x00000006:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDDD0;
          ctx->cur_opcode->insn = kv3_MSBFHQ;
          return trans_v1_MSBFHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDD0;
          ctx->cur_opcode->insn = kv3_MULHQ;
          return trans_v1_MULHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD2;
          ctx->cur_opcode->insn = kv3_MULWDC;
          return trans_v1_MULWDC_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MSBFSUWD;
          return trans_v1_MSBFSUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD3;
          ctx->cur_opcode->insn = kv3_MSBFSUHWQ;
          return trans_v1_MSBFSUHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMSWP;
        return trans_v1_FFMSWP_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FSBFHQ;
        return trans_v1_FSBFHQ_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_FQQDD;
          ctx->cur_opcode->insn = kv3_FMMS212W;
          return trans_v1_FMMS212W_rounding_silent_registerM_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000007:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DDD0;
          ctx->cur_opcode->insn = kv3_MULWC;
          return trans_v1_MULWC_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD2;
          ctx->cur_opcode->insn = kv3_MULCWDC;
          return trans_v1_MULCWDC_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSSS;
          ctx->cur_opcode->insn = kv3_MSBFW;
          return trans_v1_MSBFW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDDD;
        ctx->cur_opcode->insn = kv3_FFMSHQ;
        return trans_v1_FFMSHQ_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD1;
        ctx->cur_opcode->insn = kv3_FSBFCWC;
        return trans_v1_FSBFCWC_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      default:
        break;
      }
      return false;
    case 0x00000008:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MADDWDP;
          return trans_v1_MADDWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MADDDT;
            return trans_v1_MADDDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QDD1;
            ctx->cur_opcode->insn = kv3_MULDT;
            return trans_v1_MULDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_CRC;
          ctx->cur_opcode->insn = kv3_CRCBELMW;
          return trans_v1_CRCBELMW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSS;
          ctx->cur_opcode->insn = kv3_MULWD;
          return trans_v1_MULWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD3;
          ctx->cur_opcode->insn = kv3_MULHWQ;
          return trans_v1_MULHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FMULD;
        return trans_v1_FMULD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSSS;
        ctx->cur_opcode->insn = kv3_FFMAHW;
        return trans_v1_FFMAHW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ1;
          ctx->cur_opcode->insn = kv3_FADDWQ;
          return trans_v1_FADDWQ_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x00000009:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MADDUWDP;
          return trans_v1_MADDUWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MADDUDT;
            return trans_v1_MADDUDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QDD1;
            ctx->cur_opcode->insn = kv3_MULUDT;
            return trans_v1_MULUDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_CRC;
          ctx->cur_opcode->insn = kv3_CRCBELLW;
          return trans_v1_CRCBELLW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSS;
          ctx->cur_opcode->insn = kv3_MULUWD;
          return trans_v1_MULUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD3;
          ctx->cur_opcode->insn = kv3_MULUHWQ;
          return trans_v1_MULUHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FMULWD;
        return trans_v1_FMULWD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSSS;
        ctx->cur_opcode->insn = kv3_FFMAW;
        return trans_v1_FFMAW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ1;
          ctx->cur_opcode->insn = kv3_FADDCWCP;
          return trans_v1_FADDCWCP_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x0000000a:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MADDSUWDP;
          return trans_v1_MADDSUWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MADDSUDT;
            return trans_v1_MADDSUDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QDD1;
            ctx->cur_opcode->insn = kv3_MULSUDT;
            return trans_v1_MULSUDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_CRC;
          ctx->cur_opcode->insn = kv3_CRCLELMW;
          return trans_v1_CRCLELMW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSS;
          ctx->cur_opcode->insn = kv3_MULSUWD;
          return trans_v1_MULSUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QDD3;
          ctx->cur_opcode->insn = kv3_MULSUHWQ;
          return trans_v1_MULSUHWQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FMULWP;
        return trans_v1_FMULWP_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSSS;
        ctx->cur_opcode->insn = kv3_FFMSHW;
        return trans_v1_FFMSHW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ1;
          ctx->cur_opcode->insn = kv3_FSBFWQ;
          return trans_v1_FSBFWQ_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x0000000b:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MMA212W;
          return trans_v1_MMA212W_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MADDUZDT;
            return trans_v1_MADDUZDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QDD1;
            ctx->cur_opcode->insn = kv3_CMULDT;
            return trans_v1_CMULDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_CRC;
          ctx->cur_opcode->insn = kv3_CRCLELLW;
          return trans_v1_CRCLELLW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_SSS;
          ctx->cur_opcode->insn = kv3_MULW;
          return trans_v1_MULW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FMULHQ;
        return trans_v1_FMULHQ_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSSS;
        ctx->cur_opcode->insn = kv3_FFMSW;
        return trans_v1_FFMSW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ1;
          ctx->cur_opcode->insn = kv3_FSBFCWCP;
          return trans_v1_FSBFCWCP_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x0000000c:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MSBFWDP;
          return trans_v1_MSBFWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MSBFDT;
            return trans_v1_MSBFDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD2;
            ctx->cur_opcode->insn = kv3_CMULXDT;
            return trans_v1_CMULXDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DPP;
          ctx->cur_opcode->insn = kv3_DOT2WD;
          return trans_v1_DOT2WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FDOT2W;
        return trans_v1_FDOT2W_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSS;
        ctx->cur_opcode->insn = kv3_FADDW;
        return trans_v1_FADDW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ0;
          ctx->cur_opcode->insn = kv3_FADDDP;
          return trans_v1_FADDDP_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ1;
          ctx->cur_opcode->insn = kv3_FDOT2WDP;
          return trans_v1_FDOT2WDP_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x0000000d:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MSBFUWDP;
          return trans_v1_MSBFUWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MSBFUDT;
            return trans_v1_MSBFUDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD2;
            ctx->cur_opcode->insn = kv3_CMULGLXDT;
            return trans_v1_CMULGLXDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DPP;
          ctx->cur_opcode->insn = kv3_DOT2UWD;
          return trans_v1_DOT2UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FDOT2WD;
        return trans_v1_FDOT2WD_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSS;
        ctx->cur_opcode->insn = kv3_FSBFW;
        return trans_v1_FSBFW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ0;
          ctx->cur_opcode->insn = kv3_FADDCDC;
          return trans_v1_FADDCDC_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ1;
          ctx->cur_opcode->insn = kv3_FDOT2WZP;
          return trans_v1_FDOT2WZP_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x0000000e:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MSBFSUWDP;
          return trans_v1_MSBFSUWDP_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MSBFSUDT;
            return trans_v1_MSBFSUDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD2;
            ctx->cur_opcode->insn = kv3_CMULGMXDT;
            return trans_v1_CMULGMXDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DPP;
          ctx->cur_opcode->insn = kv3_DOT2SUWD;
          return trans_v1_DOT2SUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FMULWC;
        return trans_v1_FMULWC_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSS;
        ctx->cur_opcode->insn = kv3_FMULW;
        return trans_v1_FMULW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ0;
          ctx->cur_opcode->insn = kv3_FSBFDP;
          return trans_v1_FSBFDP_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        case 0x00000001:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ1;
          ctx->cur_opcode->insn = kv3_FMULWQ;
          return trans_v1_FMULWQ_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          break;
        }
        return false;
        break;
      }
      return false;
    case 0x0000000f:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 12) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_QQDD;
          ctx->cur_opcode->insn = kv3_MMS212W;
          return trans_v1_MMS212W_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          switch ((codeWord_0 >> 18) & 0x00000001) {
          case 0x00000000:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD1;
            ctx->cur_opcode->insn = kv3_MSBFUZDT;
            return trans_v1_MSBFUZDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          case 0x00000001:
            decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            ctx->cur_opcode->format = kv3_MAU_QQDD2;
            ctx->cur_opcode->insn = kv3_CMULGHXDT;
            return trans_v1_CMULGHXDT_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
            break;
          }
          return false;
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_DPP;
          ctx->cur_opcode->insn = kv3_DOT2W;
          return trans_v1_DOT2W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerM_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_MAU_COPYQ;
          ctx->cur_opcode->insn = kv3_COPYQ;
          return trans_v1_COPYQ_registerM_registerZ_registerY_simple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000001:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FDDD;
        ctx->cur_opcode->insn = kv3_FMULCWC;
        return trans_v1_FMULCWC_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000002:
        decode_kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
        ctx->cur_opcode->format = kv3_MAU_FSSS;
        ctx->cur_opcode->insn = kv3_FMULHW;
        return trans_v1_FMULHW_rounding_silent_registerW_registerZ_registerY_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerW_kv3_registerZ_kv3_registerY);
      case 0x00000003:
        switch ((codeWord_0 >> 18) & 0x00000001) {
        case 0x00000000:
          decode_kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO(ctx, codeWords, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
          ctx->cur_opcode->format = kv3_MAU_FQQQ0;
          ctx->cur_opcode->insn = kv3_FSBFCDC;
          return trans_v1_FSBFCDC_rounding_silent_registerM_registerP_registerO_simple(ctx, &u.kv3_rounding_kv3_silent_kv3_registerM_kv3_registerP_kv3_registerO);
        default:
          break;
        }
        return false;
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x00000006:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x00000000:
        decode_kv3_registerW_kv3_signed16(ctx, codeWords, &u.kv3_registerW_kv3_signed16);
        ctx->cur_opcode->format = kv3_ALU_WI;
        ctx->cur_opcode->insn = kv3_MAKE;
        return trans_v1_MAKE_registerW_signed16_simple(ctx, &u.kv3_registerW_kv3_signed16);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_ADDD;
        return trans_v1_ADDD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_MIND;
        return trans_v1_MIND_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_MAXD;
        return trans_v1_MAXD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000004:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_ABSD;
          return trans_v1_ABSD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_ABDD;
          return trans_v1_ABDD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_NEGD;
          return trans_v1_NEGD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_SBFD;
          return trans_v1_SBFD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000006:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_MINUD;
        return trans_v1_MINUD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000007:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_MAXUD;
        return trans_v1_MAXUD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000008:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_ANDD;
        return trans_v1_ANDD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000009:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_NANDD;
        return trans_v1_NANDD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000a:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_COPYD;
          return trans_v1_COPYD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_ORD;
          return trans_v1_ORD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x0000000b:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_NORD;
        return trans_v1_NORD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000c:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x000003ff:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_NOTD;
          return trans_v1_NOTD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRI;
          ctx->cur_opcode->insn = kv3_XORD;
          return trans_v1_XORD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x0000000d:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_NXORD;
        return trans_v1_NXORD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000e:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_ANDND;
        return trans_v1_ANDND_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000f:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRI;
        ctx->cur_opcode->insn = kv3_ORND;
        return trans_v1_ORND_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x0000000e:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIS;
        ctx->cur_opcode->insn = kv3_ADDSD;
        return trans_v1_ADDSD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000f:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIS;
        ctx->cur_opcode->insn = kv3_SBFSD;
        return trans_v1_SBFSD_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      default:
        decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_CWRI;
        ctx->cur_opcode->insn = kv3_COMPD;
        return trans_v1_COMPD_comparison_registerW_registerZ_signed10_simple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_signed10);
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x0000000e:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_BWRI;
        ctx->cur_opcode->insn = kv3_SBMM8;
        return trans_v1_SBMM8_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000f:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_BWRI;
        ctx->cur_opcode->insn = kv3_SBMMT8;
        return trans_v1_SBMMT8_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      default:
        decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_signed10(ctx, codeWords, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_MWRI;
        ctx->cur_opcode->insn = kv3_CMOVED;
        return trans_v1_CMOVED_scalarcond_registerZ_registerW_signed10_simple(ctx, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_signed10);
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 26) & 0x00000003) {
      case 0x00000000:
        decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
        ctx->cur_opcode->format = kv3_ALU_WRB;
        ctx->cur_opcode->insn = kv3_INSF;
        return trans_v1_INSF_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
      case 0x00000001:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x000001c0:
          switch ((codeWord_0 >> 24) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_ZXBD;
            return trans_v1_ZXBD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          default:
            decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_EXTFZ;
            return trans_v1_EXTFZ_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            break;
          }
          return false;
        case 0x000003c0:
          switch ((codeWord_0 >> 24) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_ZXHD;
            return trans_v1_ZXHD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          case 0x00000001:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_ZXWD;
            return trans_v1_ZXWD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          default:
            decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_EXTFZ;
            return trans_v1_EXTFZ_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            break;
          }
          return false;
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
          ctx->cur_opcode->format = kv3_ALU_WRB;
          ctx->cur_opcode->insn = kv3_EXTFZ;
          return trans_v1_EXTFZ_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x000001c0:
          switch ((codeWord_0 >> 24) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_SXBD;
            return trans_v1_SXBD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          default:
            decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_EXTFS;
            return trans_v1_EXTFS_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            break;
          }
          return false;
        case 0x000003c0:
          switch ((codeWord_0 >> 24) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_SXHD;
            return trans_v1_SXHD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          case 0x00000001:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_SXWD;
            return trans_v1_SXWD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          default:
            decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            ctx->cur_opcode->format = kv3_ALU_WRB;
            ctx->cur_opcode->insn = kv3_EXTFS;
            return trans_v1_EXTFS_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
            break;
          }
          return false;
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
          ctx->cur_opcode->format = kv3_ALU_WRB;
          ctx->cur_opcode->insn = kv3_EXTFS;
          return trans_v1_EXTFS_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
          break;
        }
        return false;
      case 0x00000003:
        decode_kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
        ctx->cur_opcode->format = kv3_ALU_WRB;
        ctx->cur_opcode->insn = kv3_CLRF;
        return trans_v1_CLRF_registerW_registerZ_stopbit2_stopbit4_startbit_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_stopbit2_stopbit4_kv3_startbit);
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x00000007:
    switch ((codeWord_0 >> 16) & 0x00000003) {
    case 0x00000000:
      switch ((codeWord_0 >> 24) & 0x0000000f) {
      case 0x00000000:
        decode_kv3_registerW_kv3_signed16(ctx, codeWords, &u.kv3_registerW_kv3_signed16);
        ctx->cur_opcode->format = kv3_ALU_WIPC;
        ctx->cur_opcode->insn = kv3_PCREL;
        return trans_v1_PCREL_registerW_signed16_simple(ctx, &u.kv3_registerW_kv3_signed16);
      case 0x00000001:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_ADDW;
        return trans_v1_ADDW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000002:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_MINW;
        return trans_v1_MINW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000003:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_MAXW;
        return trans_v1_MAXW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000004:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_ABSW;
          return trans_v1_ABSW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_ABDW;
          return trans_v1_ABDW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_NEGW;
          return trans_v1_NEGW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_SBFW;
          return trans_v1_SBFW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x00000006:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_MINUW;
        return trans_v1_MINUW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000007:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_MAXUW;
        return trans_v1_MAXUW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000008:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_ANDW;
        return trans_v1_ANDW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x00000009:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_NANDW;
        return trans_v1_NANDW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000a:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_COPYW;
          return trans_v1_COPYW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_ORW;
          return trans_v1_ORW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x0000000b:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_NORW;
        return trans_v1_NORW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000c:
        switch ((codeWord_0 >> 6) & 0x000003ff) {
        case 0x000003ff:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_NOTW;
          return trans_v1_NOTW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          ctx->cur_opcode->format = kv3_ALU_WRIW;
          ctx->cur_opcode->insn = kv3_XORW;
          return trans_v1_XORW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
          break;
        }
        return false;
      case 0x0000000d:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_NXORW;
        return trans_v1_NXORW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000e:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_ANDNW;
        return trans_v1_ANDNW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
      case 0x0000000f:
        decode_kv3_registerW_kv3_registerZ_kv3_signed10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        ctx->cur_opcode->format = kv3_ALU_WRIW;
        ctx->cur_opcode->insn = kv3_ORNW;
        return trans_v1_ORNW_registerW_registerZ_signed10_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_signed10);
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_0 >> 12) & 0x0000000f) {
      case 0x00000000:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_STSUD;
          return trans_v1_STSUD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_ADDD;
          return trans_v1_ADDD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_MIND;
          return trans_v1_MIND_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_MAXD;
          return trans_v1_MAXD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_ABDD;
          return trans_v1_ABDD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_SBFD;
          return trans_v1_SBFD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_MINUD;
          return trans_v1_MINUD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_MAXUD;
          return trans_v1_MAXUD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_ANDD;
          return trans_v1_ANDD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_NANDD;
          return trans_v1_NANDD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_ORD;
          return trans_v1_ORD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_NORD;
          return trans_v1_NORD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_XORD;
          return trans_v1_XORD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_NXORD;
          return trans_v1_NXORD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_ANDND;
          return trans_v1_ANDND_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR0;
          ctx->cur_opcode->insn = kv3_ORND;
          return trans_v1_ORND_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_STSUW;
          return trans_v1_STSUW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_ADDW;
          return trans_v1_ADDW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_MINW;
          return trans_v1_MINW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_MAXW;
          return trans_v1_MAXW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_ABDW;
          return trans_v1_ABDW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_SBFW;
          return trans_v1_SBFW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_MINUW;
          return trans_v1_MINUW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_MAXUW;
          return trans_v1_MAXUW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_ANDW;
          return trans_v1_ANDW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_NANDW;
          return trans_v1_NANDW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_ORW;
          return trans_v1_ORW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_NORW;
          return trans_v1_NORW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_XORW;
          return trans_v1_XORW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_NXORW;
          return trans_v1_NXORW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_ANDNW;
          return trans_v1_ANDNW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR0;
          ctx->cur_opcode->insn = kv3_ORNW;
          return trans_v1_ORNW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_ADDWP;
          return trans_v1_ADDWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_MINWP;
          return trans_v1_MINWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_MAXWP;
          return trans_v1_MAXWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_ABDWP;
          return trans_v1_ABDWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_SBFWP;
          return trans_v1_SBFWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_MINUWP;
          return trans_v1_MINUWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_MAXUWP;
          return trans_v1_MAXUWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_ADDCWC;
          return trans_v1_ADDCWC_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP0;
          ctx->cur_opcode->insn = kv3_SBFCWC;
          return trans_v1_SBFCWC_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_ADDHQ;
          return trans_v1_ADDHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_MINHQ;
          return trans_v1_MINHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_MAXHQ;
          return trans_v1_MAXHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_ABDHQ;
          return trans_v1_ABDHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_SBFHQ;
          return trans_v1_SBFHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_MINUHQ;
          return trans_v1_MINUHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_MAXUHQ;
          return trans_v1_MAXUHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_ADDCHCP;
          return trans_v1_ADDCHCP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ0;
          ctx->cur_opcode->insn = kv3_SBFCHCP;
          return trans_v1_SBFCHCP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_ADDX2D;
          return trans_v1_ADDX2D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_SBFX2D;
          return trans_v1_SBFX2D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_ADDX4D;
          return trans_v1_ADDX4D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_SBFX4D;
          return trans_v1_SBFX4D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_ADDX8D;
          return trans_v1_ADDX8D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_SBFX8D;
          return trans_v1_SBFX8D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_ADDX16D;
          return trans_v1_ADDX16D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_SBFX16D;
          return trans_v1_SBFX16D_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_ADDWD;
          return trans_v1_ADDWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_SBFWD;
          return trans_v1_SBFWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_ADDUWD;
          return trans_v1_ADDUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_SBFUWD;
          return trans_v1_SBFUWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_LANDD;
          return trans_v1_LANDD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_LNANDD;
          return trans_v1_LNANDD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_LORD;
          return trans_v1_LORD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRR1;
          ctx->cur_opcode->insn = kv3_LNORD;
          return trans_v1_LNORD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_ADDX2W;
          return trans_v1_ADDX2W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_SBFX2W;
          return trans_v1_SBFX2W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_ADDX4W;
          return trans_v1_ADDX4W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_SBFX4W;
          return trans_v1_SBFX4W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_ADDX8W;
          return trans_v1_ADDX8W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_SBFX8W;
          return trans_v1_SBFX8W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_ADDX16W;
          return trans_v1_ADDX16W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_SBFX16W;
          return trans_v1_SBFX16W_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_AVGW;
          return trans_v1_AVGW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_AVGUW;
          return trans_v1_AVGUW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_AVGRW;
          return trans_v1_AVGRW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_AVGRUW;
          return trans_v1_AVGRUW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_LANDW;
          return trans_v1_LANDW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_LNANDW;
          return trans_v1_LNANDW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_LORW;
          return trans_v1_LORW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR1;
          ctx->cur_opcode->insn = kv3_LNORW;
          return trans_v1_LNORW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_ADDX2WP;
          return trans_v1_ADDX2WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_SBFX2WP;
          return trans_v1_SBFX2WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_ADDX4WP;
          return trans_v1_ADDX4WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_SBFX4WP;
          return trans_v1_SBFX4WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_ADDX8WP;
          return trans_v1_ADDX8WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_SBFX8WP;
          return trans_v1_SBFX8WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_ADDX16WP;
          return trans_v1_ADDX16WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_SBFX16WP;
          return trans_v1_SBFX16WP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_AVGWP;
          return trans_v1_AVGWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_AVGUWP;
          return trans_v1_AVGUWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_AVGRWP;
          return trans_v1_AVGRWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_AVGRUWP;
          return trans_v1_AVGRUWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_LANDWP;
          return trans_v1_LANDWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_LNANDWP;
          return trans_v1_LNANDWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_LORWP;
          return trans_v1_LORWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRWP1;
          ctx->cur_opcode->insn = kv3_LNORWP;
          return trans_v1_LNORWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_ADDX2HQ;
          return trans_v1_ADDX2HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_SBFX2HQ;
          return trans_v1_SBFX2HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_ADDX4HQ;
          return trans_v1_ADDX4HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_SBFX4HQ;
          return trans_v1_SBFX4HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_ADDX8HQ;
          return trans_v1_ADDX8HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_SBFX8HQ;
          return trans_v1_SBFX8HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_ADDX16HQ;
          return trans_v1_ADDX16HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_SBFX16HQ;
          return trans_v1_SBFX16HQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_AVGHQ;
          return trans_v1_AVGHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_AVGUHQ;
          return trans_v1_AVGUHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_AVGRHQ;
          return trans_v1_AVGRHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_AVGRUHQ;
          return trans_v1_AVGRUHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_LANDHQ;
          return trans_v1_LANDHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_LNANDHQ;
          return trans_v1_LNANDHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_LORHQ;
          return trans_v1_LORHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRHQ1;
          ctx->cur_opcode->insn = kv3_LNORHQ;
          return trans_v1_LNORHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000008:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX2WD;
          return trans_v1_ADDX2WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX2WD;
          return trans_v1_SBFX2WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX4WD;
          return trans_v1_ADDX4WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX4WD;
          return trans_v1_SBFX4WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX8WD;
          return trans_v1_ADDX8WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX8WD;
          return trans_v1_SBFX8WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX16WD;
          return trans_v1_ADDX16WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX16WD;
          return trans_v1_SBFX16WD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX2UWD;
          return trans_v1_ADDX2UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX2UWD;
          return trans_v1_SBFX2UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX4UWD;
          return trans_v1_ADDX4UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX4UWD;
          return trans_v1_SBFX4UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX8UWD;
          return trans_v1_ADDX8UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000d:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX8UWD;
          return trans_v1_SBFX8UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_ADDX16UWD;
          return trans_v1_ADDX16UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DWRR2;
          ctx->cur_opcode->insn = kv3_SBFX16UWD;
          return trans_v1_SBFX16UWD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x0000000a:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRS;
          ctx->cur_opcode->insn = kv3_ADDSD;
          return trans_v1_ADDSD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_WRRS;
          ctx->cur_opcode->insn = kv3_SBFSD;
          return trans_v1_SBFSD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_CWRR;
          ctx->cur_opcode->insn = kv3_COMPD;
          return trans_v1_COMPD_comparison_registerW_registerZ_registerY_simple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x0000000b:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_AWRRW;
          ctx->cur_opcode->insn = kv3_ADDSW;
          return trans_v1_ADDSW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_AWRRW;
          ctx->cur_opcode->insn = kv3_SBFSW;
          return trans_v1_SBFSW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DCWRR;
          ctx->cur_opcode->insn = kv3_COMPW;
          return trans_v1_COMPW_comparison_registerW_registerZ_registerY_simple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x0000000c:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DCWRRC;
          ctx->cur_opcode->insn = kv3_ADDD_C;
          return trans_v1_ADDD_C_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DCWRRC;
          ctx->cur_opcode->insn = kv3_SBFD_C;
          return trans_v1_SBFD_C_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DCWRR;
          ctx->cur_opcode->insn = kv3_COMPWD;
          return trans_v1_COMPWD_comparison_registerW_registerZ_registerY_simple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x0000000d:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DCWRRCI;
          ctx->cur_opcode->insn = kv3_ADDD_CI;
          return trans_v1_ADDD_CI_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DCWRRCI;
          ctx->cur_opcode->insn = kv3_SBFD_CI;
          return trans_v1_SBFD_CI_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DCWRR;
          ctx->cur_opcode->insn = kv3_COMPUWD;
          return trans_v1_COMPUWD_comparison_registerW_registerZ_registerY_simple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x0000000e:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_AWRRWP;
          ctx->cur_opcode->insn = kv3_ADDSWP;
          return trans_v1_ADDSWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_AWRRWP;
          ctx->cur_opcode->insn = kv3_SBFSWP;
          return trans_v1_SBFSWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_CWRRWP;
          ctx->cur_opcode->insn = kv3_COMPNWP;
          return trans_v1_COMPNWP_comparison_registerW_registerZ_registerY_simple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_AWRRHQ;
          ctx->cur_opcode->insn = kv3_ADDSHQ;
          return trans_v1_ADDSHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_AWRRHQ;
          ctx->cur_opcode->insn = kv3_SBFSHQ;
          return trans_v1_SBFSHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_CWRRHQ;
          ctx->cur_opcode->insn = kv3_COMPNHQ;
          return trans_v1_COMPNHQ_comparison_registerW_registerZ_registerY_simple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_0 >> 12) & 0x0000000f) {
      case 0x00000000:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_BWRR;
          ctx->cur_opcode->insn = kv3_SBMM8;
          return trans_v1_SBMM8_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_BWRR;
          ctx->cur_opcode->insn = kv3_SBMMT8;
          return trans_v1_SBMMT8_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_registerY(ctx, codeWords, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_MWRR0;
          ctx->cur_opcode->insn = kv3_CMOVED;
          return trans_v1_CMOVED_scalarcond_registerZ_registerW_registerY_simple(ctx, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_registerY);
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 27) & 0x00000001) {
        case 0x00000000:
          decode_kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY(ctx, codeWords, &u.kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_MWRRWP;
          ctx->cur_opcode->insn = kv3_CMOVEWP;
          return trans_v1_CMOVEWP_simdcond_registerZ_registerW_registerY_simple(ctx, &u.kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY);
        case 0x00000001:
          decode_kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY(ctx, codeWords, &u.kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_MWRRHQ;
          ctx->cur_opcode->insn = kv3_CMOVEHQ;
          return trans_v1_CMOVEHQ_simdcond_registerZ_registerW_registerY_simple(ctx, &u.kv3_simdcond_kv3_registerZ_kv3_registerW_kv3_registerY);
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DBWR;
          ctx->cur_opcode->insn = kv3_CLZD;
          return trans_v1_CLZD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DBWR;
          ctx->cur_opcode->insn = kv3_CLSD;
          return trans_v1_CLSD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DBWR;
          ctx->cur_opcode->insn = kv3_CBSD;
          return trans_v1_CBSD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DBWR;
          ctx->cur_opcode->insn = kv3_CTZD;
          return trans_v1_CTZD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRI;
          ctx->cur_opcode->insn = kv3_SRSD;
          return trans_v1_SRSD_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRI;
          ctx->cur_opcode->insn = kv3_SLLD;
          return trans_v1_SLLD_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRI;
          ctx->cur_opcode->insn = kv3_SRAD;
          return trans_v1_SRAD_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRI;
          ctx->cur_opcode->insn = kv3_SRLD;
          return trans_v1_SRLD_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRI;
          ctx->cur_opcode->insn = kv3_SLSD;
          return trans_v1_SLSD_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000e:
          switch ((codeWord_0 >> 6) & 0x0000003f) {
          case 0x00000010:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_DSWRI;
            ctx->cur_opcode->insn = kv3_SATDH;
            return trans_v1_SATDH_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          case 0x00000020:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_DSWRI;
            ctx->cur_opcode->insn = kv3_SATDW;
            return trans_v1_SATDW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          default:
            decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
            ctx->cur_opcode->format = kv3_ALU_DSWRI;
            ctx->cur_opcode->insn = kv3_SATD;
            return trans_v1_SATD_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_0 >> 6) & 0x0000003f) {
          case 0x00000010:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_DSWRI;
            ctx->cur_opcode->insn = kv3_SATUDH;
            return trans_v1_SATUDH_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          case 0x00000020:
            decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
            ctx->cur_opcode->format = kv3_ALU_DSWRI;
            ctx->cur_opcode->insn = kv3_SATUDW;
            return trans_v1_SATUDW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
          default:
            decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
            ctx->cur_opcode->format = kv3_ALU_DSWRI;
            ctx->cur_opcode->insn = kv3_SATUD;
            return trans_v1_SATUD_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWR;
          ctx->cur_opcode->insn = kv3_CLZW;
          return trans_v1_CLZW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWR;
          ctx->cur_opcode->insn = kv3_CLSW;
          return trans_v1_CLSW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWR;
          ctx->cur_opcode->insn = kv3_CBSW;
          return trans_v1_CBSW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWR;
          ctx->cur_opcode->insn = kv3_CTZW;
          return trans_v1_CTZW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRI;
          ctx->cur_opcode->insn = kv3_SRSW;
          return trans_v1_SRSW_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRI;
          ctx->cur_opcode->insn = kv3_SLLW;
          return trans_v1_SLLW_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRI;
          ctx->cur_opcode->insn = kv3_SRAW;
          return trans_v1_SRAW_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRI;
          ctx->cur_opcode->insn = kv3_SRLW;
          return trans_v1_SRLW_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRI;
          ctx->cur_opcode->insn = kv3_SLSW;
          return trans_v1_SLSW_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRI;
          ctx->cur_opcode->insn = kv3_ROLW;
          return trans_v1_ROLW_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRI;
          ctx->cur_opcode->insn = kv3_RORW;
          return trans_v1_RORW_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWRWP;
          ctx->cur_opcode->insn = kv3_CLZWP;
          return trans_v1_CLZWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWRWP;
          ctx->cur_opcode->insn = kv3_CLSWP;
          return trans_v1_CLSWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWRWP;
          ctx->cur_opcode->insn = kv3_CBSWP;
          return trans_v1_CBSWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWRWP;
          ctx->cur_opcode->insn = kv3_CTZWP;
          return trans_v1_CTZWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWRWP;
          ctx->cur_opcode->insn = kv3_SXLHWP;
          return trans_v1_SXLHWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_BWRWP;
          ctx->cur_opcode->insn = kv3_SXMHWP;
          return trans_v1_SXMHWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRIWP;
          ctx->cur_opcode->insn = kv3_SRSWPS;
          return trans_v1_SRSWPS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRIWP;
          ctx->cur_opcode->insn = kv3_SLLWPS;
          return trans_v1_SLLWPS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRIWP;
          ctx->cur_opcode->insn = kv3_SRAWPS;
          return trans_v1_SRAWPS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRIWP;
          ctx->cur_opcode->insn = kv3_SRLWPS;
          return trans_v1_SRLWPS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRIWP;
          ctx->cur_opcode->insn = kv3_SLSWPS;
          return trans_v1_SLSWPS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRIWP;
          ctx->cur_opcode->insn = kv3_ROLWPS;
          return trans_v1_ROLWPS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_DSWRIWP;
          ctx->cur_opcode->insn = kv3_RORWPS;
          return trans_v1_RORWPS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        default:
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_AWRHQ;
          ctx->cur_opcode->insn = kv3_SXLBHQ;
          return trans_v1_SXLBHQ_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_AWRHQ;
          ctx->cur_opcode->insn = kv3_SXMBHQ;
          return trans_v1_SXMBHQ_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRIHQ;
          ctx->cur_opcode->insn = kv3_SRSHQS;
          return trans_v1_SRSHQS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRIHQ;
          ctx->cur_opcode->insn = kv3_SLLHQS;
          return trans_v1_SLLHQS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRIHQ;
          ctx->cur_opcode->insn = kv3_SRAHQS;
          return trans_v1_SRAHQS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRIHQ;
          ctx->cur_opcode->insn = kv3_SRLHQS;
          return trans_v1_SRLHQS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_unsigned6(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
          ctx->cur_opcode->format = kv3_ALU_SWRIHQ;
          ctx->cur_opcode->insn = kv3_SLSHQS;
          return trans_v1_SLSHQS_registerW_registerZ_unsigned6_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_unsigned6);
        default:
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRR;
          ctx->cur_opcode->insn = kv3_SRSD;
          return trans_v1_SRSD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRR;
          ctx->cur_opcode->insn = kv3_SLLD;
          return trans_v1_SLLD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRR;
          ctx->cur_opcode->insn = kv3_SRAD;
          return trans_v1_SRAD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRR;
          ctx->cur_opcode->insn = kv3_SRLD;
          return trans_v1_SRLD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRR;
          ctx->cur_opcode->insn = kv3_SLSD;
          return trans_v1_SLSD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRR;
          ctx->cur_opcode->insn = kv3_SATD;
          return trans_v1_SATD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRR;
          ctx->cur_opcode->insn = kv3_SATUD;
          return trans_v1_SATUD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000007:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRR;
          ctx->cur_opcode->insn = kv3_SRSW;
          return trans_v1_SRSW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRR;
          ctx->cur_opcode->insn = kv3_SLLW;
          return trans_v1_SLLW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRR;
          ctx->cur_opcode->insn = kv3_SRAW;
          return trans_v1_SRAW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRR;
          ctx->cur_opcode->insn = kv3_SRLW;
          return trans_v1_SRLW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRR;
          ctx->cur_opcode->insn = kv3_SLSW;
          return trans_v1_SLSW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRR;
          ctx->cur_opcode->insn = kv3_ROLW;
          return trans_v1_ROLW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRR;
          ctx->cur_opcode->insn = kv3_RORW;
          return trans_v1_RORW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000008:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRRWP;
          ctx->cur_opcode->insn = kv3_SRSWPS;
          return trans_v1_SRSWPS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRRWP;
          ctx->cur_opcode->insn = kv3_SLLWPS;
          return trans_v1_SLLWPS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRRWP;
          ctx->cur_opcode->insn = kv3_SRAWPS;
          return trans_v1_SRAWPS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRRWP;
          ctx->cur_opcode->insn = kv3_SRLWPS;
          return trans_v1_SRLWPS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRRWP;
          ctx->cur_opcode->insn = kv3_SLSWPS;
          return trans_v1_SLSWPS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000e:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRRWP;
          ctx->cur_opcode->insn = kv3_ROLWPS;
          return trans_v1_ROLWPS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000f:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DSWRRWP;
          ctx->cur_opcode->insn = kv3_RORWPS;
          return trans_v1_RORWPS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x00000009:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRRHQ;
          ctx->cur_opcode->insn = kv3_SRSHQS;
          return trans_v1_SRSHQS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000009:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRRHQ;
          ctx->cur_opcode->insn = kv3_SLLHQS;
          return trans_v1_SLLHQS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRRHQ;
          ctx->cur_opcode->insn = kv3_SRAHQS;
          return trans_v1_SRAHQS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000b:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRRHQ;
          ctx->cur_opcode->insn = kv3_SRLHQS;
          return trans_v1_SRLHQS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_SWRRHQ;
          ctx->cur_opcode->insn = kv3_SLSHQS;
          return trans_v1_SLSHQS_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x0000000e:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000f:
          decode_kv3_registerAl_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerAl_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_MOVETQE;
          ctx->cur_opcode->insn = kv3_MOVETQ;
          return trans_v1_MOVETQ_registerAl_registerZ_registerY_simple(ctx, &u.kv3_registerAl_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000f:
          decode_kv3_registerAh_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerAh_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_MOVETQO;
          ctx->cur_opcode->insn = kv3_MOVETQ;
          return trans_v1_MOVETQ_registerAh_registerZ_registerY_simple(ctx, &u.kv3_registerAh_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_0 >> 12) & 0x0000000f) {
      case 0x00000000:
        switch ((codeWord_0 >> 27) & 0x00000001) {
        case 0x00000000:
          decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_FCWRRS;
          ctx->cur_opcode->insn = kv3_FCOMPW;
          return trans_v1_FCOMPW_floatcomp_registerW_registerZ_registerY_simple(ctx, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_FCWRRS;
          ctx->cur_opcode->insn = kv3_FCOMPD;
          return trans_v1_FCOMPD_floatcomp_registerW_registerZ_registerY_simple(ctx, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 27) & 0x00000001) {
        case 0x00000000:
          decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_FCWRRWP;
          ctx->cur_opcode->insn = kv3_FCOMPNWP;
          return trans_v1_FCOMPNWP_floatcomp_registerW_registerZ_registerY_simple(ctx, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_FCWRRHQ;
          ctx->cur_opcode->insn = kv3_FCOMPNHQ;
          return trans_v1_FCOMPNHQ_floatcomp_registerW_registerZ_registerY_simple(ctx, &u.kv3_floatcomp_kv3_registerW_kv3_registerZ_kv3_registerY);
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FNEGD;
          return trans_v1_FNEGD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FABSD;
          return trans_v1_FABSD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FNEGW;
          return trans_v1_FNEGW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FABSW;
          return trans_v1_FABSW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FNEGWP;
          return trans_v1_FNEGWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FABSWP;
          return trans_v1_FABSWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FNEGHQ;
          return trans_v1_FNEGHQ_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FABSHQ;
          return trans_v1_FABSHQ_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x00000008:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FSISRD;
          return trans_v1_FSISRD_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x0000000a:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FSISRW;
          return trans_v1_FSISRW_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        case 0x0000000c:
          decode_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_DFWR;
          ctx->cur_opcode->insn = kv3_FSISRWP;
          return trans_v1_FSISRWP_registerW_registerZ_simple(ctx, &u.kv3_registerW_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000008:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWR;
          ctx->cur_opcode->insn = kv3_FWIDENLWD;
          return trans_v1_FWIDENLWD_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x00000009:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWR;
          ctx->cur_opcode->insn = kv3_FWIDENMWD;
          return trans_v1_FWIDENMWD_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x0000000a:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWR;
          ctx->cur_opcode->insn = kv3_FWIDENLHW;
          return trans_v1_FWIDENLHW_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x0000000b:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWR;
          ctx->cur_opcode->insn = kv3_FWIDENMHW;
          return trans_v1_FWIDENMHW_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x0000000c:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWR;
          ctx->cur_opcode->insn = kv3_FWIDENLHWP;
          return trans_v1_FWIDENLHWP_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x0000000d:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWR;
          ctx->cur_opcode->insn = kv3_FWIDENMHWP;
          return trans_v1_FWIDENMHWP_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000004:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWRHQ;
          ctx->cur_opcode->insn = kv3_FSINVD;
          return trans_v1_FSINVD_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x00000002:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWRHQ;
          ctx->cur_opcode->insn = kv3_FSINVW;
          return trans_v1_FSINVW_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x00000004:
          decode_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWRHQ;
          ctx->cur_opcode->insn = kv3_FSINVWP;
          return trans_v1_FSINVWP_silent2_registerW_registerZ_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerZ);
        default:
          break;
        }
        return false;
      case 0x00000005:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_DFWRR;
          ctx->cur_opcode->insn = kv3_FSDIVD;
          return trans_v1_FSDIVD_silent2_registerW_registerP_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerP);
        case 0x00000001:
          decode_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_DFWRR;
          ctx->cur_opcode->insn = kv3_FCDIVD;
          return trans_v1_FCDIVD_silent2_registerW_registerP_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerP);
        case 0x00000002:
          decode_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_DFWRR;
          ctx->cur_opcode->insn = kv3_FSDIVW;
          return trans_v1_FSDIVW_silent2_registerW_registerP_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerP);
        case 0x00000003:
          decode_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_DFWRR;
          ctx->cur_opcode->insn = kv3_FCDIVW;
          return trans_v1_FCDIVW_silent2_registerW_registerP_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerP);
        case 0x00000004:
          decode_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_DFWRR;
          ctx->cur_opcode->insn = kv3_FSDIVWP;
          return trans_v1_FSDIVWP_silent2_registerW_registerP_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerP);
        case 0x00000005:
          decode_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_DFWRR;
          ctx->cur_opcode->insn = kv3_FCDIVWP;
          return trans_v1_FCDIVWP_silent2_registerW_registerP_simple(ctx, &u.kv3_silent2_kv3_registerW_kv3_registerP);
        default:
          break;
        }
        return false;
      case 0x00000006:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000002:
          decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWRWP;
          ctx->cur_opcode->insn = kv3_FINVW;
          return trans_v1_FINVW_rounding2_silent2_registerW_registerZ_simple(ctx, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x00000003:
          decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWRWP;
          ctx->cur_opcode->insn = kv3_FISRW;
          return trans_v1_FISRW_rounding2_silent2_registerW_registerZ_simple(ctx, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x00000008:
          decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWRWP;
          ctx->cur_opcode->insn = kv3_FNARROWDW;
          return trans_v1_FNARROWDW_rounding2_silent2_registerW_registerZ_simple(ctx, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x0000000a:
          decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ(ctx, codeWords, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
          ctx->cur_opcode->format = kv3_ALU_FWRWP;
          ctx->cur_opcode->insn = kv3_FNARROWWH;
          return trans_v1_FNARROWWH_rounding2_silent2_registerW_registerZ_simple(ctx, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerZ);
        case 0x0000000c:
          decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_FWRWPN;
          ctx->cur_opcode->insn = kv3_FNARROWDWP;
          return trans_v1_FNARROWDWP_rounding2_silent2_registerW_registerP_simple(ctx, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP);
        case 0x0000000e:
          decode_kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP(ctx, codeWords, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP);
          ctx->cur_opcode->format = kv3_ALU_FWRWPN;
          ctx->cur_opcode->insn = kv3_FNARROWWHQ;
          return trans_v1_FNARROWWHQ_rounding2_silent2_registerW_registerP_simple(ctx, &u.kv3_rounding2_kv3_silent2_kv3_registerW_kv3_registerP);
        default:
          break;
        }
        return false;
      case 0x00000008:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMIND;
          return trans_v1_FMIND_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000001:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMAXD;
          return trans_v1_FMAXD_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000002:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMINW;
          return trans_v1_FMINW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000003:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMAXW;
          return trans_v1_FMAXW_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000004:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMINWP;
          return trans_v1_FMINWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000005:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMAXWP;
          return trans_v1_FMAXWP_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000006:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMINHQ;
          return trans_v1_FMINHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        case 0x00000007:
          decode_kv3_registerW_kv3_registerZ_kv3_registerY(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
          ctx->cur_opcode->format = kv3_ALU_DFWRD;
          ctx->cur_opcode->insn = kv3_FMAXHQ;
          return trans_v1_FMAXHQ_registerW_registerZ_registerY_simple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_registerY);
        default:
          break;
        }
        return false;
      case 0x0000000f:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000f:
          ctx->cur_opcode->format = kv3_ALU_NOP;
          ctx->cur_opcode->insn = kv3_NOP;
          return trans_v1_NOP_simple(ctx, &u.empty);
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
      break;
    }
    return false;
    break;
  }
  return false;
}

bool decode_v1_triple(DisasContext *ctx, const uint32_t *buffer)
{
  const uint32_t *codeWords = (const uint32_t *)buffer;
  union decode_formats u;
  uint32_t codeWord_0 = codeWords[0];
  uint32_t codeWord_1 = codeWords[1];
  uint32_t codeWord_2 = codeWords[2];
  switch ((codeWord_0 >> 28) & 0x0000000f) {
  case 0x0000000a:
    switch ((codeWord_1 >> 29) & 0x00000007) {
    case 0x00000004:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 26) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSBO_Y;
            ctx->cur_opcode->insn = kv3_LBZ;
            return trans_v1_LBZ_variant_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSBO_Y;
            ctx->cur_opcode->insn = kv3_LBS;
            return trans_v1_LBS_variant_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSBO_Y;
            ctx->cur_opcode->insn = kv3_LHZ;
            return trans_v1_LHZ_variant_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSBO_Y;
            ctx->cur_opcode->insn = kv3_LHS;
            return trans_v1_LHS_variant_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_0 >> 25) & 0x00000007) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_speculate_kv3_registerA_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_speculate_kv3_registerA_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LVBO_Y;
            ctx->cur_opcode->insn = kv3_LV;
            return trans_v1_LV_speculate_registerA_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_speculate_kv3_registerA_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_column_kv3_speculate_kv3_registerAq_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_column_kv3_speculate_kv3_registerAq_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LVSBO_Y;
            ctx->cur_opcode->insn = kv3_LV;
            return trans_v1_LV_column_speculate_registerAq_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_column_kv3_speculate_kv3_registerAq_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSBO_Y;
              ctx->cur_opcode->insn = kv3_SB;
              return trans_v1_SB_extend27_upper27_lower10_registerZ_registerT_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
            case 0x00000001:
              decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSBO_Y;
              ctx->cur_opcode->insn = kv3_SH;
              return trans_v1_SH_extend27_upper27_lower10_registerZ_registerT_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSBO_Y;
              ctx->cur_opcode->insn = kv3_SW;
              return trans_v1_SW_extend27_upper27_lower10_registerZ_registerT_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
            case 0x00000001:
              decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSBO_Y;
              ctx->cur_opcode->insn = kv3_SD;
              return trans_v1_SD_extend27_upper27_lower10_registerZ_registerT_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              switch ((codeWord_0 >> 18) & 0x00000001) {
              case 0x00000000:
                decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerU);
                ctx->cur_opcode->format = kv3_LSU_SPBO_Y;
                ctx->cur_opcode->insn = kv3_SQ;
                return trans_v1_SQ_extend27_upper27_lower10_registerZ_registerU_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerU);
              case 0x00000001:
                switch ((codeWord_0 >> 19) & 0x00000001) {
                case 0x00000000:
                  decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerV(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerV);
                  ctx->cur_opcode->format = kv3_LSU_SQBO_Y;
                  ctx->cur_opcode->insn = kv3_SO;
                  return trans_v1_SO_extend27_upper27_lower10_registerZ_registerV_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerV);
                case 0x00000001:
                  switch ((codeWord_0 >> 20) & 0x0000000f) {
                  case 0x00000000:
                    decode_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                    ctx->cur_opcode->format = kv3_LSU_FZBO_Y;
                    ctx->cur_opcode->insn = kv3_DZEROL;
                    return trans_v1_DZEROL_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                  default:
                    break;
                  }
                  return false;
                  break;
                }
                return false;
                break;
              }
              return false;
            case 0x00000001:
              decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerE(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerE);
              ctx->cur_opcode->format = kv3_LSU_SVBO_Y;
              ctx->cur_opcode->insn = kv3_SV;
              return trans_v1_SV_extend27_upper27_lower10_registerZ_registerE_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerE);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_ASBO_Y;
              ctx->cur_opcode->insn = kv3_ALCLRW;
              return trans_v1_ALCLRW_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            case 0x00000001:
              decode_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_ASBO_Y;
              ctx->cur_opcode->insn = kv3_ALCLRD;
              return trans_v1_ALCLRD_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_AABO_Y;
              ctx->cur_opcode->insn = kv3_ALADDW;
              return trans_v1_ALADDW_extend27_upper27_lower10_registerZ_registerT_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
            case 0x00000001:
              decode_kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_AABO_Y;
              ctx->cur_opcode->insn = kv3_ALADDD;
              return trans_v1_ALADDD_extend27_upper27_lower10_registerZ_registerT_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ_kv3_registerT);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 26) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSPB_Y;
            ctx->cur_opcode->insn = kv3_LBZ;
            return trans_v1_LBZ_variant_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSPB_Y;
            ctx->cur_opcode->insn = kv3_LBS;
            return trans_v1_LBS_variant_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSPB_Y;
            ctx->cur_opcode->insn = kv3_LHZ;
            return trans_v1_LHZ_variant_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSPB_Y;
            ctx->cur_opcode->insn = kv3_LHS;
            return trans_v1_LHS_variant_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 25) & 0x00000007) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LVPB_Y;
            ctx->cur_opcode->insn = kv3_LV;
            return trans_v1_LV_speculate_lsucond_registerY_registerA_extend27_offset27_registerZ_triple(ctx, &u.kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerA_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LVSPB_Y;
            ctx->cur_opcode->insn = kv3_LV;
            return trans_v1_LV_column_speculate_lsucond_registerY_registerAq_extend27_offset27_registerZ_triple(ctx, &u.kv3_column_kv3_speculate_kv3_lsucond_kv3_registerY_kv3_registerAq_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSPB_Y;
              ctx->cur_opcode->insn = kv3_SB;
              return trans_v1_SB_lsucond_registerY_extend27_offset27_registerZ_registerT_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
            case 0x00000001:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSPB_Y;
              ctx->cur_opcode->insn = kv3_SH;
              return trans_v1_SH_lsucond_registerY_extend27_offset27_registerZ_registerT_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSPB_Y;
              ctx->cur_opcode->insn = kv3_SW;
              return trans_v1_SW_lsucond_registerY_extend27_offset27_registerZ_registerT_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
            case 0x00000001:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_SSPB_Y;
              ctx->cur_opcode->insn = kv3_SD;
              return trans_v1_SD_lsucond_registerY_extend27_offset27_registerZ_registerT_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              switch ((codeWord_0 >> 18) & 0x00000001) {
              case 0x00000000:
                decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU);
                ctx->cur_opcode->format = kv3_LSU_SPPB_Y;
                ctx->cur_opcode->insn = kv3_SQ;
                return trans_v1_SQ_lsucond_registerY_extend27_offset27_registerZ_registerU_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU);
              case 0x00000001:
                switch ((codeWord_0 >> 19) & 0x00000001) {
                case 0x00000000:
                  decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerV(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerV);
                  ctx->cur_opcode->format = kv3_LSU_SQPB_Y;
                  ctx->cur_opcode->insn = kv3_SO;
                  return trans_v1_SO_lsucond_registerY_extend27_offset27_registerZ_registerV_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerV);
                case 0x00000001:
                  switch ((codeWord_0 >> 20) & 0x0000000f) {
                  case 0x00000000:
                    decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                    ctx->cur_opcode->format = kv3_LSU_FZCB_Y;
                    ctx->cur_opcode->insn = kv3_DZEROL;
                    return trans_v1_DZEROL_lsucond_registerY_extend27_offset27_registerZ_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                  default:
                    break;
                  }
                  return false;
                  break;
                }
                return false;
                break;
              }
              return false;
            case 0x00000001:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerE(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerE);
              ctx->cur_opcode->format = kv3_LSU_SVPB_Y;
              ctx->cur_opcode->insn = kv3_SV;
              return trans_v1_SV_lsucond_registerY_extend27_offset27_registerZ_registerE_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerE);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_ASPB_Y;
              ctx->cur_opcode->insn = kv3_ALCLRW;
              return trans_v1_ALCLRW_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            case 0x00000001:
              decode_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_ASPB_Y;
              ctx->cur_opcode->insn = kv3_ALCLRD;
              return trans_v1_ALCLRD_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 24) & 0x00000001) {
            case 0x00000000:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_AAPB_Y;
              ctx->cur_opcode->insn = kv3_ALADDW;
              return trans_v1_ALADDW_lsucond_registerY_extend27_offset27_registerZ_registerT_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
            case 0x00000001:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              ctx->cur_opcode->format = kv3_LSU_AAPB_Y;
              ctx->cur_opcode->insn = kv3_ALADDD;
              return trans_v1_ALADDD_lsucond_registerY_extend27_offset27_registerZ_registerT_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerT);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
        break;
      }
      return false;
    default:
      break;
    }
    return false;
  case 0x0000000b:
    switch ((codeWord_1 >> 29) & 0x00000007) {
    case 0x00000004:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 26) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSBO_Y;
            ctx->cur_opcode->insn = kv3_LWZ;
            return trans_v1_LWZ_variant_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSBO_Y;
            ctx->cur_opcode->insn = kv3_LWS;
            return trans_v1_LWS_variant_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSBO_Y;
            ctx->cur_opcode->insn = kv3_LD;
            return trans_v1_LD_variant_registerW_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerW_kv3_extend27_upper27_lower10_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_variant_kv3_registerM_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerM_kv3_extend27_upper27_lower10_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_LPBO_Y;
              ctx->cur_opcode->insn = kv3_LQ;
              return trans_v1_LQ_variant_registerM_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerM_kv3_extend27_upper27_lower10_kv3_registerZ);
            case 0x00000001:
              switch ((codeWord_0 >> 19) & 0x00000001) {
              case 0x00000000:
                decode_kv3_variant_kv3_registerN_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_registerN_kv3_extend27_upper27_lower10_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_LQBO_Y;
                ctx->cur_opcode->insn = kv3_LO;
                return trans_v1_LO_variant_registerN_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_variant_kv3_registerN_kv3_extend27_upper27_lower10_kv3_registerZ);
              case 0x00000001:
                switch ((codeWord_0 >> 20) & 0x0000000f) {
                case 0x00000000:
                  decode_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FXBO_Y;
                  ctx->cur_opcode->insn = kv3_DTOUCHL;
                  return trans_v1_DTOUCHL_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                case 0x00000001:
                  decode_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FXBO_Y;
                  ctx->cur_opcode->insn = kv3_DINVALL;
                  return trans_v1_DINVALL_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                case 0x00000005:
                  decode_kv3_extend27_upper27_lower10_kv3_registerZ(ctx, codeWords, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FXBO_Y;
                  ctx->cur_opcode->insn = kv3_IINVALS;
                  return trans_v1_IINVALS_extend27_upper27_lower10_registerZ_triple(ctx, &u.kv3_extend27_upper27_lower10_kv3_registerZ);
                default:
                  break;
                }
                return false;
                break;
              }
              return false;
              break;
            }
            return false;
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_0 >> 26) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSPB_Y;
            ctx->cur_opcode->insn = kv3_LWZ;
            return trans_v1_LWZ_variant_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSPB_Y;
            ctx->cur_opcode->insn = kv3_LWS;
            return trans_v1_LWS_variant_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
            ctx->cur_opcode->format = kv3_LSU_LSPB_Y;
            ctx->cur_opcode->insn = kv3_LD;
            return trans_v1_LD_variant_lsucond_registerY_registerW_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerW_kv3_extend27_offset27_kv3_registerZ);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_extend27_offset27_kv3_registerZ);
              ctx->cur_opcode->format = kv3_LSU_LPPB_Y;
              ctx->cur_opcode->insn = kv3_LQ;
              return trans_v1_LQ_variant_lsucond_registerY_registerM_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerM_kv3_extend27_offset27_kv3_registerZ);
            case 0x00000001:
              switch ((codeWord_0 >> 19) & 0x00000001) {
              case 0x00000000:
                decode_kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_extend27_offset27_kv3_registerZ);
                ctx->cur_opcode->format = kv3_LSU_LPPB1_Y;
                ctx->cur_opcode->insn = kv3_LO;
                return trans_v1_LO_variant_lsucond_registerY_registerN_extend27_offset27_registerZ_triple(ctx, &u.kv3_variant_kv3_lsucond_kv3_registerY_kv3_registerN_kv3_extend27_offset27_kv3_registerZ);
              case 0x00000001:
                switch ((codeWord_0 >> 20) & 0x0000000f) {
                case 0x00000000:
                  decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FXCB_Y;
                  ctx->cur_opcode->insn = kv3_DTOUCHL;
                  return trans_v1_DTOUCHL_lsucond_registerY_extend27_offset27_registerZ_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                case 0x00000001:
                  decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FXCB_Y;
                  ctx->cur_opcode->insn = kv3_DINVALL;
                  return trans_v1_DINVALL_lsucond_registerY_extend27_offset27_registerZ_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                case 0x00000005:
                  decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                  ctx->cur_opcode->format = kv3_LSU_FXCB_Y;
                  ctx->cur_opcode->insn = kv3_IINVALS;
                  return trans_v1_IINVALS_lsucond_registerY_extend27_offset27_registerZ_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ);
                default:
                  break;
                }
                return false;
                break;
              }
              return false;
              break;
            }
            return false;
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000003:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x0000000e:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU);
              ctx->cur_opcode->format = kv3_LSU_APPB_Y;
              ctx->cur_opcode->insn = kv3_ACSWAPW;
              return trans_v1_ACSWAPW_lsucond_registerY_extend27_offset27_registerZ_registerU_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU);
            default:
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU(ctx, codeWords, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU);
              ctx->cur_opcode->format = kv3_LSU_APPB_Y;
              ctx->cur_opcode->insn = kv3_ACSWAPD;
              return trans_v1_ACSWAPD_lsucond_registerY_extend27_offset27_registerZ_registerU_triple(ctx, &u.kv3_lsucond_kv3_registerY_kv3_extend27_offset27_kv3_registerZ_kv3_registerU);
            default:
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    default:
      break;
    }
    return false;
  case 0x0000000c:
    switch ((codeWord_0 >> 24) & 0x0000000f) {
    case 0x00000000:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DDDI0_Y;
            ctx->cur_opcode->insn = kv3_MADDD;
            return trans_v1_MADDD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMAD;
            return trans_v1_FFMAD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FADDD;
            return trans_v1_FADDD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQQDI_Y;
              ctx->cur_opcode->insn = kv3_FFMAWDP;
              return trans_v1_FFMAWDP_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQDI_Y;
              ctx->cur_opcode->insn = kv3_FMULWDP;
              return trans_v1_FMULWDP_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000001:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DDDI0_Y;
            ctx->cur_opcode->insn = kv3_MADDWP;
            return trans_v1_MADDWP_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMAWD;
            return trans_v1_FFMAWD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FADDWP;
            return trans_v1_FADDWP_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQQDI_Y;
              ctx->cur_opcode->insn = kv3_FFMAHWQ;
              return trans_v1_FFMAHWQ_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQDI_Y;
              ctx->cur_opcode->insn = kv3_FMULHWQ;
              return trans_v1_FMULHWQ_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000002:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DDDI0_Y;
            ctx->cur_opcode->insn = kv3_MADDHQ;
            return trans_v1_MADDHQ_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMAWP;
            return trans_v1_FFMAWP_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FADDHQ;
            return trans_v1_FADDHQ_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQQDI_Y;
              ctx->cur_opcode->insn = kv3_FFMSWDP;
              return trans_v1_FFMSWDP_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQDI_Y;
              ctx->cur_opcode->insn = kv3_FMULWDC;
              return trans_v1_FMULWDC_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000003:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMAHQ;
            return trans_v1_FFMAHQ_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FADDCWC;
            return trans_v1_FADDCWC_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQQDI_Y;
              ctx->cur_opcode->insn = kv3_FFMSHWQ;
              return trans_v1_FFMSHWQ_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_FQDI_Y;
              ctx->cur_opcode->insn = kv3_FMULCWDC;
              return trans_v1_FMULCWDC_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000004:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DDI_Y;
            ctx->cur_opcode->insn = kv3_MULD;
            return trans_v1_MULD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMSD;
            return trans_v1_FFMSD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FSBFD;
            return trans_v1_FSBFD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000005:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DDI_Y;
            ctx->cur_opcode->insn = kv3_MULWP;
            return trans_v1_MULWP_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMSWD;
            return trans_v1_FFMSWD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FSBFWP;
            return trans_v1_FSBFWP_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000006:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DDI_Y;
            ctx->cur_opcode->insn = kv3_MULHQ;
            return trans_v1_MULHQ_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMSWP;
            return trans_v1_FFMSWP_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FSBFHQ;
            return trans_v1_FSBFHQ_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000007:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DDI_Y;
            ctx->cur_opcode->insn = kv3_MULWC;
            return trans_v1_MULWC_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDDI_Y;
            ctx->cur_opcode->insn = kv3_FFMSHQ;
            return trans_v1_FFMSHQ_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI1_Y;
            ctx->cur_opcode->insn = kv3_FSBFCWC;
            return trans_v1_FSBFCWC_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000008:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QQDI_Y;
              ctx->cur_opcode->insn = kv3_MADDDT;
              return trans_v1_MADDDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QDI_Y;
              ctx->cur_opcode->insn = kv3_MULDT;
              return trans_v1_MULDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FMULD;
            return trans_v1_FMULD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSSI_Y;
            ctx->cur_opcode->insn = kv3_FFMAHW;
            return trans_v1_FFMAHW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x00000009:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QQDI_Y;
              ctx->cur_opcode->insn = kv3_MADDUDT;
              return trans_v1_MADDUDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QDI_Y;
              ctx->cur_opcode->insn = kv3_MULUDT;
              return trans_v1_MULUDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FMULWD;
            return trans_v1_FMULWD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSSI_Y;
            ctx->cur_opcode->insn = kv3_FFMAW;
            return trans_v1_FFMAW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000a:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QQDI_Y;
              ctx->cur_opcode->insn = kv3_MADDSUDT;
              return trans_v1_MADDSUDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QDI_Y;
              ctx->cur_opcode->insn = kv3_MULSUDT;
              return trans_v1_MULSUDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FMULWP;
            return trans_v1_FMULWP_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSSI_Y;
            ctx->cur_opcode->insn = kv3_FFMSHW;
            return trans_v1_FFMSHW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000b:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            switch ((codeWord_0 >> 18) & 0x00000001) {
            case 0x00000000:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QQDI_Y;
              ctx->cur_opcode->insn = kv3_MADDUZDT;
              return trans_v1_MADDUZDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
            case 0x00000001:
              decode_kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              ctx->cur_opcode->format = kv3_MAU_QDI_Y;
              ctx->cur_opcode->insn = kv3_CMULDT;
              return trans_v1_CMULDT_registerM_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerM_kv3_registerZ_kv3_extend27_upper27_lower10);
              break;
            }
            return false;
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FMULHQ;
            return trans_v1_FMULHQ_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSSI_Y;
            ctx->cur_opcode->insn = kv3_FFMSW;
            return trans_v1_FFMSW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000c:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DPI_Y;
            ctx->cur_opcode->insn = kv3_DOT2WD;
            return trans_v1_DOT2WD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FDOT2W;
            return trans_v1_FDOT2W_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSI_Y;
            ctx->cur_opcode->insn = kv3_FADDW;
            return trans_v1_FADDW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000d:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DPI_Y;
            ctx->cur_opcode->insn = kv3_DOT2UWD;
            return trans_v1_DOT2UWD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FDOT2WD;
            return trans_v1_FDOT2WD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSI_Y;
            ctx->cur_opcode->insn = kv3_FSBFW;
            return trans_v1_FSBFW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000e:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DPI_Y;
            ctx->cur_opcode->insn = kv3_DOT2SUWD;
            return trans_v1_DOT2SUWD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FMULWC;
            return trans_v1_FMULWC_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSI_Y;
            ctx->cur_opcode->insn = kv3_FMULW;
            return trans_v1_FMULW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    case 0x0000000f:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_DPI_Y;
            ctx->cur_opcode->insn = kv3_DOT2W;
            return trans_v1_DOT2W_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FDDI_Y;
            ctx->cur_opcode->insn = kv3_FMULCWC;
            return trans_v1_FMULCWC_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_MAU_FSSI_Y;
            ctx->cur_opcode->insn = kv3_FMULHW;
            return trans_v1_FMULHW_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
      break;
    }
    return false;
  case 0x0000000e:
    switch ((codeWord_1 >> 29) & 0x00000007) {
    case 0x00000004:
      switch ((codeWord_0 >> 16) & 0x00000003) {
      case 0x00000000:
        switch ((codeWord_0 >> 24) & 0x0000000f) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WI_Y;
            ctx->cur_opcode->insn = kv3_MAKE;
            return trans_v1_MAKE_registerW_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000001:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_ADDD;
            return trans_v1_ADDD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000002:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_MIND;
            return trans_v1_MIND_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000003:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_MAXD;
            return trans_v1_MAXD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000004:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_ABDD;
            return trans_v1_ABDD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000005:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_SBFD;
            return trans_v1_SBFD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000006:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_MINUD;
            return trans_v1_MINUD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000007:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_MAXUD;
            return trans_v1_MAXUD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000008:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_ANDD;
            return trans_v1_ANDD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x00000009:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_NANDD;
            return trans_v1_NANDD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x0000000a:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_ORD;
            return trans_v1_ORD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x0000000b:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_NORD;
            return trans_v1_NORD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x0000000c:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_XORD;
            return trans_v1_XORD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x0000000d:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_NXORD;
            return trans_v1_NXORD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x0000000e:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_ANDND;
            return trans_v1_ANDND_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        case 0x0000000f:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRI_Y;
            ctx->cur_opcode->insn = kv3_ORND;
            return trans_v1_ORND_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
          break;
        }
        return false;
      case 0x00000001:
        switch ((codeWord_2 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x0000000f) {
          case 0x0000000e:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRIS_Y;
            ctx->cur_opcode->insn = kv3_ADDSD;
            return trans_v1_ADDSD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          case 0x0000000f:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WRIS_Y;
            ctx->cur_opcode->insn = kv3_SBFSD;
            return trans_v1_SBFSD_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            decode_kv3_comparison_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_CWRI_Y;
            ctx->cur_opcode->insn = kv3_COMPD;
            return trans_v1_COMPD_comparison_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_comparison_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      case 0x00000002:
        switch ((codeWord_2 >> 29) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_0 >> 24) & 0x0000000f) {
          case 0x0000000e:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_BWRI_Y;
            ctx->cur_opcode->insn = kv3_SBMM8;
            return trans_v1_SBMM8_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          case 0x0000000f:
            decode_kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_BWRI_Y;
            ctx->cur_opcode->insn = kv3_SBMMT8;
            return trans_v1_SBMMT8_registerW_registerZ_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_registerZ_kv3_extend27_upper27_lower10);
          default:
            decode_kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_MWRI_Y;
            ctx->cur_opcode->insn = kv3_CMOVED;
            return trans_v1_CMOVED_scalarcond_registerZ_registerW_extend27_upper27_lower10_triple(ctx, &u.kv3_scalarcond_kv3_registerZ_kv3_registerW_kv3_extend27_upper27_lower10);
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    default:
      break;
    }
    return false;
  case 0x0000000f:
    switch ((codeWord_0 >> 24) & 0x0000000f) {
    case 0x00000000:
      switch ((codeWord_1 >> 29) & 0x00000007) {
      case 0x00000004:
        switch ((codeWord_0 >> 16) & 0x00000003) {
        case 0x00000000:
          switch ((codeWord_2 >> 29) & 0x00000003) {
          case 0x00000000:
            decode_kv3_registerW_kv3_extend27_upper27_lower10(ctx, codeWords, &u.kv3_registerW_kv3_extend27_upper27_lower10);
            ctx->cur_opcode->format = kv3_ALU_WIPC_Y;
            ctx->cur_opcode->insn = kv3_PCREL;
            return trans_v1_PCREL_registerW_extend27_upper27_lower10_triple(ctx, &u.kv3_registerW_kv3_extend27_upper27_lower10);
          default:
            break;
          }
          return false;
        default:
          break;
        }
        return false;
      default:
        break;
      }
      return false;
    default:
      break;
    }
    return false;
  default:
    break;
  }
  return false;
}

