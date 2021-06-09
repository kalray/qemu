#ifndef _GEN_MODIFIERS_H
#define _GEN_MODIFIERS_H
typedef enum Modifier_kv3_exunum Modifier_kv3_exunum;
enum Modifier_kv3_exunum {
    EXUNUM_ALU0 = 0,
    EXUNUM_ALU1 = 1,
    EXUNUM_MAU = 2,
    EXUNUM_LSU = 3,
};

typedef enum Modifier_kv3_scalarcond Modifier_kv3_scalarcond;
enum Modifier_kv3_scalarcond {
    SCALARCOND_DNEZ = 0,
    SCALARCOND_DEQZ = 1,
    SCALARCOND_DLTZ = 2,
    SCALARCOND_DGEZ = 3,
    SCALARCOND_DLEZ = 4,
    SCALARCOND_DGTZ = 5,
    SCALARCOND_ODD = 6,
    SCALARCOND_EVEN = 7,
    SCALARCOND_WNEZ = 8,
    SCALARCOND_WEQZ = 9,
    SCALARCOND_WLTZ = 10,
    SCALARCOND_WGEZ = 11,
    SCALARCOND_WLEZ = 12,
    SCALARCOND_WGTZ = 13,
};

typedef enum Modifier_kv3_simplecond Modifier_kv3_simplecond;
enum Modifier_kv3_simplecond {
    SIMPLECOND_NEZ = 0,
    SIMPLECOND_EQZ = 1,
    SIMPLECOND_LTZ = 2,
    SIMPLECOND_GEZ = 3,
    SIMPLECOND_LEZ = 4,
    SIMPLECOND_GTZ = 5,
    SIMPLECOND_ODD = 6,
    SIMPLECOND_EVEN = 7,
};

typedef enum Modifier_kv3_comparison Modifier_kv3_comparison;
enum Modifier_kv3_comparison {
    COMPARISON_NE = 0,
    COMPARISON_EQ = 1,
    COMPARISON_LT = 2,
    COMPARISON_GE = 3,
    COMPARISON_LE = 4,
    COMPARISON_GT = 5,
    COMPARISON_LTU = 6,
    COMPARISON_GEU = 7,
    COMPARISON_LEU = 8,
    COMPARISON_GTU = 9,
    COMPARISON_ALL = 10,
    COMPARISON_NALL = 11,
    COMPARISON_ANY = 12,
    COMPARISON_NONE = 13,
};

typedef enum Modifier_kv3_floatcomp Modifier_kv3_floatcomp;
enum Modifier_kv3_floatcomp {
    FLOATCOMP_ONE = 0,
    FLOATCOMP_UEQ = 1,
    FLOATCOMP_OEQ = 2,
    FLOATCOMP_UNE = 3,
    FLOATCOMP_OLT = 4,
    FLOATCOMP_UGE = 5,
    FLOATCOMP_OGE = 6,
    FLOATCOMP_ULT = 7,
};

typedef enum Modifier_kv3_rounding Modifier_kv3_rounding;
enum Modifier_kv3_rounding {
    ROUNDING_RN = 0,
    ROUNDING_RU = 1,
    ROUNDING_RD = 2,
    ROUNDING_RZ = 3,
    ROUNDING_RNA = 4,
    ROUNDING_RNZ = 5,
    ROUNDING_RO = 6,
    ROUNDING_EMPTY = 7,
};

typedef enum Modifier_kv3_silent Modifier_kv3_silent;
enum Modifier_kv3_silent {
    SILENT_EMPTY = 0,
    SILENT_S = 1,
};

typedef enum Modifier_kv3_roundint Modifier_kv3_roundint;
enum Modifier_kv3_roundint {
    ROUNDINT_RN = 0,
    ROUNDINT_RU = 1,
    ROUNDINT_RD = 2,
    ROUNDINT_RZ = 3,
    ROUNDINT_RHU = 4,
};

typedef enum Modifier_kv3_saturate Modifier_kv3_saturate;
enum Modifier_kv3_saturate {
    SATURATE_SAT = 0,
    SATURATE_SATU = 1,
};

typedef enum Modifier_kv3_rectify Modifier_kv3_rectify;
enum Modifier_kv3_rectify {
    RECTIFY_EMPTY = 0,
    RECTIFY_RELU = 1,
};

typedef enum Modifier_kv3_variant Modifier_kv3_variant;
enum Modifier_kv3_variant {
    VARIANT_EMPTY = 0,
    VARIANT_S = 1,
    VARIANT_U = 2,
    VARIANT_US = 3,
};

typedef enum Modifier_kv3_speculate Modifier_kv3_speculate;
enum Modifier_kv3_speculate {
    SPECULATE_EMPTY = 0,
    SPECULATE_S = 1,
};

typedef enum Modifier_kv3_column Modifier_kv3_column;
enum Modifier_kv3_column {
    COLUMN_C0 = 0,
    COLUMN_C1 = 1,
    COLUMN_C2 = 2,
    COLUMN_C3 = 3,
};

typedef enum Modifier_kv3_doscale Modifier_kv3_doscale;
enum Modifier_kv3_doscale {
    DOSCALE_EMPTY = 0,
    DOSCALE_XS = 1,
};

typedef enum Modifier_kv3_splat32 Modifier_kv3_splat32;
enum Modifier_kv3_splat32 {
    SPLAT32_EMPTY = 0,
    SPLAT32_AT = 1,
};

typedef enum Modifier_kv3_cachelev Modifier_kv3_cachelev;
enum Modifier_kv3_cachelev {
    CACHELEV_EMPTY = 0,
    CACHELEV_L2 = 1,
    CACHELEV_L1L2 = 2,
};

typedef enum Modifier_kv3_boolcas Modifier_kv3_boolcas;
enum Modifier_kv3_boolcas {
    BOOLCAS_V = 0,
    BOOLCAS_EMPTY = 1,
};

typedef enum Modifier_kv3_channel Modifier_kv3_channel;
enum Modifier_kv3_channel {
    CHANNEL_F = 0,
    CHANNEL_B = 1,
};

#endif
