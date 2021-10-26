/*
 * Kalray KVX MPPA cluster DMA
 *
 * Copyright (c) 2021 Kalray Inc.
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
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "hw/dma/kvx-dma.h"

#include "trace.h"
#include "registers.h"
#include "internals.h"

/*
 * Decode a bundle. Return false when the bundle is invalid
 */
bool kvx_dma_bundle_decode(uint64_t bundle, DecodedBundle *decoded)
{
    bool is_send_cmd;

    decoded->dcnt = FIELD_EX64(bundle, TX_PGRM_MEM, SEL_DCNT);
    decoded->move_size = 1 << FIELD_EX64(bundle, TX_PGRM_MEM, MOVE_SIZE);
    decoded->wptr_type = FIELD_EX64(bundle, TX_PGRM_MEM, WPTR_TYPE);

    decoded->reg_size = FIELD_EX64(bundle, TX_PGRM_MEM, REG_SIZE);
    decoded->reg = FIELD_EX64(bundle, TX_PGRM_MEM, REG_INDEX);
    decoded->reg >>= decoded->reg_size;
    decoded->reg_size = 1 << decoded->reg_size;

    decoded->branch_op = FIELD_EX64(bundle, TX_PGRM_MEM, BR_COND);

    if (decoded->branch_op) {
        decoded->branch_addr = FIELD_EX64(bundle, TX_PGRM_MEM, BR_ADDR);
    } else {
        decoded->branch_addr = 0;
    }

    decoded->dcnt_op = FIELD_EX64(bundle, TX_PGRM_MEM, WR_DCNT);
    decoded->wptr_op = FIELD_EX64(bundle, TX_PGRM_MEM, INCR_WPTR);
    decoded->rptr_op = FIELD_EX64(bundle, TX_PGRM_MEM, INCR_RPTR);
    decoded->cmd_op = FIELD_EX64(bundle, TX_PGRM_MEM, CMD);

    if (decoded->cmd_op > BUNDLE_CMD_RESET_STROBE) {
        /* Reserved values */
        return false;
    }

    is_send_cmd = (decoded->cmd_op == BUNDLE_CMD_SEND_RPTR)
        || (decoded->cmd_op== BUNDLE_CMD_SEND_REG);

    if ((decoded->wptr_op == BUNDLE_WPTR_INC) && !is_send_cmd) {
        /*
         * Must have a send command to increment the write pointer by the read
         * size.
         */
        return false;
    }

    if ((decoded->rptr_op == BUNDLE_RPTR_INC) && !is_send_cmd) {
        /*
         * Must have a send command to increment the read pointer by the read
         * size.
         */
        return false;
    }

    decoded->load_strobe = FIELD_EX64(bundle, TX_PGRM_MEM, LOAD_STRB);
    decoded->reg_sign_ext = FIELD_EX64(bundle, TX_PGRM_MEM, REG_SIGNED_EXT);
    decoded->stop = FIELD_EX64(bundle, TX_PGRM_MEM, STOP);
    decoded->send_eot = FIELD_EX64(bundle, TX_PGRM_MEM, SEND_EOT);
    decoded->flush = FIELD_EX64(bundle, TX_PGRM_MEM, FLUSH);

    return true;
}

/*
 * Disassemble a bundle for debugging and tracing purposes.
 */
void kvx_dma_bundle_disas(const DecodedBundle *decoded, const char *sep,
                          GString *out)
{
    static const char REG_SIZE_SUFFIX[] = {
        [1] = 'b',
        [2] = 'h',
        [4] = 'w',
        [8] = 'd',
    };

    switch (decoded->wptr_type) {
    case BUNDLE_WPTR_TYPE_NOC_ABS:
        g_string_append_printf(out, "%swptr_type(noc_abs)", sep);
        break;

    case BUNDLE_WPTR_TYPE_NOC_REL:
        g_string_append_printf(out, "%swptr_type(noc_rel)", sep);
        break;

    case BUNDLE_WPTR_TYPE_STORE_ADDR:
        g_string_append_printf(out, "%swptr_type(store_addr)", sep);
        break;

    case BUNDLE_WPTR_TYPE_ATOMIC_ADD_ADDR:
        g_string_append_printf(out, "%swptr_type(atomic_add_addr)", sep);
        break;

    }

    switch (decoded->cmd_op) {
    case BUNDLE_CMD_FENCE:
        g_string_append_printf(out, "%sfence", sep);
        break;

    case BUNDLE_CMD_SEND_RPTR:
        g_string_append_printf(out, "%srecv(rprt) -> send(wptr)", sep);
        break;

    case BUNDLE_CMD_NOTIFY:
        g_string_append_printf(out, "%snotify", sep);
        break;

    case BUNDLE_CMD_SEND_REG:
        g_string_append_printf(out, "%sr%c[%d] -> send(wptr)", sep,
                               REG_SIZE_SUFFIX[decoded->reg_size],
                               decoded->reg);
        break;

    case BUNDLE_CMD_RESET_STROBE:
        g_string_append_printf(out, "%sstrobe <- 0xffff", sep);
        break;

    default:
        break;
    }

    switch (decoded->dcnt_op) {
    case BUNDLE_DCNT_DEC:
        g_string_append_printf(out, "%sdcnt%u--", sep, decoded->dcnt);
        break;

    case BUNDLE_DCNT_LOAD_REG:
        g_string_append_printf(out, "%sdcnt%u <- r%c[%d]", sep,
                               decoded->dcnt,
                               REG_SIZE_SUFFIX[decoded->reg_size],
                               decoded->reg);
        break;

    case BUNDLE_DCNT_LOAD_DCNT:
        g_string_append_printf(out, "%sdcnt%u <- dcnt%u",
                               sep, decoded->dcnt, decoded->reg & 3);
        break;

    default:
        break;
    }

    switch (decoded->rptr_op) {
    case BUNDLE_RPTR_LOAD:
        g_string_append_printf(out, "%srptr <- r%c[%d]", sep,
                               REG_SIZE_SUFFIX[decoded->reg_size],
                                decoded->reg);
        break;

    case BUNDLE_RPTR_INC:
        g_string_append_printf(out, "%srptr += %d", sep,
                               decoded->move_size);
        break;

    case BUNDLE_RPTR_ADD:
        g_string_append_printf(out, "%srptr += r%c[%d]", sep,
                               REG_SIZE_SUFFIX[decoded->reg_size],
                               decoded->reg);
        break;

    default:
        break;
    }

    switch (decoded->wptr_op) {
    case BUNDLE_WPTR_LOAD:
        g_string_append_printf(out, "%swptr <- r%c[%d]", sep,
                               REG_SIZE_SUFFIX[decoded->reg_size],
                               decoded->reg);
        break;

    case BUNDLE_WPTR_INC:
        g_string_append_printf(out, "%swptr += %d",
                               sep, decoded->move_size);
        break;

    case BUNDLE_WPTR_ADD:
        g_string_append_printf(out, "%swptr += r%c[%d]", sep,
                               REG_SIZE_SUFFIX[decoded->reg_size],
                               decoded->reg);
        break;

    default:
        break;
    }

    switch (decoded->branch_op) {
    case BUNDLE_BR:
        g_string_append_printf(out, "%sb %02" PRIx8 "",
                               sep, decoded->branch_addr);
        break;

    case BUNDLE_BR_BEZ:
        g_string_append_printf(out, "%sbez dcnt%u, %02" PRIx8 "",
                               sep, decoded->dcnt, decoded->branch_addr);
        break;

    case BUNDLE_BR_BNEZ:
        g_string_append_printf(out, "%sbnez dcnt%u, %02" PRIx8 "",
                               sep, decoded->dcnt, decoded->branch_addr);
        break;

    default:
        break;
    }

    if (decoded->send_eot) {
        g_string_append_printf(out, "%ssend-eot", sep);
    }

    if (decoded->flush) {
        g_string_append_printf(out, "%sflush", sep);
    }

    if (decoded->stop) {
        g_string_append_printf(out, "%sstop", sep);
    }

    g_string_append(out, ";;");
}
