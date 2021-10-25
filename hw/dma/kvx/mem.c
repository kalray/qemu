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
#include "hw/dma/kvx-dma.h"

#include "trace.h"
#include "registers.h"
#include "internals.h"

static inline uint64_t dma_mem_read(uint64_t *mem, hwaddr offset,
                                    size_t mem_elts)
{
    if (offset >= (mem_elts * sizeof(uint64_t))) {
        return 0;
    }

    return mem[offset / sizeof(uint64_t)];
}

static inline void dma_mem_write(uint64_t *mem, hwaddr offset,
                                 uint64_t value, size_t mem_elts)
{
    if (offset >= (mem_elts * sizeof(uint64_t))) {
        return;
    }

    mem[offset / sizeof(uint64_t)] = value;
}

uint64_t kvx_dma_tx_pgrm_mem_read(KvxDmaState *s, size_t id,
                                  hwaddr offset, unsigned int size)
{
    uint64_t ret = dma_mem_read(s->tx_pgrm_mem, offset,
                                KVX_DMA_TX_PGRM_MEM_SIZE);

    trace_kvx_dma_tx_pgrm_mem_read(offset, ret);

    return ret;
}

void kvx_dma_tx_pgrm_mem_write(KvxDmaState *s, size_t id, hwaddr offset,
                               uint64_t value, unsigned int size)
{
    trace_kvx_dma_tx_pgrm_mem_write(offset, value);

    dma_mem_write(s->tx_pgrm_mem, offset, value, KVX_DMA_TX_PGRM_MEM_SIZE);
}

uint64_t kvx_dma_tx_pgrm_table_read(KvxDmaState *s, size_t id,
                                  hwaddr offset, unsigned int size)
{
    uint64_t ret = dma_mem_read(s->tx_pgrm_table, offset,
                                KVX_DMA_TX_PGRM_TABLE_SIZE);

    trace_kvx_dma_tx_pgrm_table_read(offset, ret);

    return ret;
}

void kvx_dma_tx_pgrm_table_write(KvxDmaState *s, size_t id, hwaddr offset,
                               uint64_t value, unsigned int size)
{
    trace_kvx_dma_tx_pgrm_table_write(offset, value);

    dma_mem_write(s->tx_pgrm_table, offset, value, KVX_DMA_TX_PGRM_TABLE_SIZE);
}

uint64_t kvx_dma_noc_route_table_read(KvxDmaState *s, size_t id,
                                  hwaddr offset, unsigned int size)
{
    uint64_t ret = dma_mem_read(s->noc_route_table, offset,
                                KVX_DMA_NOC_ROUTE_TABLE_SIZE);

    trace_kvx_dma_noc_route_table_read(offset, ret);

    return ret;
}

void kvx_dma_noc_route_table_write(KvxDmaState *s, size_t id, hwaddr offset,
                               uint64_t value, unsigned int size)
{
    trace_kvx_dma_noc_route_table_write(offset, value);

    dma_mem_write(s->noc_route_table, offset, value, KVX_DMA_NOC_ROUTE_TABLE_SIZE);
}

uint64_t kvx_dma_bw_limiter_table_read(KvxDmaState *s, size_t id,
                                  hwaddr offset, unsigned int size)
{
    uint64_t ret = dma_mem_read(s->bw_limiter_table, offset,
                                KVX_DMA_BW_LIMITER_TABLE_SIZE);

    trace_kvx_dma_bw_limiter_table_read(offset, ret);

    return ret;
}

void kvx_dma_bw_limiter_table_write(KvxDmaState *s, size_t id, hwaddr offset,
                               uint64_t value, unsigned int size)
{
    trace_kvx_dma_bw_limiter_table_write(offset, value);

    dma_mem_write(s->bw_limiter_table, offset, value, KVX_DMA_BW_LIMITER_TABLE_SIZE);
}

void kvx_dma_mem_reset(KvxDmaState *s)
{
    memset(s->tx_pgrm_mem, 0, sizeof(s->tx_pgrm_mem));
    memset(s->tx_pgrm_table, 0, sizeof(s->tx_pgrm_table));
    memset(s->noc_route_table, 0, sizeof(s->noc_route_table));
    memset(s->bw_limiter_table, 0, sizeof(s->bw_limiter_table));
}
