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

#ifndef HW_KVX_DMA_H
#define HW_KVX_DMA_H

#include "hw/sysbus.h"

#define KVX_DMA_NUM_RX_CHANNEL     64
#define KVX_DMA_NUM_RX_JOB_QUEUE   8
#define KVX_DMA_NUM_RX_JOB_CACHE   4
#define KVX_DMA_NUM_TX_THREAD      4
#define KVX_DMA_NUM_TX_JOB_QUEUE   64
#define KVX_DMA_NUM_TX_COMP_QUEUE  64

#define TYPE_KVX_DMA "kvx-dma"
#define KVX_DMA(obj) \
    OBJECT_CHECK(KvxDmaState, (obj), TYPE_KVX_DMA)

#define KVX_DMA_MMIO_LEN 0x100000

typedef struct KvxDmaState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    /* Errors */
    uint64_t rx_channel_err;
    uint64_t rx_job_queue_err;
    uint64_t tx_job_queue_err;
    uint64_t tx_thread_err;
    uint64_t tx_comp_queue_err;

    /* IRQs */
    uint64_t irq_en;
    uint64_t irq_vector;

    MemoryRegion iomem;

    MemoryRegion *root_mr;
    AddressSpace as;

    qemu_irq irq;
} KvxDmaState;

#endif
