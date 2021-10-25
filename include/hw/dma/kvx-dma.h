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

#define KVX_DMA_TX_PGRM_MEM_SIZE      128
#define KVX_DMA_TX_PGRM_TABLE_SIZE    16
#define KVX_DMA_NOC_ROUTE_TABLE_SIZE  512
#define KVX_DMA_BW_LIMITER_TABLE_SIZE 16

#define TYPE_KVX_DMA "kvx-dma"
#define KVX_DMA(obj) \
    OBJECT_CHECK(KvxDmaState, (obj), TYPE_KVX_DMA)

#define KVX_DMA_MMIO_LEN 0x100000

typedef struct KvxDmaTxCompQueue {
    uint64_t errors;
    uint64_t start_addr;
    uint64_t write_ptr;
    uint64_t read_ptr;
    uint64_t valid_read_ptr;
    uint64_t notify_addr;
    uint64_t notify_arg;
    uint16_t asn;
    uint8_t num_slots;
    uint8_t field_enabled;
    bool running;
    bool static_mode;
    bool global;
} KvxDmaTxCompQueue;

typedef struct KvxDmaState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    KvxDmaTxCompQueue tx_comp_queue[KVX_DMA_NUM_TX_COMP_QUEUE];

    uint64_t tx_pgrm_mem[KVX_DMA_TX_PGRM_MEM_SIZE];
    uint64_t tx_pgrm_table[KVX_DMA_TX_PGRM_TABLE_SIZE];
    uint64_t noc_route_table[KVX_DMA_NOC_ROUTE_TABLE_SIZE];
    uint64_t bw_limiter_table[KVX_DMA_BW_LIMITER_TABLE_SIZE];

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
