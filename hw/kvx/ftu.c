/*
 * Kalray KVX MPPA FTU
 *
 * Copyright (c) 2021 Kalray
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
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/kvx/coolidge-cluster.h"
#include "hw/kvx/ftu.h"
#include "trace.h"

REG32(OTHER_CLUSTERS_CTRL, 0x4)
    FIELD(OTHER_CLUSTERS_CTRL, RM_WUP, 0, 1)
    FIELD(OTHER_CLUSTERS_CTRL, RST, 1, 1)
    FIELD(OTHER_CLUSTERS_CTRL, CLKEN, 2, 1)
    FIELD(OTHER_CLUSTERS_CTRL, SCRAM_DIS, 3, 1)

#define OTHER_CLUSTERS_CTRL_MASK (R_OTHER_CLUSTERS_CTRL_RM_WUP_MASK \
                           | R_OTHER_CLUSTERS_CTRL_RST_MASK \
                           | R_OTHER_CLUSTERS_CTRL_CLKEN_MASK \
                           | R_OTHER_CLUSTERS_CTRL_SCRAM_DIS_MASK)

REG32(CLUSTERS_STATUS, 0x20)
    FIELD(CLUSTERS_STATUS, RM_RUNNING, 0, 1)
    FIELD(CLUSTERS_STATUS, RST, 1, 1)
    FIELD(CLUSTERS_STATUS, CLKEN, 2, 1)
    FIELD(CLUSTERS_STATUS, SCRAM_DIS, 3, 1)

#define CLUSTERS_STATUS_MASK (R_CLUSTERS_STATUS_RM_RUNNING_MASK \
                           | R_CLUSTERS_STATUS_RST_MASK \
                           | R_CLUSTERS_STATUS_CLKEN_MASK \
                           | R_CLUSTERS_STATUS_SCRAM_DIS_MASK)

REG32(RM_RESET_PC, 0x60)
    FIELD(RM_RESET_PC, VALUE, 12, 20)

#define RM_RESET_PC_MASK (R_RM_RESET_PC_VALUE_MASK)



static inline void kvx_ftu_update_cluster(KvxFtuState *s, int cluster_id)
{
    KvxCoolidgeClusterState *cluster = &s->clusters[cluster_id+1]; /* cluster id + 1 because OTHER_CLUSTERS_CTRL can't control cluster 0 */
    CPUState *cpu = kvx_coolidge_cluster_get_rm(cluster);
    KVXCPU *kvx_cpu = KVX_CPU(cpu);
    CPUKVXState *env = &kvx_cpu->env;

    if (FIELD_EX64(s->other_clusters_ctrl[cluster_id], OTHER_CLUSTERS_CTRL, RM_WUP)) {
        if (env->sleep_state != KVX_RESETTING) {
            return;
        }
        env->sleep_state = KVX_RUNNING;
        trace_kvx_ftu_wakeup_cluster(cluster_id+1);
        qemu_cpu_kick(cpu);

    }

    if (FIELD_EX64(s->other_clusters_ctrl[cluster_id], OTHER_CLUSTERS_CTRL, RST)) {
        cpu_set_pc(cpu, s->rm_reset_pc[cluster_id]);
        env->sleep_state = KVX_RESETTING;
    }
}

static inline int kvx_ftu_offset_decode(hwaddr *offset)
{
    hwaddr true_offset = *offset;
    if (*offset >= A_OTHER_CLUSTERS_CTRL && *offset < 0x14) {
        *offset = A_OTHER_CLUSTERS_CTRL;
        return (true_offset - A_OTHER_CLUSTERS_CTRL) / 0x4;
    } else if (*offset >= A_CLUSTERS_STATUS && *offset < 0x34) {
        *offset = A_CLUSTERS_STATUS;
        return (true_offset - A_CLUSTERS_STATUS) / 0x4;
    }
    else if (*offset >= A_RM_RESET_PC && *offset < 0x74) {
        *offset = A_RM_RESET_PC;
        return (true_offset - A_RM_RESET_PC) / 0x4;
    }

    return 0;
}

static uint64_t kvx_ftu_read(void *opaque, hwaddr offset, unsigned size)
{
    KvxFtuState *s = KVX_FTU(opaque);
    uint64_t ret;

    hwaddr offset_decoded = offset;
    int cluster_id = 0;

    cluster_id = kvx_ftu_offset_decode(&offset_decoded);

    switch (offset_decoded) {
    case A_OTHER_CLUSTERS_CTRL:
        ret = s->other_clusters_ctrl[cluster_id];
        break;
    case A_CLUSTERS_STATUS:
        ret = s->cluster_status[cluster_id];
        break;

    case A_RM_RESET_PC:
        ret = s->rm_reset_pc[cluster_id];
        break;

    default:
        ret = 0;
    }

    trace_kvx_ftu_read(offset, ret);
    return ret;
}

static void kvx_ftu_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    KvxFtuState *s = KVX_FTU(opaque);
    trace_kvx_ftu_write(offset, value);

    hwaddr offset_decoded = offset;
    int cluster_id = 0;

    cluster_id = kvx_ftu_offset_decode(&offset_decoded);

    switch (offset_decoded) {
    case A_OTHER_CLUSTERS_CTRL:
        s->other_clusters_ctrl[cluster_id] = value & OTHER_CLUSTERS_CTRL_MASK;
        kvx_ftu_update_cluster(s, cluster_id);
        break;
    case A_CLUSTERS_STATUS:
       s->cluster_status[cluster_id] = value & CLUSTERS_STATUS_MASK;
        break;

    case A_RM_RESET_PC:
        s->rm_reset_pc[cluster_id] = value & RM_RESET_PC_MASK;
        break;

    default:
        break;
    }
}

static const MemoryRegionOps kvx_ftu_ops = {
    .read = kvx_ftu_read,
    .write = kvx_ftu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void kvx_ftu_reset(DeviceState *dev)
{
    KvxFtuState *s = KVX_FTU(dev);

    memset(s->other_clusters_ctrl, 0, sizeof(s->other_clusters_ctrl));
    memset(s->cluster_status, 0, sizeof(s->cluster_status));

    memset(s->rm_reset_pc, 0, sizeof(s->rm_reset_pc));
}

static void kvx_ftu_init(Object *obj)
{
    KvxFtuState *s = KVX_FTU(obj);

    memory_region_init_io(&s->iomem, obj, &kvx_ftu_ops, s,
                          TYPE_KVX_FTU, KVX_FTU_MMIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static Property kvx_ftu_properties[] = {
    DEFINE_PROP_LINK("clusters", KvxFtuState, clusters,
                      TYPE_KVX_CLUSTER, KvxCoolidgeClusterState *),
    DEFINE_PROP_END_OF_LIST(),
};

static void kvx_ftu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = kvx_ftu_reset;

    device_class_set_props(dc, kvx_ftu_properties);
}

static TypeInfo kvx_ftu_info = {
    .name          = TYPE_KVX_FTU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(KvxFtuState),
    .class_init    = kvx_ftu_class_init,
    .instance_init = kvx_ftu_init,
};

static void kvx_ftu_register_types(void)
{
    type_register_static(&kvx_ftu_info);
}

type_init(kvx_ftu_register_types)
