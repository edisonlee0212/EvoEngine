import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = EXT_TASK_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((EXT_TASK_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

groupshared Task ts_out;

#if EXT_TASK_SUBGROUP_COUNT > 1
groupshared uint s_out_foliage_count;
#endif
[[vk::push_constant]] cbuffer STRANDS_RENDER_CONSTANTS {
  float4 min_color;
  float4 max_color;

  uint camera_index;
  uint foliage_size;
  uint color_mode;
};

[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
#if EXT_TASK_SUBGROUP_COUNT > 1
  if (laneID == 0) {
    s_out_foliage_count = 0;
  }
  GroupMemoryBarrierWithGroupSync();
#endif
  uint out_foliage_count = 0;
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint leaf_index_local = laneID + i * WORKGROUP_SIZE;
    uint leaf_index_global = baseID + leaf_index_local;

    bool render = !(leaf_index_global > foliage_size - 1);

    uint4 vote_foliage = WaveActiveBallot(render);
    uint num_foliage = (countbits(vote_foliage.x) + countbits(vote_foliage.y) + countbits(vote_foliage.z) + countbits(vote_foliage.w));

#if EXT_TASK_SUBGROUP_COUNT > 1
    if (WaveGetLaneIndex() == 0) {
      InterlockedAdd(s_out_foliage_count, num_foliage, out_foliage_count);
    }
    out_foliage_count = WaveReadLaneFirst(out_foliage_count);
#endif
    uint idx_offset = WavePrefixCountBits(render) + out_foliage_count;
    if (render) {
      ts_out.deltaIDs[idx_offset] = uint8_t(leaf_index_local);
    }
#if EXT_TASK_SUBGROUP_COUNT == 1
    out_foliage_count += num_foliage;
#endif
  }

#if EXT_TASK_SUBGROUP_COUNT > 1
  GroupMemoryBarrierWithGroupSync();
  out_foliage_count = s_out_foliage_count;
#endif
  if (laneID == 0) {
    ts_out.baseID = baseID;
  }
  DispatchMesh(out_foliage_count, 1, 1, ts_out);
}