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
groupshared uint s_out_segments_count;
#endif
[[vk::push_constant]] cbuffer STRANDS_RENDER_CONSTANTS {
  float4 min_color;
  float4 max_color;

  uint camera_index;
  uint segments_size;
  uint color_mode;
  float multiplier;
  float boundary_layer_radius;
};

[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
#if EXT_TASK_SUBGROUP_COUNT > 1
  if (laneID == 0) {
    s_out_segments_count = 0;
  }
  GroupMemoryBarrierWithGroupSync();
#endif
  uint out_segments_count = 0;
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint segment_index_local = laneID + i * WORKGROUP_SIZE;
    uint segment_index_global = baseID + segment_index_local;

    bool render = !(segment_index_global > segments_size - 1);

    uint4 vote_segments = WaveActiveBallot(render);
    uint num_segments = (countbits(vote_segments.x) + countbits(vote_segments.y) + countbits(vote_segments.z) + countbits(vote_segments.w));

#if EXT_TASK_SUBGROUP_COUNT > 1
    if (WaveGetLaneIndex() == 0) {
      InterlockedAdd(s_out_segments_count, num_segments, out_segments_count);
    }
    out_segments_count = WaveReadLaneFirst(out_segments_count);
#endif
    uint idx_offset = WavePrefixCountBits(render) + out_segments_count;
    if (render) {
      ts_out.deltaIDs[idx_offset] = uint8_t(segment_index_local);
    }
#if EXT_TASK_SUBGROUP_COUNT == 1
    out_segments_count += num_segments;
#endif
  }

#if EXT_TASK_SUBGROUP_COUNT > 1
  GroupMemoryBarrierWithGroupSync();
  out_segments_count = s_out_segments_count;
#endif
  if (laneID == 0) {
    ts_out.baseID = baseID;
  }
  DispatchMesh(out_segments_count, 1, 1, ts_out);
}