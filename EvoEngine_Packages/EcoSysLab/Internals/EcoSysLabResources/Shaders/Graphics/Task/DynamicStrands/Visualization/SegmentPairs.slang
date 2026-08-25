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
groupshared uint s_out_segment_pairs_count;
#endif
[[vk::push_constant]] cbuffer STRANDS_RENDER_CONSTANTS {
  float4 min_color;
  float4 max_color;

  uint camera_index;
  uint segment_pairs_size;
  uint color_mode;
  uint multiplier;
};

[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
#if EXT_TASK_SUBGROUP_COUNT > 1
  if (laneID == 0) {
    s_out_segment_pairs_count = 0;
  }
  GroupMemoryBarrierWithGroupSync();
#endif
  uint out_segment_pairs_count = 0;
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint segment_pair_index_local = laneID + i * WORKGROUP_SIZE;
    uint segment_pair_index_global = baseID + segment_pair_index_local;

    bool render = segment_pair_index_global < segment_pairs_size;
    if (render) {
      SegmentPair segment_pair = segment_pairs[segment_pair_index_global];
      render = segment_pair.connectivity_integrity > 0.0f || segment_pair.bend_twist_bundle_integrity > 0.0f;
    }

    uint4 vote_segment_pairs = WaveActiveBallot(render);
    uint num_segment_pairs = (countbits(vote_segment_pairs.x) + countbits(vote_segment_pairs.y) + countbits(vote_segment_pairs.z) + countbits(vote_segment_pairs.w));

#if EXT_TASK_SUBGROUP_COUNT > 1
    if (WaveGetLaneIndex() == 0) {
      InterlockedAdd(s_out_segment_pairs_count, num_segment_pairs, out_segment_pairs_count);
    }
    out_segment_pairs_count = WaveReadLaneFirst(out_segment_pairs_count);
#endif
    uint idx_offset = WavePrefixCountBits(render) + out_segment_pairs_count;
    if (render) {
      ts_out.deltaIDs[idx_offset] = uint8_t(segment_pair_index_local);
    }
#if EXT_TASK_SUBGROUP_COUNT == 1
    out_segment_pairs_count += num_segment_pairs;
#endif
  }

#if EXT_TASK_SUBGROUP_COUNT > 1
  GroupMemoryBarrierWithGroupSync();
  out_segment_pairs_count = s_out_segment_pairs_count;
#endif
  if (laneID == 0) {
    ts_out.baseID = baseID;
  }
  DispatchMesh(out_segment_pairs_count, 1, 1, ts_out);
}
