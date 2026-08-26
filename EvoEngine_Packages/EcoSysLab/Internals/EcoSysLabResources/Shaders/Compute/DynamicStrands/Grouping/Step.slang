import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK =
    (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;

struct PushConstants {
  uint segment_size;
};

[[vk::push_constant]] ConstantBuffer<PushConstants> constants;
[[vk::binding(0, 1)]] RWStructuredBuffer<int, Std430DataLayout> new_group_indices;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint base_id = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]] for (uint iteration = 0; iteration < ITERATIONS_PER_TASK; ++iteration) {
    const uint segment_index = base_id + lane_id + iteration * WORKGROUP_SIZE;
    if (segment_index >= constants.segment_size)
      break;

    Segment segment = segments[segment_index];
    int min_group_index = segment.group_index;
    [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; ++i) {
      const int pair_handle = segment_connection_handles[segment_index].handles[i];
      if (pair_handle < 0)
        continue;
      SegmentPair segment_pair = segment_pairs[pair_handle];
      if (segment_pair.connectivity_integrity <= 0.0f && segment_pair.bend_twist_bundle_integrity <= 0.0f)
        continue;
      const bool is_segment0 = segment_index == segment_pair.segment0_handle;
      const int neighbor_group_index =
          segments[is_segment0 ? segment_pair.segment1_handle : segment_pair.segment0_handle].group_index;
      min_group_index = min(min_group_index, neighbor_group_index);
    }
    new_group_indices[segment_index] = min_group_index;
  }
}
