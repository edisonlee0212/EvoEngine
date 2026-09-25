import EcoSysLab.DynamicStrandsSet0;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint pair_size;
  float HC_threshold;
  float HL_threshold;
  float padding;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint base_id = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    const uint pair_index = base_id + lane_id + i * WORKGROUP_SIZE;
    if (pair_index >= pair_size)
      break;
    if (segment_pairs[pair_index].connectivity_integrity <= 0.0 &&
        segment_pairs[pair_index].bend_twist_bundle_integrity <= 0.0)
      continue;

    const int segment0_handle = segment_pairs[pair_index].segment0_handle;
    const int segment1_handle = segment_pairs[pair_index].segment1_handle;
    const bool same_color = all(uint4(clamp(segments[segment0_handle].color * 255.0, 0.0, 255.0)) ==
                                uint4(clamp(segments[segment1_handle].color * 255.0, 0.0, 255.0)));
    if (segment_biology[segment0_handle].HL < HL_threshold || segment_biology[segment1_handle].HL < HL_threshold ||
        (!same_color && segment_biology[segment0_handle].HC < HC_threshold &&
         segment_biology[segment1_handle].HC < HC_threshold)) {
      segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
      segment_pairs[pair_index].connectivity_integrity = 0.0;
    }

    if (same_color && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                       segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0)) {
      if (segments[segment0_handle].reach_ground == 1)
        segments[segment1_handle].reach_ground = 1;
      if (segments[segment1_handle].reach_ground == 1)
        segments[segment0_handle].reach_ground = 1;
      if (segments[segment0_handle].quasi_stable == 1)
        segments[segment1_handle].quasi_stable = 1;
      if (segments[segment1_handle].quasi_stable == 1)
        segments[segment0_handle].quasi_stable = 1;
    }
  }
}
