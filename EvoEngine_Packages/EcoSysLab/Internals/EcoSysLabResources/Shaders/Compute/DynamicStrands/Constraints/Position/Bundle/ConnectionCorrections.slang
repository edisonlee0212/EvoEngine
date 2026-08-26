import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint segment_pair_size;
  float inv_time_step;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_pair_index = baseID + local_index;

    if (segment_pair_index >= segment_pair_size)
      break;

    if (segment_pairs[segment_pair_index].connectivity_integrity <= 0.f)
      continue;
    int segment0_handle = segment_pairs[segment_pair_index].segment0_handle;
    int segment1_handle = segment_pairs[segment_pair_index].segment1_handle;
    float inv_mass0 = segments[segment0_handle].inv_mass;
    float inv_mass1 = segments[segment1_handle].inv_mass;
    float3 new_position =
        (segment_particle1s[segment0_handle].x * inv_mass1 + segment_particle0s[segment1_handle].x * inv_mass0) /
        (inv_mass0 + inv_mass1);
    segment_particle1s[segment0_handle].x = new_position;
    segment_particle0s[segment1_handle].x = new_position;
  }
}
