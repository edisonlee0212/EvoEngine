import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint skip_index;
  uint skip_size;
  uint segment_size;
  uint treespace;
  float inv_time_step;
  float over_relaxation;
  float crack_bd_factor_offset;
  float crack_R_scale;
  float crack_T_scale;
  uint internal_pattern;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_index = (baseID + local_index) * skip_size + skip_index;
    if (segment_index >= segment_size)
      break;
    BundleSegmentPosition(segment_index, inv_time_step, over_relaxation, crack_bd_factor_offset, crack_R_scale,
                          crack_T_scale, treespace, internal_pattern);
  }
}