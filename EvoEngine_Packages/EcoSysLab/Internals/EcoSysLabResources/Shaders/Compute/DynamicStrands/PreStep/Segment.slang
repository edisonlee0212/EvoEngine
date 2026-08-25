import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float3 acceleration;
  uint segment_size;
  float time_step;
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
    uint segment_index = baseID + local_index;

    if (segment_index >= segment_size)
      break;

    Segment segment = segments[segment_index];
    segment.torque = float3(0.0, 0.0, 0.0);
    segment.inv_mass = 1.f / (segment.original_mass + segment.extra_mass);

    segment.shear_stretch_strain = 0.0f;
    segment.particle0.acceleration =
        segment.inv_mass != 0.0f ? acceleration * (segment.original_mass + segment.extra_mass) / segment.original_mass
                                 : float3(0.0, 0.0, 0.0);
    segment.particle1.acceleration =
        segment.inv_mass != 0.0f ? acceleration * (segment.original_mass + segment.extra_mass) / segment.original_mass
                                 : float3(0.0, 0.0, 0.0);

    segment.extra_mass = 0.f;
    segments[segment_index] = segment;
  }
}