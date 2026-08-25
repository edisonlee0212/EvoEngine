import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float4 obb_center;
  float radius;
  float softness;
  uint segment_size;
};

float3 process_point(in float3 p) {
  float3 projected_p = p - obb_center.xyz;
  float current_radius = length(projected_p);
  if (current_radius == 0.0f)
    return obb_center.xyz + radius * float3(0, 1, 0);
  return obb_center.xyz + max(current_radius, radius) * normalize(projected_p);
}

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
    Particle particle0 = segment.particle0;
    Particle particle1 = segment.particle1;

    float3 p0 = particle0.x;
    float3 p1 = particle1.x;

    p0 = process_point(p0);
    p1 = process_point(p1);

    segments[segment_index].particle0.x = p0;
    segments[segment_index].particle1.x = p1;
  }
}