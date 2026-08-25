import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
struct ExternalForce {
  float4 force_index;
};

[[vk::binding(0, 1)]] StructuredBuffer<ExternalForce, Std430DataLayout> external_forces;

[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint command_size;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint global_index = baseID + local_index;

    if (global_index >= command_size)
      break;

    ExternalForce external_force = external_forces[global_index];
    uint segment_index = asuint(external_force.force_index.w);
    float3 force = external_force.force_index.xyz;
    Segment segment = segments[segment_index];
    segments[segment_index].particle0.acceleration.xyz += force * segment.inv_mass;
    segments[segment_index].particle1.acceleration.xyz += force * segment.inv_mass;
  }
}
