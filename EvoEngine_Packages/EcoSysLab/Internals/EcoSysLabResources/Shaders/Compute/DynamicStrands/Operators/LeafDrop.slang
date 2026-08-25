import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.Noise;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint leaf_size;
  float ground_height;
  float rotation_correction_strength;
  float air_resistance_strength;

  float3 disturbance_frequency;
  float disturbance_strength;
};

float3 calculate_torque(in float4 current_q, in float4 target_q) {
  float4 relative_q = quat_mul(conjugate(current_q), target_q);
  float angle = 2.f * acos(clamp(relative_q.w, -1.f, 1.f));
  float3 axis = float3(relative_q.x, relative_q.y, relative_q.z);

  float3 angular_v = angle * axis;
  return angular_v;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint leaf_index = baseID + local_index;

    if (leaf_index >= leaf_size)
      break;
    Leaf leaf = foliage[leaf_index];

    if (leaf.inv_mass != 0.0f) {
      float3 up = rotate_vec3(leaf.q, float3(0, 1, 0));
      float angle;
      float3 axis;
      CalculateAlignUpDirectionRotation(leaf.q, float3(0, 1, 0), angle, axis);
      leaf.torque += axis * angle * rotation_correction_strength;
      float l_a = length(leaf.acceleration);
      leaf.torque += float3(EE_SIMPLEX_NOISE(leaf.x * disturbance_frequency.x) * 2.f - 1.f,
                          EE_SIMPLEX_NOISE((leaf.x + float3(10.f, 10.f, 10.f)) * disturbance_frequency.y) * 2.f - 1.f,
                          EE_SIMPLEX_NOISE((leaf.x + float3(20.f, 20.f, 20.f)) * disturbance_frequency.z) * 2.f - 1.f) *
                     l_a * disturbance_strength;
      leaf.acceleration += up * l_a * air_resistance_strength;
    }
    foliage[leaf_index].acceleration = leaf.acceleration;
    foliage[leaf_index].torque = leaf.torque;
  }
}