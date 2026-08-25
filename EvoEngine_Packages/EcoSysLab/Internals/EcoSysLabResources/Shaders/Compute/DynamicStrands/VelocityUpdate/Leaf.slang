import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float3 max_angular_velocity;
  float time_step;
  float3 max_velocity;
  float inv_time_step;

  uint leaf_size;
  float angular_velocity_damping;
  float velocity_damping;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint leaf_index = baseID + local_index;

    if (leaf_index >= leaf_size)
      break;

    Leaf leaf = foliage[leaf_index];
    update_inertia_w(leaf.q, leaf.inertia_tensor, leaf.inv_inertia_tensor, leaf.inertia_w, leaf.inv_inertia_w);

    // Calculate angular velocity and apply torque.
    float3 angular_velocity = float3(0, 0, 0);
    float3 velocity = float3(0.0f, 0.0f, 0.0f);
    if (leaf.inv_mass != 0.0f) {
      float4 rot = quat_mul(leaf.q, conjugate(leaf.last_q));
      angular_velocity = rot.xyz * 2.0f * inv_time_step;
      velocity = inv_time_step * (leaf.x - leaf.last_x);

      angular_velocity *= max(0.f, 1.f - angular_velocity_damping * length(angular_velocity) * time_step);
      angular_velocity +=
          time_step * mul(float3x3(leaf.inv_inertia_w),
                          leaf.torque - cross(angular_velocity, mul(float3x3(leaf.inertia_w), angular_velocity)));

      velocity *= max(0.f, 1.f - velocity_damping * length(velocity) * time_step);
      velocity += leaf.acceleration.xyz * time_step;
    }

    foliage[leaf_index].angular_v = min(max_angular_velocity, angular_velocity);
    foliage[leaf_index].v = min(max_velocity, velocity);
  }
}
