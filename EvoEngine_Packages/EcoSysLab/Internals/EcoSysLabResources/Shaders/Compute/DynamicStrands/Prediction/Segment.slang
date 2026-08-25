import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
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

    // Shift rotation values
    segment.last_q = segment.q;
    // Apply angular velocity
    float4 angular_velocity_q = float4(segment.angular_v, 0.0f);

    segment.q += 0.5f * time_step * quat_mul(angular_velocity_q, segment.q);
    segment.q = normalize(segment.q);

    // Update w
    update_inertia_w(segment.q, segment.inertia_tensor, segment.inv_inertia_tensor, segment.inertia_w,
                     segment.inv_inertia_w);

    // Shift position values
    segment.particle0.last_x = segment.particle0.x;
    // Apply velocity
    segment.particle0.x += time_step * segment.particle0.v;

    if (segment.quasi_stable == 1 && length(segment.particle0.v) < 5e-3) {
      segment.particle0.x = segment.particle0.last_x;
    }

    // Shift position values
    segment.particle1.last_x = segment.particle1.x;
    // Apply velocity
    segment.particle1.x += time_step * segment.particle1.v;

    if (segment.quasi_stable == 1 && length(segment.particle1.v) < 5e-3)  // 5e-3
    {
      segment.particle1.x = segment.particle1.last_x;
    }

    float shear_stretch_strain =
        shear_stretch_strain(segment.particle0.x, segment.particle1.x, segment.q, segment.rest_length);

    segment.shear_stretch_strain = shear_stretch_strain * inv_time_step;
    // Update moisture content.

    // Update shear_stretch_strain_limit
    segment.shear_stretch_strain_limit = segment.max_shear_stretch_strain * segment.strength;
    segments[segment_index] = segment;
  }
}