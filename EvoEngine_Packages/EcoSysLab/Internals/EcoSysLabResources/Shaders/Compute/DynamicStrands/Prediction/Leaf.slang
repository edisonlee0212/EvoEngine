import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint leaf_size;
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
    uint leaf_index = baseID + local_index;

    if (leaf_index >= leaf_size)
      break;

    Leaf leaf = foliage[leaf_index];

    // Shift rotation values
    leaf.last_q = leaf.q;
    // Apply angular velocity
    float4 angular_velocity_q = float4(leaf.angular_v, 0.0f);

    leaf.q += 0.5f * time_step * quat_mul(angular_velocity_q, leaf.q);
    leaf.q = normalize(leaf.q);

    // Update inertia
    update_inertia_w(leaf.q, leaf.inertia_tensor, leaf.inv_inertia_tensor, leaf.inertia_w, leaf.inv_inertia_w);

    // Shift position values
    leaf.last_x = leaf.x;
    // Apply velocity
    leaf.x += time_step * leaf.v;

    if (leaf.attachment_integrity != 0.0f) {
      Segment segment = segments[leaf.segment_handle];
      float3 segment_center_position = (segment.particle0.x + segment.particle1.x) * 0.5f;
      float3 desired_position = segment_center_position + rotate_vec3(segment.q, leaf.position_offset);

      float offset_length = length(leaf.position_offset);
      leaf.position_strain = length(desired_position - leaf.x) / offset_length * inv_time_step;
    }

    foliage[leaf_index] = leaf;
  }
}