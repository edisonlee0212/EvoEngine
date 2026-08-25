import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint leaf_size;
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
    if (leaf.attachment_integrity != 0.0f) {
      Segment segment = segments[leaf.segment_handle];
      float3 segment_center_position = (segment.particle0.x + segment.particle1.x) * 0.5f;
      float3 position_correction = segment_center_position + rotate_vec3(segment.q, leaf.position_offset) - leaf.x;
      position_correction /= 1.f + leaf.position_alpha;
      leaf.x += position_correction;
      leaf.q = quat_mul(segment.q, quat_mul(conjugate(segment.q0), leaf.q0));
    }
    foliage[leaf_index] = leaf;
  }
}