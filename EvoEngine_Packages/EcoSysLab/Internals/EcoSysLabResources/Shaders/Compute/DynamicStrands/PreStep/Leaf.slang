import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float3 acceleration;
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
    leaf.torque = float3(0.0, 0.0, 0.0);
    leaf.inv_mass = 1.f / (leaf.original_mass + leaf.extra_mass);
    leaf.rotation_strain = 0.0f;
    leaf.position_strain = 0.0f;
    leaf.acceleration = leaf.inv_mass != 0.0f
                            ? acceleration * (leaf.original_mass + leaf.extra_mass) / leaf.original_mass
                            : float3(0.0, 0.0, 0.0);
    leaf.extra_mass = 0.f;

    foliage[leaf_index] = leaf;
  }
}