import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float4 obb_rotation;
  float4 obb_center;
  float radius;
  float height;
  float softness;
  uint leaf_size;
};

float3 process_point(in float3 p) {
  float3 projected_p = p - obb_center.xyz;
  projected_p = rotate_vec3(conjugate(obb_rotation), projected_p);
  float current_height = abs(projected_p.y);
  if (height - current_height < 0.f)
    return p;
  float current_radius = length(projected_p.xz);
  if (radius - current_radius < 0.f)
    return p;
  float radius_diff = radius - current_radius;
  float height_diff = height - current_height;
  if (radius_diff > height_diff) {
    if (projected_p.y >= 0.f)
      projected_p += float3(0, height_diff, 0);
    else
      projected_p -= float3(0, height_diff, 0);
  } else {
    float2 direction = float2(0, 1);
    if (current_radius > 0.f) {
      direction = normalize(projected_p.xz);
    }
    float2 push = radius_diff * direction;
    projected_p += float3(push.x, 0.f, push.y);
  }
  return rotate_vec3(obb_rotation, projected_p) + obb_center.xyz;
}

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
    leaf.x = process_point(leaf.x);
    foliage[leaf_index].x = leaf.x;
  }
}