import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float4 obb_rotation;
  float4 obb_center;
  float4 obb_scale;
  float softness;
  uint leaf_size;
  float friction;
  float rotational_friction;
};

float3 project_velocity_on_plane(in float3 n, in float3 v) {
  float dist = dot(v, n);
  return v - dist * n;
}

void process_point(inout float3 p, in float4 q, in float3 v, inout float3 torque, inout float3 acceleration) {
  float3 projected_p = p - obb_center.xyz;
  projected_p = rotate_vec3(conjugate(obb_rotation), projected_p);
  float3 projected_v = rotate_vec3(conjugate(obb_rotation), v);

  float abs_x = abs(projected_p.x);
  float abs_y = abs(projected_p.y);
  float abs_z = abs(projected_p.z);

  if (abs_x > obb_scale.x || abs_y > obb_scale.y || abs_z > obb_scale.z)
    return;
  float factor_x = obb_scale.x - abs_x;
  float factor_y = obb_scale.y - abs_y;
  float factor_z = obb_scale.z - abs_z;

  float3 axis;
  float angle;
  if (factor_x <= factor_y && factor_x <= factor_z) {
    if (projected_p.x < 0.f) {
      projected_p.x -= softness * (obb_scale.x - abs_x);
      CalculateAlignUpDirectionRotation(q, normalize(rotate_vec3(obb_rotation, float3(-1, 0, 0))), angle, axis);
    } else {
      projected_p.x += softness * (obb_scale.x - abs_x);
      CalculateAlignUpDirectionRotation(q, normalize(rotate_vec3(obb_rotation, float3(1, 0, 0))), angle, axis);
    }
    acceleration -= length(float3(0.f, projected_v.y, projected_v.z)) > 1e-9f
                        ? friction * normalize(rotate_vec3(obb_rotation, float3(0.f, projected_v.y, projected_v.z)))
                        : float3(0.f);
  } else if (factor_y <= factor_z) {
    if (projected_p.y < 0.f) {
      projected_p.y -= softness * (obb_scale.y - abs_y);
      CalculateAlignUpDirectionRotation(q, normalize(rotate_vec3(obb_rotation, float3(0, -1, 0))), angle, axis);
    } else {
      projected_p.y += softness * (obb_scale.y - abs_y);
      CalculateAlignUpDirectionRotation(q, normalize(rotate_vec3(obb_rotation, float3(0, 1, 0))), angle, axis);
    }
    acceleration -= length(float3(projected_v.x, 0.f, projected_v.z)) > 1e-9f
                        ? friction * normalize(rotate_vec3(obb_rotation, float3(projected_v.x, 0.f, projected_v.z)))
                        : float3(0.f);
  } else {
    if (projected_p.z < 0.f) {
      projected_p.z -= softness * (obb_scale.z - abs_z);
      CalculateAlignUpDirectionRotation(q, normalize(rotate_vec3(obb_rotation, float3(0, 0, -1))), angle, axis);
    } else {
      projected_p.z += softness * (obb_scale.z - abs_z);
      CalculateAlignUpDirectionRotation(q, normalize(rotate_vec3(obb_rotation, float3(0, 0, 1))), angle, axis);
    }
    acceleration -= length(float3(projected_v.x, projected_v.y, 0.f)) > 1e-9f
                        ? friction * normalize(rotate_vec3(obb_rotation, float3(projected_v.x, projected_v.y, 0.f)))
                        : float3(0.f);
  }
  p = rotate_vec3(obb_rotation, projected_p) + obb_center.xyz;
  torque = axis * angle;
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
    process_point(leaf.x, leaf.q, leaf.v, leaf.torque, leaf.acceleration);
    foliage[leaf_index].x = leaf.x;
    foliage[leaf_index].torque = leaf.torque;
    foliage[leaf_index].acceleration = leaf.acceleration;
  }
}