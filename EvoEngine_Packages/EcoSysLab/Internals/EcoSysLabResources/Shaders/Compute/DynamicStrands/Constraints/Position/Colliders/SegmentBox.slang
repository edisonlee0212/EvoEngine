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
  uint segment_size;
  float friction;
  float rotational_friction;
};

void process_point(inout float3 p0, inout float3 p1, inout float4 q, in float3 v0, in float3 v1, inout float3 torque,
                   inout float3 acceleration0, inout float3 acceleration1) {
  float3 projected_p0 = p0 - obb_center.xyz;
  projected_p0 = rotate_vec3(conjugate(obb_rotation), projected_p0);
  float3 projected_v0 = rotate_vec3(conjugate(obb_rotation), v0);

  float abs_x = abs(projected_p0.x);
  float abs_y = abs(projected_p0.y);
  float abs_z = abs(projected_p0.z);
  bool update_torque = false;
  if (abs_x < obb_scale.x && abs_y < obb_scale.y && abs_z < obb_scale.z) {
    float factor_x = obb_scale.x - abs_x;
    float factor_y = obb_scale.y - abs_y;
    float factor_z = obb_scale.z - abs_z;

    if (factor_x <= factor_y && factor_x <= factor_z) {
      if (projected_p0.x < 0.f) {
        projected_p0.x -= softness * (obb_scale.x - abs_x);
      } else {
        projected_p0.x += softness * (obb_scale.x - abs_x);
      }
      acceleration0 -= length(float3(0.f, projected_v0.y, projected_v0.z)) > 1e-9f
                           ? friction * normalize(rotate_vec3(obb_rotation, float3(0.f, projected_v0.y, projected_v0.z)))
                           : float3(0.f);
    } else if (factor_y <= factor_z) {
      if (projected_p0.y < 0.f) {
        projected_p0.y -= softness * (obb_scale.y - abs_y);
      } else {
        projected_p0.y += softness * (obb_scale.y - abs_y);
      }
      acceleration0 -= length(float3(projected_v0.x, 0.f, projected_v0.z)) > 1e-9f
                           ? friction * normalize(rotate_vec3(obb_rotation, float3(projected_v0.x, 0.f, projected_v0.z)))
                           : float3(0.f);
    } else {
      if (projected_p0.z < 0.f) {
        projected_p0.z -= softness * (obb_scale.z - abs_z);
      } else {
        projected_p0.z += softness * (obb_scale.z - abs_z);
      }
      acceleration0 -= length(float3(projected_v0.x, projected_v0.y, 0.f)) > 1e-9f
                           ? friction * normalize(rotate_vec3(obb_rotation, float3(projected_v0.x, projected_v0.y, 0.f)))
                           : float3(0.f);
    }
    p0 = rotate_vec3(obb_rotation, projected_p0) + obb_center.xyz;
    update_torque = true;
  }

  float3 projected_p1 = p1 - obb_center.xyz;
  projected_p1 = rotate_vec3(conjugate(obb_rotation), projected_p1);
  float3 projected_v1 = rotate_vec3(conjugate(obb_rotation), v1);

  abs_x = abs(projected_p1.x);
  abs_y = abs(projected_p1.y);
  abs_z = abs(projected_p1.z);

  if (abs_x < obb_scale.x && abs_y < obb_scale.y && abs_z < obb_scale.z) {
    float factor_x = obb_scale.x - abs_x;
    float factor_y = obb_scale.y - abs_y;
    float factor_z = obb_scale.z - abs_z;

    if (factor_x <= factor_y && factor_x <= factor_z) {
      if (projected_p1.x < 0.f) {
        projected_p1.x -= softness * (obb_scale.x - abs_x);
      } else {
        projected_p1.x += softness * (obb_scale.x - abs_x);
      }
      acceleration1 -= length(float3(0.f, projected_v1.y, projected_v1.z)) > 1e-9f
                           ? friction * normalize(rotate_vec3(obb_rotation, float3(0.f, projected_v1.y, projected_v1.z)))
                           : float3(0.f);
    } else if (factor_y <= factor_z) {
      if (projected_p1.y < 0.f) {
        projected_p1.y -= softness * (obb_scale.y - abs_y);
      } else {
        projected_p1.y += softness * (obb_scale.y - abs_y);
      }
      acceleration1 -= length(float3(projected_v1.x, 0.f, projected_v1.z)) > 1e-9f
                           ? friction * normalize(rotate_vec3(obb_rotation, float3(projected_v1.x, 0.f, projected_v1.z)))
                           : float3(0.f);
    } else {
      if (projected_p1.z < 0.f) {
        projected_p1.z -= softness * (obb_scale.z - abs_z);
      } else {
        projected_p1.z += softness * (obb_scale.z - abs_z);
      }
      acceleration1 -= length(float3(projected_v1.x, projected_v1.y, 0.f)) > 1e-9f
                           ? friction * normalize(rotate_vec3(obb_rotation, float3(projected_v1.x, projected_v1.y, 0.f)))
                           : float3(0.f);
    }
    p1 = rotate_vec3(obb_rotation, projected_p1) + obb_center.xyz;
    update_torque = true;
  }
  /*
  if(update_torque){
          float3 front = normalize(p0 - p1);
          float angle;
          float3 axis;
          CalculateAlignFrontDirectionRotation(q, front, angle, axis);
          torque = axis * angle;
  }*/
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
    Particle segment_particle0 = segment_particle0s[segment_index];
    Particle segment_particle1 = segment_particle1s[segment_index];
    process_point(segment_particle0.x, segment_particle1.x, segment.q, segment_particle0.v, segment_particle1.v,
                  segment.torque, segment_particle0.acceleration, segment_particle1.acceleration);
    segments[segment_index].torque = segment.torque;
    segment_particle0s[segment_index].x = segment_particle0.x;
    segment_particle1s[segment_index].x = segment_particle1.x;
    segment_particle0s[segment_index].acceleration = segment_particle0.acceleration;
    segment_particle1s[segment_index].acceleration = segment_particle1.acceleration;
  }
}
