import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float4 obb_rotation;
  float4 obb_center;
  float4 obb_scale;
  float velocity_friction;
  float angular_velocity_friction;
  uint segment_size;
};

float3 project_velocity_on_plane(in float3 n, in float3 v) {
  float dist = dot(v, n);
  return v - dist * n;
}

void process_velocity(in float3 p, inout float3 v, inout float3 angular_v) {
  float3 x_axis = rotate_vec3(obb_rotation, float3(1, 0, 0));
  float3 y_axis = rotate_vec3(obb_rotation, float3(0, 1, 0));
  float3 z_axis = rotate_vec3(obb_rotation, float3(0, 0, 1));

  float3 projected_p = p - obb_center.xyz;
  projected_p = rotate_vec3(conjugate(obb_rotation), projected_p);
  float abs_x = abs(projected_p.x);
  float abs_y = abs(projected_p.y);
  float abs_z = abs(projected_p.z);

  float3 projected_v = rotate_vec3(conjugate(obb_rotation), v);

  if (abs_x > obb_scale.x || abs_y > obb_scale.y || abs_z > obb_scale.z)
    return;
  /*
  float factor_x = obb_scale.x - abs_x;
  float factor_y = obb_scale.y - abs_y;
  float factor_z = obb_scale.z - abs_z;
  if(factor_x <= factor_y && factor_x <= factor_z){
          projected_v.y *= 1.f - velocity_friction;
          projected_v.z *= 1.f - velocity_friction;
  }else if(factor_y <= factor_z){
          projected_v.x *= 1.f - velocity_friction;
          projected_v.z *= 1.f - velocity_friction;
  }else{
          projected_v.x *= 1.f - velocity_friction;
          projected_v.y *= 1.f - velocity_friction;
  }
  v = rotate_vec3(obb_rotation, projected_v);*/
  v *= 1.f - velocity_friction;
  angular_v *= 1.f - angular_velocity_friction;
  v *= 0.5f;
  angular_v *= 0.5f;
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

    process_velocity(segment_particle0.x, segment_particle0.v, segment.angular_v);
    process_velocity(segment_particle1.x, segment_particle1.v, segment.angular_v);

    segment_particle0s[segment_index].v = segment_particle0.v;
    segment_particle1s[segment_index].v = segment_particle1.v;
    segments[segment_index].angular_v = segment.angular_v;
  }
}
