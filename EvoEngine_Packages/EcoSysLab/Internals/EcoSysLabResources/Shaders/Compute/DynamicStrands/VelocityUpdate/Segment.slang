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

  uint segment_size;
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
    uint segment_index = baseID + local_index;

    if (segment_index >= segment_size)
      break;

    Segment segment = segments[segment_index];
    update_inertia_w(segment.q, segment.inertia_tensor, segment.inv_inertia_tensor, segment.inertia_w,
                     segment.inv_inertia_w);

    // Calculate angular velocity and apply torque.
    float3 angular_velocity = float3(0, 0, 0);
    float3 p0_velocity = float3(0.0f, 0.0f, 0.0f);
    float3 p1_velocity = float3(0.0f, 0.0f, 0.0f);
    if (segment.inv_mass != 0.0f) {
      float4 rot = quat_mul(segment.q, conjugate(segment.last_q));
      angular_velocity = rot.xyz * 2.0f * inv_time_step;
      p0_velocity = inv_time_step * (segment.particle0.x - segment.particle0.last_x);
      p1_velocity = inv_time_step * (segment.particle1.x - segment.particle1.last_x);

      angular_velocity *= max(0.f, 1.f - angular_velocity_damping * length(angular_velocity) * time_step);
      angular_velocity += time_step *
                          mul(float3x3(segment.inv_inertia_w),
                              segment.torque -
                                  cross(angular_velocity, mul(float3x3(segment.inertia_w), angular_velocity)));

      p0_velocity *= max(0.f, 1.f - velocity_damping * length(p0_velocity) * time_step);
      p0_velocity += segment.particle0.acceleration.xyz * time_step;

      p1_velocity *= max(0.f, 1.f - velocity_damping * length(p1_velocity) * time_step);
      p1_velocity += segment.particle1.acceleration.xyz * time_step;
    }

    segments[segment_index].angular_v = min(max_angular_velocity, angular_velocity);
    segments[segment_index].particle0.v = min(max_velocity, p0_velocity);
    segments[segment_index].particle1.v = min(max_velocity, p1_velocity);

    float3 up = float3(0.0, 1.0, 0.0);
    float3 down = float3(0.0, -1.0, 0.0);
    float friction = 0.8 * 9.8;
    if (segments[segment_index].reach_ground == 1) {  // Todo: Enhanced Friction Model
      segments[segment_index].particle0.v = scaleAlongDirectional(segments[segment_index].particle0.v, up, 0.1);
      segments[segment_index].particle1.v = scaleAlongDirectional(segments[segment_index].particle1.v, up, 0.1);

      // if (segments[segment_index].particle0.padding1 == 0)
      //       {
      //                  segments[segment_index].particle0.v =
      //                      scaleAlongDirectional(segments[segment_index].particle0.v, up, 0.02);
      //    segments[segment_index].particle0.padding1 = 1;
      //}
      //              if (segments[segment_index].particle1.padding1 == 0)
      //{
      //    segments[segment_index].particle1.v =
      //                    scaleAlongDirectional(segments[segment_index].particle1.v, up, 0.02);
      //                  segments[segment_index].particle1.padding1 = 1;
      //}
      segments[segment_index].particle1.acceleration -=
          length(float3(segments[segment_index].particle1.v.x, segments[segment_index].particle1.v.y, 0.f)) > 1e-9f
              ? normalize(segments[segment_index].particle1.v) * friction
              : float3(0.0);
      segments[segment_index].particle0.acceleration -=
          length(float3(segments[segment_index].particle0.v.x, segments[segment_index].particle0.v.y, 0.f)) > 1e-9f
              ? normalize(segments[segment_index].particle0.v) * friction
              : float3(0.0);
    }
    /*if (segments[segment_index].quasi_stable == 1) {
      segments[segment_index].particle0.v =
          scaleAlongDirectional(segments[segment_index].particle0.v, down, 0.2);
      segments[segment_index].particle1.v =
          scaleAlongDirectional(segments[segment_index].particle1.v, down, 0.2);
    }*/
  }
}
