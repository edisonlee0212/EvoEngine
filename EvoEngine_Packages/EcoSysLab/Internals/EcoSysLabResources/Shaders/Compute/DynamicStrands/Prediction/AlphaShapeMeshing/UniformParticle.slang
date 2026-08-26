import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;

#define EPSILON 1e-3
static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint uniform_particle_size;
  float snow_factor;
  float snow_deduction;
  int use_cubic_hermite_spline;
};

float3 CubicHermiteSpline(in float3 P0, in float3 P1, in float3 M0, in float3 M1, float t) {
  float t2 = t * t;
  float t3 = t2 * t;

  float h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  float h10 = t3 - 2.0 * t2 + t;
  float h01 = -2.0 * t3 + 3.0 * t2;
  float h11 = t3 - t2;

  return h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1;
}

float3 CubicHermiteSplineTangent(in float3 P0, in float3 P1, in float3 M0, in float3 M1, float t) {
  float t2 = t * t;

  float h00 = 6.0 * t2 - 6.0 * t;
  float h10 = 3.0 * t2 - 4.0 * t + 1.0;
  float h01 = -6.0 * t2 + 6.0 * t;
  float h11 = 3.0 * t2 - 2.0 * t;

  return h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint uniform_particle_index = baseID + local_index;

    if (uniform_particle_index >= uniform_particle_size)
      break;
    UniformParticle uniform_particle = uniform_particles[uniform_particle_index];

    float3 p0 = segment_particle0s[uniform_particle.segment_handle].x;
    float3 p1 = segment_particle1s[uniform_particle.segment_handle].x;

    float3 normal = normalize(uniform_particle.normal_deg.xyz);
    float local_extrusion_distance = uniform_particle.local_extrusion_distance;
    float snow_property = segments[uniform_particle.segment_handle].snow_amount;

    if (segments[uniform_particle.segment_handle].group_index == 0) {
      local_extrusion_distance = normal.y > max(1e-6f, 1.f - snow_property * snow_factor) ? snow_property : 0.f;
      uniform_particles[uniform_particle_index].local_extrusion_distance = local_extrusion_distance;
    } else {
      local_extrusion_distance -= snow_deduction;
    }
    uniform_particles[uniform_particle_index].override_color =
        local_extrusion_distance > 1e-6f ? float4(1, 0, 0, 0) : float4(0, 0, 0, 0);
    // use cubic hermite spline interpolation to update uniform particles

    // particle0 tangent
    int prev_segment_handle = segments[uniform_particle.segment_handle].prev_handle;
    float3 M0;
    if (prev_segment_handle == -1) {
      // start point tangent
      M0 = 0.5 * (p1 - p0);
    } else if (distance(segment_particle1s[prev_segment_handle].x, p0) > EPSILON) {
      // strand is broken here
      M0 = 0.5 * (p1 - p0);
    } else {
      float3 p_prev = segment_particle0s[prev_segment_handle].x;
      M0 = 0.5 * (p1 - p_prev);
    }

    // particle1 tangent
    int next_segment_handle = segments[uniform_particle.segment_handle].next_handle;
    float3 M1;
    if (next_segment_handle == -1) {
      // end point tangent
      M1 = 0.5 * (p1 - p0);
    } else if (distance(segment_particle0s[next_segment_handle].x, p1) > EPSILON) {
      // strand is broken here
      M1 = 0.5 * (p1 - p0);
    } else {
      float3 p_next = segment_particle1s[next_segment_handle].x;
      M1 = 0.5 * (p_next - p0);
    }

    if (use_cubic_hermite_spline == 1) {
      uniform_particles[uniform_particle_index].position_t.xyz =
          CubicHermiteSpline(p0, p1, M0, M1, uniform_particle.position_t.w);
      uniform_particles[uniform_particle_index].tangent.xyz =
          CubicHermiteSplineTangent(p0, p1, M0, M1, uniform_particle.position_t.w);
    } else {
      uniform_particles[uniform_particle_index].position_t.xyz = lerp(p0, p1, uniform_particle.position_t.w);
      uniform_particles[uniform_particle_index].tangent.xyz =
          rotate_vec3(segments[uniform_particle.segment_handle].q, float3(0, 0, 1));
    }
  }
}
