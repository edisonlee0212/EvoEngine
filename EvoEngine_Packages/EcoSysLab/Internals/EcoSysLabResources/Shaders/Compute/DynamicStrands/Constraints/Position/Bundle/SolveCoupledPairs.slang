import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

struct PairState {
  float4 positional_lambda;
  float4 angular_lambda;
};

struct PairCorrection {
  float4 segment0_position;
  float4 segment0_angular;
  float4 segment1_position;
  float4 segment1_angular;
};

[[vk::binding(0, 1)]] RWStructuredBuffer<PairState, Std430DataLayout> pair_states;
[[vk::binding(1, 1)]] RWStructuredBuffer<PairCorrection, Std430DataLayout> pair_corrections;

[[vk::push_constant]] cbuffer PushConstants {
  uint pair_begin;
  uint pair_count;
  uint segment_count;
  uint reset_lambdas;
  float inverse_time_step_squared;
  float position_compliance_scale;
  float bending_compliance_scale;
  float torsion_compliance_scale;
};

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;

float3x3 cross_matrix(float3 v) {
  return float3x3(0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0);
}

float3 solve_matrix(float3x3 matrix, float3 rhs) {
  float3 c0 = matrix[0];
  float3 c1 = matrix[1];
  float3 c2 = matrix[2];
  float determinant = dot(c0, cross(c1, c2));
  if (abs(determinant) < 1e-12f)
    return float3(0);
  return float3(dot(rhs, cross(c1, c2)), dot(rhs, cross(c2, c0)), dot(rhs, cross(c0, c1))) / determinant;
}

float3 quaternion_log(float4 q) {
  if (q.w < 0)
    q = -q;
  float sine = length(q.xyz);
  return sine < 1e-7f ? 2.f * q.xyz : 2.f * atan2(sine, q.w) * q.xyz / sine;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  uint base_index = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]] for (uint task = 0; task < ITERATIONS_PER_TASK; ++task) {
    uint local_index = lane_id + task * WORKGROUP_SIZE;
    if (local_index + base_index >= pair_count)
      break;
    uint pair_index = pair_begin + base_index + local_index;
    SegmentPair pair = segment_pairs[pair_index];
    PairCorrection correction = (PairCorrection)0;
    float cohesion = pair.bend_twist_bundle_integrity;
    Segment segment0 = segments[pair.segment0_handle];
    Segment segment1 = segments[pair.segment1_handle];
    cohesion *= saturate(min(min(segment0.HC, segment1.HC), min(segment0.HL, segment1.HL)));
    cohesion *= saturate(1.f - max(max(segment0.RW, segment1.RW), max(segment0.RB, segment1.RB)));
    cohesion *= saturate(min(segment0.moisture, segment1.moisture));
    if (cohesion <= 1e-6f) {
      pair_states[pair_index] = (PairState)0;
      pair_corrections[pair_index] = correction;
      continue;
    }

    PairState state = pair_states[pair_index];
    if (reset_lambdas != 0)
      state = (PairState)0;
    float3 center0 = .5f * (segment_particle0s[pair.segment0_handle].x + segment_particle1s[pair.segment0_handle].x);
    float3 center1 = .5f * (segment_particle0s[pair.segment1_handle].x + segment_particle1s[pair.segment1_handle].x);
    float3 r0 = rotate_vec3(segment0.q, .5f * pair.segment1_offset.xyz);
    float3 r1 = rotate_vec3(segment1.q, .5f * pair.segment0_offset.xyz);
    float3x3 inverse_inertia0 = float3x3(segment0.inv_inertia_w);
    float3x3 inverse_inertia1 = float3x3(segment1.inv_inertia_w);
    float positional_alpha = position_compliance_scale * .5f *
                             (segment0.shear_stretch_alpha + segment1.shear_stretch_alpha) *
                             inverse_time_step_squared / cohesion;
    float3x3 effective_mass = (segment0.inv_mass + segment1.inv_mass + positional_alpha) * float3x3(1) -
                              mul(cross_matrix(r0), mul(inverse_inertia0, cross_matrix(r0))) -
                              mul(cross_matrix(r1), mul(inverse_inertia1, cross_matrix(r1)));
    float3 delta_lambda = solve_matrix(effective_mass,
                                       -(center0 + r0 - center1 - r1 + positional_alpha * state.positional_lambda.xyz));
    state.positional_lambda.xyz += delta_lambda;
    correction.segment0_position.xyz = segment0.inv_mass * delta_lambda;
    correction.segment1_position.xyz = -segment1.inv_mass * delta_lambda;
    correction.segment0_angular.xyz = mul(inverse_inertia0, cross(r0, delta_lambda));
    correction.segment1_angular.xyz = mul(inverse_inertia1, cross(r1, -delta_lambda));

    float4 relative = quat_mul(conjugate(segment0.q), segment1.q);
    float3 angular_error_local = quaternion_log(quat_mul(relative, conjugate(pair.rest_darboux_vector)));
    float3 angular_error = rotate_vec3(segment0.q, angular_error_local);
    float3 local_alpha = inverse_time_step_squared / cohesion *
                         float3(bending_compliance_scale * pair.bending_alpha,
                                bending_compliance_scale * pair.bending_alpha,
                                torsion_compliance_scale * pair.twisting_alpha);
    float3x3 rotation = mat3_cast(segment0.q);
    float3x3 angular_alpha = mul(rotation, mul(float3x3(local_alpha.x, 0, 0, 0, local_alpha.y, 0, 0, 0,
                                                       local_alpha.z), transpose(rotation)));
    float3 angular_delta = solve_matrix(inverse_inertia0 + inverse_inertia1 + angular_alpha,
                                        -(angular_error + mul(angular_alpha, state.angular_lambda.xyz)));
    state.angular_lambda.xyz += angular_delta;
    correction.segment0_angular.xyz += mul(inverse_inertia0, angular_delta);
    correction.segment1_angular.xyz -= mul(inverse_inertia1, angular_delta);
    pair_states[pair_index] = state;
    pair_corrections[pair_index] = correction;
  }
}
