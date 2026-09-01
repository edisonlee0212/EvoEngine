import EcoSysLab.Math;

struct SliceTransform {
  float4 rest_center;
  float4 center;
  float4 rotation;
  float4 inverse_mass;
  float4 inverse_inertia0;
  float4 inverse_inertia1;
  float4 inverse_inertia2;
};

struct CoarseEdge {
  uint slice0;
  uint slice1;
  uint count;
  uint padding;
  float4 compliance_cohesion;
  float4 positional_lambda;
  float4 angular_lambda;
};

[[vk::binding(6, 1)]] RWStructuredBuffer<SliceTransform, Std430DataLayout> transforms;
[[vk::binding(10, 1)]] RWStructuredBuffer<CoarseEdge, Std430DataLayout> edges;
[[vk::binding(11, 1)]] StructuredBuffer<uint, Std430DataLayout> edge_count;

[[vk::push_constant]] cbuffer PushConstants {
  uint direct_pair_count;
  uint padded_count;
  uint sort_stage;
  uint sort_pass;
  float inverse_time_step_squared;
  float position_compliance_scale;
  float bending_compliance_scale;
  float torsion_compliance_scale;
};

float3x3 cross_matrix(float3 v) {
  return float3x3(0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0);
}

float3 solve_matrix(float3x3 matrix, float3 rhs) {
  float3 c0 = matrix[0];
  float3 c1 = matrix[1];
  float3 c2 = matrix[2];
  float determinant_value = dot(c0, cross(c1, c2));
  if (abs(determinant_value) < 1e-12f)
    return float3(0);
  return float3(dot(rhs, cross(c1, c2)), dot(rhs, cross(c2, c0)), dot(rhs, cross(c0, c1))) /
         determinant_value;
}

float3 quaternion_log(float4 q) {
  if (q.w < 0)
    q = -q;
  float sine = length(q.xyz);
  return sine < 1e-7f ? 2.f * q.xyz : 2.f * atan2(sine, q.w) * q.xyz / sine;
}

void apply_rotation(inout SliceTransform transform, float3 angular) {
  float angle = length(angular);
  if (angle > 1e-8f)
    transform.rotation = normalize(quat_mul(angle_axis(angle, angular / angle), transform.rotation));
}

[shader("compute")]
[numthreads(1, 1, 1)]
void main() {
  for (uint edge_index = 0; edge_index < edge_count[0]; ++edge_index) {
    CoarseEdge edge = edges[edge_index];
    float cohesion = edge.compliance_cohesion.w;
    if (cohesion <= 1e-6f)
      continue;
    SliceTransform body0 = transforms[edge.slice0];
    SliceTransform body1 = transforms[edge.slice1];
    if (body0.rest_center.w == 0 || body1.rest_center.w == 0)
      continue;
    float3x3 inverse_inertia0 =
        float3x3(body0.inverse_inertia0.xyz, body0.inverse_inertia1.xyz, body0.inverse_inertia2.xyz);
    float3x3 inverse_inertia1 =
        float3x3(body1.inverse_inertia0.xyz, body1.inverse_inertia1.xyz, body1.inverse_inertia2.xyz);
    float3 rest_delta = body1.rest_center.xyz - body0.rest_center.xyz;
    float3 arm0 = rotate_vec3(body0.rotation, .5f * rest_delta);
    float3 arm1 = rotate_vec3(body1.rotation, -.5f * rest_delta);
    float positional_alpha = position_compliance_scale * edge.compliance_cohesion.x *
                             inverse_time_step_squared / cohesion;
    float3x3 effective_mass =
        (body0.inverse_mass.x + body1.inverse_mass.x + positional_alpha) * float3x3(1) -
        mul(cross_matrix(arm0), mul(inverse_inertia0, cross_matrix(arm0))) -
        mul(cross_matrix(arm1), mul(inverse_inertia1, cross_matrix(arm1)));
    float3 error = body0.center.xyz + arm0 - body1.center.xyz - arm1;
    float3 delta_lambda =
        solve_matrix(effective_mass, -(error + positional_alpha * edge.positional_lambda.xyz));
    edge.positional_lambda.xyz += delta_lambda;
    body0.center.xyz += body0.inverse_mass.x * delta_lambda;
    body1.center.xyz -= body1.inverse_mass.x * delta_lambda;
    apply_rotation(body0, mul(inverse_inertia0, cross(arm0, delta_lambda)));
    apply_rotation(body1, mul(inverse_inertia1, cross(arm1, -delta_lambda)));

    float3 angular_error =
        rotate_vec3(body0.rotation, quaternion_log(quat_mul(conjugate(body0.rotation), body1.rotation)));
    float3 local_alpha = inverse_time_step_squared / cohesion *
                         float3(bending_compliance_scale * edge.compliance_cohesion.y,
                                bending_compliance_scale * edge.compliance_cohesion.y,
                                torsion_compliance_scale * edge.compliance_cohesion.z);
    float3x3 rotation = mat3_cast(body0.rotation);
    float3x3 angular_alpha = mul(rotation, mul(float3x3(local_alpha.x, 0, 0, 0, local_alpha.y, 0, 0, 0,
                                                       local_alpha.z), transpose(rotation)));
    float3 angular_delta = solve_matrix(inverse_inertia0 + inverse_inertia1 + angular_alpha,
                                        -(angular_error + mul(angular_alpha, edge.angular_lambda.xyz)));
    edge.angular_lambda.xyz += angular_delta;
    apply_rotation(body0, mul(inverse_inertia0, angular_delta));
    apply_rotation(body1, mul(inverse_inertia1, -angular_delta));
    transforms[edge.slice0] = body0;
    transforms[edge.slice1] = body1;
    edges[edge_index] = edge;
  }
}
