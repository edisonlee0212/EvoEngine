import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
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

[[vk::binding(5, 1)]] StructuredBuffer<uint, Std430DataLayout> segment_slices;
[[vk::binding(6, 1)]] StructuredBuffer<SliceTransform, Std430DataLayout> slice_transforms;

[[vk::push_constant]] cbuffer PushConstants {
  uint segment_count;
  uint padded_count;
  uint minimum_members;
  uint sort_stage;
  uint sort_pass;
  float shape_matching_strength;
  uint padding0;
  uint padding1;
};

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint segment_index : SV_DispatchThreadID) {
  if (segment_index >= segment_count)
    return;
  Segment segment = segments[segment_index];
  if (segment.inv_mass <= 0)
    return;
  SliceTransform transform = slice_transforms[segment_slices[segment_index]];
  if (transform.rest_center.w == 0)
    return;
  float3 target0 = transform.center.xyz +
                   rotate_vec3(transform.rotation, segment_particle0s[segment_index].x0 - transform.rest_center.xyz);
  float3 target1 = transform.center.xyz +
                   rotate_vec3(transform.rotation, segment_particle1s[segment_index].x0 - transform.rest_center.xyz);
  segment_particle0s[segment_index].x +=
      shape_matching_strength * (target0 - segment_particle0s[segment_index].x);
  segment_particle1s[segment_index].x +=
      shape_matching_strength * (target1 - segment_particle1s[segment_index].x);
  float4 target_rotation = quat_mul(transform.rotation, segment.q0);
  float4 error = quat_mul(target_rotation, conjugate(segment.q));
  if (error.w < 0)
    error = -error;
  float sine = length(error.xyz);
  float angle = sine < 1e-7f ? 0.f : 2.f * atan2(sine, error.w);
  if (angle > 1e-7f)
    segment.q = normalize(quat_mul(angle_axis(shape_matching_strength * angle, error.xyz / sine), segment.q));
  segment.inertia_w = mat4_cast(segment.q) * float4x4(segment.inertia_tensor.x, 0, 0, 0, 0, segment.inertia_tensor.y,
                                                     0, 0, 0, 0, segment.inertia_tensor.z, 0, 0, 0, 0, 1) *
                      transpose(mat4_cast(segment.q));
  segment.inv_inertia_w = mat4_cast(segment.q) *
                          float4x4(segment.inv_inertia_tensor.x, 0, 0, 0, 0, segment.inv_inertia_tensor.y, 0, 0, 0, 0,
                                   segment.inv_inertia_tensor.z, 0, 0, 0, 0, 1) *
                          transpose(mat4_cast(segment.q));
  segments[segment_index] = segment;
}
