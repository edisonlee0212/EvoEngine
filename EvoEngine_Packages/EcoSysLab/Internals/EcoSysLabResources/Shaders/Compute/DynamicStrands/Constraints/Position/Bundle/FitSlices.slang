import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

struct SliceMember {
  uint base_slice;
  int group;
  uint segment;
  uint dynamic_slice;
};

struct SliceRange {
  uint begin;
  uint count;
  uint base_slice;
  int group;
};

struct SliceTransform {
  float4 rest_center;
  float4 center;
  float4 rotation;
};

[[vk::binding(3, 1)]] StructuredBuffer<SliceMember, Std430DataLayout> slice_members;
[[vk::binding(4, 1)]] StructuredBuffer<SliceRange, Std430DataLayout> slice_ranges;
[[vk::binding(6, 1)]] RWStructuredBuffer<SliceTransform, Std430DataLayout> slice_transforms;
[[vk::binding(7, 1)]] StructuredBuffer<uint, Std430DataLayout> slice_count;

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

float3x3 outer_product(float3 a, float3 b) {
  float3x3 result;
  result[0] = a * b.x;
  result[1] = a * b.y;
  result[2] = a * b.z;
  return result;
}

float3x3 inverse_transpose(float3x3 matrix) {
  float determinant_value = dot(matrix[0], cross(matrix[1], matrix[2]));
  return float3x3(cross(matrix[1], matrix[2]), cross(matrix[2], matrix[0]),
                  cross(matrix[0], matrix[1])) /
         determinant_value;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint slice_index : SV_DispatchThreadID) {
  if (slice_index >= slice_count[0])
    return;
  SliceRange range = slice_ranges[slice_index];
  SliceTransform transform = (SliceTransform)0;
  transform.rotation = float4(0, 0, 0, 1);
  if (range.count < minimum_members) {
    slice_transforms[slice_index] = transform;
    return;
  }
  float total_mass = 0;
  for (uint index = range.begin; index < range.begin + range.count; ++index) {
    uint segment_index = slice_members[index].segment;
    Segment segment = segments[segment_index];
    float endpoint_mass = .5f * max(segment.original_mass + segment.extra_mass, 1e-6f);
    transform.rest_center.xyz += endpoint_mass *
                                 (segment_particle0s[segment_index].x0 + segment_particle1s[segment_index].x0);
    transform.center.xyz +=
        endpoint_mass * (segment_particle0s[segment_index].x + segment_particle1s[segment_index].x);
    total_mass += 2.f * endpoint_mass;
  }
  transform.rest_center.xyz /= total_mass;
  transform.center.xyz /= total_mass;
  float3x3 covariance = float3x3(0);
  for (uint index = range.begin; index < range.begin + range.count; ++index) {
    uint segment_index = slice_members[index].segment;
    Segment segment = segments[segment_index];
    float endpoint_mass = .5f * max(segment.original_mass + segment.extra_mass, 1e-6f);
    float3 rest0 = segment_particle0s[segment_index].x0 - transform.rest_center.xyz;
    float3 rest1 = segment_particle1s[segment_index].x0 - transform.rest_center.xyz;
    float3 current0 = segment_particle0s[segment_index].x - transform.center.xyz;
    float3 current1 = segment_particle1s[segment_index].x - transform.center.xyz;
    covariance += endpoint_mass * (outer_product(current0, rest0) + outer_product(current1, rest1));
  }
  float norm_squared = dot(covariance[0], covariance[0]) + dot(covariance[1], covariance[1]) +
                       dot(covariance[2], covariance[2]);
  if (norm_squared < 1e-12f) {
    slice_transforms[slice_index] = transform;
    return;
  }
  float3x3 rotation = covariance * rsqrt(norm_squared);
  for (uint iteration = 0; iteration < 8; ++iteration) {
    if (abs(determinant(rotation)) < 1e-6f) {
      slice_transforms[slice_index] = transform;
      return;
    }
    rotation = .5f * (rotation + inverse_transpose(rotation));
  }
  if (determinant(rotation) < 0)
    rotation[2] = -rotation[2];
  transform.rest_center.w = 1.f;
  transform.rotation = normalize(quat_cast(rotation));
  slice_transforms[slice_index] = transform;
}
