// @evoengine-dialect native
__exported import EvoEngine.PerFrame;

public static const uint EE_GAUSSIAN_SPLAT_SORTED_INDICES_FLAG = 1u;
public static const uint EE_GAUSSIAN_SPLAT_INDIRECT_DRAW_FLAG = 2u;
public static const uint EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK = 32u;

public struct GaussianSplat {
  public float4 position_opacity;
  public float4 scale_reserved;
  public float4 rotation;
  public float4 color_rest_offset;
};

[[vk::binding(0, 1)]]
StructuredBuffer<GaussianSplat> EE_GAUSSIAN_SPLATS;

[[vk::binding(1, 1)]]
StructuredBuffer<uint> EE_GAUSSIAN_SPLAT_INDICES;

[[vk::binding(2, 1)]]
StructuredBuffer<float> EE_GAUSSIAN_SPLAT_SH_REST;

public struct EeGaussianSplatConstants {
  public uint4 camera_instance_count_flags;
  public uint4 sh_degree_rest_count_reserved;
  public float4 opacity_extent_min_max;
};

[[vk::push_constant]]
public ConstantBuffer<EeGaussianSplatConstants> EE_GAUSSIAN_SPLAT_CONSTANTS;

public static const float2 EE_GAUSSIAN_SPLAT_TRIANGLE_CORNERS[6] = {
    float2(-1.0f, -1.0f), float2(1.0f, -1.0f), float2(-1.0f, 1.0f),
    float2(-1.0f, 1.0f), float2(1.0f, -1.0f), float2(1.0f, 1.0f)};
public static const float2 EE_GAUSSIAN_SPLAT_QUAD_CORNERS[4] = {
    float2(-1.0f, -1.0f), float2(1.0f, -1.0f), float2(-1.0f, 1.0f), float2(1.0f, 1.0f)};
public static const float EE_GAUSSIAN_SPLAT_SH_C0 = 0.28209479177387814f;
static const float EE_GAUSSIAN_SPLAT_SH_C1 = 0.4886025119029199f;
static const float EE_GAUSSIAN_SPLAT_SH_C2[5] = {1.0925484f, -1.0925484f, 0.3153916f, -1.0925484f, 0.5462742f};
static const float EE_GAUSSIAN_SPLAT_SH_C3[7] = {
    -0.5900436f, 2.8906114f, -0.4570458f, 0.3731763f, -0.4570458f, 1.4453057f, -0.5900436f};

float EE_GAUSSIAN_SPLAT_SIGMOID(const float value) {
  return 1.0f / (1.0f + exp(-value));
}

float3 EE_GAUSSIAN_SPLAT_DECODE_SCALE(const float3 raw_scale) {
  const float max_abs_component = max(max(abs(raw_scale.x), abs(raw_scale.y)), abs(raw_scale.z));
  if (max_abs_component <= 0.5f && max(max(raw_scale.x, raw_scale.y), raw_scale.z) > 0.0f) {
    return max(raw_scale, float3(0.0001f));
  }
  return exp(clamp(raw_scale, float3(-12.0f), float3(6.0f)));
}

float3 EE_GAUSSIAN_SPLAT_ROTATE(const float4 raw_rotation, const float3 value) {
  const float4 q = normalize(raw_rotation);
  const float3 u = q.yzw;
  return value + 2.0f * cross(u, cross(u, value) + q.x * value);
}

float3x3 EE_GAUSSIAN_SPLAT_ADD_OUTER_PRODUCT_3D(float3x3 covariance, const float3 value) {
  covariance[0][0] += value.x * value.x;
  covariance[0][1] += value.x * value.y;
  covariance[0][2] += value.x * value.z;
  covariance[1][0] += value.y * value.x;
  covariance[1][1] += value.y * value.y;
  covariance[1][2] += value.y * value.z;
  covariance[2][0] += value.z * value.x;
  covariance[2][1] += value.z * value.y;
  covariance[2][2] += value.z * value.z;
  return covariance;
}

float2x2 EE_GAUSSIAN_SPLAT_SCREEN_COVARIANCE(const int camera_index, const float4x4 model, const GaussianSplat splat,
                                         const float3 world_center, const float2 resolution) {
  const float3 scale = EE_GAUSSIAN_SPLAT_DECODE_SCALE(splat.scale_reserved.xyz);
  const float3x3 model_view_linear = mul(float3x3(model), float3x3(EE_CAMERAS[camera_index].view));
  float3x3 covariance_view = float3x3(0.0f);
  for (uint axis = 0u; axis < 3u; ++axis) {
    float3 basis = float3(0.0f);
    basis[axis] = scale[axis];
    const float3 view_axis = mul(EE_GAUSSIAN_SPLAT_ROTATE(splat.rotation, basis), model_view_linear);
    covariance_view = EE_GAUSSIAN_SPLAT_ADD_OUTER_PRODUCT_3D(covariance_view, view_axis);
  }
  const float3 view_center = mul(float4(world_center, 1.0f), EE_CAMERAS[camera_index].view).xyz;
  const float view_z = min(view_center.z, -0.0001f);
  const float inv_z = 1.0f / view_z;
  const float inv_z2 = inv_z * inv_z;
  const float2 focal = abs(float2(EE_CAMERAS[camera_index].projection[0][0], EE_CAMERAS[camera_index].projection[1][1])) *
                     resolution * 0.5f;
  const float3 jacobian_x = float3(-focal.x * inv_z, 0.0f, focal.x * view_center.x * inv_z2);
  const float3 jacobian_y = float3(0.0f, -focal.y * inv_z, focal.y * view_center.y * inv_z2);
  float2x2 covariance =
      float2x2(dot(jacobian_x, mul(jacobian_x, covariance_view)),
               dot(jacobian_x, mul(jacobian_y, covariance_view)),
               dot(jacobian_y, mul(jacobian_x, covariance_view)),
               dot(jacobian_y, mul(jacobian_y, covariance_view)));
  covariance[0][0] += 0.35f;
  covariance[1][1] += 0.35f;
  return covariance;
}

bool EE_GAUSSIAN_SPLAT_COVARIANCE_AXES(const float2x2 covariance, const float extent, const float min_radius,
                                       const float max_radius, out float2x2 axes) {
  const float a = covariance[0][0];
  const float b = 0.5f * (covariance[0][1] + covariance[1][0]);
  const float c = covariance[1][1];
  const float midpoint = 0.5f * (a + c);
  const float radius = sqrt(max(0.0f, midpoint * midpoint - a * c + b * b));
  const float lambda0 = midpoint + radius;
  const float lambda1 = midpoint - radius;
  if (lambda1 <= 0.0f) {
    axes = float2x2(0.0f);
    return false;
  }
  float2 axis0 =
      abs(b) > 0.00001f ? normalize(float2(b, lambda0 - a)) : (a >= c ? float2(1.0f, 0.0f) : float2(0.0f, 1.0f));
  float2 axis1 = float2(-axis0.y, axis0.x);
  const float radius0 = clamp(sqrt(lambda0) * extent, min_radius, max_radius);
  const float radius1 = clamp(sqrt(lambda1) * extent, min_radius, max_radius);
  axes = float2x2(axis0 * radius0, axis1 * radius1);
  return true;
}

uint EE_GAUSSIAN_SPLAT_AVAILABLE_SH_DEGREE(const uint rest_float_count) {
  const uint coefficients_per_channel = rest_float_count / 3u;
  if (coefficients_per_channel >= 15u) {
    return 3u;
  }
  if (coefficients_per_channel >= 8u) {
    return 2u;
  }
  if (coefficients_per_channel >= 3u) {
    return 1u;
  }
  return 0u;
}

float3 EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(const uint splat_index, const uint rest_float_count,
                                      const uint coefficient_index) {
  const uint coefficients_per_channel = rest_float_count / 3u;
  const uint base = splat_index * rest_float_count;
  return float3(EE_GAUSSIAN_SPLAT_SH_REST[base + coefficient_index],
              EE_GAUSSIAN_SPLAT_SH_REST[base + coefficients_per_channel + coefficient_index],
              EE_GAUSSIAN_SPLAT_SH_REST[base + 2u * coefficients_per_channel + coefficient_index]);
}

float3 EE_GAUSSIAN_SPLAT_SH_RADIANCE(const uint splat_index, const uint requested_degree,
                                   const uint rest_float_count, const float3 direction) {
  const uint degree = min(min(requested_degree, 3u), EE_GAUSSIAN_SPLAT_AVAILABLE_SH_DEGREE(rest_float_count));
  if (degree == 0u) {
    return float3(0.0f);
  }

  const float x = direction.x;
  const float y = direction.y;
  const float z = direction.z;

  float3 radiance = EE_GAUSSIAN_SPLAT_SH_C1 *
                  (-EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 0u) * y +
                   EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 1u) * z -
                   EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 2u) * x);
  if (degree >= 2u) {
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float yz = y * z;
    const float xz = x * z;
    radiance += (EE_GAUSSIAN_SPLAT_SH_C2[0] * xy) *
                    EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 3u) +
                (EE_GAUSSIAN_SPLAT_SH_C2[1] * yz) *
                    EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 4u) +
                (EE_GAUSSIAN_SPLAT_SH_C2[2] * (2.0f * zz - xx - yy)) *
                    EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 5u) +
                (EE_GAUSSIAN_SPLAT_SH_C2[3] * xz) *
                    EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 6u) +
                (EE_GAUSSIAN_SPLAT_SH_C2[4] * (xx - yy)) *
                    EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 7u);

    if (degree >= 3u) {
      radiance += EE_GAUSSIAN_SPLAT_SH_C3[0] *
                      EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 8u) *
                      (3.0f * x * x - y * y) * y +
                  EE_GAUSSIAN_SPLAT_SH_C3[1] *
                      EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 9u) * x * y * z +
                  EE_GAUSSIAN_SPLAT_SH_C3[2] *
                      EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 10u) *
                      (4.0f * z * z - x * x - y * y) * y +
                  EE_GAUSSIAN_SPLAT_SH_C3[3] *
                      EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 11u) * z *
                      (2.0f * z * z - 3.0f * x * x - 3.0f * y * y) +
                  EE_GAUSSIAN_SPLAT_SH_C3[4] *
                      EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 12u) * x *
                      (4.0f * z * z - x * x - y * y) +
                  EE_GAUSSIAN_SPLAT_SH_C3[5] *
                      EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 13u) * (x * x - y * y) * z +
                  EE_GAUSSIAN_SPLAT_SH_C3[6] *
                      EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(splat_index, rest_float_count, 14u) * x *
                      (x * x - 3.0f * y * y);
    }
  }
  return radiance;
}

public void EE_GAUSSIAN_SPLAT_BUILD_VERTEX(const uint splat_order_index, const float2 corner,
                                           out float4 clip_position, out float3 color, out float2 local_coord,
                                           out float opacity, out float cutoff_radius_squared) {
  const bool sorted = (EE_GAUSSIAN_SPLAT_CONSTANTS.camera_instance_count_flags.w & EE_GAUSSIAN_SPLAT_SORTED_INDICES_FLAG) != 0u;
  const uint splat_index = sorted ? EE_GAUSSIAN_SPLAT_INDICES[splat_order_index] : splat_order_index;
  const GaussianSplat splat = EE_GAUSSIAN_SPLATS[splat_index];
  const int camera_index = int(EE_GAUSSIAN_SPLAT_CONSTANTS.camera_instance_count_flags.x);
  const int instance_index = int(EE_GAUSSIAN_SPLAT_CONSTANTS.camera_instance_count_flags.y);
  const float4x4 model = EE_INSTANCES[instance_index].model;
  const float3 world_center = mul(float4(splat.position_opacity.xyz, 1.0f), model).xyz;
  const float2 resolution = float2(EE_CAMERA_RESOLUTION_X(camera_index), EE_CAMERA_RESOLUTION_Y(camera_index));
  const float extent = max(EE_GAUSSIAN_SPLAT_CONSTANTS.opacity_extent_min_max.y, 0.0001f);
  const float min_radius = max(EE_GAUSSIAN_SPLAT_CONSTANTS.opacity_extent_min_max.z, 0.0f);
  const float max_radius = max(EE_GAUSSIAN_SPLAT_CONSTANTS.opacity_extent_min_max.w, min_radius);
  float4 center_clip = mul(float4(world_center, 1.0f), EE_CAMERAS[camera_index].projection_view);
  const float raw_opacity = splat.position_opacity.w;
  const float decoded_opacity =
      raw_opacity >= 0.0f && raw_opacity <= 1.0f ? raw_opacity : EE_GAUSSIAN_SPLAT_SIGMOID(raw_opacity);
  const float scaled_opacity = clamp(decoded_opacity * EE_GAUSSIAN_SPLAT_CONSTANTS.opacity_extent_min_max.x, 0.0f, 1.0f);
  color = splat.color_rest_offset.rgb * EE_GAUSSIAN_SPLAT_SH_C0 + float3(0.5f);
  if (EE_GAUSSIAN_SPLAT_CONSTANTS.sh_degree_rest_count_reserved.x > 0u && EE_GAUSSIAN_SPLAT_CONSTANTS.sh_degree_rest_count_reserved.y > 0u) {
    const float3 view_direction = normalize(world_center - EE_CAMERA_POSITION(camera_index));
    color += EE_GAUSSIAN_SPLAT_SH_RADIANCE(splat_index, EE_GAUSSIAN_SPLAT_CONSTANTS.sh_degree_rest_count_reserved.x,
                                           EE_GAUSSIAN_SPLAT_CONSTANTS.sh_degree_rest_count_reserved.y, view_direction);
  }
  color = clamp(color, float3(0.0f), float3(1.0f));
  local_coord = corner * extent;
  opacity = scaled_opacity;
  cutoff_radius_squared = extent * extent;
  if (center_clip.w <= 0.0f || scaled_opacity <= 1.0f / 255.0f) {
    clip_position = float4(0.0f, 0.0f, 2.0f, 1.0f);
    return;
  }
  float2x2 axes;
  if (!EE_GAUSSIAN_SPLAT_COVARIANCE_AXES(
          EE_GAUSSIAN_SPLAT_SCREEN_COVARIANCE(camera_index, model, splat, world_center, resolution), extent,
          min_radius, max_radius, axes)) {
    clip_position = float4(0.0f, 0.0f, 2.0f, 1.0f);
    return;
  }
  const float2 pixel_offset = mul(corner, axes);
  center_clip.xy += pixel_offset * 2.0f / max(resolution, float2(1.0f)) * center_clip.w;
  clip_position = center_clip;
}
