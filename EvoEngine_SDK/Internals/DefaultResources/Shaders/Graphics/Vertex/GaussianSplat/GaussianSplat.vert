#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"

struct GaussianSplat {
  vec4 position_opacity;
  vec4 scale_reserved;
  vec4 rotation;
  vec4 color_rest_offset;
};

layout(set = 1, binding = 0) readonly buffer EE_GAUSSIAN_SPLAT_BLOCK {
  GaussianSplat EE_GAUSSIAN_SPLATS[];
};

layout(set = 1, binding = 1) readonly buffer EE_GAUSSIAN_SPLAT_INDEX_BLOCK {
  uint EE_GAUSSIAN_SPLAT_INDICES[];
};

layout(set = 1, binding = 2) readonly buffer EE_GAUSSIAN_SPLAT_SH_REST_BLOCK {
  float EE_GAUSSIAN_SPLAT_SH_REST[];
};

layout(push_constant) uniform EE_GAUSSIAN_SPLAT_CONSTANTS {
  uvec4 camera_instance_count_flags;
  uvec4 sh_degree_rest_count_reserved;
  vec4 opacity_extent_min_max;
};

layout(location = 0) out VS_OUT {
  vec3 Color;
  vec2 LocalCoord;
  flat float Opacity;
  flat float CutoffRadiusSquared;
} vs_out;

const vec2 EE_GAUSSIAN_SPLAT_CORNERS[6] = vec2[](
    vec2(-1.0f, -1.0f), vec2(1.0f, -1.0f), vec2(-1.0f, 1.0f),
    vec2(-1.0f, 1.0f), vec2(1.0f, -1.0f), vec2(1.0f, 1.0f));
const float EE_GAUSSIAN_SPLAT_SH_C0 = 0.28209479177387814f;
const float EE_GAUSSIAN_SPLAT_SH_C1 = 0.4886025119029199f;
const float EE_GAUSSIAN_SPLAT_SH_C2[5] = float[](1.0925484f, -1.0925484f, 0.3153916f, -1.0925484f, 0.5462742f);
const float EE_GAUSSIAN_SPLAT_SH_C3[7] =
    float[](-0.5900436f, 2.8906114f, -0.4570458f, 0.3731763f, -0.4570458f, 1.4453057f, -0.5900436f);

float EE_GAUSSIAN_SPLAT_SIGMOID(const float value) {
  return 1.0f / (1.0f + exp(-value));
}

vec3 EE_GAUSSIAN_SPLAT_DECODE_SCALE(const vec3 raw_scale) {
  const float max_abs_component = max(max(abs(raw_scale.x), abs(raw_scale.y)), abs(raw_scale.z));
  if (max_abs_component <= 0.5f && max(max(raw_scale.x, raw_scale.y), raw_scale.z) > 0.0f) {
    return max(raw_scale, vec3(0.0001f));
  }
  return exp(clamp(raw_scale, vec3(-12.0f), vec3(6.0f)));
}

vec3 EE_GAUSSIAN_SPLAT_ROTATE(const vec4 raw_rotation, const vec3 value) {
  const vec4 q = normalize(raw_rotation);
  const vec3 u = q.yzw;
  return value + 2.0f * cross(u, cross(u, value) + q.x * value);
}

mat3 EE_GAUSSIAN_SPLAT_ADD_OUTER_PRODUCT_3D(mat3 covariance, const vec3 value) {
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

mat2 EE_GAUSSIAN_SPLAT_SCREEN_COVARIANCE(const int camera_index, const mat4 model, const GaussianSplat splat,
                                         const vec3 world_center, const vec2 resolution) {
  const vec3 scale = EE_GAUSSIAN_SPLAT_DECODE_SCALE(splat.scale_reserved.xyz);
  const mat3 model_view_linear = mat3(EE_CAMERAS[camera_index].view) * mat3(model);
  mat3 covariance_view = mat3(0.0f);
  for (uint axis = 0u; axis < 3u; ++axis) {
    vec3 basis = vec3(0.0f);
    basis[axis] = scale[axis];
    const vec3 view_axis = model_view_linear * EE_GAUSSIAN_SPLAT_ROTATE(splat.rotation, basis);
    covariance_view = EE_GAUSSIAN_SPLAT_ADD_OUTER_PRODUCT_3D(covariance_view, view_axis);
  }
  const vec3 view_center = vec3(EE_CAMERAS[camera_index].view * vec4(world_center, 1.0f));
  const float view_z = min(view_center.z, -0.0001f);
  const float inv_z = 1.0f / view_z;
  const float inv_z2 = inv_z * inv_z;
  const vec2 focal = abs(vec2(EE_CAMERAS[camera_index].projection[0][0], EE_CAMERAS[camera_index].projection[1][1])) *
                     resolution * 0.5f;
  const vec3 jacobian_x = vec3(-focal.x * inv_z, 0.0f, focal.x * view_center.x * inv_z2);
  const vec3 jacobian_y = vec3(0.0f, -focal.y * inv_z, focal.y * view_center.y * inv_z2);
  mat2 covariance = mat2(dot(jacobian_x, covariance_view * jacobian_x),
                         dot(jacobian_x, covariance_view * jacobian_y),
                         dot(jacobian_y, covariance_view * jacobian_x),
                         dot(jacobian_y, covariance_view * jacobian_y));
  covariance[0][0] += 0.35f;
  covariance[1][1] += 0.35f;
  return covariance;
}

bool EE_GAUSSIAN_SPLAT_COVARIANCE_AXES(const mat2 covariance, const float extent, const float min_radius,
                                       const float max_radius, out mat2 axes) {
  const float a = covariance[0][0];
  const float b = 0.5f * (covariance[0][1] + covariance[1][0]);
  const float c = covariance[1][1];
  const float midpoint = 0.5f * (a + c);
  const float radius = sqrt(max(0.0f, midpoint * midpoint - a * c + b * b));
  const float lambda0 = midpoint + radius;
  const float lambda1 = midpoint - radius;
  if (lambda1 <= 0.0f) {
    axes = mat2(0.0f);
    return false;
  }
  vec2 axis0 =
      abs(b) > 0.00001f ? normalize(vec2(b, lambda0 - a)) : (a >= c ? vec2(1.0f, 0.0f) : vec2(0.0f, 1.0f));
  vec2 axis1 = vec2(-axis0.y, axis0.x);
  const float radius0 = clamp(sqrt(lambda0) * extent, min_radius, max_radius);
  const float radius1 = clamp(sqrt(lambda1) * extent, min_radius, max_radius);
  axes = mat2(axis0 * radius0, axis1 * radius1);
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

vec3 EE_GAUSSIAN_SPLAT_SH_COEFFICIENT(const uint splat_index, const uint rest_float_count,
                                      const uint coefficient_index) {
  const uint coefficients_per_channel = rest_float_count / 3u;
  const uint base = splat_index * rest_float_count;
  return vec3(EE_GAUSSIAN_SPLAT_SH_REST[base + coefficient_index],
              EE_GAUSSIAN_SPLAT_SH_REST[base + coefficients_per_channel + coefficient_index],
              EE_GAUSSIAN_SPLAT_SH_REST[base + 2u * coefficients_per_channel + coefficient_index]);
}

vec3 EE_GAUSSIAN_SPLAT_SH_RADIANCE(const uint splat_index, const uint requested_degree,
                                   const uint rest_float_count, const vec3 direction) {
  const uint degree = min(min(requested_degree, 3u), EE_GAUSSIAN_SPLAT_AVAILABLE_SH_DEGREE(rest_float_count));
  if (degree == 0u) {
    return vec3(0.0f);
  }

  const float x = direction.x;
  const float y = direction.y;
  const float z = direction.z;

  vec3 radiance = EE_GAUSSIAN_SPLAT_SH_C1 *
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

void main() {
  const bool sorted = (camera_instance_count_flags.w & 1u) != 0u;
  const uint splat_index = sorted ? EE_GAUSSIAN_SPLAT_INDICES[gl_InstanceIndex] : gl_InstanceIndex;
  const GaussianSplat splat = EE_GAUSSIAN_SPLATS[splat_index];
  const int camera_index = int(camera_instance_count_flags.x);
  const int instance_index = int(camera_instance_count_flags.y);
  const mat4 model = EE_INSTANCES[instance_index].model;
  const vec3 world_center = vec3(model * vec4(splat.position_opacity.xyz, 1.0f));
  const vec2 resolution = vec2(EE_CAMERA_RESOLUTION_X(camera_index), EE_CAMERA_RESOLUTION_Y(camera_index));
  const float extent = max(opacity_extent_min_max.y, 0.0001f);
  const float min_radius = max(opacity_extent_min_max.z, 0.0f);
  const float max_radius = max(opacity_extent_min_max.w, min_radius);
  const vec2 corner = EE_GAUSSIAN_SPLAT_CORNERS[gl_VertexIndex];
  vec4 center_clip = EE_CAMERAS[camera_index].projection_view * vec4(world_center, 1.0f);
  const float raw_opacity = splat.position_opacity.w;
  const float opacity = raw_opacity >= 0.0f && raw_opacity <= 1.0f ? raw_opacity : EE_GAUSSIAN_SPLAT_SIGMOID(raw_opacity);
  const float scaled_opacity = clamp(opacity * opacity_extent_min_max.x, 0.0f, 1.0f);
  vec3 color = splat.color_rest_offset.rgb * EE_GAUSSIAN_SPLAT_SH_C0 + vec3(0.5f);
  if (sh_degree_rest_count_reserved.x > 0u && sh_degree_rest_count_reserved.y > 0u) {
    const vec3 view_direction = normalize(world_center - EE_CAMERA_POSITION(camera_index));
    color += EE_GAUSSIAN_SPLAT_SH_RADIANCE(splat_index, sh_degree_rest_count_reserved.x,
                                           sh_degree_rest_count_reserved.y, view_direction);
  }
  vs_out.Color = clamp(color, vec3(0.0f), vec3(1.0f));
  vs_out.LocalCoord = corner * extent;
  vs_out.Opacity = scaled_opacity;
  vs_out.CutoffRadiusSquared = extent * extent;
  if (center_clip.w <= 0.0f || scaled_opacity <= 1.0f / 255.0f) {
    gl_Position = vec4(0.0f, 0.0f, 2.0f, 1.0f);
    return;
  }
  mat2 axes;
  if (!EE_GAUSSIAN_SPLAT_COVARIANCE_AXES(
          EE_GAUSSIAN_SPLAT_SCREEN_COVARIANCE(camera_index, model, splat, world_center, resolution), extent,
          min_radius, max_radius, axes)) {
    gl_Position = vec4(0.0f, 0.0f, 2.0f, 1.0f);
    return;
  }
  const vec2 pixel_offset = axes * corner;
  center_clip.xy += pixel_offset * 2.0f / max(resolution, vec2(1.0f)) * center_clip.w;
  gl_Position = center_clip;
}
