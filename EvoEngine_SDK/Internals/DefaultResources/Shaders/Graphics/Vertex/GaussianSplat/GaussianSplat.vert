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

layout(push_constant) uniform EE_GAUSSIAN_SPLAT_CONSTANTS {
  uvec4 camera_instance_count_flags;
  vec4 opacity_extent_min_max;
};

layout(location = 0) out VS_OUT {
  vec3 Color;
  vec2 LocalCoord;
  flat float Opacity;
} vs_out;

const vec2 EE_GAUSSIAN_SPLAT_CORNERS[6] = vec2[](
    vec2(-1.0f, -1.0f), vec2(1.0f, -1.0f), vec2(-1.0f, 1.0f),
    vec2(-1.0f, 1.0f), vec2(1.0f, -1.0f), vec2(1.0f, 1.0f));
const float EE_GAUSSIAN_SPLAT_SH_C0 = 0.28209479177387814f;

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

mat2 EE_GAUSSIAN_SPLAT_ADD_OUTER_PRODUCT(mat2 covariance, const vec2 value) {
  covariance[0][0] += value.x * value.x;
  covariance[0][1] += value.x * value.y;
  covariance[1][0] += value.y * value.x;
  covariance[1][1] += value.y * value.y;
  return covariance;
}

vec2 EE_GAUSSIAN_SPLAT_PROJECT_PIXEL(const int camera_index, const vec3 world_position, const vec2 resolution) {
  vec4 clip = EE_CAMERAS[camera_index].projection_view * vec4(world_position, 1.0f);
  clip /= max(abs(clip.w), 0.00001f);
  return clip.xy * resolution * 0.5f;
}

mat2 EE_GAUSSIAN_SPLAT_SCREEN_COVARIANCE(const int camera_index, const mat4 model, const GaussianSplat splat,
                                         const vec3 world_center, const vec2 resolution) {
  const vec3 scale = EE_GAUSSIAN_SPLAT_DECODE_SCALE(splat.scale_reserved.xyz);
  const mat3 model_linear = mat3(model);
  const vec2 center_pixel = EE_GAUSSIAN_SPLAT_PROJECT_PIXEL(camera_index, world_center, resolution);
  mat2 covariance = mat2(0.0f);
  for (uint axis = 0u; axis < 3u; ++axis) {
    vec3 basis = vec3(0.0f);
    basis[axis] = scale[axis];
    const vec3 world_axis = model_linear * EE_GAUSSIAN_SPLAT_ROTATE(splat.rotation, basis);
    const vec2 axis_pixel = EE_GAUSSIAN_SPLAT_PROJECT_PIXEL(camera_index, world_center + world_axis, resolution);
    covariance = EE_GAUSSIAN_SPLAT_ADD_OUTER_PRODUCT(covariance, axis_pixel - center_pixel);
  }
  covariance[0][0] += 0.35f;
  covariance[1][1] += 0.35f;
  return covariance;
}

mat2 EE_GAUSSIAN_SPLAT_COVARIANCE_AXES(const mat2 covariance, const float extent, const float min_radius,
                                       const float max_radius) {
  const float a = covariance[0][0];
  const float b = 0.5f * (covariance[0][1] + covariance[1][0]);
  const float c = covariance[1][1];
  const float midpoint = 0.5f * (a + c);
  const float radius = sqrt(max(0.0f, midpoint * midpoint - a * c + b * b));
  const float lambda0 = max(midpoint + radius, 0.0f);
  const float lambda1 = max(midpoint - radius, 0.0f);
  vec2 axis0 = abs(b) > 0.00001f ? normalize(vec2(b, lambda0 - a)) : vec2(1.0f, 0.0f);
  vec2 axis1 = vec2(-axis0.y, axis0.x);
  const float radius0 = clamp(sqrt(lambda0) * extent, min_radius, max_radius);
  const float radius1 = clamp(sqrt(lambda1) * extent, min_radius, max_radius);
  return mat2(axis0 * radius0, axis1 * radius1);
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
  const float extent = max(opacity_extent_min_max.y, 1.0f);
  const float min_radius = max(opacity_extent_min_max.z, 0.0f);
  const float max_radius = max(opacity_extent_min_max.w, min_radius);
  const mat2 axes = EE_GAUSSIAN_SPLAT_COVARIANCE_AXES(
      EE_GAUSSIAN_SPLAT_SCREEN_COVARIANCE(camera_index, model, splat, world_center, resolution), extent, min_radius,
      max_radius);
  const vec2 corner = EE_GAUSSIAN_SPLAT_CORNERS[gl_VertexIndex];
  const vec2 pixel_offset = axes * corner;
  vec4 center_clip = EE_CAMERAS[camera_index].projection_view * vec4(world_center, 1.0f);
  center_clip.xy += pixel_offset * 2.0f / max(resolution, vec2(1.0f)) * center_clip.w;
  gl_Position = center_clip;

  const float raw_opacity = splat.position_opacity.w;
  const float opacity = raw_opacity >= 0.0f && raw_opacity <= 1.0f ? raw_opacity : EE_GAUSSIAN_SPLAT_SIGMOID(raw_opacity);
  vs_out.Color = clamp(splat.color_rest_offset.rgb * EE_GAUSSIAN_SPLAT_SH_C0 + vec3(0.5f), vec3(0.0f), vec3(1.0f));
  vs_out.LocalCoord = corner * extent;
  vs_out.Opacity = clamp(opacity * opacity_extent_min_max.x, 0.0f, 1.0f);
}
