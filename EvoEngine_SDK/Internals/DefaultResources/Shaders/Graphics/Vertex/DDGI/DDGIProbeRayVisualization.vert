#extension GL_ARB_shading_language_include : enable
#extension GL_ARB_gpu_shader_int64 : enable

#include "PerFrame.glsl"
#include "PointCloudRayTracingPayload.glsl"
#include "DDGI.glsl"

struct DdgiProbeRaySample {
  vec3 direction;
  float padding0;
  vec3 start;

  uint hit_count;
  uint64_t handle;
  uint64_t padding1;

  HitInfo hit_info;
};

layout(set = 1, binding = 0) readonly buffer EE_DDGI_PROBE_RAY_SAMPLE_BLOCK {
  DdgiProbeRaySample EE_DDGI_PROBE_RAY_SAMPLES[];
};

layout(push_constant) uniform EE_DDGI_PROBE_RAY_VISUALIZATION_CONSTANTS {
  uvec4 camera_selected_probe_ray_count_flags;
  vec4 miss_distance_alpha_padding;
};

layout(location = 0) out vec4 VS_COLOR;

vec3 EE_DDGI_DEBUG_RAY_END(const DdgiProbeRaySample ray_sample) {
  const vec3 direction = EE_DDGI_SAFE_NORMALIZE(ray_sample.direction, vec3(0.0f, 1.0f, 0.0f));
  if (EE_DDGI_IS_INACTIVE_PROBE_RAY(ray_sample.hit_count, ray_sample.hit_info.color.a)) {
    return ray_sample.start + direction * miss_distance_alpha_padding.x * 0.15f;
  }
  if (ray_sample.hit_count == 0u) {
    return ray_sample.start + direction * miss_distance_alpha_padding.x;
  }
  return ray_sample.hit_info.position;
}

vec4 EE_DDGI_DEBUG_RAY_COLOR(const DdgiProbeRaySample ray_sample) {
  const float alpha = clamp(miss_distance_alpha_padding.y, 0.0f, 1.0f);
  vec4 color = vec4(0.1f, 0.55f, 1.0f, 0.35f * alpha);
  if (EE_DDGI_IS_INACTIVE_PROBE_RAY(ray_sample.hit_count, ray_sample.hit_info.color.a)) {
    color = vec4(0.45f, 0.45f, 0.48f, 0.2f * alpha);
  } else if (ray_sample.hit_count != 0u && ray_sample.hit_info.color.a < 0.0f) {
    color = vec4(1.0f, 0.35f, 0.08f, 0.8f * alpha);
  } else if (ray_sample.hit_count != 0u) {
    color = vec4(0.15f, 1.0f, 0.35f, alpha);
  }
  if (ray_sample.padding0 > 0.5f) {
    color.rgb = mix(color.rgb, vec3(1.0f, 0.95f, 0.15f), 0.45f);
    color.a = max(color.a, 0.6f * alpha);
  }
  return color;
}

void main() {
  const uint ray_count = max(camera_selected_probe_ray_count_flags.z, 1u);
  const uint ray_index = uint(gl_VertexIndex) / 2u;
  const uint endpoint_index = uint(gl_VertexIndex) & 1u;
  const uint sample_index = camera_selected_probe_ray_count_flags.y * ray_count + ray_index;
  const DdgiProbeRaySample ray_sample = EE_DDGI_PROBE_RAY_SAMPLES[sample_index];
  const vec3 world_position = endpoint_index == 0u ? ray_sample.start : EE_DDGI_DEBUG_RAY_END(ray_sample);

  VS_COLOR = EE_DDGI_DEBUG_RAY_COLOR(ray_sample);
  gl_Position = EE_CAMERAS[camera_selected_probe_ray_count_flags.x].projection_view * vec4(world_position, 1.0f);
}
