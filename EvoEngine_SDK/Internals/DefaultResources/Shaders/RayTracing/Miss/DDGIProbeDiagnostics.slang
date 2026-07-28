#extension GL_ARB_shading_language_include : enable
#include "RayTracingBasic.slangh"
#include "PointCloudRayTracingPayload.slangh"

layout(push_constant) uniform EE_DDGI_PROBE_RAY_CONSTANTS {
  vec4 first_probe;
  vec4 probe_step_x;
  vec4 probe_step_y;
  vec4 probe_step_z;
  uvec4 probe_counts_and_ray_count;
  uvec4 selected_probe_volume_flags_environment;
};

vec3 EE_DDGI_SAMPLE_ENVIRONMENT_CUBEMAP(const uint environment_index, const vec3 direction) {
  vec3 environment_color =
      textureLod(EE_CUBEMAPS[int(environment_index)], normalize(EE_ENVIRONMENT_LOCAL_DIRECTION(direction)), 0.0f).rgb;
  if (EE_ENVIRONMENT.gamma != 1.0f) {
    environment_color = pow(max(environment_color, vec3(0.0f)), vec3(1.0f / max(EE_ENVIRONMENT.gamma, 0.001f)));
  }
  return max(environment_color, vec3(0.0f));
}

vec3 EE_DDGI_MISS_RADIANCE(const vec3 direction) {
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return max(EE_ENVIRONMENT.background_color.rgb * EE_ENVIRONMENT.diffuse_fallback_intensity, vec3(0.0f));
  }
  if (EE_ENVIRONMENT.diffuse_fallback_intensity <= 0.0f) {
    return vec3(0.0f);
  }
  const uint environment_index = selected_probe_volume_flags_environment.w;
  return EE_DDGI_SAMPLE_ENVIRONMENT_CUBEMAP(environment_index, direction) *
         EE_ENVIRONMENT.diffuse_fallback_intensity;
}

[shader("miss")]
void main(inout PointCloudRayTracingPayload hit_value) {
  hit_value.hit_count = 0u;
  hit_value.handle = 0ul;
  hit_value.hit_info.position = vec3(0.0f);
  hit_value.hit_info.normal = vec3(0.0f);
  hit_value.hit_info.tangent = vec3(0.0f);
  hit_value.hit_info.color = vec4(EE_DDGI_MISS_RADIANCE(WorldRayDirection()), 0.0f);
  hit_value.hit_info.tex_coord = vec2(0.0f);
  hit_value.hit_info.vertex_info1 = 0.0f;
  hit_value.hit_info.vertex_info2 = 0.0f;
  hit_value.hit_info.vertex_info3 = 0.0f;
  hit_value.hit_info.vertex_info4 = vec2(0.0f);
}
