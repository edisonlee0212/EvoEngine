#extension GL_ARB_shading_language_include : enable
#include "RayTracingBasic.glsl"
#include "PointCloudRayTracingPayload.glsl"

layout(location = 0) rayPayloadInEXT PointCloudRayTracingPayload hit_value;

vec3 EE_DDGI_MISS_RADIANCE(const vec3 direction) {
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return max(EE_ENVIRONMENT.background_color.rgb * EE_ENVIRONMENT.light_intensity, vec3(0.0f));
  }
  return vec3(0.0f);
}

void main() {
  hit_value.hit_count = 0u;
  hit_value.handle = 0ul;
  hit_value.hit_info.position = vec3(0.0f);
  hit_value.hit_info.normal = vec3(0.0f);
  hit_value.hit_info.tangent = vec3(0.0f);
  hit_value.hit_info.color = vec4(EE_DDGI_MISS_RADIANCE(gl_WorldRayDirectionEXT), 0.0f);
  hit_value.hit_info.tex_coord = vec2(0.0f);
  hit_value.hit_info.vertex_info1 = 0.0f;
  hit_value.hit_info.vertex_info2 = 0.0f;
  hit_value.hit_info.vertex_info3 = 0.0f;
  hit_value.hit_info.vertex_info4 = vec2(0.0f);
}
