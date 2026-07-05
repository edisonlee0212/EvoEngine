#extension GL_ARB_shading_language_include : enable

#include "RayTracingBasic.glsl"
#include "CameraRayTracingPayload.glsl"
#include "PhysicalSky.glsl"

layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
};

const float EE_CAMERA_PI = 3.14159265359f;
const uint EE_CAMERA_RAY_PAYLOAD_SHADOW = 1u;
const uint EE_CAMERA_RAY_PAYLOAD_MISS = 2u;
const float EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL_SKY = 2.0f;

vec3 EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(const int cubemap_index, const vec3 direction, const float lod) {
  vec3 environment_color = textureLod(EE_CUBEMAPS[cubemap_index], normalize(direction), lod).rgb;
  if (EE_ENVIRONMENT.gamma != 1.0f) {
    environment_color = pow(max(environment_color, vec3(0.0f)), vec3(1.0f / max(EE_ENVIRONMENT.gamma, 0.001f)));
  }
  return max(environment_color, vec3(0.0f));
}

vec2 EE_CAMERA_ENVIRONMENT_SPHERICAL_UV(const vec3 direction) {
  const vec3 dir = normalize(direction);
  return vec2(atan(dir.z, dir.x) / (2.0f * EE_CAMERA_PI) + 0.5f, asin(clamp(dir.y, -1.0f, 1.0f)) / EE_CAMERA_PI + 0.5f);
}

float EE_CAMERA_ENVIRONMENT_MAP_PDF(const vec3 direction) {
  const int pdf_texture_index = int(round(EE_ENVIRONMENT.environment_pdf_texture_index));
  if (pdf_texture_index < 0) {
    return 1.0f / (4.0f * EE_CAMERA_PI);
  }
  const ivec2 texture_size = textureSize(EE_TEXTURE_2DS[pdf_texture_index], 0);
  if (texture_size.x <= 0 || texture_size.y <= 0) {
    return 1.0f / (4.0f * EE_CAMERA_PI);
  }
  const vec2 uv = EE_CAMERA_ENVIRONMENT_SPHERICAL_UV(direction);
  const ivec2 texel = clamp(ivec2(uv * vec2(texture_size)), ivec2(0), texture_size - ivec2(1));
  return max(texelFetch(EE_TEXTURE_2DS[pdf_texture_index], texel, 0).b, 0.0f);
}

vec3 EE_CAMERA_SKY_RADIANCE(const vec3 ray_direction) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  if (camera.use_clear_color == 1) {
    return max(camera.clear_color.xyz, vec3(0.0f)) * max(camera.clear_color.w, 0.0f);
  }
  if (EE_ENVIRONMENT.environment_type == EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL_SKY) {
    return max(EE_PHYSICAL_SKY_EVALUATE(EE_PHYSICAL_SKY_DEFAULT_PARAMETERS(), normalize(ray_direction)), vec3(0.0f));
  }

  return EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index, ray_direction, 0.0f) *
         max(camera.clear_color.w, 0.0f);
}

vec3 EE_CAMERA_ENVIRONMENT_RADIANCE(const vec3 ray_direction) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  if (camera.use_clear_color == 1) {
    return EE_CAMERA_SKY_RADIANCE(ray_direction);
  }
  if (EE_ENVIRONMENT.environment_type == EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL_SKY) {
    return max(EE_PHYSICAL_SKY_EVALUATE(EE_PHYSICAL_SKY_DEFAULT_PARAMETERS(), normalize(ray_direction)), vec3(0.0f));
  }
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return max(EE_ENVIRONMENT.background_color.rgb, vec3(0.0f)) * max(EE_ENVIRONMENT.light_intensity, 0.0f);
  }

  return EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index, ray_direction, 0.0f) *
         max(camera.clear_color.w, 0.0f) * max(EE_ENVIRONMENT.light_intensity, 0.0f);
}

float EE_CAMERA_ENVIRONMENT_PDF() {
  if (EE_ENVIRONMENT.environment_type == EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL_SKY &&
      EE_CAMERAS[EE_CAMERA_INDEX].use_clear_color != 1) {
    return EE_PHYSICAL_SKY_PDF(EE_PHYSICAL_SKY_DEFAULT_PARAMETERS(), normalize(gl_WorldRayDirectionEXT));
  }
  if (EE_ENVIRONMENT.light_intensity <= 0.0f && EE_CAMERAS[EE_CAMERA_INDEX].use_clear_color != 1) {
    return 0.0f;
  }
  if (EE_ENVIRONMENT.environment_type != EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL_SKY &&
      EE_ENVIRONMENT.background_color.w != 1.0f && EE_CAMERAS[EE_CAMERA_INDEX].use_clear_color != 1) {
    return EE_CAMERA_ENVIRONMENT_MAP_PDF(normalize(gl_WorldRayDirectionEXT));
  }
  return 1.0f / (4.0f * EE_CAMERA_PI);
}

void main() {
  if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW) {
    hit_value.hit_count = 0u;
    return;
  }

  const vec3 ray_direction = normalize(gl_WorldRayDirectionEXT);
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
  hit_value.hit_count = 0u;
  hit_value.hit_t = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
  hit_value.position = vec3(0.0f);
  hit_value.normal = -ray_direction;
  hit_value.geometric_normal = -ray_direction;
  hit_value.environment_radiance = EE_CAMERA_ENVIRONMENT_RADIANCE(ray_direction);
  hit_value.environment_pdf = EE_CAMERA_ENVIRONMENT_PDF();
  hit_value.color = EE_CAMERA_SKY_RADIANCE(ray_direction);
}
