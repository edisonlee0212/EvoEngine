#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"
#include "DDGI.glsl"

layout(set = 1, binding = 17) uniform sampler2D EE_DDGI_IRRADIANCE_ATLAS;

layout(push_constant) uniform EE_DDGI_PROBE_VISUALIZATION_CONSTANTS {
  uvec4 camera_selected_mode;
  vec4 radius_intensity_alpha_selected_scale;
};

layout(location = 0) in VS_OUT {
  vec3 Normal;
  vec4 Irradiance;
  vec4 Visibility;
  vec4 State;
  flat uint PhysicalProbeIndex;
  flat float Selected;
  flat float CameraFade;
} fs_in;

layout(location = 0) out vec4 FragColor;

vec3 EE_DDGI_DECODE_DEBUG_IRRADIANCE(const vec3 encoded_irradiance) {
  const float irradiance_gamma = max(EE_RENDER_INFO.ddgi_volumes[0].probe_counts.w, 1.0f);
  const vec3 sqrt_irradiance = pow(max(encoded_irradiance, vec3(0.0f)), vec3(irradiance_gamma * 0.5f));
  return sqrt_irradiance * sqrt_irradiance * (2.0f * EE_DDGI_PI);
}

vec3 EE_DDGI_TONEMAP_DEBUG_COLOR(const vec3 value) {
  const vec3 positive = max(value, vec3(0.0f));
  const vec3 mapped = positive / (positive + vec3(1.0f));
  return pow(clamp(mapped, vec3(0.0f), vec3(1.0f)), vec3(1.0f / 2.2f));
}

vec3 EE_DDGI_IRRADIANCE_DEBUG_COLOR(const vec3 encoded_irradiance, const float intensity) {
  return EE_DDGI_TONEMAP_DEBUG_COLOR(EE_DDGI_DECODE_DEBUG_IRRADIANCE(encoded_irradiance) * intensity);
}

vec3 EE_DDGI_VISIBILITY_DEBUG_COLOR(const vec4 visibility) {
  const float average_distance = clamp(visibility.x, 0.0f, 1.0f);
  const float backface_ratio = clamp(visibility.y, 0.0f, 1.0f);
  const float relocation = clamp(visibility.z, 0.0f, 1.0f);
  return mix(vec3(0.95f, 0.1f, 0.05f), vec3(0.05f, 0.85f, 1.0f), average_distance) +
         vec3(relocation * 0.15f, backface_ratio * 0.2f, 0.0f);
}

void main() {
  const uint mode = camera_selected_mode.z;
  const float probe_inactive = clamp(fs_in.State.w, 0.0f, 1.0f);
  vec3 color;
  if (mode == 0u) {
    const vec3 normal = normalize(fs_in.Normal);
    const uint irradiance_tile_size = max(EE_RENDER_INFO.ddgi_volumes[0].atlas_parameters.x, 1u);
    const uint irradiance_columns = max(EE_RENDER_INFO.ddgi_volumes[0].atlas_parameters.y, 1u);
    const vec2 irradiance_atlas_size = vec2(textureSize(EE_DDGI_IRRADIANCE_ATLAS, 0));
    const vec2 irradiance_uv =
        EE_DDGI_ATLAS_UV(fs_in.PhysicalProbeIndex, irradiance_columns, irradiance_tile_size, normal,
                         max(irradiance_atlas_size, vec2(1.0f)));
    color = EE_DDGI_IRRADIANCE_DEBUG_COLOR(texture(EE_DDGI_IRRADIANCE_ATLAS, irradiance_uv).rgb,
                                           radius_intensity_alpha_selected_scale.y);
  } else if (mode == 1u) {
    color = EE_DDGI_IRRADIANCE_DEBUG_COLOR(fs_in.Irradiance.rgb, radius_intensity_alpha_selected_scale.y);
  } else if (mode == 2u) {
    color = EE_DDGI_VISIBILITY_DEBUG_COLOR(fs_in.Visibility);
  } else {
    const float hit_ratio = clamp(fs_in.Irradiance.a, 0.0f, 1.0f);
    color = mix(vec3(0.8f, 0.05f, 0.08f), vec3(0.08f, 0.85f, 0.2f), hit_ratio);
  }

  if (probe_inactive > 0.5f) {
    color = mix(vec3(0.08f, 0.08f, 0.1f), color, 0.2f);
  }
  if (fs_in.Selected > 0.5f) {
    color = mix(color, vec3(1.0f, 0.18f, 0.08f), 0.35f);
  }

  const float alpha =
      clamp(radius_intensity_alpha_selected_scale.z, 0.0f, 1.0f) * clamp(fs_in.CameraFade, 0.0f, 1.0f);
  FragColor = vec4(color, alpha);
}
