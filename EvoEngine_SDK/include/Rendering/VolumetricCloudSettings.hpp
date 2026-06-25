#pragma once
#include <glm/glm.hpp>

namespace evo_engine {
struct VolumetricCloudSettings {
  bool enabled = true;
  float coverage = 0.62f;
  float density = 1.15f;
  float bottom_altitude = 25.0f;
  float top_altitude = 180.0f;
  float max_march_distance = 900.0f;
  glm::vec2 wind_direction = {1.0f, 0.0f};
  float wind_speed = 8.0f;
  int primary_step_count = 96;
  int light_step_count = 12;
  int resolution_divisor = 2;
  float lighting_intensity = 2.2f;
  float ambient_lighting_strength = 0.18f;
  float phase_anisotropy = 0.72f;
  float base_noise_scale = 0.018f;
  float detail_noise_scale = 0.075f;
  float extinction_scale = 0.018f;
  bool use_spherical_atmosphere = true;
  float atmosphere_radius = 10000.0f;
  float cloud_type = 0.55f;
  float curl_strength = 1.9f;
  float coarse_step_fraction = 0.05f;
  float fine_step_scale = 0.3f;
  int empty_step_fallback_count = 10;
  bool enable_temporal_reprojection = true;
  float temporal_blend_factor = 0.9f;
  bool enable_cloud_shadows = true;
  float cloud_shadow_strength = 0.35f;
  int cloud_shadow_step_count = 6;
  bool debug_visualization = false;
  int debug_mode = 0;

  void ClampSettings() {
    coverage = glm::clamp(coverage, 0.0f, 1.0f);
    density = glm::clamp(density, 0.0f, 10.0f);
    if (bottom_altitude < 0.0f)
      bottom_altitude = 0.0f;
    if (top_altitude < bottom_altitude + 1.0f)
      top_altitude = bottom_altitude + 1.0f;
    max_march_distance = glm::clamp(max_march_distance, 1.0f, 1000000.0f);
    if (glm::dot(wind_direction, wind_direction) > 0.0001f) {
      wind_direction = glm::normalize(wind_direction);
    } else {
      wind_direction = {1.0f, 0.0f};
    }
    wind_speed = glm::clamp(wind_speed, 0.0f, 10000.0f);
    primary_step_count = glm::clamp(primary_step_count, 1, 512);
    light_step_count = glm::clamp(light_step_count, 1, 128);
    if (resolution_divisor <= 1) {
      resolution_divisor = 1;
    } else if (resolution_divisor <= 2) {
      resolution_divisor = 2;
    } else {
      resolution_divisor = 4;
    }
    lighting_intensity = glm::clamp(lighting_intensity, 0.0f, 100.0f);
    ambient_lighting_strength = glm::clamp(ambient_lighting_strength, 0.0f, 10.0f);
    phase_anisotropy = glm::clamp(phase_anisotropy, -0.99f, 0.99f);
    base_noise_scale = glm::clamp(base_noise_scale, 0.00001f, 10.0f);
    detail_noise_scale = glm::clamp(detail_noise_scale, 0.00001f, 10.0f);
    extinction_scale = glm::clamp(extinction_scale, 0.00001f, 1.0f);
    const float min_atmosphere_radius = (glm::max)((top_altitude - bottom_altitude) * 2.0f + 1.0f, 10.0f);
    atmosphere_radius = glm::clamp(atmosphere_radius, min_atmosphere_radius, 10000000.0f);
    cloud_type = glm::clamp(cloud_type, 0.0f, 1.0f);
    curl_strength = glm::clamp(curl_strength, 0.0f, 1000.0f);
    coarse_step_fraction = glm::clamp(coarse_step_fraction, 0.0001f, 1.0f);
    fine_step_scale = glm::clamp(fine_step_scale, 0.01f, 1.0f);
    empty_step_fallback_count = glm::clamp(empty_step_fallback_count, 1, 64);
    temporal_blend_factor = glm::clamp(temporal_blend_factor, 0.0f, 0.98f);
    cloud_shadow_strength = glm::clamp(cloud_shadow_strength, 0.0f, 1.0f);
    cloud_shadow_step_count = glm::clamp(cloud_shadow_step_count, 1, 64);
    debug_mode = glm::clamp(debug_mode, 0, 6);
  }
};
}  // namespace evo_engine
