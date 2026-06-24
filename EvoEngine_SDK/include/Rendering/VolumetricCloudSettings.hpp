#pragma once
#include <glm/glm.hpp>

namespace evo_engine {
struct VolumetricCloudSettings {
  bool enabled = false;
  float coverage = 0.65f;
  float density = 1.0f;
  float bottom_altitude = 80.0f;
  float top_altitude = 550.0f;
  float max_march_distance = 5000.0f;
  glm::vec2 wind_direction = {1.0f, 0.0f};
  float wind_speed = 25.0f;
  int primary_step_count = 64;
  int light_step_count = 8;
  int resolution_divisor = 1;
  float lighting_intensity = 1.0f;
  float ambient_lighting_strength = 0.2f;
  float phase_anisotropy = 0.65f;
  float base_noise_scale = 0.012f;
  float detail_noise_scale = 0.05f;
  float extinction_scale = 0.01f;
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
    debug_mode = glm::clamp(debug_mode, 0, 3);
  }
};
}  // namespace evo_engine
