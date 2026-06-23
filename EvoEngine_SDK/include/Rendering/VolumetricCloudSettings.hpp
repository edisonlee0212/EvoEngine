#pragma once
#include <glm/glm.hpp>

namespace evo_engine {
struct VolumetricCloudSettings {
  bool enabled = false;
  float coverage = 0.45f;
  float density = 0.35f;
  float bottom_altitude = 1500.0f;
  float top_altitude = 4500.0f;
  glm::vec2 wind_direction = {1.0f, 0.0f};
  float wind_speed = 25.0f;
  int primary_step_count = 64;
  int light_step_count = 8;
  float lighting_intensity = 1.0f;
  float ambient_lighting_strength = 0.2f;
  float phase_anisotropy = 0.65f;
  bool debug_visualization = false;
  int debug_mode = 0;

  void ClampSettings() {
    coverage = glm::clamp(coverage, 0.0f, 1.0f);
    density = glm::clamp(density, 0.0f, 10.0f);
    if (bottom_altitude < 0.0f)
      bottom_altitude = 0.0f;
    if (top_altitude < bottom_altitude + 1.0f)
      top_altitude = bottom_altitude + 1.0f;
    if (glm::dot(wind_direction, wind_direction) > 0.0001f) {
      wind_direction = glm::normalize(wind_direction);
    } else {
      wind_direction = {1.0f, 0.0f};
    }
    wind_speed = glm::clamp(wind_speed, 0.0f, 10000.0f);
    primary_step_count = glm::clamp(primary_step_count, 1, 512);
    light_step_count = glm::clamp(light_step_count, 1, 128);
    lighting_intensity = glm::clamp(lighting_intensity, 0.0f, 100.0f);
    ambient_lighting_strength = glm::clamp(ambient_lighting_strength, 0.0f, 10.0f);
    phase_anisotropy = glm::clamp(phase_anisotropy, -0.99f, 0.99f);
    debug_mode = glm::clamp(debug_mode, 0, 3);
  }
};
}  // namespace evo_engine
