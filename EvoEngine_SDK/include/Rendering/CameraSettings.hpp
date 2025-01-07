#pragma once

namespace evo_engine {
struct CameraSettings {
  float near_distance = 0.1f;
  float far_distance = 200.0f;
  float fade_ratio = 0.8f;
  float fade_factor = 1.f;
  float fov = 120;
  bool use_clear_color = false;
  glm::vec3 clear_color = glm::vec3(0.0f);
  float background_intensity = 1.0f;
};

}  // namespace evo_engine