#pragma once
#include "Atmosphere.hpp"
namespace evo_engine {
class EditorLayer;

struct SkyIllumination {
  Atmosphere atmosphere{};

  glm::vec3 sun_direction = glm::vec3(0, 1, 0);
  float gamma = 2.2f;

  glm::vec3 ground_color = glm::vec3(0.75f);
  float ground_transmittance = 0.5f;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};
}  // namespace evo_engine