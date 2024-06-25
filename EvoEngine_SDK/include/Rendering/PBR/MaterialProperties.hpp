#pragma once
#include <glm/glm.hpp>
namespace evo_engine {
struct MaterialProperties {
  glm::vec3 albedo_color = glm::vec3(1.0f);
  glm::vec3 subsurface_color = glm::vec3(1.0f);
  float subsurface_factor = 0.0f;
  glm::vec3 subsurface_radius = glm::vec3(1.0f, 0.2f, 0.1f);
  float metallic = 0.1f;
  float specular = 0.5f;
  float specular_tint = 0.0f;
  float roughness = 0.3f;
  float sheen = 0.0f;
  float sheen_tint = 0.5f;
  float clear_coat = 0.0f;
  float clear_coat_roughness = 0.03f;
  float ior = 1.45f;
  float transmission = 0.0f;
  float transmission_roughness = 0.0f;
  float emission = 0.0f;
};
}  // namespace evo_engine