#pragma once

#include <array>
#include <glm/gtc/quaternion.hpp>
#include "GraphicsResources.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

struct GpuTreePhysicsLodStorage;
struct GpuTreePhysicsState {
  std::shared_ptr<GpuTreePhysicsLodStorage> lod;
  std::array<std::shared_ptr<Buffer>, 2> vertices, rods;
  std::shared_ptr<Buffer> angles, vertex_rods, rod_angles, node_rods, predicted, previous;
  std::shared_ptr<Buffer> vertex_coefficients, rod_coefficients, angle_coefficients;
  bool attribute_arrays = true, coefficients_ready = false;
  glm::vec4 coefficient_mechanics{};
  float reference_density = 700.0f;
  uint32_t vertex_count = 0, rod_count = 0, node_count = 0, angle_count = 0;
  uint32_t vertex_index = 0, rod_index = 0;
  glm::mat4 previous_model{1.0f};
  glm::quat initial_rotation{1, 0, 0, 0};
};
}  // namespace eco_sys_lab_package
