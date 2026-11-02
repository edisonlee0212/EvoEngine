#pragma once

#include <array>
#include <glm/gtc/quaternion.hpp>
#include "GraphicsResources.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

struct GpuTreePhysicsState {
  std::array<std::shared_ptr<Buffer>, 2> vertices, rods;
  std::shared_ptr<Buffer> angles, vertex_rods, rod_angles, node_rods, predicted, previous;
  uint32_t vertex_count = 0, rod_count = 0, node_count = 0;
  uint32_t vertex_index = 0, rod_index = 0;
  glm::mat4 previous_model{1.0f};
  glm::quat initial_rotation{1, 0, 0, 0};
};
}  // namespace eco_sys_lab_package
