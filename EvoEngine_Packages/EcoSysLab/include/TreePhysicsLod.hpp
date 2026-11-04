#pragma once
#include "DynamicTreeSkeleton.hpp"

namespace eco_sys_lab_package {
struct TreePhysicsLodLevel {
  struct VertexMap {
    glm::uvec4 links{UINT32_MAX};  // Coarse rod, retained vertex, unused, unused.
    glm::vec4 offset_t{};
  };
  struct RodMap {
    glm::uvec4 links{};
    glm::vec4 relative_rotation{0, 0, 0, 1};
  };
  DynamicTreeSkeleton::GpuRestState rest;
  std::vector<uint32_t> original_vertices;
  std::vector<VertexMap> vertices;
  std::vector<RodMap> rods;
  std::vector<std::vector<uint32_t>> groups;
  float approximation_error = 0, span_error = 0;
};
std::vector<TreePhysicsLodLevel> BuildTreePhysicsLod(const DynamicTreeSkeleton::GpuRestState& original);
}  // namespace eco_sys_lab_package
