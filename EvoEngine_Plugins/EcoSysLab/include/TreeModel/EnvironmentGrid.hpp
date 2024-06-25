#pragma once
#include "Skeleton.hpp"
#include "VoxelGrid.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
class SimulationSettings;
struct InternodeVoxelRegistration {
  glm::vec3 position = glm::vec3(0.0f);
  SkeletonNodeHandle node_handle = -1;
  unsigned tree_skeleton_index = 0;
  float thickness = 0.0f;
};

struct EnvironmentVoxel {
  glm::vec3 light_direction = glm::vec3(0, 1, 0);
  float self_shadow = 0.0f;
  float light_intensity = 1.0f;
  float total_biomass = 0.0f;

  std::vector<InternodeVoxelRegistration> internode_voxel_registrations{};
};

class EnvironmentGrid {
 public:
  float voxel_size = 0.2f;
  VoxelGrid<EnvironmentVoxel> voxel_grid;
  [[nodiscard]] float Sample(const glm::vec3& position, glm::vec3& light_direction) const;
  void AddShadowValue(const glm::vec3& position, float value);
  void LightPropagation(const SimulationSettings& simulation_settings);
  void AddBiomass(const glm::vec3& position, float value);
  void AddNode(const InternodeVoxelRegistration& registration);
};
}  // namespace eco_sys_lab