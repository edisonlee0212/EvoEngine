
#pragma once
#include "Skeleton.hpp"
#include "VoxelGrid.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class SimulationSettings
 * @brief Forward declaration for simulation settings.
 */
class SimulationSettings;

/**
 * @struct InternodeVoxelRegistration
 * @brief Represents a voxel registration for an internode in a tree structure.
 */
struct InternodeVoxelRegistration {
  glm::vec3 position = glm::vec3(0.0f);  ///< Position of the internode voxel.
  SkeletonNodeHandle node_handle = -1;   ///< Handle to the skeleton node.
  unsigned tree_skeleton_index = 0;      ///< Index referring to the tree skeleton.
  float thickness = 0.0f;                ///< Thickness of the internode.
};

/**
 * @struct EnvironmentVoxel
 * @brief Represents a voxel in the environment containing light and biomass data.
 */
struct EnvironmentVoxel {
  glm::vec3 light_direction = glm::vec3(0, 1, 0);  ///< Direction of the incoming light.
  float self_shadow = 0.0f;                        ///< Self-shadow factor of the voxel.
  float light_intensity = 1.0f;                    ///< Light intensity received by this voxel.
  float total_biomass = 0.0f;                      ///< Total biomass accumulated in this voxel.

  std::vector<InternodeVoxelRegistration>
      internode_voxel_registrations{};  ///< Registrations of internodes in this voxel.
};

/**
 * @class EnvironmentGrid
 * @brief Represents a voxel-based environmental grid for light and biomass simulation.
 */
class EnvironmentGrid {
 public:
  float voxel_size = 0.2f;  ///< Size of each voxel in the grid.

  VoxelGrid<EnvironmentVoxel> voxel_grid;  ///< The voxel grid storing environment data.

  /**
   * @brief Samples the environment at a given position.
   * @param position The position to sample.
   * @param light_direction The direction of light at the sampled position.
   * @return The light intensity at the given position.
   */
  [[nodiscard]] float Sample(const glm::vec3& position, glm::vec3& light_direction) const;

  /**
   * @brief Adds a shadow value to a given position.
   * @param position The position where the shadow value is applied.
   * @param value The shadow value to add.
   */
  void AddShadowValue(const glm::vec3& position, float value);

  /**
   * @brief Propagates light through the voxel grid.
   * @param simulation_settings The settings used for light propagation.
   */
  void LightPropagation(const SimulationSettings& simulation_settings);

  /**
   * @brief Adds biomass to a specified position in the grid.
   * @param position The position where biomass is added.
   * @param value The biomass value to add.
   */
  void AddBiomass(const glm::vec3& position, float value);

  /**
   * @brief Registers an internode voxel in the environmental grid.
   * @param registration The internode voxel registration details.
   */
  void AddNode(const InternodeVoxelRegistration& registration);
};

}  // namespace eco_sys_lab_plugin
