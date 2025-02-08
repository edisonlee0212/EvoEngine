
#pragma once
#include "SimulationSettings.hpp"
#include "Tree.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class ForestPatch
 * @brief Represents a patch of forest consisting of multiple trees.
 */
class ForestPatch : public IAsset {
 public:
  /// Distance between grid points when laying out trees.
  glm::vec2 grid_distance = glm::vec2(1.5f);

  /// Mean offset for tree positions.
  glm::vec2 position_offset_mean = glm::vec2(0.f);

  /// Variance in position offset for tree placement.
  glm::vec2 position_offset_variance = glm::vec2(0.0f);

  /// Variance in rotation offset for tree orientation.
  glm::vec3 rotation_offset_variance = glm::vec3(0.0f);

  /// Reference to the tree descriptor.
  AssetRef tree_descriptor{};

  /// Settings for tree growth within the patch.
  TreeGrowthSettings tree_growth_settings{};

  /// Settings for simulation of tree growth.
  SimulationSettings simulation_settings{};

  /// Minimum pruning of lower branches.
  float min_low_branch_pruning = 0.f;

  /// Maximum pruning of lower branches.
  float max_low_branch_pruning = 0.f;

  /// Total simulation time.
  float simulation_time = 0.f;

  /// Maximum start time offset for simulations.
  float start_time_max = 0.0f;

  /**
   * @brief Instantiates a forest patch with given grid size.
   * @param gridSize Size of the grid for tree placement.
   * @param setSimulationSettings Whether to apply simulation settings during instantiation.
   * @return Root entity of the instantiated patch.
   */
  Entity InstantiatePatch(const glm::ivec2& gridSize, bool setSimulationSettings = true);

  /**
   * @brief Instantiates a forest patch using custom tree growth settings and descriptors.
   * @param candidates List of tree candidates with growth settings and descriptors.
   * @param gridSize Size of the grid for tree placement.
   * @param setSimulationSettings Whether to apply simulation settings during instantiation.
   * @return Root entity of the instantiated patch.
   */
  Entity InstantiatePatch(const std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>>& candidates,
                          const glm::ivec2& gridSize, bool setSimulationSettings = true) const;

  /**
   * @brief Generates a thumbnail texture representing the forest patch.
   * @return Shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Collects asset references from the forest patch.
   * @param list List to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Serializes the forest patch data into YAML format.
   * @param out Output YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the forest patch data from YAML format.
   * @param in Input YAML node.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Inspects the asset within the editor layer.
   * @param editorLayer Shared pointer to the editor layer.
   * @return True if the asset content remains unchanged.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
};

/**
 * @struct TreeInfo
 * @brief Stores data related to an individual tree in a forest.
 */
struct TreeInfo {
  /// Global transformation of the tree.
  GlobalTransform global_transform{};

  /// Reference to the tree descriptor.
  AssetRef tree_descriptor{};

  /**
   * @brief Serializes tree information into YAML format.
   * @param out Output YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes tree information from YAML format.
   * @param in Input YAML node.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Collects asset references from tree information.
   * @param list List to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) const;
};

/**
 * @class ForestDescriptor
 * @brief Describes a forest, composed of multiple trees and their settings.
 */
class ForestDescriptor : public IAsset {
 public:
  /// List of tree information objects.
  std::vector<TreeInfo> tree_infos{};

  /// Growth settings for the entire forest.
  TreeGrowthSettings tree_growth_settings{};

  /**
   * @brief Generates a thumbnail texture representing the forest.
   * @return Shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Applies a single tree descriptor to the forest.
   * @param treeDescriptor Shared pointer to the tree descriptor.
   */
  void ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor);

  /**
   * @brief Applies multiple tree descriptors to the forest.
   * @param treeDescriptors Vector of shared pointers to tree descriptors.
   */
  void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors);

  /**
   * @brief Loads and applies tree descriptors from a specified folder.
   * @param folderPath Path to the folder containing tree descriptors.
   */
  void ApplyTreeDescriptors(const std::filesystem::path& folderPath);

  /**
   * @brief Applies multiple tree descriptors with specific ratios.
   * @param treeDescriptors Vector of shared pointers to tree descriptors.
   * @param ratios Vector containing ratio values for each tree descriptor.
   */
  void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors,
                            const std::vector<float>& ratios);

  /**
   * @brief Loads and applies tree descriptors from a folder with specific ratios.
   * @param folderPath Path to the folder containing tree descriptors.
   * @param ratios Vector containing ratio values for each tree descriptor.
   */
  void ApplyTreeDescriptors(const std::filesystem::path& folderPath, const std::vector<float>& ratios);

  /**
   * @brief Inspects the forest descriptor within the editor layer.
   * @param editorLayer Shared pointer to the editor layer.
   * @return True if the asset content remains unchanged.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  /**
   * @brief Called when the asset is created.
   */
  void OnCreate() override;

  /**
   * @brief Collects asset references from the forest descriptor.
   * @param list List to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Serializes the forest descriptor data into YAML format.
   * @param out Output YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the forest descriptor data from YAML format.
   * @param in Input YAML node.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Sets up a grid for tree placement within the forest.
   * @param grid_size Size of the grid in x and y directions.
   * @param grid_distance Distance between grid points.
   * @param random_shift Random shift value for tree placement variation.
   */
  void SetupGrid(const glm::ivec2& grid_size, float grid_distance, float random_shift);

  /**
   * @brief Instantiates a forest patch.
   * @param set_parent Whether to set a parent entity for the instantiated patch.
   * @param seed The seed of the patch.
   * @return Root entity of the instantiated forest patch.
   */
  Entity InstantiatePatch(bool set_parent, int seed) const;
};

}  // namespace eco_sys_lab_plugin
