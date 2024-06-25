#pragma once
#include "SimulationSettings.hpp"
#include "Tree.hpp"
using namespace evo_engine;
namespace eco_sys_lab {

class ForestPatch : public IAsset {
 public:
  glm::vec2 grid_distance = glm::vec2(1.5f);
  glm::vec2 position_offset_mean = glm::vec2(0.f);
  glm::vec2 position_offset_variance = glm::vec2(0.0f);
  glm::vec3 rotation_offset_variance = glm::vec3(0.0f);
  AssetRef tree_descriptor;
  TreeGrowthSettings tree_growth_settings{};
  SimulationSettings simulation_settings;

  float min_low_branch_pruning = 0.f;
  float max_low_branch_pruning = 0.f;

  float simulation_time = 0.f;
  float start_time_max = 0.0f;
  Entity InstantiatePatch(const glm::ivec2& gridSize, bool setSimulationSettings = true);

  Entity InstantiatePatch(const std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>>& candidates,
                          const glm::ivec2& gridSize, bool setSimulationSettings = true) const;

  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
};

struct TreeInfo {
  GlobalTransform m_globalTransform{};
  AssetRef m_treeDescriptor{};
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  void CollectAssetRef(std::vector<AssetRef>& list) const;
};

class ForestDescriptor : public IAsset {
 public:
  std::vector<TreeInfo> m_treeInfos;
  TreeGrowthSettings m_treeGrowthSettings;

  void ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor);
  void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors);
  void ApplyTreeDescriptors(const std::filesystem::path& folderPath);
  void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors,
                            const std::vector<float>& ratios);
  void ApplyTreeDescriptors(const std::filesystem::path& folderPath, const std::vector<float>& ratios);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  void OnCreate() override;

  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  void SetupGrid(const glm::ivec2& gridSize, float gridDistance, float randomShift);

  void InstantiatePatch(bool setParent);
};
}  // namespace eco_sys_lab