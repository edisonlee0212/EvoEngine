#pragma once

#include "LSystemComponentBase.hpp"
#include "SorghumGrowthModel.hpp"
#include "SorghumLeafMesh.hpp"

#include <cstdint>
#include <filesystem>

namespace l_system_plugin {
using namespace evo_engine;

class SorghumLSDescriptor;

class SorghumLS final : public LSystemComponentBase<SorghumLS> {
 public:
  enum class ColorMode : int {
    Shaded = 0,
    ByType = 1,
    ByInstance = 2,
    ByNode = 3,
    LeafSenescence = 4,
  };

  static void SetGlobalColorMode(ColorMode mode);
  [[nodiscard]] static ColorMode GetGlobalColorMode();

  // Leaf mesh controls.
  SorghumLeafMeshSettings leaf_mesh_settings;
  bool leaf_bottom_face = true;

  // Growth model (runtime only, not serialized).
  SorghumGrowthModel growth_model;

  // Runtime counters/profiling.
  double last_grow_seconds = 0.0;
  double last_rebuild_seconds = 0.0;
  uint32_t last_node_count = 0;
  uint32_t last_internode_count = 0;
  uint32_t last_leaf_count = 0;
  uint32_t last_live_leaf_count = 0;
  uint32_t last_invalid_instance_count = 0;

  float GetInfancyTargetGDD() const { return 0.0f; }

  void GenerateGeometryEntities(bool uncapped_growth = false);
  void GeneratePreviewGeometryEntities(float preview_target_gdd, uint32_t preview_max_growth_steps);
  void GrowToTargetGDD(bool uncapped_growth = false);
  void SetSeasonalChronologicalMode(bool enable_independent_chronological_clock);
  bool AdvanceChronologicalAging(float delta_years);
  void RebuildGeometry();
  void ClearGeometryEntities() const;
  void ExportObj(const std::filesystem::path& path) const;
  void ExportFlowGraph(YAML::Emitter& out);
  void ExportFlowGraph(const std::filesystem::path& path);
  void ExportNodeGraph(YAML::Emitter& out);
  void ExportNodeGraph(const std::filesystem::path& path);

  void OnDestroy() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

}  // namespace l_system_plugin
