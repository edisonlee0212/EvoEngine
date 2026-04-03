#pragma once

#include "IPrivateComponent.hpp"
#include "TasselGrowthModel.hpp"
#include <cstdint>

namespace l_system_plugin {
using namespace evo_engine;

class MaizeTasselDescriptor;

class MaizeTassel final : public IPrivateComponent {
 public:
  enum class ColorMode : int {
    Shaded = 0,
    ByType = 1,
    ByInstance = 2,
    ByNode = 3,
  };

  static void SetGlobalColorMode(ColorMode mode);
  [[nodiscard]] static ColorMode GetGlobalColorMode();

  /// Reference to the genotype descriptor asset (required).
  AssetRef descriptor_ref;

  /// Seed for deterministic generation.
  unsigned int seed = 42;

  /// Target GDD to grow to after topology derivation.
  float target_gdd = 0.0f;

  /// Optional per-frame growth step cap forwarded to TasselGrowthModel (0 = unlimited).
  uint32_t max_growth_steps_per_frame = 0;

  /// The growth model (non-serialized, rebuilt on Generate).
  TasselGrowthModel growth_model;

  // Runtime profiling/debug counters (not serialized).
  double last_grow_seconds = 0.0;
  double last_rebuild_seconds = 0.0;
  double last_rebuild_internode_collect_seconds = 0.0;
  double last_rebuild_internode_upload_seconds = 0.0;
  double last_rebuild_spikelet_collect_seconds = 0.0;
  double last_rebuild_spikelet_upload_seconds = 0.0;
  uint32_t last_node_count = 0;
  uint32_t last_internode_count = 0;
  uint32_t last_spikelet_count = 0;
  uint32_t last_invalid_instance_count = 0;

  void GenerateGeometryEntities(bool uncapped_growth = false);
  void GeneratePreviewGeometryEntities(float preview_target_gdd, uint32_t preview_max_growth_steps);
  void GrowToTargetGDD(bool uncapped_growth = false);
  void RebuildGeometry();
  void ClearGeometryEntities() const;

  void OnDestroy() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

}  // namespace l_system_plugin
