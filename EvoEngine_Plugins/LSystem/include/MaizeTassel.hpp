#pragma once

#include "IPrivateComponent.hpp"
#include "TasselGrowthModel.hpp"
#include <cstdint>
#include <filesystem>

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

  /// Process-wide scanner-compatibility toggle. When true, RebuildGeometry()
  /// always emits the legacy CPU `Particles` "Tassel Internodes" entity
  /// instead of the GPU mesh-shader SSBO path, so TasselPointCloudScanner
  /// (which enumerates only standard renderable components via
  /// RenderInstanceStorage) can see the internodes. Default false: keep the
  /// per-frame GPU win for interactive editing. Headless dataset generators
  /// (DatasetGenerator::GenerateDataForTassel) flip this on for the
  /// duration of generation. Do NOT toggle during interactive editing of an
  /// already-rendering tassel.
  static void SetForceCpuParticlesPath(bool force);
  [[nodiscard]] static bool IsForceCpuParticlesPath();

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

#ifdef LSYSTEM_GPU_PIPELINE
  /// Phase 1b: GPU pipeline instance id allocated lazily in RebuildGeometry.
  /// 0 means "never allocated for this component"; the engine reserves 0
  /// as the sentinel and starts handing out ids at 1. Released in
  /// OnDestroy via LSystemGPUEngine::DestroyInstance.
  uint32_t gpu_instance_id = 0;
#endif

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
