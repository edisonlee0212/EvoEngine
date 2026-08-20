#pragma once

#include "LSystemComponentBase.hpp"
#include "PlantRenderTarget.hpp"
#include "SorghumGeometrySnapshot.hpp"
#include "SorghumGrowthModel.hpp"
#include "SorghumLeafMesh.hpp"

#include <cstdint>
#include <filesystem>

namespace l_system_package {
using namespace evo_engine;

class SorghumLSDescriptor;

class SorghumLS final : public LSystemComponentBase<SorghumLS> {
 public:
  SorghumLS() = default;
  SorghumLS(const SorghumLS& other)
      : LSystemComponentBase<SorghumLS>(other),
        leaf_mesh_settings(other.leaf_mesh_settings),
        leaf_bottom_face(other.leaf_bottom_face),
        growth_model(other.growth_model),
        last_grow_seconds(other.last_grow_seconds),
        last_rebuild_seconds(other.last_rebuild_seconds),
        last_rebuild_internode_seconds(other.last_rebuild_internode_seconds),
        last_leaf_spline_seconds(other.last_leaf_spline_seconds),
        last_leaf_mesh_seconds(other.last_leaf_mesh_seconds),
        last_mesh_upload_seconds(other.last_mesh_upload_seconds),
        last_node_count(other.last_node_count),
        last_internode_count(other.last_internode_count),
        last_leaf_count(other.last_leaf_count),
        last_live_leaf_count(other.last_live_leaf_count),
        last_panicle_branch_count(other.last_panicle_branch_count),
        last_panicle_spikelet_count(other.last_panicle_spikelet_count),
        last_invalid_instance_count(other.last_invalid_instance_count),
        geometry_snapshot_(other.geometry_snapshot_),
        geometry_version_(other.geometry_version_) {
  }
  SorghumLS& operator=(const SorghumLS& other) {
    if (this == &other) {
      return *this;
    }
    LSystemComponentBase<SorghumLS>::operator=(other);
    leaf_mesh_settings = other.leaf_mesh_settings;
    leaf_bottom_face = other.leaf_bottom_face;
    growth_model = other.growth_model;
    last_grow_seconds = other.last_grow_seconds;
    last_rebuild_seconds = other.last_rebuild_seconds;
    last_rebuild_internode_seconds = other.last_rebuild_internode_seconds;
    last_leaf_spline_seconds = other.last_leaf_spline_seconds;
    last_leaf_mesh_seconds = other.last_leaf_mesh_seconds;
    last_mesh_upload_seconds = other.last_mesh_upload_seconds;
    last_node_count = other.last_node_count;
    last_internode_count = other.last_internode_count;
    last_leaf_count = other.last_leaf_count;
    last_live_leaf_count = other.last_live_leaf_count;
    last_panicle_branch_count = other.last_panicle_branch_count;
    last_panicle_spikelet_count = other.last_panicle_spikelet_count;
    last_invalid_instance_count = other.last_invalid_instance_count;
    geometry_snapshot_ = other.geometry_snapshot_;
    geometry_version_ = other.geometry_version_;
    render_target_.reset();
    return *this;
  }
  SorghumLS(SorghumLS&&) noexcept = default;
  SorghumLS& operator=(SorghumLS&&) noexcept = default;

  static constexpr int kChannelCulm = 0;
  static constexpr int kChannelLeaves = 1;
  static constexpr int kChannelPanicle = 2;

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
  double last_rebuild_internode_seconds = 0.0;
  double last_leaf_spline_seconds = 0.0;
  double last_leaf_mesh_seconds = 0.0;
  double last_mesh_upload_seconds = 0.0;
  uint32_t last_node_count = 0;
  uint32_t last_internode_count = 0;
  uint32_t last_leaf_count = 0;
  uint32_t last_live_leaf_count = 0;
  uint32_t last_panicle_branch_count = 0;
  uint32_t last_panicle_spikelet_count = 0;
  uint32_t last_invalid_instance_count = 0;

  float GetInfancyTargetGDD() const {
    return 0.0f;
  }

  void GenerateGeometryEntities(bool uncapped_growth = false, bool reuse_geometry_entities = false);
  [[nodiscard]] std::shared_ptr<const SorghumGeometrySnapshot> GenerateGeometrySnapshot(bool uncapped_growth = false,
                                                                                        uint32_t max_growth_steps = 0);
  /// Advance an already-initialized plant to target_gdd without replaying its
  /// prior thermal history.  Reinitializes safely when target_gdd is rewound.
  [[nodiscard]] std::shared_ptr<const SorghumGeometrySnapshot> AdvanceGeometrySnapshot(bool uncapped_growth = false,
                                                                                       uint32_t max_growth_steps = 0);
  [[nodiscard]] std::shared_ptr<const SorghumGeometrySnapshot> GeneratePreviewGeometrySnapshot(
      float preview_target_gdd, uint32_t preview_max_growth_steps);
  void GeneratePreviewGeometryEntities(float preview_target_gdd, uint32_t preview_max_growth_steps);
  void GrowToTargetGDD(bool uncapped_growth = false, uint32_t max_growth_steps = 0);
  void SetSeasonalChronologicalMode(bool enable_independent_chronological_clock);
  bool AdvanceChronologicalAging(float delta_years);
  [[nodiscard]] std::shared_ptr<const SorghumGeometrySnapshot> BuildGeometrySnapshot();
  void PublishGeometrySnapshot(const std::shared_ptr<const SorghumGeometrySnapshot>& snapshot,
                               bool update_render_geometry = true);
  void RebuildGeometry();
  void ClearGeometryEntities() const;
  [[nodiscard]] const std::shared_ptr<const SorghumGeometrySnapshot>& GetGeometrySnapshot() const;
  void ExportObj(const std::filesystem::path& path) const;
  void ExportFlowGraph(YAML::Emitter& out);
  void ExportFlowGraph(const std::filesystem::path& path);
  void ExportNodeGraph(YAML::Emitter& out);
  void ExportNodeGraph(const std::filesystem::path& path);

  void Start() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list);

 private:
  mutable std::shared_ptr<const SorghumGeometrySnapshot> geometry_snapshot_;
  mutable std::unique_ptr<PlantRenderTarget> render_target_ = nullptr;
  uint64_t geometry_version_ = 0;
};

}  // namespace l_system_package
