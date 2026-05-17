#pragma once

#include "LSystemComponentBase.hpp"
#include "PineGrowthModel.hpp"
#include <cstdint>
#include <filesystem>
#include <glm/vec3.hpp>
#include <vector>

namespace l_system_plugin {
using namespace evo_engine;

class ScotsPineDescriptor;

/**
 * @brief Per-instance Scots pine private component.
 *
 * Architectural mirror of MaizeTassel (single source of generated geometry per
 * scene entity), specialized for Pinus sylvestris:
 *   - Annual scheduling driven by `target_gdd` (growing degree-days), matching MaizeTassel.
 *   - Owns a PineGrowthModel that runs single-shot topology derivation
 *     (Phase 3 grammar — no per-frame growth animation yet).
 *   - Hybrid render path: internodes via `Particles`, aggregate needles via `MeshRenderer`.
 *
 * Geometry is emitted as two child entities:
 *   - "Pine Internodes" — instanced unit cylinders (one per PineInternode).
 *   - "Pine Needles" / "Pine Needles Geometry" — legacy markers or aggregate swept needle mesh.
 */
class ScotsPine final : public LSystemComponentBase<ScotsPine> {
 public:
  struct NeedleSkeletonLine {
    int cluster_node_handle = -1;
    int parent_node_handle = -1;
    int needle_index = -1;
    std::vector<glm::vec3> points_world;
  };

  enum class ColorMode : int {
    Shaded = 0,
    ByType = 1,
    ByInstance = 2,
    ByNode = 3,
    NeedleLignification = 4,
    NeedleStripeProxy = 5,
    NeedleSheath = 6,
  };

  static void SetGlobalColorMode(ColorMode mode);
  [[nodiscard]] static ColorMode GetGlobalColorMode();

  /// Scanner-compatibility toggle. CPU-only path is the default for Pine,
  /// so this currently has no effect; kept for API parity with MaizeTassel
  /// to ease future GPU-pipeline integration.
  static void SetForceCpuParticlesPath(bool force);
  [[nodiscard]] static bool IsForceCpuParticlesPath();

  /// Optional post-repot descriptor used after the switch trigger is reached.
  AssetRef post_repot_descriptor_ref;

  /// Enables one-time pre->post descriptor switching in the growth model.
  bool enable_repot_profile_switch = false;

  /// Trigger GDD where post-repot profile becomes active.
  float repot_switch_gdd = 6000.0f;

  /// Returns infancy GDD for reset.
  [[nodiscard]] float GetInfancyTargetGDD() const;

  /// The growth model (non-serialized, rebuilt on Generate).
  PineGrowthModel growth_model;

  // Runtime profiling/debug counters (not serialized).
  double last_grow_seconds = 0.0;
  double last_rebuild_seconds = 0.0;
  uint32_t last_node_count = 0;
  uint32_t last_internode_count = 0;
  uint32_t last_needle_count = 0;
  uint32_t last_invalid_instance_count = 0;

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
  void ExportNeedleSkeleton(YAML::Emitter& out);
  void ExportNeedleSkeleton(const std::filesystem::path& path);

  // Last generated needle centerlines in world space (runtime only).
  std::vector<NeedleSkeletonLine> last_needle_skeleton_lines;

  void OnDestroy() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

}  // namespace l_system_plugin
