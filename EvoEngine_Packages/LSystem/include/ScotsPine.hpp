#pragma once

#include <cstdint>
#include <filesystem>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <iosfwd>
#include <limits>
#include <memory>
#include <vector>
#include "LSystemComponentBase.hpp"
#include "PineGrowthModel.hpp"
#include "PlantRenderTarget.hpp"

namespace l_system_package {
using namespace evo_engine;

class ScotsPineDescriptor;

/**
 * @brief Per-instance Scots pine private component.
 *
 * Architectural mirror of MaizeTassel (single source of generated geometry per
 * scene entity), specialized for Pinus sylvestris:
 *   - Annual scheduling driven by `target_gdd` (growing degree-days), matching MaizeTassel.
 *   - Owns a PineGrowthModel that runs single-shot topology derivation
 *     (Phase 3 grammar - no per-frame growth animation yet).
 *   - Render path: internodes via `Particles`, needles via `StrandsRenderer`.
 *
 * Geometry is emitted as two child entities:
   *   - "Pine Internodes" - instanced unit cylinders (one per PineInternode).
   *   - "Pine Fascicle Sheaths" - instanced short cylinders (one per PineNeedleSheath).
   *   - "Pine Needles Strands" - strands payload generated from needle centerlines.
 */
class ScotsPine final : public LSystemComponentBase<ScotsPine> {
 public:
  ScotsPine() = default;
  ScotsPine(const ScotsPine& other)
      : LSystemComponentBase<ScotsPine>(other),
        growth_model(other.growth_model),
        last_grow_seconds(other.last_grow_seconds),
        last_rebuild_seconds(other.last_rebuild_seconds),
        last_node_count(other.last_node_count),
        last_internode_count(other.last_internode_count),
        last_sheath_count(other.last_sheath_count),
        last_needle_count(other.last_needle_count),
        last_invalid_instance_count(other.last_invalid_instance_count),
        last_render_snapshot_version(other.last_render_snapshot_version),
        last_raytrace_internodes_ready(other.last_raytrace_internodes_ready),
        last_raytrace_sheaths_ready(other.last_raytrace_sheaths_ready),
        last_raytrace_needles_ready(other.last_raytrace_needles_ready),
        last_raytrace_internode_instances(other.last_raytrace_internode_instances),
        last_raytrace_sheath_instances(other.last_raytrace_sheath_instances),
        last_raytrace_needle_segments(other.last_raytrace_needle_segments),
        last_raytrace_needle_points(other.last_raytrace_needle_points),
        last_applied_internode_visual_radius_multiplier(other.last_applied_internode_visual_radius_multiplier),
        last_applied_render_needles_enabled(other.last_applied_render_needles_enabled),
        last_applied_leader_debug_color(other.last_applied_leader_debug_color),
        last_applied_color_mode(other.last_applied_color_mode),
        last_needle_skeleton_lines(other.last_needle_skeleton_lines),
        render_target_(nullptr) {
  }
  ScotsPine& operator=(const ScotsPine& other) {
    if (this == &other) {
      return *this;
    }
    LSystemComponentBase<ScotsPine>::operator=(other);
    growth_model = other.growth_model;
    last_grow_seconds = other.last_grow_seconds;
    last_rebuild_seconds = other.last_rebuild_seconds;
    last_node_count = other.last_node_count;
    last_internode_count = other.last_internode_count;
    last_sheath_count = other.last_sheath_count;
    last_needle_count = other.last_needle_count;
    last_invalid_instance_count = other.last_invalid_instance_count;
    last_render_snapshot_version = other.last_render_snapshot_version;
    last_raytrace_internodes_ready = other.last_raytrace_internodes_ready;
    last_raytrace_sheaths_ready = other.last_raytrace_sheaths_ready;
    last_raytrace_needles_ready = other.last_raytrace_needles_ready;
    last_raytrace_internode_instances = other.last_raytrace_internode_instances;
    last_raytrace_sheath_instances = other.last_raytrace_sheath_instances;
    last_raytrace_needle_segments = other.last_raytrace_needle_segments;
    last_raytrace_needle_points = other.last_raytrace_needle_points;
    last_applied_internode_visual_radius_multiplier = other.last_applied_internode_visual_radius_multiplier;
    last_applied_render_needles_enabled = other.last_applied_render_needles_enabled;
    last_applied_leader_debug_color = other.last_applied_leader_debug_color;
    last_applied_color_mode = other.last_applied_color_mode;
    last_needle_skeleton_lines = other.last_needle_skeleton_lines;
    render_target_.reset();
    return *this;
  }
  ScotsPine(ScotsPine&&) noexcept = default;
  ScotsPine& operator=(ScotsPine&&) noexcept = default;

  static constexpr int kChannelInternodes = 0;
  static constexpr int kChannelNeedles = 1;
  static constexpr int kChannelNeedleSheaths = 2;

  struct NeedleSkeletonLine {
    int sheath_node_handle = -1;
    int cluster_node_handle = -1;
    int parent_node_handle = -1;
    int needle_index = -1;
    int initiation_year_index = 0;
    float age_years = 0.0f;
    float length_m = 0.0f;
    float target_length_m = 0.0f;
    float maturation_years = 0.0f;
    bool maturity_reached = false;
    bool year0_cohort = false;
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
    SyntheticOrganLabels = 7,
  };

  struct SeasonalColorTint {
    bool enabled = false;
    glm::vec4 color = glm::vec4(1.0f);
    float strength = 0.0f;
  };

  static void SetGlobalColorMode(ColorMode mode);
  [[nodiscard]] static ColorMode GetGlobalColorMode();
  static void SetGlobalSeasonalColorTint(const SeasonalColorTint& tint);
  [[nodiscard]] static SeasonalColorTint GetGlobalSeasonalColorTint();

  /// Visualization-only multiplier applied to internode rendered cylinder
  /// half-thickness. Does NOT modify `node.info.thickness`, so exported
  /// flow-graph YAML / OBJ ground-truth radii are preserved. Default 1.0.
  /// Intended to make sub-pixel-thin trunks resolvable on rendered images
  /// without violating the phenotype-fidelity constraint of the project.
  static void SetInternodeVisualRadiusMultiplier(float multiplier);
  [[nodiscard]] static float GetInternodeVisualRadiusMultiplier();

  /// When false, skip needle entity construction during RebuildGeometry and
  /// tear down any previously created needle entity. Useful for trunk-only
  /// diagnostic renders. Default true (preserves existing behaviour).
  static void SetRenderNeedlesEnabled(bool enabled);
  [[nodiscard]] static bool IsRenderNeedlesEnabled();

  /// Optional debug colour override for leader-axis (order==0) internodes.
  /// If alpha > 0, the override replaces the normal ColorMode tint on every
  /// leader internode instance, regardless of color mode. Default {0,0,0,0}
  /// (disabled). Visualization-only; does not affect exported assets.
  static void SetLeaderInternodeDebugColor(const glm::vec4& color);
  [[nodiscard]] static glm::vec4 GetLeaderInternodeDebugColor();

  /// Returns infancy GDD for reset.
  [[nodiscard]] float GetInfancyTargetGDD() const;

  /// The growth model (non-serialized, rebuilt on Generate).
  PineGrowthModel growth_model;

  // Runtime profiling/debug counters (not serialized).
  double last_grow_seconds = 0.0;
  double last_rebuild_seconds = 0.0;
  uint32_t last_node_count = 0;
  uint32_t last_internode_count = 0;
  uint32_t last_sheath_count = 0;
  uint32_t last_needle_count = 0;
  uint32_t last_invalid_instance_count = 0;
  std::uint64_t last_render_snapshot_version = 0;
  bool last_raytrace_internodes_ready = false;
  bool last_raytrace_sheaths_ready = false;
  bool last_raytrace_needles_ready = false;
  uint32_t last_raytrace_internode_instances = 0;
  uint32_t last_raytrace_sheath_instances = 0;
  uint32_t last_raytrace_needle_segments = 0;
  uint32_t last_raytrace_needle_points = 0;

  // Cache of visualization-only knobs that require geometry refresh even when
  // growth_model.last_growth_steps == 0 in loaded-scene workflows.
  float last_applied_internode_visual_radius_multiplier = std::numeric_limits<float>::quiet_NaN();
  bool last_applied_render_needles_enabled = true;
  glm::vec4 last_applied_leader_debug_color = glm::vec4(std::numeric_limits<float>::quiet_NaN());
  int last_applied_color_mode = -1;

  void GenerateGeometryEntities(bool uncapped_growth = false);
  void GeneratePreviewGeometryEntities(float preview_target_gdd, uint32_t preview_max_growth_steps);
  bool EnsureGrowthModelInitializedForGrowth();
  void GrowToTargetGDD(bool uncapped_growth = false, bool rebuild_geometry = true);
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
  void WriteAnnotationSkeletonTreeJson(std::ostream& out, int tree_index);
  void CollectAnnotationSkeletonPoints(std::vector<glm::vec3>& points);

  // Last generated needle centerlines in world space (runtime only).
  mutable std::vector<NeedleSkeletonLine> last_needle_skeleton_lines;

  // Runtime render target and channels (not serialized).
  mutable std::unique_ptr<PlantRenderTarget> render_target_ = nullptr;

  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list);
};

}  // namespace l_system_package
