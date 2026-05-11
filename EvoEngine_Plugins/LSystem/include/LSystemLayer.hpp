#pragma once

#include "ILayer.hpp"
#include "Input.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#ifdef LSYSTEM_GPU_PIPELINE
#include <glm/glm.hpp>
#endif
namespace evo_engine {
class GraphicsPipeline;
class DescriptorSetLayout;
}  // namespace evo_engine

namespace l_system_plugin {

class LSystemLayer : public evo_engine::ILayer {
 public:
#ifdef LSYSTEM_GPU_PIPELINE
  /// Phase 1b: descriptor set layout that exposes the TasselInternodeInstance
  /// SSBO at (set=1, binding=0). Owned by the plugin so the SDK does not
  /// gain knowledge of L-system-specific resources. Initialized lazily in
  /// OnCreate when the device supports mesh shaders.
  inline static std::shared_ptr<evo_engine::DescriptorSetLayout> tassel_internode_layout;

  /// Phase 1b: graphics pipeline binding tassel_internode.{task,mesh,frag}.
  /// PSO is registered but not yet recorded into any command buffer.
  /// Phase 1b-final wires the indirect draw.
  inline static std::shared_ptr<evo_engine::GraphicsPipeline> tassel_internode_pipeline;

  /// Push-constant payload shared by the task and mesh stages.
  /// Mirrors the combined `TASSEL_INTERNODE_CONSTANTS` block declared in
  /// tassel_internode.task / tassel_internode.mesh. The first three ints
  /// are slot-compatible with the engine's RenderInstancePushConstant so
  /// downstream tooling (e.g. RenderLayer instance dispatch) can reuse
  /// the same push semantics. Field 4 is our per-PSO instance count.
  struct TasselInternodePushConstant {
    int32_t  instance_index = 0;
    int32_t  camera_index = 0;
    int32_t  light_split_index = 0;
    uint32_t tassel_internode_count = 0;
    // World transform of the owning MaizeTassel entity. Lifts entity-local
    // packed positions/normals into world space inside the mesh shader so
    // the GPU draw matches the engine-managed CPU `Particles` path.
    glm::mat4 model{1.0f};
  };
  static_assert(sizeof(TasselInternodePushConstant) == 80,
                "TasselInternodePushConstant must be 80 bytes to match "
                "TASSEL_INTERNODE_CONSTANTS in tassel_internode.{task,mesh}");
#endif
 public:
  struct ProfileFrame {
    double update_ms = 0.0;
    double grow_ms = 0.0;
    double rebuild_ms = 0.0;
    double apply_growth_rules_ms = 0.0;
    double apply_topology_rules_ms = 0.0;
    double sort_lists_ms = 0.0;
    double update_node_info_ms = 0.0;
    double propagate_geometry_ms = 0.0;
    double topology_scan_ms = 0.0;
    double rebuild_internode_collect_ms = 0.0;
    double rebuild_internode_upload_ms = 0.0;
    double rebuild_spikelet_collect_ms = 0.0;
    double rebuild_spikelet_upload_ms = 0.0;
    uint32_t tassel_count = 0;
    uint32_t growth_steps = 0;
    uint32_t node_count = 0;
    uint32_t internode_count = 0;
    uint32_t spikelet_count = 0;
    uint32_t invalid_instance_count = 0;
  };

  /// When true, all MaizeTassel components auto-grow each frame.
  bool auto_grow = false;

  /// When true, Ctrl+W reset assigns fresh seeds to every tassel before regeneration.
  bool reseed_on_reset = false;

  /// Thermal-time accumulation rate during auto-grow (developmental vigor).
  float gdd_per_second = 60.0f;

  /// Optional per-frame GDD cap during auto-grow (0 = unlimited).
  float max_gdd_per_frame = 10.0f;

  /// Optional per-tassel growth step cap per frame (0 = unlimited).
  int max_growth_steps_per_frame = 10;

  /// Enable EcoSysLab-style calendar season gating for auto-grow.
  bool seasonality_enabled = false;

  /// Active-season window [start, end] in day-of-year (0..364), inclusive.
  int season_start_day = 60;
  int season_end_day = 334;

  /// Calendar progression rate for season windows, chronological aging,
  /// and winter-count dormancy updates.
  float chronological_days_per_second = 30.0f;

  /// Simulation day-of-year used by seasonal gating logic.
  float simulation_day_of_year = 60.0f;

  /// Global plant coloring mode (0-3 shared, 4-6 pine-only diagnostics).
  int tassel_color_mode = 0;

  /// Global tint gate for scene/plant debug view modes.
  /// False forces effective color mode to Shaded while preserving selection.
  bool scene_plant_view_tint_enabled = true;

  /// Profiling controls.
  bool profiling_enabled = false;
  int profiling_history_size = 240;
  std::string profiling_export_path = "lsystem_profile.csv";

  /// Last frame + rolling history (not serialized as assets).
  ProfileFrame last_profile_frame{};
  std::vector<ProfileFrame> profiling_history{};

  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  void OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

 private:
    static constexpr float kAutoGrowFailsafeMinFps = 5.0f;
    bool fps_failsafe_tripped_ = false;
    float last_failsafe_fps_ = 0.0f;

  void PushProfileFrame(const ProfileFrame& frame);
  void ExportProfileCsv(const std::string& path) const;
};

}  // namespace l_system_plugin
