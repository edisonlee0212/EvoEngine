#pragma once

#include <IAsset.hpp>
#include <Plot2D.hpp>
#include "MaizeTasselRules.hpp"
#include "ParamSpaceExplorer.hpp"
#include "ILSystemExplorableDescriptor.hpp"
#include <cstdint>
#include <random>

namespace l_system_plugin {

/**
 * @brief Genotype asset for maize tassel L-system generation.
 *
 * Holds SingleDistribution parameter ranges that are sampled per-instance
 * to produce concrete SampledTasselParams for derivation and growth.
 *
 * File extension: .mtassel
 */
class MaizeTasselDescriptor : public evo_engine::IAsset, public ILSystemExplorableDescriptor {
 public:
  MaizeTasselDescriptor();

  // ===== Branch Zone (lower rachis — lateral branches) =====
  evo_engine::SingleDistribution<float> branch_node_count{5.0f};
  evo_engine::PlottedDistribution<float> branch_internode_length;
  evo_engine::PlottedDistribution<float> branch_internode_thickness;
  evo_engine::PlottedDistribution<float> lateral_insertion_angle;
  evo_engine::PlottedDistribution<float> lateral_internode_length;
  evo_engine::PlottedDistribution<float> lateral_node_count;
  evo_engine::PlottedDistribution<float> peduncle_branch_probability;

  // ===== Central Spike (upper rachis) =====
  evo_engine::SingleDistribution<float> spike_node_count{8.0f};
  evo_engine::PlottedDistribution<float> spike_internode_length;
  evo_engine::PlottedDistribution<float> spike_internode_thickness;
  evo_engine::PlottedDistribution<float> spike_zone_branch_probability;

  // ===== Main-rachis spikelet pair morphology =====
  evo_engine::PlottedDistribution<float> main_pair_proximal_scale_x;
  evo_engine::PlottedDistribution<float> main_pair_proximal_scale_y;
  evo_engine::PlottedDistribution<float> main_pair_proximal_scale_z;
  evo_engine::PlottedDistribution<float> main_pair_proximal_angle;
  evo_engine::PlottedDistribution<float> main_pair_internode_length;
  evo_engine::PlottedDistribution<float> main_pair_internode_thickness;
  evo_engine::PlottedDistribution<float> main_pair_internode_angle;
  evo_engine::PlottedDistribution<float> main_pair_distal_scale_x;
  evo_engine::PlottedDistribution<float> main_pair_distal_scale_y;
  evo_engine::PlottedDistribution<float> main_pair_distal_scale_z;
  evo_engine::PlottedDistribution<float> main_pair_distal_angle;

  // ===== Non-main-axis spikelet pair morphology =====
  evo_engine::PlottedDistribution<float> branch_pair_proximal_scale_x;
  evo_engine::PlottedDistribution<float> branch_pair_proximal_scale_y;
  evo_engine::PlottedDistribution<float> branch_pair_proximal_scale_z;
  evo_engine::PlottedDistribution<float> branch_pair_proximal_angle;
  evo_engine::PlottedDistribution<float> branch_pair_internode_length;
  evo_engine::PlottedDistribution<float> branch_pair_internode_thickness;
  evo_engine::PlottedDistribution<float> branch_pair_internode_angle;
  evo_engine::PlottedDistribution<float> branch_pair_distal_scale_x;
  evo_engine::PlottedDistribution<float> branch_pair_distal_scale_y;
  evo_engine::PlottedDistribution<float> branch_pair_distal_scale_z;
  evo_engine::PlottedDistribution<float> branch_pair_distal_angle;

  // ===== Thermal and branch timing =====
  evo_engine::PlottedDistribution<float> lateral_initiation_delay_gdd;
  evo_engine::PlottedDistribution<float> spike_anthesis_offset_gdd;
  evo_engine::PlottedDistribution<float> primary_lateral_branch_probability;
  evo_engine::PlottedDistribution<float> secondary_lateral_branch_probability;

  // ===== Shared =====
  evo_engine::SingleDistribution<float> phyllotaxis_angle{180.0f};
  evo_engine::SingleDistribution<float> branch_azimuth_offset{0.0f};
  evo_engine::SingleDistribution<float> lateral_thickness_ratio{0.6f};

  // -- Secondary branches --
  evo_engine::SingleDistribution<float> secondary_insertion_angle{30.0f};
  evo_engine::SingleDistribution<float> secondary_internode_length{2.0f};
  evo_engine::SingleDistribution<float> secondary_internode_thickness{0.1f};
  evo_engine::SingleDistribution<float> secondary_node_count{1.0f};

  // -- Growth curves --
  evo_engine::Curve2D rachis_elongation_curve;
  evo_engine::Curve2D rachis_thickness_curve;
  evo_engine::Curve2D lateral_elongation_curve;
  evo_engine::Curve2D lateral_thickness_curve;
  evo_engine::Curve2D lateral_angle_development_curve;
  evo_engine::Curve2D pair_proximal_scale_curve;
  evo_engine::Curve2D pair_proximal_angle_curve;
  evo_engine::Curve2D pair_internode_length_curve;
  evo_engine::Curve2D pair_internode_thickness_curve;
  evo_engine::Curve2D pair_internode_angle_curve;
  evo_engine::Curve2D pair_distal_scale_curve;
  evo_engine::Curve2D pair_distal_angle_curve;

  // -- Per-spikelet absolute final age distribution (GDD since spikelet birth) --
  evo_engine::SingleDistribution<float> final_age_gdd{400.0f};

  // -- Dynamic tropism array --
  std::vector<TropismEntry> tropisms;

  // -- GDD milestones --
  evo_engine::SingleDistribution<float> target_gdd{400.0f};
  evo_engine::SingleDistribution<float> base_temperature{10.0f};
  evo_engine::SingleDistribution<float> plastochron_gdd{30.0f};
  evo_engine::SingleDistribution<float> anthesis_gdd{200.0f};
  evo_engine::SingleDistribution<float> maturity_gdd{400.0f};

  // -- Initiation clock decoupling (legacy defaults preserve current behavior). --
  evo_engine::SingleDistribution<float> main_axis_plastochron_scale{1.0f};
  evo_engine::SingleDistribution<float> lateral_axis_plastochron_scale{1.0f};
  evo_engine::SingleDistribution<float> lateral_bud_plastochron_scale{1.0f};
  evo_engine::SingleDistribution<float> maturity_initiation_coupling{0.0f};
  evo_engine::SingleDistribution<float> reference_maturity_gdd{400.0f};

  // -- Development pacing controls. --
  evo_engine::SingleDistribution<float> branch_angle_relaxation{0.08f};
  evo_engine::SingleDistribution<float> pair_angle_relaxation{1.0f};

  // -- Stage-based tassel unfurling controls (normalized by maturity_gdd). --
  evo_engine::SingleDistribution<float> stage_1_end_t{0.167f};
  evo_engine::SingleDistribution<float> stage_2_end_t{0.50f};
  evo_engine::SingleDistribution<float> stage_3_end_t{0.85f};
  evo_engine::SingleDistribution<float> secondary_ramp_start_t{0.167f};
  evo_engine::SingleDistribution<float> secondary_ramp_end_t{0.50f};
  evo_engine::SingleDistribution<float> mature_droop_start_t{0.85f};
  evo_engine::SingleDistribution<float> mature_droop_strength{0.0f};

  SampledTasselParams Sample(std::mt19937& rng) const;
  [[nodiscard]] evo_engine::Entity Instantiate() const;

  bool OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
  [[nodiscard]] bool SupportsDefaultsOverwrite() const override {
    return true;
  }
  [[nodiscard]] std::filesystem::path ResolveWritableDefaultsPath() const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  // -- ILSystemExplorableDescriptor --
  void RegisterExplorableAxes(ParamSpaceExplorer& explorer) override;
  uint64_t ExplorableSchemaFingerprint() const override {
    return static_cast<uint64_t>(tropisms.size());
  }

  // -- Editor preferences (serialized) --
  bool live_preview = false;
  float live_preview_rate_hz = 12.0f;
  bool live_preview_representative_only = true;
  bool live_preview_cap_target_gdd = true;
  float live_preview_max_gdd = 800.0f;
  int live_preview_max_growth_steps = 64;
  int grid_rows = 5;
  int grid_cols = 5;
  float grid_spacing = 3.0f;

  // Runtime-only parameter space explorer (not serialized except mode/speed prefs).
  ParamSpaceExplorer explorer_;

 private:
  // Runtime-only live-preview scheduling state (not serialized).
  bool live_preview_dirty_ = false;
  double live_preview_last_apply_seconds_ = -1.0;
  bool live_preview_was_dragging_ = false;
  bool live_preview_needs_full_apply_ = false;

  // Runtime-only preview diagnostics (not serialized).
  uint32_t live_preview_request_count_ = 0;
  uint32_t live_preview_apply_count_ = 0;
  uint32_t live_preview_coalesced_count_ = 0;
  double live_preview_last_apply_ms_ = 0.0;
  double live_preview_total_apply_ms_ = 0.0;
};

}  // namespace l_system_plugin
