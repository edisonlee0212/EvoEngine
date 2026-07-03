#pragma once

#include "ILSystemExplorableDescriptor.hpp"
#include "ParamSpaceExplorer.hpp"
#include "SorghumRules.hpp"
#include <AssetRef.hpp>
#include <IAsset.hpp>
#include <Plot2D.hpp>
#include <cstdint>
#include <random>

namespace l_system_package {

/**
 * @brief Genotype asset for sorghum L-system generation.
 *
 * Holds SingleDistribution / PlottedDistribution parameter ranges that are
 * sampled per-instance to produce a concrete SampledSorghumParams for
 * derivation and growth.
 *
 * File extension: .sorghumls
 *
 * Foundation pass: minimal serialization and inspector. Field set is
 * complete; full editor UI mirrors MaizeTasselDescriptor in a follow-up.
 */
class SorghumLSDescriptor : public evo_engine::IAsset, public ILSystemExplorableDescriptor {
 public:
  SorghumLSDescriptor();

  // ===== Culm topology =====
  evo_engine::SingleDistribution<float> total_phytomer_count{14.0f};
  evo_engine::SingleDistribution<float> phyllotaxis_angle{180.0f};  ///< distichous
  evo_engine::SingleDistribution<float> branch_azimuth_offset{0.0f};

  // ===== Internode morphology (rank-indexed) =====
  evo_engine::PlottedDistribution<float> internode_length;
  evo_engine::PlottedDistribution<float> internode_thickness;

  // ===== Leaf morphology (rank-indexed) =====
  evo_engine::PlottedDistribution<float> leaf_blade_length;
  evo_engine::PlottedDistribution<float> leaf_blade_max_width;  ///< [deprecated] legacy absolute blade width
  evo_engine::PlottedDistribution<float> leaf_sheath_length;
  evo_engine::PlottedDistribution<float> leaf_neck_length;
  evo_engine::PlottedDistribution<float> leaf_sheath_end_width_ratio;
  evo_engine::PlottedDistribution<float> leaf_neck_end_width_ratio;
  evo_engine::PlottedDistribution<float> leaf_blade_end_width_ratio;
  evo_engine::PlottedDistribution<float> leaf_insertion_angle;
  evo_engine::PlottedDistribution<float> leaf_roll_angle;
  evo_engine::PlottedDistribution<float> leaf_curling;
  evo_engine::PlottedDistribution<float> leaf_bending;
  evo_engine::PlottedDistribution<float> leaf_waviness;
  evo_engine::SingleDistribution<float> leaf_waviness_frequency{8.0f};
  evo_engine::SingleDistribution<float> leaf_sheath_radius_ratio{1.05f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage1_length_ratio{0.33f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage2_length_ratio{0.34f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage3_length_ratio{0.33f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage1_width_scale{0.85f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage2_width_scale{1.0f};   ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage3_width_scale{0.4f};   ///< [deprecated]

  // ===== Leaf lifecycle (chronological, mirrors ScotsPine) =====
  evo_engine::SingleDistribution<float> leaf_lifespan_years{2.5f};
  evo_engine::SingleDistribution<float> leaf_wilting_years{0.5f};

  // ===== Tillering =====
  evo_engine::SingleDistribution<float> tiller_count{3.0f};
  evo_engine::PlottedDistribution<float> tiller_initiation_delay_gdd;
  evo_engine::SingleDistribution<float> tiller_insertion_angle{30.0f};
  evo_engine::SingleDistribution<float> tiller_phytomer_count_scale{0.7f};
  evo_engine::SingleDistribution<float> tiller_thickness_ratio{0.6f};

  // ===== Thermal block =====
  evo_engine::SingleDistribution<float> target_gdd{1500.0f};
  evo_engine::SingleDistribution<float> gdd_per_day{10.0f};
  evo_engine::SingleDistribution<float> plastochron_gdd{50.0f};
  evo_engine::SingleDistribution<float> maturity_gdd{600.0f};
  evo_engine::SingleDistribution<float> main_axis_plastochron_scale{1.0f};
  evo_engine::SingleDistribution<float> lateral_axis_plastochron_scale{1.0f};
  evo_engine::SingleDistribution<float> lateral_bud_plastochron_scale{1.0f};
  evo_engine::SingleDistribution<float> maturity_initiation_coupling{0.0f};
  evo_engine::SingleDistribution<float> reference_maturity_gdd{600.0f};

  // ===== Growth curves =====
  evo_engine::PlottedDistribution<float> internode_elongation_curve;
  evo_engine::PlottedDistribution<float> internode_thickness_curve;
  evo_engine::PlottedDistribution<float> leaf_sheath_length_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_neck_length_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_blade_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_sheath_width_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_neck_width_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_width_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_angle_development_curve;
  evo_engine::PlottedDistribution<float> leaf_curling_development_curve;
  evo_engine::PlottedDistribution<float> leaf_bending_development_curve;
  evo_engine::PlottedDistribution<float> width_along_sheath;
  evo_engine::PlottedDistribution<float> width_along_neck;
  evo_engine::PlottedDistribution<float> width_along_leaf;
  evo_engine::PlottedDistribution<float> curling_along_leaf;
  evo_engine::PlottedDistribution<float> waviness_along_leaf;

  // ===== Tropisms =====
  std::vector<TropismEntry> tropisms;

  // ===== Leaf material atlas =====
  evo_engine::AssetRef leaf_atlas_albedo_texture;
  evo_engine::AssetRef leaf_atlas_normal_texture;
  evo_engine::AssetRef leaf_atlas_roughness_texture;
  evo_engine::AssetRef leaf_atlas_metallic_texture;
  evo_engine::AssetRef leaf_atlas_ao_texture;
  uint32_t leaf_atlas_variant_columns = 1u;
  uint32_t leaf_atlas_variant_rows = 1u;
  uint32_t leaf_atlas_variant_count = 1u;
  float leaf_atlas_tile_uv_inset = 0.001f;
  bool leaf_atlas_distal_region_uses_top_half = false;

  // -- Sampling + instantiation --
  SampledSorghumParams Sample(std::mt19937& rng) const;
  [[nodiscard]] evo_engine::Entity Instantiate() const;

  // -- IAsset --
  bool DrawEditorControls(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
  [[nodiscard]] bool SupportsDefaultsOverwrite() const {
    return true;
  }
  [[nodiscard]] std::filesystem::path ResolveWritableDefaultsPath() const;

  // -- ILSystemExplorableDescriptor --
  void RegisterExplorableAxes(ParamSpaceExplorer& explorer) override;
  uint64_t ExplorableSchemaFingerprint() const override {
    return static_cast<uint64_t>(tropisms.size());
  }

  // -- Editor preferences (serialized) --
  bool live_preview = false;
  bool live_preview_representative_only = true;
  bool live_preview_cap_target_gdd = true;
  int grid_rows = 5;
  int grid_cols = 5;
  float grid_spacing = 2.0f;

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

}  // namespace l_system_package
