#pragma once

#include <AssetRef.hpp>
#include <Entity.hpp>
#include <IAsset.hpp>
#include <Plot2D.hpp>
#include <array>
#include <cstdint>
#include <random>
#include "ILSystemExplorableDescriptor.hpp"
#include "ParamSpaceExplorer.hpp"
#include "SorghumRules.hpp"

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
  evo_engine::SingleDistribution<float> main_culm_lean_angle{0.0f};

  // ===== Internode morphology (rank-indexed) =====
  evo_engine::PlottedDistribution<float> internode_length;
  evo_engine::PlottedDistribution<float> internode_thickness;

  // ===== Leaf morphology (rank-indexed) =====
  evo_engine::PlottedDistribution<float> leaf_blade_length;
  evo_engine::PlottedDistribution<float> leaf_blade_max_width;   ///< Mature full blade width (m).
  evo_engine::PlottedDistribution<float> leaf_blade_thickness;   ///< Mature blade thickness (m).
  evo_engine::PlottedDistribution<float> leaf_sheath_thickness;  ///< Mature sheath wall thickness (m).
  float leaf_width_scale = 1.0f;  ///< Legacy migration multiplier; v4 blade widths are absolute.
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
  evo_engine::PlottedDistribution<float> leaf_waviness_width_fraction;
  evo_engine::SingleDistribution<float> leaf_waviness_wavelength_m{0.0f};
  evo_engine::SingleDistribution<float> leaf_centerline_waviness_fraction{0.0f};
  evo_engine::SingleDistribution<float> leaf_static_wind_deflection_fraction{0.0f};
  evo_engine::SingleDistribution<float> leaf_axial_twist_max_degrees{0.0f};
  evo_engine::SingleDistribution<float> leaf_axial_twist_frequency_ratio_min{0.35f};
  evo_engine::SingleDistribution<float> leaf_axial_twist_frequency_ratio_max{0.5f};
  evo_engine::SingleDistribution<float> leaf_gravity_droop_compliance{0.0f};
  evo_engine::PlottedDistribution<float> leaf_gravity_droop_age_response;
  evo_engine::PlottedDistribution<float> leaf_flexural_stiffness_along_leaf;
  evo_engine::SingleDistribution<float> leaf_damage_severity{0.0f};
  evo_engine::SingleDistribution<float> leaf_sheath_radius_ratio{1.05f};
  evo_engine::SingleDistribution<float> leaf_sheath_cross_section_ratio{1.0f};
  evo_engine::SingleDistribution<float> leaf_sheath_wrap_angle{390.0f};         ///< Total wrap including overlap (deg).
  evo_engine::SingleDistribution<float> leaf_blade_stage1_length_ratio{0.33f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage2_length_ratio{0.34f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage3_length_ratio{0.33f};  ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage1_width_scale{0.85f};   ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage2_width_scale{1.0f};    ///< [deprecated]
  evo_engine::SingleDistribution<float> leaf_blade_stage3_width_scale{0.4f};    ///< [deprecated]

  // ===== Leaf lifecycle (chronological, mirrors ScotsPine) =====
  evo_engine::SingleDistribution<float> leaf_lifespan_years{2.5f};
  evo_engine::SingleDistribution<float> leaf_wilting_years{0.5f};
  evo_engine::SingleDistribution<float> flag_leaf_length_scale{0.78f};
  evo_engine::SingleDistribution<float> flag_leaf_width_scale{0.82f};
  evo_engine::SingleDistribution<float> flag_leaf_insertion_angle_offset{-12.0f};
  evo_engine::SingleDistribution<float> flag_leaf_bending_scale{0.65f};

  // ===== Reproductive panicle =====
  bool enable_panicle = true;
  evo_engine::SingleDistribution<float> panicle_initiation_gdd{1.0f};
  evo_engine::SingleDistribution<float> panicle_maturity_gdd{260.0f};
  /// Unbranched exserted segment between the flag-leaf sheath and the rachis.
  evo_engine::SingleDistribution<float> panicle_peduncle_length_m{0.24f, 0.03f};
  evo_engine::SingleDistribution<float> panicle_rachis_length_m{0.30f, 0.04f};
  evo_engine::SingleDistribution<float> panicle_rachis_radius_m{0.006f, 0.001f};
  evo_engine::SingleDistribution<float> panicle_primary_branch_count{18.0f, 2.0f};
  // Number of sessile-plus-two-pedicellate spikelet triads along each primary branch.
  // The legacy field name remains for descriptor compatibility.
  evo_engine::SingleDistribution<float> panicle_spikelet_pairs_per_branch{6.0f, 1.0f};
  evo_engine::SingleDistribution<float> panicle_branch_length_m{0.13f, 0.02f};
  evo_engine::SingleDistribution<float> panicle_branch_length_taper{0.42f};
  evo_engine::SingleDistribution<float> panicle_branch_radius_m{0.0022f, 0.0003f};
  evo_engine::SingleDistribution<float> panicle_branch_angle_degrees{32.0f, 5.0f};
  evo_engine::SingleDistribution<float> panicle_spikelet_length_m{0.008f, 0.001f};
  evo_engine::SingleDistribution<float> panicle_spikelet_radius_m{0.0036f, 0.0005f};
  evo_engine::SingleDistribution<float> panicle_pedicel_length_m{0.006f, 0.001f};

  // ===== Tillering =====
  uint32_t tiller_model_version = 4u;
  evo_engine::SingleDistribution<float> tiller_count{4.0f, 1.0f};
  int tiller_count_min = 3;
  int tiller_count_max = 5;
  std::array<int, 6> tiller_origin_rank_order{3, 4, 2, 1, 5, 6};
  std::array<int, 6> tiller_emergence_main_leaf_stages{5, 5, 6, 7, 8, 9};
  evo_engine::SingleDistribution<float> tiller_insertion_angle{35.0f, 5.0f};
  evo_engine::SingleDistribution<float> tiller_final_lean_angle{15.0f, 5.0f};
  evo_engine::SingleDistribution<float> tiller_azimuth_jitter{0.0f, 10.0f};
  evo_engine::SingleDistribution<float> tiller_same_side_splay_angle{12.0f};
  float tiller_recovery_axis_fraction = 1.0f;
  evo_engine::SingleDistribution<float> tiller_leaf_count_ratio{0.90f, 0.03f};
  evo_engine::SingleDistribution<float> tiller_height_ratio{0.90f, 0.03f};
  evo_engine::PlottedDistribution<float> tiller_leaf_area_ratio_by_origin;
  evo_engine::SingleDistribution<float> tiller_thickness_ratio{0.80f, 0.05f};
  evo_engine::SingleDistribution<float> tiller_max_axis_length_ratio{1.10f};

  // Legacy v1 controls are load-only migration inputs. They are intentionally
  // ignored by the v4 crown-tiller grammar.
  evo_engine::PlottedDistribution<float> tiller_initiation_delay_gdd;
  evo_engine::SingleDistribution<float> tiller_phytomer_count_scale{0.7f};

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
  bool finalize_snapshot_morphology = true;  ///< Complete snapshot organ dimensions without erasing posture age.

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
  evo_engine::PlottedDistribution<float> bending_along_leaf;
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
  bool leaf_atlas_semantic_quadrants = false;
  glm::vec3 leaf_material_albedo_color{0.26f, 0.52f, 0.18f};
  float leaf_material_roughness = 0.72f;
  float leaf_material_metallic = 0.0f;
  float leaf_material_specular = 0.45f;
  float leaf_material_subsurface_factor = 0.0f;
  glm::vec3 leaf_material_subsurface_color{0.26f, 0.52f, 0.18f};
  glm::vec3 leaf_material_subsurface_radius{0.001f};

  // ===== Stem/sheath-support material =====
  evo_engine::AssetRef stem_albedo_texture;
  evo_engine::AssetRef stem_normal_texture;
  evo_engine::AssetRef stem_roughness_texture;
  evo_engine::AssetRef stem_metallic_texture;
  evo_engine::AssetRef stem_ao_texture;
  glm::vec3 stem_material_albedo_color{0.30f, 0.58f, 0.22f};
  float stem_material_roughness = 0.74f;
  float stem_material_metallic = 0.0f;
  float stem_material_specular = 0.4f;
  glm::vec3 panicle_immature_color{0.32f, 0.56f, 0.16f};
  glm::vec3 panicle_mature_color{0.48f, 0.16f, 0.07f};
  float panicle_material_roughness = 0.78f;
  uint32_t culm_radial_segments = 24u;
  float culm_node_radius_scale = 1.08f;
  float culm_texture_repeat_m = 0.25f;

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
  bool live_preview = true;
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
