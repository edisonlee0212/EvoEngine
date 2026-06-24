#pragma once

#include <filesystem>
#include <string>
#include <cstdint>
#include <array>
#include <cctype>
#include <iomanip>
#include <sstream>

namespace evo_engine {

struct ScotsPineSyntheticRenderOptions {
  std::filesystem::path project_path{};
  std::filesystem::path scene_path{"ScotsPineDataGenerator.evescene"};
  std::filesystem::path descriptor_path{"New ScotsPineDescriptor.spine"};
  std::filesystem::path output_root{"output/scots_pine_synthetic"};
  std::string output_name{"scene"};
  std::filesystem::path camera_rig_file{};
  std::filesystem::path background_image{};
  std::filesystem::path background_dir{};
  std::string scene_pine_growth_mode{"calendar_scene_multiplier"};
  std::string render_mode{"rasterization"};
  std::string rgb_output_format{"jpg"};

  std::uint32_t base_seed = 12345;
  std::uint32_t seed_a = 12345;
  std::uint32_t seed_b = 12346;
  std::uint32_t seed_c = 12347;

  int sample_count = 1;
  int start_index = 0;
  int worker_id = 0;
  int worker_count = 1;
  int frame_count = 1;
  int width = 1943;
  int height = 1221;
  int profile_warmup_datapoints = 1;
  int writer_threads = 1;
  int jpg_quality = 95;
  int png_compression_level = 1;
  float plant_blur_radius_px = 0.0f;
  float max_target_gdd = 12000.0f;
  float triangle_side_length = 3.0f;
  float triangle_offset_x = 0.0f;
  float triangle_offset_z = 0.0f;
  float triangle_yaw_deg = 0.0f;
  float camera_position_offset_y = -0.48f;
  float camera_screen_offset_x_percent = 0.0f;
  float camera_screen_offset_y_percent = 0.0f;
  float visual_scale = 20.0f;
  float ambient_light = 0.8f;
  float directional_light = 1.25f;
  float scene_pine_target_gdd_multiplier = 1.0f;
  float calendar_step_days = 1.0f;
  float annual_whorl_probability = 0.22f;
  float whorl_position_norm = 0.92f;
  float branches_per_whorl = 2.0f;
  float whorl_dormancy_years = 1.0f;
  int max_branching_order = 1;

  std::array<float, 4> young_needle_palette_rgba{0.0f, 0.7455683f, 0.051418442f, 1.0f};
  std::array<float, 4> older_needle_palette_rgba{0.21606492f, 0.28904234f, 0.20766766f, 1.0f};
  std::array<float, 4> dry_brown_needle_palette_rgba{0.77059436f, 0.5399785f, 0.0f, 1.0f};
  std::array<float, 4> main_stem_palette_rgba{0.7861458f, 0.9165798f, 0.47310424f, 1.0f};
  std::array<float, 4> mature_bark_stem_palette_rgba{0.7009804f, 0.45447198f, 0.18555366f, 1.0f};
  std::array<float, 4> node_sheath_brown_palette_rgba{0.49f, 0.31f, 0.13f, 1.0f};
  std::array<float, 4> fascicle_sheath_palette_rgba{0.42f, 0.34f, 0.24f, 1.0f};
  float needle_tip_color_mix_start = 0.35f;
  float needle_tip_color_exponent = 1.35f;
  float needle_old_thinning_fraction = 0.33f;
  float needle_min_strand_thickness_m = 0.00002f;
  float needle_micro_variation = 0.012f;
  float stem_micro_variation = 0.012f;
  float young_needle_roughness = 0.92f;
  float old_needle_roughness = 0.97f;
  float young_needle_specular = 0.09f;
  float old_needle_specular = 0.04f;
  float stem_roughness = 0.86f;
  float stem_specular = 0.12f;
  float node_browning_strength = 0.28f;
  float sheath_browning_strength = 0.40f;
  float node_browning_radius_norm = 0.18f;
  float needle_twist_turns = 0.25f;
  float needle_edge_darkening = 0.08f;
  int needle_segment_count = 20;
  float fascicle_sheath_length_m = 0.006f;
  float fascicle_sheath_width_m = 0.0018f;
  float needle_year0_length_multiplier = 1.33f;
  float needle_axial_age_span = 0.34f;
  float needle_axial_age_exponent = 1.0f;

  std::uint32_t ray_trace_samples = 4;
  std::uint32_t ray_trace_bounces = 4;

  bool transparent_bg = true;
  bool load_scene = true;
  bool use_scene_main_camera = true;
  bool use_scene_pine_transforms = true;
  bool use_scene_pine_growth = true;
  bool render_needles = true;
  bool preserve_scene_pine_seed = true;
  bool calendar_start_from_reset = true;
  bool strict_parity = true;
  bool batch_output_subdirs = false;
  bool keep_going = true;
  bool composite_background = true;
  bool write_raw_rgba = false;
  bool write_foreground_mask = true;
  bool override_max_target_gdd = false;
  bool override_scene_pine_target_gdd_multiplier = false;
  bool override_annual_whorl_probability = false;
  bool override_whorl_position_norm = false;
  bool override_branches_per_whorl = false;
  bool override_whorl_dormancy_years = false;
  bool override_max_branching_order = false;
  bool override_young_needle_palette_rgba = false;
  bool override_older_needle_palette_rgba = false;
  bool override_dry_brown_needle_palette_rgba = false;
  bool override_main_stem_palette_rgba = false;
  bool override_mature_bark_stem_palette_rgba = false;
  bool override_node_sheath_brown_palette_rgba = false;
  bool override_fascicle_sheath_palette_rgba = false;
  bool override_needle_tip_color_mix_start = false;
  bool override_needle_tip_color_exponent = false;
  bool override_needle_old_thinning_fraction = false;
  bool override_needle_min_strand_thickness_m = false;
  bool override_needle_micro_variation = false;
  bool override_stem_micro_variation = false;
  bool override_young_needle_roughness = false;
  bool override_old_needle_roughness = false;
  bool override_young_needle_specular = false;
  bool override_old_needle_specular = false;
  bool override_stem_roughness = false;
  bool override_stem_specular = false;
  bool override_node_browning_strength = false;
  bool override_sheath_browning_strength = false;
  bool override_node_browning_radius_norm = false;
  bool override_needle_twist_turns = false;
  bool override_needle_edge_darkening = false;
  bool override_needle_segment_count = false;
  bool override_fascicle_sheath_length_m = false;
  bool override_fascicle_sheath_width_m = false;
  bool override_needle_year0_length_multiplier = false;
  bool override_needle_axial_age_span = false;
  bool override_needle_axial_age_exponent = false;
  bool export_depth = true;
  bool export_instance_mask = true;
  bool export_synthetic_labels = false;
  bool export_flow_graph = true;
  bool export_node_graph = true;
  bool export_needle_skeleton = true;
  bool export_annotation_skeleton = true;
  bool export_annotation_overlay = true;
  bool uncapped_growth = false;
};

inline std::string ScotsPineSyntheticSceneMainCameraLabel() {
  return "scene_main_camera";
}

inline std::string ScotsPineSyntheticSanitizeViewLabel(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (const char ch : value) {
    const bool ok = (std::isalnum(static_cast<unsigned char>(ch)) != 0) || ch == '_' || ch == '-';
    out.push_back(ok ? ch : '_');
  }
  return out.empty() ? "view" : out;
}

inline std::filesystem::path ScotsPineSyntheticExpectedFirstRgbPath(const ScotsPineSyntheticRenderOptions& options) {
  const int global_sample_index = options.start_index;

  std::ostringstream sample_suffix;
  sample_suffix << std::setw(6) << std::setfill('0') << global_sample_index;
  const std::string sample_name =
      (options.batch_output_subdirs || options.sample_count > 1) ? options.output_name + "_" + sample_suffix.str()
                                                                 : options.output_name;
  const std::filesystem::path sample_output_root =
      (options.batch_output_subdirs || options.sample_count > 1) ? options.output_root / ("scene_" + sample_suffix.str())
                                                                 : options.output_root;

  std::ostringstream view_stem;
  view_stem << sample_name << "_f" << std::setw(4) << std::setfill('0') << 0 << "_v" << std::setw(2)
            << std::setfill('0') << 0 << "_" << ScotsPineSyntheticSanitizeViewLabel(ScotsPineSyntheticSceneMainCameraLabel());
  const std::string rgb_extension = options.rgb_output_format == "png" ? ".png" : ".jpg";
  return sample_output_root / (view_stem.str() + "_composited" + rgb_extension);
}

}  // namespace evo_engine
