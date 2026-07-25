#include "DdgiSettings.hpp"

#include "Serialization.hpp"

using namespace evo_engine;

void evo_engine::SerializeDdgiSettings(YAML::Emitter& out, const DdgiSettings& settings) {
  out << YAML::BeginMap;

  out << YAML::Key << "runtime" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << settings.runtime.enabled;
  out << YAML::Key << "pause_updates" << YAML::Value << settings.runtime.pause_updates;
  out << YAML::Key << "enable_emissive_mesh_sampling" << YAML::Value << settings.runtime.enable_emissive_mesh_sampling;
  out << YAML::Key << "ray_count" << YAML::Value << settings.runtime.ray_count;
  out << YAML::Key << "warmup_frames" << YAML::Value << settings.runtime.warmup_frames;
  out << YAML::Key << "hysteresis" << YAML::Value << settings.runtime.hysteresis;
  out << YAML::Key << "normal_bias" << YAML::Value << settings.runtime.normal_bias;
  out << YAML::Key << "view_bias" << YAML::Value << settings.runtime.view_bias;
  out << YAML::Key << "max_ray_distance" << YAML::Value << settings.runtime.max_ray_distance;
  out << YAML::Key << "distance_exponent" << YAML::Value << settings.runtime.distance_exponent;
  out << YAML::Key << "irradiance_gamma" << YAML::Value << settings.runtime.irradiance_gamma;
  out << YAML::Key << "visibility_moment_bias" << YAML::Value << settings.runtime.visibility_moment_bias;
  out << YAML::Key << "irradiance_threshold" << YAML::Value << settings.runtime.irradiance_threshold;
  out << YAML::Key << "brightness_threshold" << YAML::Value << settings.runtime.brightness_threshold;
  out << YAML::Key << "deterministic_ray_seed_enabled" << YAML::Value
      << settings.runtime.deterministic_ray_seed_enabled;
  out << YAML::Key << "deterministic_ray_seed" << YAML::Value << settings.runtime.deterministic_ray_seed;
  out << YAML::EndMap;

  out << YAML::Key << "volume_defaults" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "probe_counts" << YAML::Value << settings.volume_defaults.probe_counts;
  out << YAML::Key << "probe_spacing" << YAML::Value << settings.volume_defaults.probe_spacing;
  out << YAML::Key << "volume_origin" << YAML::Value << settings.volume_defaults.volume_origin;
  out << YAML::Key << "movement_type" << YAML::Value << settings.volume_defaults.movement_type;
  out << YAML::Key << "enable_probe_relocation" << YAML::Value << settings.volume_defaults.enable_probe_relocation;
  out << YAML::Key << "enable_probe_classification" << YAML::Value
      << settings.volume_defaults.enable_probe_classification;
  out << YAML::Key << "enable_probe_variability" << YAML::Value << settings.volume_defaults.enable_probe_variability;
  out << YAML::Key << "enable_probe_variability_gating" << YAML::Value
      << settings.volume_defaults.enable_probe_variability_gating;
  out << YAML::Key << "relocation_distance" << YAML::Value << settings.volume_defaults.relocation_distance;
  out << YAML::Key << "random_ray_backface_threshold" << YAML::Value
      << settings.volume_defaults.random_ray_backface_threshold;
  out << YAML::Key << "fixed_ray_backface_threshold" << YAML::Value
      << settings.volume_defaults.fixed_ray_backface_threshold;
  out << YAML::Key << "probe_variability_threshold" << YAML::Value
      << settings.volume_defaults.probe_variability_threshold;
  out << YAML::Key << "probe_variability_min_samples" << YAML::Value
      << settings.volume_defaults.probe_variability_min_samples;
  out << YAML::EndMap;

  out << YAML::Key << "storage" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "max_probe_count" << YAML::Value << settings.storage.max_probe_count;
  out << YAML::Key << "irradiance_tile_resolution" << YAML::Value << settings.storage.irradiance_tile_resolution;
  out << YAML::Key << "visibility_tile_resolution" << YAML::Value << settings.storage.visibility_tile_resolution;
  out << YAML::Key << "atlas_probe_columns" << YAML::Value << settings.storage.atlas_probe_columns;
  out << YAML::EndMap;

  out << YAML::Key << "debug" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << settings.debug.enabled;
  out << YAML::Key << "visualize_probe_positions" << YAML::Value << settings.debug.visualize_probe_positions;
  out << YAML::Key << "visualize_selected_probe" << YAML::Value << settings.debug.visualize_selected_probe;
  out << YAML::Key << "visualize_probe_state" << YAML::Value << settings.debug.visualize_probe_state;
  out << YAML::Key << "visualize_probe_illumination" << YAML::Value << settings.debug.visualize_probe_illumination;
  out << YAML::Key << "show_rays" << YAML::Value << settings.debug.show_rays;
  out << YAML::Key << "selected_probe_index" << YAML::Value << settings.debug.selected_probe_index;
  out << YAML::Key << "visualization_scale" << YAML::Value << settings.debug.visualization_scale;
  out << YAML::Key << "probe_visualization_mode" << YAML::Value << settings.debug.probe_visualization_mode;
  out << YAML::Key << "probe_visualization_depth_mode" << YAML::Value << settings.debug.probe_visualization_depth_mode;
  out << YAML::Key << "probe_visualization_radius" << YAML::Value << settings.debug.probe_visualization_radius;
  out << YAML::Key << "probe_visualization_intensity" << YAML::Value << settings.debug.probe_visualization_intensity;
  out << YAML::Key << "probe_visualization_alpha" << YAML::Value << settings.debug.probe_visualization_alpha;
  out << YAML::Key << "selected_probe_visualization_scale" << YAML::Value
      << settings.debug.selected_probe_visualization_scale;
  out << YAML::EndMap;

  out << YAML::EndMap;
}

void evo_engine::DeserializeDdgiSettings(const YAML::Node& in, DdgiSettings& settings) {
  if (const auto runtime = in["runtime"]) {
    if (runtime["enabled"])
      settings.runtime.enabled = runtime["enabled"].as<bool>();
    if (runtime["pause_updates"])
      settings.runtime.pause_updates = runtime["pause_updates"].as<bool>();
    if (runtime["enable_emissive_mesh_sampling"])
      settings.runtime.enable_emissive_mesh_sampling = runtime["enable_emissive_mesh_sampling"].as<bool>();
    if (runtime["ray_count"])
      settings.runtime.ray_count = runtime["ray_count"].as<int>();
    if (runtime["warmup_frames"])
      settings.runtime.warmup_frames = runtime["warmup_frames"].as<int>();
    if (runtime["hysteresis"])
      settings.runtime.hysteresis = runtime["hysteresis"].as<float>();
    if (runtime["normal_bias"])
      settings.runtime.normal_bias = runtime["normal_bias"].as<float>();
    if (runtime["view_bias"])
      settings.runtime.view_bias = runtime["view_bias"].as<float>();
    if (runtime["max_ray_distance"])
      settings.runtime.max_ray_distance = runtime["max_ray_distance"].as<float>();
    if (runtime["distance_exponent"])
      settings.runtime.distance_exponent = runtime["distance_exponent"].as<float>();
    if (runtime["irradiance_gamma"])
      settings.runtime.irradiance_gamma = runtime["irradiance_gamma"].as<float>();
    if (runtime["visibility_moment_bias"])
      settings.runtime.visibility_moment_bias = runtime["visibility_moment_bias"].as<float>();
    if (runtime["irradiance_threshold"])
      settings.runtime.irradiance_threshold = runtime["irradiance_threshold"].as<float>();
    if (runtime["brightness_threshold"])
      settings.runtime.brightness_threshold = runtime["brightness_threshold"].as<float>();
    if (runtime["deterministic_ray_seed_enabled"])
      settings.runtime.deterministic_ray_seed_enabled = runtime["deterministic_ray_seed_enabled"].as<bool>();
    if (runtime["deterministic_ray_seed"])
      settings.runtime.deterministic_ray_seed = runtime["deterministic_ray_seed"].as<uint32_t>();
  }
  if (const auto volume_defaults = in["volume_defaults"]) {
    if (volume_defaults["probe_counts"])
      settings.volume_defaults.probe_counts = volume_defaults["probe_counts"].as<glm::ivec3>();
    if (volume_defaults["probe_spacing"])
      settings.volume_defaults.probe_spacing = volume_defaults["probe_spacing"].as<glm::vec3>();
    if (volume_defaults["volume_origin"])
      settings.volume_defaults.volume_origin = volume_defaults["volume_origin"].as<glm::vec3>();
    if (volume_defaults["movement_type"])
      settings.volume_defaults.movement_type = volume_defaults["movement_type"].as<int>();
    if (volume_defaults["enable_probe_relocation"])
      settings.volume_defaults.enable_probe_relocation = volume_defaults["enable_probe_relocation"].as<bool>();
    if (volume_defaults["enable_probe_classification"])
      settings.volume_defaults.enable_probe_classification = volume_defaults["enable_probe_classification"].as<bool>();
    if (volume_defaults["enable_probe_variability"])
      settings.volume_defaults.enable_probe_variability = volume_defaults["enable_probe_variability"].as<bool>();
    if (volume_defaults["enable_probe_variability_gating"])
      settings.volume_defaults.enable_probe_variability_gating =
          volume_defaults["enable_probe_variability_gating"].as<bool>();
    if (volume_defaults["relocation_distance"])
      settings.volume_defaults.relocation_distance = volume_defaults["relocation_distance"].as<float>();
    if (volume_defaults["random_ray_backface_threshold"])
      settings.volume_defaults.random_ray_backface_threshold =
          volume_defaults["random_ray_backface_threshold"].as<float>();
    if (volume_defaults["fixed_ray_backface_threshold"])
      settings.volume_defaults.fixed_ray_backface_threshold =
          volume_defaults["fixed_ray_backface_threshold"].as<float>();
    if (volume_defaults["probe_variability_threshold"])
      settings.volume_defaults.probe_variability_threshold = volume_defaults["probe_variability_threshold"].as<float>();
    if (volume_defaults["probe_variability_min_samples"])
      settings.volume_defaults.probe_variability_min_samples =
          volume_defaults["probe_variability_min_samples"].as<int>();
  }
  if (const auto storage = in["storage"]) {
    if (storage["max_probe_count"])
      settings.storage.max_probe_count = storage["max_probe_count"].as<int>();
    if (storage["irradiance_tile_resolution"])
      settings.storage.irradiance_tile_resolution = storage["irradiance_tile_resolution"].as<int>();
    if (storage["visibility_tile_resolution"])
      settings.storage.visibility_tile_resolution = storage["visibility_tile_resolution"].as<int>();
    if (storage["atlas_probe_columns"])
      settings.storage.atlas_probe_columns = storage["atlas_probe_columns"].as<int>();
  }
  if (const auto debug = in["debug"]) {
    if (debug["enabled"])
      settings.debug.enabled = debug["enabled"].as<bool>();
    if (debug["visualize_probe_positions"])
      settings.debug.visualize_probe_positions = debug["visualize_probe_positions"].as<bool>();
    if (debug["visualize_selected_probe"])
      settings.debug.visualize_selected_probe = debug["visualize_selected_probe"].as<bool>();
    if (debug["visualize_probe_state"])
      settings.debug.visualize_probe_state = debug["visualize_probe_state"].as<bool>();
    if (debug["visualize_probe_illumination"])
      settings.debug.visualize_probe_illumination = debug["visualize_probe_illumination"].as<bool>();
    if (debug["show_rays"])
      settings.debug.show_rays = debug["show_rays"].as<bool>();
    if (debug["selected_probe_index"])
      settings.debug.selected_probe_index = debug["selected_probe_index"].as<int>();
    if (debug["visualization_scale"])
      settings.debug.visualization_scale = debug["visualization_scale"].as<float>();
    if (debug["probe_visualization_mode"])
      settings.debug.probe_visualization_mode = debug["probe_visualization_mode"].as<int>();
    if (debug["probe_visualization_depth_mode"])
      settings.debug.probe_visualization_depth_mode = debug["probe_visualization_depth_mode"].as<int>();
    if (debug["probe_visualization_radius"])
      settings.debug.probe_visualization_radius = debug["probe_visualization_radius"].as<float>();
    if (debug["probe_visualization_intensity"])
      settings.debug.probe_visualization_intensity = debug["probe_visualization_intensity"].as<float>();
    if (debug["probe_visualization_alpha"])
      settings.debug.probe_visualization_alpha = debug["probe_visualization_alpha"].as<float>();
    if (debug["selected_probe_visualization_scale"])
      settings.debug.selected_probe_visualization_scale = debug["selected_probe_visualization_scale"].as<float>();
  }
}
