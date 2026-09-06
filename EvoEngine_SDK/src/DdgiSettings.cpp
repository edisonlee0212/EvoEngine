#include "DdgiSettings.hpp"

#include "Serialization.hpp"

#include <glm/common.hpp>

using namespace evo_engine;

void evo_engine::SerializeDdgiSettings(YAML::Emitter& out, const DdgiSettings& settings) {
  out << YAML::BeginMap;

  out << YAML::Key << "runtime" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << settings.runtime.enabled;
  out << YAML::Key << "enable_emissive_mesh_sampling" << YAML::Value << settings.runtime.enable_emissive_mesh_sampling;
  out << YAML::Key << "ray_count" << YAML::Value << settings.runtime.ray_count;
  out << YAML::Key << "emissive_ray_count" << YAML::Value << settings.runtime.emissive_ray_count;
  out << YAML::Key << "warmup_frames" << YAML::Value << settings.runtime.warmup_frames;
  out << YAML::Key << "history_count" << YAML::Value << settings.runtime.history_count;
  out << YAML::Key << "normal_bias" << YAML::Value << settings.runtime.normal_bias;
  out << YAML::Key << "view_bias" << YAML::Value << settings.runtime.view_bias;
  out << YAML::Key << "max_ray_distance" << YAML::Value << settings.runtime.max_ray_distance;
  out << YAML::Key << "distance_exponent" << YAML::Value << settings.runtime.distance_exponent;
  out << YAML::Key << "irradiance_gamma" << YAML::Value << settings.runtime.irradiance_gamma;
  out << YAML::Key << "visibility_moment_bias" << YAML::Value << settings.runtime.visibility_moment_bias;
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
  out << YAML::Key << "relocation_distance" << YAML::Value << settings.volume_defaults.relocation_distance;
  out << YAML::EndMap;

  out << YAML::Key << "storage" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "max_probe_count" << YAML::Value << settings.storage.max_probe_count;
  out << YAML::Key << "irradiance_tile_resolution" << YAML::Value << settings.storage.irradiance_tile_resolution;
  out << YAML::Key << "visibility_tile_resolution" << YAML::Value << settings.storage.visibility_tile_resolution;
  out << YAML::Key << "atlas_probe_columns" << YAML::Value << settings.storage.atlas_probe_columns;
  out << YAML::EndMap;

  out << YAML::EndMap;
}

void evo_engine::DeserializeDdgiSettings(const YAML::Node& in, DdgiSettings& settings) {
  settings.runtime.history_count = 30;
  if (const auto runtime = in["runtime"]) {
    if (runtime["enabled"])
      settings.runtime.enabled = runtime["enabled"].as<bool>();
    if (runtime["enable_emissive_mesh_sampling"])
      settings.runtime.enable_emissive_mesh_sampling = runtime["enable_emissive_mesh_sampling"].as<bool>();
    if (runtime["ray_count"])
      settings.runtime.ray_count = runtime["ray_count"].as<int>();
    if (runtime["emissive_ray_count"])
      settings.runtime.emissive_ray_count = runtime["emissive_ray_count"].as<int>();
    if (runtime["warmup_frames"])
      settings.runtime.warmup_frames = runtime["warmup_frames"].as<int>();
    if (runtime["history_count"])
      settings.runtime.history_count = runtime["history_count"].as<int>();
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
    if (volume_defaults["relocation_distance"])
      settings.volume_defaults.relocation_distance = volume_defaults["relocation_distance"].as<float>();
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
  settings.ClampSettings();
}

void evo_engine::DdgiSettings::ClampSettings() {
  runtime.ray_count = glm::clamp(runtime.ray_count, 1, 4096);
  runtime.emissive_ray_count = glm::clamp(runtime.emissive_ray_count, 0, 4096);
  runtime.warmup_frames = glm::clamp(runtime.warmup_frames, 0, 4096);
  runtime.normal_bias = glm::clamp(runtime.normal_bias, 0.0f, 10.0f);
  runtime.view_bias = glm::clamp(runtime.view_bias, 0.0f, 10.0f);
  runtime.max_ray_distance = glm::clamp(runtime.max_ray_distance, 0.05f, 1e27f);
  runtime.distance_exponent = glm::clamp(runtime.distance_exponent, 0.0f, 256.0f);
  runtime.irradiance_gamma = glm::clamp(runtime.irradiance_gamma, 0.1f, 16.0f);
  runtime.visibility_moment_bias = glm::clamp(runtime.visibility_moment_bias, 0.0f, 10.0f);
  volume_defaults.probe_counts = glm::clamp(volume_defaults.probe_counts, glm::ivec3(1), glm::ivec3(256));
  volume_defaults.probe_spacing = glm::clamp(volume_defaults.probe_spacing, glm::vec3(0.05f), glm::vec3(10000.0f));
  volume_defaults.movement_type =
      glm::clamp(volume_defaults.movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                 static_cast<int>(DdgiVolumeMovementType::Scrolling));
  volume_defaults.relocation_distance = glm::clamp(volume_defaults.relocation_distance, 0.0f, 10000.0f);
  storage.max_probe_count = glm::clamp(storage.max_probe_count, 1, 16777216);
  storage.irradiance_tile_resolution = glm::clamp(storage.irradiance_tile_resolution, 1, 128);
  storage.visibility_tile_resolution = glm::clamp(storage.visibility_tile_resolution, 1, 128);
  storage.atlas_probe_columns = glm::clamp(storage.atlas_probe_columns, 1, 4096);
}
