#include "DdgiSettings.hpp"
#include <tuple>
#include "Serialization.hpp"

using namespace evo_engine;

void evo_engine::SerializeDdgiSettings(YAML::Emitter& out, const DdgiSettings& settings) {
  out << YAML::BeginMap;
  out << YAML::Key << "runtime" << YAML::Value << YAML::BeginMap;
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
  out << YAML::Key << "random_ray_backface_threshold" << YAML::Value << settings.runtime.random_ray_backface_threshold;
  out << YAML::Key << "fixed_ray_backface_threshold" << YAML::Value << settings.runtime.fixed_ray_backface_threshold;
  out << YAML::Key << "visibility_smoothing" << YAML::Value << settings.runtime.visibility_smoothing;
  out << YAML::Key << "enable_probe_relocation" << YAML::Value << settings.runtime.enable_probe_relocation;
  out << YAML::Key << "enable_probe_classification" << YAML::Value << settings.runtime.enable_probe_classification;
  out << YAML::Key << "relocation_distance" << YAML::Value << settings.runtime.relocation_distance;
  out << YAML::Key << "deterministic_ray_seed_enabled" << YAML::Value
      << settings.runtime.deterministic_ray_seed_enabled;
  out << YAML::Key << "deterministic_ray_seed" << YAML::Value << settings.runtime.deterministic_ray_seed;
  out << YAML::EndMap;
  out << YAML::Key << "storage" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "irradiance_tile_resolution" << YAML::Value << settings.storage.irradiance_tile_resolution;
  out << YAML::Key << "visibility_tile_resolution" << YAML::Value << settings.storage.visibility_tile_resolution;
  out << YAML::Key << "atlas_probe_columns" << YAML::Value << settings.storage.atlas_probe_columns;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void evo_engine::DeserializeDdgiSettings(const YAML::Node& in, DdgiSettings& settings) {
  settings = {};
  if (const auto legacy = in["volume_defaults"]; legacy && legacy["enable_probe_relocation"])
    settings.runtime.enable_probe_relocation = legacy["enable_probe_relocation"].as<bool>();
  if (const auto legacy = in["volume_defaults"]; legacy && legacy["enable_probe_classification"])
    settings.runtime.enable_probe_classification = legacy["enable_probe_classification"].as<bool>();
  if (const auto legacy = in["volume_defaults"]; legacy && legacy["relocation_distance"])
    settings.runtime.relocation_distance = legacy["relocation_distance"].as<float>();
  if (const auto values = in["runtime"]) {
    if (values["enable_emissive_mesh_sampling"])
      settings.runtime.enable_emissive_mesh_sampling = values["enable_emissive_mesh_sampling"].as<bool>();
    if (values["ray_count"])
      settings.runtime.ray_count = values["ray_count"].as<int>();
    if (values["emissive_ray_count"])
      settings.runtime.emissive_ray_count = values["emissive_ray_count"].as<int>();
    if (values["warmup_frames"])
      settings.runtime.warmup_frames = values["warmup_frames"].as<int>();
    if (values["history_count"])
      settings.runtime.history_count = values["history_count"].as<int>();
    if (values["normal_bias"])
      settings.runtime.normal_bias = values["normal_bias"].as<float>();
    if (values["view_bias"])
      settings.runtime.view_bias = values["view_bias"].as<float>();
    if (values["max_ray_distance"])
      settings.runtime.max_ray_distance = values["max_ray_distance"].as<float>();
    if (values["distance_exponent"])
      settings.runtime.distance_exponent = values["distance_exponent"].as<float>();
    if (values["irradiance_gamma"])
      settings.runtime.irradiance_gamma = values["irradiance_gamma"].as<float>();
    if (values["visibility_moment_bias"])
      settings.runtime.visibility_moment_bias = values["visibility_moment_bias"].as<float>();
    if (values["visibility_smoothing"])
      settings.runtime.visibility_smoothing = values["visibility_smoothing"].as<float>();
    if (values["enable_probe_relocation"])
      settings.runtime.enable_probe_relocation = values["enable_probe_relocation"].as<bool>();
    if (values["enable_probe_classification"])
      settings.runtime.enable_probe_classification = values["enable_probe_classification"].as<bool>();
    if (values["relocation_distance"])
      settings.runtime.relocation_distance = values["relocation_distance"].as<float>();
    if (values["random_ray_backface_threshold"])
      settings.runtime.random_ray_backface_threshold = values["random_ray_backface_threshold"].as<float>();
    if (values["fixed_ray_backface_threshold"])
      settings.runtime.fixed_ray_backface_threshold = values["fixed_ray_backface_threshold"].as<float>();
    if (values["deterministic_ray_seed_enabled"])
      settings.runtime.deterministic_ray_seed_enabled = values["deterministic_ray_seed_enabled"].as<bool>();
    if (values["deterministic_ray_seed"])
      settings.runtime.deterministic_ray_seed = values["deterministic_ray_seed"].as<uint32_t>();
  }
  if (const auto values = in["storage"]) {
    if (values["irradiance_tile_resolution"])
      settings.storage.irradiance_tile_resolution = values["irradiance_tile_resolution"].as<int>();
    if (values["visibility_tile_resolution"])
      settings.storage.visibility_tile_resolution = values["visibility_tile_resolution"].as<int>();
    if (values["atlas_probe_columns"])
      settings.storage.atlas_probe_columns = values["atlas_probe_columns"].as<int>();
  }
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
  runtime.relocation_distance = glm::clamp(runtime.relocation_distance, 0.0f, 10000.0f);
  storage.irradiance_tile_resolution = glm::clamp(storage.irradiance_tile_resolution, 1, 128);
  storage.visibility_tile_resolution = glm::clamp(storage.visibility_tile_resolution, 1, 128);
  storage.atlas_probe_columns = glm::clamp(storage.atlas_probe_columns, 1, 4096);
}

bool DdgiSettings::operator==(const DdgiSettings& other) const {
  return std::tie(runtime.fixed_ray_backface_threshold, runtime.random_ray_backface_threshold,
                  runtime.enable_emissive_mesh_sampling, runtime.ray_count, runtime.emissive_ray_count,
                  runtime.warmup_frames, runtime.history_count, runtime.normal_bias, runtime.view_bias,
                  runtime.max_ray_distance, runtime.distance_exponent, runtime.irradiance_gamma,
                  runtime.visibility_moment_bias, runtime.visibility_smoothing, runtime.enable_probe_relocation,
                  runtime.enable_probe_classification, runtime.relocation_distance,
                  runtime.deterministic_ray_seed_enabled, runtime.deterministic_ray_seed,
                  storage.irradiance_tile_resolution, storage.visibility_tile_resolution,
                  storage.atlas_probe_columns) ==
         std::tie(other.runtime.fixed_ray_backface_threshold, other.runtime.random_ray_backface_threshold,
                  other.runtime.enable_emissive_mesh_sampling, other.runtime.ray_count,
                  other.runtime.emissive_ray_count, other.runtime.warmup_frames, other.runtime.history_count,
                  other.runtime.normal_bias, other.runtime.view_bias, other.runtime.max_ray_distance,
                  other.runtime.distance_exponent, other.runtime.irradiance_gamma, other.runtime.visibility_moment_bias,
                  other.runtime.visibility_smoothing, other.runtime.enable_probe_relocation,
                  other.runtime.enable_probe_classification, other.runtime.relocation_distance,
                  other.runtime.deterministic_ray_seed_enabled, other.runtime.deterministic_ray_seed,
                  other.storage.irradiance_tile_resolution, other.storage.visibility_tile_resolution,
                  other.storage.atlas_probe_columns);
}
