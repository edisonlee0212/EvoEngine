#include "Scene.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "Entities.hpp"
#include "EntityMetadata.hpp"
#include "Jobs.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "Resources.hpp"
#include "Serialization.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "UnknownPrivateComponent.hpp"

#include <system_error>

using namespace evo_engine;

void WriteSceneSystem(const std::shared_ptr<ISystem>& system, YAML::Emitter& out);

namespace {
glm::vec3 DeserializeDdgiProbeSpacing(const YAML::Node& in) {
  if (in.IsSequence()) {
    return in.as<glm::vec3>();
  }
  return glm::vec3(in.as<float>());
}

void SerializeDdgiSettings(YAML::Emitter& out, const DdgiSettings& settings) {
  out << YAML::BeginMap;

  out << YAML::Key << "runtime" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << settings.runtime.enabled;
  out << YAML::Key << "pause_updates" << YAML::Value << settings.runtime.pause_updates;
  out << YAML::Key << "ray_count" << YAML::Value << settings.runtime.ray_count;
  out << YAML::Key << "warmup_frames" << YAML::Value << settings.runtime.warmup_frames;
  out << YAML::Key << "hysteresis" << YAML::Value << settings.runtime.hysteresis;
  out << YAML::Key << "normal_bias" << YAML::Value << settings.runtime.normal_bias;
  out << YAML::Key << "view_bias" << YAML::Value << settings.runtime.view_bias;
  out << YAML::Key << "max_ray_distance" << YAML::Value << settings.runtime.max_ray_distance;
  out << YAML::Key << "distance_exponent" << YAML::Value << settings.runtime.distance_exponent;
  out << YAML::Key << "irradiance_gamma" << YAML::Value << settings.runtime.irradiance_gamma;
  out << YAML::Key << "visibility_moment_bias" << YAML::Value << settings.runtime.visibility_moment_bias;
  out << YAML::Key << "indirect_intensity" << YAML::Value << settings.runtime.indirect_intensity;
  out << YAML::Key << "irradiance_threshold" << YAML::Value << settings.runtime.irradiance_threshold;
  out << YAML::Key << "brightness_threshold" << YAML::Value << settings.runtime.brightness_threshold;
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
  out << YAML::Key << "visualize_volume_bounds" << YAML::Value << settings.debug.visualize_volume_bounds;
  out << YAML::Key << "visualize_probe_positions" << YAML::Value << settings.debug.visualize_probe_positions;
  out << YAML::Key << "visualize_selected_probe" << YAML::Value << settings.debug.visualize_selected_probe;
  out << YAML::Key << "visualize_probe_state" << YAML::Value << settings.debug.visualize_probe_state;
  out << YAML::Key << "visualize_probe_illumination" << YAML::Value << settings.debug.visualize_probe_illumination;
  out << YAML::Key << "show_atlas_preview" << YAML::Value << settings.debug.show_atlas_preview;
  out << YAML::Key << "show_update_age" << YAML::Value << settings.debug.show_update_age;
  out << YAML::Key << "show_rays" << YAML::Value << settings.debug.show_rays;
  out << YAML::Key << "show_irradiance" << YAML::Value << settings.debug.show_irradiance;
  out << YAML::Key << "show_visibility" << YAML::Value << settings.debug.show_visibility;
  out << YAML::Key << "show_sampling_weights" << YAML::Value << settings.debug.show_sampling_weights;
  out << YAML::Key << "selected_probe_index" << YAML::Value << settings.debug.selected_probe_index;
  out << YAML::Key << "atlas_layer" << YAML::Value << settings.debug.atlas_layer;
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

void DeserializeDdgiSettings(const YAML::Node& in, DdgiSettings& settings) {
  if (const auto runtime = in["runtime"]) {
    if (runtime["enabled"])
      settings.runtime.enabled = runtime["enabled"].as<bool>();
    if (runtime["pause_updates"])
      settings.runtime.pause_updates = runtime["pause_updates"].as<bool>();
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
    if (runtime["indirect_intensity"])
      settings.runtime.indirect_intensity = runtime["indirect_intensity"].as<float>();
    if (runtime["irradiance_threshold"])
      settings.runtime.irradiance_threshold = runtime["irradiance_threshold"].as<float>();
    if (runtime["brightness_threshold"])
      settings.runtime.brightness_threshold = runtime["brightness_threshold"].as<float>();
  }
  if (const auto volume_defaults = in["volume_defaults"]) {
    if (volume_defaults["probe_counts"])
      settings.volume_defaults.probe_counts = volume_defaults["probe_counts"].as<glm::ivec3>();
    if (volume_defaults["probe_spacing"])
      settings.volume_defaults.probe_spacing = DeserializeDdgiProbeSpacing(volume_defaults["probe_spacing"]);
    if (volume_defaults["volume_origin"])
      settings.volume_defaults.volume_origin = volume_defaults["volume_origin"].as<glm::vec3>();
    else if (volume_defaults["volume_offset"])
      settings.volume_defaults.volume_origin =
          volume_defaults["volume_offset"].as<glm::vec3>() +
          glm::vec3(glm::clamp(settings.volume_defaults.probe_counts.x, 1, 256) - 1,
                    glm::clamp(settings.volume_defaults.probe_counts.y, 1, 256) - 1,
                    glm::clamp(settings.volume_defaults.probe_counts.z, 1, 256) - 1) *
              (glm::clamp(settings.volume_defaults.probe_spacing, glm::vec3(0.05f), glm::vec3(10000.0f)) * 0.5f);
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
    if (debug["visualize_volume_bounds"])
      settings.debug.visualize_volume_bounds = debug["visualize_volume_bounds"].as<bool>();
    if (debug["visualize_probe_positions"])
      settings.debug.visualize_probe_positions = debug["visualize_probe_positions"].as<bool>();
    if (debug["visualize_selected_probe"])
      settings.debug.visualize_selected_probe = debug["visualize_selected_probe"].as<bool>();
    if (debug["visualize_probe_state"])
      settings.debug.visualize_probe_state = debug["visualize_probe_state"].as<bool>();
    if (debug["visualize_probe_illumination"])
      settings.debug.visualize_probe_illumination = debug["visualize_probe_illumination"].as<bool>();
    if (debug["show_atlas_preview"])
      settings.debug.show_atlas_preview = debug["show_atlas_preview"].as<bool>();
    if (debug["show_update_age"])
      settings.debug.show_update_age = debug["show_update_age"].as<bool>();
    if (debug["show_rays"])
      settings.debug.show_rays = debug["show_rays"].as<bool>();
    if (debug["show_irradiance"])
      settings.debug.show_irradiance = debug["show_irradiance"].as<bool>();
    if (debug["show_visibility"])
      settings.debug.show_visibility = debug["show_visibility"].as<bool>();
    if (debug["show_sampling_weights"])
      settings.debug.show_sampling_weights = debug["show_sampling_weights"].as<bool>();
    if (debug["selected_probe_index"])
      settings.debug.selected_probe_index = debug["selected_probe_index"].as<int>();
    if (debug["atlas_layer"])
      settings.debug.atlas_layer = debug["atlas_layer"].as<int>();
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

void SerializeVolumetricCloudSettings(YAML::Emitter& out, const VolumetricCloudSettings& settings) {
  out << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << settings.enabled;
  out << YAML::Key << "coverage" << YAML::Value << settings.coverage;
  out << YAML::Key << "density" << YAML::Value << settings.density;
  out << YAML::Key << "bottom_altitude" << YAML::Value << settings.bottom_altitude;
  out << YAML::Key << "top_altitude" << YAML::Value << settings.top_altitude;
  out << YAML::Key << "max_march_distance" << YAML::Value << settings.max_march_distance;
  out << YAML::Key << "wind_direction" << YAML::Value << settings.wind_direction;
  out << YAML::Key << "wind_speed" << YAML::Value << settings.wind_speed;
  out << YAML::Key << "primary_step_count" << YAML::Value << settings.primary_step_count;
  out << YAML::Key << "light_step_count" << YAML::Value << settings.light_step_count;
  out << YAML::Key << "resolution_divisor" << YAML::Value << settings.resolution_divisor;
  out << YAML::Key << "lighting_intensity" << YAML::Value << settings.lighting_intensity;
  out << YAML::Key << "ambient_lighting_strength" << YAML::Value << settings.ambient_lighting_strength;
  out << YAML::Key << "phase_anisotropy" << YAML::Value << settings.phase_anisotropy;
  out << YAML::Key << "base_noise_scale" << YAML::Value << settings.base_noise_scale;
  out << YAML::Key << "detail_noise_scale" << YAML::Value << settings.detail_noise_scale;
  out << YAML::Key << "extinction_scale" << YAML::Value << settings.extinction_scale;
  out << YAML::Key << "use_spherical_atmosphere" << YAML::Value << settings.use_spherical_atmosphere;
  out << YAML::Key << "atmosphere_radius" << YAML::Value << settings.atmosphere_radius;
  out << YAML::Key << "cloud_type" << YAML::Value << settings.cloud_type;
  out << YAML::Key << "curl_strength" << YAML::Value << settings.curl_strength;
  out << YAML::Key << "coarse_step_fraction" << YAML::Value << settings.coarse_step_fraction;
  out << YAML::Key << "fine_step_scale" << YAML::Value << settings.fine_step_scale;
  out << YAML::Key << "empty_step_fallback_count" << YAML::Value << settings.empty_step_fallback_count;
  out << YAML::Key << "enable_temporal_reprojection" << YAML::Value << settings.enable_temporal_reprojection;
  out << YAML::Key << "temporal_blend_factor" << YAML::Value << settings.temporal_blend_factor;
  out << YAML::Key << "enable_cloud_shadows" << YAML::Value << settings.enable_cloud_shadows;
  out << YAML::Key << "cloud_shadow_strength" << YAML::Value << settings.cloud_shadow_strength;
  out << YAML::Key << "cloud_shadow_step_count" << YAML::Value << settings.cloud_shadow_step_count;
  out << YAML::Key << "debug_visualization" << YAML::Value << settings.debug_visualization;
  out << YAML::Key << "debug_mode" << YAML::Value << settings.debug_mode;
  out << YAML::EndMap;
}

bool AlmostEqual(const float left, const float right) {
  return glm::abs(left - right) <= 0.0001f;
}

bool MatchesLegacyVolumetricCloudDefaults(const VolumetricCloudSettings& settings) {
  const bool common_default = AlmostEqual(settings.wind_direction.x, 1.0f) &&
                              AlmostEqual(settings.wind_direction.y, 0.0f) && settings.debug_mode == 0 &&
                              !settings.debug_visualization;
  const bool first_visible_default =
      AlmostEqual(settings.coverage, 0.82f) && AlmostEqual(settings.density, 2.0f) &&
      AlmostEqual(settings.bottom_altitude, 0.0f) && AlmostEqual(settings.top_altitude, 160.0f) &&
      AlmostEqual(settings.max_march_distance, 600.0f) && AlmostEqual(settings.wind_speed, 25.0f) &&
      settings.primary_step_count == 64 && settings.light_step_count == 8 && settings.resolution_divisor == 1 &&
      AlmostEqual(settings.lighting_intensity, 1.6f) && AlmostEqual(settings.ambient_lighting_strength, 0.35f) &&
      AlmostEqual(settings.phase_anisotropy, 0.65f) && AlmostEqual(settings.base_noise_scale, 0.035f) &&
      AlmostEqual(settings.detail_noise_scale, 0.14f) && AlmostEqual(settings.extinction_scale, 0.035f);
  const bool high_altitude_default =
      AlmostEqual(settings.coverage, 0.65f) && AlmostEqual(settings.density, 1.0f) &&
      AlmostEqual(settings.bottom_altitude, 80.0f) && AlmostEqual(settings.top_altitude, 550.0f) &&
      AlmostEqual(settings.max_march_distance, 5000.0f) && AlmostEqual(settings.wind_speed, 25.0f) &&
      settings.primary_step_count == 64 && settings.light_step_count == 8 && settings.resolution_divisor == 1 &&
      AlmostEqual(settings.lighting_intensity, 1.0f) && AlmostEqual(settings.ambient_lighting_strength, 0.2f) &&
      AlmostEqual(settings.phase_anisotropy, 0.65f) && AlmostEqual(settings.base_noise_scale, 0.012f) &&
      AlmostEqual(settings.detail_noise_scale, 0.05f) && AlmostEqual(settings.extinction_scale, 0.01f);
  return common_default && (first_visible_default || high_altitude_default);
}

void UpgradeLegacyVolumetricCloudDefaults(VolumetricCloudSettings& settings) {
  settings = VolumetricCloudSettings{};
}

void DeserializeVolumetricCloudSettings(const YAML::Node& in, VolumetricCloudSettings& settings) {
  if (in["enabled"])
    settings.enabled = in["enabled"].as<bool>();
  if (in["coverage"])
    settings.coverage = in["coverage"].as<float>();
  if (in["density"])
    settings.density = in["density"].as<float>();
  if (in["bottom_altitude"])
    settings.bottom_altitude = in["bottom_altitude"].as<float>();
  if (in["top_altitude"])
    settings.top_altitude = in["top_altitude"].as<float>();
  if (in["max_march_distance"])
    settings.max_march_distance = in["max_march_distance"].as<float>();
  if (in["wind_direction"])
    settings.wind_direction = in["wind_direction"].as<glm::vec2>();
  if (in["wind_speed"])
    settings.wind_speed = in["wind_speed"].as<float>();
  if (in["primary_step_count"])
    settings.primary_step_count = in["primary_step_count"].as<int>();
  if (in["light_step_count"])
    settings.light_step_count = in["light_step_count"].as<int>();
  if (in["resolution_divisor"])
    settings.resolution_divisor = in["resolution_divisor"].as<int>();
  if (in["lighting_intensity"])
    settings.lighting_intensity = in["lighting_intensity"].as<float>();
  if (in["ambient_lighting_strength"])
    settings.ambient_lighting_strength = in["ambient_lighting_strength"].as<float>();
  if (in["phase_anisotropy"])
    settings.phase_anisotropy = in["phase_anisotropy"].as<float>();
  if (in["base_noise_scale"])
    settings.base_noise_scale = in["base_noise_scale"].as<float>();
  if (in["detail_noise_scale"])
    settings.detail_noise_scale = in["detail_noise_scale"].as<float>();
  if (in["extinction_scale"])
    settings.extinction_scale = in["extinction_scale"].as<float>();
  if (in["use_spherical_atmosphere"])
    settings.use_spherical_atmosphere = in["use_spherical_atmosphere"].as<bool>();
  if (in["atmosphere_radius"])
    settings.atmosphere_radius = in["atmosphere_radius"].as<float>();
  if (in["cloud_type"])
    settings.cloud_type = in["cloud_type"].as<float>();
  if (in["curl_strength"])
    settings.curl_strength = in["curl_strength"].as<float>();
  if (in["coarse_step_fraction"])
    settings.coarse_step_fraction = in["coarse_step_fraction"].as<float>();
  if (in["fine_step_scale"])
    settings.fine_step_scale = in["fine_step_scale"].as<float>();
  if (in["empty_step_fallback_count"])
    settings.empty_step_fallback_count = in["empty_step_fallback_count"].as<int>();
  if (in["enable_temporal_reprojection"])
    settings.enable_temporal_reprojection = in["enable_temporal_reprojection"].as<bool>();
  if (in["temporal_blend_factor"])
    settings.temporal_blend_factor = in["temporal_blend_factor"].as<float>();
  if (in["enable_cloud_shadows"])
    settings.enable_cloud_shadows = in["enable_cloud_shadows"].as<bool>();
  if (in["cloud_shadow_strength"])
    settings.cloud_shadow_strength = in["cloud_shadow_strength"].as<float>();
  if (in["cloud_shadow_step_count"])
    settings.cloud_shadow_step_count = in["cloud_shadow_step_count"].as<int>();
  if (in["debug_visualization"])
    settings.debug_visualization = in["debug_visualization"].as<bool>();
  if (in["debug_mode"])
    settings.debug_mode = in["debug_mode"].as<int>();
  settings.ClampSettings();
  if (MatchesLegacyVolumetricCloudDefaults(settings)) {
    UpgradeLegacyVolumetricCloudDefaults(settings);
  }
}
}  // namespace

Entity evo_engine::MakeSceneEntity(const uint32_t index, const uint32_t version) {
  Entity entity;
  entity.index_ = index;
  entity.version_ = version;
  return entity;
}

void Scene::Purge() {
  pressed_keys_.clear();
  main_camera.Clear();

  scene_data_storage_.entity_private_component_storage = PrivateComponentStorage();
  scene_data_storage_.entity_private_component_storage.owner_scene = std::dynamic_pointer_cast<Scene>(GetSelf());
  scene_data_storage_.entities.clear();
  scene_data_storage_.entity_metadata_list.clear();
  scene_data_storage_.data_component_storage_list.clear();

  scene_data_storage_.data_component_storage_list.emplace_back();
  scene_data_storage_.entities.emplace_back();
  scene_data_storage_.entity_metadata_list.emplace_back();
}

Bound Scene::GetBound() const {
  return world_bound_;
}

void Scene::SetBound(const Bound& value) {
  world_bound_ = value;
}

Scene::~Scene() {
  Purge();
  for (const auto& i : systems_) {
    i.second->OnDestroy();
  }
}

void Scene::Start() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_)
        continue;
      if (!private_component_element.private_component_data->started_) {
        private_component_element.private_component_data->Start();
        if (entity.version_ != entity_info.entity_version)
          break;
        private_component_element.private_component_data->started_ = true;
      }
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }
  for (auto& i : systems_) {
    if (i.second->Enabled()) {
      if (!i.second->started_) {
        i.second->Start();
        i.second->started_ = true;
      }
    }
  }
}

void Scene::Update() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_ ||
          !private_component_element.private_component_data->started_)
        continue;
      private_component_element.private_component_data->Update();
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }

  for (auto& i : systems_) {
    if (i.second->Enabled() && i.second->started_) {
      i.second->Update();
    }
  }
}

void Scene::LateUpdate() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_ ||
          !private_component_element.private_component_data->started_)
        continue;
      private_component_element.private_component_data->LateUpdate();
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }

  for (auto& i : systems_) {
    if (i.second->Enabled() && i.second->started_) {
      i.second->LateUpdate();
    }
  }
}
void Scene::FixedUpdate() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_ ||
          !private_component_element.private_component_data->started_)
        continue;
      private_component_element.private_component_data->FixedUpdate();
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }

  for (const auto& i : systems_) {
    if (i.second->Enabled() && i.second->started_) {
      i.second->FixedUpdate();
    }
  }
}
const std::multimap<float, std::shared_ptr<ISystem>>& Scene::PeekSystems() const {
  return systems_;
}

bool Scene::HasSystemType(const size_t& type_id) const {
  return indexed_systems_.find(type_id) != indexed_systems_.end();
}

std::shared_ptr<ISystem> Scene::CreateSystemByTypeId(const size_t& type_id, const float order) {
  return CreateSystem(type_id, order);
}

std::shared_ptr<Scene> Scene::GetSelfScene() {
  return std::dynamic_pointer_cast<Scene>(GetSelf());
}

std::shared_ptr<ISystem> Scene::GetOrCreateSystem(const std::string& system_name, float order) {
  size_t type_index;
  const auto ptr = Serialization::ProduceSerializable(system_name, type_index);
  auto system = std::dynamic_pointer_cast<ISystem>(ptr);
  system->scene_ = std::dynamic_pointer_cast<Scene>(GetSelf());
  system->handle_ = Handle();
  system->rank_ = order;
  systems_.insert({order, system});
  indexed_systems_[type_index] = system;
  mapped_systems_[system->handle_] = system;
  system->started_ = false;
  system->OnCreate();
  SetUnsaved();
  return std::dynamic_pointer_cast<ISystem>(ptr);
}

void evo_engine::SerializeScene(YAML::Emitter& out, const Scene& scene) {
  const auto self = const_cast<Scene&>(scene).GetSelfScene();
  const auto& environment = scene.environment;
  const auto& main_camera = scene.main_camera;
  const auto& scene_data_storage_ = scene.scene_data_storage_;
  const auto& systems_ = scene.systems_;
  out << YAML::Key << "environment" << YAML::Value << YAML::BeginMap;
  environment.Serialize(out);
  out << YAML::EndMap;
  main_camera.Save("main_camera", out);
  std::unordered_map<Handle, std::shared_ptr<IAsset>> asset_map;
  std::vector<AssetRef> list;
  list.push_back(environment.environmental_map);
  auto& scene_data_storage = scene_data_storage_;
#pragma region EntityInfo
  out << YAML::Key << "entity_metadata_list" << YAML::Value << YAML::BeginSeq;
  for (size_t i = 1; i < scene_data_storage.entity_metadata_list.size(); i++) {
    auto& entity_metadata = scene_data_storage.entity_metadata_list[i];
    if (entity_metadata.entity_handle == 0)
      continue;
    for (const auto& element : entity_metadata.private_component_elements) {
      Serialization::CollectAssetRefs(*element.private_component_data, list);
    }
    entity_metadata.Serialize(out, self);
  }
  out << YAML::EndSeq;
#pragma endregion

#pragma region Systems
  out << YAML::Key << "systems_" << YAML::Value << YAML::BeginSeq;
  for (const auto& i : systems_) {
    WriteSceneSystem(i.second, out);
    Serialization::CollectAssetRefs(*i.second, list);
  }
  out << YAML::EndSeq;
#pragma endregion

#pragma region Assets
  for (auto& i : list) {
    const auto asset = i.Get<IAsset>();

    if (asset && !Resources::IsResource(asset->GetHandle())) {
      if (asset->IsTemporary()) {
        asset_map[asset->GetHandle()] = asset;
      } else if (!asset->Saved()) {
        asset->Save();
      }
    }
  }
  bool list_check = true;
  while (list_check) {
    const size_t current_size = asset_map.size();
    list.clear();
    for (const auto& i : asset_map) {
      Serialization::CollectAssetRefs(*i.second, list);
    }
    for (auto& i : list) {
      if (const auto asset = i.Get<IAsset>(); asset && !Resources::IsResource(asset->GetHandle())) {
        if (asset->IsTemporary()) {
          asset_map[asset->GetHandle()] = asset;
        } else if (!asset->Saved()) {
          asset->Save();
        }
      }
    }
    if (asset_map.size() == current_size)
      list_check = false;
  }
  if (!asset_map.empty()) {
    out << YAML::Key << "LocalAssets" << YAML::Value << YAML::BeginSeq;
    for (auto& i : asset_map) {
      out << YAML::BeginMap;
      if (const auto unknown_asset = std::dynamic_pointer_cast<UnknownAsset>(i.second)) {
        out << YAML::Key << "type_name" << YAML::Value << unknown_asset->GetOriginalTypeName();
      } else {
        out << YAML::Key << "type_name" << YAML::Value << i.second->GetTypeName();
      }
      out << YAML::Key << "handle" << YAML::Value << i.second->GetHandle();
      Serialization::SerializeObject(out, *i.second);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
#pragma endregion

#pragma region DataComponentStorage
  out << YAML::Key << "data_component_storage_list" << YAML::Value << YAML::BeginSeq;
  for (size_t i = 1; i < scene_data_storage.data_component_storage_list.size(); i++) {
    WriteSceneDataComponentStorage(scene, scene_data_storage.data_component_storage_list[i], out);
  }
  out << YAML::EndSeq;
#pragma endregion
  out << YAML::EndMap;
}
void evo_engine::DeserializeScene(const YAML::Node& in, Scene& scene) {
  auto& environment = scene.environment;
  auto& main_camera = scene.main_camera;
  auto& scene_data_storage_ = scene.scene_data_storage_;
  auto& systems_ = scene.systems_;
  auto& indexed_systems_ = scene.indexed_systems_;
  auto& mapped_systems_ = scene.mapped_systems_;
  scene.Purge();
  auto self = scene.GetSelfScene();
  scene_data_storage_.entities.clear();
  scene_data_storage_.entity_metadata_list.clear();
  scene_data_storage_.data_component_storage_list.clear();
  scene_data_storage_.entities.emplace_back();
  scene_data_storage_.entity_metadata_list.emplace_back();
  scene_data_storage_.data_component_storage_list.emplace_back();

#pragma region EntityMetadata
  auto in_entity_metadata_list = in["entity_metadata_list"];
  int current_index = 1;
  for (const auto& in_entity_metadata : in_entity_metadata_list) {
    scene_data_storage_.entity_metadata_list.emplace_back();
    auto& new_info = scene_data_storage_.entity_metadata_list.back();
    new_info.Deserialize(in_entity_metadata, self);
    Entity entity = MakeSceneEntity(current_index, 1);
    scene_data_storage_.entity_map[new_info.entity_handle] = entity;
    scene_data_storage_.entities.push_back(entity);
    current_index++;
  }
  current_index = 1;
  for (const auto& in_entity_metadata : in_entity_metadata_list) {
    auto& metadata = scene_data_storage_.entity_metadata_list[current_index];
    if (in_entity_metadata["p"]) {
      metadata.parent = scene_data_storage_.entity_map[Handle(in_entity_metadata["p"].as<uint64_t>())];
      auto& parent_metadata = scene_data_storage_.entity_metadata_list[metadata.parent.GetIndex()];
      Entity entity = MakeSceneEntity(current_index, 1);
      parent_metadata.children.push_back(entity);
    }
    if (in_entity_metadata["r"])
      metadata.root = scene_data_storage_.entity_map[Handle(in_entity_metadata["r"].as<uint64_t>())];
    current_index++;
  }
#pragma endregion

#pragma region DataComponentStorage
  const auto& in_data_component_storage_list = in["data_component_storage_list"];
  int storage_index = 1;
  for (const auto& in_data_component_storage : in_data_component_storage_list) {
    scene_data_storage_.data_component_storage_list.emplace_back();
    auto& data_component_storage = scene_data_storage_.data_component_storage_list.back();
    ReadSceneDataComponentStorage(scene, storage_index, data_component_storage, in_data_component_storage);
    storage_index++;
  }
#pragma endregion
  main_camera.Load("main_camera", in, self);
#pragma region Assets
  std::vector<std::pair<int, std::shared_ptr<IAsset>>> local_assets;
  if (const auto in_local_assets = in["LocalAssets"]) {
    int index = 0;
    for (const auto& i : in_local_assets) {
      // First, find the asset in asset registry
      const auto type_name = i["type_name"].as<std::string>();
      const auto handle = Handle(i["handle"].as<uint64_t>());
      if (Serialization::HasSerializableType(type_name)) {
        auto asset = AssetManager::CreateTemporaryAsset(type_name, handle);
        if (asset) {
          local_assets.emplace_back(index, asset);
        }
      } else if (auto asset = AssetManager::CreateTemporaryAsset("UnknownAsset", handle)) {
        if (auto unknown_asset = std::dynamic_pointer_cast<UnknownAsset>(asset)) {
          unknown_asset->SetOriginalTypeName(type_name);
          unknown_asset->SetSerializedNode(i);
        }
        local_assets.emplace_back(index, asset);
      }
      index++;
    }
    for (const auto& i : local_assets) {
      Serialization::DeserializeObject(in_local_assets[i.first], *i.second);
    }
  }
#ifdef _DEBUG
  EVOENGINE_LOG(std::string("Scene Deserialization: Loaded " + std::to_string(local_assets.size()) + " assets."))
#endif
#pragma endregion
  if (in["environment"])
    environment.Deserialize(in["environment"]);
  int entity_index = 1;
  for (const auto& in_entity_info : in_entity_metadata_list) {
    auto& entity_metadata = scene_data_storage_.entity_metadata_list.at(entity_index);
    auto entity = scene_data_storage_.entities[entity_index];
    if (auto in_private_components = in_entity_info["pc"]) {
      for (const auto& in_private_component : in_private_components) {
        const auto name = in_private_component["tn"].as<std::string>();
        size_t hash_code;
        if (Serialization::HasSerializableType(name)) {
          auto ptr = std::static_pointer_cast<IPrivateComponent>(Serialization::ProduceSerializable(name, hash_code));
          ptr->enabled_ = in_private_component["e"].as<bool>();
          ptr->started_ = false;
          scene_data_storage_.entity_private_component_storage.SetPrivateComponent(entity, hash_code);
          entity_metadata.private_component_elements.emplace_back(hash_code, ptr, entity, self);
        } else {
          auto ptr = std::static_pointer_cast<IPrivateComponent>(
              Serialization::ProduceSerializable("UnknownPrivateComponent", hash_code));
          hash_code = std::hash<std::string>{}(name);
          ptr->enabled_ = false;
          ptr->started_ = false;
          if (auto unknown_component = std::dynamic_pointer_cast<UnknownPrivateComponent>(ptr)) {
            unknown_component->SetOriginalTypeName(name);
            unknown_component->SetSerializedNode(in_private_component);
          }
          scene_data_storage_.entity_private_component_storage.SetPrivateComponent(entity, hash_code);
          entity_metadata.private_component_elements.emplace_back(hash_code, ptr, entity, self);
        }
      }
    }
    entity_index++;
  }

#pragma region Systems
  if (auto in_systems = in["systems_"]) {
    std::vector<std::pair<int, std::shared_ptr<ISystem>>> systems;
    int index = 0;
    for (const auto& in_system : in_systems) {
      const auto type_name = in_system["type_name"].as<std::string>();
      if (Serialization::HasSerializableType(type_name)) {
        size_t hash_code;
        if (const auto ptr = std::static_pointer_cast<ISystem>(Serialization::ProduceSerializable(
                type_name, hash_code, Handle(in_system["handle_"].as<uint64_t>())))) {
          ptr->enabled_ = in_system["enabled_"].as<bool>();
          ptr->rank_ = in_system["rank_"].as<float>();
          ptr->started_ = false;
          systems_.insert({ptr->rank_, ptr});
          indexed_systems_.insert({hash_code, ptr});
          mapped_systems_[ptr->GetHandle()] = ptr;
          systems.emplace_back(index, ptr);
          ptr->scene_ = self;
          ptr->OnCreate();
        }
      } else {
        size_t hash_code;
        if (const auto ptr = std::static_pointer_cast<ISystem>(Serialization::ProduceSerializable(
                "UnknownSystem", hash_code, Handle(in_system["handle_"].as<uint64_t>())))) {
          hash_code = std::hash<std::string>{}(type_name);
          ptr->enabled_ = in_system["enabled_"].as<bool>();
          ptr->rank_ = in_system["rank_"].as<float>();
          ptr->started_ = false;
          if (auto unknown_system = std::dynamic_pointer_cast<UnknownSystem>(ptr)) {
            unknown_system->SetOriginalTypeName(type_name);
            unknown_system->SetSerializedNode(in_system);
          }
          systems_.insert({ptr->rank_, ptr});
          indexed_systems_.insert({hash_code, ptr});
          mapped_systems_[ptr->GetHandle()] = ptr;
          systems.emplace_back(index, ptr);
          ptr->scene_ = self;
          ptr->OnCreate();
        }
      }
      index++;
    }
#pragma endregion

    entity_index = 1;
    for (const auto& in_entity_metadata : in_entity_metadata_list) {
      auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
      if (auto in_private_components = in_entity_metadata["pc"]) {
        int component_index = 0;
        for (const auto& in_private_component : in_private_components) {
          auto name = in_private_component["tn"].as<std::string>();
          auto ptr = entity_info.private_component_elements[component_index].private_component_data;
          Serialization::DeserializeObject(in_private_component, *ptr);
          ptr->enabled_ = in_private_component["e"].as<bool>();
          component_index++;
        }
      }
      entity_index++;
    }

    for (const auto& i : systems) {
      Serialization::DeserializeObject(in_systems[i.first], *i.second);
    }
  }
}
void evo_engine::WriteSceneDataComponentStorage(const Scene& scene, const DataComponentStorage& storage,
                                                YAML::Emitter& out) {
  const auto& scene_data_storage_ = scene.scene_data_storage_;
  out << YAML::BeginMap;
  {
    out << YAML::Key << "entity_size" << YAML::Value << storage.entity_size;
    out << YAML::Key << "chunk_capacity" << YAML::Value << storage.chunk_capacity;
    out << YAML::Key << "entity_alive_count" << YAML::Value << storage.entity_alive_count;
    out << YAML::Key << "data_component_types" << YAML::Value << YAML::BeginSeq;
    for (const auto& i : storage.data_component_types) {
      out << YAML::BeginMap;
      out << YAML::Key << "type_name" << YAML::Value << i.type_name;
      out << YAML::Key << "type_size" << YAML::Value << i.type_size;
      out << YAML::Key << "type_offset" << YAML::Value << i.type_offset;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "chunk_array" << YAML::Value << YAML::BeginSeq;
    for (size_t i = 0; i < storage.entity_alive_count; i++) {
      const auto entity = storage.chunk_array.entity_array[i];
      if (entity.GetVersion() == 0)
        continue;

      out << YAML::BeginMap;
      auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity.GetIndex());
      out << YAML::Key << "h" << YAML::Value << entity_info.entity_handle;

      auto& data_component_storage =
          scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
      const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
      const auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
      auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];

      out << YAML::Key << "dc" << YAML::Value << YAML::BeginSeq;
      for (const auto& type : data_component_storage.data_component_types) {
        out << YAML::BeginMap;
        out << YAML::Key << "d" << YAML::Value
            << YAML::Binary(
                   static_cast<const unsigned char*>(chunk.PeekData(
                       type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size)),
                   type.type_size);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;

      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;
}

void evo_engine::ReadSceneDataComponentStorage(Scene& scene, const size_t storage_index,
                                               DataComponentStorage& data_component_storage, const YAML::Node& in) {
  auto& scene_data_storage_ = scene.scene_data_storage_;
  if (in["entity_size"])
    data_component_storage.entity_size = in["entity_size"].as<size_t>();
  if (in["chunk_capacity"])
    data_component_storage.chunk_capacity = in["chunk_capacity"].as<size_t>();
  if (in["entity_alive_count"])
    data_component_storage.entity_alive_count = data_component_storage.entity_count =
        in["entity_alive_count"].as<size_t>();
  data_component_storage.chunk_array.entity_array.resize(data_component_storage.entity_alive_count);
  const size_t chunk_size = data_component_storage.entity_count / data_component_storage.chunk_capacity + 1;
  while (data_component_storage.chunk_array.chunks.size() <= chunk_size) {
    // Allocate new chunk;
    data_component_storage.chunk_array.chunks.emplace_back();
  }
  auto in_data_component_types = in["data_component_types"];
  for (const auto& in_data_component_type : in_data_component_types) {
    DataComponentType data_component_type;
    if (in_data_component_type["type_name"])
      data_component_type.type_name = in_data_component_type["type_name"].as<std::string>();
    if (in_data_component_type["type_size"])
      data_component_type.type_size = in_data_component_type["type_size"].as<size_t>();
    if (in_data_component_type["type_offset"])
      data_component_type.type_offset = in_data_component_type["type_offset"].as<size_t>();
    if (Serialization::HasComponentDataType(data_component_type.type_name)) {
      data_component_type.type_index = Serialization::GetDataComponentTypeId(data_component_type.type_name);
    } else {
      data_component_type.type_index = std::hash<std::string>{}(data_component_type.type_name);
    }
    data_component_storage.data_component_types.push_back(data_component_type);
  }
  auto in_data_chunk_array = in["chunk_array"];
  int chunk_array_index = 0;
  for (const auto& entity_data_component : in_data_chunk_array) {
    Handle handle = entity_data_component["h"].as<uint64_t>();
    const Entity entity = scene_data_storage_.entity_map[handle];
    data_component_storage.chunk_array.entity_array[chunk_array_index] = entity;
    auto& metadata = scene_data_storage_.entity_metadata_list[entity.GetIndex()];
    metadata.data_component_storage_index = storage_index;
    metadata.chunk_array_index = chunk_array_index;
    const auto chunk_index = metadata.chunk_array_index / data_component_storage.chunk_capacity;
    const auto chunk_pointer = metadata.chunk_array_index % data_component_storage.chunk_capacity;
    auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];

    int type_index = 0;
    for (const auto& in_data_component : entity_data_component["dc"]) {
      auto& type = data_component_storage.data_component_types[type_index];
      auto data = in_data_component["d"].as<YAML::Binary>();
      std::memcpy(
          chunk.RefData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size),
          data.data(), data.size());
      type_index++;
    }
    chunk_array_index++;
  }
}

void WriteSceneSystem(const std::shared_ptr<ISystem>& system, YAML::Emitter& out) {
  out << YAML::BeginMap;
  {
    if (const auto unknown_system = std::dynamic_pointer_cast<UnknownSystem>(system)) {
      out << YAML::Key << "type_name" << YAML::Value << unknown_system->GetOriginalTypeName();
    } else {
      out << YAML::Key << "type_name" << YAML::Value << system->GetTypeName();
    }
    out << YAML::Key << "enabled_" << YAML::Value << system->Enabled();
    out << YAML::Key << "rank_" << YAML::Value << system->GetRank();
    out << YAML::Key << "handle_" << YAML::Value << system->GetHandle();
    Serialization::SerializeObject(out, *system);
  }
  out << YAML::EndMap;
}

size_t Scene::RestoreUnknownRuntimeTypes() {
  size_t restored_count = 0;
  const auto self = std::dynamic_pointer_cast<Scene>(GetSelf());

  for (auto& entity_metadata : scene_data_storage_.entity_metadata_list) {
    for (auto& element : entity_metadata.private_component_elements) {
      const auto unknown_component = std::dynamic_pointer_cast<UnknownPrivateComponent>(element.private_component_data);
      if (!unknown_component) {
        continue;
      }
      const auto& original_type_name = unknown_component->GetOriginalTypeName();
      if (original_type_name.empty() || !Serialization::HasSerializableType(original_type_name)) {
        continue;
      }

      size_t restored_type_id = 0;
      const auto restored_component = std::dynamic_pointer_cast<IPrivateComponent>(
          Serialization::ProduceSerializable(original_type_name, restored_type_id));
      if (!restored_component) {
        continue;
      }

      const auto owner = element.private_component_data->owner_;
      const auto old_type_id = element.type_index;
      restored_component->enabled_ = element.private_component_data->enabled_;
      restored_component->started_ = false;
      restored_component->owner_ = owner;
      restored_component->scene_ = self;
      restored_component->OnCreate();
      Serialization::DeserializeObject(unknown_component->GetSerializedNode(), *restored_component);

      scene_data_storage_.entity_private_component_storage.RemovePrivateComponent(owner, old_type_id,
                                                                                  element.private_component_data);
      scene_data_storage_.entity_private_component_storage.SetPrivateComponent(owner, restored_type_id);
      element.type_index = restored_type_id;
      element.private_component_data = restored_component;
      ++restored_count;
    }
  }

  for (auto system_iterator = systems_.begin(); system_iterator != systems_.end();) {
    const auto unknown_system = std::dynamic_pointer_cast<UnknownSystem>(system_iterator->second);
    if (!unknown_system) {
      ++system_iterator;
      continue;
    }
    const auto& original_type_name = unknown_system->GetOriginalTypeName();
    if (original_type_name.empty() || !Serialization::HasSerializableType(original_type_name)) {
      ++system_iterator;
      continue;
    }

    size_t restored_type_id = 0;
    const auto restored_system =
        std::dynamic_pointer_cast<ISystem>(Serialization::ProduceSerializable(original_type_name, restored_type_id));
    if (!restored_system) {
      ++system_iterator;
      continue;
    }

    const auto old_type_id = std::hash<std::string>{}(original_type_name);
    restored_system->handle_ = system_iterator->second->handle_;
    restored_system->enabled_ = system_iterator->second->enabled_;
    restored_system->rank_ = system_iterator->second->rank_;
    restored_system->started_ = false;
    restored_system->scene_ = self;
    restored_system->OnCreate();
    Serialization::DeserializeObject(unknown_system->GetSerializedNode(), *restored_system);

    indexed_systems_.erase(old_type_id);
    indexed_systems_[restored_type_id] = restored_system;
    mapped_systems_[restored_system->handle_] = restored_system;
    const auto rank = restored_system->rank_;
    system_iterator = systems_.erase(system_iterator);
    systems_.insert({rank, restored_system});
    ++restored_count;
  }

  for (auto& data_component_storage : scene_data_storage_.data_component_storage_list) {
    for (auto& type : data_component_storage.data_component_types) {
      if (type.type_name.empty() || !Serialization::HasComponentDataType(type.type_name)) {
        continue;
      }
      const auto registered_size =
          Serialization::GetDataComponentTypeSize(Serialization::GetDataComponentTypeId(type.type_name));
      if (type.type_size != registered_size) {
        EVOENGINE_WARNING("Cannot restore data component " + type.type_name +
                          " because the stored size does not match the registered type size.")
        continue;
      }
      const auto registered_type_id = Serialization::GetDataComponentTypeId(type.type_name);
      if (type.type_index == registered_type_id) {
        continue;
      }
      type.type_index = registered_type_id;
      ++restored_count;
    }
  }

  return restored_count;
}

void Scene::OnCreate() {
  scene_data_storage_.entities.emplace_back();
  scene_data_storage_.entity_metadata_list.emplace_back();
  scene_data_storage_.data_component_storage_list.emplace_back();
  scene_data_storage_.entity_private_component_storage.owner_scene = std::dynamic_pointer_cast<Scene>(GetSelf());
  if (!IsTemporary()) {
    std::error_code error_code;
    if (std::filesystem::exists(GetAbsolutePath(), error_code)) {
      return;
    }
  }

#pragma region Main Camera
  const auto main_camera_entity = CreateEntity("Main Camera");
  Transform ltw;
  ltw.SetPosition(glm::vec3(0.0f, 5.0f, 10.0f));
  ltw.SetScale(glm::vec3(1, 1, 1));
  ltw.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  SetDataComponent(main_camera_entity, ltw);
  const auto main_camera_component = GetOrSetPrivateComponent<Camera>(main_camera_entity).lock();
  main_camera = main_camera_component;
  main_camera_component->skybox = Resources::GetInstance().GetDefaultSkybox();
#pragma endregion

#pragma region Directional Light
  const auto directional_light_entity = CreateEntity("Directional Light");
  ltw.SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
  ltw.SetEulerRotation(glm::radians(glm::vec3(90, 0, 0)));
  SetDataComponent(directional_light_entity, ltw);
  auto direction_light = GetOrSetPrivateComponent<DirectionalLight>(directional_light_entity).lock();
#pragma endregion
  /*
#pragma region Ground
  const auto ground_entity = CreateEntity("Ground");
  ltw.SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
  ltw.SetScale(glm::vec3(10, 1, 10));
  ltw.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  SetDataComponent(ground_entity, ltw);
  const auto ground_mesh_renderer_component = GetOrSetPrivateComponent<MeshRenderer>(ground_entity).lock();
  ground_mesh_renderer_component->material = AssetManager::CreateTemporaryAsset<Material>();
  ground_mesh_renderer_component->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_QUAD");
#pragma endregion
  */
}

bool Scene::LoadInternal(const std::filesystem::path& path) {
  const auto previous_scene = ApplicationContext::Get().GetActiveScene();
  ApplicationContext::Get().Attach(std::shared_ptr<Scene>(this, [](Scene*) {
  }));
  std::ifstream stream(path.string());
  std::stringstream string_stream;
  string_stream << stream.rdbuf();
  YAML::Node in = YAML::Load(string_stream.str());
  DeserializeScene(in, *this);
  ApplicationContext::Get().Attach(previous_scene);
  return true;
}

bool Scene::SupportsStagedLoading(const std::filesystem::path& path) const {
  return path.extension() == ".evescene";
}

std::shared_ptr<StagedAssetLoadPayload> Scene::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  return Serialization::LoadAssetYamlPayload(path);
}

bool Scene::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                       const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto previous_scene = ApplicationContext::Get().GetActiveScene();
  ApplicationContext::Get().Attach(std::shared_ptr<Scene>(this, [](Scene*) {
  }));
  const bool loaded = Serialization::ApplyAssetYamlPayload(*this, payload);
  ApplicationContext::Get().Attach(previous_scene);
  return loaded;
}

bool Scene::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<Scene>(
      {},
      [](Scene& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const Scene& asset, const std::filesystem::path& path) {
        return asset.SupportsStagedLoading(path);
      },
      [](const Scene& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](Scene& asset, const std::filesystem::path& path, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

std::shared_ptr<Texture2D> Scene::GenerateThumbnailTexture() {
  return EditorLayer::FindIcon("Scene");
}

void Scene::Clone(const std::shared_ptr<Scene>& source, const std::shared_ptr<Scene>& new_scene) {
  new_scene->environment = source->environment;
  new_scene->saved_ = source->saved_;
  new_scene->world_bound_ = source->world_bound_;
  std::unordered_map<Handle, Handle> entity_map;

  new_scene->scene_data_storage_.Clone(entity_map, source->scene_data_storage_, new_scene);
  for (const auto& i : source->systems_) {
    auto system_name = i.second->GetTypeName();
    size_t hash_code;
    auto system = std::dynamic_pointer_cast<ISystem>(
        Serialization::ProduceSerializable(system_name, hash_code, i.second->GetHandle()));
    new_scene->systems_.insert({i.first, system});
    new_scene->indexed_systems_[hash_code] = system;
    new_scene->mapped_systems_[i.second->GetHandle()] = system;
    system->scene_ = new_scene;
    system->OnCreate();
    Serialization::CloneSystem(system, i.second);
    system->scene_ = new_scene;
  }
  new_scene->main_camera.entity_handle_ = source->main_camera.entity_handle_;
  new_scene->main_camera.private_component_type_name_ = source->main_camera.private_component_type_name_;
  new_scene->main_camera.Relink(entity_map, new_scene);
}

std::shared_ptr<LightProbe> Scene::Environment::GetLightProbe(const glm::vec3& position) {
  if (const auto em = environmental_map.Get<EnvironmentalMap>()) {
    if (auto light_probe = em->light_probe.Get<LightProbe>())
      return light_probe;
  }
  return nullptr;
}

std::shared_ptr<ReflectionProbe> Scene::Environment::GetReflectionProbe(const glm::vec3& position) {
  if (const auto em = environmental_map.Get<EnvironmentalMap>()) {
    if (auto reflection_probe = em->reflection_probe.Get<ReflectionProbe>())
      return reflection_probe;
  }
  return nullptr;
}

void Scene::Environment::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "background_color" << YAML::Value << background_color;
  out << YAML::Key << "environment_gamma" << YAML::Value << environment_gamma;
  out << YAML::Key << "ambient_light_intensity" << YAML::Value << ambient_light_intensity;
  out << YAML::Key << "environment_type" << YAML::Value << static_cast<unsigned>(environment_type);
  environmental_map.Save("environmental_map", out);
  out << YAML::Key << "volumetric_cloud_settings" << YAML::Value;
  SerializeVolumetricCloudSettings(out, volumetric_cloud_settings);
  out << YAML::Key << "ddgi_settings" << YAML::Value;
  SerializeDdgiSettings(out, ddgi_settings);
}
void Scene::Environment::Deserialize(const YAML::Node& in) {
  if (in["background_color"])
    background_color = in["background_color"].as<glm::vec3>();
  if (in["environment_gamma"])
    environment_gamma = in["environment_gamma"].as<float>();
  if (in["ambient_light_intensity"])
    ambient_light_intensity = in["ambient_light_intensity"].as<float>();
  if (in["environment_type"])
    environment_type = static_cast<EnvironmentType>(in["environment_type"].as<unsigned>());
  environmental_map.Load("environmental_map", in);
  if (in["volumetric_cloud_settings"])
    DeserializeVolumetricCloudSettings(in["volumetric_cloud_settings"], volumetric_cloud_settings);
  if (in["ddgi_settings"])
    DeserializeDdgiSettings(in["ddgi_settings"], ddgi_settings);
}
void SceneDataStorage::Clone(std::unordered_map<Handle, Handle>& entity_links, const SceneDataStorage& source,
                             const std::shared_ptr<Scene>& new_scene) {
  entities = source.entities;
  entity_metadata_list.resize(source.entity_metadata_list.size());

  for (const auto& i : source.entity_metadata_list) {
    entity_links.insert({i.entity_handle, i.entity_handle});
  }
  data_component_storage_list.resize(source.data_component_storage_list.size());
  for (size_t i = 0; i < data_component_storage_list.size(); i++)
    data_component_storage_list[i] = source.data_component_storage_list[i];
  for (size_t i = 0; i < entity_metadata_list.size(); i++)
    entity_metadata_list[i].Clone(entity_links, source.entity_metadata_list[i], new_scene);

  entity_map = source.entity_map;
  entity_private_component_storage = source.entity_private_component_storage;
  entity_private_component_storage.owner_scene = new_scene;
}

Input::KeyActionType Scene::GetKey(int key) {
  const auto search = pressed_keys_.find(key);
  if (search != pressed_keys_.end())
    return search->second;
  return Input::KeyActionType::Release;
}

#pragma region Entity Management
void Scene::UnsafeForEachDataComponent(const Entity& entity,
                                       const std::function<void(const DataComponentType& type, void* data)>& func) {
  assert(IsEntityValid(entity));
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const size_t chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const size_t chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  for (const auto& i : data_component_storage.data_component_types) {
    func(i, chunk.RefData(i.type_offset * data_component_storage.chunk_capacity + chunk_pointer * i.type_size));
  }
}

void Scene::ForEachPrivateComponent(const Entity& entity,
                                    const std::function<void(PrivateComponentElement& data)>& func) const {
  assert(IsEntityValid(entity));
  auto elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (auto& component : elements) {
    func(component);
  }
}

void Scene::UnsafeForEachEntityStorage(
    const std::function<void(size_t i, const std::string& name, const DataComponentStorage& storage)>& func) {
  const auto& archetype_infos = Entities::GetInstance().entity_archetype_infos_;
  for (size_t i = 0; i < archetype_infos.size(); i++) {
    auto dcs = GetDataComponentStorage(i);
    if (!dcs.has_value())
      continue;
    func(i, archetype_infos[i].archetype_name, dcs->first.get());
  }
}

void Scene::DeleteEntityInternal(const unsigned entity_index) {
  EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  Entity actual_entity = scene_data_storage_.entities.at(entity_index);

  scene_data_storage_.entity_private_component_storage.DeleteEntity(actual_entity);
  entity_info.entity_version = actual_entity.version_ + 1;
  entity_info.entity_enabled = true;
  entity_info.entity_static = false;
  entity_info.ancestor_selected = false;
  scene_data_storage_.entity_map.erase(entity_info.entity_handle);
  entity_info.entity_handle = Handle(0);

  entity_info.private_component_elements.clear();
  // Set to version 0, marks it as deleted.
  actual_entity.version_ = 0;
  data_component_storage.chunk_array.entity_array[entity_info.chunk_array_index] = actual_entity;
  const auto original_index = entity_info.chunk_array_index;
  if (entity_info.chunk_array_index != data_component_storage.entity_alive_count - 1) {
    const auto swapped_index = SwapEntity(data_component_storage, entity_info.chunk_array_index,
                                          data_component_storage.entity_alive_count - 1);
    entity_info.chunk_array_index = data_component_storage.entity_alive_count - 1;
    scene_data_storage_.entity_metadata_list.at(swapped_index).chunk_array_index = original_index;
  }
  data_component_storage.entity_alive_count--;

  scene_data_storage_.entities.at(entity_index) = actual_entity;
}

std::optional<std::pair<std::reference_wrapper<DataComponentStorage>, unsigned>> Scene::GetDataComponentStorage(
    const size_t entity_archetype_index) {
  auto& archetype_info = Entities::GetInstance().entity_archetype_infos_.at(entity_archetype_index);
  unsigned target_index = 0;
  for (auto& i : scene_data_storage_.data_component_storage_list) {
    if (i.data_component_types.size() != archetype_info.data_component_types.size()) {
      target_index++;
      continue;
    }
    bool check = true;
    for (size_t j = 0; j < i.data_component_types.size(); j++) {
      if (i.data_component_types[j].type_name != archetype_info.data_component_types[j].type_name) {
        check = false;
        break;
      }
    }
    if (check) {
      return {{std::ref(i), target_index}};
    }
    target_index++;
  }
  // If we didn't find the target storage, then we need to create a new one.
  scene_data_storage_.data_component_storage_list.emplace_back(archetype_info);
  return {{std::ref(scene_data_storage_.data_component_storage_list.back()),
           static_cast<unsigned>(scene_data_storage_.data_component_storage_list.size() - 1)}};
}

std::optional<std::pair<std::reference_wrapper<DataComponentStorage>, unsigned>> Scene::GetDataComponentStorage(
    const EntityArchetype& entity_archetype) {
  return GetDataComponentStorage(entity_archetype.index_);
}

std::vector<std::reference_wrapper<DataComponentStorage>> Scene::QueryDataComponentStorageList(
    const EntityQuery& entity_query) {
  return QueryDataComponentStorageList(entity_query.index_);
}

void Scene::GetEntityStorage(const DataComponentStorage& storage, std::vector<Entity>& container,
                             const bool check_enable) const {
  const size_t amount = storage.entity_alive_count;
  if (amount == 0)
    return;
  if (check_enable) {
    const auto worker_size = Jobs::GetWorkerSize();
    std::vector<std::vector<Entity>> temp_storage;
    temp_storage.resize(worker_size);
    const auto& chunk_array = storage.chunk_array;
    const auto& entities = &chunk_array.entity_array;
    Jobs::RunParallelFor(
        amount,
        [this, &entities, &temp_storage](const size_t i, const size_t worker_index) {
          const auto entity = entities->at(i);
          if (!scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
            return;
          temp_storage[worker_index].push_back(entity);
        },
        worker_size);
    for (auto& i : temp_storage) {
      container.insert(container.end(), i.begin(), i.end());
    }
  } else {
    container.resize(container.size() + amount);
    memcpy(&container.at(container.size() - amount), storage.chunk_array.entity_array.data(), amount * sizeof(Entity));
  }
}

auto Scene::SwapEntity(DataComponentStorage& storage, const size_t index1, const size_t index2) -> size_t {
  if (index1 == index2)
    return -1;
  const size_t ret_val = storage.chunk_array.entity_array[index2].index_;
  const auto other = storage.chunk_array.entity_array[index2];
  storage.chunk_array.entity_array[index2] = storage.chunk_array.entity_array[index1];
  storage.chunk_array.entity_array[index1] = other;
  const auto capacity = storage.chunk_capacity;
  const auto chunk_index1 = index1 / capacity;
  const auto chunk_index2 = index2 / capacity;
  const auto chunk_pointer1 = index1 % capacity;
  const auto chunk_pointer2 = index2 % capacity;
  for (const auto& i : storage.data_component_types) {
    void* temp = malloc(i.type_size);
    void* d1 =
        storage.chunk_array.chunks[chunk_index1].RefData(i.type_offset * capacity + i.type_size * chunk_pointer1);

    void* d2 =
        storage.chunk_array.chunks[chunk_index2].RefData(i.type_offset * capacity + i.type_size * chunk_pointer2);

    memcpy(temp, d1, i.type_size);
    memcpy(d1, d2, i.type_size);
    memcpy(d2, temp, i.type_size);
    free(temp);
  }
  return ret_val;
}

void Scene::GetAllEntities(std::vector<Entity>& target) {
  target.insert(target.end(), scene_data_storage_.entities.begin() + 1, scene_data_storage_.entities.end());
}

void Scene::ForEachDescendant(const Entity& target, const std::function<void(const Entity& entity)>& func,
                              const bool& from_root) {
  Entity real_target = target;
  if (!IsEntityValid(real_target))
    return;
  if (from_root)
    real_target = GetRoot(real_target);
  ForEachDescendantHelper(real_target, func);
}

const std::vector<Entity>& Scene::UnsafeGetAllEntities() {
  return scene_data_storage_.entities;
}

Entity Scene::CreateEntity(const std::string& name) {
  return CreateEntity(Entities::GetInstance().basic_archetype_, name);
}

Entity Scene::CreateEntity(const EntityArchetype& archetype, const std::string& name, const Handle& handle) {
  assert(archetype.IsValid());

  Entity ret_val;
  const auto search = GetDataComponentStorage(archetype);
  if (!search.has_value())
    throw std::runtime_error("Archetype not registered!");
  if (DataComponentStorage& storage = search->first; storage.entity_count == storage.entity_alive_count) {
    if (const size_t chunk_index = storage.entity_count / storage.chunk_capacity + 1;
        storage.chunk_array.chunks.size() <= chunk_index) {
      // Allocate new chunk;
      storage.chunk_array.chunks.emplace_back();
    }
    ret_val.index_ = static_cast<unsigned>(scene_data_storage_.entities.size());
    // If the version is 0 in chunk means it's deleted.
    ret_val.version_ = 1;
    EntityMetadata entity_info;
    entity_info.root = ret_val;
    entity_info.entity_static = false;
    entity_info.entity_name = name;
    entity_info.entity_handle = handle;
    entity_info.data_component_storage_index = search->second;
    entity_info.chunk_array_index = storage.entity_count;
    storage.chunk_array.entity_array.push_back(ret_val);

    scene_data_storage_.entity_map[entity_info.entity_handle] = ret_val;
    scene_data_storage_.entity_metadata_list.push_back(std::move(entity_info));
    scene_data_storage_.entities.push_back(ret_val);
    storage.entity_count++;
    storage.entity_alive_count++;
  } else {
    ret_val = storage.chunk_array.entity_array.at(storage.entity_alive_count);
    EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(ret_val.index_);
    entity_info.root = ret_val;
    entity_info.entity_static = false;
    entity_info.entity_handle = handle;
    entity_info.entity_enabled = true;
    entity_info.entity_name = name;
    ret_val.version_ = entity_info.entity_version;

    scene_data_storage_.entity_map[entity_info.entity_handle] = ret_val;
    storage.chunk_array.entity_array[entity_info.chunk_array_index] = ret_val;
    scene_data_storage_.entities.at(ret_val.index_) = ret_val;
    storage.entity_alive_count++;
    // Reset all component data
    const auto chunk_index = entity_info.chunk_array_index / storage.chunk_capacity;
    const auto chunk_pointer = entity_info.chunk_array_index % storage.chunk_capacity;
    auto& chunk = storage.chunk_array.chunks[chunk_index];
    for (const auto& i : storage.data_component_types) {
      const auto offset = i.type_offset * storage.chunk_capacity + chunk_pointer * i.type_size;
      chunk.ClearData(offset, i.type_size);
    }
  }
  SetDataComponent(ret_val, Transform());
  SetDataComponent(ret_val, GlobalTransform());
  SetDataComponent(ret_val, TransformUpdateFlag());
  SetUnsaved();
  return ret_val;
}

std::vector<Entity> Scene::CreateEntities(const EntityArchetype& archetype, const size_t& amount,
                                          const std::string& name) {
  assert(archetype.IsValid());
  std::vector<Entity> ret_val;
  const auto search = GetDataComponentStorage(archetype);
  if (!search.has_value())
    throw std::runtime_error("Archetype not registered!");
  DataComponentStorage& storage = search->first;
  auto remain_amount = amount;
  const Transform transform;
  const GlobalTransform global_transform;
  while (remain_amount > 0 && storage.entity_alive_count != storage.entity_count) {
    remain_amount--;
    Entity entity = storage.chunk_array.entity_array.at(storage.entity_alive_count);
    EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
    entity_info.root = entity;
    entity_info.entity_static = false;
    entity_info.entity_enabled = true;
    entity_info.entity_name = name;
    entity.version_ = entity_info.entity_version;
    entity_info.entity_handle = Handle();
    scene_data_storage_.entity_map[entity_info.entity_handle] = entity;
    storage.chunk_array.entity_array[entity_info.chunk_array_index] = entity;
    scene_data_storage_.entities.at(entity.index_) = entity;
    storage.entity_alive_count++;
    // Reset all component data
    const size_t chunk_index = entity_info.chunk_array_index / storage.chunk_capacity;
    const size_t chunk_pointer = entity_info.chunk_array_index % storage.chunk_capacity;
    ComponentDataChunk& chunk = storage.chunk_array.chunks[chunk_index];
    for (const auto& i : storage.data_component_types) {
      const size_t offset = i.type_offset * storage.chunk_capacity + chunk_pointer * i.type_size;
      chunk.ClearData(offset, i.type_size);
    }
    ret_val.push_back(entity);
    SetDataComponent(entity, transform);
    SetDataComponent(entity, global_transform);
    SetDataComponent(entity, TransformUpdateFlag());
  }
  if (remain_amount == 0)
    return ret_val;
  storage.entity_count += remain_amount;
  storage.entity_alive_count += remain_amount;
  const size_t chunk_index = storage.entity_count / storage.chunk_capacity + 1;
  while (storage.chunk_array.chunks.size() <= chunk_index) {
    // Allocate new chunk;
    storage.chunk_array.chunks.emplace_back();
  }
  const size_t original_size = scene_data_storage_.entities.size();
  scene_data_storage_.entities.resize(original_size + remain_amount);
  scene_data_storage_.entity_metadata_list.resize(original_size + remain_amount);

  for (size_t i = 0; i < remain_amount; i++) {
    auto& entity = scene_data_storage_.entities.at(original_size + i);
    entity.index_ = static_cast<unsigned>(original_size + i);
    entity.version_ = 1;

    auto& entity_info = scene_data_storage_.entity_metadata_list.at(original_size + i);
    entity_info = EntityMetadata();
    entity_info.root = entity;
    entity_info.entity_static = false;
    entity_info.entity_name = name;
    entity_info.data_component_storage_index = search->second;
    entity_info.chunk_array_index = storage.entity_alive_count - remain_amount + i;

    entity_info.entity_handle = Handle();

    scene_data_storage_.entity_map[entity_info.entity_handle] = entity;
  }

  storage.chunk_array.entity_array.insert(storage.chunk_array.entity_array.end(),
                                          scene_data_storage_.entities.begin() + original_size,
                                          scene_data_storage_.entities.end());
  Jobs::RunParallelFor(remain_amount, [&, original_size](const size_t i) {
    const auto& entity = scene_data_storage_.entities.at(original_size + i);
    SetDataComponent(entity, transform);
    SetDataComponent(entity, global_transform);
    SetDataComponent(entity, TransformUpdateFlag());
  });

  ret_val.insert(ret_val.end(), scene_data_storage_.entities.begin() + original_size,
                 scene_data_storage_.entities.end());
  SetUnsaved();
  return ret_val;
}

std::vector<Entity> Scene::CreateEntities(const size_t& amount, const std::string& name) {
  return CreateEntities(Entities::GetInstance().basic_archetype_, amount, name);
}

void Scene::DeleteEntity(const Entity& entity) {
  if (!IsEntityValid(entity)) {
    return;
  }
  const size_t entity_index = entity.index_;
  const auto children = scene_data_storage_.entity_metadata_list.at(entity_index).children;
  for (const auto& child : children) {
    DeleteEntity(child);
  }
  if (scene_data_storage_.entity_metadata_list.at(entity_index).parent.index_ != 0)
    RemoveChild(entity, scene_data_storage_.entity_metadata_list.at(entity_index).parent);
  DeleteEntityInternal(entity.index_);
  SetUnsaved();
}

std::string Scene::GetEntityName(const Entity& entity) {
  assert(IsEntityValid(entity));
  const size_t index = entity.index_;
  if (entity != scene_data_storage_.entities.at(index)) {
    EVOENGINE_ERROR("Child already deleted!")
    return "";
  }
  return scene_data_storage_.entity_metadata_list.at(index).entity_name;
}

void Scene::SetEntityName(const Entity& entity, const std::string& name) {
  assert(IsEntityValid(entity));
  const size_t index = entity.index_;
  if (entity != scene_data_storage_.entities.at(index)) {
    EVOENGINE_ERROR("Child already deleted!")
    return;
  }
  if (name.length() != 0) {
    scene_data_storage_.entity_metadata_list.at(index).entity_name = name;
    return;
  }
  scene_data_storage_.entity_metadata_list.at(index).entity_name = "Unnamed";
  SetUnsaved();
}
void Scene::SetEntityStatic(const Entity& entity, bool value) {
  assert(IsEntityValid(entity));
  auto& entity_info = scene_data_storage_.entity_metadata_list.at(GetRoot(entity).index_);
  entity_info.entity_static = value;
  SetUnsaved();
}
void Scene::SetParent(const Entity& child, const Entity& parent, const bool& recalculate_transform) {
  assert(IsEntityValid(child) && IsEntityValid(parent));
  const size_t child_index = child.index_;
  const size_t parent_index = parent.index_;
  auto& parent_entity_info = scene_data_storage_.entity_metadata_list.at(parent_index);
  for (const auto& i : parent_entity_info.children) {
    if (i == child)
      return;
  }
  auto& child_entity_info = scene_data_storage_.entity_metadata_list.at(child_index);
  if (child_entity_info.parent.GetIndex() != 0) {
    RemoveChild(child, child_entity_info.parent);
  }

  if (recalculate_transform) {
    const auto child_global_transform = GetDataComponent<GlobalTransform>(child);
    const auto parent_global_transform = GetDataComponent<GlobalTransform>(parent);
    Transform child_transform;
    child_transform.value = glm::inverse(parent_global_transform.value) * child_global_transform.value;
    SetDataComponent(child, child_transform);
  }
  child_entity_info.parent = parent;
  if (parent_entity_info.parent.GetIndex() == child_index) {
    parent_entity_info.parent = Entity();
    parent_entity_info.root = parent;
    const size_t children_count = child_entity_info.children.size();

    for (size_t i = 0; i < children_count; i++) {
      if (child_entity_info.children[i].index_ == parent.GetIndex()) {
        child_entity_info.children[i] = child_entity_info.children.back();
        child_entity_info.children.pop_back();
        break;
      }
    }
  }
  child_entity_info.root = parent_entity_info.root;
  child_entity_info.entity_static = false;
  parent_entity_info.children.push_back(child);
  if (parent_entity_info.ancestor_selected) {
    const auto descendants = GetDescendants(child);
    for (const auto& i : descendants) {
      GetEntityMetadata(i).ancestor_selected = true;
    }
    child_entity_info.ancestor_selected = true;
  }
  SetUnsaved();
}

Entity Scene::GetParent(const Entity& entity) const {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  return scene_data_storage_.entity_metadata_list.at(entity_index).parent;
}

std::vector<Entity> Scene::GetChildren(const Entity& entity) {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  return scene_data_storage_.entity_metadata_list.at(entity_index).children;
}

Entity Scene::GetChild(const Entity& entity, const size_t index) const {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  if (auto& children = scene_data_storage_.entity_metadata_list.at(entity_index).children; children.size() > index)
    return children[index];
  return {};
}

size_t Scene::GetChildrenAmount(const Entity& entity) const {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  return scene_data_storage_.entity_metadata_list.at(entity_index).children.size();
}

void Scene::ForEachChild(const Entity& entity, const std::function<void(Entity child)>& func) const {
  assert(IsEntityValid(entity));
  const auto children = scene_data_storage_.entity_metadata_list.at(entity.index_).children;
  for (auto i : children) {
    if (IsEntityValid(i))
      func(i);
  }
}

void Scene::RemoveChild(const Entity& child, const Entity& parent) {
  assert(IsEntityValid(child) && IsEntityValid(parent));
  const size_t child_index = child.index_;
  const size_t parent_index = parent.index_;
  auto& child_entity_metadata = scene_data_storage_.entity_metadata_list.at(child_index);
  auto& parent_entity_metadata = scene_data_storage_.entity_metadata_list.at(parent_index);
  if (child_entity_metadata.parent.index_ == 0) {
    EVOENGINE_ERROR("No child by the parent!")
  }
  child_entity_metadata.parent = Entity();
  child_entity_metadata.root = child;
  if (parent_entity_metadata.ancestor_selected) {
    const auto descendants = GetDescendants(child);
    for (const auto& i : descendants) {
      GetEntityMetadata(i).ancestor_selected = false;
    }
    child_entity_metadata.ancestor_selected = false;
  }
  const size_t children_count = parent_entity_metadata.children.size();

  for (size_t i = 0; i < children_count; i++) {
    if (parent_entity_metadata.children[i].index_ == child_index) {
      parent_entity_metadata.children[i] = parent_entity_metadata.children.back();
      parent_entity_metadata.children.pop_back();
      break;
    }
  }
  const auto child_global_transform = GetDataComponent<GlobalTransform>(child);
  Transform child_transform;
  child_transform.value = child_global_transform.value;
  SetDataComponent(child, child_transform);
  SetUnsaved();
}

void Scene::RemoveDataComponent(const Entity& entity, const size_t& type_index) {
  assert(IsEntityValid(entity));
  if (type_index == typeid(Transform).hash_code() || type_index == typeid(GlobalTransform).hash_code() ||
      type_index == typeid(TransformUpdateFlag).hash_code()) {
    return;
  }
  EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  if (data_component_storage.data_component_types.size() <= 3) {
    EVOENGINE_ERROR(
        "Remove Component Data failed: Entity must have at least 1 data component besides 3 basic data "
        "components!")
    return;
  }
#pragma region Create new archetype
  EntityArchetypeInfo new_archetype_info;
  new_archetype_info.archetype_name = "New archetype";
  new_archetype_info.data_component_types = data_component_storage.data_component_types;
  bool found = false;
  for (size_t i = 0; i < new_archetype_info.data_component_types.size(); i++) {
    if (new_archetype_info.data_component_types[i].type_index == type_index) {
      new_archetype_info.data_component_types.erase(new_archetype_info.data_component_types.begin() + i);
      found = true;
      break;
    }
  }
  if (!found) {
    EVOENGINE_ERROR("Failed to remove component data: Component not found")
    return;
  }
  size_t offset = 0;
  DataComponentType prev = new_archetype_info.data_component_types[0];
  for (auto& i : new_archetype_info.data_component_types) {
    i.type_offset = offset;
    offset += i.type_size;
  }
  new_archetype_info.entity_size = new_archetype_info.data_component_types.back().type_offset +
                                   new_archetype_info.data_component_types.back().type_size;
  new_archetype_info.chunk_capacity = Entities::GetArchetypeChunkSize() / new_archetype_info.entity_size;
  auto archetype = Entities::CreateEntityArchetypeHelper(new_archetype_info);
#pragma endregion
#pragma region Create new Entity with new archetype
  const Entity new_entity = CreateEntity(archetype);
  // Transfer component data
  for (const auto& type : new_archetype_info.data_component_types) {
    SetDataComponent(new_entity.index_, type.type_index, type.type_size,
                     GetDataComponentPointer(entity, type.type_index));
  }
  // 5. Swap entity.
  EntityMetadata& new_entity_info = scene_data_storage_.entity_metadata_list.at(new_entity.index_);
  const auto temp_archetype_info_index = new_entity_info.data_component_storage_index;
  const auto temp_chunk_array_index = new_entity_info.chunk_array_index;
  new_entity_info.data_component_storage_index = entity_info.data_component_storage_index;
  new_entity_info.chunk_array_index = entity_info.chunk_array_index;
  entity_info.data_component_storage_index = temp_archetype_info_index;
  entity_info.chunk_array_index = temp_chunk_array_index;
  // Apply to chunk.
  scene_data_storage_.data_component_storage_list.at(entity_info.data_component_storage_index)
      .chunk_array.entity_array[entity_info.chunk_array_index] = entity;
  scene_data_storage_.data_component_storage_list.at(new_entity_info.data_component_storage_index)
      .chunk_array.entity_array[new_entity_info.chunk_array_index] = new_entity;
  DeleteEntity(new_entity);
#pragma endregion
  SetUnsaved();
}

bool Scene::HasDataComponent(const Entity& entity, const size_t type_id) const {
  return HasDataComponent(entity.GetIndex(), type_id);
}

bool Scene::HasDataComponent(const size_t& index, const size_t type_id) const {
  if (index > scene_data_storage_.entity_metadata_list.size())
    return false;
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];

  if (type_id == typeid(Transform).hash_code()) {
    return true;
  }
  if (type_id == typeid(GlobalTransform).hash_code()) {
    return true;
  }
  if (type_id == typeid(TransformUpdateFlag).hash_code()) {
    return true;
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == type_id) {
      return true;
    }
  }
  return false;
}

void Scene::AddDataComponent(const Entity& entity, const size_t type_id) {
  assert(IsEntityValid(entity));
  auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);

#pragma region Check if componentdata already exists.If yes, go to SetComponentData
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list.at(entity_info.data_component_storage_index);
  const auto original_component_types = data_component_storage.data_component_types;
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == type_id) {
      EVOENGINE_ERROR("Data Component already exists!")
      return;
    }
  }
#pragma endregion
#pragma region If not exist, we first need to create a new archetype
  EntityArchetypeInfo new_archetype_info;
  new_archetype_info.archetype_name = "New archetype";
  new_archetype_info.data_component_types = original_component_types;
  DataComponentType data_component_type{};
  data_component_type.type_index = type_id;
  data_component_type.type_name = Serialization::GetSerializableTypeName(type_id);
  data_component_type.type_size = Serialization::GetDataComponentTypeSize(type_id);
  new_archetype_info.data_component_types.emplace_back(data_component_type);
  std::sort(new_archetype_info.data_component_types.begin() + 3, new_archetype_info.data_component_types.end(),
            ComponentTypeComparator);
  size_t offset = 0;
  DataComponentType prev = new_archetype_info.data_component_types[0];
  // Erase duplicates

  std::vector<DataComponentType> copy;
  copy.insert(copy.begin(), new_archetype_info.data_component_types.begin(),
              new_archetype_info.data_component_types.end());
  new_archetype_info.data_component_types.clear();
  for (const auto& i : copy) {
    bool found = false;
    for (const auto& j : new_archetype_info.data_component_types) {
      if (i == j) {
        found = true;
        break;
      }
    }
    if (found)
      continue;
    new_archetype_info.data_component_types.push_back(i);
  }

  for (auto& i : new_archetype_info.data_component_types) {
    i.type_offset = offset;
    offset += i.type_size;
  }
  new_archetype_info.entity_size = new_archetype_info.data_component_types.back().type_offset +
                                   new_archetype_info.data_component_types.back().type_size;
  new_archetype_info.chunk_capacity = Entities::GetArchetypeChunkSize() / new_archetype_info.entity_size;
  const auto archetype = Entities::CreateEntityArchetypeHelper(new_archetype_info);
#pragma endregion
#pragma region Create new Entity with new archetype.
  Entity new_entity = CreateEntity(archetype);
  auto& original_entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  // Transfer component data
  for (const auto& type : original_component_types) {
    SetDataComponent(new_entity.index_, type.type_index, type.type_size,
                     GetDataComponentPointer(entity.index_, type.type_index));
  }
  // 5. Swap entity.
  auto& new_entity_info = scene_data_storage_.entity_metadata_list.at(new_entity.index_);
  const auto temp_archetype_info_index = new_entity_info.data_component_storage_index;
  const auto temp_chunk_array_index = new_entity_info.chunk_array_index;
  new_entity_info.data_component_storage_index = original_entity_info.data_component_storage_index;
  new_entity_info.chunk_array_index = original_entity_info.chunk_array_index;
  original_entity_info.data_component_storage_index = temp_archetype_info_index;
  original_entity_info.chunk_array_index = temp_chunk_array_index;
  // Apply to chunk.
  scene_data_storage_.data_component_storage_list.at(original_entity_info.data_component_storage_index)
      .chunk_array.entity_array[original_entity_info.chunk_array_index] = entity;
  scene_data_storage_.data_component_storage_list.at(new_entity_info.data_component_storage_index)
      .chunk_array.entity_array[new_entity_info.chunk_array_index] = new_entity;
  DeleteEntity(new_entity);
#pragma endregion
  SetUnsaved();
}

void Scene::SetDataComponent(const unsigned& entity_index, const size_t id, const size_t size, const void* data) {
  const auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  if (id == typeid(Transform).hash_code()) {
    chunk.SetData(chunk_pointer * sizeof(Transform), sizeof(Transform), data);
    static_cast<TransformUpdateFlag*>(
        chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                      chunk_pointer * sizeof(TransformUpdateFlag)))
        ->transform_modified = true;
  } else if (id == typeid(GlobalTransform).hash_code()) {
    chunk.SetData(sizeof(Transform) * data_component_storage.chunk_capacity + chunk_pointer * sizeof(GlobalTransform),
                  sizeof(GlobalTransform), data);
    static_cast<TransformUpdateFlag*>(
        chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                      chunk_pointer * sizeof(TransformUpdateFlag)))
        ->global_transform_modified = true;
  } else if (id == typeid(TransformUpdateFlag).hash_code()) {
    chunk.SetData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                      chunk_pointer * sizeof(TransformUpdateFlag),
                  sizeof(TransformUpdateFlag), data);
  } else {
    for (const auto& type : data_component_storage.data_component_types) {
      if (type.type_index == id) {
        chunk.SetData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size, size,
                      data);
        return;
      }
    }
    EVOENGINE_LOG("ComponentData doesn't exist")
  }
  SetUnsaved();
}
void* Scene::GetDataComponentPointer(unsigned entity_index, const size_t& id) {
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  if (id == typeid(Transform).hash_code()) {
    return chunk.RefData(chunk_pointer * sizeof(Transform));
  }
  if (id == typeid(GlobalTransform).hash_code()) {
    return chunk.RefData(sizeof(Transform) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(GlobalTransform));
  }
  if (id == typeid(TransformUpdateFlag).hash_code()) {
    return chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(TransformUpdateFlag));
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      return chunk.RefData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size);
    }
  }
  EVOENGINE_LOG("ComponentData doesn't exist")
  return nullptr;
}
void* Scene::GetDataComponentPointer(const Entity& entity, const size_t& id) {
  assert(IsEntityValid(entity));
  EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  if (id == typeid(Transform).hash_code()) {
    return chunk.RefData(chunk_pointer * sizeof(Transform));
  }
  if (id == typeid(GlobalTransform).hash_code()) {
    return chunk.RefData(sizeof(Transform) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(GlobalTransform));
  }
  if (id == typeid(TransformUpdateFlag).hash_code()) {
    return chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(TransformUpdateFlag));
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      return chunk.RefData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size);
    }
  }
  EVOENGINE_LOG("ComponentData doesn't exist")
  return nullptr;
}
Handle Scene::GetEntityHandle(const Entity& entity) {
  return scene_data_storage_.entity_metadata_list.at(entity.index_).entity_handle;
}
void Scene::SetPrivateComponent(const Entity& entity, const std::shared_ptr<IPrivateComponent>& ptr) {
  assert(ptr && IsEntityValid(entity));
  const auto type_name = ptr->GetTypeName();
  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (const auto& element : elements) {
    if (type_name == element.private_component_data->GetTypeName()) {
      return;
    }
  }
  const auto id = Serialization::GetSerializableTypeId(type_name);
  scene_data_storage_.entity_private_component_storage.SetPrivateComponent(entity, id);
  PrivateComponentElement private_component_element(id, ptr, entity, std::dynamic_pointer_cast<Scene>(GetSelf()));
  elements.emplace_back(private_component_element);
  SetUnsaved();
}

void Scene::ForEachDescendantHelper(const Entity& target, const std::function<void(const Entity& entity)>& func) {
  func(target);
  ForEachChild(target, [&](Entity child) {
    ForEachDescendantHelper(child, func);
  });
}

Entity Scene::GetRoot(const Entity& entity) const {
  Entity ret_val = entity;
  auto parent = GetParent(ret_val);
  while (parent.GetIndex() != 0) {
    ret_val = parent;
    parent = GetParent(ret_val);
  }
  return ret_val;
}

Entity Scene::GetEntity(const size_t& index) const {
  if (index > 0 && index < scene_data_storage_.entities.size())
    return scene_data_storage_.entities.at(index);
  return {};
}

void Scene::RemovePrivateComponent(const Entity& entity, size_t type_id) {
  assert(IsEntityValid(entity));
  auto& private_component_elements =
      scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (size_t i = 0; i < private_component_elements.size(); i++) {
    if (private_component_elements[i].type_index == type_id) {
      scene_data_storage_.entity_private_component_storage.RemovePrivateComponent(
          entity, type_id, private_component_elements[i].private_component_data);
      private_component_elements.erase(private_component_elements.begin() + i);
      SetUnsaved();
      break;
    }
  }
}

void Scene::SetEnable(const Entity& entity, const bool& value) {
  assert(IsEntityValid(entity));
  if (scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled != value) {
    for (const auto& i : scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements) {
      if (value) {
        i.private_component_data->OnEntityEnable();
      } else {
        i.private_component_data->OnEntityDisable();
      }
    }
  }
  scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled = value;

  for (const auto& i : scene_data_storage_.entity_metadata_list.at(entity.index_).children) {
    SetEnable(i, value);
  }
  SetUnsaved();
}

void Scene::SetEnableSingle(const Entity& entity, const bool& value) {
  assert(IsEntityValid(entity));
  if (auto& entity_metadata = scene_data_storage_.entity_metadata_list.at(entity.index_);
      entity_metadata.entity_enabled != value) {
    for (const auto& i : entity_metadata.private_component_elements) {
      if (value) {
        i.private_component_data->OnEntityEnable();
      } else {
        i.private_component_data->OnEntityDisable();
      }
    }
    entity_metadata.entity_enabled = value;
  }
}
EntityMetadata& Scene::GetEntityMetadata(const Entity& entity) {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_);
}

bool Scene::HasSystem(const size_t& type_id) {
  if (const auto search = indexed_systems_.find(type_id); search != indexed_systems_.end())
    return true;
  return false;
}

std::shared_ptr<ISystem> Scene::CreateSystem(const size_t& type_id, float order) {
  const auto ptr = Serialization::ProduceSerializable(Serialization::GetSerializableTypeName(type_id));
  auto system = std::dynamic_pointer_cast<ISystem>(ptr);
  system->scene_ = std::dynamic_pointer_cast<Scene>(GetSelf());
  system->handle_ = Handle();
  system->rank_ = order;
  systems_.insert({order, system});
  indexed_systems_[type_id] = system;
  mapped_systems_[system->handle_] = system;
  system->started_ = false;
  system->OnCreate();
  SetUnsaved();
  return system;
}

void Scene::AddPrivateComponent(const Entity& entity, const size_t& type_id) {
  assert(IsEntityValid(entity));
  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  auto ptr = scene_data_storage_.entity_private_component_storage.GetOrSetPrivateComponent(entity, type_id);
  elements.emplace_back(type_id, ptr, entity, std::dynamic_pointer_cast<Scene>(GetSelf()));
  SetUnsaved();
}

void Scene::ForAllEntities(const std::function<void(size_t i, Entity entity)>& func) const {
  for (size_t index = 0; index < scene_data_storage_.entities.size(); index++) {
    if (scene_data_storage_.entities.at(index).version_ != 0) {
      func(index, scene_data_storage_.entities.at(index));
    }
  }
}

Bound Scene::GetEntityBoundingBox(const Entity& entity) {
  auto descendants = GetDescendants(entity);
  descendants.emplace_back(entity);
  Bound ret_val{};
  for (const auto& walker : descendants) {
    auto gt = GetDataComponent<GlobalTransform>(walker);
    if (HasPrivateComponent<MeshRenderer>(walker)) {
      auto mesh_renderer = GetOrSetPrivateComponent<MeshRenderer>(walker).lock();
      if (const auto mesh = mesh_renderer->mesh.Get<Mesh>()) {
        auto mesh_bound = mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        ret_val.min =
            glm::vec3((glm::min)(ret_val.min.x, center.x - size.x), (glm::min)(ret_val.min.y, center.y - size.y),
                      (glm::min)(ret_val.min.z, center.z - size.z));
        ret_val.max =
            glm::vec3((glm::max)(ret_val.max.x, center.x + size.x), (glm::max)(ret_val.max.y, center.y + size.y),
                      (glm::max)(ret_val.max.z, center.z + size.z));
      }
    } else if (HasPrivateComponent<SkinnedMeshRenderer>(walker)) {
      auto mesh_renderer = GetOrSetPrivateComponent<SkinnedMeshRenderer>(walker).lock();
      if (const auto mesh = mesh_renderer->skinned_mesh.Get<SkinnedMesh>()) {
        auto mesh_bound = mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        ret_val.min =
            glm::vec3((glm::min)(ret_val.min.x, center.x - size.x), (glm::min)(ret_val.min.y, center.y - size.y),
                      (glm::min)(ret_val.min.z, center.z - size.z));
        ret_val.max =
            glm::vec3((glm::max)(ret_val.max.x, center.x + size.x), (glm::max)(ret_val.max.y, center.y + size.y),
                      (glm::max)(ret_val.max.z, center.z + size.z));
      }
    }
  }

  return ret_val;
}

void Scene::GetEntityArray(const EntityQuery& entity_query, std::vector<Entity>& container, const bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  for (const auto i : queried_storage_list) {
    GetEntityStorage(i.get(), container, check_enable);
  }
}

size_t Scene::GetEntityAmount(const EntityQuery entity_query, const bool check_enable) {
  assert(entity_query.IsValid());
  size_t ret_val = 0;
  if (check_enable) {
    const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
    for (const auto i : queried_storage_list) {
      for (size_t index = 0; index < i.get().entity_alive_count; index++) {
        if (IsEntityEnabled(i.get().chunk_array.entity_array[index]))
          ret_val++;
      }
    }
  } else {
    const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
    for (const auto i : queried_storage_list) {
      ret_val += i.get().entity_alive_count;
    }
  }
  return ret_val;
}

std::vector<Entity> Scene::GetDescendants(const Entity& entity) {
  std::vector<Entity> ret_val;
  if (!IsEntityValid(entity))
    return ret_val;
  GetDescendantsHelper(entity, ret_val);
  return ret_val;
}
void Scene::GetDescendantsHelper(const Entity& target, std::vector<Entity>& results) {
  auto& children = scene_data_storage_.entity_metadata_list.at(target.index_).children;
  if (!children.empty())
    results.insert(results.end(), children.begin(), children.end());
  for (const auto& i : children)
    GetDescendantsHelper(i, results);
}

std::weak_ptr<IPrivateComponent> Scene::GetPrivateComponent(const Entity& entity, const std::string& type_name) {
  size_t i = 0;
  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (auto& element : elements) {
    if (type_name == element.private_component_data->type_name_) {
      return element.private_component_data;
    }
    i++;
  }
  throw std::runtime_error("Private component doesn't exist!");
}

Entity Scene::GetEntity(const Handle& handle) {
  if (const auto search = scene_data_storage_.entity_map.find(handle); search != scene_data_storage_.entity_map.end()) {
    return search->second;
  }
  return {};
}
bool Scene::HasPrivateComponent(const Entity& entity, const std::string& type_name) const {
  assert(IsEntityValid(entity));
  for (auto& element : scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements) {
    if (element.private_component_data->type_name_ == type_name) {
      return true;
    }
  }
  return false;
}

bool Scene::HasPrivateComponent(const Entity& entity, const size_t& type_id) const {
  assert(IsEntityValid(entity));
  for (auto& element : scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements) {
    if (Serialization::GetSerializableTypeId(element.private_component_data->type_name_) == type_id) {
      return true;
    }
  }
  return false;
}

bool Scene::HasPrivateComponentOwners(const size_t& type_id) const {
  return scene_data_storage_.entity_private_component_storage.HasPrivateComponentOwners(type_id);
}

size_t Scene::ClearPrivateComponentPool(const size_t& type_id) {
  return scene_data_storage_.entity_private_component_storage.ClearPrivateComponentPool(type_id);
}

std::vector<std::reference_wrapper<DataComponentStorage>> Scene::QueryDataComponentStorageList(
    const size_t entity_query_index) {
  const auto& query_infos = Entities::GetInstance().entity_query_infos_.at(entity_query_index);
  auto& entity_component_storage = scene_data_storage_.data_component_storage_list;
  std::vector<std::reference_wrapper<DataComponentStorage>> queried_storage;
  // Select storage with every contained.
  if (!query_infos.all_data_component_types.empty()) {
    for (auto& data_storage : entity_component_storage) {
      bool check = true;
      for (const auto& type : query_infos.all_data_component_types) {
        if (!data_storage.HasType(type.type_index))
          check = false;
      }
      if (check)
        queried_storage.push_back(std::ref(data_storage));
    }
  } else {
    for (auto& data_storage : entity_component_storage) {
      queried_storage.push_back(std::ref(data_storage));
    }
  }
  // Erase with any
  if (!query_infos.any_data_component_types.empty()) {
    for (size_t i = 0; i < queried_storage.size(); i++) {
      bool contain = false;
      for (const auto& type : query_infos.any_data_component_types) {
        if (queried_storage.at(i).get().HasType(type.type_index))
          contain = true;
        if (contain)
          break;
      }
      if (!contain) {
        queried_storage.erase(queried_storage.begin() + i);
        i--;
      }
    }
  }
  // Erase with none
  if (!query_infos.none_data_component_types.empty()) {
    for (size_t i = 0; i < queried_storage.size(); i++) {
      bool contain = false;
      for (const auto& type : query_infos.none_data_component_types) {
        if (queried_storage.at(i).get().HasType(type.type_index))
          contain = true;
        if (contain)
          break;
      }
      if (contain) {
        queried_storage.erase(queried_storage.begin() + i);
        i--;
      }
    }
  }
  return queried_storage;
}
bool Scene::IsEntityValid(const Entity& entity) const {
  auto& storage = scene_data_storage_.entities;
  return entity.index_ != 0 && entity.version_ != 0 && entity.index_ < storage.size() &&
         storage.at(entity.index_).version_ == entity.version_;
}
bool Scene::IsEntityEnabled(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled;
}
bool Scene::IsEntityRoot(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_).root == entity;
}
bool Scene::IsEntityStatic(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(GetRoot(entity).index_).entity_static;
}

bool Scene::IsEntityAncestorSelected(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_).ancestor_selected;
}

#pragma endregion
