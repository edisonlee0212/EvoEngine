#include "EnvironmentalLighting.hpp"

#include "Camera.hpp"
#include "Serialization.hpp"

#include <cmath>
#include <unordered_set>

using namespace evo_engine;

namespace {
glm::ivec3 ClampDdgiProbeCounts(const glm::ivec3& value) {
  return {glm::clamp(value.x, 1, 256), glm::clamp(value.y, 1, 256), glm::clamp(value.z, 1, 256)};
}

glm::vec3 ClampDdgiProbeSpacing(const glm::vec3& value) {
  return glm::clamp(value, glm::vec3(0.05f), glm::vec3(10000.0f));
}

template <typename Entry>
bool RepairEntryStableIds(std::vector<Entry>& entries) {
  std::unordered_set<uint64_t> used_ids;
  std::vector<size_t> invalid_indices;
  for (size_t index = 0; index < entries.size(); ++index) {
    if (entries[index].stable_id == 0u || !used_ids.emplace(entries[index].stable_id).second) {
      invalid_indices.emplace_back(index);
    }
  }

  uint64_t candidate = 1u;
  for (const auto index : invalid_indices) {
    while (used_ids.find(candidate) != used_ids.end()) {
      ++candidate;
    }
    entries[index].stable_id = candidate;
    used_ids.emplace(candidate++);
  }
  return !invalid_indices.empty();
}

void SerializeIndirectEnvironmentSource(YAML::Emitter& out,
                                        const EnvironmentalLighting::IndirectEnvironmentSource& source) {
  out << YAML::BeginMap;
  out << YAML::Key << "kind" << YAML::Value << static_cast<uint32_t>(source.kind);
  source.environmental_map.Save("environmental_map", out);
  out << YAML::Key << "color" << YAML::Value << source.color;
  out << YAML::Key << "gamma" << YAML::Value << source.gamma;
  out << YAML::Key << "rotation" << YAML::Value << source.rotation;
  out << YAML::EndMap;
}

void DeserializeIndirectEnvironmentSource(const YAML::Node& in,
                                          EnvironmentalLighting::IndirectEnvironmentSource& source) {
  source = {};
  if (in["kind"]) {
    const auto kind = in["kind"].as<uint32_t>();
    source.kind = kind <= static_cast<uint32_t>(EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap)
                      ? static_cast<EnvironmentalLighting::IndirectEnvironmentSourceKind>(kind)
                      : EnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault;
  }
  source.environmental_map.Load("environmental_map", in);
  if (in["color"])
    source.color = in["color"].as<glm::vec3>();
  if (in["gamma"])
    source.gamma = in["gamma"].as<float>();
  if (in["rotation"])
    source.rotation = in["rotation"].as<float>();
}

void SerializeReflectionProbeBakeBackground(YAML::Emitter& out,
                                            const EnvironmentalLighting::ReflectionProbeBakeBackground& background) {
  out << YAML::BeginMap;
  out << YAML::Key << "source" << YAML::Value << Camera::GetBackgroundSourceName(background.source);
  out << YAML::Key << "intensity" << YAML::Value << background.intensity;
  out << YAML::Key << "clear_color" << YAML::Value << background.clear_color;
  background.cubemap.Save("cubemap", out);
  background.environmental_map.Save("environmental_map", out);
  out << YAML::EndMap;
}

void DeserializeReflectionProbeBakeBackground(const YAML::Node& in,
                                              EnvironmentalLighting::ReflectionProbeBakeBackground& background) {
  background = {};
  if (in["source"]) {
    background.source = Camera::ParseBackgroundSource(in["source"].as<std::string>(),
                                                      CameraSettings::BackgroundSource::InheritEnvironmentalLighting);
  }
  if (in["intensity"])
    background.intensity = in["intensity"].as<float>();
  if (in["clear_color"])
    background.clear_color = in["clear_color"].as<glm::vec4>();
  background.cubemap.Load("cubemap", in);
  background.environmental_map.Load("environmental_map", in);
}

void SerializeDynamicReflectionProbeSettings(YAML::Emitter& out,
                                             const EnvironmentalLighting::DynamicReflectionProbeSettings& settings) {
  out << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << settings.enabled;
  out << YAML::Key << "faces_per_frame" << YAML::Value << settings.faces_per_frame;
  out << YAML::EndMap;
}

void DeserializeDynamicReflectionProbeSettings(const YAML::Node& in,
                                               EnvironmentalLighting::DynamicReflectionProbeSettings& settings) {
  settings = {};
  if (in["enabled"])
    settings.enabled = in["enabled"].as<bool>();
  if (in["faces_per_frame"])
    settings.faces_per_frame = in["faces_per_frame"].as<int>();
  settings.Clamp();
}

void SerializeLocalReflectionProbe(YAML::Emitter& out, const EnvironmentalLighting::LocalReflectionProbe& probe) {
  out << YAML::BeginMap;
  out << YAML::Key << "name" << YAML::Value << probe.name;
  out << YAML::Key << "stable_id" << YAML::Value << probe.stable_id;
  probe.global_reflection_probe.Save("global_reflection_probe", out);
  out << YAML::Key << "transform" << YAML::Value << probe.transform;
  out << YAML::Key << "box_projection_extents" << YAML::Value << probe.box_projection_extents;
  out << YAML::Key << "sphere_radius" << YAML::Value << probe.sphere_radius;
  out << YAML::Key << "blend_distance" << YAML::Value << probe.blend_distance;
  out << YAML::Key << "reflection_intensity" << YAML::Value << probe.reflection_intensity;
  out << YAML::Key << "artist_priority" << YAML::Value << probe.artist_priority;
  out << YAML::Key << "shape" << YAML::Value << probe.shape;
  out << YAML::Key << "box_projection" << YAML::Value << probe.box_projection;
  out << YAML::Key << "enabled" << YAML::Value << probe.enabled;
  out << YAML::Key << "debug_draw_bounds" << YAML::Value << probe.debug_draw_bounds;
  out << YAML::EndMap;
}

void DeserializeLocalReflectionProbe(const YAML::Node& in, EnvironmentalLighting::LocalReflectionProbe& probe) {
  probe = {};
  if (in["name"])
    probe.name = in["name"].as<std::string>();
  if (in["stable_id"])
    probe.stable_id = in["stable_id"].as<uint64_t>();
  probe.global_reflection_probe.Load("global_reflection_probe", in);
  if (in["transform"])
    probe.transform = in["transform"].as<glm::mat4>();
  if (in["box_projection_extents"])
    probe.box_projection_extents = in["box_projection_extents"].as<glm::vec3>();
  if (in["sphere_radius"])
    probe.sphere_radius = in["sphere_radius"].as<float>();
  if (in["blend_distance"])
    probe.blend_distance = in["blend_distance"].as<float>();
  if (in["reflection_intensity"])
    probe.reflection_intensity = in["reflection_intensity"].as<float>();
  if (in["artist_priority"])
    probe.artist_priority = in["artist_priority"].as<int>();
  if (in["shape"])
    probe.shape = in["shape"].as<int>();
  if (in["box_projection"])
    probe.box_projection = in["box_projection"].as<bool>();
  if (in["enabled"])
    probe.enabled = in["enabled"].as<bool>();
  if (in["debug_draw_bounds"])
    probe.debug_draw_bounds = in["debug_draw_bounds"].as<bool>();
}

void SerializeDdgiVolume(YAML::Emitter& out, const EnvironmentalLighting::DdgiVolume& volume) {
  out << YAML::BeginMap;
  out << YAML::Key << "name" << YAML::Value << volume.name;
  out << YAML::Key << "stable_id" << YAML::Value << volume.stable_id;
  out << YAML::Key << "transform" << YAML::Value << volume.transform;
  out << YAML::Key << "probe_counts" << YAML::Value << volume.probe_counts;
  out << YAML::Key << "probe_spacing" << YAML::Value << volume.probe_spacing;
  out << YAML::Key << "volume_origin" << YAML::Value << volume.volume_origin;
  out << YAML::Key << "artist_priority" << YAML::Value << volume.artist_priority;
  out << YAML::Key << "movement_type" << YAML::Value << volume.movement_type;
  out << YAML::Key << "emissive_mesh_sampling_mode" << YAML::Value << volume.emissive_mesh_sampling_mode;
  out << YAML::Key << "enabled" << YAML::Value << volume.enabled;
  out << YAML::Key << "enable_probe_relocation" << YAML::Value << volume.enable_probe_relocation;
  out << YAML::Key << "enable_probe_classification" << YAML::Value << volume.enable_probe_classification;
  out << YAML::Key << "enable_probe_variability" << YAML::Value << volume.enable_probe_variability;
  out << YAML::Key << "enable_probe_variability_gating" << YAML::Value << volume.enable_probe_variability_gating;
  out << YAML::Key << "relocation_distance" << YAML::Value << volume.relocation_distance;
  out << YAML::Key << "random_ray_backface_threshold" << YAML::Value << volume.random_ray_backface_threshold;
  out << YAML::Key << "fixed_ray_backface_threshold" << YAML::Value << volume.fixed_ray_backface_threshold;
  out << YAML::Key << "probe_variability_threshold" << YAML::Value << volume.probe_variability_threshold;
  out << YAML::Key << "probe_variability_min_samples" << YAML::Value << volume.probe_variability_min_samples;
  out << YAML::Key << "hysteresis_boost_trigger_conditions" << YAML::Value
      << volume.hysteresis_boost_trigger_conditions;
  out << YAML::Key << "variability_reset_trigger_conditions" << YAML::Value
      << volume.variability_reset_trigger_conditions;
  out << YAML::EndMap;
}

void DeserializeDdgiVolume(const YAML::Node& in, EnvironmentalLighting::DdgiVolume& volume) {
  volume = {};
  if (in["name"])
    volume.name = in["name"].as<std::string>();
  if (in["stable_id"])
    volume.stable_id = in["stable_id"].as<uint64_t>();
  if (in["transform"])
    volume.transform = in["transform"].as<glm::mat4>();
  if (in["probe_counts"])
    volume.probe_counts = in["probe_counts"].as<glm::ivec3>();
  if (in["probe_spacing"])
    volume.probe_spacing = in["probe_spacing"].as<glm::vec3>();
  if (in["volume_origin"])
    volume.volume_origin = in["volume_origin"].as<glm::vec3>();
  if (in["artist_priority"])
    volume.artist_priority = in["artist_priority"].as<int>();
  if (in["movement_type"])
    volume.movement_type = in["movement_type"].as<int>();
  if (in["emissive_mesh_sampling_mode"])
    volume.emissive_mesh_sampling_mode = in["emissive_mesh_sampling_mode"].as<int>();
  if (in["enabled"])
    volume.enabled = in["enabled"].as<bool>();
  if (in["enable_probe_relocation"])
    volume.enable_probe_relocation = in["enable_probe_relocation"].as<bool>();
  if (in["enable_probe_classification"])
    volume.enable_probe_classification = in["enable_probe_classification"].as<bool>();
  if (in["enable_probe_variability"])
    volume.enable_probe_variability = in["enable_probe_variability"].as<bool>();
  if (in["enable_probe_variability_gating"])
    volume.enable_probe_variability_gating = in["enable_probe_variability_gating"].as<bool>();
  if (in["relocation_distance"])
    volume.relocation_distance = in["relocation_distance"].as<float>();
  if (in["random_ray_backface_threshold"])
    volume.random_ray_backface_threshold = in["random_ray_backface_threshold"].as<float>();
  if (in["fixed_ray_backface_threshold"])
    volume.fixed_ray_backface_threshold = in["fixed_ray_backface_threshold"].as<float>();
  if (in["probe_variability_threshold"])
    volume.probe_variability_threshold = in["probe_variability_threshold"].as<float>();
  if (in["probe_variability_min_samples"])
    volume.probe_variability_min_samples = in["probe_variability_min_samples"].as<int>();
  bool has_boost_triggers = in["hysteresis_boost_trigger_conditions"].IsDefined();
  if (has_boost_triggers)
    volume.hysteresis_boost_trigger_conditions = in["hysteresis_boost_trigger_conditions"].as<int>();
  if (in["scene_change_hysteresis_trigger_conditions"]) {
    const auto legacy_triggers = in["scene_change_hysteresis_trigger_conditions"].as<int>();
    volume.hysteresis_boost_trigger_conditions =
        has_boost_triggers ? volume.hysteresis_boost_trigger_conditions | legacy_triggers : legacy_triggers;
    has_boost_triggers = true;
  }
  if (in["warmup_trigger_conditions"]) {
    const auto legacy_triggers = in["warmup_trigger_conditions"].as<int>();
    volume.hysteresis_boost_trigger_conditions =
        has_boost_triggers ? volume.hysteresis_boost_trigger_conditions | legacy_triggers : legacy_triggers;
    has_boost_triggers = true;
  }
  if (in["auto_invalidate_trigger_conditions"]) {
    const auto legacy_triggers = in["auto_invalidate_trigger_conditions"].as<int>();
    volume.hysteresis_boost_trigger_conditions =
        has_boost_triggers ? volume.hysteresis_boost_trigger_conditions | legacy_triggers : legacy_triggers;
  }
  if (in["variability_reset_trigger_conditions"])
    volume.variability_reset_trigger_conditions = in["variability_reset_trigger_conditions"].as<int>();
}
}  // namespace

void evo_engine::EnvironmentalLighting::IndirectEnvironmentSource::CollectAssetRef(std::vector<AssetRef>& list) {
  if (kind == IndirectEnvironmentSourceKind::EnvironmentalMap) {
    list.push_back(environmental_map);
  }
}

void evo_engine::EnvironmentalLighting::ReflectionProbeBakeBackground::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(cubemap);
  list.push_back(environmental_map);
}

void evo_engine::EnvironmentalLighting::DynamicReflectionProbeSettings::Clamp() {
  faces_per_frame = glm::clamp(faces_per_frame, 1, 6);
}

bool evo_engine::EnvironmentalLighting::RepairStableIds() {
  const bool local_probe_ids_changed = RepairEntryStableIds(local_reflection_probes);
  const bool ddgi_volume_ids_changed = RepairEntryStableIds(ddgi_volumes);
  return local_probe_ids_changed || ddgi_volume_ids_changed;
}

void evo_engine::EnvironmentalLighting::LocalReflectionProbe::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(global_reflection_probe);
}

void evo_engine::EnvironmentalLighting::DdgiVolume::ClampSettings() {
  probe_spacing = ClampDdgiProbeSpacing(probe_spacing);
  movement_type = glm::clamp(movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                             static_cast<int>(DdgiVolumeMovementType::Scrolling));
  emissive_mesh_sampling_mode =
      glm::clamp(emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit),
                 static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
  relocation_distance = glm::clamp(relocation_distance, 0.0f, 10000.0f);
  random_ray_backface_threshold = glm::clamp(random_ray_backface_threshold, 0.0f, 1.0f);
  fixed_ray_backface_threshold = glm::clamp(fixed_ray_backface_threshold, 0.0f, 1.0f);
  probe_variability_threshold = glm::clamp(probe_variability_threshold, 0.0f, 10.0f);
  probe_variability_min_samples = glm::clamp(probe_variability_min_samples, 0, 4096);
  hysteresis_boost_trigger_conditions &= DdgiVolumeTriggerConditionAll;
  variability_reset_trigger_conditions &= DdgiVolumeTriggerConditionAll;
}

uint32_t evo_engine::EnvironmentalLighting::DdgiVolume::GetProbeAmount() const {
  if (probe_counts.x < 1 || probe_counts.x > 256 || probe_counts.y < 1 || probe_counts.y > 256 || probe_counts.z < 1 ||
      probe_counts.z > 256) {
    return 0u;
  }
  return static_cast<uint32_t>(probe_counts.x) * static_cast<uint32_t>(probe_counts.y) *
         static_cast<uint32_t>(probe_counts.z);
}

glm::vec3 evo_engine::EnvironmentalLighting::DdgiVolume::GetLocalGridSize() const {
  const auto counts = ClampDdgiProbeCounts(probe_counts);
  const auto spacing = ClampDdgiProbeSpacing(probe_spacing);
  return glm::vec3(counts - glm::ivec3(1)) * spacing;
}

glm::vec3 evo_engine::EnvironmentalLighting::DdgiVolume::GetProbeLocalPosition(const glm::ivec3& probe_index) const {
  const auto counts = ClampDdgiProbeCounts(probe_counts);
  const auto spacing = ClampDdgiProbeSpacing(probe_spacing);
  const auto clamped_index =
      glm::ivec3(glm::clamp(probe_index.x, 0, counts.x - 1), glm::clamp(probe_index.y, 0, counts.y - 1),
                 glm::clamp(probe_index.z, 0, counts.z - 1));
  const auto grid_shift = glm::vec3(counts - glm::ivec3(1)) * spacing * 0.5f;
  return volume_origin + glm::vec3(clamped_index) * spacing - grid_shift;
}

void evo_engine::EnvironmentalLighting::CollectAssetRef(std::vector<AssetRef>& list) {
  indirect_environment_source.CollectAssetRef(list);
  reflection_probe_bake_background.CollectAssetRef(list);
  for (auto& probe : local_reflection_probes) {
    probe.CollectAssetRef(list);
  }
}

float evo_engine::EnvironmentalLighting::EvaluateRoughSpecularVisibility(const float material_occlusion,
                                                                         const float screen_space_visibility,
                                                                         const float ddgi_visibility,
                                                                         const float roughness,
                                                                         const float normal_dot_view) {
  const float scalar_visibility =
      glm::min(glm::clamp(material_occlusion, 0.0f, 1.0f),
               glm::min(glm::clamp(screen_space_visibility, 0.0f, 1.0f), glm::clamp(ddgi_visibility, 0.0f, 1.0f)));
  const float clamped_roughness = glm::clamp(roughness, 0.0f, 1.0f);
  const float lobe_width = clamped_roughness * clamped_roughness;
  const float scalar_occlusion = 1.0f - scalar_visibility;
  const float grazing_occlusion =
      kSpecularVisibilityGrazingOcclusionCap * std::tanh(scalar_occlusion / kSpecularVisibilityGrazingOcclusionCap);
  const float view_confidence =
      glm::smoothstep(kSpecularVisibilityFullTrustStart, 1.0f, glm::clamp(normal_dot_view, 0.0f, 1.0f));
  const float trusted_occlusion = glm::mix(grazing_occlusion, scalar_occlusion, view_confidence);
  return 1.0f - lobe_width * trusted_occlusion;
}

void evo_engine::SerializeEnvironmentalLighting(YAML::Emitter& out, const EnvironmentalLighting& lighting) {
  out << YAML::Key << "indirect_environment_source" << YAML::Value;
  SerializeIndirectEnvironmentSource(out, lighting.indirect_environment_source);
  out << YAML::Key << "reflection_probe_bake_background" << YAML::Value;
  SerializeReflectionProbeBakeBackground(out, lighting.reflection_probe_bake_background);
  out << YAML::Key << "dynamic_reflection_probe_settings" << YAML::Value;
  SerializeDynamicReflectionProbeSettings(out, lighting.dynamic_reflection_probe_settings);
  out << YAML::Key << "environment_lighting_intensity" << YAML::Value << lighting.environment_lighting_intensity;
  out << YAML::Key << "diffuse_fallback_intensity" << YAML::Value << lighting.diffuse_fallback_intensity;
  out << YAML::Key << "specular_fallback_intensity" << YAML::Value << lighting.specular_fallback_intensity;
  out << YAML::Key << "ddgi_settings" << YAML::Value;
  SerializeDdgiSettings(out, lighting.ddgi_settings);

  out << YAML::Key << "local_reflection_probes_enabled" << YAML::Value << lighting.local_reflection_probes_enabled;
  out << YAML::Key << "local_reflection_probes" << YAML::Value << YAML::BeginSeq;
  for (const auto& probe : lighting.local_reflection_probes) {
    SerializeLocalReflectionProbe(out, probe);
  }
  out << YAML::EndSeq;

  out << YAML::Key << "ddgi_volumes" << YAML::Value << YAML::BeginSeq;
  for (const auto& volume : lighting.ddgi_volumes) {
    SerializeDdgiVolume(out, volume);
  }
  out << YAML::EndSeq;
}

void evo_engine::DeserializeEnvironmentalLighting(const YAML::Node& in, EnvironmentalLighting& lighting) {
  lighting.indirect_environment_source = {};
  lighting.reflection_probe_bake_background = {};
  lighting.dynamic_reflection_probe_settings = {};
  lighting.environment_lighting_intensity = EnvironmentalLighting::kDefaultEnvironmentLightingIntensity;
  lighting.diffuse_fallback_intensity = EnvironmentalLighting::kDefaultDiffuseFallbackIntensity;
  lighting.specular_fallback_intensity = EnvironmentalLighting::kDefaultSpecularFallbackIntensity;
  lighting.ddgi_settings = {};
  lighting.local_reflection_probes_enabled = true;
  lighting.local_reflection_probes.clear();
  lighting.ddgi_volumes.clear();

  if (const auto source = in["indirect_environment_source"]) {
    DeserializeIndirectEnvironmentSource(source, lighting.indirect_environment_source);
  }
  if (const auto background = in["reflection_probe_bake_background"]) {
    DeserializeReflectionProbeBakeBackground(background, lighting.reflection_probe_bake_background);
  }
  if (const auto settings = in["dynamic_reflection_probe_settings"]) {
    DeserializeDynamicReflectionProbeSettings(settings, lighting.dynamic_reflection_probe_settings);
  }
  if (in["environment_lighting_intensity"]) {
    lighting.environment_lighting_intensity = in["environment_lighting_intensity"].as<float>();
  }
  if (in["diffuse_fallback_intensity"]) {
    lighting.diffuse_fallback_intensity = in["diffuse_fallback_intensity"].as<float>();
  }
  if (in["specular_fallback_intensity"]) {
    lighting.specular_fallback_intensity = in["specular_fallback_intensity"].as<float>();
  }
  if (const auto settings = in["ddgi_settings"]) {
    DeserializeDdgiSettings(settings, lighting.ddgi_settings);
  }
  if (in["local_reflection_probes_enabled"]) {
    lighting.local_reflection_probes_enabled = in["local_reflection_probes_enabled"].as<bool>();
  }
  if (const auto probes = in["local_reflection_probes"]) {
    for (const auto& in_probe : probes) {
      EnvironmentalLighting::LocalReflectionProbe probe;
      DeserializeLocalReflectionProbe(in_probe, probe);
      lighting.local_reflection_probes.push_back(std::move(probe));
    }
  }
  if (const auto volumes = in["ddgi_volumes"]) {
    for (const auto& in_volume : volumes) {
      EnvironmentalLighting::DdgiVolume volume;
      DeserializeDdgiVolume(in_volume, volume);
      lighting.ddgi_volumes.push_back(std::move(volume));
    }
  }
  lighting.RepairStableIds();
}
