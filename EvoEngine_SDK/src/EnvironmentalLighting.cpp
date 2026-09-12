#include "EnvironmentalLighting.hpp"

#include "AssetManager.hpp"
#include "Camera.hpp"
#include "DdgiRuntime.hpp"
#include "HddagiResources.hpp"
#include "Platform.hpp"
#include "SdfgiCapabilities.hpp"
#include "SdfgiProbeLayout.hpp"
#include "Serialization.hpp"

#include <cmath>

using namespace evo_engine;

GiSettings EnvironmentalLighting::GetGiSettings() const {
  GiSettings result{gi_probe_settings, indirect_gi_provider, DeriveSdfgiSettings(gi_probe_settings, sdfgi_settings),
                    ddgi_settings,     hddagi_settings,      use_occlusion};
  result.sdfgi_settings.use_occlusion = result.hddagi_settings.use_occlusion = use_occlusion;
  return result;
}

void EnvironmentalLighting::ApplyGiSettings(const GiSettings& settings) {
  gi_probe_settings = settings.gi_probe_settings;
  indirect_gi_provider = settings.indirect_gi_provider;
  sdfgi_settings = DeriveSdfgiSettings(gi_probe_settings, settings.sdfgi_settings);
  ddgi_settings = settings.ddgi_settings;
  hddagi_settings = settings.hddagi_settings;
  use_occlusion = settings.use_occlusion;
  sdfgi_settings.use_occlusion = hddagi_settings.use_occlusion = use_occlusion;
}

bool EnvironmentalLighting::TrySetGiSettings(const GiSettings& settings, std::string& error) {
  error = settings.Validate();
  if (!error.empty())
    return false;
  ApplyGiSettings(settings);
  SetUnsaved();
  return true;
}

bool GiSettings::operator==(const GiSettings& other) const {
  return use_occlusion == other.use_occlusion && gi_probe_settings == other.gi_probe_settings &&
         indirect_gi_provider == other.indirect_gi_provider &&
         DeriveSdfgiSettings(gi_probe_settings, sdfgi_settings) ==
             DeriveSdfgiSettings(other.gi_probe_settings, other.sdfgi_settings) &&
         ddgi_settings == other.ddgi_settings && hddagi_settings == other.hddagi_settings;
}

std::string GiSettings::Validate(const bool device_limits) const {
  if (const auto error = gi_probe_settings.Validate(); !error.empty())
    return error;
  if (indirect_gi_provider == IndirectGiProvider::Environment)
    return {};
  if (indirect_gi_provider == IndirectGiProvider::AutomaticSdfgi) {
    const auto settings = DeriveSdfgiSettings(gi_probe_settings, sdfgi_settings);
    if (const auto error = settings.Validate(); !error.empty())
      return error;
    if (device_limits) {
      const auto report = QuerySdfgiCapabilities(settings.cascade_count, settings.history_size, settings.voxel_count_x,
                                                 settings.voxel_count_y, settings.probe_spacing_cells, 0);
      return report.Supported() ? std::string{} : report.ToString();
    }
    uint64_t bytes = 0;
    const auto layout = SdfgiProbeLayout::Create(settings.voxel_count_x, settings.voxel_count_y,
                                                 settings.probe_spacing_cells, UINT32_MAX);
    return layout.HistoryBytes(settings.cascade_count, settings.history_size, bytes) && GiHistoryBudget{}.CanAdd(bytes)
               ? std::string{}
               : "GI histories must remain strictly below 4 GiB.";
  }
  if (indirect_gi_provider == IndirectGiProvider::AutomaticHddagi) {
    if (const auto error = hddagi_settings.Validate(gi_probe_settings); !error.empty())
      return error;
    if (device_limits)
      return QueryHddagiCapabilities(gi_probe_settings, hddagi_settings).failure;
    return HddagiLogicalTemporalBytes(gi_probe_settings, hddagi_settings) < (uint64_t{4} << 30)
               ? std::string{}
               : "HDDAGI temporal storage must remain below 4 GiB.";
  }
  if (indirect_gi_provider != IndirectGiProvider::AutomaticDdgi)
    return "Unknown indirect GI provider.";
  if (use_occlusion) {
    if (const auto error = HddagiSettings{}.Validate(gi_probe_settings); !error.empty())
      return error;
    if (device_limits) {
      const auto report = QueryHddagiCapabilities(gi_probe_settings, {}, true);
      if (!report.Supported())
        return report.failure;
    }
  }
  const auto& runtime = ddgi_settings.runtime;
  const auto in_range = [](const float value, const float minimum, const float maximum) {
    return std::isfinite(value) && value >= minimum && value <= maximum;
  };
  if (runtime.warmup_frames < 0 || runtime.warmup_frames > 4096 || !in_range(runtime.normal_bias, 0, 10) ||
      !in_range(runtime.view_bias, 0, 10) || !in_range(runtime.max_ray_distance, 0.05f, 1e27f) ||
      !in_range(runtime.distance_exponent, 0, 256) || !in_range(runtime.irradiance_gamma, 0.1f, 16) ||
      !in_range(runtime.visibility_moment_bias, 0, 10) || !in_range(runtime.relocation_distance, 0, 10000) ||
      !in_range(runtime.random_ray_backface_threshold, 0, 1) || !in_range(runtime.fixed_ray_backface_threshold, 0, 1))
    return "DDGI provider parameters must be finite and within their supported ranges.";
  const auto device = device_limits ? Platform::GetSelectedPhysicalDevice() : nullptr;
  if (device_limits && (!device || !Platform::RayTracingEnabled()))
    return "Automatic DDGI requires ray tracing enabled at startup.";
  const auto layout = DdgiRuntime::CalculateFrameResourceLayout(
      ddgi_settings, DdgiRuntime::GetProbeCount(gi_probe_settings.ProbeSize()),
      device ? device->properties.limits.maxImageDimension2D : UINT32_MAX,
      device ? device->properties.limits.maxStorageBufferRange : UINT64_MAX);
  if (!layout.valid)
    return layout.error;
  GiHistoryBudget budget;
  std::string error;
  for (uint32_t cascade = 0; cascade < gi_probe_settings.cascade_count; ++cascade) {
    if (device_limits) {
      if (!DdgiRuntime::AddDeviceHistoryAllocations(layout.history, budget, error))
        return error;
    } else if (!DdgiHistoryLayout::AddAllocationBytes(layout.history.buffer_bytes, budget)) {
      return "GI histories must remain strictly below 4 GiB.";
    }
  }
  return {};
}

namespace {
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
}  // namespace

void EnvironmentalLighting::IndirectEnvironmentSource::CollectAssetRef(std::vector<AssetRef>& list) {
  if (kind == IndirectEnvironmentSourceKind::EnvironmentalMap)
    list.push_back(environmental_map);
}

void EnvironmentalLighting::ReflectionProbeBakeBackground::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(cubemap);
  list.push_back(environmental_map);
}

void EnvironmentalLighting::DynamicReflectionProbeSettings::Clamp() {
  faces_per_frame = glm::clamp(faces_per_frame, 1, 6);
}

std::shared_ptr<ReflectionProbePack> EnvironmentalLighting::GetReflectionProbePack() const {
  auto ref = reflection_probe_pack;
  return ref.Get<ReflectionProbePack>();
}

std::shared_ptr<ReflectionProbePack> EnvironmentalLighting::GetOrCreateReflectionProbePack() {
  auto pack = GetReflectionProbePack();
  if (!pack) {
    pack = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
    reflection_probe_pack = pack;
  }
  return pack;
}

void EnvironmentalLighting::CollectAssetRef(std::vector<AssetRef>& list) {
  indirect_environment_source.CollectAssetRef(list);
  reflection_probe_bake_background.CollectAssetRef(list);
  list.push_back(reflection_probe_pack);
}

float EnvironmentalLighting::EvaluateRoughSpecularVisibility(const float material_occlusion,
                                                             const float screen_space_visibility,
                                                             const float ddgi_visibility, const float roughness,
                                                             const float normal_dot_view) {
  const float scalar_visibility =
      glm::min(glm::clamp(material_occlusion, 0.0f, 1.0f),
               glm::min(glm::clamp(screen_space_visibility, 0.0f, 1.0f), glm::clamp(ddgi_visibility, 0.0f, 1.0f)));
  const float lobe_width = glm::clamp(roughness, 0.0f, 1.0f) * glm::clamp(roughness, 0.0f, 1.0f);
  const float scalar_occlusion = 1.0f - scalar_visibility;
  const float grazing_occlusion =
      kSpecularVisibilityGrazingOcclusionCap * std::tanh(scalar_occlusion / kSpecularVisibilityGrazingOcclusionCap);
  const float view_confidence =
      glm::smoothstep(kSpecularVisibilityFullTrustStart, 1.0f, glm::clamp(normal_dot_view, 0.0f, 1.0f));
  return 1.0f - lobe_width * glm::mix(grazing_occlusion, scalar_occlusion, view_confidence);
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
  out << YAML::Key << "gi_probe_settings" << YAML::Value;
  SerializeGiProbeSettings(out, lighting.gi_probe_settings);
  out << YAML::Key << "indirect_gi_provider" << YAML::Value << static_cast<uint32_t>(lighting.indirect_gi_provider);
  out << YAML::Key << "use_occlusion" << YAML::Value << lighting.use_occlusion;
  if (lighting.indirect_gi_provider == IndirectGiProvider::AutomaticSdfgi ||
      !(lighting.sdfgi_settings == SdfgiSettings{})) {
    out << YAML::Key << "sdfgi_settings" << YAML::Value;
    SerializeSdfgiSettings(out, lighting.sdfgi_settings);
  }
  out << YAML::Key << "hddagi_settings" << YAML::Value;
  SerializeHddagiSettings(out, lighting.hddagi_settings);
  out << YAML::Key << "local_reflection_probes_enabled" << YAML::Value << lighting.local_reflection_probes_enabled;
  lighting.reflection_probe_pack.Save("reflection_probe_pack", out);
}

void evo_engine::DeserializeEnvironmentalLighting(const YAML::Node& in, EnvironmentalLighting& lighting) {
  lighting.indirect_environment_source = {};
  lighting.reflection_probe_bake_background = {};
  lighting.dynamic_reflection_probe_settings = {};
  lighting.environment_lighting_intensity = EnvironmentalLighting::kDefaultEnvironmentLightingIntensity;
  lighting.diffuse_fallback_intensity = EnvironmentalLighting::kDefaultDiffuseFallbackIntensity;
  lighting.specular_fallback_intensity = EnvironmentalLighting::kDefaultSpecularFallbackIntensity;
  lighting.ddgi_settings = {};
  lighting.indirect_gi_provider = IndirectGiProvider::AutomaticSdfgi;
  lighting.sdfgi_settings = {};
  lighting.hddagi_settings = {};
  lighting.gi_probe_settings = {};
  lighting.local_reflection_probes_enabled = true;
  lighting.reflection_probe_pack.Clear();
  if (const auto source = in["indirect_environment_source"])
    DeserializeIndirectEnvironmentSource(source, lighting.indirect_environment_source);
  if (const auto background = in["reflection_probe_bake_background"])
    DeserializeReflectionProbeBakeBackground(background, lighting.reflection_probe_bake_background);
  if (const auto settings = in["dynamic_reflection_probe_settings"])
    DeserializeDynamicReflectionProbeSettings(settings, lighting.dynamic_reflection_probe_settings);
  if (in["environment_lighting_intensity"])
    lighting.environment_lighting_intensity = in["environment_lighting_intensity"].as<float>();
  if (in["diffuse_fallback_intensity"])
    lighting.diffuse_fallback_intensity = in["diffuse_fallback_intensity"].as<float>();
  if (in["specular_fallback_intensity"])
    lighting.specular_fallback_intensity = in["specular_fallback_intensity"].as<float>();
  if (const auto settings = in["ddgi_settings"])
    DeserializeDdgiSettings(settings, lighting.ddgi_settings);
  if (in["indirect_gi_provider"]) {
    const auto provider = in["indirect_gi_provider"].as<uint32_t>();
    lighting.indirect_gi_provider = provider <= static_cast<uint32_t>(IndirectGiProvider::AutomaticHddagi)
                                        ? static_cast<IndirectGiProvider>(provider)
                                        : IndirectGiProvider::Environment;
  }
  if (const auto settings = in["hddagi_settings"])
    DeserializeHddagiSettings(settings, lighting.hddagi_settings);
  if (const auto settings = in["sdfgi_settings"]) {
    DeserializeSdfgiSettings(settings, lighting.sdfgi_settings);
    lighting.gi_probe_settings = GiProbesFromSdfgi(lighting.sdfgi_settings);
    if (lighting.sdfgi_settings.probe_spacing_cells == 1 || lighting.sdfgi_settings.probe_spacing_cells == 2)
      lighting.sdfgi_settings.probe_spacing_cells = 4;
  }
  if (const auto settings = in["gi_probe_settings"])
    DeserializeGiProbeSettings(settings, lighting.gi_probe_settings);
  lighting.sdfgi_settings = DeriveSdfgiSettings(lighting.gi_probe_settings, lighting.sdfgi_settings);
  lighting.use_occlusion = in["use_occlusion"] ? in["use_occlusion"].as<bool>()
                           : lighting.indirect_gi_provider == IndirectGiProvider::AutomaticDdgi ? false
                           : lighting.indirect_gi_provider == IndirectGiProvider::AutomaticHddagi
                               ? lighting.hddagi_settings.use_occlusion
                               : lighting.sdfgi_settings.use_occlusion;
  lighting.sdfgi_settings.use_occlusion = lighting.hddagi_settings.use_occlusion = lighting.use_occlusion;
  if (in["local_reflection_probes_enabled"])
    lighting.local_reflection_probes_enabled = in["local_reflection_probes_enabled"].as<bool>();
  lighting.reflection_probe_pack.Load("reflection_probe_pack", in);
}
