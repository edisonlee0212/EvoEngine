#include "EnvironmentalLightingResolver.hpp"

#include "DdgiRuntime.hpp"
#include "Scene.hpp"

#include <algorithm>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include <limits>
#include <vector>

using namespace evo_engine;

namespace {
constexpr float kMinimumExtent = 1.0e-4f;

bool IsFinite(const float value) {
  return std::isfinite(value);
}

bool IsFinite(const glm::vec3& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

bool IsFinite(const glm::mat4& value) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      if (!IsFinite(value[column][row])) {
        return false;
      }
    }
  }
  return true;
}

float FiniteOr(const float value, const float fallback) {
  return IsFinite(value) ? value : fallback;
}

glm::vec3 FiniteOr(const glm::vec3& value, const glm::vec3& fallback) {
  return {IsFinite(value.x) ? value.x : fallback.x, IsFinite(value.y) ? value.y : fallback.y,
          IsFinite(value.z) ? value.z : fallback.z};
}

float NonNegativeFiniteOr(const float value, const float fallback) {
  return glm::max(FiniteOr(value, fallback), 0.0f);
}

glm::vec3 ClampProbeSpacing(const glm::vec3& value) {
  return glm::clamp(FiniteOr(value, glm::vec3(1.5f)), glm::vec3(0.05f), glm::vec3(10000.0f));
}

glm::vec3 TransformVector(const glm::mat4& transform, const glm::vec3& vector) {
  return glm::vec3(transform * glm::vec4(vector, 0.0f));
}

float CalculateTransformedLocalProbeInfluenceVolume(const ResolvedEnvironmentalLighting::LocalReflectionProbe& probe) {
  if (!IsFinite(probe.transform)) {
    return 0.0f;
  }
  const float volume_scale = glm::abs(glm::determinant(glm::mat3(probe.transform)));
  const float local_volume =
      probe.shape == static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere)
          ? 4.0f / 3.0f * glm::pi<float>() * probe.sphere_radius * probe.sphere_radius * probe.sphere_radius
          : 1.0f;
  const float influence_volume = local_volume * volume_scale;
  return std::isfinite(volume_scale) && volume_scale > 1.0e-8f && std::isfinite(influence_volume) ? influence_volume
                                                                                                  : 0.0f;
}

float CalculateDdgiProbeDensity(const EnvironmentalLighting::DdgiVolume& volume) {
  if (!IsFinite(volume.transform)) {
    return 0.0f;
  }
  const auto spacing = ClampProbeSpacing(volume.probe_spacing);
  return DdgiRuntime::CalculateProbeDensity(TransformVector(volume.transform, {spacing.x, 0.0f, 0.0f}),
                                            TransformVector(volume.transform, {0.0f, spacing.y, 0.0f}),
                                            TransformVector(volume.transform, {0.0f, 0.0f, spacing.z}));
}

ResolvedEnvironmentalLighting::IndirectEnvironmentSource ToResolvedSource(
    const EnvironmentalLighting::IndirectEnvironmentSource& source) {
  ResolvedEnvironmentalLighting::IndirectEnvironmentSource resolved;
  resolved.kind = static_cast<ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind>(source.kind);
  resolved.environmental_map = source.environmental_map;
  resolved.color = FiniteOr(source.color, glm::vec3(0.0f));
  resolved.gamma = NonNegativeFiniteOr(source.gamma, 2.2f);
  resolved.rotation = FiniteOr(source.rotation, 0.0f);
  if (resolved.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap &&
      resolved.environmental_map.GetAssetHandle().GetValue() == 0u) {
    resolved.kind = ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault;
  }
  return resolved;
}

ResolvedEnvironmentalLighting::LocalReflectionProbe ToResolvedLocalProbe(
    const EnvironmentalLighting::LocalReflectionProbe& source) {
  ResolvedEnvironmentalLighting::LocalReflectionProbe probe;
  probe.payload = source.HasValidPayload() ? source.payload : nullptr;
  probe.transform = source.transform;
  probe.box_projection_extents =
      glm::max(FiniteOr(source.box_projection_extents, glm::vec3(0.5f)), glm::vec3(kMinimumExtent));
  probe.sphere_radius = glm::max(FiniteOr(source.sphere_radius, 5.0f), kMinimumExtent);
  probe.blend_distance = NonNegativeFiniteOr(source.blend_distance, 0.05f);
  probe.reflection_intensity = NonNegativeFiniteOr(source.reflection_intensity, 1.0f);
  probe.stable_id = source.stable_id;
  probe.artist_priority =
      glm::clamp(source.artist_priority, -EnvironmentalLighting::kMaxExactLocalReflectionProbePriority,
                 EnvironmentalLighting::kMaxExactLocalReflectionProbePriority);
  probe.shape = glm::clamp(source.shape, static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Box),
                           static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere));
  probe.box_projection = source.box_projection;
  probe.enabled = source.enabled;
  const float maximum_blend = probe.shape == static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Box)
                                  ? 0.5f
                                  : probe.sphere_radius;
  probe.blend_distance = glm::min(probe.blend_distance, maximum_blend);
  return probe;
}

ResolvedEnvironmentalLighting::DdgiVolume ToResolvedDdgiVolume(const EnvironmentalLighting::DdgiVolume& source) {
  ResolvedEnvironmentalLighting::DdgiVolume volume;
  volume.name = source.name;
  volume.transform = IsFinite(source.transform) ? source.transform : glm::mat4(1.0f);
  volume.probe_counts = source.probe_counts;
  volume.probe_spacing = ClampProbeSpacing(source.probe_spacing);
  volume.volume_origin = FiniteOr(source.volume_origin, glm::vec3(0.0f));
  volume.stable_id = source.stable_id;
  volume.artist_priority = source.artist_priority;
  volume.movement_type = glm::clamp(source.movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                                    static_cast<int>(DdgiVolumeMovementType::Scrolling));
  volume.emissive_mesh_sampling_mode =
      glm::clamp(source.emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit),
                 static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
  volume.enabled = source.enabled;
  volume.enable_probe_relocation = source.enable_probe_relocation;
  volume.enable_probe_classification = source.enable_probe_classification;
  volume.relocation_distance = glm::clamp(FiniteOr(source.relocation_distance, 0.25f), 0.0f, 10000.0f);
  return volume;
}

struct LocalProbeCandidate {
  ResolvedEnvironmentalLighting::LocalReflectionProbe probe;
  size_t source_index = 0;
  float influence_volume = 0.0f;
};

struct DdgiVolumeCandidate {
  EnvironmentalLighting::DdgiVolume volume;
  size_t source_index = 0;
  float probe_density = 0.0f;
};

void SortLocalProbeCandidates(std::vector<LocalProbeCandidate>& candidates) {
  std::sort(candidates.begin(), candidates.end(), [](const LocalProbeCandidate& lhs, const LocalProbeCandidate& rhs) {
    if (lhs.probe.artist_priority != rhs.probe.artist_priority) {
      return lhs.probe.artist_priority > rhs.probe.artist_priority;
    }
    if (lhs.influence_volume != rhs.influence_volume) {
      return lhs.influence_volume < rhs.influence_volume;
    }
    if (lhs.probe.stable_id != rhs.probe.stable_id) {
      return lhs.probe.stable_id < rhs.probe.stable_id;
    }
    return lhs.source_index < rhs.source_index;
  });
}

void SortDdgiVolumeCandidates(std::vector<DdgiVolumeCandidate>& candidates) {
  std::sort(candidates.begin(), candidates.end(), [](const DdgiVolumeCandidate& lhs, const DdgiVolumeCandidate& rhs) {
    if (lhs.volume.artist_priority != rhs.volume.artist_priority) {
      return lhs.volume.artist_priority > rhs.volume.artist_priority;
    }
    if (lhs.probe_density != rhs.probe_density) {
      return lhs.probe_density > rhs.probe_density;
    }
    if (lhs.volume.stable_id != rhs.volume.stable_id) {
      return lhs.volume.stable_id < rhs.volume.stable_id;
    }
    return lhs.source_index < rhs.source_index;
  });
}

void ResolveLocalProbes(const EnvironmentalLighting& lighting, ResolvedEnvironmentalLighting& resolved) {
  if (!lighting.local_reflection_probes_enabled) {
    return;
  }
  const auto pack = lighting.GetReflectionProbePack();
  if (!pack) {
    return;
  }
  std::vector<LocalProbeCandidate> candidates;
  candidates.reserve(pack->probes.size());
  for (size_t index = 0; index < pack->probes.size(); ++index) {
    if (!pack->probes[index].enabled) {
      continue;
    }
    auto probe = ToResolvedLocalProbe(pack->probes[index]);
    const float influence_volume = CalculateTransformedLocalProbeInfluenceVolume(probe);
    if (influence_volume <= 0.0f) {
      continue;
    }
    candidates.push_back({std::move(probe), index, influence_volume});
  }
  SortLocalProbeCandidates(candidates);
  if (candidates.size() > ResolvedEnvironmentalLighting::kMaxLocalReflectionProbeCount) {
    resolved.truncated_local_reflection_probe_count =
        static_cast<uint32_t>(candidates.size() - ResolvedEnvironmentalLighting::kMaxLocalReflectionProbeCount);
    candidates.resize(ResolvedEnvironmentalLighting::kMaxLocalReflectionProbeCount);
  }
  resolved.local_reflection_probes.reserve(candidates.size());
  for (auto& candidate : candidates) {
    resolved.local_reflection_probes.push_back(std::move(candidate.probe));
  }
}

void ResolveDdgiVolumes(const EnvironmentalLighting& lighting, ResolvedEnvironmentalLighting& resolved) {
  const auto pack = lighting.GetDdgiVolumePack();
  if (!pack) {
    return;
  }
  std::vector<DdgiVolumeCandidate> candidates;
  candidates.reserve(pack->volumes.size());
  const auto max_probe_count = lighting.ddgi_settings.storage.max_probe_count > 0
                                   ? static_cast<uint32_t>(lighting.ddgi_settings.storage.max_probe_count)
                                   : 0u;
  for (size_t index = 0; index < pack->volumes.size(); ++index) {
    const auto& volume = pack->volumes[index];
    if (!volume.enabled || !IsFinite(volume.transform) ||
        !DdgiRuntime::ValidateProbeGrid(volume.probe_counts, max_probe_count)) {
      continue;
    }
    const float probe_density = CalculateDdgiProbeDensity(volume);
    if (!(probe_density > 0.0f) || !std::isfinite(probe_density)) {
      continue;
    }
    candidates.push_back({volume, index, probe_density});
  }
  SortDdgiVolumeCandidates(candidates);
  if (candidates.size() > ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount) {
    resolved.truncated_ddgi_volume_count =
        static_cast<uint32_t>(candidates.size() - ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount);
    candidates.resize(ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount);
  }
  resolved.ddgi_volumes.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    resolved.ddgi_volumes.push_back(ToResolvedDdgiVolume(candidate.volume));
  }
}

void ResolveFromAsset(const EnvironmentalLighting& lighting, ResolvedEnvironmentalLighting& resolved) {
  resolved.indirect_environment_source = ToResolvedSource(lighting.indirect_environment_source);
  resolved.uses_engine_default_indirect_environment_source =
      resolved.indirect_environment_source.kind ==
      ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault;
  resolved.environment_lighting_intensity = NonNegativeFiniteOr(
      lighting.environment_lighting_intensity, ResolvedEnvironmentalLighting::kDefaultEnvironmentLightingIntensity);
  resolved.diffuse_fallback_intensity = NonNegativeFiniteOr(
      lighting.diffuse_fallback_intensity, ResolvedEnvironmentalLighting::kDefaultDiffuseFallbackIntensity);
  resolved.specular_fallback_intensity = NonNegativeFiniteOr(
      lighting.specular_fallback_intensity, ResolvedEnvironmentalLighting::kDefaultSpecularFallbackIntensity);
  resolved.ddgi_settings = lighting.ddgi_settings;
  resolved.indirect_gi_provider = lighting.indirect_gi_provider;
  resolved.gi_probe_settings = lighting.gi_probe_settings;
  resolved.sdfgi_settings = DeriveSdfgiSettings(lighting.gi_probe_settings, lighting.sdfgi_settings);
  if (lighting.indirect_gi_provider != IndirectGiProvider::AuthoredDdgi)
    resolved.ddgi_settings.runtime.enabled = false;
  auto dynamic_settings = lighting.dynamic_reflection_probe_settings;
  dynamic_settings.Clamp();
  resolved.dynamic_reflection_probe_settings = {static_cast<uint32_t>(dynamic_settings.faces_per_frame),
                                                dynamic_settings.enabled};
  ResolveLocalProbes(lighting, resolved);
  if (lighting.indirect_gi_provider == IndirectGiProvider::AuthoredDdgi)
    ResolveDdgiVolumes(lighting, resolved);
}
}  // namespace

ResolvedEnvironmentalLighting evo_engine::ResolveEnvironmentalLighting(const std::shared_ptr<Scene>& scene) {
  ResolvedEnvironmentalLighting resolved;
  if (!scene) {
    return resolved;
  }
  resolved.scene_global_reflection_probe_fallback = scene->global_reflection_probe_fallback;
  resolved.environmental_lighting_asset_assigned = scene->environmental_lighting.GetAssetHandle().GetValue() != 0u;
  if (!resolved.environmental_lighting_asset_assigned) {
    return resolved;
  }
  const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting) {
    resolved.environmental_lighting_asset_missing = true;
    return resolved;
  }
  ResolveFromAsset(*lighting, resolved);
  return resolved;
}

std::vector<DdgiVolumeRuntimeInfo> evo_engine::CollectDdgiVolumeRuntimeInfos(
    const ResolvedEnvironmentalLighting& lighting) {
  std::vector<DdgiVolumeRuntimeInfo> infos;
  infos.reserve(lighting.ddgi_volumes.size());
  for (size_t i = 0; i < lighting.ddgi_volumes.size(); ++i) {
    const auto& volume = lighting.ddgi_volumes[i];
    const auto counts = glm::max(volume.probe_counts, glm::ivec3(1));
    const auto spacing = glm::max(volume.probe_spacing, glm::vec3(kMinimumExtent));
    const auto first_probe_local = volume.volume_origin - glm::vec3(counts - glm::ivec3(1)) * spacing * 0.5f;
    auto& info = infos.emplace_back();
    info.sorted_index = static_cast<uint32_t>(i);
    info.stable_entity_id = volume.stable_id;
    info.artist_priority = volume.artist_priority;
    info.probe_counts = counts;
    info.probe_count = DdgiRuntime::GetProbeCount(counts);
    info.first_probe = glm::vec3(volume.transform * glm::vec4(first_probe_local, 1.0f));
    info.probe_step_x = glm::vec3(volume.transform * glm::vec4(spacing.x, 0.0f, 0.0f, 0.0f));
    info.probe_step_y = glm::vec3(volume.transform * glm::vec4(0.0f, spacing.y, 0.0f, 0.0f));
    info.probe_step_z = glm::vec3(volume.transform * glm::vec4(0.0f, 0.0f, spacing.z, 0.0f));
    info.probe_density = DdgiRuntime::CalculateProbeDensity(info.probe_step_x, info.probe_step_y, info.probe_step_z);
  }
  return infos;
}
