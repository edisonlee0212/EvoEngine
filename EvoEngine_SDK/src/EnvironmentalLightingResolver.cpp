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

struct LocalProbeCandidate {
  ResolvedEnvironmentalLighting::LocalReflectionProbe probe;
  size_t source_index = 0;
  float influence_volume = 0.0f;
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
  resolved.hddagi_settings = lighting.hddagi_settings;
  resolved.use_occlusion = lighting.use_occlusion;
  resolved.sdfgi_settings.use_occlusion = resolved.hddagi_settings.use_occlusion = resolved.use_occlusion;
  resolved.ddgi_settings.runtime.enable_probe_relocation &= !resolved.use_occlusion;
  resolved.ddgi_settings.runtime.use_voxel_occlusion = resolved.use_occlusion;
  resolved.ddgi_settings.runtime.enabled = lighting.indirect_gi_provider == IndirectGiProvider::AutomaticDdgi;
  auto dynamic_settings = lighting.dynamic_reflection_probe_settings;
  dynamic_settings.Clamp();
  resolved.dynamic_reflection_probe_settings = {static_cast<uint32_t>(dynamic_settings.faces_per_frame),
                                                dynamic_settings.enabled};
  ResolveLocalProbes(lighting, resolved);
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
  if (resolved.ddgi_settings.runtime.enabled) {
    const auto frame = scene->GetGiProbeFrame();
    if (frame && frame->settings.Validate().empty() && (frame->failure.empty() || !frame->anchor.camera_id)) {
      for (uint32_t cascade = 0; cascade < frame->placements.size(); ++cascade) {
        const auto& placement = frame->placements[cascade];
        auto& volume = resolved.ddgi_cascades.emplace_back();
        volume.name = "Cascade " + std::to_string(cascade);
        volume.stable_id = cascade + 1;
        volume.probe_center = placement.center;
        volume.probe_counts = frame->settings.ProbeSize();
        volume.probe_spacing = placement.interval;
      }
    }
  }
  return resolved;
}

std::vector<DdgiCascadeRuntimeInfo> evo_engine::CollectDdgiCascadeRuntimeInfos(
    const ResolvedEnvironmentalLighting& lighting) {
  std::vector<DdgiCascadeRuntimeInfo> infos;
  infos.reserve(lighting.ddgi_cascades.size());
  for (size_t i = 0; i < lighting.ddgi_cascades.size(); ++i) {
    const auto& volume = lighting.ddgi_cascades[i];
    const auto counts = glm::max(volume.probe_counts, glm::ivec3(1));
    const auto spacing = volume.probe_spacing;
    const auto first_probe_local = (glm::vec3(volume.probe_center) - glm::vec3(counts - 1) * 0.5f) * spacing;
    auto& info = infos.emplace_back();
    info.sorted_index = static_cast<uint32_t>(i);
    info.stable_entity_id = volume.stable_id;
    info.probe_counts = counts;
    info.probe_count = DdgiRuntime::GetProbeCount(counts);
    info.first_probe = first_probe_local;
    info.probe_step_x = glm::vec3(spacing.x, 0.0f, 0.0f);
    info.probe_step_y = glm::vec3(0.0f, spacing.y, 0.0f);
    info.probe_step_z = glm::vec3(0.0f, 0.0f, spacing.z);
  }
  return infos;
}
