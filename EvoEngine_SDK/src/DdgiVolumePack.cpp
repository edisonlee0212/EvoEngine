#include "DdgiVolumePack.hpp"

#include "Serialization.hpp"

#include <unordered_set>

using namespace evo_engine;

namespace {
glm::ivec3 ClampProbeCounts(const glm::ivec3& value) {
  return {glm::clamp(value.x, 1, 256), glm::clamp(value.y, 1, 256), glm::clamp(value.z, 1, 256)};
}

glm::vec3 ClampProbeSpacing(const glm::vec3& value) {
  return glm::clamp(value, glm::vec3(0.05f), glm::vec3(10000.0f));
}

void SerializeVolume(YAML::Emitter& out, const DdgiVolumePack::Volume& volume) {
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
  out << YAML::Key << "relocation_distance" << YAML::Value << volume.relocation_distance;
  out << YAML::EndMap;
}

void DeserializeVolume(const YAML::Node& in, DdgiVolumePack::Volume& volume) {
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
  if (in["relocation_distance"])
    volume.relocation_distance = in["relocation_distance"].as<float>();
  volume.ClampSettings();
}
}  // namespace

void DdgiVolumePack::Volume::ClampSettings() {
  probe_spacing = ClampProbeSpacing(probe_spacing);
  movement_type = glm::clamp(movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                             static_cast<int>(DdgiVolumeMovementType::Scrolling));
  emissive_mesh_sampling_mode =
      glm::clamp(emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit),
                 static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
  relocation_distance = glm::clamp(relocation_distance, 0.0f, 10000.0f);
}

uint32_t DdgiVolumePack::Volume::GetProbeAmount() const {
  if (probe_counts.x < 1 || probe_counts.x > 256 || probe_counts.y < 1 || probe_counts.y > 256 || probe_counts.z < 1 ||
      probe_counts.z > 256) {
    return 0u;
  }
  return static_cast<uint32_t>(probe_counts.x) * static_cast<uint32_t>(probe_counts.y) *
         static_cast<uint32_t>(probe_counts.z);
}

glm::vec3 DdgiVolumePack::Volume::GetLocalGridSize() const {
  return glm::vec3(ClampProbeCounts(probe_counts) - glm::ivec3(1)) * ClampProbeSpacing(probe_spacing);
}

glm::vec3 DdgiVolumePack::Volume::GetProbeLocalPosition(const glm::ivec3& probe_index) const {
  const auto counts = ClampProbeCounts(probe_counts);
  const auto spacing = ClampProbeSpacing(probe_spacing);
  const auto clamped = glm::clamp(probe_index, glm::ivec3(0), counts - glm::ivec3(1));
  return volume_origin + glm::vec3(clamped) * spacing - glm::vec3(counts - glm::ivec3(1)) * spacing * 0.5f;
}

bool DdgiVolumePack::RepairStableIds() {
  std::unordered_set<uint64_t> used;
  bool changed = false;
  uint64_t candidate = 1u;
  for (auto& volume : volumes) {
    if (volume.stable_id != 0u && used.emplace(volume.stable_id).second) {
      continue;
    }
    while (used.find(candidate) != used.end())
      ++candidate;
    volume.stable_id = candidate++;
    used.emplace(volume.stable_id);
    changed = true;
  }
  return changed;
}

void evo_engine::SerializeDdgiVolumePack(YAML::Emitter& out, const DdgiVolumePack& pack) {
  out << YAML::Key << "volumes" << YAML::Value << YAML::BeginSeq;
  for (const auto& volume : pack.volumes)
    SerializeVolume(out, volume);
  out << YAML::EndSeq;
}

void evo_engine::DeserializeDdgiVolumePack(const YAML::Node& in, DdgiVolumePack& pack) {
  pack.volumes.clear();
  if (const auto volumes = in["volumes"]) {
    for (const auto& input : volumes) {
      auto& volume = pack.volumes.emplace_back();
      DeserializeVolume(input, volume);
    }
  }
  (void)pack.RepairStableIds();
}
