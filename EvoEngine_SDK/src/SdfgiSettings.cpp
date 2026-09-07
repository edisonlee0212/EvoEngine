// Defaults and layout transitions follow Godot environment.h and render_forward_clustered.cpp::sdfgi_update,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiSettings.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

using namespace evo_engine;

const char* evo_engine::GetIndirectGiProviderName(const IndirectGiProvider provider) {
  switch (provider) {
    case IndirectGiProvider::AuthoredDdgi:
      return "Authored DDGI (RT)";
    case IndirectGiProvider::AutomaticSdfgi:
      return "Automatic SDFGI";
    default:
      return "Environment";
  }
}

float SdfgiSettings::GetCascade0Distance() const {
  return min_cell_size * (voxel_count_x * 0.5f);
}

void SdfgiSettings::SetCascade0Distance(const float distance) {
  min_cell_size = distance / (voxel_count_x * 0.5f);
}

float SdfgiSettings::GetMaxDistance() const {
  // Godot's max-distance property is the outer cascade's full width, not its half extent.
  return std::ldexp(GetCascade0Distance(), static_cast<int>(cascade_count));
}

void SdfgiSettings::SetMaxDistance(const float distance) {
  SetCascade0Distance(std::ldexp(distance, -static_cast<int>(cascade_count)));
}

std::string SdfgiSettings::Validate() const {
  if (probe_spacing_cells != 1 && probe_spacing_cells != 2 && probe_spacing_cells != 4 && probe_spacing_cells != 8)
    return "Probe spacing must be 1, 2, 4, or 8 cells";
  if (voxel_count_x < 64 || voxel_count_x > 256 || voxel_count_x % 16 != 0 || voxel_count_y < 64 ||
      voxel_count_y > 256 || voxel_count_y % 16 != 0)
    return "Voxel counts must be 64..256 in steps of 16";
  constexpr std::array rays{4u, 8u, 16u, 32u, 64u, 96u, 128u};
  constexpr std::array light_frames{1u, 2u, 4u, 8u, 16u};
  if (cascade_count < 1 || cascade_count > 8)
    return "Cascade count must be 1..8";
  if (positional_light_cascade_count < 1 || positional_light_cascade_count > 8)
    return "Positional light cascade count must be 1..8";
  if (!std::isfinite(min_cell_size) || min_cell_size <= 0)
    return "Minimum cell size must be finite and positive";
  if (static_cast<uint32_t>(vertical_scale) > 2)
    return "Vertical scale must be 100%, 75%, or 50%";
  if (std::find(rays.begin(), rays.end(), ray_count) == rays.end())
    return "Ray count must be 4, 8, 16, 32, 64, 96, or 128";
  if (history_size < 5 || history_size > 30 || history_size % 5 != 0)
    return "History must be 5..30 frames in steps of 5";
  if (std::find(light_frames.begin(), light_frames.end(), light_update_frames) == light_frames.end())
    return "Light update cadence must be 1, 2, 4, 8, or 16 frames";
  if (!std::isfinite(bounce_feedback) || bounce_feedback < 0 || !std::isfinite(energy) || energy < 0 ||
      !std::isfinite(normal_bias) || normal_bias < 0 || !std::isfinite(probe_bias) || probe_bias < 0)
    return "Feedback, energy, and biases must be finite and nonnegative";
  return {};
}

bool SdfgiSettings::HasSameLayout(const SdfgiSettings& other) const {
  return voxel_count_x == other.voxel_count_x && voxel_count_y == other.voxel_count_y &&
         probe_spacing_cells == other.probe_spacing_cells && cascade_count == other.cascade_count &&
         min_cell_size == other.min_cell_size && vertical_scale == other.vertical_scale &&
         history_size == other.history_size && use_occlusion == other.use_occlusion &&
         probe_relocation == other.probe_relocation;
}

bool SdfgiSettings::operator==(const SdfgiSettings& other) const {
  return std::tie(voxel_count_x, voxel_count_y, probe_spacing_cells, cascade_count, positional_light_cascade_count,
                  min_cell_size, vertical_scale, use_occlusion, probe_relocation, static_entities_only, ray_count,
                  history_size, light_update_frames, bounce_feedback, read_sky_light, energy, normal_bias, probe_bias,
                  anchor_camera_entity) ==
         std::tie(other.voxel_count_x, other.voxel_count_y, other.probe_spacing_cells, other.cascade_count,
                  other.positional_light_cascade_count, other.min_cell_size, other.vertical_scale, other.use_occlusion,
                  other.probe_relocation, other.static_entities_only, other.ray_count, other.history_size,
                  other.light_update_frames, other.bounce_feedback, other.read_sky_light, other.energy,
                  other.normal_bias, other.probe_bias, other.anchor_camera_entity);
}

void evo_engine::SerializeSdfgiSettings(YAML::Emitter& out, const SdfgiSettings& settings) {
  out << YAML::BeginMap;
  out << YAML::Key << "voxel_count_x" << YAML::Value << settings.voxel_count_x;
  out << YAML::Key << "voxel_count_y" << YAML::Value << settings.voxel_count_y;
  out << YAML::Key << "probe_spacing_cells" << YAML::Value << settings.probe_spacing_cells;
  out << YAML::Key << "cascade_count" << YAML::Value << settings.cascade_count;
  out << YAML::Key << "positional_light_cascade_count" << YAML::Value << settings.positional_light_cascade_count;
  out << YAML::Key << "min_cell_size" << YAML::Value << settings.min_cell_size;
  out << YAML::Key << "vertical_scale" << YAML::Value << static_cast<uint32_t>(settings.vertical_scale);
  out << YAML::Key << "use_occlusion" << YAML::Value << settings.use_occlusion;
  out << YAML::Key << "probe_relocation" << YAML::Value << settings.probe_relocation;
  out << YAML::Key << "static_entities_only" << YAML::Value << settings.static_entities_only;
  out << YAML::Key << "ray_count" << YAML::Value << settings.ray_count;
  out << YAML::Key << "history_size" << YAML::Value << settings.history_size;
  out << YAML::Key << "light_update_frames" << YAML::Value << settings.light_update_frames;
  out << YAML::Key << "bounce_feedback" << YAML::Value << settings.bounce_feedback;
  out << YAML::Key << "read_sky_light" << YAML::Value << settings.read_sky_light;
  out << YAML::Key << "energy" << YAML::Value << settings.energy;
  out << YAML::Key << "normal_bias" << YAML::Value << settings.normal_bias;
  out << YAML::Key << "probe_bias" << YAML::Value << settings.probe_bias;
  out << YAML::Key << "anchor_camera_entity" << YAML::Value << settings.anchor_camera_entity;
  out << YAML::EndMap;
}

void evo_engine::DeserializeSdfgiSettings(const YAML::Node& in, SdfgiSettings& settings) {
  settings = {};
  if (in["voxel_count_x"])
    settings.voxel_count_x = in["voxel_count_x"].as<uint32_t>();
  if (in["voxel_count_y"])
    settings.voxel_count_y = in["voxel_count_y"].as<uint32_t>();
  if (in["probe_spacing_cells"])
    settings.probe_spacing_cells = in["probe_spacing_cells"].as<uint32_t>();
  if (in["cascade_count"])
    settings.cascade_count = in["cascade_count"].as<uint32_t>();
  if (in["positional_light_cascade_count"])
    settings.positional_light_cascade_count = in["positional_light_cascade_count"].as<uint32_t>();
  if (in["min_cell_size"])
    settings.min_cell_size = in["min_cell_size"].as<float>();
  if (in["vertical_scale"])
    settings.vertical_scale = static_cast<SdfgiSettings::VerticalScale>(in["vertical_scale"].as<uint32_t>());
  if (in["use_occlusion"])
    settings.use_occlusion = in["use_occlusion"].as<bool>();
  if (in["probe_relocation"])
    settings.probe_relocation = in["probe_relocation"].as<bool>();
  if (in["static_entities_only"])
    settings.static_entities_only = in["static_entities_only"].as<bool>();
  if (in["ray_count"])
    settings.ray_count = in["ray_count"].as<uint32_t>();
  if (in["history_size"])
    settings.history_size = in["history_size"].as<uint32_t>();
  if (in["light_update_frames"])
    settings.light_update_frames = in["light_update_frames"].as<uint32_t>();
  if (in["bounce_feedback"])
    settings.bounce_feedback = in["bounce_feedback"].as<float>();
  if (in["read_sky_light"])
    settings.read_sky_light = in["read_sky_light"].as<bool>();
  if (in["energy"])
    settings.energy = in["energy"].as<float>();
  if (in["normal_bias"])
    settings.normal_bias = in["normal_bias"].as<float>();
  if (in["probe_bias"])
    settings.probe_bias = in["probe_bias"].as<float>();
  if (in["anchor_camera_entity"])
    settings.anchor_camera_entity = in["anchor_camera_entity"].as<uint64_t>();
}
