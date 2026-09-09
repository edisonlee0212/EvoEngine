// Defaults from Godot environment.h / rendering_server.cpp, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt.
#include "HddagiSettings.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>
#include <type_traits>

using namespace evo_engine;

std::string HddagiSettings::Validate(const GiProbeSettings& probes) const {
  if (const auto error = probes.Validate(); !error.empty())
    return error;
  for (const auto count : {probes.probe_count_x, probes.probe_count_y})
    if (count < 9 || count > 33 || ((count - 1) * 8) % 16 != 0)
      return "HDDAGI requires 64..256 voxels per axis in multiples of 16 at eight cells per probe.";
  constexpr std::array histories{6u, 12u, 18u, 24u, 32u};
  constexpr std::array intervals{1u, 2u, 4u, 8u, 16u};
  if (std::find(histories.begin(), histories.end(), history_size) == histories.end())
    return "HDDAGI history must be 6, 12, 18, 24 or 32 updates.";
  if (std::find(intervals.begin(), intervals.end(), light_update_frames) == intervals.end())
    return "HDDAGI light interval must be 1, 2, 4, 8 or 16 frames.";
  for (const auto value : {bounce_feedback, energy, normal_bias, probe_bias, reflection_bias})
    if (!std::isfinite(value) || value < 0)
      return "HDDAGI lighting parameters must be finite and nonnegative.";
  return {};
}

namespace {
template <typename F>
void Fields(HddagiSettings& s, F&& f) {
  f("history_size", s.history_size);
  f("light_update_frames", s.light_update_frames);
  f("filter_probes", s.filter_probes);
  f("filter_ambient", s.filter_ambient);
  f("filter_reflections", s.filter_reflections);
  f("read_sky_light", s.read_sky_light);
  f("static_entities_only", s.static_entities_only);
  f("bounce_feedback", s.bounce_feedback);
  f("energy", s.energy);
  f("normal_bias", s.normal_bias);
  f("probe_bias", s.probe_bias);
  f("reflection_bias", s.reflection_bias);
  f("use_occlusion", s.use_occlusion);
}
}  // namespace

void evo_engine::SerializeHddagiSettings(YAML::Emitter& out, const HddagiSettings& settings) {
  auto copy = settings;
  out << YAML::BeginMap;
  Fields(copy, [&](const char* name, auto& value) {
    out << YAML::Key << name << YAML::Value << value;
  });
  out << YAML::EndMap;
}

void evo_engine::DeserializeHddagiSettings(const YAML::Node& in, HddagiSettings& settings) {
  if (!in["use_occlusion"] && in["occlusion_bias"])
    settings.use_occlusion = in["occlusion_bias"].as<float>() < 1.0f;
  Fields(settings, [&](const char* name, auto& value) {
    if (const auto node = in[name])
      value = node.as<std::decay_t<decltype(value)>>();
  });
}

bool HddagiSettings::operator==(const HddagiSettings& other) const {
  return std::tie(history_size, light_update_frames, filter_probes, filter_ambient, filter_reflections, read_sky_light,
                  static_entities_only, bounce_feedback, energy, normal_bias, probe_bias, reflection_bias,
                  use_occlusion) ==
         std::tie(other.history_size, other.light_update_frames, other.filter_probes, other.filter_ambient,
                  other.filter_reflections, other.read_sky_light, other.static_entities_only, other.bounce_feedback,
                  other.energy, other.normal_bias, other.probe_bias, other.reflection_bias, other.use_occlusion);
}
