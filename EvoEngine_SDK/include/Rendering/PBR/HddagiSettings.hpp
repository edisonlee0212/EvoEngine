#pragma once

#include "GiProbeSettings.hpp"

namespace evo_engine {

inline constexpr const char* kHddagiReferenceCommit = "da1410fa3516d08cc31b6e86bd6673b9ce776316";

struct EVOENGINE_API HddagiSettings {
  uint32_t history_size = 12;
  uint32_t light_update_frames = 4;
  bool filter_probes = true;
  bool filter_ambient = true;
  bool filter_reflections = false;
  bool read_sky_light = true;
  bool static_entities_only = false;
  float bounce_feedback = 1.0f;
  float energy = 1.0f;
  float normal_bias = 1.1f;
  float probe_bias = 1.1f;
  float reflection_bias = 2.0f;
  float occlusion_bias = 0.1f;

  [[nodiscard]] std::string Validate(const GiProbeSettings& probes) const;
  [[nodiscard]] bool operator==(const HddagiSettings& other) const;
};

EVOENGINE_API void SerializeHddagiSettings(YAML::Emitter& out, const HddagiSettings& settings);
EVOENGINE_API void DeserializeHddagiSettings(const YAML::Node& in, HddagiSettings& settings);

}  // namespace evo_engine
