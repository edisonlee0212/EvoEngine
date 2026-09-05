#pragma once

#include <cstdint>
#include <string>

namespace YAML {
class Emitter;
class Node;
}  // namespace YAML

namespace evo_engine {

enum class IndirectGiProvider : uint32_t { Environment = 0, AuthoredDdgi = 1, AutomaticSdfgi = 2 };
EVOENGINE_API const char* GetIndirectGiProviderName(IndirectGiProvider provider);

struct EVOENGINE_API SdfgiSettings {
  enum class VerticalScale : uint32_t { Percent50 = 0, Percent75 = 1, Percent100 = 2 };

  static constexpr uint32_t kCascadeSize = 128;
  uint32_t cascade_count = 4;
  float min_cell_size = 0.2f;
  VerticalScale vertical_scale = VerticalScale::Percent75;
  bool use_occlusion = false;
  uint32_t ray_count = 16;
  uint32_t history_size = 30;
  uint32_t light_update_frames = 4;
  float bounce_feedback = 0.5f;
  bool read_sky_light = true;
  float energy = 1.0f;
  float normal_bias = 1.1f;
  float probe_bias = 1.1f;
  uint64_t anchor_camera_entity = 0;

  [[nodiscard]] std::string Validate() const;
  [[nodiscard]] bool HasSameLayout(const SdfgiSettings& other) const;
  [[nodiscard]] bool operator==(const SdfgiSettings& other) const;
};

EVOENGINE_API void SerializeSdfgiSettings(YAML::Emitter& out, const SdfgiSettings& settings);
EVOENGINE_API void DeserializeSdfgiSettings(const YAML::Node& in, SdfgiSettings& settings);

}  // namespace evo_engine
