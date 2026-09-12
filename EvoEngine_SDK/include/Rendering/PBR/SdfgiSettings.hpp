#pragma once

#include "GiProbeSettings.hpp"

#include <cstdint>
#include <glm/vec3.hpp>
#include <string>

namespace YAML {
class Emitter;
class Node;
}  // namespace YAML

namespace evo_engine {

enum class IndirectGiProvider : uint32_t {
  Environment = 0,
  AutomaticDdgi = 1,
  AutomaticSdfgi = 2,
  AutomaticHddagi = 3
};
EVOENGINE_API const char* GetIndirectGiProviderName(IndirectGiProvider provider);

struct EVOENGINE_API SdfgiSettings {
  using VerticalScale = GiProbeSettings::VerticalScale;

  uint32_t voxel_count_x = 256;
  uint32_t voxel_count_y = 128;
  uint32_t probe_spacing_cells = 8;
  uint32_t cascade_count = 4;
  uint32_t positional_light_cascade_count = 8;
  float min_cell_size = 0.1f;
  VerticalScale vertical_scale = VerticalScale::Percent100;
  bool use_occlusion = true;
  bool static_entities_only = false;
  uint32_t ray_count = 16;
  uint32_t history_size = 30;
  uint32_t light_update_frames = 4;
  float bounce_feedback = 1.0f;
  bool read_sky_light = true;
  float energy = 1.0f;
  float normal_bias = 1.1f;
  float probe_bias = 1.1f;
  uint64_t anchor_camera_entity = 0;

  [[nodiscard]] float GetCascade0Distance() const;
  void SetCascade0Distance(float distance);
  [[nodiscard]] float GetMaxDistance() const;
  void SetMaxDistance(float distance);

  [[nodiscard]] glm::ivec3 GridSize() const {
    return {voxel_count_x, voxel_count_y, voxel_count_x};
  }
  [[nodiscard]] glm::ivec3 ProbeSize() const {
    return probe_spacing_cells ? GridSize() / static_cast<int>(probe_spacing_cells) + 1 : glm::ivec3(0);
  }
  [[nodiscard]] uint32_t SolidCellCapacity() const {
    const auto size = GridSize();
    return size.x * size.y * size.z / 4;
  }
  [[nodiscard]] std::string Validate() const;
  [[nodiscard]] bool HasSameLayout(const SdfgiSettings& other) const;
  [[nodiscard]] bool operator==(const SdfgiSettings& other) const;
};

EVOENGINE_API void SerializeSdfgiSettings(YAML::Emitter& out, const SdfgiSettings& settings);
EVOENGINE_API void DeserializeSdfgiSettings(const YAML::Node& in, SdfgiSettings& settings);

}  // namespace evo_engine
