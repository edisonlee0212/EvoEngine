#pragma once

#include "HddagiResources.hpp"
#include "HddagiTypes.hpp"

namespace evo_engine {
struct alignas(16) HddagiGatherData {
  HddagiCascadeBlock cascades;
  glm::ivec3 grid{0};
  uint32_t cascade_count = 0;
  glm::ivec3 probe_size{0};
  float energy = 1;
  glm::vec3 anchor_origin{0};
  float y_mult = 1;
  float normal_bias = 1.1f;
  float reflection_bias = 2;
  uint32_t use_occlusion = 1;
  uint32_t blend_ambient = 1;
};
static_assert(sizeof(HddagiGatherData) == 320);

struct HddagiCameraLayout {
  glm::uvec2 viewport{1};
  glm::uvec2 gi{1};
  uint32_t pixel_stride = 1;
  uint32_t reflection_filter_radius = 12;
};

EVOENGINE_API HddagiCameraLayout BuildHddagiCameraLayout(glm::uvec2 viewport);
EVOENGINE_API HddagiGatherData BuildHddagiGatherData(const GiProbeSettings& probes, const HddagiSettings& settings,
                                                     const std::vector<SdfgiCascade>& cascades, glm::vec3 anchor);
}  // namespace evo_engine
