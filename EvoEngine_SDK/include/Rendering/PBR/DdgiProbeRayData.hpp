#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <type_traits>

namespace evo_engine {

inline constexpr float kDdgiProbeRayMissDistance = 1e27f;
inline constexpr float kDdgiProbeRayInactiveDistance = -1e27f;
inline constexpr float kDdgiProbeRayBackfaceDistanceScale = -0.2f;

struct alignas(16) DdgiProbeRayData {
  glm::vec4 radiance_and_signed_distance = glm::vec4(0.0f);
};

struct alignas(8) DdgiProbeRaySampleInfo {
  glm::uvec2 packed_direction_and_inverse_pdf = glm::uvec2(0u);
};

static_assert(std::is_standard_layout_v<DdgiProbeRayData>);
static_assert(sizeof(DdgiProbeRayData) == 16);
static_assert(alignof(DdgiProbeRayData) == 16);
static_assert(offsetof(DdgiProbeRayData, radiance_and_signed_distance) == 0);
static_assert(std::is_standard_layout_v<DdgiProbeRaySampleInfo>);
static_assert(sizeof(DdgiProbeRaySampleInfo) == 8);
static_assert(alignof(DdgiProbeRaySampleInfo) == 8);
static_assert(offsetof(DdgiProbeRaySampleInfo, packed_direction_and_inverse_pdf) == 0);

}  // namespace evo_engine
