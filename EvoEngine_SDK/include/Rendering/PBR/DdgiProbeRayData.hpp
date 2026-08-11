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

struct alignas(16) DdgiProbeRaySampleInfo {
  glm::vec4 direction_and_inverse_pdf = glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
};

static_assert(std::is_standard_layout_v<DdgiProbeRayData>);
static_assert(sizeof(DdgiProbeRayData) == 16);
static_assert(alignof(DdgiProbeRayData) == 16);
static_assert(offsetof(DdgiProbeRayData, radiance_and_signed_distance) == 0);
static_assert(std::is_standard_layout_v<DdgiProbeRaySampleInfo>);
static_assert(sizeof(DdgiProbeRaySampleInfo) == 16);
static_assert(alignof(DdgiProbeRaySampleInfo) == 16);
static_assert(offsetof(DdgiProbeRaySampleInfo, direction_and_inverse_pdf) == 0);

}  // namespace evo_engine
