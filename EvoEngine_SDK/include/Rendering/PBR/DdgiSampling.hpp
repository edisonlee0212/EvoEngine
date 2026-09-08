#pragma once

#include "DdgiHistory.hpp"

#include <glm/glm.hpp>

namespace evo_engine {

struct DdgiSamplingPhase {
  glm::vec4 rotation{0, 0, 0, 1};
  uint32_t emissive_seed = 0;
};

inline uint32_t HashDdgiSamplingSeed(uint32_t value) {
  value ^= value >> 16u;
  value *= 0x7feb352du;
  value ^= value >> 15u;
  value *= 0x846ca68bu;
  return value ^ (value >> 16u);
}

inline DdgiSamplingPhase CreateDdgiSamplingPhase(const uint64_t volume_id, const uint32_t update, const uint32_t count,
                                                 const uint32_t base_seed) {
  if (!IsDdgiHistoryCountSupported(static_cast<int>(count)))
    return {};
  const auto phase = update % count;
  const auto volume_seed = HashDdgiSamplingSeed(static_cast<uint32_t>(volume_id)) ^
                           HashDdgiSamplingSeed(static_cast<uint32_t>(volume_id >> 32u) ^ 0x85ebca6bu);
  const auto seed = base_seed ^ phase ^ (volume_seed * 0x9e3779b9u);
  const auto unit_float = [](const uint32_t value) {
    return static_cast<float>(HashDdgiSamplingSeed(value) >> 8u) * (1.0f / 16777216.0f);
  };
  constexpr float two_pi = 6.28318530718f;
  const float u1 = unit_float(seed ^ 0x68bc21ebu);
  const float u2 = unit_float(seed ^ 0x02e5be93u);
  const float u3 = unit_float(seed ^ 0x967a889bu);
  const float r1 = std::sqrt(std::max(0.0f, 1.0f - u1));
  const float r2 = std::sqrt(std::max(0.0f, u1));
  return {
      {r1 * std::sin(two_pi * u2), r1 * std::cos(two_pi * u2), r2 * std::sin(two_pi * u3), r2 * std::cos(two_pi * u3)},
      HashDdgiSamplingSeed(base_seed ^ (phase * 0x9e3779b9u) ^ (volume_seed * 0xc2b2ae35u)) & 0x7fffffffu};
}

}  // namespace evo_engine
