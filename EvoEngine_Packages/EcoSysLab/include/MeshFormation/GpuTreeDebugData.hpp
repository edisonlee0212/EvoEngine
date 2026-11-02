#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <glm/glm.hpp>
#include <optional>

namespace eco_sys_lab_package {
struct GpuTreeDebugState {
  glm::mat4 model{1.0f};
  glm::mat4 projection_view{1.0f};
  glm::uvec4 selected{UINT32_MAX};
  glm::vec4 selection{};
};
static_assert(sizeof(GpuTreeDebugState) == 160);

struct GpuTreeDebugPick {
  uint32_t node;
  bool shoot;
  float length_factor;
};

inline std::optional<GpuTreeDebugPick> DecodeGpuTreeDebugPick(const glm::uvec4& pixel, size_t shoot_nodes,
                                                              size_t root_nodes) {
  if (!pixel.x || pixel.y > 1 || pixel.x - 1 >= (pixel.y ? shoot_nodes : root_nodes))
    return {};
  float factor;
  std::memcpy(&factor, &pixel.z, sizeof(factor));
  if (!std::isfinite(factor) || factor < 0.0f || factor > 1.0f)
    return {};
  return GpuTreeDebugPick{pixel.x - 1, pixel.y != 0, factor};
}
}  // namespace eco_sys_lab_package
