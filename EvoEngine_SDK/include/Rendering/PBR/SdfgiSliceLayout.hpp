#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace evo_engine {

// Packed X/Y/Z slice readbacks; presentation keeps the existing axis orientation.
struct SdfgiSliceLayout {
  glm::ivec3 grid{128};
  [[nodiscard]] uint32_t Count(uint32_t axis) const {
    return axis == 0 ? grid.y * grid.z : axis == 1 ? grid.x * grid.z : grid.x * grid.y;
  }
  [[nodiscard]] uint32_t Offset(uint32_t axis) const {
    return axis == 0 ? 0 : axis == 1 ? Count(0) : Count(0) + Count(1);
  }
  [[nodiscard]] uint32_t Total() const {
    return Offset(2) + Count(2);
  }
  [[nodiscard]] uint32_t Pixel(uint32_t axis, uint32_t u, uint32_t v, uint32_t display_size) const {
    const uint32_t width = axis == 0 ? grid.y : axis == 1 ? grid.z : grid.x;
    const uint32_t height = axis == 0 ? grid.z : axis == 1 ? grid.x : grid.y;
    const uint32_t right = u * width / display_size, up = height - 1 - v * height / display_size;
    return axis == 1 ? up + right * grid.x : right + up * width;
  }
};

}  // namespace evo_engine
