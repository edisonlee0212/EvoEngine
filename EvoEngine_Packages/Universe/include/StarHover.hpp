#pragma once

#include <cstddef>
#include <cstdint>
#include <glm/vec2.hpp>

namespace universe_package {
struct StarHoverPushConstant {
  int32_t camera_index = 0;
  float brightness_limit = 1;
  glm::vec2 viewport_size{1};
  glm::vec2 display_size{1};
  float minimum_radius = 3;
  uint32_t star_index = 0;
};
static_assert(sizeof(StarHoverPushConstant) == 32);
static_assert(offsetof(StarHoverPushConstant, display_size) == 16);
static_assert(offsetof(StarHoverPushConstant, star_index) == 28);
}  // namespace universe_package
