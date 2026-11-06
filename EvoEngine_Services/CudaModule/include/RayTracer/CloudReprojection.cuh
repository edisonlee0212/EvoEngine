#pragma once
#include <glm/glm.hpp>

namespace evo_engine {
struct CloudTexel {
  glm::vec4 light_transmittance;
  float depth;  // Distance from the camera to the representative cloud sample; zero for clear sky.
};

GLM_FUNC_QUALIFIER bool ProjectCloudUv(const glm::mat4& view_projection, glm::vec3 world, glm::vec2& uv) {
  const glm::vec4 clip = view_projection * glm::vec4(world, 1.f);
  if (clip.w <= 0.f)
    return false;
  uv = glm::vec2(clip) * (0.5f / clip.w) + 0.5f;
  // Cloud rays extend beyond the opaque camera's far plane. Only reject outside the image.
  return uv.x >= 0.f && uv.y >= 0.f && uv.x <= 1.f && uv.y <= 1.f;
}

// Sample texel centers. When sampling history, reject taps from a different cloud or clear sky.
GLM_FUNC_QUALIFIER glm::vec4 FilterCloud(const CloudTexel* image, glm::ivec2 size, glm::vec2 uv,
                                         glm::vec4 current, float expected_depth = 0.f) {
  const glm::vec2 pos = uv * glm::vec2(size) - 0.5f;
  const glm::ivec2 base(glm::floor(pos));
  const glm::vec2 f = glm::fract(pos);
  glm::vec4 total(0.f);
  float weight = 0.f;
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 2; ++x) {
      const auto p = glm::clamp(base + glm::ivec2(x, y), glm::ivec2(0), size - 1);
      const auto& tap = image[p.x + p.y * size.x];
      if (expected_depth > 0.f &&
          (tap.depth <= 0.f || glm::abs(tap.depth - expected_depth) > (glm::max)(1.f, 0.1f * expected_depth) ||
           glm::abs(tap.light_transmittance.w - current.w) > 0.35f))
        continue;
      const float w = (x ? f.x : 1.f - f.x) * (y ? f.y : 1.f - f.y);
      total += w * tap.light_transmittance;
      weight += w;
    }
  }
  return weight > 0.f ? total / weight : current;
}
}  // namespace evo_engine
