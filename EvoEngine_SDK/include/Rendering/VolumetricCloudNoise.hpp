#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace evo_engine {
struct CloudNoiseTexture {
  uint32_t width;
  bool volume;
  std::vector<uint8_t> rgba;
};

// Base shape, detail erosion, weather coverage and curl in shader binding order.
std::array<CloudNoiseTexture, 4> BuildVolumetricCloudNoise();
}  // namespace evo_engine
