#pragma once

#include <glm/gtc/noise.hpp>
#include "IPrivateComponent.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class Blower : public IPrivateComponent {
 public:
  float speed = 5.0f;
  bool speed_noise = false;
  float speed_max = 200.0f;
  float speed_noise_frequency = 0.2f;
  [[nodiscard]] float SampleSpeed(float simulation_time) const {
    const float minimum = glm::max(0.0f, speed);
    if (!speed_noise)
      return minimum;
    const float noise = glm::perlin(
        glm::vec2(simulation_time * glm::max(0.0f, speed_noise_frequency), static_cast<float>(seed % 65536u) + 0.5f));
    return glm::mix(minimum, glm::max(minimum, speed_max), glm::clamp(0.5f + 0.5f * noise, 0.0f, 1.0f));
  }
  float particle_rate = 1000.0f;
  float particle_radius = 0.35f;
  float wind_strength = 1.0f;
  float temperature_c = 20.0f;
  float relative_humidity = 0.5f;
  float spread_degrees = 0.0f;
  uint32_t seed = 1;
};
}  // namespace eco_sys_lab_package
