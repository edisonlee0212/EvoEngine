#pragma once

#include "IPrivateComponent.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class Blower : public IPrivateComponent {
 public:
  float speed = 5.0f;
  float particle_rate = 1000.0f;
  float particle_radius = 0.35f;
  float wind_strength = 1.0f;
  float temperature_c = 20.0f;
  float relative_humidity = 0.5f;
  float spread_degrees = 0.0f;
  uint32_t seed = 1;
};
}  // namespace eco_sys_lab_package
