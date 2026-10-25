#pragma once

#include <cmath>

#include "IPrivateComponent.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class AirEnvironment : public IPrivateComponent {
 public:
  float temperature_c = 20.0f;
  float relative_humidity = 0.5f;
  float pressure_pa = 101325.0f;
  float coupling_rate = 4.0f;
  float friction = 4.0f;

  [[nodiscard]] static float SpecificHumidity(float temperature_c, float relative_humidity, float pressure_pa) {
    const float saturation = 610.94f * std::exp(17.625f * temperature_c / (temperature_c + 243.04f));
    const float vapor = glm::min(glm::clamp(relative_humidity, 0.0f, 1.0f) * saturation, pressure_pa * 0.99f);
    return 0.622f * vapor / glm::max(pressure_pa - 0.378f * vapor, 1.0f);
  }
  [[nodiscard]] float SpecificHumidity() const {
    return SpecificHumidity(temperature_c, relative_humidity, pressure_pa);
  }
};
}  // namespace eco_sys_lab_package
