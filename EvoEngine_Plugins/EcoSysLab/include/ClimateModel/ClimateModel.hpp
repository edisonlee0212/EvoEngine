#pragma once
#include "EnvironmentGrid.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
struct ClimateParameters {};
class ClimateModel {
 public:
  float month_avg_temp[12] = {38, 42, 46, 54, 61, 68, 77, 83, 77, 67, 55, 43};
  float time = 0.0f;
  EnvironmentGrid environment_grid{};
  [[nodiscard]] float GetTemperature(const glm::vec3& position) const;
  [[nodiscard]] float GetEnvironmentalLight(const glm::vec3& position, glm::vec3& light_direction) const;

  void Initialize(const ClimateParameters& climate_parameters);
};
}  // namespace eco_sys_lab
