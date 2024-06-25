#include "ClimateModel.hpp"

#include "Tree.hpp"

using namespace eco_sys_lab;

float ClimateModel::GetTemperature(const glm::vec3& position) const {
  int month = static_cast<int>(time * 365) / 30 % 12;
  int days = static_cast<int>(time * 365) % 30;

  int start_index = month - 1;
  int end_index = month + 1;
  if (start_index < 0)
    start_index += 12;
  if (end_index > 11)
    end_index -= 12;
  const float start_temp = month_avg_temp[start_index];
  const float avg_temp = month_avg_temp[month];
  const float end_temp = month_avg_temp[end_index];
  if (days < 15) {
    return glm::mix(start_temp, avg_temp, days / 15.0f);
  }
  if (days > 15) {
    return glm::mix(avg_temp, end_temp, (days - 15) / 15.0f);
  }
  return avg_temp;
}

float ClimateModel::GetEnvironmentalLight(const glm::vec3& position, glm::vec3& light_direction) const {
  return environment_grid.Sample(position, light_direction);
}

void ClimateModel::Initialize(const ClimateParameters& climate_parameters) {
  time = 0;
}
