#include "ClimateModel.hpp"

#include "Tree.hpp"

using namespace eco_sys_lab_plugin;

float ClimateModel::InterpolateMonthly(const float* monthly_values) const {
  constexpr float days_per_month = 365.f / 12.f;  // 30.416667
  const float calendar_day = glm::mod(time + spring_start_offset, 365.f);
  // Each monthly value sits at the midpoint of its month.
  // Shift so that month-0's midpoint maps to 0, then find the two bracketing months.
  const float shifted = calendar_day - 0.5f * days_per_month;
  const float phase = shifted / days_per_month;
  const int month_a_raw = static_cast<int>(glm::floor(phase));
  const float t = phase - static_cast<float>(month_a_raw);
  const int month_a = ((month_a_raw % 12) + 12) % 12;
  const int month_b = (month_a + 1) % 12;
  return glm::mix(monthly_values[month_a], monthly_values[month_b], t);
}

float ClimateModel::GetHighTemp(const glm::vec3& position) const {
  return InterpolateMonthly(monthly_max_temp_mean);
}

float ClimateModel::GetLowTemp(const glm::vec3& position) const {
  return InterpolateMonthly(monthly_min_temp_mean);
}

float ClimateModel::GetMaxRh(const glm::vec3& position) const {
  return InterpolateMonthly(monthly_max_rh_mean);
}

float ClimateModel::GetMinRh(const glm::vec3& position) const {
  return InterpolateMonthly(monthly_min_rh_mean);
}

float ClimateModel::GetDaylightHours(const glm::vec3& position) const {
  return InterpolateMonthly(monthly_daylight_hrs_mean);
}

float ClimateModel::GetEnvironmentalLight(const glm::vec3& position, glm::vec3& light_direction) const {
  return environment_grid.Sample(position, light_direction);
}

float ClimateModel::GetTimeInYear() const {
  return glm::mod(time + spring_start_offset, 365.f) / 365.f;
}

void ClimateModel::Initialize(const ClimateParameters& climate_parameters) {
  time = 0;
}
