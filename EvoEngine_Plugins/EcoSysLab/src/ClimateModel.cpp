#include "ClimateModel.hpp"

#include "Tree.hpp"

using namespace eco_sys_lab_plugin;

float ClimateModel::GetHighTemp(const glm::vec3& position) const {
  const int month = glm::mod(time / 30.416667f, 12.f);
  const int days = glm::mod(time, 30.416667f);
  int start_index = month - 1;
  int end_index = month + 1;
  if (start_index < 0)
    start_index += 12;
  if (end_index > 11)
    end_index -= 12;

  const float avg_temp_max = monthly_max_temp_mean[month];
  float temp_max = avg_temp_max;

  if (days < 15) {
    const float start_temp_max = monthly_max_temp_mean[start_index];
    temp_max = glm::mix(start_temp_max, avg_temp_max, days / 15.0f);
  }
  if (days > 15) {
    const float end_temp_max = monthly_max_temp_mean[end_index];
    temp_max = glm::mix(avg_temp_max, end_temp_max, (days - 15) / 15.0f);
  }

  return temp_max;
}

float ClimateModel::GetLowTemp(const glm::vec3& position) const {
  const int month = glm::mod(time / 30.416667f, 12.f);
  const int days = glm::mod(time, 30.416667f);
  int start_index = month - 1;
  int end_index = month + 1;
  if (start_index < 0)
    start_index += 12;
  if (end_index > 11)
    end_index -= 12;

  const float avg_temp_min = monthly_min_temp_mean[month];
  float temp_min = avg_temp_min;
  if (days < 15) {
    const float start_temp_min = monthly_min_temp_mean[start_index];
    temp_min = glm::mix(start_temp_min, avg_temp_min, days / 15.0f);
  }
  if (days > 15) {
    const float end_temp_min = monthly_min_temp_mean[end_index];
    temp_min = glm::mix(avg_temp_min, end_temp_min, (days - 15) / 15.0f);
  }
  return temp_min;
}

float ClimateModel::GetMaxRh(const glm::vec3& position) const {
  const int month = glm::mod(time / 30.416667f, 12.f);
  const int days = glm::mod(time, 30.416667f);
  int start_index = month - 1;
  int end_index = month + 1;
  if (start_index < 0)
    start_index += 12;
  if (end_index > 11)
    end_index -= 12;

  const float avg_temp_max = monthly_max_rh_mean[month];
  float temp_max = avg_temp_max;

  if (days < 15) {
    const float start_temp_max = monthly_max_rh_mean[start_index];
    temp_max = glm::mix(start_temp_max, avg_temp_max, days / 15.0f);
  }
  if (days > 15) {
    const float end_temp_max = monthly_max_rh_mean[end_index];
    temp_max = glm::mix(avg_temp_max, end_temp_max, (days - 15) / 15.0f);
  }

  return temp_max;
}

float ClimateModel::GetMinRh(const glm::vec3& position) const {
  const int month = glm::mod(time / 30.416667f, 12.f);
  const int days = glm::mod(time, 30.416667f);

  int start_index = month - 1;
  int end_index = month + 1;
  if (start_index < 0)
    start_index += 12;
  if (end_index > 11)
    end_index -= 12;

  const float avg_temp_min = monthly_min_rh_mean[month];
  float temp_min = avg_temp_min;
  if (days < 15) {
    const float start_temp_min = monthly_min_rh_mean[start_index];
    temp_min = glm::mix(start_temp_min, avg_temp_min, days / 15.0f);
  }
  if (days > 15) {
    const float end_temp_min = monthly_min_rh_mean[end_index];
    temp_min = glm::mix(avg_temp_min, end_temp_min, (days - 15) / 15.0f);
  }
  return temp_min;
}

float ClimateModel::GetDaylightHours(const glm::vec3& position) const {
  const int month = glm::mod(time / 30.416667f, 12.f);
  const int days = glm::mod(time, 30.416667f);

  int start_index = month - 1;
  int end_index = month + 1;
  if (start_index < 0)
    start_index += 12;
  if (end_index > 11)
    end_index -= 12;

  const float avg_temp_min = monthly_daylight_hrs_mean[month];
  float temp_min = avg_temp_min;
  if (days < 15) {
    const float start_temp_min = monthly_daylight_hrs_mean[start_index];
    temp_min = glm::mix(start_temp_min, avg_temp_min, days / 15.0f);
  }
  if (days > 15) {
    const float end_temp_min = monthly_daylight_hrs_mean[end_index];
    temp_min = glm::mix(avg_temp_min, end_temp_min, (days - 15) / 15.0f);
  }
  return temp_min;
}

float ClimateModel::GetEnvironmentalLight(const glm::vec3& position, glm::vec3& light_direction) const {
  return environment_grid.Sample(position, light_direction);
}

float ClimateModel::GetTimeInYear() const {
  return glm::mod(time, 365.f) / 365.f;
}

void ClimateModel::Initialize(const ClimateParameters& climate_parameters) {
  time = 0;
}