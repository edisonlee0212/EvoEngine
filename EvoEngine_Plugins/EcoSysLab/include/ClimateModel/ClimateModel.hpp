#pragma once
#include "EnvironmentGrid.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief Struct representing parameters for configuring the climate model.
 */
struct ClimateParameters {};

/**
 * @brief Class for simulating climate conditions in an environment grid.
 */
class ClimateModel {
 public:
  /**
   * @brief Current simulation time in days.
   */
  float time = 0.0f;

  /**
   * @brief Calendar offset in days so that simulation day 0 corresponds to spring.
   * Default is March 1 (~60.8 days into the year).
   */
  static constexpr float spring_start_offset = 2.f * (365.f / 12.f);

  float monthly_max_temp_mean[12] = {2, 5, 10, 16, 22, 27, 30, 29, 24, 17, 9, 4};
  float monthly_min_temp_mean[12] = {-5, -3, 1, 6, 11, 16, 18, 17, 13, 7, 1, -2};
  float monthly_max_rh_mean[12] = {85, 80, 75, 70, 70, 65, 65, 70, 75, 80, 85, 90};
  float monthly_min_rh_mean[12] = {60, 55, 50, 45, 45, 40, 40, 45, 50, 55, 60, 65};
  float monthly_daylight_hrs_mean[12] = {9.5f, 10.5f, 12.f, 13.5f, 14.5f, 15.f, 14.8f, 14.f, 12.5f, 11.f, 10.f, 9.5f};

  /**
   * @brief Grid structure representing environmental data.
   */
  EnvironmentGrid environment_grid{};

  /**
   * @brief Computes the high temperature of current date at a given position.
   * @param position The 3D position to query.
   * @return The temperature at the queried position.
   */
  [[nodiscard]] float GetHighTemp(const glm::vec3& position) const;

  /**
   * @brief Computes the low temperature of current date at a given position.
   * @param position The 3D position to query.
   * @return The temperature at the queried position.
   */
  [[nodiscard]] float GetLowTemp(const glm::vec3& position) const;

 private:
  /**
   * @brief Continuously interpolates a 12-element monthly array at the current time.
   *
   * Each monthly value is treated as the value at the midpoint of that month.
   * Linear interpolation between adjacent midpoints ensures a smooth, continuous
   * curve with no discontinuities at month boundaries.
   */
  [[nodiscard]] float InterpolateMonthly(const float* monthly_values) const;

 public:

  /**
   * \brief Samples the max relative humidity of current date at given position.
   * \param position The 3D position to query.
   * \return The max relative humidity at the queried position.
   */
  [[nodiscard]] float GetMaxRh(const glm::vec3& position) const;
  /**
   * \brief Samples the min relative humidity of current date at given position.
   * \param position The 3D position to query.
   * \return The min relative humidity at the queried position.
   */
  [[nodiscard]] float GetMinRh(const glm::vec3& position) const;
  /**
   * \brief Samples the daylight hours of current date at given position.
   * \param position The 3D position to query.
   * \return The daylight hours at the queried position.
   */
  [[nodiscard]] float GetDaylightHours(const glm::vec3& position) const;
  /**
   * @brief Computes environmental lighting conditions at a given position.
   * @param position The 3D position to query.
   * @param light_direction Outputs the direction of environmental light.
   * @return The intensity of environmental light at the queried position.
   */
  [[nodiscard]] float GetEnvironmentalLight(const glm::vec3& position, glm::vec3& light_direction) const;

  [[nodiscard]] float GetTimeInYear() const;
  /**
   * @brief Initializes the climate model with the given parameters.
   * @param climate_parameters The parameters defining the climate conditions.
   */
  void Initialize(const ClimateParameters& climate_parameters);
};
}  // namespace eco_sys_lab_plugin