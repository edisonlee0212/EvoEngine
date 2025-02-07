
#pragma once
#include "EnvironmentGrid.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {

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
   * @brief Average monthly temperatures (in Fahrenheit).
   */
  float month_avg_temp[12] = {38, 42, 46, 54, 61, 68, 77, 83, 77, 67, 55, 43};

  /**
   * @brief Current simulation time in arbitrary units.
   */
  float time = 0.0f;

  /**
   * @brief Grid structure representing environmental data.
   */
  EnvironmentGrid environment_grid{};

  /**
   * @brief Computes the temperature at a given position.
   * @param position The 3D position to query.
   * @return The temperature at the queried position.
   */
  [[nodiscard]] float GetTemperature(const glm::vec3& position) const;

  /**
   * @brief Computes environmental lighting conditions at a given position.
   * @param position The 3D position to query.
   * @param light_direction Outputs the direction of environmental light.
   * @return The intensity of environmental light at the queried position.
   */
  [[nodiscard]] float GetEnvironmentalLight(const glm::vec3& position, glm::vec3& light_direction) const;

  /**
   * @brief Initializes the climate model with the given parameters.
   * @param climate_parameters The parameters defining the climate conditions.
   */
  void Initialize(const ClimateParameters& climate_parameters);
};

}  // namespace eco_sys_lab_plugin
