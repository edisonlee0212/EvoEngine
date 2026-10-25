#pragma once

#include <cmath>

#include "ClimateModel.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class ClimateDescriptor
 * @brief Represents a descriptor for climate parameters, which can be serialized and inspected in the editor.
 */
class ClimateDescriptor : public IAsset {
 public:
  /// The parameters defining the climate conditions.
  ClimateParameters climate_parameters;
};

/**
 * @class Climate
 * @brief Represents a climate system with a model and a reference to a climate descriptor.
 */
class Climate : public IPrivateComponent {
 public:
  static constexpr float default_air_cell_size = 0.1f;
  float air_temperature_c = 20.0f;
  float air_relative_humidity = 0.5f;
  float air_pressure_pa = 101325.0f;
  float air_coupling_rate = 4.0f;
  float air_friction = 4.0f;
  float air_cell_size = default_air_cell_size;

  [[nodiscard]] static float SpecificHumidity(float temperature_c, float relative_humidity, float pressure_pa) {
    const float saturation = 610.94f * std::exp(17.625f * temperature_c / (temperature_c + 243.04f));
    const float vapor = glm::min(glm::clamp(relative_humidity, 0.0f, 1.0f) * saturation, pressure_pa * 0.99f);
    return 0.622f * vapor / glm::max(pressure_pa - 0.378f * vapor, 1.0f);
  }
  [[nodiscard]] float AirSpecificHumidity() const {
    return SpecificHumidity(air_temperature_c, air_relative_humidity, air_pressure_pa);
  }

  /// The climate model used for simulation.
  ClimateModel climate_model;

  /// Reference to the associated climate descriptor asset.
  AssetRef climate_descriptor_ref;

  /**
   * @brief Collects asset references used by this component.
   * @param list The list to store asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Initializes the climate model with relevant parameters.
   */
  void InitializeClimateModel();

  /**
   * @brief Prepares the climate system for the growth phase.
   */
  void PrepareForGrowth();
};
}  // namespace eco_sys_lab_package
