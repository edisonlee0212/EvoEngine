#pragma once

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
