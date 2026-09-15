#pragma once
#include "ShootModel.hpp"
#include "SpatialPlantDistribution.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class SpatialPlantDistributionSimulator
 * @brief Simulates the spatial distribution of plants in an ecosystem.
 *
 * This class is responsible for managing the procedural generation and simulation of tree growth within
 * an ecosystem using a spatial distribution model.
 */
class SpatialPlantDistributionSimulator : public IPrivateComponent {
 public:
  /** @brief Settings controlling tree growth simulations. */
  TreeGrowthSettings m_treeGrowthSettings;

  /** @brief A list of asset references to tree descriptors used in the simulation. */
  std::vector<AssetRef> m_treeDescriptors{};

  /** @brief The spatial plant distribution model used for simulation. */
  SpatialPlantDistribution m_distribution{};

  /** @brief Flag to indicate whether simulation is active. */
  bool m_simulate = false;

  /**
   * @brief Performs a fixed time-step update for the simulation.
   *
   * This method executes simulation logic that needs to be updated at a fixed
   * time interval rather than per frame.
   */
  void FixedUpdate() override;

  /**
   * @brief Initializes the spatial plant distribution simulator.
   *
   * This method is called when the component is created, allowing for
   * any necessary setup operations.
   */
  void OnCreate() override;
};
}  // namespace eco_sys_lab_package