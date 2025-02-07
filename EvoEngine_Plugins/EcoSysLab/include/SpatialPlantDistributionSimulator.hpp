
#pragma once
#include "SpatialPlantDistribution.hpp"
#include "TreeModel.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

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
   * @brief Opens an inspector interface for the spatial plant distribution model.
   *
   * This function provides an interactive GUI interface for inspecting and modifying the
   * spatial plant distribution settings.
   *
   * @param spatialPlantDistribution The spatial distribution data to inspect.
   * @param func The function to execute when a plant position is selected.
   * @param drawFunc The function to handle rendering overlays.
   */
  static void OnInspectSpatialPlantDistributionFunction(
      const SpatialPlantDistribution& spatialPlantDistribution, const std::function<void(glm::vec2 position)>& func,
      const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc);

  /**
   * @brief Called to inspect the asset in the editor.
   *
   * This method displays the inspector interface for the asset in the editor and allows
   * modifications. It returns true if the asset's content hasn't changed during inspection.
   *
   * @param editorLayer The editor environment calling this inspection.
   * @return true if no modifications were made, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

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

}  // namespace eco_sys_lab_plugin
