
#pragma once
#include "DynamicSkeleton.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class DynamicTreeSkeleton
 * @brief Represents a procedural tree skeleton with dynamic simulation capabilities.
 *
 * This class is responsible for managing a dynamic tree skeleton, allowing for physics-based
 * simulation and visualization of tree structures.
 */
class DynamicTreeSkeleton : public IPrivateComponent {
 public:
  /**
   * @brief Represents the dynamic skeleton structure of the tree.
   */
  DynamicSkeleton dynamic_skeleton{};

  /**
   * @brief Parameters used for initializing the dynamic skeleton.
   */
  DynamicSkeleton::InitializeParameters initialize_parameters{};

  /**
   * @brief Stores debug matrices for visualization purposes.
   */
  std::shared_ptr<ParticleInfoList> debug_matrices;

  /**
   * @brief Indicates whether the skeleton simulation is enabled.
   */
  bool simulate = false;

  /**
   * @brief Called when the object is inspected in the editor.
   *
   * @param editor_layer The current editor layer.
   * @return Returns true if the asset's content is not modified during inspection.
   *
   * This function allows the user to inspect and modify various dynamic skeleton parameters.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Called during the late update phase of the application loop.
   *
   * Performs any necessary updates to the dynamic skeleton after the main update phase.
   */
  void LateUpdate() override;

  /**
   * @brief Called when the object is created.
   *
   * Used to initialize the tree skeleton and any required resources.
   */
  void OnCreate() override;

  /**
   * @brief Performs a physics simulation step on the dynamic skeleton.
   *
   * @param physics_parameters The parameters controlling the physics simulation step.
   */
  void PhysicsStep(const DynamicSkeleton::PhysicsParameters& physics_parameters);

  /**
   * @brief Renders the visualization of the dynamic tree skeleton.
   *
   * @param target_camera The camera used for rendering.
   * @param visualization_parameters The parameters controlling visualization appearance.
   */
  void Visualization(const std::shared_ptr<Camera>& target_camera,
                     const DynamicSkeleton::VisualizationParameters& visualization_parameters) const;
};

}  // namespace eco_sys_lab_plugin
