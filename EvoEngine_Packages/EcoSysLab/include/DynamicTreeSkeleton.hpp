#pragma once
#include "DynamicSkeleton.hpp"

namespace eco_sys_lab_package {
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
   * @brief Indicates whether the skeleton simulation is enabled.
   */
  bool simulate = false;

  /**
   * @brief Performs a physics simulation step on the dynamic skeleton.
   *
   * @param physics_parameters The parameters controlling the physics simulation step.
   */
  void PhysicsStep(const DynamicSkeleton::PhysicsParameters& physics_parameters);
};
}  // namespace eco_sys_lab_package