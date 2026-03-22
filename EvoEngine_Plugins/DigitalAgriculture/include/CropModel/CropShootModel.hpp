#pragma once

#include "CropDescriptor.hpp"

#include <random>

namespace digital_agriculture_plugin {
using namespace evo_engine;

#ifdef ECOSYSLAB_PLUGIN

// Forward declare legacy types for bridge methods.
class SorghumState;
class SorghumDescriptor;

// ============================================================================
// CropShootModel — developmental growth model for grass crops
// ============================================================================

/**
 * @brief Developmental growth model for monocot grass crops (maize, sorghum).
 *
 * Manages a CropSkeleton and advances it through thermal-time-driven growth:
 *  1. Plastochron-based phytomer initiation
 *  2. Leaf and internode elongation toward genotype targets
 *  3. Phenological phase transitions (emergence → growth → maturity → senescence)
 *
 * The model reads genotype parameters from a CropDescriptor and environmental
 * input (daily temperature) each timestep. Bridge methods allow converting the
 * skeleton state to legacy SorghumState / SorghumDescriptor for rendering.
 */
class CropShootModel {
 public:
  // ------------------------------------------------------------------
  // Lifecycle
  // ------------------------------------------------------------------

  /**
   * @brief Initialize the model with a genotype descriptor and random seed.
   *
   * Creates the skeleton root node and first phytomer.
   */
  void Initialize(const std::shared_ptr<CropDescriptor>& descriptor, unsigned int seed = 0);

  /**
   * @brief Reset the model to its uninitialized state.
   */
  void Clear();

  /**
   * @brief Returns true if Initialize() has been called successfully.
   */
  [[nodiscard]] bool IsInitialized() const { return initialized_; }

  // ------------------------------------------------------------------
  // Growth
  // ------------------------------------------------------------------

  /**
   * @brief Advance the model by one day (or one timestep).
   *
   * @param daily_mean_temperature Daily mean temperature (°C).
   *
   * Internally computes delta-GDD, initiates new phytomers if the plastochron
   * threshold is crossed, elongates leaves and internodes, and updates
   * phenological phases.
   */
  void Grow(float daily_mean_temperature);

  // ------------------------------------------------------------------
  // Access
  // ------------------------------------------------------------------

  [[nodiscard]] CropSkeleton& RefSkeleton() { return skeleton_; }
  [[nodiscard]] const CropSkeleton& PeekSkeleton() const { return skeleton_; }

  [[nodiscard]] int GetPhytomerCount() const;
  [[nodiscard]] float GetCumulativeGdd() const;

  // ------------------------------------------------------------------
  // Bridge to legacy rendering pipeline
  // ------------------------------------------------------------------

  /**
   * @brief Convert the current skeleton state to a SorghumState.
   *
   * Populates stem, leaf states, and panicle from the phytomer chain
   * using the genotype shape curves in the descriptor.
   */
  void ToSorghumState(const std::shared_ptr<SorghumState>& target) const;

  /**
   * @brief Convert the current skeleton state to a SorghumDescriptor.
   *
   * First generates a SorghumState, then calls its Apply() to produce
   * spline geometry in the descriptor.
   */
  void ToSorghumDescriptor(const std::shared_ptr<SorghumDescriptor>& target) const;

  // ------------------------------------------------------------------
  // Serialization
  // ------------------------------------------------------------------
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);

 private:
  bool initialized_ = false;
  CropSkeleton skeleton_;
  std::shared_ptr<CropDescriptor> descriptor_;
  std::mt19937 random_engine_;

  // ------------------------------------------------------------------
  // Internal growth helpers
  // ------------------------------------------------------------------

  /**
   * @brief Accumulate GDD and return the delta for this timestep.
   */
  float AccumulateGdd(float daily_mean_temperature);

  /**
   * @brief Initiate new phytomers if plastochron threshold is crossed.
   */
  void InitiatePhytomers(float delta_gdd);

  /**
   * @brief Create a single new phytomer at the apex of the given parent node.
   */
  eco_sys_lab_plugin::SkeletonNodeHandle CreatePhytomer(eco_sys_lab_plugin::SkeletonNodeHandle parent_handle);

  /**
   * @brief Grow all existing leaves and internodes toward their targets.
   */
  void GrowOrgans(float delta_gdd);

  /**
   * @brief Update phenological phase of individual phytomers.
   */
  void UpdatePhases();

  /**
   * @brief Recalculate plant geometry (positions, rotations) in the skeleton.
   */
  void RecalculateGeometry();
};

#endif  // ECOSYSLAB_PLUGIN

}  // namespace digital_agriculture_plugin
