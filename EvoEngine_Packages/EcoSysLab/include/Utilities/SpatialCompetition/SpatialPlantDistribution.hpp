#pragma once
#include "CellGrid.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/// Represents a handle to a spatial plant parameter.
typedef int SpatialPlantParameterHandle;

/// Represents a handle to a spatial plant.
typedef int SpatialPlantHandle;

/**
 * \brief Holds parameters defining the growth and seeding behavior of a spatial plant.
 */
struct SpatialPlantParameter {
  /**
   * \brief Final size of plant.
   */
  float m_finalRadius = 5.f;

  /**
   * \brief Growth rate of the plant.
   */
  float m_k = 0.05f;

  /// Minimum seeding range.
  float m_seedingRangeMin = 3.0f;

  /// Maximum seeding range.
  float m_seedingRangeMax = 8.0f;

  /// Factor influencing seeding size.
  float m_seedingSizeFactor = .2f;

  /// Initial radius of a plant seed.
  float m_seedInitialRadius = 1.f;

  /// Probability of seeding.
  float m_seedingPossibility = 0.001f;

  /**
   * \brief The represented color of the plant.
   */
  glm::vec4 m_color = glm::vec4(0.6f, 0.3f, 0.0f, 1.0f);
};

/**
 * \brief Represents an individual spatial plant, including its position, size, and associated parameters.
 */
struct SpatialPlant {
  /// Index of the plant within the grid cell.
  unsigned m_gridCellIndex = 0;

  /// Indicates whether this plant has been recycled.
  bool m_recycled = false;

  /// Handle referencing the plant's parameters.
  SpatialPlantParameterHandle m_parameterHandle = 0;

  /// Handle referencing this plant.
  SpatialPlantHandle m_handle;

  /// Position of the plant in a 2D plane.
  glm::vec2 m_position = glm::vec2(0.0f);

  /// Radius of the plant.
  float m_radius = 0.0f;

  /**
   * \brief Computes the overlap between this plant and another plant.
   * \param other_plant The other plant.
   * \return The overlap amount.
   */
  [[nodiscard]] float Overlap(const SpatialPlant& other_plant) const;

  /**
   * \brief Computes the symmetric influence exerted by another plant.
   * \param otherPlant The other plant.
   * \return The symmetric influence value.
   */
  [[nodiscard]] float SymmetricInfluence(const SpatialPlant& otherPlant) const;

  /**
   * \brief Computes the asymmetric influence exerted by another plant.
   * \param otherPlant The other plant.
   * \return The asymmetric influence value.
   */
  [[nodiscard]] float AsymmetricInfluence(const SpatialPlant& otherPlant) const;

  /**
   * \brief Computes the asymmetrical competition factor.
   * \details If the radius of this plant is larger, it gains more resources.
   * \param otherPlant The competing plant.
   * \param weightingFactor Weighing factor for competition distribution.
   * \return Competition influence value.
   */
  [[nodiscard]] float AsymmetricalCompetition(const SpatialPlant& otherPlant, float weightingFactor) const;

  /**
   * \brief Computes the area occupied by the plant.
   * \return The area covered by the plant.
   */
  [[nodiscard]] float GetArea() const;

  /**
   * \brief Increases the size of the plant.
   * \param size The amount to grow the plant by.
   */
  void Grow(float size);
};

/**
 * \brief Holds global parameters influencing spatial plant growth simulations.
 */
struct SpatialPlantGlobalParameters {
  /**
   * \brief Weighting factor for asymmetrical competition.
   */
  float m_p = 0.5f;

  /**
   * \brief Delta factor for Richard's growth model.
   */
  float m_delta = 2;

  /**
   * \brief Factor defining the plant size.
   */
  float m_a = 1.f;

  /// Rate of simulation execution.
  float m_simulationRate = 5.f;

  /// Factor affecting spawn protection.
  float m_spawnProtectionFactor = 0.5f;

  /// Maximum allowed plant radius.
  float m_maxRadius = 300.0f;

  /// If true, forces removal of overlapping plants.
  bool m_forceRemoveOverlap = true;

  /// Factor influencing dynamic balance in the simulation.
  float m_dynamicBalanceFactor = 3.0f;
};

/**
 * \brief Represents a grid cell containing spatial plants.
 */
struct SpatialPlantGridCell {
  /// List of plant handles registered in this cell.
  std::vector<SpatialPlantHandle> m_plantHandles;

  /**
   * \brief Registers a plant handle in this grid cell.
   * \param handle The handle referencing the plant to register.
   */
  void RegisterParticle(SpatialPlantHandle handle);

  /**
   * \brief Unregisters a plant handle from this grid cell.
   * \param handle The handle referencing the plant to unregister.
   */
  void UnregisterParticle(SpatialPlantHandle handle);
};

/**
 * \brief A spatial grid structure managing the distribution of plants.
 */
class SpatialPlantGrid : public CellGrid<SpatialPlantGridCell> {
 public:
  /**
   * \brief Registers a new plant within the grid.
   * \param position The position of the plant.
   * \param handle The handle of the plant to register.
   * \return The index of the cell where the plant was placed.
   */
  [[nodiscard]] unsigned RegisterPlant(const glm::vec2& position, SpatialPlantHandle handle);

  /**
   * \brief Unregisters a plant from the grid.
   * \param cellIndex Index of the cell containing the plant.
   * \param handle The handle of the plant to unregister.
   */
  void UnregisterPlant(unsigned cellIndex, SpatialPlantHandle handle);

  /**
   * \brief Clears all registered plants from the grid.
   */
  void Clear() override;

  /**
   * \brief Iterates over plants within a given radius and executes a function on each.
   * \param position The center position of the search area.
   * \param radius The radius of the search area.
   * \param func The function to execute for each plant.
   */
  void ForEachPlant(const glm::vec2& position, float radius,
                    const std::function<void(SpatialPlantHandle plantHandle)>& func);
};

/**
 * \brief Handles spatial plant distribution and simulation.
 */
class SpatialPlantDistribution {
  /// Grid managing spatial plant positions.
  SpatialPlantGrid m_plantGrid;

 public:
  /// Constructs a new spatial plant distribution.
  SpatialPlantDistribution();

  /// Tracks the current simulation time step.
  int m_simulationTime = 0;

  /// List of parameter sets for different spatial plants.
  std::vector<SpatialPlantParameter> m_spatialPlantParameters{};

  /// List of all spatial plants currently active.
  std::vector<SpatialPlant> m_plants{};

  /// Queue of recycled plant handles for reuse.
  std::queue<SpatialPlantHandle> m_recycledPlants{};

  /// Global parameters influencing plant growth simulation.
  SpatialPlantGlobalParameters m_spatialPlantGlobalParameters{};

  /**
   * \brief Computes the growth of a plant based on nearby plants.
   * \param richardGrowthModelParameters The parameters governing the growth model.
   * \param plantHandle Handle to the plant being evaluated.
   * \param neighborPlantHandles List of neighboring plants.
   * \return Growth factor of the plant.
   */
  [[nodiscard]] float CalculateGrowth(const SpatialPlantGlobalParameters& richardGrowthModelParameters,
                                      SpatialPlantHandle plantHandle,
                                      const std::vector<SpatialPlantHandle>& neighborPlantHandles) const;

  /**
   * \brief Simulates plant growth over time.
   */
  void Simulate();

  /**
   * \brief Adds a new plant to the distribution.
   * \param spatialPlantParameterHandle Handle referencing plant parameters.
   * \param radius Initial radius of the plant.
   * \param position Position where the plant is placed.
   * \return Handle of the new plant.
   */
  SpatialPlantHandle AddPlant(SpatialPlantParameterHandle spatialPlantParameterHandle, float radius,
                              const glm::vec2& position);

  /**
   * \brief Recycles a plant, removing or reusing it.
   * \param plantHandle Handle referring to the plant.
   * \param removeFromGrid If true, removes the plant from the grid.
   */
  void RecyclePlant(SpatialPlantHandle plantHandle, bool removeFromGrid = true);
};
}  // namespace eco_sys_lab_package