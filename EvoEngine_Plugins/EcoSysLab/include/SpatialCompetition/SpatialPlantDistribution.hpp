#pragma once
#include "CellGrid.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
typedef int SpatialPlantParameterHandle;
typedef int SpatialPlantHandle;
struct SpatialPlantParameter {
  /**
   * \brief Final size of plant
   */
  float m_finalRadius = 5.f;
  /**
   * \brief Growth Rate
   */
  float m_k = 0.05f;

  float m_seedingRangeMin = 3.0f;
  float m_seedingRangeMax = 8.0f;
  float m_seedingSizeFactor = .2f;
  float m_seedInitialRadius = 1.f;
  float m_seedingPossibility = 0.001f;
  /**
   * \brief The represented color of the plant.
   */
  glm::vec4 m_color = glm::vec4(0.6f, 0.3f, 0.0f, 1.0f);
};

struct SpatialPlant {
  unsigned m_gridCellIndex = 0;
  bool m_recycled = false;
  SpatialPlantParameterHandle m_parameterHandle = 0;
  SpatialPlantHandle m_handle;
  glm::vec2 m_position = glm::vec2(0.0f);
  float m_radius = 0.0f;
  [[nodiscard]] float Overlap(const SpatialPlant& other_plant) const;
  [[nodiscard]] float SymmetricInfluence(const SpatialPlant& otherPlant) const;
  [[nodiscard]] float AsymmetricInfluence(const SpatialPlant& otherPlant) const;
  /**
   * \brief If the radii of self is larger, self gets more resources.
   * \param otherPlant
   * \param weightingFactor
   * \return
   */
  [[nodiscard]] float AsymmetricalCompetition(const SpatialPlant& otherPlant, float weightingFactor) const;
  [[nodiscard]] float GetArea() const;
  void Grow(float size);
};

struct SpatialPlantGlobalParameters {
  /**
   * \brief Weighting factor of asymmetrical competition.
   */
  float m_p = 0.5f;
  /**
   * \brief Delta of Richard growth model.
   */
  float m_delta = 2;
  /**
   * \brief Plant size factor.
   */
  float m_a = 1.f;
  float m_simulationRate = 5.f;
  float m_spawnProtectionFactor = 0.5f;

  float m_maxRadius = 300.0f;

  bool m_forceRemoveOverlap = true;

  float m_dynamicBalanceFactor = 3.0f;
};

struct SpatialPlantGridCell {
  std::vector<SpatialPlantHandle> m_plantHandles;
  void RegisterParticle(SpatialPlantHandle handle);
  void UnregisterParticle(SpatialPlantHandle handle);
};

class SpatialPlantGrid : public CellGrid<SpatialPlantGridCell> {
 public:
  [[nodiscard]] unsigned RegisterPlant(const glm::vec2& position, SpatialPlantHandle handle);
  void UnregisterPlant(unsigned cellIndex, SpatialPlantHandle handle);
  void Clear() override;
  void ForEachPlant(const glm::vec2& position, float radius,
                    const std::function<void(SpatialPlantHandle plantHandle)>& func);
};

class SpatialPlantDistribution {
  SpatialPlantGrid m_plantGrid;

 public:
  SpatialPlantDistribution();
  int m_simulationTime = 0;
  std::vector<SpatialPlantParameter> m_spatialPlantParameters{};
  std::vector<SpatialPlant> m_plants{};
  std::queue<SpatialPlantHandle> m_recycledPlants{};
  SpatialPlantGlobalParameters m_spatialPlantGlobalParameters{};
  [[nodiscard]] float CalculateGrowth(const SpatialPlantGlobalParameters& richardGrowthModelParameters,
                                      SpatialPlantHandle plantHandle,
                                      const std::vector<SpatialPlantHandle>& neighborPlantHandles) const;
  void Simulate();

  SpatialPlantHandle AddPlant(SpatialPlantParameterHandle spatialPlantParameterHandle, float radius,
                              const glm::vec2& position);
  void RecyclePlant(SpatialPlantHandle plantHandle, bool removeFromGrid = true);
};
}  // namespace eco_sys_lab