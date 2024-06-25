#include "SpatialPlantDistribution.hpp"

using namespace eco_sys_lab;

float SpatialPlant::Overlap(const SpatialPlant& other_plant) const {
  const auto& r0 = m_radius;
  const auto& r1 = other_plant.m_radius;
  const auto& x0 = m_position.x;
  const auto& x1 = other_plant.m_position.x;

  const auto& y0 = m_position.y;
  const auto& y1 = other_plant.m_position.y;
  const auto distance = glm::distance(m_position, other_plant.m_position);
  if (distance >= m_radius + other_plant.m_radius)
    return 0.0f;

  const float rr0 = r0 * r0;
  const float rr1 = r1 * r1;
  const float c = glm::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
  const float phi = (glm::acos((rr0 + (c * c) - rr1) / (2.f * r0 * c))) * 2.f;
  const float theta = (glm::acos((rr1 + (c * c) - rr0) / (2.f * r1 * c))) * 2.f;
  const float area1 = 0.5f * theta * rr1 - 0.5f * rr1 * glm::sin(theta);
  const float area2 = 0.5f * phi * rr0 - 0.5f * rr0 * glm::sin(phi);
  return area1 + area2;
}

float SpatialPlant::SymmetricInfluence(const SpatialPlant& otherPlant) const {
  return Overlap(otherPlant) * 0.5f;
}

float SpatialPlant::AsymmetricInfluence(const SpatialPlant& otherPlant) const {
  return m_radius > otherPlant.m_radius ? Overlap(otherPlant) : 0.0f;
}

float SpatialPlant::AsymmetricalCompetition(const SpatialPlant& otherPlant, const float weightingFactor) const {
  const auto symmetricInfluence = SymmetricInfluence(otherPlant);
  if (m_radius == otherPlant.m_radius)
    return symmetricInfluence;
  return weightingFactor * AsymmetricInfluence(otherPlant) + (1.f - weightingFactor) * symmetricInfluence;
}

float SpatialPlant::GetArea() const {
  return 2.f * glm::pi<float>() * m_radius * m_radius;
}

void SpatialPlant::Grow(const float size) {
  const float newArea = GetArea() + size;
  m_radius = glm::sqrt(newArea * 0.5f / glm::pi<float>());
}

void SpatialPlantGridCell::RegisterParticle(const SpatialPlantHandle handle) {
  m_plantHandles.emplace_back(handle);
}

void SpatialPlantGridCell::UnregisterParticle(const SpatialPlantHandle handle) {
  for (int i = 0; i < m_plantHandles.size(); i++) {
    if (m_plantHandles.at(i) == handle) {
      m_plantHandles.at(i) = m_plantHandles.back();
      m_plantHandles.pop_back();
      return;
    }
  }
}

unsigned SpatialPlantGrid::RegisterPlant(const glm::vec2& position, const SpatialPlantHandle handle) {
  const auto minBound = GetMinBound();
  const auto coordinate =
      glm::ivec2(floor((position.x - minBound.x) / GetCellSize()), floor((position.y - minBound.y) / GetCellSize()));
  const auto resolution = GetResolution();
  assert(coordinate.x < resolution.x && coordinate.y < resolution.y);
  const auto cellIndex = coordinate.x + coordinate.y * resolution.x;
  RefCell(cellIndex).RegisterParticle(handle);
  return cellIndex;
}

void SpatialPlantGrid::UnregisterPlant(const unsigned cellIndex, const SpatialPlantHandle handle) {
  RefCell(cellIndex).UnregisterParticle(handle);
}

void SpatialPlantGrid::Clear() {
  for (auto& cell : RefCells()) {
    cell.m_plantHandles.clear();
  }
}

void SpatialPlantGrid::ForEachPlant(const glm::vec2& position, const float radius,
                                    const std::function<void(SpatialPlantHandle plantHandle)>& func) {
  ForEach(position, radius, [&](const SpatialPlantGridCell& cell) {
    for (const auto& plantHandle : cell.m_plantHandles) {
      func(plantHandle);
    }
  });
}

SpatialPlantDistribution::SpatialPlantDistribution() {
  m_plantGrid.Reset(10.f, glm::vec2(-m_spatialPlantGlobalParameters.m_maxRadius),
                    glm::vec2(m_spatialPlantGlobalParameters.m_maxRadius));
}

float SpatialPlantDistribution::CalculateGrowth(const SpatialPlantGlobalParameters& richardGrowthModelParameters,
                                                const SpatialPlantHandle plantHandle,
                                                const std::vector<SpatialPlantHandle>& neighborPlantHandles) const {
  const auto& plant = m_plants[plantHandle];
  assert(!plant.m_recycled);
  const auto& plantParameter = m_spatialPlantParameters[plant.m_parameterHandle];
  const float area = plant.GetArea();
  assert(area > 0.0f);

  const auto w = 2.f * glm::pi<float>() * plantParameter.m_finalRadius * plantParameter.m_finalRadius;
  if (area >= w)
    return 0.0f;
  const auto f = glm::pow(area, richardGrowthModelParameters.m_a);

  float areaReduction = 0.0f;
  for (const auto& neighborPlantHandle : neighborPlantHandles) {
    const auto& otherPlant = m_plants[neighborPlantHandle];
    assert(!otherPlant.m_recycled);
    areaReduction += otherPlant.GetArea() * otherPlant.AsymmetricalCompetition(plant, richardGrowthModelParameters.m_p);
  }

  const float kf = plantParameter.m_k * f;
  const float remainingArea = glm::max(0.0f, area + areaReduction);
  if (richardGrowthModelParameters.m_delta != 1.f) {
    const float growthRateWithoutCompetition = 1.f / (richardGrowthModelParameters.m_delta - 1.f);
    const float growthFactor = 1.f - glm::pow(remainingArea / w, richardGrowthModelParameters.m_delta - 1.f);
    const auto retVal = kf * growthRateWithoutCompetition * growthFactor;
    return m_spatialPlantGlobalParameters.m_simulationRate * glm::max(0.0f, retVal);
  }
  return m_spatialPlantGlobalParameters.m_simulationRate * glm::max(0.0f, kf * (glm::log(w) - glm::log(remainingArea)));
}

void SpatialPlantDistribution::Simulate() {
  const auto cellSize = 10.f;
  const auto newMin = glm::vec2(-m_spatialPlantGlobalParameters.m_maxRadius);
  const auto newMax = glm::vec2(m_spatialPlantGlobalParameters.m_maxRadius);
  const auto newResolution =
      glm::ivec2(glm::ceil((newMax.x - newMin.x) / cellSize) + 1, glm::ceil((newMax.y - newMin.y) / cellSize) + 1);
  const auto maxBound = newMin + cellSize * glm::vec2(newResolution);
  if (m_plantGrid.GetMinBound() != newMin || m_plantGrid.GetMaxBound() != maxBound) {
    m_plantGrid.Reset(cellSize, newMin, newMax);
    for (auto& plant : m_plants) {
      if (plant.m_recycled)
        continue;
      if (glm::abs(plant.m_position.x) >= m_spatialPlantGlobalParameters.m_maxRadius ||
          glm::abs(plant.m_position.y) >= m_spatialPlantGlobalParameters.m_maxRadius) {
        RecyclePlant(plant.m_handle, false);
      } else {
        plant.m_gridCellIndex = m_plantGrid.RegisterPlant(plant.m_position, plant.m_handle);
      }
    }
  }
  float maxRadius = 0.0f;
  for (const auto& parameter : m_spatialPlantParameters) {
    maxRadius = glm::max(maxRadius, parameter.m_finalRadius);
  }
  Jobs::RunParallelFor(m_plants.size(), [&](unsigned plantIndex) {
    auto& plant = m_plants[plantIndex];
    if (plant.m_recycled)
      return;
    std::vector<SpatialPlantHandle> neighbors{};
    m_plantGrid.ForEachPlant(
        plant.m_position, 2.f * maxRadius + plant.m_radius, [&](SpatialPlantHandle otherPlantHandle) {
          const auto& otherPlant = m_plants[otherPlantHandle];
          if (otherPlantHandle != plant.m_handle && !otherPlant.m_recycled &&
              glm::distance(otherPlant.m_position, plant.m_position) < otherPlant.m_radius + plant.m_radius) {
            neighbors.emplace_back(otherPlantHandle);
          }
        });
    const auto growSize = CalculateGrowth(m_spatialPlantGlobalParameters, plant.m_handle, neighbors);
    plant.Grow(growSize);
  });

  std::vector<SpatialPlantHandle> oldPlants;
  for (const auto& plant : m_plants) {
    if (!plant.m_recycled)
      oldPlants.emplace_back(plant.m_handle);
  }
  for (const auto& plantHandle : oldPlants) {
    const auto& parameter = m_spatialPlantParameters[m_plants[plantHandle].m_parameterHandle];
    if (m_spatialPlantGlobalParameters.m_simulationRate * parameter.m_seedingPossibility *
            m_plants[plantHandle].GetArea() >
        glm::linearRand(0.0f, 1.0f)) {
      auto direction = glm::circularRand(1.0f);
      const auto position = m_plants[plantHandle].m_position +
                            direction * (glm::linearRand(parameter.m_seedingRangeMin, parameter.m_seedingRangeMax) *
                                             m_plants[plantHandle].m_radius +
                                         parameter.m_seedInitialRadius);
      if (glm::abs(position.x) < m_spatialPlantGlobalParameters.m_maxRadius &&
          glm::abs(position.y) < m_spatialPlantGlobalParameters.m_maxRadius) {
        AddPlant(m_plants[plantHandle].m_parameterHandle, parameter.m_seedInitialRadius, position);
      }
    }
  }
  for (const auto& plant : m_plants) {
    if (plant.m_recycled)
      continue;
    if (glm::abs(plant.m_position.x) > m_spatialPlantGlobalParameters.m_maxRadius ||
        glm::abs(plant.m_position.y) > m_spatialPlantGlobalParameters.m_maxRadius)
      RecyclePlant(plant.m_handle);
  }

  std::vector<float> plantSizes;
  std::vector<float> inverseStatisticalDistributions;
  plantSizes.resize(m_spatialPlantParameters.size());
  inverseStatisticalDistributions.resize(m_spatialPlantParameters.size());
  for (auto& plantSize : plantSizes)
    plantSize = 0;
  float totalSize = 0.0f;
  for (const auto& plant : m_plants) {
    if (plant.m_recycled)
      continue;
    const auto area = plant.GetArea();
    plantSizes[plant.m_parameterHandle] += area;
    totalSize += area;
  }
  for (int i = 0; i < inverseStatisticalDistributions.size(); i++) {
    inverseStatisticalDistributions[i] =
        glm::pow(1.f - plantSizes[i] / totalSize, m_spatialPlantGlobalParameters.m_dynamicBalanceFactor);
  }
  for (int i = 0; i < m_plants.size(); i++) {
    auto& plantI = m_plants[i];
    if (plantI.m_recycled)
      continue;
    std::vector<SpatialPlantHandle> neighbors{};
    m_plantGrid.ForEachPlant(
        plantI.m_position, 2.f * maxRadius + plantI.m_radius, [&](SpatialPlantHandle otherPlantHandle) {
          const auto& otherPlant = m_plants[otherPlantHandle];
          if (otherPlantHandle != plantI.m_handle && !otherPlant.m_recycled &&
              glm::distance(otherPlant.m_position, plantI.m_position) < otherPlant.m_radius + plantI.m_radius) {
            neighbors.emplace_back(otherPlantHandle);
          }
        });
    for (const auto& j : neighbors) {
      auto& plantJ = m_plants[j];
      if (plantJ.m_recycled)
        continue;
      if (glm::distance(plantI.m_position, plantJ.m_position) < plantI.m_radius + plantJ.m_radius) {
        const float relativeSizeI = plantI.m_radius / m_spatialPlantParameters[plantI.m_parameterHandle].m_finalRadius;
        const float relativeSizeJ = plantJ.m_radius / m_spatialPlantParameters[plantJ.m_parameterHandle].m_finalRadius;
        const float vi = 1.0f / m_spatialPlantGlobalParameters.m_simulationRate *
                         inverseStatisticalDistributions[plantI.m_parameterHandle] *
                         (relativeSizeI > m_spatialPlantGlobalParameters.m_spawnProtectionFactor ? 1.f : relativeSizeI);
        const float vj = 1.0f / m_spatialPlantGlobalParameters.m_simulationRate *
                         inverseStatisticalDistributions[plantJ.m_parameterHandle] *
                         (relativeSizeJ > m_spatialPlantGlobalParameters.m_spawnProtectionFactor ? 1.f : relativeSizeJ);
        const bool equalFirst = glm::linearRand(0.0f, 1.0f) > 0.5f;

        if (vi > vj || (vi == vj && equalFirst)) {
          if (m_spatialPlantGlobalParameters.m_forceRemoveOverlap) {
            RecyclePlant(j);
          } else {
            if (vj < glm::linearRand(0.0f, 1.0f)) {
              RecyclePlant(j);
            } else if (vi < glm::linearRand(0.0f, 1.0f)) {
              RecyclePlant(i);
            }
          }
        } else if (vi < vj || (vi == vj && !equalFirst)) {
          if (m_spatialPlantGlobalParameters.m_forceRemoveOverlap) {
            RecyclePlant(i);
          } else {
            if (vi < glm::linearRand(0.0f, 1.0f)) {
              RecyclePlant(i);
            } else if (vj < glm::linearRand(0.0f, 1.0f)) {
              RecyclePlant(j);
            }
          }
        }
      }
      if (plantI.m_recycled)
        break;
    }
  }
  m_simulationTime++;
}

SpatialPlantHandle SpatialPlantDistribution::AddPlant(const SpatialPlantParameterHandle spatialPlantParameterHandle,
                                                      const float radius, const glm::vec2& position) {
  assert(spatialPlantParameterHandle >= 0 && spatialPlantParameterHandle < m_spatialPlantParameters.size());
  SpatialPlantHandle newPlantHandle;
  if (m_recycledPlants.empty()) {
    newPlantHandle = m_plants.size();
    m_plants.emplace_back();
  } else {
    newPlantHandle = m_recycledPlants.front();
    m_recycledPlants.pop();
  }
  auto& newPlant = m_plants[newPlantHandle];
  newPlant.m_parameterHandle = spatialPlantParameterHandle;
  newPlant.m_handle = newPlantHandle;
  newPlant.m_radius = radius;
  newPlant.m_position = position;
  newPlant.m_recycled = false;
  newPlant.m_gridCellIndex = m_plantGrid.RegisterPlant(position, newPlantHandle);
  return newPlant.m_handle;
}

void SpatialPlantDistribution::RecyclePlant(SpatialPlantHandle plantHandle, const bool removeFromGrid) {
  assert(m_plants.size() > plantHandle && plantHandle >= 0);
  auto& plant = m_plants[plantHandle];
  if (plant.m_recycled)
    return;
  m_recycledPlants.emplace(plantHandle);
  plant.m_recycled = true;
  plant.m_radius = 0.0f;
  if (removeFromGrid)
    m_plantGrid.UnregisterPlant(plant.m_gridCellIndex, plant.m_handle);
  plant.m_gridCellIndex = INT_MAX;
}
