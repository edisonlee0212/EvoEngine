#include "SpatialPlantDistributionSimulator.hpp"

using namespace eco_sys_lab_package;

void SpatialPlantDistributionSimulator::FixedUpdate() {
  if (m_simulate) {
    m_distribution.Simulate();
  }
}

void SpatialPlantDistributionSimulator::OnCreate() {
  m_distribution = {};
  m_distribution.m_spatialPlantParameters.emplace_back();
  m_treeDescriptors.resize(m_distribution.m_spatialPlantParameters.size());
}
