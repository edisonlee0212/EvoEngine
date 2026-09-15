//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "ClassRegistry.hpp"
#include "DynamicTreeStrands.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_package;

std::weak_ptr<Soil> EcoSysLabLayer::FindSoil() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soil_entities && !soil_entities->empty()) {
    return scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0));
  }
  return {};
}
