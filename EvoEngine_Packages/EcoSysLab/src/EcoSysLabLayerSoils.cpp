//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSystem.hpp"

#include "AssetManager.hpp"
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "DynamicTreeStrands.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_package;

void EcoSysLabSystem::Environment::EnsureConfiguration(const std::shared_ptr<Scene>& scene) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Climate>()) {
    const bool use_legacy = !climate_descriptor.Get<ClimateDescriptor>();
    bool selected = false;
    bool selected_enabled = false;
    const auto legacy_entities = *owners;
    for (const auto entity : legacy_entities) {
      const auto legacy = scene->GetOrSetPrivateComponent<Climate>(entity).lock();
      const bool enabled = legacy && legacy->IsEnabled() && scene->IsEntityEnabled(entity);
      if (use_legacy && legacy && (!selected || (enabled && !selected_enabled))) {
        climate_descriptor = legacy->climate_descriptor_ref;
        air_temperature_c = legacy->air_temperature_c;
        air_relative_humidity = legacy->air_relative_humidity;
        air_pressure_pa = legacy->air_pressure_pa;
        air_coupling_rate = legacy->air_coupling_rate;
        air_friction = legacy->air_friction;
        air_cell_size = legacy->air_cell_size;
        selected = true;
        selected_enabled = enabled;
      }
      scene->RemovePrivateComponent<Climate>(entity);
    }
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Soil>()) {
    const bool use_legacy = !soil_descriptor.Get<SoilDescriptor>();
    bool selected = false;
    bool selected_enabled = false;
    const auto legacy_entities = *owners;
    for (const auto entity : legacy_entities) {
      const auto legacy = scene->GetOrSetPrivateComponent<Soil>(entity).lock();
      const bool enabled = legacy && legacy->IsEnabled() && scene->IsEntityEnabled(entity);
      if (use_legacy && legacy && (!selected || (enabled && !selected_enabled))) {
        soil_descriptor = legacy->soil_descriptor_ref;
        soil_visualization_owner = scene->GetEntityHandle(entity);
        selected = true;
        selected_enabled = enabled;
      }
      scene->RemovePrivateComponent<Soil>(entity);
    }
  }
  if (!climate_descriptor.Get<ClimateDescriptor>())
    climate_descriptor = AssetManager::CreateTemporaryAsset<ClimateDescriptor>();
  if (!soil_descriptor.Get<SoilDescriptor>()) {
    const auto descriptor = AssetManager::CreateTemporaryAsset<SoilDescriptor>();
    descriptor->texture_resolution = {64, 64};
    descriptor->height_field = AssetManager::CreateTemporaryAsset<HeightField>();
    soil_descriptor = descriptor;
  }
}

void EcoSysLabSystem::Environment::InitializeModels() {
  if (initialized)
    return;
  if (const auto descriptor = climate_descriptor.Get<ClimateDescriptor>())
    climate.Initialize(descriptor->climate_parameters);
  BuildSoilModel(soil, soil_descriptor.Get<SoilDescriptor>());
  initialized = true;
}

Entity EcoSysLabSystem::Environment::GetOrCreateVisualizationOwner(const std::shared_ptr<Scene>& scene) {
  auto owner = scene->GetEntity(soil_visualization_owner);
  if (!scene->IsEntityValid(owner)) {
    owner = scene->CreateEntity("Soil visualization");
    soil_visualization_owner = scene->GetEntityHandle(owner);
  }
  return owner;
}

void EcoSysLabSystem::Environment::PrepareForGrowth(const std::shared_ptr<Scene>& scene,
                                                    const SimulationSettings& settings) {
  const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (!entities || entities->empty())
    return;
  auto& grid = climate.environment_grid;
  auto min_bound = grid.voxel_grid.GetMinBound();
  auto max_bound = grid.voxel_grid.GetMaxBound();
  bool changed = false;
  for (const auto entity : *entities) {
    if (!scene->IsEntityEnabled(entity))
      continue;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
    if (!tree || !tree->IsEnabled())
      continue;
    const auto transform = scene->GetDataComponent<GlobalTransform>(entity).value;
    auto& skeleton = tree->shoot_model.RefShootSkeleton();
    skeleton.CalculateMinMax();
    const glm::vec3 minimum = transform * glm::vec4(skeleton.min, 1.0f);
    const glm::vec3 maximum = transform * glm::vec4(skeleton.max, 1.0f);
    if (glm::any(glm::lessThanEqual(minimum, min_bound)) || glm::any(glm::greaterThanEqual(maximum, max_bound))) {
      min_bound = glm::min(minimum - glm::vec3(1.0f, 0.1f, 1.0f), min_bound);
      max_bound = glm::max(maximum + glm::vec3(1.0f), max_bound);
      changed = true;
    }
  }
  if (changed)
    grid.voxel_grid.Initialize(grid.voxel_size, min_bound, max_bound);
  grid.voxel_grid.Reset();
  for (const auto entity : *entities) {
    if (!scene->IsEntityEnabled(entity))
      continue;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
    if (tree && tree->IsEnabled())
      tree->RegisterVoxel();
  }
  grid.LightPropagation(settings);
}

void EcoSysLabSystem::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(environment.climate_descriptor);
  list.push_back(environment.soil_descriptor);
}
