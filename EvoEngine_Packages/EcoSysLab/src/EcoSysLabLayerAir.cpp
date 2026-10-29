#include "EcoSysLabLayer.hpp"

#include "AirEnvironment.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSystem.hpp"
#include "GpuTreePhysics.hpp"
#include "Platform/Platform.hpp"
#include "WindParticleField.hpp"

using namespace eco_sys_lab_package;

void EcoSysLabSystem::Dynamics::Reset() {
  for (const auto& entry : entries)
    if (const auto physics = entry.physics.lock())
      physics->SetGpuPhysics(nullptr, 0);
  entries.clear();
  tree_physics.reset();
  physics_scene.reset();
  wind.reset();
}

void EcoSysLabSystem::Dynamics::UpdateAir(const std::shared_ptr<Scene>& scene, Environment& environment,
                                          const bool growing) {
  auto& wind_particle_field_ = wind;
  auto& gpu_physics_entries_ = entries;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<AirEnvironment>()) {
    const std::vector<Entity> legacy_entities = *owners;
    bool migrated = false;
    for (const auto entity : legacy_entities) {
      if (!scene->IsEntityValid(entity))
        continue;
      const auto legacy = scene->GetOrSetPrivateComponent<AirEnvironment>(entity).lock();
      if (!migrated && legacy && legacy->IsEnabled() && scene->IsEntityEnabled(entity)) {
        environment.air_temperature_c = legacy->temperature_c;
        environment.air_relative_humidity = legacy->relative_humidity;
        environment.air_pressure_pa = legacy->pressure_pa;
        environment.air_coupling_rate = legacy->coupling_rate;
        environment.air_friction = legacy->friction;
        migrated = true;
      }
      scene->RemovePrivateComponent<AirEnvironment>(entity);
    }
  }
  SyncTrees(scene, growing);
  if (wind_particle_field_) {
    wind_particle_field_->ambient_temperature_kelvin = environment.air_temperature_c + 273.15f;
    wind_particle_field_->ambient_specific_humidity = Climate::SpecificHumidity(
        environment.air_temperature_c, environment.air_relative_humidity, environment.air_pressure_pa);
    wind_particle_field_->ambient_pressure_pa = environment.air_pressure_pa;
    wind_particle_field_->air_coupling_rate = environment.air_coupling_rate;
    wind_particle_field_->air_friction = environment.air_friction;
    const float cell_size = glm::max(environment.air_cell_size, Climate::minimum_air_cell_size);
    if (wind_particle_field_->GetAirCellSize() != cell_size)
      ResetAirGrid(scene, cell_size);
    std::vector<glm::mat4> models;
    models.reserve(gpu_physics_entries_.size());
    for (const auto& item : gpu_physics_entries_)
      models.push_back(scene->GetDataComponent<GlobalTransform>(item.entity).value);
    wind_particle_field_->RegisterTreeModels(models);
  }
}

void EcoSysLabLayer::ResetAirGrid(const float cell_size) {
  if (const auto system = GetScene()->GetSystem<EcoSysLabSystem>())
    system->dynamics.ResetAirGrid(GetScene(), cell_size);
}

void EcoSysLabSystem::Dynamics::ResetAirGrid(const std::shared_ptr<Scene>& scene, const float cell_size) {
  auto& wind_particle_field_ = wind;
  auto& gpu_physics_entries_ = entries;
  auto& gpu_tree_physics_ = tree_physics;
  if (!wind_particle_field_)
    return;
  std::vector<DynamicTreeSkeleton::GpuRestState> states;
  std::vector<glm::mat4> models;
  states.reserve(gpu_physics_entries_.size());
  models.reserve(gpu_physics_entries_.size());
  for (const auto& item : gpu_physics_entries_)
    if (const auto physics = item.physics.lock()) {
      states.push_back(physics->ExportGpuRestState());
      models.push_back(scene->GetDataComponent<GlobalTransform>(item.entity).value);
    }
  Platform::WaitForFrameSubmissions("EcoSysLab air grid reset");
  wind_particle_field_->ResetAirGrid(states, gpu_tree_physics_, models, cell_size);
}

void EcoSysLabSystem::Dynamics::SyncTrees(const std::shared_ptr<Scene>& scene, const bool growing) {
  if (!scene)
    return;
  auto& wind_particle_field_ = wind;
  auto& gpu_tree_physics_ = tree_physics;
  auto& gpu_physics_entries_ = entries;
  auto& gpu_physics_scene_ = physics_scene;
  struct ActiveGpuTree {
    Entity entity;
    std::shared_ptr<Tree> tree;
    std::shared_ptr<DynamicTreeSkeleton> physics;
  };
  std::vector<ActiveGpuTree> active;
  if (const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeSkeleton>()) {
    for (const auto entity : *entities) {
      if (!scene->IsEntityEnabled(entity) || !scene->HasPrivateComponent<Tree>(entity))
        continue;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
      const auto physics = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity).lock();
      if (tree && tree->IsEnabled() && physics && physics->IsEnabled()) {
        physics->RestoreGpuPoseAfterLoad(*tree);
        if (physics->SyncGpuPhysics(*tree, scene->GetDataComponent<GlobalTransform>(entity), growing))
          active.push_back({entity, tree, physics});
      }
    }
  }
  bool rebuild = (!gpu_tree_physics_ && !active.empty()) || gpu_physics_scene_.lock() != scene ||
                 active.size() != gpu_physics_entries_.size();
  if (!rebuild) {
    for (size_t i = 0; i < active.size(); ++i) {
      const auto& previous = gpu_physics_entries_[i];
      if (previous.entity != active[i].entity || previous.tree.lock() != active[i].tree ||
          previous.physics.lock() != active[i].physics ||
          previous.revision != active[i].tree->GetShootModelRevision().content ||
          active[i].physics->GetGpuPhysics() != gpu_tree_physics_ || active[i].physics->GetGpuTreeIndex() != i) {
        rebuild = true;
        break;
      }
    }
  }
  if (rebuild) {
    if (gpu_physics_scene_.lock() != scene)
      wind_particle_field_.reset();
    for (const auto& entry : gpu_physics_entries_)
      if (const auto physics = entry.physics.lock())
        physics->SetGpuPhysics(nullptr, 0);
    gpu_physics_entries_.clear();
    gpu_tree_physics_.reset();
    gpu_physics_scene_ = scene;
    if (!active.empty()) {
      std::vector<DynamicTreeSkeleton::GpuRestState> states;
      states.reserve(active.size());
      for (const auto& item : active)
        states.push_back(item.physics->ExportGpuRestState());
      if (!wind_particle_field_)
        wind_particle_field_ = std::make_shared<WindParticleField>();
      wind_particle_field_->RegisterTreeCoverage(states);
      auto shared = std::make_shared<GpuTreePhysics>();
      if (shared->Initialize(states)) {
        gpu_tree_physics_ = shared;
        EVOENGINE_LOG("Initialized shared GPU tree physics for " + std::to_string(active.size()) + " trees.")
        for (uint32_t i = 0; i < active.size(); ++i) {
          const auto& item = active[i];
          item.physics->SetGpuPhysics(shared, i);
          gpu_physics_entries_.push_back(
              {item.entity, item.tree, item.physics, item.tree->GetShootModelRevision().content});
        }
      }
    }
  }
}
