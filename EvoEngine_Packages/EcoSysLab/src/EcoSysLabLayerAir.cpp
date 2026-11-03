#include <algorithm>
#include "EcoSysLabLayer.hpp"

#include "AirEnvironment.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSystem.hpp"
#include "GpuTreeClusterPhysics.hpp"
#include "GpuTreePhysics.hpp"
#include "Platform/Platform.hpp"
#include "Profiler.hpp"
#include "WindParticleField.hpp"

using namespace eco_sys_lab_package;

void EcoSysLabSystem::Dynamics::Reset() {
  if (tree_physics || wind || cluster_forest)
    if (const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>())
      layer->RetireDynamics(tree_physics, wind, cluster_forest);
  for (const auto& entry : entries)
    if (const auto physics = entry.physics.lock())
      physics->SetGpuPhysics(nullptr, 0);
  entries.clear();
  tree_physics.reset();
  cluster_forest.reset();
  cluster_members.clear();
  physics_scene.reset();
  wind.reset();
}

void EcoSysLabSystem::Dynamics::UpdateAir(const std::shared_ptr<Scene>& scene, Environment& environment,
                                          const bool growing) {
  const ProfilerScope cpu_scope("EcoSysLab / Update Air", "EcoSysLab");
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
    std::vector<uint32_t> ids;
    models.reserve(gpu_physics_entries_.size());
    for (const auto& item : gpu_physics_entries_) {
      models.push_back(scene->GetDataComponent<GlobalTransform>(item.entity).value);
      ids.push_back(item.gpu_id);
    }
    wind_particle_field_->RegisterTreeModels(models, ids);
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
  if (gpu_physics_scene_.lock() != scene) {
    if (gpu_tree_physics_ || wind_particle_field_ || cluster_forest)
      if (const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>())
        layer->RetireDynamics(gpu_tree_physics_, wind_particle_field_, cluster_forest);
    for (const auto& entry : gpu_physics_entries_)
      if (const auto component = entry.physics.lock())
        component->SetGpuPhysics(nullptr, 0);
    gpu_physics_entries_.clear();
    gpu_tree_physics_.reset();
    cluster_forest.reset();
    cluster_members.clear();
    wind_particle_field_.reset();
    gpu_physics_scene_ = scene;
  }
  if (!gpu_tree_physics_ && !active.empty())
    gpu_tree_physics_ = std::make_shared<GpuTreePhysics>();
  if (!wind_particle_field_ && !active.empty())
    wind_particle_field_ = std::make_shared<WindParticleField>();
  std::vector<Entry> next;
  std::vector<uint32_t> ids, changed_ids;
  std::vector<DynamicTreeSkeleton::GpuRestState> changed;
  for (const auto& item : active) {
    const auto old = std::find_if(gpu_physics_entries_.begin(), gpu_physics_entries_.end(), [&](const auto& entry) {
      return entry.entity == item.entity && entry.tree.lock() == item.tree && entry.physics.lock() == item.physics;
    });
    auto id = old == gpu_physics_entries_.end() ? UINT32_MAX : old->gpu_id;
    const auto revision = item.tree->GetShootModelRevision().content;
    if (old == gpu_physics_entries_.end() || old->revision != revision ||
        item.physics->GetGpuPhysics() != gpu_tree_physics_) {
      auto state = item.physics->ExportGpuRestState();
      id = gpu_tree_physics_->RegisterTree(state, id, !growing);
      if (id == UINT32_MAX)
        continue;
      changed.push_back(std::move(state));
      changed_ids.push_back(id);
      item.physics->SetGpuPhysics(gpu_tree_physics_, id);
    }
    if (!growing && gpu_tree_physics_->GetTreeState(id)->attribute_arrays && !gpu_tree_physics_->GetTreeState(id)->lod)
      gpu_tree_physics_->InitializeLod(id, item.physics->ExportGpuRestState());
    Entry entry{item.entity, item.tree, item.physics, revision, id};
    if (old != gpu_physics_entries_.end() && old->cluster_source == gpu_tree_physics_->GetTreeState(id)) {
      entry.cluster = old->cluster;
      entry.cluster_source = old->cluster_source;
      entry.cluster_running = old->cluster_running;
      entry.pose_revision = old->pose_revision;
    }
    next.push_back(std::move(entry));
    ids.push_back(id);
  }
  for (const auto& old : gpu_physics_entries_) {
    if (std::find(ids.begin(), ids.end(), old.gpu_id) != ids.end())
      continue;
    gpu_tree_physics_->RemoveTree(old.gpu_id);
    if (const auto component = old.physics.lock())
      if (component->GetGpuTreeIndex() == old.gpu_id)
        component->SetGpuPhysics(nullptr, 0);
  }
  gpu_physics_entries_ = std::move(next);
  if (gpu_tree_physics_) {
    gpu_tree_physics_->SetActiveTrees(ids);
    // Retire old allocations even when no solver step runs.
    if (growing || ids.empty())
      gpu_tree_physics_->PrepareRegistry();
  }
  if (wind_particle_field_ && !changed.empty())
    wind_particle_field_->RegisterTreeCoverage(changed, changed_ids);
  SyncClusters(growing);
}

void EcoSysLabLayer::RetireDynamics(std::shared_ptr<GpuTreePhysics> physics, std::shared_ptr<WindParticleField> wind,
                                    std::shared_ptr<GpuTreeClusterForest> clusters) {
  if (!Platform::Initialized())
    return;
  retired_dynamics_.push_back(
      {Platform::TrackCurrentFrameSubmission(), std::move(physics), std::move(wind), std::move(clusters)});
}
