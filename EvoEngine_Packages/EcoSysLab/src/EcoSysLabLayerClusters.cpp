#include <algorithm>
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSystem.hpp"
#include "GpuTreeClusterPhysics.hpp"
#include "WindParticleField.hpp"
using namespace eco_sys_lab_package;

void EcoSysLabSystem::Dynamics::SyncClusters(bool growing) {
  const auto retire = [&] {
    if (cluster_forest)
      if (const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>())
        layer->RetireDynamics({}, {}, cluster_forest);
    cluster_forest.reset();
    cluster_members.clear();
  };
  bool ready = use_subtree_clusters && !growing && !entries.empty();
  std::vector<std::shared_ptr<GpuTreeClusterPhysics>> members;
  if (ready)
    for (auto& entry : entries) {
      const auto state = tree_physics->GetTreeState(entry.gpu_id);
      if (entry.cluster_source != state) {
        entry.cluster_source = state;
        auto cluster = std::make_shared<GpuTreeClusterPhysics>();
        if (cluster->InitializeDynamic(entry.physics.lock()->ExportGpuRestState(), state))
          entry.cluster = std::move(cluster);
        else
          EVOENGINE_ERROR("Cannot initialize subtree physics; retaining rod simulation.")
      }
      ready &= entry.cluster != nullptr;
      members.push_back(entry.cluster);
    }
  if (!ready) {
    for (auto& entry : entries) {
      if (entry.cluster_running)
        tree_physics->ResetLodSelection(entry.gpu_id);
      entry.cluster_running = false;
      if (const auto physics = entry.physics.lock())
        physics->physics_settings.use_subtree_clusters = false;
    }
    retire();
    return;
  }
  if (members != cluster_members) {
    retire();
    auto forest = std::make_shared<GpuTreeClusterForest>();
    if (!forest->Initialize(members))
      return;
    cluster_forest = std::move(forest);
    cluster_members = std::move(members);
  }
  for (auto& entry : entries) {
    if (!entry.cluster_running) {
      ++entry.pose_revision;
      entry.cluster_running = true;
    }
    if (const auto physics = entry.physics.lock()) {
      physics->physics_settings.use_subtree_clusters = true;
      physics->physics_settings.cluster_lod = cluster_lod;
    }
  }
}

bool EcoSysLabSystem::Dynamics::StepClusters(const std::shared_ptr<Scene>& scene) {
  if (!cluster_forest)
    return false;
  std::vector<GpuTreeClusterForest::TreeStep> steps;
  steps.reserve(entries.size());
  for (const auto& entry : entries) {
    const auto physics = entry.physics.lock();
    const auto& p = physics->physics_settings;
    GpuTreeClusterForest::TreeStep step;
    step.settings.time_step = p.time_step;
    step.settings.substeps = glm::clamp(p.substeps, 1, 64);
    step.settings.gravity = p.gravity;
    step.settings.damping = p.damping;
    step.settings.wind_drag = p.wind_drag;
    step.bend_damping_time = p.cluster_bend_damping_time;
    step.settings.angular_stiffness = step.settings.base_angular_stiffness = glm::vec3(p.bend_stiffness);
    step.model = scene->GetDataComponent<GlobalTransform>(entry.entity);
    auto& state = *tree_physics->GetTreeState(entry.gpu_id);
    state.previous_model = step.model.value;
    step.model.SetPosition(step.model.GetPosition() + glm::vec3(step.model.value * glm::vec4(p.anchor_offset, 0)));
    step.running = physics->physics_running;
    step.lod = p.cluster_lod;
    step.pose_revision = entry.pose_revision;
    step.mass_scale = p.density / std::max(state.reference_density, 0.001f);
    steps.push_back(step);
  }
  GpuTreePhysics::WindInput input;
  if (wind)
    input = {wind->GetAirDescriptorSet(), wind->GetAirCellCount(), GpuSpatialGrid::bucket_count,
             wind->GetAirCellSize()};
  cluster_forest->Step(steps, input);
  return true;
}
