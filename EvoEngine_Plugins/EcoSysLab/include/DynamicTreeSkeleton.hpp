#pragma once
#include "DynamicSkeleton.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DynamicTreeSkeleton : public IPrivateComponent {
 public:
  DynamicSkeleton dynamic_skeleton{};
  DynamicSkeleton::InitializeParameters initialize_parameters{};
  std::shared_ptr<ParticleInfoList> debug_matrices;
  bool simulate = true;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void LateUpdate() override;
  void OnCreate() override;
  void PhysicsStep(const DynamicSkeleton::PhysicsParameters& physics_parameters);
  void Visualization(const std::shared_ptr<Camera>& target_camera,
                     const DynamicSkeleton::VisualizationParameters& visualization_parameters) const;
};
}  // namespace eco_sys_lab_plugin