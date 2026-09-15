#pragma once
#include "DsColliders.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicTreeStrands.hpp"
#include "InspectorRegistry.hpp"
namespace eco_sys_lab_package {
struct DsBoxColliderInspector {
  PrivateComponentRef mesh_renderer_ref;
  bool display_bound = true;
  bool Inspect(evo_engine::InspectorContext& context, DsBoxCollider& target);
};
struct DsCylinderColliderInspector {
  bool display_bound = true;
  bool Inspect(evo_engine::InspectorContext& context, DsCylinderCollider& target);
};
struct DsSphereColliderInspector {
  bool display_bound = true;
  bool Inspect(evo_engine::InspectorContext& context, DsSphereCollider& target);
};
struct DynamicTreeStrandsInspector {
  PrivateComponentRef dynamic_tree_strands_tree_ref{};
  DynamicTreeStrands::BoardExperimentSetupSettings multiple_rod_experiment_setup_settings{};
  DynamicTreeStrands::LogExperimentSetupSettings log_experiment_setup_settings{};
  bool Inspect(evo_engine::InspectorContext& context, DynamicTreeStrands& target);
};
struct DynamicStrandsDemoInspector {
  GlobalTransform camera_pose{};
  bool Inspect(evo_engine::InspectorContext& context, DynamicStrandsDemo& target);
  void ResetEnvironment(DynamicStrandsDemo& target, const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
};
void DrawColliderBound(const DsBoxCollider& target, const std::shared_ptr<evo_engine::EditorLayer>& editor_layer,
                       const std::shared_ptr<evo_engine::Camera>& editor_camera, const glm::vec4& color);
void DrawColliderBound(const DsCylinderCollider& target, const std::shared_ptr<evo_engine::EditorLayer>& editor_layer,
                       const std::shared_ptr<evo_engine::Camera>& editor_camera, const glm::vec4& color);
void DrawColliderBound(const DsSphereCollider& target, const std::shared_ptr<evo_engine::EditorLayer>& editor_layer,
                       const std::shared_ptr<evo_engine::Camera>& editor_camera, const glm::vec4& color);
void DrawColliderBound(const IDsCollider& target, const std::shared_ptr<evo_engine::EditorLayer>& editor_layer,
                       const std::shared_ptr<evo_engine::Camera>& editor_camera, const glm::vec4& color);
}  // namespace eco_sys_lab_package
