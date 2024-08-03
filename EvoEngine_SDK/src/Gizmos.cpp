#include "Application.hpp"
#include "EditorLayer.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
using namespace evo_engine;

void GizmoSettings::ApplySettings(GraphicsPipelineStates& global_pipeline_state) const {
  draw_settings.ApplySettings(global_pipeline_state);
  global_pipeline_state.depth_test = depth_test;
  global_pipeline_state.depth_write = depth_write;
}

void EditorLayer::DrawGizmoMesh(const std::shared_ptr<Mesh>& mesh,
                                const std::shared_ptr<Camera>& editor_camera_component, const glm::vec4& color,
                                const glm::mat4& model, const float& size, const GizmoSettings& gizmo_settings) {
  if (Application::GetApplicationExecutionStatus() == ApplicationExecutionStatus::LateUpdate) {
    EVOENGINE_ERROR("Gizmos command ignored! Submit gizmos command during LateUpdate is now allowed!")
    return;
  }
  gizmo_mesh_tasks_.push_back({mesh, editor_camera_component, color, model, size, gizmo_settings});
}

void EditorLayer::DrawGizmoStrands(const std::shared_ptr<Strands>& strands,
                                   const std::shared_ptr<Camera>& editor_camera_component, const glm::vec4& color,
                                   const glm::mat4& model, const float& size, const GizmoSettings& gizmo_settings) {
  if (Application::GetApplicationExecutionStatus() == ApplicationExecutionStatus::LateUpdate) {
    EVOENGINE_ERROR("Gizmos command ignored! Submit gizmos command during LateUpdate is now allowed!")
    return;
  }
  gizmo_strands_tasks_.push_back({strands, editor_camera_component, color, model, size, gizmo_settings});
}

void EditorLayer::DrawGizmoMeshInstancedColored(const std::shared_ptr<Mesh>& mesh,
                                                const std::shared_ptr<Camera>& editor_camera_component,
                                                const std::shared_ptr<ParticleInfoList>& instanced_data,
                                                const glm::mat4& model, const float& size,
                                                const GizmoSettings& gizmo_settings) {
  if (Application::GetApplicationExecutionStatus() == ApplicationExecutionStatus::LateUpdate) {
    EVOENGINE_ERROR("Gizmos command ignored! Submit gizmos command during LateUpdate is now allowed!")
    return;
  }
  gizmo_instanced_mesh_tasks_.push_back({mesh, editor_camera_component, instanced_data, model, size, gizmo_settings});
}

void EditorLayer::DrawGizmoMeshInstancedColored(const std::shared_ptr<Mesh>& mesh,
                                                const std::shared_ptr<ParticleInfoList>& instanced_data,
                                                const glm::mat4& model, const float& size,
                                                const GizmoSettings& gizmo_settings) {
  if (const auto render_layer = Application::GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto scene_camera = GetSceneCamera();
  DrawGizmoMeshInstancedColored(mesh, scene_camera, instanced_data, model, size, gizmo_settings);
}

void EditorLayer::DrawGizmoMesh(const std::shared_ptr<Mesh>& mesh, const glm::vec4& color, const glm::mat4& model,
                                const float& size, const GizmoSettings& gizmo_settings) {
  if (const auto render_layer = Application::GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto scene_camera = GetSceneCamera();
  DrawGizmoMesh(mesh, scene_camera, color, model, size, gizmo_settings);
}

void EditorLayer::DrawGizmoStrands(const std::shared_ptr<Strands>& strands, const glm::vec4& color,
                                   const glm::mat4& model, const float& size, const GizmoSettings& gizmo_settings) {
  if (const auto render_layer = Application::GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto scene_camera = GetSceneCamera();
  DrawGizmoStrands(strands, scene_camera, color, model, size, gizmo_settings);
}

void EditorLayer::DrawGizmoCubes(const std::shared_ptr<ParticleInfoList>& instanced_data, const glm::mat4& model,
                                 const float& size, const GizmoSettings& gizmo_settings) {
  DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), instanced_data, model, size,
                                gizmo_settings);
}

void EditorLayer::DrawGizmoCube(const glm::vec4& color, const glm::mat4& model, const float& size,
                                const GizmoSettings& gizmo_settings) {
  DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), color, model, size, gizmo_settings);
}

void EditorLayer::DrawGizmoSpheres(const std::shared_ptr<ParticleInfoList>& instanced_data, const glm::mat4& model,
                                   const float& size, const GizmoSettings& gizmo_settings) {
  DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"), instanced_data, model, size,
                                gizmo_settings);
}

void EditorLayer::DrawGizmoSphere(const glm::vec4& color, const glm::mat4& model, const float& size,
                                  const GizmoSettings& gizmo_settings) {
  DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"), color, model, size, gizmo_settings);
}

void EditorLayer::DrawGizmoCylinders(const std::shared_ptr<ParticleInfoList>& instanced_data, const glm::mat4& model,
                                     const float& size, const GizmoSettings& gizmo_settings) {
  DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), instanced_data, model, size,
                                gizmo_settings);
}

void EditorLayer::DrawGizmoCylinder(const glm::vec4& color, const glm::mat4& model, const float& size,
                                    const GizmoSettings& gizmo_settings) {
  DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), color, model, size, gizmo_settings);
}
