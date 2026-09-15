#include "DsColliders.hpp"
#include "DynamicStrandsComponentInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "Shader.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
void eco_sys_lab_package::DrawColliderBound(const DsBoxCollider& target,
                                            const std::shared_ptr<EditorLayer>& editor_layer,
                                            const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) {
  const auto scene = target.GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(target.GetOwner());
  glm::vec3 size = target.scale * global_transform.GetScale() * 2.0f;
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.z < 0.001f)
    size.z = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
  gizmo_settings.draw_settings.line_width = 1.0f;
  gizmo_settings.depth_test = true;
  editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cube, editor_camera, color,
                              glm::translate(global_transform.GetPosition()) *
                                  glm::mat4_cast(global_transform.GetRotation()) * glm::scale(size),
                              1, gizmo_settings);
}
bool DsBoxColliderInspector::Inspect(InspectorContext& context, DsBoxCollider& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  if (editor_layer->DragAndDropButton<MeshRenderer>(mesh_renderer_ref, "Apply bound from Mesh Renderer")) {
    if (const auto mesh_renderer = mesh_renderer_ref.Get<MeshRenderer>()) {
      if (const auto mesh = mesh_renderer->mesh.Get<Mesh>()) {
        const auto& bound = mesh->GetBound();
        target.scale = bound.Size();
      }
    }
    mesh_renderer_ref.Clear();
  }

  if (ImGui::DragFloat3("Scale", &target.scale.x, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }

  if (ImGui::DragFloat("Softness", &target.softness, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Friction", &target.friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Rotational friction", &target.rotational_friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Velocity friction", &target.velocity_friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Angular velocity friction", &target.angular_velocity_friction, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  ImGui::ColorEdit4("Bound Color:##DsBoxCollider", (float*)(void*)&target.bound_color);

  ImGui::Checkbox("Display bounds##DsBoxCollider", &display_bound);
  if (display_bound) {
    DrawColliderBound(target, editor_layer, editor_layer->GetSceneCamera(), target.bound_color);
  }
  return changed;
}
