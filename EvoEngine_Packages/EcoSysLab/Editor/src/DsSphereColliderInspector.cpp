#include "DsColliders.hpp"
#include "DynamicStrandsComponentInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "Shader.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
void eco_sys_lab_package::DrawColliderBound(const DsSphereCollider& target,
                                            const std::shared_ptr<EditorLayer>& editor_layer,
                                            const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) {
  const auto scene = target.GetScene();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(target.GetOwner());
  const auto scale = global_transform.GetScale();
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
  gizmo_settings.draw_settings.line_width = 1.0f;
  gizmo_settings.depth_test = true;
  editor_layer->DrawGizmoMesh(
      Resources::GetInstance().GetPrimitives().sphere, editor_camera, color,
      glm::translate(global_transform.GetPosition()) *
          glm::scale(glm::vec3(glm::max(0.001f, target.radius * 2.f)) * glm::max(glm::max(scale.x, scale.y), scale.z)),
      1, gizmo_settings);
}
bool DsSphereColliderInspector::Inspect(InspectorContext& context, DsSphereCollider& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::DragFloat("Radius", &target.radius, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }

  if (ImGui::DragFloat("Softness", &target.softness, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }

  ImGui::ColorEdit4("Bound Color:##DsBoxCollider", (float*)(void*)&target.bound_color);

  ImGui::Checkbox("Display bounds##DsBoxCollider", &display_bound);
  if (display_bound) {
    DrawColliderBound(target, editor_layer, editor_layer->GetSceneCamera(), target.bound_color);
  }

  return changed;
}
