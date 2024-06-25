#include "StrandsRenderer.hpp"
#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "Prefab.hpp"
#include "Strands.hpp"
using namespace evo_engine;
void StrandsRenderer::RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, glm::vec4& color) {
  const auto transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;
  glm::vec3 size = strands.Get<Strands>()->bound_.Size();
  if (size.x < 0.01f)
    size.x = 0.01f;
  if (size.z < 0.01f)
    size.z = 0.01f;
  if (size.y < 0.01f)
    size.y = 0.01f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
  gizmo_settings.draw_settings.line_width = 3.0f;
  editor_layer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), color,
                              transform * (glm::translate(strands.Get<Strands>()->bound_.Center()) * glm::scale(size)),
                              1, gizmo_settings);
}

bool StrandsRenderer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast shadow##StrandsRenderer:", &cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<Strands>(strands, "Strands"))
    changed = true;
  if (strands.Get<Strands>()) {
    if (ImGui::TreeNode("Strands##StrandsRenderer")) {
      static bool display_bound = true;
      ImGui::Checkbox("Display bounds##StrandsRenderer", &display_bound);
      if (display_bound) {
        static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
        ImGui::ColorEdit4("Color:##StrandsRenderer", (float*)(void*)&display_bound_color);
        RenderBound(editor_layer, display_bound_color);
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

void StrandsRenderer::OnCreate() {
  SetEnabled(true);
}

void StrandsRenderer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "cast_shadow" << cast_shadow;

  strands.Save("strands", out);
  material.Save("material", out);
}

void StrandsRenderer::Deserialize(const YAML::Node& in) {
  cast_shadow = in["cast_shadow"].as<bool>();

  strands.Load("strands", in);
  material.Load("material", in);
}
void StrandsRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
void StrandsRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(strands);
  list.push_back(material);
}
void StrandsRenderer::OnDestroy() {
  strands.Clear();
  material.Clear();

  material.Clear();
  cast_shadow = true;
}
