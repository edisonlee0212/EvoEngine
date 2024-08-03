#include "MeshRenderer.hpp"

#include "EditorLayer.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "Resources.hpp"
#include "Transform.hpp"
using namespace evo_engine;

void MeshRenderer::RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const glm::vec4& color) {
  const auto scene = GetScene();
  const auto transform = scene->GetDataComponent<GlobalTransform>(GetOwner()).value;
  glm::vec3 size = mesh.Get<Mesh>()->GetBound().Size() * 2.0f;
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
  gizmo_settings.draw_settings.line_width = 5.0f;
  editor_layer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), color,
                              transform * (glm::translate(mesh.Get<Mesh>()->GetBound().Center()) * glm::scale(size)), 1,
                              gizmo_settings);
}

bool MeshRenderer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast shadow##MeshRenderer", &cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<Mesh>(mesh, "Mesh"))
    changed = true;
  if (mesh.Get<Mesh>()) {
    if (ImGui::TreeNodeEx("Mesh##MeshRenderer", ImGuiTreeNodeFlags_DefaultOpen)) {
      static bool display_bound = true;
      ImGui::Checkbox("Display bounds##MeshRenderer", &display_bound);
      if (display_bound) {
        static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.1f);
        ImGui::ColorEdit4("Color:##MeshRenderer", (float*)(void*)&display_bound_color);
        RenderBound(editor_layer, display_bound_color);
      }
      ImGui::TreePop();
    }
  }

  return changed;
}

void MeshRenderer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "cast_shadow" << cast_shadow;

  mesh.Save("mesh", out);
  material.Save("material", out);
}

void MeshRenderer::Deserialize(const YAML::Node& in) {
  cast_shadow = in["cast_shadow"].as<bool>();

  mesh.Load("mesh", in);
  material.Load("material", in);
}
void MeshRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
void MeshRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(mesh);
  list.push_back(material);
}
void MeshRenderer::OnDestroy() {
  mesh.Clear();
  material.Clear();

  cast_shadow = true;
}
