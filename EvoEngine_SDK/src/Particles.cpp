#include "Particles.hpp"
#include "EditorLayer.hpp"

using namespace evo_engine;

void Particles::OnCreate() {
  particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  bounding_box = Bound();
  SetEnabled(true);
}

void Particles::RecalculateBoundingBox() {
  const auto pil = particle_info_list.Get<ParticleInfoList>();
  if (!pil)
    return;
  if (pil->PeekParticleInfoList().empty()) {
    bounding_box.min = glm::vec3(0.0f);
    bounding_box.max = glm::vec3(0.0f);
    return;
  }
  auto min_bound = glm::vec3(FLT_MAX);
  auto max_bound = glm::vec3(FLT_MAX);
  const auto mesh_bound = mesh.Get<Mesh>()->GetBound();
  for (const auto& i : pil->PeekParticleInfoList()) {
    const glm::vec3 center = i.instance_matrix.value * glm::vec4(mesh_bound.Center(), 1.0f);
    const glm::vec3 size = glm::vec4(mesh_bound.Size(), 0) * i.instance_matrix.value / 2.0f;
    min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                         (glm::min)(min_bound.z, center.z - size.z));

    max_bound = glm::vec3((glm::max)(max_bound.x, center.x + size.x), (glm::max)(max_bound.y, center.y + size.y),
                         (glm::max)(max_bound.z, center.z + size.z));
  }
  bounding_box.max = max_bound;
  bounding_box.min = min_bound;
}

bool Particles::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast shadow##Particles", &cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<Mesh>(mesh, "Mesh"))
    changed = true;
  if (editor_layer->DragAndDropButton<ParticleInfoList>(particle_info_list, "ParticleInfoList"))
    changed = true;

  if (const auto pil = particle_info_list.Get<ParticleInfoList>()) {
    ImGui::Text(
        ("Instance count##Particles" + std::to_string(pil->PeekParticleInfoList().size())).c_str());
    if (ImGui::Button("Calculate bounds##Particles")) {
      RecalculateBoundingBox();
    }
    static bool display_bound;
    ImGui::Checkbox("Display bounds##Particles", &display_bound);
    if (display_bound) {
      static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
      ImGui::ColorEdit4("Color:##Particles", (float*)(void*)&display_bound_color);
      const auto transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner()).value;

      GizmoSettings gizmo_settings;
      gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
      gizmo_settings.draw_settings.blending = true;
      gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
      gizmo_settings.draw_settings.line_width = 3.0f;

      editor_layer->DrawGizmoCube(display_bound_color,
                                  transform * glm::translate(bounding_box.Center()) * glm::scale(bounding_box.Size()),
                                  1, gizmo_settings);
    }
  }
  return changed;
}

void Particles::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "cast_shadow" << cast_shadow;

  mesh.Save("mesh", out);
  material.Save("material", out);
  particle_info_list.Save("particle_info_list", out);
}

void Particles::Deserialize(const YAML::Node& in) {
  cast_shadow = in["cast_shadow"].as<bool>();

  mesh.Load("mesh", in);
  material.Load("material", in);
  particle_info_list.Load("particle_info_list", in);
}
void Particles::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}

void Particles::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(mesh);
  list.push_back(material);
  list.push_back(particle_info_list);
}
void Particles::OnDestroy() {
  mesh.Clear();
  material.Clear();
  particle_info_list.Clear();

  material.Clear();
  cast_shadow = true;
}