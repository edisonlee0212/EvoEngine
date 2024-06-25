#include "SkinnedMeshRenderer.hpp"

#include "EditorLayer.hpp"
using namespace evo_engine;
void SkinnedMeshRenderer::RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, glm::vec4& color) {
  const auto scene = GetScene();
  const auto transform = scene->GetDataComponent<GlobalTransform>(GetOwner()).value;
  glm::vec3 size = skinned_mesh.Get<SkinnedMesh>()->bound_.Size() * 2.0f;
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
  editor_layer->DrawGizmoMesh(
      Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), color,
      transform * (glm::translate(skinned_mesh.Get<SkinnedMesh>()->bound_.Center()) * glm::scale(size)), 1,
      gizmo_settings);
}

void SkinnedMeshRenderer::UpdateBoneMatrices() {
  const auto scene = GetScene();
  const auto tmp = this->animator.Get<Animator>();
  if (!tmp)
    return;
  const auto tmp_mesh = skinned_mesh.Get<SkinnedMesh>();
  if (!tmp_mesh)
    return;
  if (rag_doll_) {
    if (rag_doll_freeze)
      return;

    bone_matrices->value.resize(tmp_mesh->bone_animator_indices.size());
    for (int i = 0; i < bound_entities_.size(); i++) {
      auto entity = bound_entities_[i].Get();
      if (entity.GetIndex() != 0) {
        rag_doll_transform_chain_[i] =
            scene->GetDataComponent<GlobalTransform>(entity).value * tmp->offset_matrices_[i];
      }
    }
    for (int i = 0; i < tmp_mesh->bone_animator_indices.size(); i++) {
      bone_matrices->value[i] = rag_doll_transform_chain_[tmp_mesh->bone_animator_indices[i]];
    }
  } else {
    if (tmp->bone_size_ == 0)
      return;
    bone_matrices->value.resize(tmp_mesh->bone_animator_indices.size());
    for (int i = 0; i < tmp_mesh->bone_animator_indices.size(); i++) {
      bone_matrices->value[i] = tmp->transform_chain_[tmp_mesh->bone_animator_indices[i]];
    }
  }
}

bool SkinnedMeshRenderer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<Animator>(animator, "Animator"))
    changed = true;
  if (ImGui::Checkbox("Cast shadow##SkinnedMeshRenderer", &cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<SkinnedMesh>(skinned_mesh, "Skinned Mesh"))
    changed = true;
  if (skinned_mesh.Get<SkinnedMesh>()) {
    if (ImGui::TreeNode("Skinned Mesh:##SkinnedMeshRenderer")) {
      static bool display_bound = true;
      ImGui::Checkbox("Display bounds##SkinnedMeshRenderer", &display_bound);
      if (display_bound) {
        static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
        ImGui::ColorEdit4("Color:##SkinnedMeshRenderer", static_cast<float*>(static_cast<void*>(&display_bound_color)));
        RenderBound(editor_layer, display_bound_color);
      }
      ImGui::TreePop();
    }
  }
  if (const auto amt = animator.Get<Animator>()) {
    static bool debug_render_bones = true;
    static float debug_render_bones_size = 0.01f;
    static glm::vec4 debug_render_bones_color = glm::vec4(1, 0, 0, 0.5);
    ImGui::Checkbox("Display bones", &debug_render_bones);
    if (amt && debug_render_bones) {
      ImGui::DragFloat("Size", &debug_render_bones_size, 0.01f, 0.01f, 3.0f);
      ImGui::ColorEdit4("Color", &debug_render_bones_color.x);
      auto scene = GetScene();
      auto owner = GetOwner();
      const auto self_scale = scene->GetDataComponent<GlobalTransform>(owner).GetScale();
      std::vector<ParticleInfo> debug_rendering_matrices;
      GlobalTransform ltw;

      static std::shared_ptr<ParticleInfoList> particle_info_list;
      if (!particle_info_list)
        particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
      if (!rag_doll_) {
        debug_rendering_matrices.resize(amt->transform_chain_.size());
        Jobs::RunParallelFor(amt->transform_chain_.size(), [&](unsigned i) {
          debug_rendering_matrices.at(i).instance_matrix.value = amt->transform_chain_.at(i);
          debug_rendering_matrices.at(i).instance_color = debug_render_bones_color;
        });

        ltw = scene->GetDataComponent<GlobalTransform>(owner);
      } else {
        debug_rendering_matrices.resize(rag_doll_transform_chain_.size());
        Jobs::RunParallelFor(rag_doll_transform_chain_.size(), [&](unsigned i) {
          debug_rendering_matrices.at(i).instance_matrix.value = rag_doll_transform_chain_.at(i);
          debug_rendering_matrices.at(i).instance_color = debug_render_bones_color;
        });
      }
      for (int index = 0; index < debug_rendering_matrices.size(); index++) {
        debug_rendering_matrices[index].instance_matrix.value = debug_rendering_matrices[index].instance_matrix.value *
                                                              glm::inverse(amt->offset_matrices_[index]) *
                                                              glm::inverse(glm::scale(self_scale));
      }
      particle_info_list->SetParticleInfos(debug_rendering_matrices);
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"), particle_info_list,
                                                 ltw.value, debug_render_bones_size);
    }

    if (ImGui::Checkbox("RagDoll", &rag_doll_)) {
      if (rag_doll_) {
        SetRagDoll(rag_doll_);
      }
      changed = true;
    }
    if (rag_doll_) {
      ImGui::Checkbox("Freeze", &rag_doll_freeze);

      if (ImGui::TreeNode("RagDoll")) {
        for (int i = 0; i < bound_entities_.size(); i++) {
          if (editor_layer->DragAndDropButton(bound_entities_[i], "Bone: " + amt->names_[i])) {
            auto entity = bound_entities_[i].Get();
            SetRagDollBoundEntity(i, entity);
            changed = true;
          }
        }
        ImGui::TreePop();
      }
    }
  }
  return changed;
}

void SkinnedMeshRenderer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "cast_shadow" << cast_shadow;

  animator.Save("animator", out);
  skinned_mesh.Save("skinned_mesh", out);
  material.Save("material", out);

  out << YAML::Key << "rag_doll_" << YAML::Value << rag_doll_;
  out << YAML::Key << "rag_doll_freeze" << YAML::Value << rag_doll_freeze;

  if (!bound_entities_.empty()) {
    out << YAML::Key << "bound_entities_" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < bound_entities_.size(); i++) {
      out << YAML::BeginMap;
      bound_entities_[i].Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  if (!rag_doll_transform_chain_.empty()) {
    out << YAML::Key << "rag_doll_transform_chain_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(rag_doll_transform_chain_.data()),
                        rag_doll_transform_chain_.size() * sizeof(glm::mat4));
  }
}

void SkinnedMeshRenderer::Deserialize(const YAML::Node& in) {
  cast_shadow = in["cast_shadow"].as<bool>();

  animator.Load("animator", in, GetScene());
  skinned_mesh.Load("skinned_mesh", in);
  material.Load("material", in);

  rag_doll_ = in["rag_doll_"].as<bool>();
  rag_doll_freeze = in["rag_doll_freeze"].as<bool>();
  if (auto in_bound_entities = in["bound_entities_"]) {
    for (const auto& i : in_bound_entities) {
      EntityRef ref;
      ref.Deserialize(i);
      bound_entities_.push_back(ref);
    }
  }

  if (in["rag_doll_transform_chain_"]) {
    const auto chains = in["rag_doll_transform_chain_"].as<YAML::Binary>();
    rag_doll_transform_chain_.resize(chains.size() / sizeof(glm::mat4));
    std::memcpy(rag_doll_transform_chain_.data(), chains.data(), chains.size());
  }
}
void SkinnedMeshRenderer::OnCreate() {
  bone_matrices = std::make_shared<BoneMatrices>();
  SetEnabled(true);
}
void SkinnedMeshRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
void SkinnedMeshRenderer::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  animator.Relink(map, scene);
  for (auto& i : bound_entities_) {
    i.Relink(map);
  }
}
void SkinnedMeshRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(skinned_mesh);
  list.push_back(material);
}
bool SkinnedMeshRenderer::RagDoll() const {
  return rag_doll_;
}
void SkinnedMeshRenderer::SetRagDoll(bool value) {
  const auto amt = animator.Get<Animator>();
  if (value && !amt) {
    EVOENGINE_ERROR("Failed! No animator!");
    return;
  }
  rag_doll_ = value;
  if (rag_doll_) {
    const auto scene = GetScene();
    // Resize entities
    bound_entities_.resize(amt->transform_chain_.size());
    // Copy current transform chain
    rag_doll_transform_chain_ = amt->transform_chain_;
    const auto ltw = scene->GetDataComponent<GlobalTransform>(GetOwner()).value;
    for (auto& i : rag_doll_transform_chain_) {
      i = ltw * i;
    }
  }
}
void SkinnedMeshRenderer::SetRagDollBoundEntity(int index, const Entity& entity, bool reset_transform) {
  if (!rag_doll_) {
    EVOENGINE_ERROR("Not ragdoll!");
    return;
  }
  if (index >= bound_entities_.size()) {
    EVOENGINE_ERROR("Index exceeds limit!");
    return;
  }
  if (const auto scene = GetScene(); scene->IsEntityValid(entity)) {
    if (const auto amt = animator.Get<Animator>()) {
      if (reset_transform) {
        GlobalTransform global_transform;
        global_transform.value = rag_doll_transform_chain_[index] * glm::inverse(amt->offset_matrices_[index]);
        scene->SetDataComponent(entity, global_transform);
      }
    }
    bound_entities_[index] = entity;
  }
}
void SkinnedMeshRenderer::SetRagDollBoundEntities(const std::vector<Entity>& entities, bool reset_transform) {
  if (!rag_doll_) {
    EVOENGINE_ERROR("Not ragdoll!");
    return;
  }
  for (int i = 0; i < entities.size(); i++) {
    SetRagDollBoundEntity(i, entities[i], reset_transform);
  }
}
size_t SkinnedMeshRenderer::GetRagDollBoneSize() const {
  if (!rag_doll_) {
    EVOENGINE_ERROR("Not ragdoll!");
    return 0;
  }
  return bound_entities_.size();
}
void SkinnedMeshRenderer::OnDestroy() {
  rag_doll_transform_chain_.clear();
  bound_entities_.clear();
  animator.Clear();
  bone_matrices.reset();
  skinned_mesh.Clear();
  material.Clear();
  rag_doll_ = false;
  rag_doll_freeze = false;
  cast_shadow = true;
}
