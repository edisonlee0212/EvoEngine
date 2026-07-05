#include "DigitalAgricultureSerializationAdapters.hpp"

#include "DigitalAgricultureInspectionAdapters.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
#ifdef CUDA_MODULE_SERVICE
#  include "BtfMaterial.hpp"
#  include "BtfMeshRenderer.hpp"
#  include "CBTFGroup.hpp"
#endif
using namespace digital_agriculture_package;

void Sorghum::ClearGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Panicle Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Leaf Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Stem Mesh") {
      scene->DeleteEntity(child);
    }
  }
}

void Sorghum::GenerateGeometryEntities(const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings) {
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  if (!sorghum_layer)
    return;
  auto target_sorghum_descriptor = sorghum_descriptor.Get<SorghumDescriptor>();
  if (!target_sorghum_descriptor) {
    if (const auto target_sorghum_generator = sorghum_generator.Get<SorghumGenerator>()) {
      sorghum_descriptor = target_sorghum_descriptor = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
      target_sorghum_generator->Apply(target_sorghum_descriptor);
    } else if (const auto target_sorghum_growth_stages = sorghum_growth_stages.Get<SorghumGrowthStages>()) {
      sorghum_descriptor = target_sorghum_descriptor = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
      target_sorghum_growth_stages->Apply(target_sorghum_descriptor, 1.f);
    } else if (const auto target_sorghum_state = sorghum_state.Get<SorghumState>()) {
      sorghum_descriptor = target_sorghum_descriptor = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
      target_sorghum_state->Apply(target_sorghum_descriptor);
    }
  }

  if (!target_sorghum_descriptor) {
    EVOENGINE_ERROR(
        "Failed to generate sorghum geometry: No SorghumDescriptor/SorghumGenerator/SorghumGrowthStages/SorghumState "
        "provided.")
    return;
  }

  if (target_sorghum_descriptor->stem.spline.segments.empty()) {
    EVOENGINE_ERROR("Failed to generate sorghum geometry: No stem.")
    return;
  }
  ClearGeometryEntities();
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto copy_material = [](const std::shared_ptr<Material>& target, const std::shared_ptr<Material>& source) {
    if (!target || !source) {
      return;
    }
    target->SetGltfMaterialData(source->material_data);
    target->RefTextureRefs() = source->PeekTextureRefs();
    target->MarkDirty();
  };
  if (sorghum_mesh_generator_settings.enable_panicle && target_sorghum_descriptor->panicle.seed_amount > 0) {
    const auto panicle_entity = scene->CreateEntity("Panicle Mesh");
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(panicle_entity).lock();
    const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    const auto particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    particles->mesh = mesh;
    particles->material = material;
    const auto panicle_material = sorghum_layer->panicle_material.Get<Material>();
    copy_material(material, panicle_material);
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    target_sorghum_descriptor->panicle.GenerateGeometry(target_sorghum_descriptor->stem.spline.segments.back().position,
                                                        vertices, indices, particle_info_list);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);

    particles->particle_info_list = particle_info_list;
    scene->SetParent(panicle_entity, owner);
  }
  if (sorghum_mesh_generator_settings.enable_stem) {
    const auto stem_entity = scene->CreateEntity("Stem Mesh");
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(stem_entity).lock();
    const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
    const auto stem_material = sorghum_layer->leaf_material.Get<Material>();
    copy_material(material, stem_material);
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    target_sorghum_descriptor->stem.GenerateGeometry(vertices, indices);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);
    scene->SetParent(stem_entity, owner);
  }
  if (sorghum_mesh_generator_settings.enable_leaves) {
    if (sorghum_mesh_generator_settings.leaf_separated) {
#ifdef CUDA_MODULE_SERVICE
      const auto btf_group = sorghum_layer->leaf_cbtf_group.Get<CBTFGroup>();
#endif
      if (sorghum_mesh_generator_settings.single_leaf_index != -1) {
        if (sorghum_mesh_generator_settings.single_leaf_index < target_sorghum_descriptor->leaves.size()) {
          const auto& leaf_state = target_sorghum_descriptor->leaves[sorghum_mesh_generator_settings.single_leaf_index];
          const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
          const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
          if (sorghum_layer->enable_compressed_btf) {
#ifdef CUDA_MODULE_SERVICE
            if (btf_group) {
              const auto btf_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(leaf_entity).lock();
              btf_renderer->mesh = mesh;
              btf_renderer->btf = btf_group->GetRandom();
            }
#endif
          } else {
            const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leaf_entity).lock();
            const auto material = AssetManager::CreateTemporaryAsset<Material>();
            mesh_renderer->mesh = mesh;
            mesh_renderer->material = material;
            const auto leaf_material = sorghum_layer->leaf_material.Get<Material>();
            copy_material(material, leaf_material);
          }
          std::vector<Vertex> vertices;
          std::vector<unsigned int> indices;
          leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, false);
          if (sorghum_mesh_generator_settings.bottom_face) {
            leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, true);
          }
          VertexAttributes attributes{};
          attributes.tex_coord = true;
          mesh->SetVertices(attributes, vertices, indices);
          scene->SetParent(leaf_entity, owner);
        }
      } else {
        for (const auto& leaf_state : target_sorghum_descriptor->leaves) {
          const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
          const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
          if (sorghum_layer->enable_compressed_btf) {
#ifdef CUDA_MODULE_SERVICE
            if (btf_group) {
              const auto btf_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(leaf_entity).lock();
              btf_renderer->mesh = mesh;
              btf_renderer->btf = btf_group->GetRandom();
            }
#endif
          } else {
            const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leaf_entity).lock();
            const auto material = AssetManager::CreateTemporaryAsset<Material>();
            mesh_renderer->mesh = mesh;
            mesh_renderer->material = material;
            const auto leaf_material = sorghum_layer->leaf_material.Get<Material>();
            copy_material(material, leaf_material);
          }
          std::vector<Vertex> vertices;
          std::vector<unsigned int> indices;
          leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, false);
          if (sorghum_mesh_generator_settings.bottom_face) {
            leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, true);
          }
          VertexAttributes attributes{};
          attributes.tex_coord = true;
          mesh->SetVertices(attributes, vertices, indices);
          scene->SetParent(leaf_entity, owner);
        }
      }
    } else {
      const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
      const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
      if (sorghum_layer->enable_compressed_btf) {
#ifdef CUDA_MODULE_SERVICE
        if (const auto btf_group = sorghum_layer->leaf_cbtf_group.Get<CBTFGroup>()) {
          const auto btf_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(leaf_entity).lock();
          btf_renderer->mesh = mesh;
          btf_renderer->btf = btf_group->GetRandom();
        }
#endif
      } else {
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leaf_entity).lock();
        const auto material = AssetManager::CreateTemporaryAsset<Material>();
        mesh_renderer->mesh = mesh;
        mesh_renderer->material = material;
        const auto leaf_material = sorghum_layer->leaf_material.Get<Material>();
        copy_material(material, leaf_material);
      }
      std::vector<Vertex> vertices;
      std::vector<unsigned int> indices;
      for (const auto& leaf_state : target_sorghum_descriptor->leaves) {
        leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, false);
        if (sorghum_mesh_generator_settings.bottom_face) {
          leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, true);
        }
      }
      VertexAttributes attributes{};
      attributes.tex_coord = true;
      mesh->SetVertices(attributes, vertices, indices);
      scene->SetParent(leaf_entity, owner);
    }
  }
}

void Sorghum::OnDestroy() {
  sorghum_descriptor.Clear();
  sorghum_generator.Clear();
  sorghum_state.Clear();
  sorghum_growth_stages.Clear();
}

void digital_agriculture_package::SerializeSorghum(YAML::Emitter& out, const Sorghum& target) {
  target.sorghum_descriptor.Save("sorghum_descriptor", out);
  target.sorghum_generator.Save("sorghum_generator", out);
  target.sorghum_state.Save("sorghum_state", out);

  target.sorghum_growth_stages.Save("sorghum_growth_stages", out);
}

void digital_agriculture_package::DeserializeSorghum(const YAML::Node& in, Sorghum& target) {
  target.sorghum_descriptor.Load("sorghum_descriptor", in);
  target.sorghum_growth_stages.Load("sorghum_growth_stages", in);
  target.sorghum_state.Load("sorghum_state", in);
  target.sorghum_generator.Load("sorghum_generator", in);
}

bool digital_agriculture_package::InspectSorghum(InspectorContext& context, Sorghum& sorghum) {
  const auto& editor_layer = context.editor_layer;
  auto& sorghum_generator = sorghum.sorghum_generator;
  auto& sorghum_growth_stages = sorghum.sorghum_growth_stages;
  auto& sorghum_state = sorghum.sorghum_state;
  auto& sorghum_descriptor = sorghum.sorghum_descriptor;
  bool changed = false;
  if (editor_layer->DragAndDropButton<SorghumGenerator>(sorghum_generator, "SorghumGenerator"))
    changed = true;
  if (editor_layer->DragAndDropButton<SorghumGrowthStages>(sorghum_growth_stages, "SorghumGrowthStages"))
    changed = true;

  if (editor_layer->DragAndDropButton<SorghumState>(sorghum_state, "SorghumState"))
    changed = true;

  if (editor_layer->DragAndDropButton<SorghumDescriptor>(sorghum_descriptor, "SorghumDescriptor"))
    changed = true;

  if (ImGui::Button("Form meshes")) {
    sorghum.GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  }

  if (const auto ssg = sorghum_generator.Get<SorghumGenerator>()) {
    if (ImGui::TreeNode("Sorghum Descriptor settings")) {
      static int seed = 0;
      if (ImGui::DragInt("Seed", &seed)) {
        auto sd = sorghum_descriptor.Get<SorghumDescriptor>();
        if (!sd) {
          sd = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
          sorghum_descriptor = sd;
        }
        ssg->Apply(sd, seed);
        sorghum.GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
      }
      ImGui::TreePop();
    }
  }
  if (const auto sgs = sorghum_growth_stages.Get<SorghumGrowthStages>()) {
    if (ImGui::TreeNode("Sorghum Growth Descriptor settings")) {
      static float time = 0.0f;
      if (ImGui::SliderFloat("Time", &time, 0.0f, sgs->GetCurrentEndTime())) {
        time = glm::clamp(time, 0.0f, sgs->GetCurrentEndTime());
        auto sd = sorghum_descriptor.Get<SorghumDescriptor>();
        if (!sd) {
          sd = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
          sorghum_descriptor = sd;
        }
        sgs->Apply(sd, time);
        sorghum.GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
      }
      ImGui::TreePop();
    }
  }
  static bool debug_rendering = false;
  ImGui::Checkbox("Debug", &debug_rendering);
  if (debug_rendering) {
    static float node_render_size = .5f;
    if (ImGui::TreeNode("Debug settings")) {
      ImGui::DragFloat("Node size", &node_render_size, 0.01f, 0.0f, 1.f);
      ImGui::TreePop();
    }
    static Entity previous_referenced_entity;
    static std::shared_ptr<ParticleInfoList> node_debug_info_list;
    if (!node_debug_info_list)
      node_debug_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    constexpr bool show_all_node = false;
    if (show_all_node) {
      if (const auto sd = sorghum_descriptor.Get<SorghumDescriptor>()) {
        std::vector<ParticleInfo> particle_infos;
        const auto owner = sorghum.GetOwner();
        const auto scene = sorghum.GetScene();
        const auto plant_position = scene->GetDataComponent<GlobalTransform>(owner).GetPosition();
        for (const auto& leaf_state : sd->leaves) {
          const auto start_index = particle_infos.size();
          particle_infos.resize(start_index + leaf_state.spline.segments.size());
          for (int i = 0; i < leaf_state.spline.segments.size(); i++) {
            auto& matrix = particle_infos[start_index + i].instance_matrix;
            matrix.value = glm::translate(leaf_state.spline.segments.at(i).position + plant_position) *
                           glm::scale(glm::vec3(node_render_size * leaf_state.spline.segments.at(i).radius));
            particle_infos[start_index + i].instance_color =
                glm::vec4((leaf_state.index % 3) * 0.5f, ((leaf_state.index / 3) % 3) * 0.5f,
                          ((leaf_state.index / 9) % 3) * 0.5f, 1.0f);
          }
        }
        node_debug_info_list->SetParticleInfos(particle_infos);
      }
    } else {
      const auto owner = sorghum.GetOwner();
      if (ImGui::Button("Refresh leaf nodes") || previous_referenced_entity != owner) {
        if (const auto sd = sorghum_descriptor.Get<SorghumDescriptor>()) {
          std::vector<ParticleInfo> particle_infos;

          const auto scene = sorghum.GetScene();
          const auto plant_position = scene->GetDataComponent<GlobalTransform>(owner).GetPosition();
          for (const auto& leaf_state : sd->leaves) {
            SorghumSpline leaf_part;
            leaf_part.segments = leaf_state.spline.GetLeafPart();
            const auto segments = leaf_part.RebuildFixedSizeSegments(8);
            const auto start_index = particle_infos.size();
            particle_infos.resize(start_index + segments.size());
            for (int i = 0; i < segments.size(); i++) {
              auto& matrix = particle_infos[start_index + i].instance_matrix;
              matrix.value = glm::translate(segments.at(i).position + plant_position) *
                             glm::scale(glm::vec3(node_render_size * segments.at(i).radius));
              particle_infos[start_index + i].instance_color =
                  glm::vec4((leaf_state.index % 3) * 0.5f, ((leaf_state.index / 3) % 3) * 0.5f,
                            ((leaf_state.index / 9) % 3) * 0.5f, 1.0f);
            }
          }
          node_debug_info_list->SetParticleInfos(particle_infos);
        }
      }
    }
    editor_layer->DrawGizmoCubes(node_debug_info_list);
  }

  return changed;
}

void Sorghum::CollectAssetRef(std::vector<AssetRef>& list) {
  if (sorghum_descriptor.Get<SorghumDescriptor>())
    list.push_back(sorghum_descriptor);
  if (sorghum_growth_stages.Get<SorghumGrowthStages>())
    list.push_back(sorghum_growth_stages);
  if (sorghum_generator.Get<SorghumGenerator>())
    list.push_back(sorghum_generator);
  if (sorghum_state.Get<SorghumState>())
    list.push_back(sorghum_state);
}
uint32_t Sorghum::GetLeafSize() {
  if (const auto sd = sorghum_descriptor.Get<SorghumDescriptor>())
    return sd->leaves.size();
  if (const auto ss = sorghum_state.Get<SorghumState>())
    return ss->leaves.size();
  EVOENGINE_ERROR("GetLeafSize failed: SorghumDescriptor or SorghumState missing!")
  return 0;
}
