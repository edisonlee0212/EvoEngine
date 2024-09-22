#include "Sorghum.hpp"

#include "SorghumLayer.hpp"
#include "SorghumDescriptorGenerator.hpp"

using namespace digital_agriculture_plugin;

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
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  if (!sorghum_layer)
    return;
  const auto sorghum_state = sorghum_descriptor.Get<SorghumDescriptor>();
  if (!sorghum_state) {
    if (const auto sdg = sorghum_state_generator.Get<SorghumDescriptorGenerator>()) {
      sdg->Apply(sorghum_state);
    } else if (const auto sorghum_growth_descriptor = sorghum_growth_stages.Get<SorghumGrowthStages>()) {
      sorghum_growth_descriptor->Apply(sorghum_state, 1.f);
    }
  }

  if (!sorghum_state)
    return;
  if (sorghum_state->stem.spline.segments.empty())
    return;
  ClearGeometryEntities();
  const auto scene = GetScene();
  const auto owner = GetOwner();
  if (sorghum_mesh_generator_settings.enable_panicle && sorghum_state->panicle.seed_amount > 0) {
    const auto panicle_entity = scene->CreateEntity("Panicle Mesh");
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(panicle_entity).lock();
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    particles->mesh = mesh;
    particles->material = material;
    const auto panicle_material = sorghum_layer->panicle_material.Get<Material>();
    // material->SetAlbedoTexture(panicleMaterial->GetAlbedoTexture());
    // material->SetNormalTexture(panicleMaterial->GetNormalTexture());
    // material->SetRoughnessTexture(panicleMaterial->GetRoughnessTexture());
    // material->SetMetallicTexture(panicleMaterial->GetMetallicTexture());
    material->material_properties = panicle_material->material_properties;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    sorghum_state->panicle.GenerateGeometry(sorghum_state->stem.spline.segments.back().position, vertices,
                                             indices, particle_info_list);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);

    particles->particle_info_list = particle_info_list;
    scene->SetParent(panicle_entity, owner);
  }
  if (sorghum_mesh_generator_settings.enable_stem) {
    const auto stem_entity = scene->CreateEntity("Stem Mesh");
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(stem_entity).lock();
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
    const auto  stem_material = sorghum_layer->leaf_material.Get<Material>();
    // material->SetAlbedoTexture(stemMaterial->GetAlbedoTexture());
    // material->SetNormalTexture(stemMaterial->GetNormalTexture());
    // material->SetRoughnessTexture(stemMaterial->GetRoughnessTexture());
    // material->SetMetallicTexture(stemMaterial->GetMetallicTexture());
    material->material_properties = stem_material->material_properties;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    sorghum_state->stem.GenerateGeometry(vertices, indices);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);
    scene->SetParent(stem_entity, owner);
  }
  if (sorghum_mesh_generator_settings.enable_leaves) {
    if (sorghum_mesh_generator_settings.leaf_separated) {
      for (const auto& leaf_state : sorghum_state->leaves) {
        const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leaf_entity).lock();
        const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
        const auto material = ProjectManager::CreateTemporaryAsset<Material>();
        mesh_renderer->mesh = mesh;
        mesh_renderer->material = material;
        const auto leaf_material = sorghum_layer->leaf_material.Get<Material>();
        // material->SetAlbedoTexture(leafMaterial->GetAlbedoTexture());
        // material->SetNormalTexture(leafMaterial->GetNormalTexture());
        // material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
        // material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
        material->material_properties = leaf_material->material_properties;
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        leaf_state.GenerateGeometry(vertices, indices, false, 0.f);
        if (sorghum_mesh_generator_settings.bottom_face) {
          leaf_state.GenerateGeometry(vertices, indices, true, sorghum_mesh_generator_settings.leaf_thickness);
        }
        VertexAttributes attributes{};
        attributes.tex_coord = true;
        mesh->SetVertices(attributes, vertices, indices);
        scene->SetParent(leaf_entity, owner);
      }
    } else {
      const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leaf_entity).lock();
      const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      mesh_renderer->mesh = mesh;
      mesh_renderer->material = material;
      const auto leaf_material = sorghum_layer->leaf_material.Get<Material>();
      material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
      // material->SetNormalTexture(leafMaterial->GetNormalTexture());
      // material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
      // material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
      material->material_properties = leaf_material->material_properties;
      std::vector<Vertex> vertices;
      std::vector<unsigned int> indices;
      for (const auto& leaf_state : sorghum_state->leaves) {
        leaf_state.GenerateGeometry(vertices, indices, false, 0.f);
        if (sorghum_mesh_generator_settings.bottom_face) {
          leaf_state.GenerateGeometry(vertices, indices, true, sorghum_mesh_generator_settings.leaf_thickness);
        }
      }
      VertexAttributes attributes{};
      attributes.tex_coord = true;
      mesh->SetVertices(attributes, vertices, indices);
      scene->SetParent(leaf_entity, owner);
    }
  }
}

void Sorghum::Serialize(YAML::Emitter& out) const {
  sorghum_descriptor.Save("sorghum_descriptor", out);
  sorghum_state_generator.Save("sorghum_state_generator", out);
  sorghum_growth_stages.Save("sorghum_growth_stages", out);
}

void Sorghum::Deserialize(const YAML::Node& in) {
  sorghum_descriptor.Load("sorghum_descriptor", in);
  sorghum_growth_stages.Load("sorghum_growth_stages", in);
  sorghum_state_generator.Load("sorghum_state_generator", in);
}

bool Sorghum::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<SorghumDescriptorGenerator>(sorghum_state_generator, "SorghumDescriptorGenerator"))
    changed = true;
  if (editor_layer->DragAndDropButton<SorghumGrowthStages>(sorghum_growth_stages, "SorghumGrowthStages"))
    changed = true;
  if (editor_layer->DragAndDropButton<SorghumDescriptor>(sorghum_descriptor, "SorghumDescriptor"))
    changed = true;

  if (ImGui::Button("Form meshes")) {
    GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  }

  if (const auto ssg = sorghum_state_generator.Get<SorghumDescriptorGenerator>()) {
    if (ImGui::TreeNode("Sorghum Descriptor settings")) {
      static int seed = 0;
      if (ImGui::DragInt("Seed", &seed)) {
        auto sd = sorghum_descriptor.Get<SorghumDescriptor>();
        if (!sd) {
          sd = ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();
          sorghum_descriptor = sd;
        }
        ssg->Apply(sd, seed);
        GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
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
          sd = ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();
          sorghum_descriptor = sd;
        }
        sgs->Apply(sd, time);
        GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
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
    static std::shared_ptr<ParticleInfoList> node_debug_info_list;
    if (!node_debug_info_list)
      node_debug_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    constexpr bool show_all_node = false;
    if (show_all_node){
      if (const auto sd = sorghum_descriptor.Get<SorghumDescriptor>()) {
        std::vector<ParticleInfo> particle_infos;
        const auto owner = GetOwner();
        const auto scene = GetScene();
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
    }else {
      if (ImGui::Button("Refresh leaf nodes")) {
        if (const auto sd = sorghum_descriptor.Get<SorghumDescriptor>()) {
          std::vector<ParticleInfo> particle_infos;
          const auto owner = GetOwner();
          const auto scene = GetScene();
          const auto plant_position = scene->GetDataComponent<GlobalTransform>(owner).GetPosition();
          for (const auto& leaf_state : sd->leaves) {
            SorghumSpline leaf_part;
            leaf_part.segments = leaf_state.spline.GetLeafPart();
            const auto segments = leaf_part.RebuildFixedSizeSegments(8);
            const auto start_index = particle_infos.size();
            particle_infos.resize(start_index + segments.size());
            for(int i = 0; i < segments.size(); i++) {
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
  if (sorghum_state_generator.Get<SorghumDescriptorGenerator>())
    list.push_back(sorghum_state_generator);
}
