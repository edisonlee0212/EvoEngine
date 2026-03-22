#include "Maize.hpp"

#include "MaizeGenerator.hpp"
#include "MaizeLayer.hpp"
#include "MaizeGrowthStages.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include "BtfMaterial.hpp"
#  include "BtfMeshRenderer.hpp"
#  include "CBTFGroup.hpp"
#endif
using namespace digital_agriculture_plugin;

void Maize::ClearGeometryEntities() const {
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

void Maize::GenerateGeometryEntities(const MaizeMeshGeneratorSettings& maize_mesh_generator_settings) {
  const auto maize_layer = Application::GetLayer<MaizeLayer>();
  if (!maize_layer)
    return;
  auto target_maize_descriptor = maize_descriptor.Get<MaizeDescriptor>();
  if (!target_maize_descriptor) {
    if (const auto target_maize_generator = maize_generator.Get<MaizeGenerator>()) {
      maize_descriptor = target_maize_descriptor = AssetManager::CreateTemporaryAsset<MaizeDescriptor>();
      const auto temp_state = AssetManager::CreateTemporaryAsset<MaizeState>();
      target_maize_generator->Apply(temp_state); 
      target_maize_generator->ApplyGrowth(temp_state, plant_age); // Use component's plant_age
      temp_state->Apply(target_maize_descriptor);
    } else if (const auto target_maize_growth_stages = maize_growth_stages.Get<MaizeGrowthStages>()) {
      maize_descriptor = target_maize_descriptor = AssetManager::CreateTemporaryAsset<MaizeDescriptor>();
      target_maize_growth_stages->Apply(target_maize_descriptor, 1.f);
    } else if (const auto target_maize_state = maize_state.Get<MaizeState>()) {
      maize_descriptor = target_maize_descriptor = AssetManager::CreateTemporaryAsset<MaizeDescriptor>();
      target_maize_state->Apply(target_maize_descriptor);
    }
  }

  if (!target_maize_descriptor) {
    EVOENGINE_ERROR(
        "Failed to generate maize geometry: No MaizeDescriptor/MaizeGenerator/MaizeGrowthStages/MaizeState "
        "provided.")
    return;
  }

  if (target_maize_descriptor->stem.spline.segments.empty()) {
    EVOENGINE_ERROR("Failed to generate maize geometry: No stem.")
    return;
  }
  ClearGeometryEntities();
  const auto scene = GetScene();
  const auto owner = GetOwner();
  if (maize_mesh_generator_settings.enable_panicle && target_maize_descriptor->panicle.seed_amount > 0) {
    const auto panicle_entity = scene->CreateEntity("Panicle Mesh");
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(panicle_entity).lock();
    const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    const auto particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    particles->mesh = mesh;
    particles->material = material;
    const auto panicle_material = maize_layer->panicle_material.Get<Material>();
    material->SetAlbedoTexture(panicle_material->GetAlbedoTexture());
    material->SetNormalTexture(panicle_material->GetNormalTexture());
    material->SetRoughnessTexture(panicle_material->GetRoughnessTexture());
    material->SetMetallicTexture(panicle_material->GetMetallicTexture());
    material->material_properties = panicle_material->material_properties;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    target_maize_descriptor->panicle.GenerateGeometry(target_maize_descriptor->stem.spline.segments.back().position,
                                                        vertices, indices, particle_info_list);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);

    particles->particle_info_list = particle_info_list;
    scene->SetParent(panicle_entity, owner);
  }
  if (maize_mesh_generator_settings.enable_stem) {
    const auto stem_entity = scene->CreateEntity("Stem Mesh");
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(stem_entity).lock();
    const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
    const auto stem_material = maize_layer->leaf_material.Get<Material>();
    material->SetAlbedoTexture(stem_material->GetAlbedoTexture());
    material->SetNormalTexture(stem_material->GetNormalTexture());
    material->SetRoughnessTexture(stem_material->GetRoughnessTexture());
    material->SetMetallicTexture(stem_material->GetMetallicTexture());
    material->material_properties = stem_material->material_properties;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    target_maize_descriptor->stem.GenerateGeometry(vertices, indices);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);
    scene->SetParent(stem_entity, owner);
  }
  if (maize_mesh_generator_settings.enable_leaves) {
    if (maize_mesh_generator_settings.leaf_separated) {
#ifdef CUDA_MODULE_PLUGIN
      const auto btf_group = maize_layer->leaf_cbtf_group.Get<CBTFGroup>();
#endif
      if (maize_mesh_generator_settings.single_leaf_index != -1) {
        if (maize_mesh_generator_settings.single_leaf_index < target_maize_descriptor->leaves.size()) {
          const auto& leaf_state = target_maize_descriptor->leaves[maize_mesh_generator_settings.single_leaf_index];
          const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
          const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
          if (maize_layer->enable_compressed_btf) {
#ifdef CUDA_MODULE_PLUGIN
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
            const auto leaf_material = maize_layer->leaf_material.Get<Material>();
            material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
            material->SetNormalTexture(leaf_material->GetNormalTexture());
            material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
            material->SetMetallicTexture(leaf_material->GetMetallicTexture());
            material->material_properties = leaf_material->material_properties;
          }
          std::vector<Vertex> vertices;
          std::vector<unsigned int> indices;
          leaf_state.GenerateGeometry(vertices, indices, maize_mesh_generator_settings, false);
          if (maize_mesh_generator_settings.bottom_face) {
            leaf_state.GenerateGeometry(vertices, indices, maize_mesh_generator_settings, true);
          }
          VertexAttributes attributes{};
          attributes.tex_coord = true;
          mesh->SetVertices(attributes, vertices, indices);
          scene->SetParent(leaf_entity, owner);
        }
      } else {
        for (const auto& leaf_state : target_maize_descriptor->leaves) {
          const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
          const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
          if (maize_layer->enable_compressed_btf) {
#ifdef CUDA_MODULE_PLUGIN
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
            const auto leaf_material = maize_layer->leaf_material.Get<Material>();
            material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
            material->SetNormalTexture(leaf_material->GetNormalTexture());
            material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
            material->SetMetallicTexture(leaf_material->GetMetallicTexture());
            material->material_properties = leaf_material->material_properties;
          }
          std::vector<Vertex> vertices;
          std::vector<unsigned int> indices;
          leaf_state.GenerateGeometry(vertices, indices, maize_mesh_generator_settings, false);
          if (maize_mesh_generator_settings.bottom_face) {
            leaf_state.GenerateGeometry(vertices, indices, maize_mesh_generator_settings, true);
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
      if (maize_layer->enable_compressed_btf) {
#ifdef CUDA_MODULE_PLUGIN
        if (const auto btf_group = maize_layer->leaf_cbtf_group.Get<CBTFGroup>()) {
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
        const auto leaf_material = maize_layer->leaf_material.Get<Material>();
        material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
        material->SetNormalTexture(leaf_material->GetNormalTexture());
        material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
        material->SetMetallicTexture(leaf_material->GetMetallicTexture());
        material->material_properties = leaf_material->material_properties;
      }
      std::vector<Vertex> vertices;
      std::vector<unsigned int> indices;
      for (const auto& leaf_state : target_maize_descriptor->leaves) {
        leaf_state.GenerateGeometry(vertices, indices, maize_mesh_generator_settings, false);
        if (maize_mesh_generator_settings.bottom_face) {
          leaf_state.GenerateGeometry(vertices, indices, maize_mesh_generator_settings, true);
        }
      }
      VertexAttributes attributes{};
      attributes.tex_coord = true;
      mesh->SetVertices(attributes, vertices, indices);
      scene->SetParent(leaf_entity, owner);
    }
  }
}

void Maize::OnDestroy() {
  maize_descriptor.Clear();
  maize_generator.Clear();
  maize_growth_stages.Clear();
  maize_state.Clear();
}

void Maize::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "plant_age" << YAML::Value << plant_age;
  out << YAML::Key << "seed" << YAML::Value << seed;
  maize_descriptor.Save("maize_descriptor", out);
  maize_generator.Save("maize_generator", out);
  maize_state.Save("maize_state", out);

  maize_growth_stages.Save("maize_growth_stages", out);
}

void Maize::Deserialize(const YAML::Node& in) {
  if (in["plant_age"])
    plant_age = in["plant_age"].as<float>();
  if (in["seed"])
    seed = in["seed"].as<int>();
  maize_descriptor.Load("maize_descriptor", in);
  maize_growth_stages.Load("maize_growth_stages", in);
  maize_state.Load("maize_state", in);
  maize_generator.Load("maize_generator", in);
}

bool Maize::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<MaizeGenerator>(maize_generator, "MaizeGenerator"))
    changed = true;
  
  if (maize_generator.Get<MaizeGenerator>()) {
    if (ImGui::SliderFloat("Plant Age", &plant_age, 0.0f, 1.0f)) {
      Regenerate();
      changed = true;
    }
  }

  if (editor_layer->DragAndDropButton<MaizeGrowthStages>(maize_growth_stages, "MaizeGrowthStages"))
    changed = true;

  if (editor_layer->DragAndDropButton<MaizeState>(maize_state, "MaizeState"))
    changed = true;

  if (editor_layer->DragAndDropButton<MaizeDescriptor>(maize_descriptor, "MaizeDescriptor"))
    changed = true;

  if (ImGui::Button("Form meshes")) {
    GenerateGeometryEntities(MaizeMeshGeneratorSettings{});
  }

  if (const auto ssg = maize_generator.Get<MaizeGenerator>()) {
    if (ImGui::TreeNode("Maize Generator settings")) {
      if (ImGui::DragInt("Seed", &seed)) {
        Regenerate();
        changed = true;
      }
      ImGui::TreePop();
    }
  }
  if (const auto sgs = maize_growth_stages.Get<MaizeGrowthStages>()) {
    if (ImGui::TreeNode("Maize Growth Descriptor settings")) {
      static float time = 0.0f;
      if (ImGui::SliderFloat("Time", &time, 0.0f, sgs->GetCurrentEndTime())) {
        time = glm::clamp(time, 0.0f, sgs->GetCurrentEndTime());
        auto sd = maize_descriptor.Get<MaizeDescriptor>();
        if (!sd) {
          sd = AssetManager::CreateTemporaryAsset<MaizeDescriptor>();
          maize_descriptor = sd;
        }
        sgs->Apply(sd, time);
        GenerateGeometryEntities(MaizeMeshGeneratorSettings{});
      }
      ImGui::TreePop();
    }
  }
  static bool debug_rendering = false;
  ImGui::Checkbox("Debug", &debug_rendering);
  if (debug_rendering) {
    if (const auto sd = maize_descriptor.Get<MaizeDescriptor>()) {
        // Debug rendering logic for maize similar to sorghum
    }
  }
  return changed;
}

void Maize::Regenerate() {
  if (plant_age <= 0.001f) {
    ClearGeometryEntities();
    return;
  }
  if (const auto generator = maize_generator.Get<MaizeGenerator>()) {
    auto state = maize_state.Get<MaizeState>();
    if (!state) {
      state = AssetManager::CreateTemporaryAsset<MaizeState>();
      maize_state = state;
    }
    
    // Use the component's seed for consistent generation.
    // If seed is 0, it will use global random state (and flicker if Regenerate is called often).
    // Users should set non-zero seed for stability during drag.
    // Default seed is 1.
    generator->Apply(state, seed); 
    
    if (plant_age > 0.0f) {
      generator->ApplyGrowth(state, plant_age); // Scales it.
    }
    
    // Ensure descriptor is updated from the state
    auto descriptor = maize_descriptor.Get<MaizeDescriptor>();
    if (!descriptor) {
        descriptor = AssetManager::CreateTemporaryAsset<MaizeDescriptor>();
        maize_descriptor = descriptor;
    }
    state->Apply(descriptor);

    GenerateGeometryEntities(MaizeMeshGeneratorSettings{});
  }
}

void Maize::CollectAssetRef(std::vector<AssetRef>& list) {
  if (maize_descriptor.Get<MaizeDescriptor>())
    list.push_back(maize_descriptor);
  if (maize_growth_stages.Get<MaizeGrowthStages>())
    list.push_back(maize_growth_stages);
  if (maize_generator.Get<MaizeGenerator>())
    list.push_back(maize_generator);
  if (maize_state.Get<MaizeState>())
    list.push_back(maize_state);
}
uint32_t Maize::GetLeafSize() {
  if (const auto sd = maize_descriptor.Get<MaizeDescriptor>())
    return sd->leaves.size();
  if (const auto ss = maize_state.Get<MaizeState>())
    return ss->leaves.size();
  EVOENGINE_ERROR("GetLeafSize failed: MaizeDescriptor or MaizeState missing!")
  return 0;
}
