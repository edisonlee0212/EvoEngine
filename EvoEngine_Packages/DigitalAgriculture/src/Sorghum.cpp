#include "DigitalAgricultureSerializationAdapters.hpp"

#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFGroup.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
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
      const auto btf_group = sorghum_layer->leaf_cbtf_group.Get<CBTFGroup>();
      if (sorghum_mesh_generator_settings.single_leaf_index != -1) {
        if (sorghum_mesh_generator_settings.single_leaf_index < target_sorghum_descriptor->leaves.size()) {
          const auto& leaf_state = target_sorghum_descriptor->leaves[sorghum_mesh_generator_settings.single_leaf_index];
          const auto leaf_entity = scene->CreateEntity("Leaf Mesh");
          const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
          if (sorghum_layer->enable_compressed_btf) {
            if (btf_group) {
              const auto btf_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(leaf_entity).lock();
              btf_renderer->mesh = mesh;
              btf_renderer->btf = btf_group->GetRandom();
            }
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
            if (btf_group) {
              const auto btf_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(leaf_entity).lock();
              btf_renderer->mesh = mesh;
              btf_renderer->btf = btf_group->GetRandom();
            }
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
        if (const auto btf_group = sorghum_layer->leaf_cbtf_group.Get<CBTFGroup>()) {
          const auto btf_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(leaf_entity).lock();
          btf_renderer->mesh = mesh;
          btf_renderer->btf = btf_group->GetRandom();
        }
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
