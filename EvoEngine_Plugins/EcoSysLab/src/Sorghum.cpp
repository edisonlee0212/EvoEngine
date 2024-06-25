#include "Sorghum.hpp"

#include "SorghumLayer.hpp"
#include "SorghumStateGenerator.hpp"

using namespace eco_sys_lab;

void Sorghum::ClearGeometryEntities() {
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
  const auto sorghumLayer = Application::GetLayer<SorghumLayer>();
  if (!sorghumLayer)
    return;
  const auto sorghumState = m_sorghumState.Get<SorghumState>();
  if (!sorghumState) {
    if (const auto sorghumDescriptor = m_sorghumDescriptor.Get<SorghumStateGenerator>()) {
      sorghumDescriptor->Apply(sorghumState);
    } else if (const auto sorghumGrowthDescriptor = m_sorghumGrowthDescriptor.Get<SorghumGrowthStages>()) {
      sorghumGrowthDescriptor->Apply(sorghumState, 1.f);
    }
  }

  if (!sorghumState)
    return;
  if (sorghumState->m_stem.m_spline.m_segments.empty())
    return;
  ClearGeometryEntities();
  const auto scene = GetScene();
  const auto owner = GetOwner();
  if (sorghum_mesh_generator_settings.m_enablePanicle && sorghumState->m_panicle.m_seedAmount > 0) {
    const auto panicleEntity = scene->CreateEntity("Panicle Mesh");
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(panicleEntity).lock();
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    const auto particleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    particles->mesh = mesh;
    particles->material = material;
    const auto panicleMaterial = sorghumLayer->panicle_material.Get<Material>();
    // material->SetAlbedoTexture(panicleMaterial->GetAlbedoTexture());
    // material->SetNormalTexture(panicleMaterial->GetNormalTexture());
    // material->SetRoughnessTexture(panicleMaterial->GetRoughnessTexture());
    // material->SetMetallicTexture(panicleMaterial->GetMetallicTexture());
    material->material_properties = panicleMaterial->material_properties;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    sorghumState->m_panicle.GenerateGeometry(sorghumState->m_stem.m_spline.m_segments.back().m_position, vertices,
                                             indices, particleInfoList);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);

    particles->particle_info_list = particleInfoList;
    scene->SetParent(panicleEntity, owner);
  }
  if (sorghum_mesh_generator_settings.m_enableStem) {
    const auto stemEntity = scene->CreateEntity("Stem Mesh");
    const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(stemEntity).lock();
    const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    meshRenderer->mesh = mesh;
    meshRenderer->material = material;
    const auto stemMaterial = sorghumLayer->leaf_material.Get<Material>();
    // material->SetAlbedoTexture(stemMaterial->GetAlbedoTexture());
    // material->SetNormalTexture(stemMaterial->GetNormalTexture());
    // material->SetRoughnessTexture(stemMaterial->GetRoughnessTexture());
    // material->SetMetallicTexture(stemMaterial->GetMetallicTexture());
    material->material_properties = stemMaterial->material_properties;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    sorghumState->m_stem.GenerateGeometry(vertices, indices);
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    mesh->SetVertices(attributes, vertices, indices);
    scene->SetParent(stemEntity, owner);
  }
  if (sorghum_mesh_generator_settings.m_enableLeaves) {
    if (sorghum_mesh_generator_settings.m_leafSeparated) {
      for (const auto& leafState : sorghumState->m_leaves) {
        const auto leafEntity = scene->CreateEntity("Leaf Mesh");
        const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leafEntity).lock();
        const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
        const auto material = ProjectManager::CreateTemporaryAsset<Material>();
        meshRenderer->mesh = mesh;
        meshRenderer->material = material;
        const auto leafMaterial = sorghumLayer->leaf_material.Get<Material>();
        // material->SetAlbedoTexture(leafMaterial->GetAlbedoTexture());
        // material->SetNormalTexture(leafMaterial->GetNormalTexture());
        // material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
        // material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
        material->material_properties = leafMaterial->material_properties;
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        leafState.GenerateGeometry(vertices, indices, false, 0.f);
        if (sorghum_mesh_generator_settings.m_bottomFace) {
          leafState.GenerateGeometry(vertices, indices, true, sorghum_mesh_generator_settings.m_leafThickness);
        }
        VertexAttributes attributes{};
        attributes.tex_coord = true;
        mesh->SetVertices(attributes, vertices, indices);
        scene->SetParent(leafEntity, owner);
      }
    } else {
      const auto leafEntity = scene->CreateEntity("Leaf Mesh");
      const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leafEntity).lock();
      const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      meshRenderer->mesh = mesh;
      meshRenderer->material = material;
      const auto leafMaterial = sorghumLayer->leaf_material.Get<Material>();
      material->SetAlbedoTexture(leafMaterial->GetAlbedoTexture());
      // material->SetNormalTexture(leafMaterial->GetNormalTexture());
      // material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
      // material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
      material->material_properties = leafMaterial->material_properties;
      std::vector<Vertex> vertices;
      std::vector<unsigned int> indices;
      for (const auto& leafState : sorghumState->m_leaves) {
        leafState.GenerateGeometry(vertices, indices, false, 0.f);
        if (sorghum_mesh_generator_settings.m_bottomFace) {
          leafState.GenerateGeometry(vertices, indices, true, sorghum_mesh_generator_settings.m_leafThickness);
        }
      }
      VertexAttributes attributes{};
      attributes.tex_coord = true;
      mesh->SetVertices(attributes, vertices, indices);
      scene->SetParent(leafEntity, owner);
    }
  }
}

void Sorghum::Serialize(YAML::Emitter& out) const {
  m_sorghumState.Save("m_sorghumState", out);
  m_sorghumDescriptor.Save("m_sorghumDescriptor", out);
  m_sorghumGrowthDescriptor.Save("m_sorghumGrowthDescriptor", out);
}

void Sorghum::Deserialize(const YAML::Node& in) {
  m_sorghumState.Load("m_sorghumState", in);
  m_sorghumGrowthDescriptor.Load("m_sorghumGrowthDescriptor", in);
  m_sorghumDescriptor.Load("m_sorghumDescriptor", in);
}

bool Sorghum::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  if (editorLayer->DragAndDropButton<SorghumStateGenerator>(m_sorghumDescriptor, "SorghumStateGenerator"))
    changed = true;
  if (editorLayer->DragAndDropButton<SorghumGrowthStages>(m_sorghumGrowthDescriptor, "SorghumGrowthStages"))
    changed = true;
  if (editorLayer->DragAndDropButton<SorghumState>(m_sorghumState, "SorghumState"))
    changed = true;

  if (ImGui::Button("Form meshes")) {
    GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  }

  if (const auto sorghumDescriptor = m_sorghumDescriptor.Get<SorghumStateGenerator>()) {
    if (ImGui::TreeNode("Sorghum Descriptor settings")) {
      static int seed = 0;
      if (ImGui::DragInt("Seed", &seed)) {
        auto sorghumState = m_sorghumState.Get<SorghumState>();
        if (!sorghumState) {
          sorghumState = ProjectManager::CreateTemporaryAsset<SorghumState>();
          m_sorghumState = sorghumState;
        }
        sorghumDescriptor->Apply(sorghumState, seed);
        GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
      }
      ImGui::TreePop();
    }
  }
  if (const auto sorghumGrowthDescriptor = m_sorghumGrowthDescriptor.Get<SorghumGrowthStages>()) {
    if (ImGui::TreeNode("Sorghum Growth Descriptor settings")) {
      static float time = 0.0f;
      if (ImGui::SliderFloat("Time", &time, 0.0f, sorghumGrowthDescriptor->GetCurrentEndTime())) {
        time = glm::clamp(time, 0.0f, sorghumGrowthDescriptor->GetCurrentEndTime());
        auto sorghumState = m_sorghumState.Get<SorghumState>();
        if (!sorghumState) {
          sorghumState = ProjectManager::CreateTemporaryAsset<SorghumState>();
          m_sorghumState = sorghumState;
        }
        sorghumGrowthDescriptor->Apply(sorghumState, time);
        GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
      }
      ImGui::TreePop();
    }
  }
  static bool debugRendering = false;
  ImGui::Checkbox("Debug", &debugRendering);
  if (debugRendering) {
    static float nodeRenderSize = .5f;
    if (ImGui::TreeNode("Debug settings")) {
      ImGui::DragFloat("Node size", &nodeRenderSize, 0.01f, 0.0f, 1.f);
      ImGui::TreePop();
    }
    static std::shared_ptr<ParticleInfoList> nodeDebugInfoList;
    if (!nodeDebugInfoList)
      nodeDebugInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    std::vector<ParticleInfo> particleInfos;

    if (const auto sorghumState = m_sorghumState.Get<SorghumState>()) {
      const auto owner = GetOwner();
      const auto scene = GetScene();
      const auto plantPosition = scene->GetDataComponent<GlobalTransform>(owner).GetPosition();
      for (const auto& leafState : sorghumState->m_leaves) {
        const auto startIndex = particleInfos.size();
        particleInfos.resize(startIndex + leafState.m_spline.m_segments.size());
        for (int i = 0; i < leafState.m_spline.m_segments.size(); i++) {
          auto& matrix = particleInfos[startIndex + i].instance_matrix;
          matrix.value = glm::translate(leafState.m_spline.m_segments.at(i).m_position + plantPosition) *
                           glm::scale(glm::vec3(nodeRenderSize * leafState.m_spline.m_segments.at(i).m_radius));
          particleInfos[startIndex + i].instance_color =
              glm::vec4((leafState.m_index % 3) * 0.5f, ((leafState.m_index / 3) % 3) * 0.5f,
                        ((leafState.m_index / 9) % 3) * 0.5f, 1.0f);
        }
      }
      nodeDebugInfoList->SetParticleInfos(particleInfos);
    }
    editorLayer->DrawGizmoCubes(nodeDebugInfoList);
  }

  return changed;
}

void Sorghum::CollectAssetRef(std::vector<AssetRef>& list) {
  if (m_sorghumState.Get<SorghumState>())
    list.push_back(m_sorghumState);
  if (m_sorghumGrowthDescriptor.Get<SorghumGrowthStages>())
    list.push_back(m_sorghumGrowthDescriptor);
  if (m_sorghumDescriptor.Get<SorghumStateGenerator>())
    list.push_back(m_sorghumDescriptor);
}
