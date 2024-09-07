#include "MeshColoring.hpp"

#include "ClassRegistry.hpp"
#include "RayTracer.hpp"
#include "Prefab.hpp"
#include "Times.hpp"
#include "VisibilityTest.hpp"
using namespace evo_engine;


bool MeshColoring::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto scene = GetScene();
  auto find_mesh = [&](const std::shared_ptr<Prefab>& target_prefab) {
    target_prefab->GatherAssets();
    for (auto& asset_ref : target_prefab->collected_assets) {
      if (const auto asset = asset_ref.second.Get<Mesh>()) {
        return asset;
      }
    }
    return std::shared_ptr<Mesh>();
  };
  const auto owner = GetOwner();
  
  static AssetRef visibility_test_prefab_ref;
  static AssetRef visibility_test_mesh_ref;

  EditorLayer::DragAndDropButton<Prefab>(visibility_test_prefab_ref, "Visibility Prefab");
  EditorLayer::DragAndDropButton<Mesh>(visibility_test_mesh_ref, "Visibility Mesh");

  if (const auto boundary_test_prefab = visibility_test_prefab_ref.Get<Prefab>()) {
    boundary_test_prefab->GatherAssets();
    if (const std::shared_ptr<Mesh> target_mesh = find_mesh(boundary_test_prefab)) {
      VisibilityTest::VisibilityTestParams params;
      std::vector<VisibilityTest::Visibility> visibility_results;
      VisibilityTest::Execute(target_mesh, params, visibility_results);
      VertexAttributes vertex_attributes{};
      vertex_attributes.color = true;

      auto vertices = target_mesh->UnsafeGetVertices();
      const auto& triangles = target_mesh->UnsafeGetTriangles();

      for (int i = 0; i < triangles.size(); i++) {
        glm::vec4 color = visibility_results.at(i).visible ? glm::vec4(1, 1, 1, 1) : glm::vec4(0, 0, 0, 1);
        vertices.at(triangles.at(i).x).color = color;
        vertices.at(triangles.at(i).y).color = color;
        vertices.at(triangles.at(i).z).color = color;
      }

      const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
      new_mesh->SetVertices(vertex_attributes, vertices, triangles);
      const auto new_entity = scene->CreateEntity("Visibility tested mesh");
      const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
      mmr->mesh = new_mesh;
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      material->vertex_color_only = true;
      mmr->material = material;
    }
    visibility_test_prefab_ref.Clear();
  }
  if (const auto boundary_test_mesh = visibility_test_mesh_ref.Get<Mesh>()) {
    VisibilityTest::VisibilityTestParams params;
    std::vector<VisibilityTest::Visibility> visibility_results;
    VisibilityTest::Execute(boundary_test_mesh, params, visibility_results);
    VertexAttributes vertex_attributes{};
    vertex_attributes.color = true;

    auto vertices = boundary_test_mesh->UnsafeGetVertices();
    const auto& triangles = boundary_test_mesh->UnsafeGetTriangles();
    for (int i = 0; i < triangles.size(); i++) {
      glm::vec4 color = visibility_results.at(i).visible ? glm::vec4(1, 1, 1, 1) : glm::vec4(0, 0, 0, 1);
      vertices.at(triangles.at(i).x).color = color;
      vertices.at(triangles.at(i).y).color = color;
      vertices.at(triangles.at(i).z).color = color;
    }

    const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    new_mesh->SetVertices(vertex_attributes, vertices, triangles);
    const auto new_entity = scene->CreateEntity("Visibility tested mesh");
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
    mmr->mesh = new_mesh;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    material->vertex_color_only = true;
    mmr->material = material;
    visibility_test_mesh_ref.Clear();
  }

  static std::vector<float> triangle_errors{};

  FileUtils::OpenFile(
      "Load triangle map", "Text", {".txt"},
      [&](const std::filesystem::path& path) {
        auto string = FileUtils::LoadFileAsString(path);
        std::istringstream iss(string);
        std::string line;
        triangle_errors.clear();
        while (std::getline(iss, line)) {
          if (line.empty())
            continue;
          triangle_errors.emplace_back(std::stof(line));
        }

        EVOENGINE_LOG("Loaded " + std::to_string(triangle_errors.size()) + " errors.");
      },
      false);

  static AssetRef triangle_error_prefab_ref;
  EditorLayer::DragAndDropButton<Prefab>(triangle_error_prefab_ref, "Triangle error Prefab");
  static float error_threshold = 0.0f;
  if (ImGui::SliderFloat("Error threshold", &error_threshold, 0.0f, 1.f)) {
    if (const auto error_prefab = triangle_error_prefab_ref.Get<Prefab>()) {
      error_prefab->GatherAssets();
      if (const std::shared_ptr<Mesh> target_mesh = find_mesh(error_prefab)) {
        const auto& vertices = target_mesh->UnsafeGetVertices();
        const auto& triangles = target_mesh->UnsafeGetTriangles();

        std::vector<Vertex> new_vertices{};
        std::vector<glm::uvec3> new_triangles{};
        if (triangle_errors.size() == triangles.size()) {
          for (int i = 0; i < triangles.size(); i++) {
            glm::vec4 color = triangle_errors[i] <= error_threshold ? glm::vec4(1, 1, 1, 1) : glm::vec4(0, 0, 0, 1);
            new_vertices.emplace_back(vertices.at(triangles.at(i).x)).color = color;
            new_vertices.emplace_back(vertices.at(triangles.at(i).y)).color = color;
            new_vertices.emplace_back(vertices.at(triangles.at(i).z)).color = color;
            new_triangles.emplace_back(i * 3, i * 3 + 1, i * 3 + 2);
          }
          VertexAttributes vertex_attributes{};
          vertex_attributes.color = true;
          const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
          new_mesh->SetVertices(vertex_attributes, new_vertices, new_triangles);
          const auto children = scene->GetChildren(owner);
          for (const auto& child : children) {
            if (scene->GetEntityName(child) == "Triangle error") {
              scene->DeleteEntity(child);
            }
          }
          const auto new_entity = scene->CreateEntity("Triangle error");
          const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
          mmr->mesh = new_mesh;
          const auto material = ProjectManager::CreateTemporaryAsset<Material>();
          material->vertex_color_only = true;
          mmr->material = material;
          
          scene->SetDataComponent(new_entity, error_prefab->CalculateAdjustedTransform(true, true));
          scene->SetParent(new_entity, owner);
        }
      }
    }
  }

  static PrivateComponentRef mesh_entity_ref;
  EditorLayer::DragAndDropButton<MeshRenderer>(mesh_entity_ref, "Mesh entity");
  if (mesh_entity_ref.Get<MeshRenderer>()) {
    const auto mesh_entity = mesh_entity_ref.Get<MeshRenderer>()->GetOwner();
    VisibilityTest::VisibilityTestParams params{};
    params.sample_depth = 1;
    params.sample_budget = 5000;
    params.sample_minimum = 1;
    std::vector<VisibilityTest::Visibility> visibility_results;
    VisibilityTest::Execute(scene, mesh_entity, params, visibility_results);
    int visible_count = 0;
    for (const auto& i : visibility_results) {
      if (i.visible)
        visible_count++;
    }
    EVOENGINE_LOG("Visible triangles size: " + std::to_string(visible_count))
    VertexAttributes vertex_attributes{};
    vertex_attributes.color = true;
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(mesh_entity).lock();
    const auto mesh = mesh_renderer->mesh.Get<Mesh>();
    auto vertices = mesh->UnsafeGetVertices();
    const auto& triangles = mesh->UnsafeGetTriangles();
    for (int i = 0; i < triangles.size(); i++) {
      glm::vec4 color = visibility_results.at(i).visible ? glm::vec4(1, 1, 1, 1) : glm::vec4(0, 0, 0, 1);
      vertices.at(triangles.at(i).x).color = color;
      vertices.at(triangles.at(i).y).color = color;
      vertices.at(triangles.at(i).z).color = color;
    }

    const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    new_mesh->SetVertices(vertex_attributes, vertices, triangles);
    const auto new_entity = scene->CreateEntity("Visibility tested mesh");
    scene->SetDataComponent(new_entity, scene->GetDataComponent<GlobalTransform>(mesh_entity));
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
    mmr->mesh = new_mesh;
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    material->vertex_color_only = true;
    mmr->material = material;
    mesh_entity_ref.Clear();
  }

  return false;
}