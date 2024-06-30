#include "MeshColoring.hpp"

#include "CpuRayTracer.hpp"
#include "MeshTracer.hpp"
#include "Prefab.hpp"
#include "Times.hpp"

using namespace evo_engine;
bool MeshColoring::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  static AssetRef input_prefab_ref;
  static AssetRef output_prefab_ref_vertex;
  static AssetRef output_prefab_ref_triangle;
  static std::vector<glm::vec3> colors;
  if (colors.empty()) {
    colors.resize(32768);
    for (auto& color : colors) {
      color = glm::abs(glm::sphericalRand(1.f));
    }
  }
  const auto scene = GetScene();
  const auto owner = GetOwner();
  auto find_mesh = [&](const std::shared_ptr<Prefab>& target_prefab) {
    target_prefab->GatherAssets();
    for (auto& asset_ref : target_prefab->collected_assets) {
      if (const auto asset = asset_ref.second.Get<Mesh>()) {
        return asset;
      }
    }
    return std::shared_ptr<Mesh>();
  };

  // EditorLayer::DragAndDropButton<Prefab>(input_prefab_ref, "Input Prefab");
  // EditorLayer::DragAndDropButton<Prefab>(output_prefab_ref_vertex, "Output Prefab (Color by vertex)");
  // EditorLayer::DragAndDropButton<Prefab>(output_prefab_ref_triangle, "Output Prefab (Color by triangle)");

  if (const auto output_prefab = output_prefab_ref_vertex.Get<Prefab>()) {
    output_prefab->GatherAssets();
    if (const std::shared_ptr<Mesh> target_mesh = find_mesh(output_prefab)) {
      auto vertices = target_mesh->UnsafeGetVertices();
      const auto triangles = target_mesh->UnsafeGetTriangles();
      for (int vertex_index = 0; vertex_index < vertices.size(); vertex_index++) {
        vertices.at(vertex_index).color = glm::vec4(colors[vertex_index], 1.f);
      }
      VertexAttributes vertex_attributes{};
      vertex_attributes.color = true;
      const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
      new_mesh->SetVertices(vertex_attributes, vertices, triangles);
      const auto new_entity = scene->CreateEntity("Output Mesh (V-Colored)");
      const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
      mmr->mesh = new_mesh;
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      material->vertex_color_only = true;
      mmr->material = material;
    }
    output_prefab_ref_vertex.Clear();
  }

  if (const auto output_prefab = output_prefab_ref_triangle.Get<Prefab>()) {
    output_prefab->GatherAssets();
    if (const std::shared_ptr<Mesh> target_mesh = find_mesh(output_prefab)) {
      auto vertices = target_mesh->UnsafeGetVertices();
      const auto triangles = target_mesh->UnsafeGetTriangles();
      for (int triangle_index = 0; triangle_index < triangles.size(); triangle_index++) {
        const auto& triangle = triangles.at(triangle_index);
        vertices.at(triangle.x).color = glm::vec4(colors[triangle_index], 1.f);
        vertices.at(triangle.y).color = glm::vec4(colors[triangle_index], 1.f);
        vertices.at(triangle.z).color = glm::vec4(colors[triangle_index], 1.f);
      }
      VertexAttributes vertex_attributes{};
      vertex_attributes.color = true;
      const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
      new_mesh->SetVertices(vertex_attributes, vertices, triangles);

      const auto new_entity = scene->CreateEntity("Output Mesh (T-Colored)");
      const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
      mmr->mesh = new_mesh;
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      material->vertex_color_only = true;
      mmr->material = material;
    }

    output_prefab_ref_triangle.Clear();
  }

  if (const auto input_prefab = input_prefab_ref.Get<Prefab>()) {
    FileUtils::OpenFile(
        "Load vertex map", "Text", {".txt"},
        [&](const std::filesystem::path& path) {
          input_prefab->GatherAssets();
          if (const std::shared_ptr<Mesh> target_mesh = find_mesh(input_prefab)) {
            auto vertices = target_mesh->UnsafeGetVertices();
            auto triangles = target_mesh->UnsafeGetTriangles();
            auto string = FileUtils::LoadFileAsString(path);
            std::istringstream iss(string);
            std::string line;
            int input_vertex_index = 0;
            while (std::getline(iss, line)) {
              if (line.empty())
                continue;
              if (const auto vertex_index = std::stoi(line); vertex_index != -1 && vertex_index < vertices.size()) {
                vertices.at(input_vertex_index).color = glm::vec4(colors[vertex_index], 1.f);
              }
              input_vertex_index++;
            }
            VertexAttributes vertex_attributes{};
            vertex_attributes.color = true;
            const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
            new_mesh->SetVertices(vertex_attributes, vertices, triangles);
            const auto new_entity = scene->CreateEntity("Input Mesh (V-Colored)");
            const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
            mmr->mesh = new_mesh;
            const auto material = ProjectManager::CreateTemporaryAsset<Material>();
            material->vertex_color_only = true;
            mmr->material = material;
          }
        },
        false);
    FileUtils::OpenFile(
        "Load triangle map", "Text", {".txt"},
        [&](const std::filesystem::path& path) {
          input_prefab->GatherAssets();
          if (const std::shared_ptr<Mesh> target_mesh = find_mesh(input_prefab)) {
            auto vertices = target_mesh->UnsafeGetVertices();
            auto triangles = target_mesh->UnsafeGetTriangles();
            auto string = FileUtils::LoadFileAsString(path);
            std::istringstream iss(string);
            std::string line;
            int input_triangle_index = 0;
            while (std::getline(iss, line)) {
              if (line.empty())
                continue;
              const auto triangle_index = std::stoi(line);
              if (triangle_index != -1) {
                const auto& triangle = triangles.at(input_triangle_index);
                vertices.at(triangle.x).color = glm::vec4(colors[triangle_index], 1.f);
                vertices.at(triangle.y).color = glm::vec4(colors[triangle_index], 1.f);
                vertices.at(triangle.z).color = glm::vec4(colors[triangle_index], 1.f);
              }
              input_triangle_index++;
            }
            VertexAttributes vertex_attributes{};
            vertex_attributes.color = true;
            const auto new_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
            new_mesh->SetVertices(vertex_attributes, vertices, triangles);
            const auto new_entity = scene->CreateEntity("Input Mesh (T-Colored)");
            const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(new_entity).lock();
            mmr->mesh = new_mesh;
            const auto material = ProjectManager::CreateTemporaryAsset<Material>();
            material->vertex_color_only = true;
            mmr->material = material;
          }
        },
        false);
  }

  static AssetRef visibility_test_prefab_ref;
  static AssetRef visibility_test_mesh_ref;

  EditorLayer::DragAndDropButton<Prefab>(visibility_test_prefab_ref, "Visibility Prefab");
  EditorLayer::DragAndDropButton<Mesh>(visibility_test_mesh_ref, "Visibility Mesh");

  if (const auto boundary_test_prefab = visibility_test_prefab_ref.Get<Prefab>()) {
    boundary_test_prefab->GatherAssets();
    if (const std::shared_ptr<Mesh> target_mesh = find_mesh(boundary_test_prefab)) {
      VisibilityTesting::VisibilityTestParams params;
      std::vector<float> visibility_results;
      VisibilityTesting::VisibilityTest(target_mesh, params, visibility_results);
      VertexAttributes vertex_attributes{};
      vertex_attributes.color = true;

      auto vertices = target_mesh->UnsafeGetVertices();
      const auto& triangles = target_mesh->UnsafeGetTriangles();
      
      for (int i = 0; i < triangles.size(); i++) {
        glm::vec4 color = visibility_results.at(i) > 0 ? glm::vec4(1, 1, 1, 1) : glm::vec4(0, 0, 0, 1);
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
    VisibilityTesting::VisibilityTestParams params;
    std::vector<float> visibility_results;
    VisibilityTesting::VisibilityTest(boundary_test_mesh, params, visibility_results);
    VertexAttributes vertex_attributes{};
    vertex_attributes.color = true;

    auto vertices = boundary_test_mesh->UnsafeGetVertices();
    const auto& triangles = boundary_test_mesh->UnsafeGetTriangles();
    for (int i = 0; i < triangles.size(); i++) {
      glm::vec4 color = visibility_results.at(i) > 0 ? glm::vec4(1, 1, 1, 1) : glm::vec4(0, 0, 0, 1);
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
  return false;
}