#include "MeshColoring.hpp"
#include "MeshTracing.hpp"
#include "Prefab.hpp"

using namespace evo_engine;
using namespace mesh_tracing;
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
  EditorLayer::DragAndDropButton<Prefab>(input_prefab_ref, "Input Prefab");
  EditorLayer::DragAndDropButton<Prefab>(output_prefab_ref_vertex, "Output Prefab (Color by vertex)");
  EditorLayer::DragAndDropButton<Prefab>(output_prefab_ref_triangle, "Output Prefab (Color by triangle)");
  auto find_mesh = [&](const std::shared_ptr<Prefab>& target_prefab) {
    target_prefab->GatherAssets();
    for (auto& asset_ref : target_prefab->collected_assets) {
      if (const auto asset = asset_ref.second.Get<Mesh>()) {
        return asset;
      }
    }
    return std::shared_ptr<Mesh>();
  };
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

  static AssetRef bvh_prefab_ref;
  static MeshTracer mesh_tracer;

  static std::shared_ptr<ParticleInfoList> particle_info_list;
  static std::vector<ParticleInfo> particle_infos;
  bool refresh_aabb = false;
  if (!particle_info_list)
    particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  EditorLayer::DragAndDropButton<Prefab>(bvh_prefab_ref, "BVH Prefab");
  if (const auto bvh_prefab = bvh_prefab_ref.Get<Prefab>()) {
    bvh_prefab->GatherAssets();
    if (const std::shared_ptr<Mesh> target_mesh = find_mesh(bvh_prefab)) {
      auto vertices = target_mesh->UnsafeGetVertices();
      const auto triangles = target_mesh->UnsafeGetTriangles();
      std::vector<glm::vec3> positions(vertices.size());
      Jobs::RunParallelFor(vertices.size(), [&](const size_t i) {
        positions.at(i) = vertices.at(i).position;
      });
      mesh_tracer.Initialize(positions, triangles, 8, 8192);
      refresh_aabb = true;
    }
    bvh_prefab_ref.Clear();
  }

  static int min_tree_depth = 0;
  static int max_tree_depth = 10;
  std::vector<MeshTracer::Aabb> aabbs{};
  
  if (ImGui::DragInt("Min Depth", &min_tree_depth, 1, 0, max_tree_depth)) {
    refresh_aabb = true;
    min_tree_depth = glm::clamp(min_tree_depth, 0, max_tree_depth);
  }
  if (ImGui::DragInt("Max Depth", &max_tree_depth, 1, min_tree_depth, 8192)) {
    refresh_aabb = true;
    max_tree_depth = glm::clamp(max_tree_depth, min_tree_depth, 8192);
  }
  if (refresh_aabb) {
    mesh_tracer.bvh.CollectAabb(min_tree_depth, max_tree_depth, aabbs);
    particle_infos.resize(aabbs.size());
    Jobs::RunParallelFor(aabbs.size(), [&](const size_t i) {
      auto& particle_info = particle_infos.at(i);
      const auto& aabb = aabbs.at(i);
      particle_info.instance_color = glm::vec4(glm::abs(glm::sphericalRand(1.f)), 0.5f);
      particle_info.instance_matrix.value = glm::translate(aabb.center) * glm::scale(aabb.size * 2.f);
    });
    particle_info_list->SetParticleInfos(particle_infos);
  }
  if (!particle_infos.empty()) {
    GizmoSettings gizmo_settings;
    gizmo_settings.draw_settings.blending = true;
    auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
    editor_layer->DrawGizmoCubes(particle_info_list, global_transform.value, 1, gizmo_settings);
  }
  return false;
}