#include "MeshColoring.hpp"
#include "MeshTracer.hpp"
#include "Prefab.hpp"
#include "Times.hpp"

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
  static AssetRef bvh_mesh_ref;
  static MeshTracer bvh_mesh_tracer;

  static std::shared_ptr<ParticleInfoList> bvh_particle_info_list;
  static std::vector<ParticleInfo> bvh_particle_infos;
  bool bvh_refresh_aabb = false;
  static bool has_bvh = false;
  if (!bvh_particle_info_list)
    bvh_particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  EditorLayer::DragAndDropButton<Prefab>(bvh_prefab_ref, "BVH Prefab");
  EditorLayer::DragAndDropButton<Prefab>(bvh_mesh_ref, "BVH Mesh");
  if (const auto bvh_prefab = bvh_prefab_ref.Get<Prefab>()) {
    bvh_prefab->GatherAssets();
    if (const std::shared_ptr<Mesh> target_mesh = find_mesh(bvh_prefab)) {
      auto vertices = target_mesh->UnsafeGetVertices();
      const auto triangles = target_mesh->UnsafeGetTriangles();
      std::vector<glm::vec3> positions(vertices.size());
      Jobs::RunParallelFor(vertices.size(), [&](const size_t i) {
        positions.at(i) = vertices.at(i).position;
      });
      bvh_mesh_tracer.Initialize(positions, triangles, 8, 8192);
      bvh_refresh_aabb = true;
      has_bvh = true;
    }
    bvh_prefab_ref.Clear();
  }
  if (const auto bvh_mesh = bvh_mesh_ref.Get<Mesh>()) {
    auto vertices = bvh_mesh->UnsafeGetVertices();
    const auto triangles = bvh_mesh->UnsafeGetTriangles();
    std::vector<glm::vec3> positions(vertices.size());
    Jobs::RunParallelFor(vertices.size(), [&](const size_t i) {
      positions.at(i) = vertices.at(i).position;
    });
    bvh_mesh_tracer.Initialize(positions, triangles, 8, 8192);
    bvh_refresh_aabb = true;
    has_bvh = true;

    bvh_mesh_ref.Clear();
  }

  if (has_bvh) {
    std::vector<MeshTracer::Aabb> aabbs{};
    static bool display_level = true;
    if (display_level) {
      static int level = 0;
      if (ImGui::DragInt("Level", &level, 1, 0, 8192)) {
        bvh_refresh_aabb = true;
        level = glm::clamp(level, 0, 8192);
        bvh_mesh_tracer.bvh.CollectAabb(level, level + 1, aabbs);
      }
    } else {
      static int min_tree_depth = 0;
      static int max_tree_depth = 10;
      if (ImGui::DragInt("Min Depth", &min_tree_depth, 1, 0, 8192)) {
        bvh_refresh_aabb = true;
        min_tree_depth = glm::clamp(min_tree_depth, 0, max_tree_depth);
        bvh_mesh_tracer.bvh.CollectAabb(min_tree_depth, max_tree_depth, aabbs);
      }
      if (ImGui::DragInt("Max Depth", &max_tree_depth, 1, min_tree_depth, 8192)) {
        bvh_refresh_aabb = true;
        max_tree_depth = glm::clamp(max_tree_depth, min_tree_depth, 8192);
        bvh_mesh_tracer.bvh.CollectAabb(min_tree_depth, max_tree_depth, aabbs);
      }
    }
    if (bvh_refresh_aabb) {
      bvh_particle_infos.resize(aabbs.size());
      Jobs::RunParallelFor(aabbs.size(), [&](const size_t i) {
        auto& particle_info = bvh_particle_infos.at(i);
        const auto& aabb = aabbs.at(i);
        particle_info.instance_color = glm::vec4(glm::abs(glm::sphericalRand(1.f)), 0.5f);
        particle_info.instance_matrix.value = glm::translate(aabb.center) * glm::scale(aabb.size * 2.f);
      });
      bvh_particle_info_list->SetParticleInfos(bvh_particle_infos);
    }
  }

  static AssetRef boundary_test_prefab_ref;
  static AssetRef boundary_test_mesh_ref;
  static MeshTracer mesh_tracer;

  static std::shared_ptr<ParticleInfoList> boundary_test_particle_info_list;
  static std::vector<ParticleInfo> boundary_test_particle_infos;
  static bool boundary_test_updated = false;
  if (!boundary_test_particle_info_list)
    boundary_test_particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  EditorLayer::DragAndDropButton<Prefab>(boundary_test_prefab_ref, "Boundary Prefab");
  EditorLayer::DragAndDropButton<Mesh>(boundary_test_mesh_ref, "Boundary Mesh");

  size_t testing_ray_size = 65536 * 16;
  if (const auto boundary_test_prefab = boundary_test_prefab_ref.Get<Prefab>()) {
    boundary_test_prefab->GatherAssets();
    if (const std::shared_ptr<Mesh> target_mesh = find_mesh(boundary_test_prefab)) {
      auto vertices = target_mesh->UnsafeGetVertices();
      const auto triangles = target_mesh->UnsafeGetTriangles();
      std::vector<glm::vec3> positions(vertices.size());
      Jobs::RunParallelFor(vertices.size(), [&](const size_t i) {
        positions.at(i) = vertices.at(i).position;
      });
      mesh_tracer.Initialize(positions, triangles, 8, 8192);
      boundary_test_updated = true;
    }
    boundary_test_prefab_ref.Clear();
  }
  if (const auto boundary_test_mesh = boundary_test_mesh_ref.Get<Mesh>()) {
    auto vertices = boundary_test_mesh->UnsafeGetVertices();
    const auto triangles = boundary_test_mesh->UnsafeGetTriangles();
    std::vector<glm::vec3> positions(vertices.size());
    Jobs::RunParallelFor(vertices.size(), [&](const size_t i) {
      positions.at(i) = vertices.at(i).position;
    });
    mesh_tracer.Initialize(positions, triangles, 8, 8192);
    boundary_test_updated = true;
    boundary_test_mesh_ref.Clear();
  }
  if (boundary_test_updated) {
    boundary_test_particle_infos.resize(testing_ray_size);
    std::vector<glm::vec3> ray_directions(testing_ray_size);
    for(auto& direction : ray_directions) {
      direction = glm::sphericalRand(1.f);
    }
    ray_directions.back() = glm::vec3(0, 0, 1);
    const auto start_time = Times::Now();
    Jobs::RunParallelFor(testing_ray_size, [&](const size_t testing_ray_index) {
      MeshTracer::TraceParameters trace_parameters{};
      trace_parameters.origin = glm::vec3(0.f);
      trace_parameters.direction = ray_directions.at(testing_ray_index);
      trace_parameters.t_min = 0.f;
      trace_parameters.t_max = FLT_MAX;
      mesh_tracer.Trace(
          trace_parameters,
          [&](const MeshTracer::HitInfo& hit_info) {
            auto& particle_info = boundary_test_particle_infos.at(testing_ray_index);
            particle_info.instance_color = glm::vec4(1.f, 1.f, 1.f, 0.5f);
            particle_info.instance_matrix.value = glm::translate(hit_info.hit) * glm::scale(glm::vec3(1.f));
          },
          [&]() {
            EVOENGINE_ERROR("Missed!")
          },
          [&](const MeshTracer::HitInfo&) {
          });
    });
    const auto elapsed_time = Times::Now() - start_time;
    EVOENGINE_LOG("Tracing finished in " + std::to_string(elapsed_time) + " seconds with " + std::to_string(Jobs::GetWorkerSize()) + " thread(s).")
    boundary_test_particle_info_list->SetParticleInfos(boundary_test_particle_infos);
    boundary_test_updated = false;
  }

  if (!bvh_particle_infos.empty()) {
    GizmoSettings gizmo_settings;
    gizmo_settings.depth_test = true;
    gizmo_settings.draw_settings.blending = true;
    auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
    editor_layer->DrawGizmoCubes(bvh_particle_info_list, global_transform.value, 1, gizmo_settings);
  }
  static float point_size = 0.01f;
  ImGui::DragFloat("Point size", &point_size, 0.001f, 0.001f, 1.f);
  if (!boundary_test_particle_infos.empty()) {
    GizmoSettings gizmo_settings;
    gizmo_settings.depth_test = true;
    gizmo_settings.depth_write = true;
    gizmo_settings.draw_settings.blending = true;
    auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
    editor_layer->DrawGizmoCubes(boundary_test_particle_info_list, global_transform.value, point_size, gizmo_settings);
  }
  return false;
}