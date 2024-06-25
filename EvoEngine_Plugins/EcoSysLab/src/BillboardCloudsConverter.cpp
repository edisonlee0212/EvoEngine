#include "BillboardCloudsConverter.hpp"

#include <PointCloud.hpp>

#include "Prefab.hpp"
using namespace eco_sys_lab;

bool BillboardCloudsConverter::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto scene = GetScene();
  static BillboardCloud::GenerateSettings billboard_cloud_generate_settings{};
  billboard_cloud_generate_settings.OnInspect("Billboard clouds generation settings");

  if (ImGui::TreeNodeEx("Mesh -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static AssetRef mesh_ref;
    if (EditorLayer::DragAndDropButton<Mesh>(mesh_ref, "Drop mesh here...")) {
      if (const auto mesh = mesh_ref.Get<Mesh>()) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(mesh, ProjectManager::CreateTemporaryAsset<Material>());
        billboard_cloud.Generate(billboard_cloud_generate_settings);
        if (const auto entity = billboard_cloud.BuildEntity(scene); scene->IsEntityValid(entity))
          scene->SetEntityName(entity, "Billboard cloud (" + mesh->GetTitle() + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!")
        }
      }
      mesh_ref.Clear();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Prefab -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static AssetRef prefab_ref;
    if (EditorLayer::DragAndDropButton<Prefab>(prefab_ref, "Drop prefab here...")) {
      if (const auto prefab = prefab_ref.Get<Prefab>()) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(prefab);
        billboard_cloud.Generate(billboard_cloud_generate_settings);
        const auto entity = billboard_cloud.BuildEntity(scene);
        if (scene->IsEntityValid(entity))
          scene->SetEntityName(entity, "Billboard cloud (" + prefab->GetTitle() + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!")
        }
      }
      prefab_ref.Clear();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Entity -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static EntityRef entity_ref;

    if (EditorLayer::DragAndDropButton(entity_ref, "Drop Entity here...")) {
      if (const auto entity = entity_ref.Get(); scene->IsEntityValid(entity)) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(scene, entity);
        billboard_cloud.Generate(billboard_cloud_generate_settings);
        if (const auto billboard_entity = billboard_cloud.BuildEntity(scene); scene->IsEntityValid(billboard_entity))
          scene->SetEntityName(billboard_entity, "Billboard cloud (" + scene->GetEntityName(entity) + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!")
        }
      }
      entity_ref.Clear();
    }
    ImGui::TreePop();
  }
  static std::vector<glm::vec3> points;
  if (ImGui::TreeNodeEx("Entity -> Point Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static EntityRef entity_ref;

    if (EditorLayer::DragAndDropButton(entity_ref, "Drop Entity here...")) {
      if (const auto entity = entity_ref.Get(); scene->IsEntityValid(entity)) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(scene, entity);
        points = billboard_cloud.ExtractPointCloud(0.005f);
      }
      entity_ref.Clear();
    }
    ImGui::TreePop();
  }
  if (!points.empty()) {
    FileUtils::SaveFile(
        "Save point cloud...", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          const auto point_cloud = ProjectManager::CreateTemporaryAsset<PointCloud>();
          point_cloud->positions.resize(points.size());
          Jobs::RunParallelFor(points.size(), [&](const unsigned point_index) {
            point_cloud->positions[point_index] = glm::dvec3(points[point_index]);
          });
          point_cloud->has_positions = true;
          PointCloud::PointCloudSaveSettings save_settings{};
          save_settings.binary = false;
          save_settings.double_precision = false;
          if (point_cloud->Save(save_settings, path)) {
            EVOENGINE_LOG("PointCloud Saved!")
          }
          points.clear();
        },
        false);
  }

  if (ImGui::TreeNodeEx("Entity -> Color by distance", ImGuiTreeNodeFlags_DefaultOpen)) {
    static EntityRef entity_ref;

    if (EditorLayer::DragAndDropButton(entity_ref, "Drop Entity here...")) {
      if (const auto entity = entity_ref.Get(); scene->IsEntityValid(entity)) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(scene, entity);
        for (auto& element : billboard_cloud.elements) {
          const auto level_set = element.CalculateLevelSets();
          Entity clone = scene->CreateEntity("Cloned");
          const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(clone).lock();
          const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
          VertexAttributes attributes{};
          attributes.color = true;
          attributes.normal = true;
          attributes.tangent = true;
          mesh->SetVertices(attributes, element.vertices, element.triangles);
          mesh_renderer->mesh = mesh;

          const auto material = ProjectManager::CreateTemporaryAsset<Material>();
          material->vertex_color_only = true;
          mesh_renderer->material = material;
        }
      }
      entity_ref.Clear();
    }
    ImGui::TreePop();
  }

  return changed;
}
