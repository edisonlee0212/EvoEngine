#include "BillboardCloudSettingsEditor.hpp"
#include "BillboardCloudsConverter.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"

#include <PointCloud.hpp>

#include "BillboardCloudsInspectionAdapters.hpp"
#include "Prefab.hpp"
using namespace billboard_clouds_package;

bool billboard_clouds_package::InspectBillboardCloudsConverter(InspectorContext& context,
                                                               BillboardCloudsConverter& billboard_clouds_converter,
                                                               BillboardCloudsInspectorState& state) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  const auto scene = billboard_clouds_converter.GetScene();
  InspectBillboardSettings(state.settings, "Billboard clouds generation settings");

  if (ImGui::TreeNodeEx("Mesh -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (editor_layer->DragAndDropButton<Mesh>(state.mesh_ref, "Drop mesh here...")) {
      if (const auto mesh = state.mesh_ref.Get<Mesh>()) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(mesh, AssetManager::CreateTemporaryAsset<Material>());
        billboard_cloud.Generate(state.settings);
        if (const auto entity = billboard_cloud.BuildEntity(scene); scene->IsEntityValid(entity))
          scene->SetEntityName(entity, "Billboard cloud (" + mesh->GetTitle() + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!")
        }
      }
      state.mesh_ref.Clear();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Prefab -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (editor_layer->DragAndDropButton<Prefab>(state.prefab_ref, "Drop prefab here...")) {
      if (const auto prefab = state.prefab_ref.Get<Prefab>()) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(prefab);
        billboard_cloud.Generate(state.settings);
        const auto entity = billboard_cloud.BuildEntity(scene);
        if (scene->IsEntityValid(entity))
          scene->SetEntityName(entity, "Billboard cloud (" + prefab->GetTitle() + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!")
        }
      }
      state.prefab_ref.Clear();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Entity -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (editor_layer->DragAndDropButton(state.billboard_entity, "Drop Entity here...")) {
      if (const auto entity = state.billboard_entity.Get(); scene->IsEntityValid(entity)) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(scene, entity);
        billboard_cloud.Generate(state.settings);
        if (const auto billboard_entity = billboard_cloud.BuildEntity(scene); scene->IsEntityValid(billboard_entity))
          scene->SetEntityName(billboard_entity, "Billboard cloud (" + scene->GetEntityName(entity) + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!")
        }
      }
      state.billboard_entity.Clear();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Entity -> Point Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (editor_layer->DragAndDropButton(state.point_entity, "Drop Entity here...")) {
      if (const auto entity = state.point_entity.Get(); scene->IsEntityValid(entity)) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(scene, entity);
        state.points = billboard_cloud.ExtractPointCloud(0.005f);
      }
      state.point_entity.Clear();
    }
    ImGui::TreePop();
  }
  if (!state.points.empty()) {
    EditorFileDialogs::SaveFile(
        "Save point cloud...", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          const auto point_cloud = AssetManager::CreateTemporaryAsset<PointCloud>();
          point_cloud->positions.resize(state.points.size());
          Jobs::RunParallelFor(state.points.size(), [&](size_t point_index) {
            point_cloud->positions[point_index] = glm::dvec3(state.points[point_index]);
          });
          PointCloud::PointCloudSaveSettings save_settings{};
          save_settings.binary = true;
          save_settings.double_precision = false;
          if (point_cloud->SavePly(save_settings, path)) {
            EVOENGINE_LOG("PointCloud Saved!")
          }
          state.points.clear();
        },
        false);
  }

  if (ImGui::TreeNodeEx("Entity -> Color by distance", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (editor_layer->DragAndDropButton(state.color_entity, "Drop Entity here...")) {
      if (const auto entity = state.color_entity.Get(); scene->IsEntityValid(entity)) {
        BillboardCloud billboard_cloud{};
        billboard_cloud.Process(scene, entity);
        for (auto& element : billboard_cloud.elements) {
          const auto level_set = element.CalculateLevelSets();
          Entity clone = scene->CreateEntity("Cloned");
          const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(clone).lock();
          const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
          VertexAttributes attributes{};
          attributes.color = true;
          attributes.normal = true;
          attributes.tangent = true;
          mesh->SetVertices(attributes, element.vertices, element.triangles);
          mesh_renderer->mesh = mesh;

          const auto material = AssetManager::CreateTemporaryAsset<Material>();
          material->vertex_color_only = true;
          mesh_renderer->material = material;
        }
      }
      state.color_entity.Clear();
    }
    ImGui::TreePop();
  }

  return changed;
}
