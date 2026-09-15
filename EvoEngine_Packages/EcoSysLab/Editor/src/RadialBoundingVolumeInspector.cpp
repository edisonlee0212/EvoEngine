#include <Material.hpp>
#include "EcoSysLabAuthoringInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "Platform.hpp"
#include "PointCloud.hpp"
#include "RadialBoundingVolume.hpp"
#include "SDKInspectionAdapters.hpp"
#include "Tree.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
bool RadialBoundingVolumeInspector::Inspect(InspectorContext& context, RadialBoundingVolume& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  PrivateComponentRef treeRef;
  if (editor_layer->DragAndDropButton<Tree>(treeRef, "Apply tree volume")) {
    auto tree = treeRef.Get<Tree>();
    if (tree) {
      // CopyVolume(tree->shoot_model);
      treeRef.Clear();
      changed = true;
    }
  }

  ImGui::ColorEdit4("Display Color", &target.m_displayColor.x);
  ImGui::DragFloat("Display Scale", &target.m_displayScale, 0.01f, 0.01f, 1.0f);
  if (ImGui::DragInt("Layer Amount", &target.m_layerAmount, 1, 1, 100))
    changed = true;
  if (ImGui::DragInt("Slice Amount", &target.m_sectorAmount, 1, 1, 100))
    changed = true;
  if (ImGui::Button("Form Entity")) {
    target.FormEntity();
  }

  bool displayLayer = false;
  if (target.m_meshGenerated) {
    if (ImGui::TreeNodeEx("Transformations")) {
      ImGui::DragFloat("Max height", &target.m_maxHeight, 0.01f);

      ImGui::DragFloat("Augmentation radius", &augmentation, 0.01f);
      if (ImGui::Button("Process")) {
        target.Augmentation(augmentation);
      }
      if (ImGui::Button("Generate mesh")) {
        target.GenerateMesh();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Layers", ImGuiTreeNodeFlags_DefaultOpen)) {
      for (int i = 0; i < target.m_layerAmount; i++) {
        if (ImGui::TreeNodeEx(("Layer " + std::to_string(i)).c_str())) {
          for (int j = 0; j < target.m_sectorAmount; j++) {
            if (ImGui::DragFloat(("Sector " + std::to_string(j) + "##" + std::to_string(i)).c_str(),
                                 &target.m_layers[i][j].m_maxDistance, 0.1f, 0.0f, 100.0f))
              target.GenerateMesh();
          }

          editor_layer->DrawGizmoMesh(target.m_boundMeshes[i], target.m_displayColor);

          displayLayer = true;
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
  }
  EditorFileDialogs::SaveFile("Save RBV", "RBV", {".rbv"}, [&target](const std::filesystem::path& path) {
    const std::string data = target.AsString();
    std::ofstream ofs;
    ofs.open(path.string().c_str(), std::ofstream::out | std::ofstream::trunc);
    ofs.write(data.c_str(), data.length());
    ofs.flush();
    ofs.close();
  });
  EditorFileDialogs::OpenFile("Load RBV", "RBV", {".rbv"}, [&target](const std::filesystem::path& path) {
    target.FromString(FileUtils::LoadFileAsString(path));
  });
  EditorFileDialogs::SaveFile("Export RBV as OBJ", "3D Model", {".obj"}, [&target](const std::filesystem::path& path) {
    target.ExportAsObj(path.string());
  });

  if (editor_layer->DragAndDropButton(pointCloud, "Import from Point Cloud", {"PointCloud"}, true)) {
    if (auto pc = pointCloud.Get<PointCloud>()) {
      std::vector<glm::vec3> points;
      points.resize(pc->positions.size());
      for (int i = 0; i < pc->positions.size(); i++) {
        auto point = pc->positions[i] + pc->offset;
        points[i] = glm::vec3(point.z, point.x, point.y);
      }
      target.CalculateVolume(points);
      pointCloud.Clear();
    }
  }

  ImGui::Checkbox("Display Bound", &displayBound);
  if (!displayLayer && displayBound && target.m_meshGenerated) {
    for (auto& i : target.m_boundMeshes) {
      editor_layer->DrawGizmoMesh(i, target.m_displayColor);
    }
    for (int i = 0; i < target.m_layerAmount; i++) {
      for (int j = 0; j < target.m_sectorAmount; j++) {
        editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cube, glm::vec4(0, 0, 0, 1),
                                    glm::translate(target.TipPosition(i, j)) * glm::scale(glm::vec3(0.1f)));
      }
    }
  }
  return changed;
}
