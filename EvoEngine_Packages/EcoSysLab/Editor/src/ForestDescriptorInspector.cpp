#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabObjectInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "ForestDescriptor.hpp"
#include "Platform.hpp"
#include "Tree.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

bool ForestPatchInspector::Inspect(InspectorContext& context, ForestPatch& target) {
  const auto& editorLayer = context.editor_layer;
  bool changed = false;
  editorLayer->DragAndDropButton<TreeDescriptor>(target.tree_descriptor, "TreeDescriptor");

  ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
  if (ImGui::DragFloat2("Grid distance", &target.grid_distance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  ImGui::Separator();
  if (ImGui::DragFloat2("Position offset mean", &target.position_offset_mean.x, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat2("Position offset variance", &target.position_offset_variance.x, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat2("Rotation offset variance", &target.rotation_offset_variance.x, 0.01f, 0.0f, 5.f))
    changed = true;

  ImGui::Checkbox("Set Parent", &setParent);

  ImGui::Checkbox("Set Simulation settings", &setSimulationSettings);
  if (ImGui::TreeNodeEx("Simulation Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectSettings(target.simulation_settings, editorLayer))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::DragFloat("Simulation time", &target.simulation_time, 0.1f, 0.0f, 100.f))
    changed = true;
  if (ImGui::DragFloat("Start time max", &target.start_time_max, 0.01f, 0.0f, 10.f))
    changed = true;

  if (ImGui::Button("Instantiate")) {
    target.InstantiatePatch(gridSize, setParent);
  }
  EditorFileDialogs::OpenFolder("Create forest from folder...", [&](const std::filesystem::path& folderPath) {
    int index = 0;
    const auto ecoSysLabLayer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
    std::shared_ptr<Soil> soil;
    const auto soilCandidate = EcoSysLabLayer::FindSoil();
    if (!soilCandidate.expired())
      soil = soilCandidate.lock();
    std::shared_ptr<SoilDescriptor> soilDescriptor;
    if (soil) {
      soilDescriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>();
    }
    std::shared_ptr<HeightField> heightField{};
    if (soilDescriptor) {
      heightField = soilDescriptor->height_field.Get<HeightField>();
    }
    std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>> treeDescriptors;
    for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath)) {
      if (i.is_regular_file() && i.path().extension().string() == ".tree") {
        const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
            ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(i.path())));
        if (treeDescriptor) {
          treeDescriptors.emplace_back(std::make_pair(target.tree_growth_settings, treeDescriptor));
        }
        index++;
      }
    }
    if (!treeDescriptors.empty()) {
      const auto patch = target.InstantiatePatch(treeDescriptors, gridSize, setParent);
    }
  });
  return changed;
}

bool ForestDescriptorInspector::Inspect(InspectorContext& context, ForestDescriptor& target) {
  const auto& editorLayer = context.editor_layer;
  bool changed = false;

  ImGui::Checkbox("Enable history", &enableHistory);
  if (enableHistory)
    ImGui::DragInt("History iteration", &historyIteration, 1, 1, 999);
  if (ImGui::TreeNodeEx("Grid...", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
    ImGui::DragFloat("Grid distance", &gridDistance, 0.1f, 0.0f, 100.0f);
    ImGui::DragFloat("Random shift", &randomShift, 0.01f, 0.0f, 0.5f);
    if (ImGui::Button("Reset Grid")) {
      target.SetupGrid(gridSize, gridDistance, randomShift);
    }
    ImGui::TreePop();
  }

  EditorFileDialogs::OpenFolder(
      "Parameters sample",
      [&](const std::filesystem::path& path) {
        int index = 0;
        const auto ecoSysLabLayer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
        std::shared_ptr<Soil> soil;
        const auto soilCandidate = EcoSysLabLayer::FindSoil();
        if (!soilCandidate.expired())
          soil = soilCandidate.lock();
        std::shared_ptr<SoilDescriptor> soilDescriptor;
        if (soil) {
          soilDescriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>();
        }
        std::shared_ptr<HeightField> heightField{};
        if (soilDescriptor) {
          heightField = soilDescriptor->height_field.Get<HeightField>();
        }
        for (const auto& i : std::filesystem::recursive_directory_iterator(path)) {
          if (i.is_regular_file() && i.path().extension().string() == ".tree") {
            const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
                ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(i.path())));
            target.tree_infos.emplace_back();
            glm::vec3 position = glm::vec3(5.f * index, 0.0f, 0.0f);
            if (heightField)
              position.y = heightField->GetValue({position.x, position.z}) - 0.05f;
            target.tree_infos.back().global_transform.SetPosition(position);
            target.tree_infos.back().tree_descriptor = treeDescriptor;
            index++;
          }
        }
      },
      false);

  EditorFileDialogs::OpenFolder(
      "Randomly assign tree descriptors",
      [&](const std::filesystem::path& path) {
        target.ApplyTreeDescriptors(path);
      },
      false);

  if (editorLayer->DragAndDropButton<TreeDescriptor>(treeDescriptorRef, "Apply all with tree descriptor...", true)) {
    if (const auto treeDescriptor = treeDescriptorRef.Get<TreeDescriptor>()) {
      target.ApplyTreeDescriptor(treeDescriptor);
    }
    treeDescriptorRef.Clear();
  }

  if (ImGui::TreeNode("Tree Instances")) {
    int index = 1;
    for (auto& i : target.tree_infos) {
      editorLayer->DragAndDropButton<TreeDescriptor>(i.tree_descriptor, "Tree No." + std::to_string(index), true);
      index++;
    }
    ImGui::TreePop();
  }

  if (ImGui::Button("Instantiate patch")) {
    target.InstantiatePatch(setParent, 0);
  }

  if (!target.tree_infos.empty() && ImGui::Button("Clear")) {
    target.tree_infos.clear();
  }

  return changed;
}
