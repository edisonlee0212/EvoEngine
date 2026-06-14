//
// Created by lllll on 10/25/2022.
//

#include "ForestDescriptor.hpp"
#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "Platform.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_package;

Entity ForestPatch::InstantiatePatch(const glm::ivec2& gridSize, const bool setSimulationSettings) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
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
  const glm::vec2 startPoint = glm::vec2((gridSize.x - 1) * grid_distance.x, (gridSize.y - 1) * grid_distance.y) * 0.5f;

  const auto retVal = scene->CreateEntity("Forest (" + std::to_string(gridSize.x * gridSize.y) + ") - " + GetTitle());
  const auto forest = scene->CreateEntity("Center");
  const auto boundary = scene->CreateEntity("Boundary");
  scene->SetParent(forest, retVal);
  scene->SetParent(boundary, retVal);

  int index = 0;

  const auto offset = glm::linearRand(glm::vec3(-10000), glm::vec3(10000));
  for (int i = 0; i < gridSize.x; i++) {
    for (int j = 0; j < gridSize.y; j++) {
      auto position = glm::vec3(-startPoint.x + i * grid_distance.x, 0.0f, -startPoint.y + j * grid_distance.y);
      position.x +=
          glm::linearRand(-grid_distance.x * position_offset_mean.x, grid_distance.x * position_offset_mean.x);
      position.z +=
          glm::linearRand(-grid_distance.y * position_offset_mean.y, grid_distance.y * position_offset_mean.y);
      position +=
          glm::gaussRand(glm::vec3(0.0f), glm::vec3(position_offset_variance.x, 0.0f, position_offset_variance.y));
      if (heightField)
        position.y = heightField->GetValue({position.x, position.z}) - 0.01f;
      GlobalTransform transform{};
      transform.SetPosition(position);
      auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), rotation_offset_variance))));
      transform.SetRotation(rotation);
      transform.SetScale(glm::vec3(1.f));

      auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(index));
      index++;

      scene->SetDataComponent(treeEntity, transform);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      tree->shoot_model.tree_growth_settings = tree_growth_settings;
      tree->tree_descriptor_ref = tree_descriptor.Get<TreeDescriptor>();
      if (i == 0 || j == 0 || i == gridSize.x - 1 || j == gridSize.y - 1) {
        scene->SetParent(treeEntity, boundary);
        tree->generate_mesh = false;
      } else {
        scene->SetParent(treeEntity, forest);
        tree->generate_mesh = true;
      }
      tree->start_time = glm::linearRand(0.0f, start_time_max);
    }
  }

  if (setSimulationSettings) {
    const auto lab = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
    lab->simulation_settings = simulation_settings;
  }

  return retVal;
}

Entity ForestPatch::InstantiatePatch(
    const std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>>& candidates,
    const glm::ivec2& gridSize, bool setSimulationSettings) const {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  std::shared_ptr<Soil> soil;
  const auto soilCandidate = EcoSysLabLayer::FindSoil();
  if (!soilCandidate.expired())
    soil = soilCandidate.lock();
  std::shared_ptr<SoilDescriptor> soil_descriptor;
  if (soil) {
    soil_descriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>();
  }
  std::shared_ptr<HeightField> heightField{};
  if (soil_descriptor) {
    heightField = soil_descriptor->height_field.Get<HeightField>();
  }
  const glm::vec2 startPoint = glm::vec2((gridSize.x - 1) * grid_distance.x, (gridSize.y - 1) * grid_distance.y) * 0.5f;

  const auto retVal = scene->CreateEntity("Forest (" + std::to_string(gridSize.x * gridSize.y) + ") - " + GetTitle());
  const auto forest = scene->CreateEntity("Center");
  const auto boundary = scene->CreateEntity("Boundary");
  scene->SetParent(forest, retVal);
  scene->SetParent(boundary, retVal);

  int index = 0;

  const auto offset = glm::linearRand(glm::vec3(-10000), glm::vec3(10000));
  for (int i = 0; i < gridSize.x; i++) {
    for (int j = 0; j < gridSize.y; j++) {
      auto position = glm::vec3(-startPoint.x + i * grid_distance.x, 0.0f, -startPoint.y + j * grid_distance.y);
      position.x +=
          glm::linearRand(-grid_distance.x * position_offset_mean.x, grid_distance.x * position_offset_mean.x);
      position.z +=
          glm::linearRand(-grid_distance.y * position_offset_mean.y, grid_distance.y * position_offset_mean.y);
      position +=
          glm::gaussRand(glm::vec3(0.0f), glm::vec3(position_offset_variance.x, 0.0f, position_offset_variance.y));
      if (heightField)
        position.y = heightField->GetValue({position.x, position.z}) - 0.01f;
      GlobalTransform transform{};
      transform.SetPosition(position);
      auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), rotation_offset_variance))));
      transform.SetRotation(rotation);
      transform.SetScale(glm::vec3(1.f));

      auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(index));
      index++;

      scene->SetDataComponent(treeEntity, transform);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

      const auto candidateIndex = glm::linearRand(0, static_cast<int>(candidates.size() - 1));
      tree->shoot_model.tree_growth_settings = candidates.at(candidateIndex).first;
      tree->tree_descriptor_ref = candidates.at(candidateIndex).second;  // tree_descriptor_ref.Get<TreeDescriptor>();
      if (i == 0 || j == 0 || i == gridSize.x - 1 || j == gridSize.y - 1) {
        scene->SetParent(treeEntity, boundary);
        tree->generate_mesh = false;
      } else {
        scene->SetParent(treeEntity, forest);
        tree->generate_mesh = true;
      }
      tree->start_time = glm::linearRand(0.0f, start_time_max);
    }
  }

  if (setSimulationSettings) {
    const auto lab = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
    lab->simulation_settings = simulation_settings;
  }

  return retVal;
}

std::shared_ptr<Texture2D> ForestPatch::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/ForestPatch.png"));
  }
  return thumbnail;
}

void ForestPatch::CollectAssetRef(std::vector<AssetRef>& list) {
  if (tree_descriptor.Get<TreeDescriptor>())
    list.push_back(tree_descriptor);
}

void eco_sys_lab_package::SerializeForestPatch(YAML::Emitter& out, const ForestPatch& target) {
  out << YAML::Key << "grid_distance" << YAML::Value << target.grid_distance;
  out << YAML::Key << "position_offset_mean" << YAML::Value << target.position_offset_mean;
  out << YAML::Key << "position_offset_variance" << YAML::Value << target.position_offset_variance;
  out << YAML::Key << "rotation_offset_variance" << YAML::Value << target.rotation_offset_variance;

  out << YAML::Key << "simulation_time" << YAML::Value << target.simulation_time;
  out << YAML::Key << "start_time_max" << YAML::Value << target.start_time_max;

  target.tree_descriptor.Save("tree_descriptor", out);

  target.simulation_settings.Save("simulation_settings", out);
}

void eco_sys_lab_package::DeserializeForestPatch(const YAML::Node& in, ForestPatch& target) {
  if (in["grid_distance"])
    target.grid_distance = in["grid_distance"].as<glm::vec2>();
  if (in["position_offset_mean"])
    target.position_offset_mean = in["position_offset_mean"].as<glm::vec2>();
  if (in["position_offset_variance"])
    target.position_offset_variance = in["position_offset_variance"].as<glm::vec2>();
  if (in["rotation_offset_variance"])
    target.rotation_offset_variance = in["rotation_offset_variance"].as<glm::vec3>();

  if (in["simulation_time"])
    target.simulation_time = in["simulation_time"].as<float>();
  if (in["start_time_max"])
    target.start_time_max = in["start_time_max"].as<float>();
  target.tree_descriptor.Load("tree_descriptor", in);

  target.simulation_settings.Load("simulation_settings", in);
}

bool ForestPatch::DrawGui(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  editorLayer->DragAndDropButton<TreeDescriptor>(tree_descriptor, "TreeDescriptor");
  static glm::ivec2 gridSize = {8, 8};
  ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
  if (ImGui::DragFloat2("Grid distance", &grid_distance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  ImGui::Separator();
  if (ImGui::DragFloat2("Position offset mean", &position_offset_mean.x, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat2("Position offset variance", &position_offset_variance.x, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat2("Rotation offset variance", &rotation_offset_variance.x, 0.01f, 0.0f, 5.f))
    changed = true;
  static bool setParent = true;
  ImGui::Checkbox("Set Parent", &setParent);
  static bool setSimulationSettings = true;
  ImGui::Checkbox("Set Simulation settings", &setSimulationSettings);
  if (ImGui::TreeNodeEx("Simulation Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (simulation_settings.DrawGui(editorLayer))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::DragFloat("Simulation time", &simulation_time, 0.1f, 0.0f, 100.f))
    changed = true;
  if (ImGui::DragFloat("Start time max", &start_time_max, 0.01f, 0.0f, 10.f))
    changed = true;

  if (ImGui::Button("Instantiate")) {
    InstantiatePatch(gridSize, setParent);
  }
  FileUtils::OpenFolder("Create forest from folder...", [&](const std::filesystem::path& folderPath) {
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
          treeDescriptors.emplace_back(std::make_pair(tree_growth_settings, treeDescriptor));
        }
        index++;
      }
    }
    if (!treeDescriptors.empty()) {
      const auto patch = InstantiatePatch(treeDescriptors, gridSize, setParent);
    }
  });
  return changed;
}

void eco_sys_lab_package::SerializeTreeInfo(YAML::Emitter& out, const TreeInfo& target) {
  out << YAML::Key << "global_transform" << YAML::Value << target.global_transform.value;
  target.tree_descriptor.Save("tree_descriptor", out);
}

void eco_sys_lab_package::DeserializeTreeInfo(const YAML::Node& in, TreeInfo& target) {
  if (in["global_transform"])
    target.global_transform.value = in["global_transform"].as<glm::mat4>();
  target.tree_descriptor.Load("tree_descriptor", in);
}

void TreeInfo::CollectAssetRef(std::vector<AssetRef>& list) const {
  list.push_back(tree_descriptor);
}

std::shared_ptr<Texture2D> ForestDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/ForestDescriptor.png"));
  }
  return thumbnail;
}

void ForestDescriptor::ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor) {
  if (treeDescriptor) {
    for (auto& i : tree_infos) {
      i.tree_descriptor = treeDescriptor;
    }
  }
}

void ForestDescriptor::ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors) {
  if (treeDescriptors.empty())
    return;
  for (auto& i : tree_infos) {
    i.tree_descriptor = treeDescriptors.at(glm::linearRand(0, static_cast<int>(treeDescriptors.size()) - 1));
  }
}

void ForestDescriptor::ApplyTreeDescriptors(const std::filesystem::path& folderPath) {
  std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
          ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(i.path())));
      collectedTreeDescriptors.emplace_back(treeDescriptor);
    }
  }
  ApplyTreeDescriptors(collectedTreeDescriptors);
}

void ForestDescriptor::ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors,
                                            const std::vector<float>& ratios) {
  if (treeDescriptors.empty())
    return;
  for (auto& i : tree_infos) {
    i.tree_descriptor = treeDescriptors.at(glm::linearRand(0, static_cast<int>(treeDescriptors.size()) - 1));
  }

  std::random_device rd;
  std::mt19937 g(rd());
  auto copiedDescriptors = treeDescriptors;
  std::shuffle(copiedDescriptors.begin(), copiedDescriptors.end(), g);
  std::vector<std::shared_ptr<TreeDescriptor>> appliedTreeDescriptors;
  int count = 0;
  for (int i = 0; i < ratios.size(); i++) {
    if (count >= tree_infos.size())
      break;
    const int localSize = tree_infos.size() * ratios[i];
    for (int j = 0; j < localSize; j++) {
      if (count >= tree_infos.size())
        break;
      tree_infos[count].tree_descriptor = copiedDescriptors[i];
      count++;
    }
  }
}

void ForestDescriptor::ApplyTreeDescriptors(const std::filesystem::path& folderPath, const std::vector<float>& ratios) {
  std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
          ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(i.path())));
      collectedTreeDescriptors.emplace_back(treeDescriptor);
    }
  }
  ApplyTreeDescriptors(collectedTreeDescriptors, ratios);
}

bool ForestDescriptor::DrawGui(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  static glm::ivec2 gridSize = {4, 4};
  static float gridDistance = 1.5f;
  static float randomShift = 0.5f;
  static bool setParent = true;
  static bool enableHistory = false;
  static int historyIteration = 30;
  ImGui::Checkbox("Enable history", &enableHistory);
  if (enableHistory)
    ImGui::DragInt("History iteration", &historyIteration, 1, 1, 999);
  if (ImGui::TreeNodeEx("Grid...", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
    ImGui::DragFloat("Grid distance", &gridDistance, 0.1f, 0.0f, 100.0f);
    ImGui::DragFloat("Random shift", &randomShift, 0.01f, 0.0f, 0.5f);
    if (ImGui::Button("Reset Grid")) {
      SetupGrid(gridSize, gridDistance, randomShift);
    }
    ImGui::TreePop();
  }

  FileUtils::OpenFolder(
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
            tree_infos.emplace_back();
            glm::vec3 position = glm::vec3(5.f * index, 0.0f, 0.0f);
            if (heightField)
              position.y = heightField->GetValue({position.x, position.z}) - 0.05f;
            tree_infos.back().global_transform.SetPosition(position);
            tree_infos.back().tree_descriptor = treeDescriptor;
            index++;
          }
        }
      },
      false);

  FileUtils::OpenFolder(
      "Randomly assign tree descriptors",
      [&](const std::filesystem::path& path) {
        ApplyTreeDescriptors(path);
      },
      false);
  static AssetRef treeDescriptorRef;
  if (editorLayer->DragAndDropButton<TreeDescriptor>(treeDescriptorRef, "Apply all with tree descriptor...", true)) {
    if (const auto treeDescriptor = treeDescriptorRef.Get<TreeDescriptor>()) {
      ApplyTreeDescriptor(treeDescriptor);
    }
    treeDescriptorRef.Clear();
  }

  if (ImGui::TreeNode("Tree Instances")) {
    int index = 1;
    for (auto& i : tree_infos) {
      editorLayer->DragAndDropButton<TreeDescriptor>(i.tree_descriptor, "Tree No." + std::to_string(index), true);
      index++;
    }
    ImGui::TreePop();
  }

  if (ImGui::Button("Instantiate patch")) {
    InstantiatePatch(setParent, 0);
  }

  if (!tree_infos.empty() && ImGui::Button("Clear")) {
    tree_infos.clear();
  }

  return changed;
}

void ForestDescriptor::OnCreate() {
}

void ForestDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  for (const auto& i : tree_infos) {
    i.CollectAssetRef(list);
  }
}

void eco_sys_lab_package::SerializeForestDescriptor(YAML::Emitter& out, const ForestDescriptor& target) {
  out << YAML::Key << "tree_infos" << YAML::BeginSeq;
  for (const auto& i : target.tree_infos) {
    SerializeTreeInfo(out, i);
  }
  out << YAML::EndSeq;
  target.tree_growth_settings.Save("tree_growth_settings", out);
}

void eco_sys_lab_package::DeserializeForestDescriptor(const YAML::Node& in, ForestDescriptor& target) {
  if (in["tree_infos"]) {
    target.tree_infos.clear();
    for (const auto& i : in["tree_infos"]) {
      target.tree_infos.emplace_back();
      auto& back = target.tree_infos.back();
      DeserializeTreeInfo(i, back);
    }
  }
  target.tree_growth_settings.Load("tree_growth_settings", in);
}

auto ForestDescriptor::SetupGrid(const glm::ivec2& grid_size, const float grid_distance, float random_shift) -> void {
  tree_infos.clear();
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Soil> soil;
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  std::shared_ptr<SoilDescriptor> soil_descriptor;
  if (soil) {
    soil_descriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>();
  }
  std::shared_ptr<HeightField> height_field{};
  if (soil_descriptor) {
    height_field = soil_descriptor->height_field.Get<HeightField>();
  }
  const glm::vec2 start_point =
      glm::vec2((grid_size.x - 0.5f) * grid_distance, (grid_size.y - 0.5f) * grid_distance) * 0.5f;
  for (int i = 0; i < grid_size.x; i++) {
    for (int j = 0; j < grid_size.y; j++) {
      tree_infos.emplace_back();
      glm::vec3 position = glm::vec3(-start_point.x + i * grid_distance, 0.0f, -start_point.y + j * grid_distance);
      position.x += glm::linearRand(-grid_distance * random_shift, grid_distance * random_shift);
      position.z += glm::linearRand(-grid_distance * random_shift, grid_distance * random_shift);
      if (height_field)
        position.y = height_field->GetValue({position.x, position.z}) - 0.05f;
      tree_infos.back().global_transform.SetPosition(position);
    }
  }
}

Entity ForestDescriptor::InstantiatePatch(const bool set_parent, const int seed) const {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  Entity parent;
  if (set_parent) {
    parent = scene->CreateEntity("Forest (" + std::to_string(tree_infos.size()) + ") - " + GetTitle());
  }
  int i = 0;
  for (const auto& gt : tree_infos) {
    auto tree_entity = scene->CreateEntity("Tree No." + std::to_string(i));
    i++;
    scene->SetDataComponent(tree_entity, gt.global_transform);
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->shoot_model.tree_growth_settings = tree_growth_settings;
    tree->tree_descriptor_ref = gt.tree_descriptor;
    tree->shoot_model.seed = seed * tree_infos.size();
    if (set_parent)
      scene->SetParent(tree_entity, parent);
  }
  return parent;
}
