#include "DatasetGenerator.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "ForestDescriptor.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
using namespace eco_sys_lab_plugin;
using namespace dataset_generation_plugin;
using namespace digital_agriculture_plugin;

bool CheckApplication() {
  const auto application_status = Application::GetApplicationStatus();
  if (application_status == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return false;
  }
  if (application_status == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return false;
  }
  if (application_status == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return false;
  }
  return true;
}

void DatasetGenerator::GenerateTreeTrunkMesh(const std::string& tree_parameters_path, float delta_time,
                                             int max_iterations, int max_tree_node_count,
                                             const TreeMeshGeneratorSettings& mesh_generator_settings,
                                             const std::string& tree_mesh_output_path,
                                             const std::string& tree_trunk_output_path,
                                             const std::string& tree_info_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> tree_descriptor;
  if (ProjectManager::IsInProjectFolder(tree_parameters_path)) {
    tree_descriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(tree_parameters_path)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (const auto& tree_entity : *tree_entities) {
      scene->DeleteEntity(tree_entity);
    }
  }

  const auto tree_entity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();

  tree->tree_descriptor = tree_descriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  eco_sys_lab_layer->m_simulationSettings.delta_time = delta_time;
  for (int i = 0; i < max_iterations; i++) {
    eco_sys_lab_layer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= max_tree_node_count) {
      break;
    }
  }
  tree->GenerateGeometryEntities(mesh_generator_settings);
  Application::Loop();
  tree->ExportTrunkObj(tree_trunk_output_path, mesh_generator_settings);
  tree->ExportObj(tree_mesh_output_path, mesh_generator_settings);
  Application::Loop();
  const auto& skeleton = tree->tree_model.RefShootSkeleton();
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
  if (sorted_internode_list.size() > 1) {
    float top_diameter = 0.0f;
    float base_diameter = 0.0f;
    float trunk_height = 0.0f;
    std::unordered_set<SkeletonNodeHandle> trunk_handles{};
    for (const auto& node_handle : sorted_internode_list) {
      const auto& node = skeleton.PeekNode(node_handle);
      trunk_handles.insert(node_handle);
      if (node.PeekChildHandles().size() > 1) {
        trunk_height = node.info.GetGlobalEndPosition().y;
        top_diameter = node.info.thickness;
        break;
      }
    }
    base_diameter = skeleton.PeekNode(0).info.thickness;
    std::ofstream of;
    of.open(tree_info_path, std::ofstream::out | std::ofstream::trunc);
    std::stringstream data;
    data << "TrunkHeight " << std::to_string(trunk_height) << "\n";
    data << "TrunkBaseDiameter " << std::to_string(base_diameter) << "\n";
    data << "TrunkTopDiameter " << std::to_string(top_diameter) << "\n";

    data << "TreeBoundingBoxMinX " << std::to_string(skeleton.min.x) << "\n";
    data << "TreeBoundingBoxMinY " << std::to_string(skeleton.min.y) << "\n";
    data << "TreeBoundingBoxMinZ " << std::to_string(skeleton.min.z) << "\n";
    data << "TreeBoundingBoxMaxX " << std::to_string(skeleton.max.x) << "\n";
    data << "TreeBoundingBoxMaxY " << std::to_string(skeleton.max.y) << "\n";
    data << "TreeBoundingBoxMaxZ " << std::to_string(skeleton.max.z) << "\n";
    const auto result = data.str();
    of.write(result.c_str(), result.size());
    of.flush();
  }
  scene->DeleteEntity(tree_entity);
  Application::Loop();
}

void DatasetGenerator::GenerateTreeMesh(const std::string& tree_parameters_path, float delta_time, int max_iterations,
                                        int max_tree_node_count,
                                        const TreeMeshGeneratorSettings& mesh_generator_settings,
                                        const std::string& tree_mesh_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInProjectFolder(tree_parameters_path)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(tree_parameters_path)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }

  const auto treeEntity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

  tree->tree_descriptor = treeDescriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  ecoSysLabLayer->m_simulationSettings.delta_time = delta_time;

  for (int i = 0; i < max_iterations; i++) {
    ecoSysLabLayer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= max_tree_node_count) {
      break;
    }
  }
  tree->GenerateGeometryEntities(mesh_generator_settings);
  Application::Loop();
  tree->ExportObj(tree_mesh_output_path, mesh_generator_settings);
  Application::Loop();
  scene->DeleteEntity(treeEntity);
  Application::Loop();
}

void DatasetGenerator::GenerateTreeMesh(const std::string& tree_parameters_path, float delta_time, int max_iterations,
                                        std::vector<int> target_tree_node_count,
                                        const TreeMeshGeneratorSettings& mesh_generator_settings,
                                        const std::string& tree_mesh_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInProjectFolder(tree_parameters_path)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(tree_parameters_path)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }

  const auto treeEntity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

  tree->tree_descriptor = treeDescriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  int testIndex = 0;
  std::filesystem::path basePath = tree_mesh_output_path;
  ecoSysLabLayer->m_simulationSettings.delta_time = delta_time;

  for (int i = 0; i < max_iterations; i++) {
    ecoSysLabLayer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= target_tree_node_count[testIndex]) {
      auto copyPath = basePath;
      tree->GenerateGeometryEntities(mesh_generator_settings);
      Application::Loop();
      copyPath.replace_filename(basePath.replace_extension("").string() + "_" + std::to_string(testIndex) + ".obj");
      tree->ExportObj(copyPath, mesh_generator_settings);
      testIndex++;
    }
    if (testIndex == target_tree_node_count.size())
      break;
  }

  Application::Loop();
  scene->DeleteEntity(treeEntity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForTree(const TreePointCloudPointSettings& point_settings,
                                                 const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                 const std::string& tree_parameters_path, const float delta_time,
                                                 const int max_iterations, const int max_tree_node_count,
                                                 const TreeMeshGeneratorSettings& mesh_generator_settings,
                                                 const std::string& point_cloud_output_path, bool export_tree_mesh,
                                                 const std::string& tree_mesh_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInProjectFolder(tree_parameters_path)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(tree_parameters_path)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }

  const auto treeEntity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

  tree->tree_descriptor = treeDescriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  ecoSysLabLayer->m_simulationSettings.delta_time = delta_time;

  for (int i = 0; i < max_iterations; i++) {
    ecoSysLabLayer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= max_tree_node_count) {
      break;
    }
  }
  tree->GenerateGeometryEntities(mesh_generator_settings);
  Application::Loop();
  if (export_tree_mesh) {
    tree->ExportObj(tree_mesh_output_path, mesh_generator_settings);
  }
  Application::Loop();
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
  scanner->m_pointSettings = point_settings;
  Application::Loop();
  scanner->Capture(mesh_generator_settings, point_cloud_output_path, capture_settings);
  Application::Loop();
  scene->DeleteEntity(treeEntity);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForest(const int grid_size, const float grid_distance,
                                                   const float random_shift,
                                                   const TreePointCloudPointSettings& point_settings,
                                                   const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                   const std::string& tree_parameters_folder_path, float delta_time,
                                                   int max_iterations, int max_tree_node_count,
                                                   const TreeMeshGeneratorSettings& mesh_generator_settings,
                                                   const std::string& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<ForestDescriptor> forestPatch = ProjectManager::CreateTemporaryAsset<ForestDescriptor>();
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soilEntities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soilEntities && !soilEntities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }

  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }
  forestPatch->m_treeGrowthSettings.use_space_colonization = false;

  Application::Loop();

  forestPatch->SetupGrid({grid_size, grid_size}, grid_distance, random_shift);
  forestPatch->ApplyTreeDescriptors(tree_parameters_folder_path, {1.f});
  forestPatch->InstantiatePatch(false);

  ecoSysLabLayer->m_simulationSettings.max_node_count = max_tree_node_count;
  ecoSysLabLayer->m_simulationSettings.delta_time = delta_time;

  for (int i = 0; i < max_iterations; i++) {
    ecoSysLabLayer->Simulate();
  }
  ecoSysLabLayer->GenerateMeshes(mesh_generator_settings);
  Application::Loop();
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
  scanner->m_pointSettings = point_settings;
  Application::Loop();
  scanner->Capture(mesh_generator_settings, point_cloud_output_path, capture_settings);

  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatch(
    const glm::ivec2& grid_size, const TreePointCloudPointSettings& point_settings,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    const std::shared_ptr<ForestPatch>& forest_patch, const TreeMeshGeneratorSettings& mesh_generator_settings,
    const std::string& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<Soil> soil;
  if (const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
      soil_entities && !soil_entities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }

  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (const auto& tree_entity : *tree_entities) {
      scene->DeleteEntity(tree_entity);
    }
    eco_sys_lab_layer->ResetAllTrees(tree_entities);
  } else {
    eco_sys_lab_layer->ResetAllTrees(tree_entities);
  }

  forest_patch->tree_growth_settings.use_space_colonization = false;

  Application::Loop();

  const auto forest = forest_patch->InstantiatePatch(grid_size, true);

  while (eco_sys_lab_layer->GetSimulatedTime() < forest_patch->simulation_time) {
    eco_sys_lab_layer->Simulate();
  }
  eco_sys_lab_layer->GenerateMeshes(mesh_generator_settings);
  Application::Loop();
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
  scanner->m_pointSettings = point_settings;
  Application::Loop();
  scanner->Capture(mesh_generator_settings, point_cloud_output_path, capture_settings);
  Application::Loop();
  scene->DeleteEntity(forest);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatchJoinedSpecies(
    const glm::ivec2& grid_size, const TreePointCloudPointSettings& point_settings,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    const std::shared_ptr<ForestPatch>& forest_patch, const std::string& species_folder_path,
    const TreeMeshGeneratorSettings& mesh_generator_settings, const std::string& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soil_entities && !soil_entities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }

  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (const auto& tree_entity : *tree_entities) {
      scene->DeleteEntity(tree_entity);
    }
    eco_sys_lab_layer->ResetAllTrees(tree_entities);
  } else {
    eco_sys_lab_layer->ResetAllTrees(tree_entities);
  }

  forest_patch->tree_growth_settings.use_space_colonization = false;

  Application::Loop();
  Entity forest;

  std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>> tree_descriptors;
  for (const auto& i : std::filesystem::recursive_directory_iterator(species_folder_path)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      if (const auto tree_descriptor = std::dynamic_pointer_cast<TreeDescriptor>(
              ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())))) {
        tree_descriptors.emplace_back(forest_patch->tree_growth_settings, tree_descriptor);
      }
    }
  }
  if (!tree_descriptors.empty()) {
    forest = forest_patch->InstantiatePatch(tree_descriptors, grid_size, true);
  }

  while (eco_sys_lab_layer->GetSimulatedTime() < forest_patch->simulation_time) {
    eco_sys_lab_layer->Simulate();
  }
  eco_sys_lab_layer->GenerateMeshes(mesh_generator_settings);
  Application::Loop();
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
  scanner->m_pointSettings = point_settings;
  Application::Loop();
  scanner->Capture(mesh_generator_settings, point_cloud_output_path, capture_settings);
  Application::Loop();
  if (scene->IsEntityValid(forest))
    scene->DeleteEntity(forest);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForSorghum(const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                                    const SorghumPointCloudPointSettings& point_settings,
                                                    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                    const bool avoid_occlusion,
                                                    const std::filesystem::path& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soil_entities && !soil_entities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }
  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  Application::Loop();
  const auto sorghum_entity = scene->CreateEntity("Sorghum");
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_descriptor = sorghum_descriptor;
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scanner_entity).lock();
  scanner->left_random_offset = {0, 0, 0};
  scanner->right_random_offset = {0, 0, 0};

  Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();
  scanner->sorghum_point_cloud_point_settings = point_settings;
  scanner->Capture(point_cloud_output_path, capture_settings);
  Application::Loop();

  if (avoid_occlusion) {
    auto mesh_settings_copy = sorghum_mesh_generator_settings;
    mesh_settings_copy.leaf_separated = true;
    std::vector<glm::vec3> points;
    std::vector<int> leaf_indices;
    std::vector<int> instance_indices;
    std::vector<int> type_indices;
    for (int leaf_index = 0; leaf_index < sorghum_descriptor->leaves.size(); leaf_index++) {
      mesh_settings_copy.single_leaf_index = leaf_index;
      sorghum->GenerateGeometryEntities(mesh_settings_copy);
      Application::Loop();
      scanner->sorghum_point_cloud_point_settings = point_settings;
      scanner->Scan(capture_settings, points, leaf_indices, instance_indices, type_indices);
      if (leaf_index == 0) {
        mesh_settings_copy.enable_stem = false;
        const auto children = scene->GetChildren(soil->GetOwner());
        for (const auto& child : children) {
          scene->DeleteEntity(child);
        }
      }
      sorghum->ClearGeometryEntities();
      Application::Loop();
    }
    auto temp_path = point_cloud_output_path;
    temp_path.replace_filename(temp_path.filename().stem().string() + "_nc.ply");
    scanner->SavePointCloud(temp_path, points, leaf_indices, instance_indices, type_indices);
  }

  scene->DeleteEntity(sorghum_entity);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForSorghumPatch(
    const SorghumFieldPatch& pattern, const std::shared_ptr<SorghumDescriptorGenerator>& sorghum_descriptor,
    const SorghumPointCloudPointSettings& point_settings,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
    const std::filesystem::path& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soil_entities && !soil_entities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }
  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  Application::Loop();
  const auto sorghum_field = ProjectManager::CreateTemporaryAsset<SorghumField>();
  std::vector<glm::mat4> matrices_list;
  pattern.GenerateField(matrices_list);
  sorghum_field->matrices.resize(matrices_list.size());
  for (int i = 0; i < matrices_list.size(); i++) {
    sorghum_field->matrices[i] = {sorghum_descriptor, matrices_list[i]};
  }

  const auto field = sorghum_field->InstantiateField();
  Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scanner_entity).lock();
  scanner->sorghum_point_cloud_point_settings = point_settings;
  Application::Loop();
  scanner->Capture(point_cloud_output_path, capture_settings);
  scene->DeleteEntity(field);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}
