#include "DatasetGenerator.hpp"

#include "BarkDescriptor.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "ForestDescriptor.hpp"
#include "PostProcessingStack.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
using namespace eco_sys_lab_plugin;
using namespace dataset_generation_plugin;
using namespace digital_agriculture_plugin;

bool CheckApplication() {
  const auto application_status = Application::GetApplicationStatus();
  if (!Application::GetActiveScene()) {
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

bool CheckSoil(std::shared_ptr<Soil>& soil, bool generate_ground_mesh) {
  const auto scene = Application::GetActiveScene();
  if (const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
      soil_entities && !soil_entities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return false;
  }
  soil->RandomOffset(0, 99999);
  if (generate_ground_mesh)
    soil->GenerateMesh(0.0f, 0.0f);
  return true;
}

void DatasetGenerator::GenerateDataForTree(const TreeDataGenerationParameters& data_generation_parameters,
                                           const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) {
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
  if (!CheckSoil(soil, data_generation_parameters.generate_ground_mesh))
    return;
  std::shared_ptr<SoilDescriptor> soil_descriptor;
  if (soil) {
    soil_descriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>();
  }
  std::shared_ptr<HeightField> height_field{};
  if (soil_descriptor) {
    height_field = soil_descriptor->height_field.Get<HeightField>();
  }
  std::shared_ptr<TreeDescriptor> actual_tree_descriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();

  if (data_generation_parameters.tree_descriptor_path.is_relative()) {
    std::shared_ptr<TreeDescriptor> tree_descriptor;
    const auto absolute_path = ProjectManager::GetAssetsFolderPath() / data_generation_parameters.tree_descriptor_path;
    if (std::filesystem::exists(absolute_path)) {
      tree_descriptor = std::dynamic_pointer_cast<TreeDescriptor>(
          ProjectManager::GetOrCreateAsset(data_generation_parameters.tree_descriptor_path));
    } else {
      EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
      return;
    }
    actual_tree_descriptor->shoot_descriptor = tree_descriptor->shoot_descriptor;
    actual_tree_descriptor->foliage_descriptor = tree_descriptor->foliage_descriptor;
    actual_tree_descriptor->bark_descriptor = tree_descriptor->bark_descriptor;
    actual_tree_descriptor->fruit_descriptor = tree_descriptor->fruit_descriptor;
    actual_tree_descriptor->flower_descriptor = tree_descriptor->flower_descriptor;
  } else {
    std::shared_ptr<TreeDescriptor> tree_descriptor;
    if (ProjectManager::IsInAssetsFolder(data_generation_parameters.tree_descriptor_path)) {
      tree_descriptor = std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(
          ProjectManager::GetAssetsRelativePath(data_generation_parameters.tree_descriptor_path)));
    } else {
      EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
      return;
    }
    actual_tree_descriptor->shoot_descriptor = tree_descriptor->shoot_descriptor;
    actual_tree_descriptor->foliage_descriptor = tree_descriptor->foliage_descriptor;
    actual_tree_descriptor->bark_descriptor = tree_descriptor->bark_descriptor;
    actual_tree_descriptor->fruit_descriptor = tree_descriptor->fruit_descriptor;
    actual_tree_descriptor->flower_descriptor = tree_descriptor->flower_descriptor;
  }

  if (!data_generation_parameters.foliage_descriptor_path.empty()) {
    if (data_generation_parameters.foliage_descriptor_path.is_relative()) {
      const auto absolute_path =
          ProjectManager::GetAssetsFolderPath() / data_generation_parameters.foliage_descriptor_path;
      if (std::filesystem::exists(absolute_path)) {
        actual_tree_descriptor->foliage_descriptor = std::dynamic_pointer_cast<FoliageDescriptor>(
            ProjectManager::GetOrCreateAsset(data_generation_parameters.foliage_descriptor_path));
      } else {
        EVOENGINE_ERROR("Foliage Descriptor doesn't exist!");
      }
    } else {
      if (ProjectManager::IsInAssetsFolder(data_generation_parameters.foliage_descriptor_path)) {
        actual_tree_descriptor->foliage_descriptor =
            std::dynamic_pointer_cast<FoliageDescriptor>(ProjectManager::GetOrCreateAsset(
                ProjectManager::GetAssetsRelativePath(data_generation_parameters.foliage_descriptor_path)));
      } else {
        EVOENGINE_ERROR("Foliage Descriptor doesn't exist!");
        return;
      }
    }
  }

  if (!data_generation_parameters.bark_descriptor_path.empty()) {
    if (data_generation_parameters.bark_descriptor_path.is_relative()) {
      const auto absolute_path =
          ProjectManager::GetAssetsFolderPath() / data_generation_parameters.bark_descriptor_path;
      if (std::filesystem::exists(absolute_path)) {
        actual_tree_descriptor->bark_descriptor = std::dynamic_pointer_cast<BarkDescriptor>(
            ProjectManager::GetOrCreateAsset(data_generation_parameters.bark_descriptor_path));
      } else {
        EVOENGINE_ERROR("Bark Descriptor doesn't exist!");
      }
    } else {
      if (ProjectManager::IsInAssetsFolder(data_generation_parameters.bark_descriptor_path)) {
        actual_tree_descriptor->bark_descriptor =
            std::dynamic_pointer_cast<BarkDescriptor>(ProjectManager::GetOrCreateAsset(
                ProjectManager::GetAssetsRelativePath(data_generation_parameters.bark_descriptor_path)));
      } else {
        EVOENGINE_ERROR("Bark Descriptor doesn't exist!");
        return;
      }
    }
  }

  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (const auto& tree_entity : *tree_entities) {
      scene->DeleteEntity(tree_entity);
    }
  }
  std::filesystem::create_directories(data_generation_parameters.output_folder);

  const auto tree_entity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();

  if (height_field) {
    auto tree_position = glm::vec3(0.0f);
    tree_position.y = height_field->GetValue({tree_position.x, tree_position.z}) - 0.01f;
    GlobalTransform gt{};
    gt.SetPosition(tree_position);
    scene->SetDataComponent(tree_entity, gt);
  }

  tree->pruning_settings = data_generation_parameters.pruning_settings;
  tree->tree_descriptor_ref = actual_tree_descriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  int max_iterations = 2048;
  if (data_generation_parameters.max_iteration > 0) {
    max_iterations = data_generation_parameters.max_iteration;
  }
  SimulationStats stats;
  std::set<int> growth_capture_node_sizes;
  for (const auto& i : data_generation_parameters.growth_capture) {
    growth_capture_node_sizes.emplace(i);
  }
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto camera_entity = scene->CreateEntity("Capture Camera");
  const auto capture_data = [&](const int post_fix_index) {
    const std::string post_fix = post_fix_index == -1 ? "" : std::string("_") + std::to_string(post_fix_index);
    tree->GenerateGeometryEntities(data_generation_parameters.tree_mesh_generator_settings);

    Application::Loop();
    Application::Loop();
    if (data_generation_parameters.export_mesh) {
      tree->ExportObj(data_generation_parameters.output_folder /
                          (data_generation_parameters.output_file_prefix + post_fix + ".obj"),
                      data_generation_parameters.tree_mesh_generator_settings);
    }
    if (data_generation_parameters.export_skeleton) {
      tree->ExportFlowGraph(data_generation_parameters.output_folder /
                            (data_generation_parameters.output_file_prefix + post_fix + ".yml"));
    }
    if (data_generation_parameters.export_point_cloud) {
      const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
      scanner->point_settings = data_generation_parameters.tree_point_cloud_point_settings;
      scanner->Capture(data_generation_parameters.tree_mesh_generator_settings,
                       data_generation_parameters.output_folder /
                           (data_generation_parameters.output_file_prefix + post_fix + ".ply"),
                       capture_settings);
    }

    if (data_generation_parameters.export_rendering || data_generation_parameters.export_depth) {
      const auto camera = scene->GetOrSetPrivateComponent<Camera>(camera_entity).lock();
      for (int image_index = 0; image_index < data_generation_parameters.camera_capture_settings.size();
           image_index++) {
        const auto& camera_capture_settings = data_generation_parameters.camera_capture_settings[image_index];
        camera->camera_settings = camera_capture_settings.camera_settings;
        camera->post_processing_stack_ref.Get<PostProcessingStack>()->enable_bloom = false;
        camera->Resize(camera_capture_settings.render_resolution);
        camera->SetRequireRendering(true);
        scene->SetDataComponent(camera_entity, camera_capture_settings.global_transform);
        Application::Loop();
        if (data_generation_parameters.export_rendering) {
          camera->GetRenderTexture()->StoreToPng(
              data_generation_parameters.output_folder / (data_generation_parameters.output_file_prefix + post_fix +
                                                          "_" + std::to_string(image_index) + ".png"),
              camera_capture_settings.output_resolution.x, camera_capture_settings.output_resolution.y);
        }
        if (data_generation_parameters.export_depth) {
          camera->GetRenderTexture()->StoreLinearDepthToPng(
              data_generation_parameters.output_folder / (data_generation_parameters.output_file_prefix + post_fix +
                                                          "_" + std::to_string(image_index) + "_d.png"),
              camera->camera_settings.near_distance, camera->camera_settings.far_distance,
              data_generation_parameters.max_depth, camera_capture_settings.output_resolution.x,
              camera_capture_settings.output_resolution.y);
        }
      }
    }
  };
  int post_fix = 0;
  for (int i = 0; i < max_iterations; i++) {
    eco_sys_lab_layer->Simulate(data_generation_parameters.simulation_settings, stats);
    if (!growth_capture_node_sizes.empty()) {
      if (data_generation_parameters.use_node_growth_capture) {
        if (const auto min_node_size = *growth_capture_node_sizes.begin(); stats.internode_size >= min_node_size) {
          capture_data(post_fix);
          post_fix++;
          growth_capture_node_sizes.erase(growth_capture_node_sizes.begin());
        }
      } else {
        if (const auto min_node_size = *growth_capture_node_sizes.begin(); stats.shoot_stem_size >= min_node_size) {
          capture_data(post_fix);
          post_fix++;
          growth_capture_node_sizes.erase(growth_capture_node_sizes.begin());
        }
      }
    }
  }
  if (data_generation_parameters.growth_capture.empty()) {
    capture_data(-1);
  } else {
    capture_data(post_fix);
  }
  scene->DeleteEntity(camera_entity);
  scene->DeleteEntity(scanner_entity);
  scene->DeleteEntity(tree_entity);

  Application::Loop();
}

void DatasetGenerator::GenerateDataForForest(int grid_size, float grid_distance, float random_shift,
                                             const TreeDataGenerationParameters& data_generation_parameters,
                                             const std::filesystem::path& species_folder_path,
                                             const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  const std::shared_ptr<ForestDescriptor> forest_descriptor = AssetManager::CreateTemporaryAsset<ForestDescriptor>();
  if (std::shared_ptr<Soil> soil; !CheckSoil(soil, data_generation_parameters.generate_ground_mesh))
    return;
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (const auto& tree_entity : *tree_entities) {
      scene->DeleteEntity(tree_entity);
    }
  }
  std::filesystem::create_directories(data_generation_parameters.output_folder);

  Application::Loop();

  forest_descriptor->SetupGrid({grid_size, grid_size}, grid_distance, random_shift);
  forest_descriptor->ApplyTreeDescriptors(species_folder_path, {1.f});
  const auto forest_entity = forest_descriptor->InstantiatePatch(false);

  int max_iterations = INT_MAX;
  if (data_generation_parameters.max_iteration > 0) {
    max_iterations = data_generation_parameters.max_iteration;
  }

  SimulationStats stats;
  for (int i = 0; i < max_iterations; i++) {
    eco_sys_lab_layer->Simulate(data_generation_parameters.simulation_settings, stats);
  }
  eco_sys_lab_layer->GenerateMeshes(data_generation_parameters.tree_mesh_generator_settings);
  Application::Loop();
  /*
  const auto children = scene->GetChildren(forest_entity);
  for (const auto& child : children) {
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(child).lock();
    if (data_generation_parameters.export_mesh) {
      tree->ExportObj(?, data_generation_parameters.tree_mesh_generator_settings);
    }
    if (data_generation_parameters.export_skeleton) {
      tree->ExportFlowGraph(?);
    }
  }
  */
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
  scanner->point_settings = data_generation_parameters.tree_point_cloud_point_settings;
  Application::Loop();
  Application::Loop();
  scanner->Capture(data_generation_parameters.tree_mesh_generator_settings,
                   data_generation_parameters.output_folder / (data_generation_parameters.output_file_prefix + ".ply"),
                   capture_settings);
  scene->DeleteEntity(forest_entity);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatch(
    const glm::ivec2& grid_size, const std::shared_ptr<ForestPatch>& forest_patch,
    const TreeDataGenerationParameters& data_generation_parameters,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) {
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
  if (!CheckSoil(soil, data_generation_parameters.generate_ground_mesh))
    return;

  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (const auto& tree_entity : *tree_entities) {
      scene->DeleteEntity(tree_entity);
    }
  }
  std::filesystem::create_directories(data_generation_parameters.output_folder);

  Application::Loop();

  const auto forest_entity = forest_patch->InstantiatePatch(grid_size, true);
  int max_iterations = INT_MAX;
  if (data_generation_parameters.max_iteration > 0) {
    max_iterations = data_generation_parameters.max_iteration;
  }
  while (eco_sys_lab_layer->GetSimulatedTime() < forest_patch->simulation_time) {
    SimulationStats stats;
    for (int i = 0; i < max_iterations; i++) {
      eco_sys_lab_layer->Simulate(data_generation_parameters.simulation_settings, stats);
    }
  }
  eco_sys_lab_layer->GenerateMeshes(data_generation_parameters.tree_mesh_generator_settings);
  Application::Loop();
  /*
  const auto children = scene->GetChildren(forest_entity);
  for (const auto& child : children) {
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(child).lock();
    if (data_generation_parameters.export_mesh) {
      tree->ExportObj(?, data_generation_parameters.tree_mesh_generator_settings);
    }
    if (data_generation_parameters.export_skeleton) {
      tree->ExportFlowGraph(?);
    }
  }
  */
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
  scanner->point_settings = data_generation_parameters.tree_point_cloud_point_settings;
  Application::Loop();
  Application::Loop();
  scanner->Capture(data_generation_parameters.tree_mesh_generator_settings,
                   data_generation_parameters.output_folder / (data_generation_parameters.output_file_prefix + ".ply"),
                   capture_settings);
  scene->DeleteEntity(forest_entity);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatchJoinedSpecies(
    const glm::ivec2& grid_size, const std::shared_ptr<ForestPatch>& forest_patch,
    const std::filesystem::path& species_folder_path, const TreeDataGenerationParameters& data_generation_parameters,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) {
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
  if (!CheckSoil(soil, data_generation_parameters.generate_ground_mesh))
    return;
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    for (const auto& tree_entity : *tree_entities) {
      scene->DeleteEntity(tree_entity);
    }
  }
  std::filesystem::create_directories(data_generation_parameters.output_folder);

  Application::Loop();
  Entity forest_entity;

  std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>> tree_descriptors;
  for (const auto& i : std::filesystem::recursive_directory_iterator(species_folder_path)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      if (const auto tree_descriptor = std::dynamic_pointer_cast<TreeDescriptor>(
              ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(i.path())))) {
        tree_descriptors.emplace_back(forest_patch->tree_growth_settings, tree_descriptor);
      }
    }
  }
  if (!tree_descriptors.empty()) {
    forest_entity = forest_patch->InstantiatePatch(tree_descriptors, grid_size, true);
  } else {
    EVOENGINE_ERROR("Tree descriptors not found!")
    return;
  }
  int max_iterations = INT_MAX;
  if (data_generation_parameters.max_iteration > 0) {
    max_iterations = data_generation_parameters.max_iteration;
  }
  while (eco_sys_lab_layer->GetSimulatedTime() < forest_patch->simulation_time) {
    SimulationStats stats;
    for (int i = 0; i < max_iterations; i++) {
      eco_sys_lab_layer->Simulate(data_generation_parameters.simulation_settings, stats);
    }
  }
  eco_sys_lab_layer->GenerateMeshes(data_generation_parameters.tree_mesh_generator_settings);
  Application::Loop();
  /*
  const auto children = scene->GetChildren(forest_entity);
  for (const auto& child : children) {
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(child).lock();
    if (data_generation_parameters.export_mesh) {
      tree->ExportObj(?, data_generation_parameters.tree_mesh_generator_settings);
    }
    if (data_generation_parameters.export_skeleton) {
      tree->ExportFlowGraph(?);
    }
  }
  */
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scanner_entity).lock();
  scanner->point_settings = data_generation_parameters.tree_point_cloud_point_settings;
  Application::Loop();
  Application::Loop();
  scanner->Capture(data_generation_parameters.tree_mesh_generator_settings,
                   data_generation_parameters.output_folder / (data_generation_parameters.output_file_prefix + ".ply"),
                   capture_settings);
  scene->DeleteEntity(forest_entity);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForSorghum(const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                                    const SorghumPointCloudPointSettings& point_settings,
                                                    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                    const bool avoid_occlusion, bool generate_ground,
                                                    const std::filesystem::path& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  if (generate_ground) {
    if (!CheckSoil(soil, generate_ground))
      return;
  }
  const auto sorghum_entity = scene->CreateEntity("Sorghum");
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_descriptor = sorghum_descriptor;
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scanner_entity).lock();
  scanner->left_random_offset = {0, 0, 0};
  scanner->right_random_offset = {0, 0, 0};

  Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();
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
      Application::Loop();
      scanner->sorghum_point_cloud_point_settings = point_settings;
      scanner->Scan(capture_settings, points, leaf_indices, instance_indices, type_indices);
      if (leaf_index == 0) {
        mesh_settings_copy.enable_stem = false;
        if (generate_ground) {
          const auto children = scene->GetChildren(soil->GetOwner());
          for (const auto& child : children) {
            scene->DeleteEntity(child);
          }
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

void DatasetGenerator::GeneratePointCloudForSorghum(const std::shared_ptr<SorghumState>& sorghum_state,
                                                    const SorghumPointCloudPointSettings& point_settings,
                                                    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                    bool avoid_occlusion, bool generate_ground,
                                                    const std::filesystem::path& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  if (generate_ground) {
    if (!CheckSoil(soil, generate_ground))
      return;
  }

  const auto sorghum_entity = scene->CreateEntity("Sorghum");
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_state = sorghum_state;
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scanner_entity).lock();
  scanner->left_random_offset = {0, 0, 0};
  scanner->right_random_offset = {0, 0, 0};

  Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();
  Application::Loop();
  scanner->sorghum_point_cloud_point_settings = point_settings;
  scanner->Capture(point_cloud_output_path, capture_settings);

  if (avoid_occlusion) {
    auto mesh_settings_copy = sorghum_mesh_generator_settings;
    mesh_settings_copy.leaf_separated = true;
    std::vector<glm::vec3> points;
    std::vector<int> leaf_indices;
    std::vector<int> instance_indices;
    std::vector<int> type_indices;
    for (int leaf_index = 0; leaf_index < sorghum_state->leaves.size(); leaf_index++) {
      mesh_settings_copy.single_leaf_index = leaf_index;
      sorghum->GenerateGeometryEntities(mesh_settings_copy);
      Application::Loop();
      Application::Loop();
      scanner->sorghum_point_cloud_point_settings = point_settings;
      scanner->Scan(capture_settings, points, leaf_indices, instance_indices, type_indices);
      if (leaf_index == 0) {
        mesh_settings_copy.enable_stem = false;
        if (generate_ground) {
          const auto children = scene->GetChildren(soil->GetOwner());
          for (const auto& child : children) {
            scene->DeleteEntity(child);
          }
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

void DatasetGenerator::GenerateMeshAndPointCloudForSorghum(
    const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor, const SorghumPointCloudPointSettings& point_settings,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings, bool avoid_occlusion, bool generate_ground,
    const std::filesystem::path& mesh_output_path, const std::filesystem::path& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  if (generate_ground) {
    if (!CheckSoil(soil, generate_ground))
      return;
  }
  const auto sorghum_entity = scene->CreateEntity("Sorghum");
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_descriptor = sorghum_descriptor;
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scanner_entity).lock();
  scanner->left_random_offset = {0, 0, 0};
  scanner->right_random_offset = {0, 0, 0};
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  sorghum_layer->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();
  Application::Loop();
  scanner->sorghum_point_cloud_point_settings = point_settings;
  scanner->Capture(point_cloud_output_path, capture_settings);
  std::ofstream of;
  of.open(mesh_output_path, std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    unsigned start_index = 1;
    sorghum_layer->ExportSorghum(sorghum_entity, of, start_index);
    of.close();
  }
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
      Application::Loop();
      scanner->sorghum_point_cloud_point_settings = point_settings;
      scanner->Scan(capture_settings, points, leaf_indices, instance_indices, type_indices);
      if (leaf_index == 0) {
        mesh_settings_copy.enable_stem = false;
        if (generate_ground) {
          const auto children = scene->GetChildren(soil->GetOwner());
          for (const auto& child : children) {
            scene->DeleteEntity(child);
          }
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

void DatasetGenerator::GenerateMeshAndPointCloudForSorghum(
    const std::shared_ptr<SorghumState>& sorghum_state, const SorghumPointCloudPointSettings& point_settings,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings, bool avoid_occlusion, bool generate_ground,
    const std::filesystem::path& mesh_output_path, const std::filesystem::path& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  if (generate_ground) {
    if (!CheckSoil(soil, generate_ground))
      return;
  }

  const auto sorghum_entity = scene->CreateEntity("Sorghum");
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_state = sorghum_state;
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scanner_entity).lock();
  scanner->left_random_offset = {0, 0, 0};
  scanner->right_random_offset = {0, 0, 0};
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  sorghum_layer->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();
  Application::Loop();
  scanner->sorghum_point_cloud_point_settings = point_settings;
  scanner->Capture(point_cloud_output_path, capture_settings);

  std::ofstream of;
  of.open(mesh_output_path, std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    unsigned start_index = 1;
    sorghum_layer->ExportSorghum(sorghum_entity, of, start_index);
    of.close();
  }
  if (avoid_occlusion) {
    auto mesh_settings_copy = sorghum_mesh_generator_settings;
    mesh_settings_copy.leaf_separated = true;
    std::vector<glm::vec3> points;
    std::vector<int> leaf_indices;
    std::vector<int> instance_indices;
    std::vector<int> type_indices;
    for (int leaf_index = 0; leaf_index < sorghum_state->leaves.size(); leaf_index++) {
      mesh_settings_copy.single_leaf_index = leaf_index;
      sorghum->GenerateGeometryEntities(mesh_settings_copy);
      Application::Loop();
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

void DatasetGenerator::GenerateMeshForSorghum(const std::shared_ptr<SorghumState>& sorghum_state,
                                              const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                              const std::filesystem::path& mesh_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  const auto sorghum_entity = scene->CreateEntity("Sorghum");
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_state = sorghum_state;

  sorghum_layer->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();

  std::ofstream of;
  of.open(mesh_output_path, std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    unsigned start_index = 0;
    sorghum_layer->ExportSorghum(sorghum_entity, of, start_index);
    of.close();
  }
  scene->DeleteEntity(sorghum_entity);
  Application::Loop();
}

void DatasetGenerator::GenerateMeshForSorghum(const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                              const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                              const std::filesystem::path& mesh_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  const auto sorghum_entity = scene->CreateEntity("Sorghum");
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_descriptor = sorghum_descriptor;

  sorghum_layer->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();

  std::ofstream of;
  of.open(mesh_output_path, std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    unsigned start_index = 0;
    sorghum_layer->ExportSorghum(sorghum_entity, of, start_index);
    of.close();
  }
  scene->DeleteEntity(sorghum_entity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForSorghumPatch(
    const SorghumFieldPatch& pattern, const std::shared_ptr<SorghumGenerator>& sorghum_descriptor,
    const SorghumPointCloudPointSettings& point_settings,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
    const std::filesystem::path& point_cloud_output_path) {
  if (!CheckApplication()) {
    return;
  }
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  if (!CheckSoil(soil, true))
    return;

  const auto sorghum_field = AssetManager::CreateTemporaryAsset<SorghumField>();
  std::vector<glm::mat4> matrices_list;
  pattern.GenerateField(matrices_list);
  sorghum_field->matrices.resize(matrices_list.size());
  for (int i = 0; i < matrices_list.size(); i++) {
    sorghum_field->matrices[i] = {sorghum_descriptor, matrices_list[i]};
  }

  const auto field = sorghum_field->InstantiateField();
  Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  Application::Loop();
  Application::Loop();
  const auto scanner_entity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scanner_entity).lock();
  scanner->sorghum_point_cloud_point_settings = point_settings;
  Application::Loop();
  Application::Loop();
  scanner->Capture(point_cloud_output_path, capture_settings);
  scene->DeleteEntity(field);
  scene->DeleteEntity(scanner_entity);
  Application::Loop();
}
