#include "ImGuiLayer.hpp"
#include "PyEcoSysLab.hpp"
#include "PyEvoEngine.hpp"
#include "Serialization.hpp"

#if DATASET_GENERATION_PACKAGE
#  include "DatasetGenerationSerializationAdapters.hpp"
#endif

#ifdef ECOSYSLAB_PACKAGE
namespace py = pybind11;
using namespace py_eco_sys_lab_package;
namespace {
template <typename T>
void RegisterSerializationHandler(const std::string& type_name) {
  Serialization::RegisterSerializationHandler<T>(
      [](YAML::Emitter& out, const T& target) {
        target.Serialize(out);
      },
      [](const YAML::Node& in, T& target) {
        target.Deserialize(in);
      },
      {}, type_name);
}
}  // namespace
void register_classes() {
#  ifdef ECOSYSLAB_PACKAGE
  auto& application = PyEvoEngine::GetRuntime().GetApplication();
  application.RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
  application.RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
  application.RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
  application.RegisterPrivateComponent<TreePointCloudScanner>("TreePointCloudScanner");
  RegisterSerializationHandler<ObjectRotator>("ObjectRotator");
  Serialization::RegisterSerializationHandler<TreePointCloudScanner>(
      SerializeTreePointCloudScanner, DeserializeTreePointCloudScanner, {}, "TreePointCloudScanner");
#  endif
}

void push_layers(const bool enable_window_layer, const bool enable_editor_layer) {
  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  if (enable_window_layer)
    ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  if (enable_window_layer && enable_editor_layer) {
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
    ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");
  }
  ApplicationContext::Get().PushLayer<EcoSysLabLayer>("EcoSysLab Layer");
}

std::filesystem::path get_default_project_path() {
  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  resource_folder_path = std::filesystem::absolute(resource_folder_path);

  return resource_folder_path / "EcoSysLabProject" / "test.eveproj";
}

void engine_run_windowless(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a valid project!");
    return;
  }
  register_classes();
  push_layers(false, false);
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  ApplicationContext::Get().Initialize(application_info);
  const auto new_scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset("./PlayGround.evescene"));
  ProjectManager::SetStartScene(new_scene);

  ApplicationContext::Get().Start();
}

void engine_run(const std::filesystem::path& project_path) {
  if (!project_path.empty()) {
    if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project path doesn't point to a valid project!");
      return;
    }
  }
  register_classes();
  push_layers(true, false);
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  ApplicationContext::Get().Initialize(application_info);
  ApplicationContext::Get().Start();
}

void engine_run_with_editor(const std::filesystem::path& project_path) {
  if (!project_path.empty()) {
    if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project path doesn't point to a valid project!");
      return;
    }
  }
  register_classes();
  push_layers(true, true);
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  ApplicationContext::Get().Initialize(application_info);
  ApplicationContext::Get().Start();
}

void engine_loop() {
  ApplicationContext::Get().Loop();
}

void engine_terminate() {
  ApplicationContext::Get().Terminate();
}
void scene_capture(const float pos_x, const float pos_y, const float pos_z, const float angle_x, const float angle_y,
                   const float angle_z, const int resolution_x, const int resolution_y, bool white_background,
                   const std::string& output_path) {
  if (resolution_x <= 0 || resolution_y <= 0) {
    EVOENGINE_ERROR("Resolution error!");
    return;
  }

  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("No active scene!");
    return;
  }
  auto main_camera = scene->main_camera.Get<Camera>();
  Entity main_camera_entity;
  bool temp_camera = false;
  if (!main_camera) {
    main_camera_entity = scene->CreateEntity("Main Camera");
    main_camera = scene->GetOrSetPrivateComponent<Camera>(main_camera_entity).lock();
    scene->main_camera = main_camera;
    temp_camera = true;
  } else {
    main_camera_entity = main_camera->GetOwner();
  }
  auto global_transform = scene->GetDataComponent<GlobalTransform>(main_camera_entity);
  const auto original_transform = global_transform;
  global_transform.SetPosition({pos_x, pos_y, pos_z});
  global_transform.SetEulerRotation(glm::radians(glm::vec3(angle_x, angle_y, angle_z)));
  scene->SetDataComponent(main_camera_entity, global_transform);
  main_camera->Resize({resolution_x, resolution_y});
  const auto use_clear_color = main_camera->camera_settings.use_clear_color;
  const auto clear_color = main_camera->camera_settings.clear_color;
  if (white_background) {
    main_camera->camera_settings.use_clear_color = true;
    main_camera->camera_settings.clear_color = glm::vec4(1, 1, 1, 1);
  }
  ApplicationContext::Get().Loop();
  main_camera->GetRenderTexture()->StoreToPng(output_path);
  if (temp_camera) {
    scene->DeleteEntity(main_camera_entity);
  } else {
    scene->SetDataComponent(main_camera_entity, original_transform);
    if (white_background) {
      main_camera->camera_settings.use_clear_color = use_clear_color;
      main_camera->camera_settings.clear_color = clear_color;
    }
  }

  EVOENGINE_LOG("Exported image to " + output_path);
}

Entity import_tree_point_cloud(const std::string& yaml_path) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto ret_val = scene->CreateEntity("TreeStructor");
  const auto tree_point_cloud = scene->GetOrSetPrivateComponent<TreeStructor>(ret_val).lock();
  tree_point_cloud->ImportGraph(yaml_path);
  return ret_val;
}

void tree_structor(const std::filesystem::path& yaml_path, const float import_scale_factor,
                   const ConnectivityGraphSettings& connectivity_graph_settings,
                   const ReconstructionSettings& reconstruction_settings,
                   const DatasetGenerator::TreeDataGenerationParameters& tree_data_generation_parameters) {
  if (!std::filesystem::exists(yaml_path)) {
    EVOENGINE_ERROR("Incorrect yaml path!")
    return;
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto temp_entity = scene->CreateEntity("Temp");
  const auto tree_structor = scene->GetOrSetPrivateComponent<TreeStructor>(temp_entity).lock();
  if (!tree_data_generation_parameters.tree_descriptor_path.empty()) {
    const auto actual_tree_descriptor = tree_data_generation_parameters.GetActualTreeDescriptor();
    tree_structor->tree_descriptor_ref = actual_tree_descriptor;
  }
  tree_structor->connectivity_graph_settings = connectivity_graph_settings;
  tree_structor->reconstruction_settings = reconstruction_settings;
  tree_structor->ImportGraph(yaml_path, import_scale_factor);
  tree_structor->EstablishConnectivityGraph();
  tree_structor->BuildSkeletons();

  if (tree_data_generation_parameters.export_mesh) {
    tree_structor->ExportForestObj(
        tree_data_generation_parameters.tree_mesh_generator_settings,
        tree_data_generation_parameters.output_folder / (tree_data_generation_parameters.output_file_name + ".obj"));
  }
  if (tree_data_generation_parameters.export_statistics) {
    tree_structor->ExportForestStatistics(tree_data_generation_parameters.output_folder /
                                          (tree_data_generation_parameters.output_file_name + ".yml"));
  }
  if (tree_data_generation_parameters.export_flow_graph) {
    tree_structor->ExportFlowGraphs(tree_data_generation_parameters.output_folder /
                                    (tree_data_generation_parameters.output_file_name + "_flows.yml"));
  }
  if (tree_data_generation_parameters.export_node_graph) {
    tree_structor->ExportNodeGraphs(tree_data_generation_parameters.output_folder /
                                    (tree_data_generation_parameters.output_file_name + "_nodes.yml"));
  }
  scene->DeleteEntity(temp_entity);
}

void yaml_visualization(const std::string& yaml_path, const ConnectivityGraphSettings& connectivity_graph_settings,
                        const ReconstructionSettings& reconstruction_settings,
                        const TreeMeshGeneratorSettings& mesh_generator_settings, const float pos_x, const float pos_y,
                        const float pos_z, const float angle_x, const float angle_y, const float angle_z,
                        const int resolution_x, const int resolution_y, const std::string& output_path) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto temp_entity = scene->CreateEntity("Temp");
  const auto tree_point_cloud = scene->GetOrSetPrivateComponent<TreeStructor>(temp_entity).lock();
  tree_point_cloud->connectivity_graph_settings = connectivity_graph_settings;
  tree_point_cloud->reconstruction_settings = reconstruction_settings;
  tree_point_cloud->ImportGraph(yaml_path);
  tree_point_cloud->EstablishConnectivityGraph();
  tree_point_cloud->BuildSkeletons();
  tree_point_cloud->GenerateForest();
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  eco_sys_lab_layer->GenerateMeshes(mesh_generator_settings);
  scene_capture(pos_x, pos_y, pos_z, angle_x, angle_y, angle_z, resolution_x, resolution_y, true, output_path);
  scene->DeleteEntity(temp_entity);
}

void voxel_space_colonization_tree_data(
    const float radius, const std::string& binvox_path, const std::string& tree_parameters_path, const float delta_time,
    const int iterations, const TreeMeshGeneratorSettings& mesh_generator_settings, bool export_tree_mesh,
    const std::string& tree_mesh_output_path, bool export_tree_io, const std::string& tree_io_output_path,
    bool export_radial_bounding_volume, const std::string& radial_bounding_volume_output_path,
    bool export_radial_bounding_volume_mesh, const std::string& radial_bounding_volume_mesh_output_path) {
  const auto application_status = ApplicationContext::Get().GetApplicationStatus();
  if (!ApplicationContext::Get().GetActiveScene()) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (application_status == Application::ExecutionStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (application_status == Application::ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<Soil> soil;
  std::shared_ptr<Climate> climate;

  const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soil_entities && !soil_entities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }
  const std::vector<Entity>* climate_entities = scene->UnsafeGetPrivateComponentOwnersList<Climate>();
  if (climate_entities && !climate_entities->empty()) {
    climate = scene->GetOrSetPrivateComponent<Climate>(climate_entities->at(0)).lock();
  }
  if (!climate) {
    EVOENGINE_ERROR("No climate in scene!");
    return;
  }

  const auto temp_entity = scene->CreateEntity("Temp");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(temp_entity).lock();
  tree->soil = soil;
  tree->climate = climate;
  std::shared_ptr<TreeDescriptor> tree_descriptor;
  if (ProjectManager::IsInAssetsFolder(tree_parameters_path)) {
    tree_descriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(tree_parameters_path)));
  } else {
    tree_descriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
  }
  tree->tree_descriptor_ref = tree_descriptor;
  auto& occupancy_grid = tree->shoot_model.tree_occupancy_grid;
  VoxelGrid<TreeOccupancyGridBasicData> input_grid{};
  if (tree->ParseBinvox(binvox_path, input_grid, 1.f)) {
    occupancy_grid.Initialize(input_grid, glm::vec3(-radius, 0, -radius), glm::vec3(radius, 2.0f * radius, radius),
                              tree_descriptor->shoot_descriptor.Get<BasicShootDescriptor>()->internode_length,
                              tree->shoot_model.tree_growth_settings.space_colonization_removal_distance_factor,
                              tree->shoot_model.tree_growth_settings.space_colonization_theta,
                              tree->shoot_model.tree_growth_settings.space_colonization_detection_distance_factor);
  }
  tree->shoot_model.tree_growth_settings.use_space_colonization = true;
  tree->shoot_model.tree_growth_settings.space_colonization_auto_resize = false;

  eco_sys_lab_layer->simulation_settings.delta_time = delta_time;

  ApplicationContext::Get().Loop();
  for (int i = 0; i < iterations; i++) {
    eco_sys_lab_layer->Simulate();
  }

  if (export_tree_mesh) {
    tree->ExportObj(tree_mesh_output_path, mesh_generator_settings);
  }
  if (export_tree_io) {
    bool succeed = tree->ExportIoTree(tree_io_output_path);
  }
  if (export_radial_bounding_volume || export_radial_bounding_volume_mesh) {
    const auto rbv = AssetManager::CreateTemporaryAsset<RadialBoundingVolume>();
    tree->ExportRadialBoundingVolume(rbv);
    if (export_radial_bounding_volume) {
      if (!rbv->Export(radial_bounding_volume_output_path)) {
        EVOENGINE_ERROR("Error exporting file!")
      }
    }
    if (export_radial_bounding_volume_mesh) {
      rbv->ExportAsObj(radial_bounding_volume_mesh_output_path);
    }
  }
  scene->DeleteEntity(temp_entity);
}

void rbv_to_obj(const std::string& rbv_path, const std::string& radial_bounding_volume_mesh_output_path) {
  const auto rbv = AssetManager::CreateTemporaryAsset<RadialBoundingVolume>();
  rbv->Import(rbv_path);
  rbv->ExportAsObj(radial_bounding_volume_mesh_output_path);
}

void rbv_space_colonization_tree_data(const std::string& rbv_path, const std::string& tree_parameters_path,
                                      const float delta_time, const int iterations,
                                      const TreeMeshGeneratorSettings& mesh_generator_settings, bool export_tree_mesh,
                                      const std::string& tree_mesh_output_path, bool export_tree_io,
                                      const std::string& tree_io_output_path, bool export_radial_bounding_volume_mesh,
                                      const std::string& radial_bounding_volume_mesh_output_path) {
  const auto application_status = ApplicationContext::Get().GetApplicationStatus();
  if (!ApplicationContext::Get().GetActiveScene()) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (application_status == Application::ExecutionStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (application_status == Application::ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<Soil> soil;
  std::shared_ptr<Climate> climate;

  if (const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
      soil_entities && !soil_entities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }
  if (const std::vector<Entity>* climate_entities = scene->UnsafeGetPrivateComponentOwnersList<Climate>();
      climate_entities && !climate_entities->empty()) {
    climate = scene->GetOrSetPrivateComponent<Climate>(climate_entities->at(0)).lock();
  }
  if (!climate) {
    EVOENGINE_ERROR("No climate in scene!");
    return;
  }

  const auto temp_entity = scene->CreateEntity("Temp");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(temp_entity).lock();
  tree->soil = soil;
  tree->climate = climate;
  std::shared_ptr<TreeDescriptor> tree_descriptor;
  if (ProjectManager::IsInAssetsFolder(tree_parameters_path)) {
    tree_descriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(tree_parameters_path)));
  } else {
    tree_descriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
  }
  tree->tree_descriptor_ref = tree_descriptor;
  auto& occupancy_grid = tree->shoot_model.tree_occupancy_grid;
  const auto rbv = AssetManager::CreateTemporaryAsset<RadialBoundingVolume>();
  rbv->Import(rbv_path);

  occupancy_grid.Initialize(rbv, glm::vec3(-rbv->m_maxRadius, 0, -rbv->m_maxRadius),
                            glm::vec3(rbv->m_maxRadius, 2.0f * rbv->m_maxRadius, rbv->m_maxRadius),
                            tree_descriptor->shoot_descriptor.Get<BasicShootDescriptor>()->internode_length,
                            tree->shoot_model.tree_growth_settings.space_colonization_removal_distance_factor,
                            tree->shoot_model.tree_growth_settings.space_colonization_theta,
                            tree->shoot_model.tree_growth_settings.space_colonization_detection_distance_factor);

  tree->shoot_model.tree_growth_settings.use_space_colonization = true;
  tree->shoot_model.tree_growth_settings.space_colonization_auto_resize = false;
  ApplicationContext::Get().Loop();
  eco_sys_lab_layer->simulation_settings.delta_time = delta_time;
  for (int i = 0; i < iterations; i++) {
    eco_sys_lab_layer->Simulate();
  }

  if (export_tree_mesh) {
    tree->ExportObj(tree_mesh_output_path, mesh_generator_settings);
  }
  if (export_tree_io) {
    bool succeed = tree->ExportIoTree(tree_io_output_path);
  }
  if (export_radial_bounding_volume_mesh) {
    rbv->ExportAsObj(radial_bounding_volume_mesh_output_path);
  }
  scene->DeleteEntity(temp_entity);
}

void scene_light_settings(const float ambient_light_intensity, const float directional_light_intensity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  scene->environment.ambient_light_intensity = ambient_light_intensity;
  const auto directional_light_entities = scene->GetPrivateComponentOwnersList<DirectionalLight>();
  for (const auto& directional_light_entity : directional_light_entities) {
    const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(directional_light_entity).lock();
    directional_light->diffuse_brightness = directional_light_intensity;
  }
}

void generate_tree_data(const TreePointCloudCircularCaptureSettings& capture_settings,
                        const DatasetGenerator::CameraCaptureSettings& camera_capture_settings,
                        DatasetGenerator::TreeDataGenerationParameters data_generation_parameters) {
  const auto tree_capture_settings = std::make_shared<TreePointCloudCircularCaptureSettings>();
  *tree_capture_settings = capture_settings;
  data_generation_parameters.camera_capture_settings.resize(1);
  data_generation_parameters.camera_capture_settings[0] = camera_capture_settings;
  data_generation_parameters.point_cloud_capture_settings = tree_capture_settings;
  DatasetGenerator::GenerateDataForTree(data_generation_parameters);
}

void generate_tree_growth_data(const DatasetGenerator::CameraCaptureSettings& camera_capture_settings,
                               DatasetGenerator::TreeDataGenerationParameters data_generation_parameters) {
  data_generation_parameters.camera_capture_settings.resize(1);
  data_generation_parameters.camera_capture_settings[0] = camera_capture_settings;
  DatasetGenerator::GenerateTreeGrowthData(data_generation_parameters);
}

PYBIND11_MAKE_OPAQUE(std::vector<int>)

PYBIND11_MODULE(PyEcoSysLab, m) {
  m.doc() = "PyEcoSysLab";  // optional module docstring
  PyEcoSysLab::Initialize(m);
  m.def("tree_structor", &tree_structor, "Reconstruct tree(s) and export meshes");
  m.def("scene_capture", &scene_capture, "Capture current scene");
  m.def("yaml_visualization", &yaml_visualization, "Reconstruct tree(s) and capture an image for visualization");
  m.def("voxel_space_colonization_tree_data", &voxel_space_colonization_tree_data,
        "Grow a tree in voxel space and export data");
  m.def("rbv_space_colonization_tree_data", &rbv_space_colonization_tree_data, "Grow a tree in RBV and export data");
  m.def("rbv_to_obj", &rbv_to_obj, "Convert RBV to 3D model (OBJ)");

  m.def("generate_tree_data", &generate_tree_data, "Generate data for single tree");
  m.def("generate_tree_growth_data", &generate_tree_growth_data, "Generate data for single tree growth");
  m.def("scene_light_settings", &scene_light_settings, "Configure scene lighting");
}
#endif
