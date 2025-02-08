#ifdef ECOSYSLAB_PLUGIN

#  include "AnimationPlayer.hpp"
#  include "Application.hpp"
#  include "AssetManager.hpp"
#  include "ClassRegistry.hpp"
#  include "EditorLayer.hpp"
#  include "MeshRenderer.hpp"
#  include "PlayerController.hpp"
#  include "PostProcessingStack.hpp"
#  include "Prefab.hpp"
#  include "ProjectManager.hpp"

#  include "RenderLayer.hpp"
#  include "Scene.hpp"

#  include <pybind11/stl_bind.h>
#  include "Times.hpp"
#  include "WindowLayer.hpp"
#  include "pybind11/pybind11.h"
#  include "pybind11/stl/filesystem.h"

#  ifdef CUDA_MODULE_PLUGIN
#    include <CUDAModule.hpp>
#    include <RayTracerLayer.hpp>
#  endif

#  if DATASET_GENERATION_PLUGIN
#    include <TreePointCloudScanner.hpp>
#    include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#  endif

#  include "Climate.hpp"
#  include "EcoSysLabLayer.hpp"
#  include "HeightField.hpp"
#  include "ObjectRotator.hpp"

#  include "FoliageDescriptor.hpp"
#  include "ParticlePhysics2DDemo.hpp"
#  include "Physics2DDemo.hpp"
#  include "RadialBoundingVolume.hpp"
#  include "Soil.hpp"
#  include "Tree.hpp"
#  include "TreeModel.hpp"
#  include "TreeStructor.hpp"
using namespace eco_sys_lab_plugin;

using namespace evo_engine;

namespace py = pybind11;

void register_classes() {
  PrivateComponentRegistration<ObjectRotator>("ObjectRotator");
  PrivateComponentRegistration<Physics2DDemo>("Physics2DDemo");
  PrivateComponentRegistration<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
}

void push_layers(const bool enable_window_layer, const bool enable_editor_layer) {
  Application::PushLayer<RenderLayer>("Render Layer");
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>("Window Layer");
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>("Editor Layer");
  Application::PushLayer<EcoSysLabLayer>("EcoSysLab Layer");
#  ifdef CUDA_MODULE_PLUGIN
  Application::PushLayer<RayTracerLayer>("Ray Tracer Layer");
#  endif
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
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset("./PlayGround.evescene"));
  ProjectManager::SetStartScene(new_scene);

  Application::Start();
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
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
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
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
}

void engine_loop() {
  Application::Loop();
}

void engine_terminate() {
  Application::Terminate();
}
void scene_capture(const float pos_x, const float pos_y, const float pos_z, const float angle_x, const float angle_y,
                   const float angle_z, const int resolution_x, const int resolution_y, bool white_background,
                   const std::string& output_path) {
  if (resolution_x <= 0 || resolution_y <= 0) {
    EVOENGINE_ERROR("Resolution error!");
    return;
  }

  const auto scene = Application::GetActiveScene();
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
    main_camera->camera_settings.clear_color = glm::vec3(1, 1, 1);
  }
  Application::Loop();
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
  const auto scene = Application::GetActiveScene();
  const auto ret_val = scene->CreateEntity("TreeStructor");
  const auto tree_point_cloud = scene->GetOrSetPrivateComponent<TreeStructor>(ret_val).lock();
  tree_point_cloud->ImportGraph(yaml_path);
  return ret_val;
}

void tree_structor(const std::string& yaml_path, const ConnectivityGraphSettings& connectivity_graph_settings,
                   const ReconstructionSettings& reconstruction_settings,
                   const TreeMeshGeneratorSettings& mesh_generator_settings, const std::string& mesh_path) {
  const auto scene = Application::GetActiveScene();
  const auto temp_entity = scene->CreateEntity("Temp");
  const auto tree_point_cloud = scene->GetOrSetPrivateComponent<TreeStructor>(temp_entity).lock();

  tree_point_cloud->connectivity_graph_settings = connectivity_graph_settings;
  tree_point_cloud->reconstruction_settings = reconstruction_settings;
  tree_point_cloud->ImportGraph(yaml_path);
  tree_point_cloud->EstablishConnectivityGraph();
  tree_point_cloud->BuildSkeletons();
  tree_point_cloud->ExportForestOBJ(mesh_generator_settings, mesh_path);
  EVOENGINE_LOG("Exported forest as OBJ");
  scene->DeleteEntity(temp_entity);
}
void yaml_visualization(const std::string& yaml_path, const ConnectivityGraphSettings& connectivity_graph_settings,
                        const ReconstructionSettings& reconstruction_settings,
                        const TreeMeshGeneratorSettings& mesh_generator_settings, const float pos_x, const float pos_y,
                        const float pos_z, const float angle_x, const float angle_y, const float angle_z,
                        const int resolution_x, const int resolution_y, const std::string& output_path) {
  const auto scene = Application::GetActiveScene();
  const auto temp_entity = scene->CreateEntity("Temp");
  const auto tree_point_cloud = scene->GetOrSetPrivateComponent<TreeStructor>(temp_entity).lock();
  tree_point_cloud->connectivity_graph_settings = connectivity_graph_settings;
  tree_point_cloud->reconstruction_settings = reconstruction_settings;
  tree_point_cloud->ImportGraph(yaml_path);
  tree_point_cloud->EstablishConnectivityGraph();
  tree_point_cloud->BuildSkeletons();
  tree_point_cloud->GenerateForest();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
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
  const auto application_status = Application::GetApplicationStatus();
  if (!Application::GetActiveScene()) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (application_status == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (application_status == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
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
  auto& occupancy_grid = tree->tree_model.tree_occupancy_grid;
  VoxelGrid<TreeOccupancyGridBasicData> input_grid{};
  if (tree->ParseBinvox(binvox_path, input_grid, 1.f)) {
    occupancy_grid.Initialize(input_grid, glm::vec3(-radius, 0, -radius), glm::vec3(radius, 2.0f * radius, radius),
                              tree_descriptor->shoot_descriptor.Get<ShootDescriptor>()->internode_length,
                              tree->tree_model.tree_growth_settings.space_colonization_removal_distance_factor,
                              tree->tree_model.tree_growth_settings.space_colonization_theta,
                              tree->tree_model.tree_growth_settings.space_colonization_detection_distance_factor);
  }
  tree->tree_model.tree_growth_settings.use_space_colonization = true;
  tree->tree_model.tree_growth_settings.space_colonization_auto_resize = false;

  eco_sys_lab_layer->simulation_settings.delta_time = delta_time;

  Application::Loop();
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
  const auto application_status = Application::GetApplicationStatus();
  if (!Application::GetActiveScene()) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (application_status == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (application_status == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
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
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInAssetsFolder(tree_parameters_path)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(tree_parameters_path)));
  } else {
    treeDescriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
  }
  tree->tree_descriptor_ref = treeDescriptor;
  auto& occupancyGrid = tree->tree_model.tree_occupancy_grid;
  const auto rbv = AssetManager::CreateTemporaryAsset<RadialBoundingVolume>();
  rbv->Import(rbv_path);

  occupancyGrid.Initialize(rbv, glm::vec3(-rbv->m_maxRadius, 0, -rbv->m_maxRadius),
                           glm::vec3(rbv->m_maxRadius, 2.0f * rbv->m_maxRadius, rbv->m_maxRadius),
                           treeDescriptor->shoot_descriptor.Get<ShootDescriptor>()->internode_length,
                           tree->tree_model.tree_growth_settings.space_colonization_removal_distance_factor,
                           tree->tree_model.tree_growth_settings.space_colonization_theta,
                           tree->tree_model.tree_growth_settings.space_colonization_detection_distance_factor);

  tree->tree_model.tree_growth_settings.use_space_colonization = true;
  tree->tree_model.tree_growth_settings.space_colonization_auto_resize = false;
  Application::Loop();
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
  const auto scene = Application::GetActiveScene();
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

PYBIND11_MAKE_OPAQUE(std::vector<int>)

PYBIND11_MODULE(PyEcoSysLab, m) {
  m.def("get_default_project_path", &get_default_project_path, "Get default project path");
  m.def("engine_run_windowless", &engine_run_windowless, "Start Project (Windowless)");
  m.def("engine_run", &engine_run, "Start Project (No Editor)");
  m.def("engine_run_with_editor", &engine_run_with_editor, "Start Project (with Editor)");
  m.def("engine_loop", &engine_loop, "Loop Application");
  m.def("engine_terminate", &engine_terminate, "Terminate Application");

  py::class_<ConnectivityGraphSettings>(m, "ConnectivityGraphSettings")
      .def(py::init<>())
      .def_readwrite("point_existence_check_radius", &ConnectivityGraphSettings::point_existence_check_radius)
      .def_readwrite("point_point_connection_detection_radius",
                     &ConnectivityGraphSettings::point_point_connection_detection_radius)
      .def_readwrite("point_branch_connection_detection_radius",
                     &ConnectivityGraphSettings::point_branch_connection_detection_radius)
      .def_readwrite("branch_branch_connection_max_length_range",
                     &ConnectivityGraphSettings::branch_branch_connection_max_length_range)
      .def_readwrite("direction_connection_angle_limit", &ConnectivityGraphSettings::direction_connection_angle_limit)
      .def_readwrite("indirect_connection_angle_limit", &ConnectivityGraphSettings::indirect_connection_angle_limit);

#  ifdef DATASET_GENERATION_PLUGIN
  py::class_<TreePointCloudPointSettings>(m, "TreePointCloudPointSettings")
      .def(py::init<>())
      .def_readwrite("variance", &TreePointCloudPointSettings::variance)
      .def_readwrite("ball_rand_radius", &TreePointCloudPointSettings::ball_rand_radius)
      .def_readwrite("type_index", &TreePointCloudPointSettings::type_index)
      .def_readwrite("instance_index", &TreePointCloudPointSettings::instance_index)
      .def_readwrite("tree_part_index", &TreePointCloudPointSettings::tree_part_index)
      .def_readwrite("tree_part_type_index", &TreePointCloudPointSettings::tree_part_type_index)
      .def_readwrite("line_index", &TreePointCloudPointSettings::line_index)
      .def_readwrite("branch_index", &TreePointCloudPointSettings::branch_index)
      .def_readwrite("internode_index", &TreePointCloudPointSettings::internode_index)
      .def_readwrite("bounding_box_limit", &TreePointCloudPointSettings::bounding_box_limit);

  py::class_<TreePointCloudCircularCaptureSettings>(m, "TreePointCloudCircularCaptureSettings")
      .def(py::init<>())
      .def_readwrite("pitch_angle_start", &TreePointCloudCircularCaptureSettings::pitch_angle_start)
      .def_readwrite("pitch_angle_step", &TreePointCloudCircularCaptureSettings::pitch_angle_step)
      .def_readwrite("pitch_angle_end", &TreePointCloudCircularCaptureSettings::pitch_angle_end)
      .def_readwrite("turn_angle_start", &TreePointCloudCircularCaptureSettings::turn_angle_start)
      .def_readwrite("turn_angle_step", &TreePointCloudCircularCaptureSettings::turn_angle_step)
      .def_readwrite("turn_angle_end", &TreePointCloudCircularCaptureSettings::turn_angle_end)
      .def_readwrite("distance_from_trees", &TreePointCloudCircularCaptureSettings::distance_from_trees)
      .def_readwrite("capture_height", &TreePointCloudCircularCaptureSettings::capture_height)
      .def_readwrite("camera_fov", &TreePointCloudCircularCaptureSettings::camera_fov)
      .def_readwrite("scan_resolution", &TreePointCloudCircularCaptureSettings::scan_resolution)
      .def_readwrite("max_capture_depth", &TreePointCloudCircularCaptureSettings::max_capture_depth);
#  endif

  py::class_<ReconstructionSettings>(m, "ReconstructionSettings")
      .def(py::init<>())
      .def_readwrite("internode_length", &ReconstructionSettings::internode_length)
      .def_readwrite("min_height", &ReconstructionSettings::min_height)
      .def_readwrite("minimum_tree_distance", &ReconstructionSettings::minimum_tree_distance)
      .def_readwrite("branch_shortening", &ReconstructionSettings::branch_shortening)
      .def_readwrite("end_node_thickness", &ReconstructionSettings::end_node_thickness)
      .def_readwrite("minimum_node_count", &ReconstructionSettings::minimum_node_count);

  py::class_<PresentationOverrideSettings>(m, "PresentationOverrideSettings")
      .def(py::init<>())
      .def_readwrite("max_thickness", &PresentationOverrideSettings::max_thickness);

  py::class_<TreeMeshGeneratorSettings>(m, "TreeMeshGeneratorSettings")
      .def(py::init<>())
      .def_readwrite("enable_foliage", &TreeMeshGeneratorSettings::enable_foliage)
      .def_readwrite("enable_fruit", &TreeMeshGeneratorSettings::enable_fruit)
      .def_readwrite("enable_branch", &TreeMeshGeneratorSettings::enable_branch)
      .def_readwrite("presentation_override_settings", &TreeMeshGeneratorSettings::presentation_override_settings)
      .def_readwrite("x_subdivision", &TreeMeshGeneratorSettings::x_subdivision)
      .def_readwrite("trunk_y_subdivision", &TreeMeshGeneratorSettings::trunk_y_subdivision)
      .def_readwrite("trunk_thickness", &TreeMeshGeneratorSettings::trunk_thickness)
      .def_readwrite("branch_y_subdivision", &TreeMeshGeneratorSettings::branch_y_subdivision)
      .def_readwrite("vertex_color_mode", &TreeMeshGeneratorSettings::vertex_color_mode)
      .def_readwrite("override_radius", &TreeMeshGeneratorSettings::override_radius)
      .def_readwrite("radius", &TreeMeshGeneratorSettings::radius)
      .def_readwrite("tree_part_base_distance", &TreeMeshGeneratorSettings::tree_part_base_distance)
      .def_readwrite("tree_part_end_distance", &TreeMeshGeneratorSettings::tree_part_end_distance)
      .def_readwrite("base_control_point_ratio", &TreeMeshGeneratorSettings::base_control_point_ratio)
      .def_readwrite("branch_control_point_ratio", &TreeMeshGeneratorSettings::branch_control_point_ratio)
      .def_readwrite("smoothness", &TreeMeshGeneratorSettings::smoothness)
      .def_readwrite("auto_level", &TreeMeshGeneratorSettings::auto_level)
      .def_readwrite("voxel_subdivision_level", &TreeMeshGeneratorSettings::voxel_subdivision_level)
      .def_readwrite("voxel_smooth_iteration", &TreeMeshGeneratorSettings::voxel_smooth_iteration)
      .def_readwrite("remove_duplicate", &TreeMeshGeneratorSettings::remove_duplicate)
      .def_readwrite("branch_mesh_type", &TreeMeshGeneratorSettings::branch_mesh_type);

  py::class_<SimulationSettings>(m, "SimulationSettings")
      .def(py::init<>())
      .def_readwrite("delta_time", &SimulationSettings::delta_time)
      .def_readwrite("soil_simulation", &SimulationSettings::soil_simulation)
      .def_readwrite("auto_clear_fruit_and_leaves", &SimulationSettings::auto_clear_fruit_and_leaves)
      .def_readwrite("crown_shyness_distance", &SimulationSettings::crown_shyness_distance)
      .def_readwrite("max_node_count", &SimulationSettings::max_node_count)
      .def_readwrite("max_flow_count", &SimulationSettings::max_flow_count)
      .def_readwrite("skylight_intensity", &SimulationSettings::skylight_intensity)
      .def_readwrite("shadow_distance_loss", &SimulationSettings::shadow_distance_loss)
      .def_readwrite("detection_radius", &SimulationSettings::detection_radius)
      .def_readwrite("environment_light_intensity", &SimulationSettings::environment_light_intensity)
      .def_readwrite("blur_iteration", &SimulationSettings::blur_iteration)
      .def_readwrite("auto_generate_skeletal_graph", &SimulationSettings::auto_generate_skeletal_graph);

  py::class_<TreeGrowthSettings>(m, "TreeGrowthSettings")
      .def(py::init<>())
      .def_readwrite("node_developmental_vigor_filling_rate",
                     &TreeGrowthSettings::node_developmental_vigor_filling_rate)
      .def_readwrite("use_space_colonization", &TreeGrowthSettings::use_space_colonization)
      .def_readwrite("space_colonization_auto_resize", &TreeGrowthSettings::space_colonization_auto_resize)
      .def_readwrite("space_colonization_removal_distance_factor",
                     &TreeGrowthSettings::space_colonization_removal_distance_factor)
      .def_readwrite("space_colonization_detection_distance_factor",
                     &TreeGrowthSettings::space_colonization_detection_distance_factor)
      .def_readwrite("space_colonization_theta", &TreeGrowthSettings::space_colonization_theta);

  py::class_<Tree::PruningSettings>(m, "PruningSettings")
      .def(py::init<>())
      .def_readwrite("low_branch_pruning", &Tree::PruningSettings::low_branch_pruning);

  py::class_<glm::vec3>(m, "Vec3")
      .def(py::init<>())
      .def_readwrite("x", &glm::vec3::x)
      .def_readwrite("y", &glm::vec3::y)
      .def_readwrite("z", &glm::vec3::z);

  py::class_<glm::uvec2>(m, "UVec2")
      .def(py::init<>())
      .def_readwrite("x", &glm::uvec2::x)
      .def_readwrite("y", &glm::uvec2::y);

  py::class_<CameraSettings>(m, "CameraSettings")
      .def(py::init<>())
      .def_readwrite("near_distance", &CameraSettings::near_distance)
      .def_readwrite("far_distance", &CameraSettings::far_distance)
      .def_readwrite("fade_ratio", &CameraSettings::fade_ratio)
      .def_readwrite("fade_factor", &CameraSettings::fade_factor)
      .def_readwrite("fov", &CameraSettings::fov)
      .def_readwrite("use_clear_color", &CameraSettings::use_clear_color)
      .def_readwrite("clear_color", &CameraSettings::clear_color)
      .def_readwrite("background_intensity", &CameraSettings::background_intensity);

  py::class_<DatasetGenerator::CameraCaptureSettings>(m, "CameraCaptureSettings")
      .def(py::init<>())
      .def_readwrite("position", &DatasetGenerator::CameraCaptureSettings::position)
      .def_readwrite("euler_rotation", &DatasetGenerator::CameraCaptureSettings::euler_rotation)
      .def_readwrite("camera_settings", &DatasetGenerator::CameraCaptureSettings::camera_settings)
      .def_readwrite("render_resolution", &DatasetGenerator::CameraCaptureSettings::render_resolution)
      .def_readwrite("output_resolution", &DatasetGenerator::CameraCaptureSettings::output_resolution);

  py::class_<DatasetGenerator::TreeDataGenerationParameters>(m, "TreeDataGenerationParameters")
      .def(py::init<>())
      .def_readwrite("tree_descriptor_path", &DatasetGenerator::TreeDataGenerationParameters::tree_descriptor_path)
      .def_readwrite("foliage_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::foliage_descriptor_path)
      .def_readwrite("bark_descriptor_path", &DatasetGenerator::TreeDataGenerationParameters::bark_descriptor_path)

      .def_readwrite("simulation_settings", &DatasetGenerator::TreeDataGenerationParameters::simulation_settings)
      .def_readwrite("tree_growth_settings", &DatasetGenerator::TreeDataGenerationParameters::tree_growth_settings)
      .def_readwrite("pruning_settings", &DatasetGenerator::TreeDataGenerationParameters::pruning_settings)

      .def_readwrite("max_iteration", &DatasetGenerator::TreeDataGenerationParameters::max_iteration)
      .def_readwrite("use_node_growth_capture",
                     &DatasetGenerator::TreeDataGenerationParameters::use_node_growth_capture)
      .def_readwrite("growth_capture", &DatasetGenerator::TreeDataGenerationParameters::growth_capture)

      .def_readwrite("export_point_cloud", &DatasetGenerator::TreeDataGenerationParameters::export_point_cloud)
      .def_readwrite("export_mesh", &DatasetGenerator::TreeDataGenerationParameters::export_mesh)
      .def_readwrite("export_skeleton", &DatasetGenerator::TreeDataGenerationParameters::export_skeleton)
      .def_readwrite("export_rendering", &DatasetGenerator::TreeDataGenerationParameters::export_rendering)
      .def_readwrite("export_depth", &DatasetGenerator::TreeDataGenerationParameters::export_depth)

      .def_readwrite("generate_ground_mesh", &DatasetGenerator::TreeDataGenerationParameters::generate_ground_mesh)
      .def_readwrite("tree_point_cloud_point_settings",
                     &DatasetGenerator::TreeDataGenerationParameters::tree_point_cloud_point_settings)
      .def_readwrite("tree_mesh_generator_settings",
                     &DatasetGenerator::TreeDataGenerationParameters::tree_mesh_generator_settings)

      .def_readwrite("camera_capture_settings",
                     &DatasetGenerator::TreeDataGenerationParameters::camera_capture_settings)

      .def_readwrite("seed", &DatasetGenerator::TreeDataGenerationParameters::seed)
      .def_readwrite("max_depth", &DatasetGenerator::TreeDataGenerationParameters::max_depth)
      .def_readwrite("output_folder", &DatasetGenerator::TreeDataGenerationParameters::output_folder)
      .def_readwrite("output_file_name", &DatasetGenerator::TreeDataGenerationParameters::output_file_name);

  m.doc() = "PyEcoSysLab";  // optional module docstring

  m.def("tree_structor", &tree_structor, "Reconstruct tree(s) and export meshes");
  m.def("scene_capture", &scene_capture, "Capture current scene");
  m.def("yaml_visualization", &yaml_visualization, "Reconstruct tree(s) and capture an image for visualization");
  m.def("voxel_space_colonization_tree_data", &voxel_space_colonization_tree_data,
        "Grow a tree in voxel space and export data");
  m.def("rbv_space_colonization_tree_data", &rbv_space_colonization_tree_data, "Grow a tree in RBV and export data");
  m.def("rbv_to_obj", &rbv_to_obj, "Convert RBV to 3D model (OBJ)");

  m.def("generate_tree_data", &generate_tree_data, "Generate data for single tree");
  m.def("scene_light_settings", &scene_light_settings, "Configure scene lighting");
}

#endif