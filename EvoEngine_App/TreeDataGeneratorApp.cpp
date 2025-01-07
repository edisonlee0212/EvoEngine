#include "Application.hpp"
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"
#include "Lights.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#ifdef DIGITAL_AGRICULTURE_PLUGIN

#  include "SorghumLayer.hpp"
using namespace digital_agriculture_plugin;
#endif
#include "WindowLayer.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif

#ifdef DATASET_GENERATION_PLUGIN
#  include <SorghumPointCloudScanner.hpp>
#  include <TreePointCloudScanner.hpp>
#  include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#endif
#ifdef ECOSYSLAB_PLUGIN
#  include "EcoSysLabLayer.hpp"
#  include "ObjectRotator.hpp"
#  include "ParticlePhysics2DDemo.hpp"
#  include "Physics2DDemo.hpp"
#  include "Soil.hpp"
#  include "Tree.hpp"
#  include "TreeStructor.hpp"
using namespace eco_sys_lab_plugin;
#endif

using namespace evo_engine;

void register_classes() {
#ifdef ECOSYSLAB_PLUGIN
  PrivateComponentRegistration<ObjectRotator>("ObjectRotator");
  PrivateComponentRegistration<Physics2DDemo>("Physics2DDemo");
  PrivateComponentRegistration<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
  PrivateComponentRegistration<TreePointCloudScanner>("TreePointCloudScanner");
#endif
}

void push_layers(bool enable_window_layer, bool enable_editor_layer) {
  Application::PushLayer<RenderLayer>("Render Layer");
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>("Window Layer");
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>("Editor Layer");

#ifdef ECOSYSLAB_PLUGIN
  Application::PushLayer<EcoSysLabLayer>("EcoSysLab Layer");
#endif
#ifdef CUDA_MODULE_PLUGIN
  Application::PushLayer<RayTracerLayer>("Ray Tracer Layer");
#endif
}

void run_windowless(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
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

void generate_tree_data() {
  constexpr bool export_junction = false;
  DatasetGenerator::TreeDataGenerationParameters data_generation_parameters{};
  data_generation_parameters.tree_point_cloud_point_settings.ball_rand_radius = 0.0f;
  data_generation_parameters.tree_point_cloud_point_settings.tree_part_index = export_junction;
  data_generation_parameters.tree_point_cloud_point_settings.instance_index = true;
  data_generation_parameters.tree_point_cloud_point_settings.type_index = true;
  data_generation_parameters.tree_point_cloud_point_settings.tree_part_type_index = export_junction;
  data_generation_parameters.tree_point_cloud_point_settings.branch_index = false;
  data_generation_parameters.tree_point_cloud_point_settings.line_index = export_junction;
  data_generation_parameters.tree_mesh_generator_settings.enable_foliage = true;
  data_generation_parameters.tree_mesh_generator_settings.vertex_color_mode =
      static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::InternodeColor);
  // Max amount of branches
  data_generation_parameters.simulation_settings.max_flow_count = 1024;
  // Max amound of nodes
  data_generation_parameters.simulation_settings.max_node_count = 65536;
  // Trunk length (branches will br pruned)
  data_generation_parameters.pruning_settings.low_branch_pruning = 0.2f;

  data_generation_parameters.output_folder = std::filesystem::current_path() / "TreeData";

  data_generation_parameters.export_point_cloud = false;
  data_generation_parameters.export_mesh = true;
  data_generation_parameters.export_skeleton = false;
  data_generation_parameters.export_rendering = true;
  data_generation_parameters.export_depth = true;
  // Depth value is linearized and clamp with max value. Smaller value means closer to camera. 1.0 means max depth/inf
  // depth.
  data_generation_parameters.max_depth = 8.f;
  data_generation_parameters.generate_ground_mesh = false;

  const auto scene = Application::GetActiveScene();
  scene->environment.ambient_light_intensity = 0.2f;
  const auto directional_light_entities = scene->GetPrivateComponentOwnersList<DirectionalLight>();
  for (const auto& directional_light_entity : directional_light_entities) {
    const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(directional_light_entity).lock();
    directional_light->diffuse_brightness = 2.f;
  }
  data_generation_parameters.camera_capture_settings.resize(2);
  data_generation_parameters.camera_capture_settings[0].camera_settings.fov = 60.f;
  data_generation_parameters.camera_capture_settings[1].camera_settings.fov = 60.f;
  data_generation_parameters.camera_capture_settings[0].camera_settings.use_clear_color = true;
  data_generation_parameters.camera_capture_settings[1].camera_settings.use_clear_color = true;
  data_generation_parameters.camera_capture_settings[0].camera_settings.clear_color = glm::vec3(1, 1, 1);
  data_generation_parameters.camera_capture_settings[1].camera_settings.clear_color = glm::vec3(1, 1, 1);
  data_generation_parameters.camera_capture_settings[0].camera_settings.background_intensity = 10.f;
  data_generation_parameters.camera_capture_settings[1].camera_settings.background_intensity = 10.f;
  data_generation_parameters.camera_capture_settings[0].global_transform.SetPosition(glm::vec3(0, 6, 0));
  data_generation_parameters.camera_capture_settings[1].global_transform.SetPosition(glm::vec3(0, 6, 0));
  data_generation_parameters.camera_capture_settings[0].global_transform.SetEulerRotation(
      glm::radians(glm::vec3(-90, glm::linearRand(0.f, 360.f), 0)));
  data_generation_parameters.camera_capture_settings[1].global_transform.SetEulerRotation(
      glm::radians(glm::vec3(-90, glm::linearRand(0.f, 360.f), 0)));

  data_generation_parameters.tree_descriptor_path = std::filesystem::path("./TreeStructor/TreeStructor.tree");
  data_generation_parameters.foliage_descriptor_path = std::filesystem::path("./TreeStructor/TreeStructor.foliage");
  // data_generation_parameters.bark_descriptor_path = std::filesystem::path("./TreeStructor/TreeStructor.bark");

  const auto tree_point_cloud_circular_capture_settings = std::make_shared<TreePointCloudCircularCaptureSettings>();
  const auto tree_point_cloud_grid_capture_settings = std::make_shared<TreePointCloudGridCaptureSettings>();

  tree_point_cloud_circular_capture_settings->distance_from_trees = 4.0f;
  tree_point_cloud_circular_capture_settings->capture_height = 3.0f;

  constexpr glm::ivec2 grid_size = {3, 3};
  tree_point_cloud_grid_capture_settings->grid_size = {grid_size.x + 1, grid_size.y + 1};
  tree_point_cloud_grid_capture_settings->ground_sample_size = 1024;
  tree_point_cloud_grid_capture_settings->drone_sample_size = 256;

  // data_generation_parameters.growth_capture = {4096};

  for (int index = 0; index < 2; index++) {
    data_generation_parameters.output_file_prefix =
        data_generation_parameters.tree_descriptor_path.stem().string() + "_" + std::to_string(index);
    DatasetGenerator::GenerateDataForTree(data_generation_parameters, tree_point_cloud_circular_capture_settings);
  }
}

int main() {
#ifndef ECOSYSLAB_PLUGIN
  throw std::runtime_error("EcoSysLab plugin missing!");
#endif
#ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#endif

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

  const std::filesystem::path project_path = resource_folder_path / "EcoSysLabProject" / "test.eveproj";
  run_windowless(project_path);

  generate_tree_data();

  EVOENGINE_LOG("Generation Finished!")

  Application::Terminate();
}
