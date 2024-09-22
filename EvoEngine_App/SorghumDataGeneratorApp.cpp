#include "Application.hpp"
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"

#include "RenderLayer.hpp"

#ifdef DIGITAL_AGRICULTURE_PLUGIN

#  include "SorghumLayer.hpp"
using namespace digital_agriculture_plugin;
#endif
#include "WindowLayer.hpp"
#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif

#ifdef DATASET_GENERATION_PLUGIN
#  include <SorghumPointCloudScanner.hpp>
#  include <TreePointCloudScanner.hpp>
#  include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#endif

#ifdef ECO_SYS_LAB_PLUGIN
#  include "Soil.hpp"
#endif

using namespace evo_engine;

void register_classes() {
#ifdef DATASET_GENERATION_PLUGIN
  PrivateComponentRegistration<SorghumPointCloudScanner>("SorghumPointCloudScanner");
#endif
}

void register_layers(bool enable_window_layer, bool enable_editor_layer) {
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>();
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
#ifdef DIGITAL_AGRICULTURE_PLUGIN
  Application::PushLayer<SorghumLayer>();
#endif
#ifdef OPTIX_RAY_TRACER_PLUGIN
  Application::PushLayer<RayTracerLayer>();
#endif
}

void start_project(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();
  register_layers(true, true);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene =
      std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset("DigitalAgriculture.evescene"));
  Application::Attach(new_scene);
  Application::Start();
}

void start_project_windowless(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();
  register_layers(false, false);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset("DigitalAgriculture.evescene"));
  Application::Attach(new_scene);
  Application::Start();
}

void sorghum_field_point_cloud(int grid_size, float grid_distance,
    float random_shift, float variance, uint32_t size,
                               const std::shared_ptr<SorghumGantryCaptureSettings> &sorghum_gantry_capture_settings,
    const std::filesystem::path& sorghum_descriptor_generator_relative_path, 
    const std::filesystem::path& output_folder) {
#ifndef DIGITAL_AGRICULTURE_PLUGIN
  throw std::runtime_error("DigitalAgriculture plugin missing!");
#else
#  ifndef OPTIX_RAY_TRACER_PLUGIN
  throw std::runtime_error("OptixRayTracer plugin missing!");
#  else
#    ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#    endif
#  endif
  
  SorghumMeshGeneratorSettings sorghum_mesh_generator_settings{};
  std::filesystem::create_directories(output_folder);
  SorghumPointCloudPointSettings sorghum_point_cloud_point_settings{};
  sorghum_point_cloud_point_settings.ball_rand_radius = 0.005f;
  sorghum_point_cloud_point_settings.variance = 0.0f;
  sorghum_point_cloud_point_settings.instance_index = true;
  sorghum_point_cloud_point_settings.type_index = true;
  sorghum_point_cloud_point_settings.leaf_index = true;
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

  sorghum_gantry_capture_settings->grid_size = {grid_size, grid_size};
  sorghum_gantry_capture_settings->grid_distance = {grid_distance, grid_distance};
  SorghumFieldPatch pattern{};
  pattern.grid_size = {grid_size, grid_size};
  pattern.grid_distance = {grid_distance, grid_distance};
  pattern.position_offset_mean = {random_shift, random_shift};
  pattern.position_offset_variance = {variance, variance};
  int index = 0;
  for (int i = 0; i < size; i++) {
    std::filesystem::path target_descriptor_folder_path =
        resource_folder_path / "DigitalAgricultureProject" / "SorghumDescriptorGenerator";
    const auto sorghum_descriptor = std::dynamic_pointer_cast<SorghumDescriptorGenerator>(
        ProjectManager::GetOrCreateAsset(sorghum_descriptor_generator_relative_path));
    std::string name = "Sorghum_" + std::to_string(i);
    std::filesystem::path target_tree_point_cloud_path = output_folder / (name + ".ply");
    DatasetGenerator::GeneratePointCloudForSorghumPatch(
        pattern, sorghum_descriptor, sorghum_point_cloud_point_settings, sorghum_gantry_capture_settings,
        sorghum_mesh_generator_settings, target_tree_point_cloud_path.string());
    index++;
  }
#endif
}

int main() {
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

  const std::filesystem::path project_path = resource_folder_path / "DigitalAgricultureProject" / "test.eveproj";
  //start_project(project_path);
  start_project_windowless(project_path);
  std::shared_ptr<SorghumGantryCaptureSettings> capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  capture_settings->step = glm::vec2(0.001f);
  capture_settings->scanner_angles = {30, 60};
  capture_settings->output_spline_info = true;
  const auto sdg_relative_path = std::filesystem::path("SorghumDescriptorGenerator") / "Season12.sdg";
  sorghum_field_point_cloud(1, 0.75f, 0, 0, 8, capture_settings, sdg_relative_path, "D:\\SorghumPointCloudData\\");
  Application::Run();
}
