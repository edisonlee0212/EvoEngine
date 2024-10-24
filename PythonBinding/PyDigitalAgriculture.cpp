#ifdef DIGITAL_AGRICULTURE_PLUGIN

#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "EditorLayer.hpp"
#include "HeightField.hpp"
#include "MeshRenderer.hpp"
#include "ObjectRotator.hpp"
#include "PlayerController.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RadialBoundingVolume.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "Soil.hpp"
#include "SorghumLayer.hpp"
#include "Times.hpp"
#include "Tree.hpp"
#include "TreeModel.hpp"
#include "TreeStructor.hpp"
#include "WindowLayer.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl/filesystem.h"
#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif


#  if DATASET_GENERATION_PLUGIN
#    include <SorghumPointCloudScanner.hpp>
#    include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#  endif

using namespace evo_engine;
using namespace digital_agriculture_plugin;

namespace py = pybind11;

void register_classes() {
  PrivateComponentRegistration<ObjectRotator>("ObjectRotator");
}

void push_layers(const bool enable_window_layer, const bool enable_editor_layer) {
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>();
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
  Application::PushLayer<SorghumLayer>();
#ifdef OPTIX_RAY_TRACER_PLUGIN
  Application::PushLayer<RayTracerLayer>();
#endif
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

  return resource_folder_path / "DigitalAgricultureProject" / "test.eveproj";
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

void generate_sorghum_mesh(const std::string& sorghum_descriptor_path,
                           const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                           const std::filesystem::path& mesh_output_path) {
  std::shared_ptr<SorghumDescriptor> sorghum_descriptor;
  if (const auto path = std::filesystem::path(sorghum_descriptor_path); path.is_absolute()) {
    sorghum_descriptor = ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();
    sorghum_descriptor->Import(sorghum_descriptor_path);
  } else {
    sorghum_descriptor = std::dynamic_pointer_cast<SorghumDescriptor>(ProjectManager::GetOrCreateAsset(path));
  }
  if (!sorghum_descriptor) {
    EVOENGINE_ERROR("Failed to import sorghum descriptor!")
    return;
  }
  
}

void generate_sorghum_point_cloud(const std::string& sorghum_descriptor_path,
                                  const SorghumPointCloudPointSettings& point_settings,
                                  const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                  bool avoid_occlusion, const std::filesystem::path& point_cloud_output_path) {
  std::shared_ptr<SorghumDescriptor> sorghum_descriptor;
  if (const auto path = std::filesystem::path(sorghum_descriptor_path); path.is_absolute()) {
    sorghum_descriptor = ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();
    sorghum_descriptor->Import(sorghum_descriptor_path);
  } else {
    sorghum_descriptor = std::dynamic_pointer_cast<SorghumDescriptor>(ProjectManager::GetOrCreateAsset(path));
  }
  if (!sorghum_descriptor) {
    EVOENGINE_ERROR("Failed to import sorghum descriptor!")
    return;
  }
  const auto capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  capture_settings->step = glm::vec2(0.005f);  // Smaller -> more points.
  capture_settings->scanner_angles = {30, 60};
  capture_settings->output_spline_info = true;

  DatasetGenerator::GeneratePointCloudForSorghum(sorghum_descriptor, point_settings, capture_settings,
                                                 sorghum_mesh_generator_settings, avoid_occlusion,
                                                 point_cloud_output_path);
}

PYBIND11_MODULE(PyDigitalAgriculture, m) {
  m.doc() = "PyDigitalAgriculture";  // optional module docstring
  m.def("get_default_project_path", &get_default_project_path, "Get default project path");
  m.def("engine_run_windowless", &engine_run_windowless, "Start Project (Windowless)");
  m.def("engine_run", &engine_run, "Start Project (No Editor)");
  m.def("engine_run_with_editor", &engine_run_with_editor, "Start Project (with Editor)");
  m.def("engine_loop", &engine_loop, "Loop Application");
  m.def("engine_terminate", &engine_terminate, "Terminate Application");

  m.def("generate_sorghum_point_cloud", &generate_sorghum_point_cloud, "Create a sorghum and generate point cloud");
}
#endif