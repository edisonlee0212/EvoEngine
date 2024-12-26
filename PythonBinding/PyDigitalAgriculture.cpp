#ifdef DIGITAL_AGRICULTURE_PLUGIN

#  include "AnimationPlayer.hpp"
#  include "Application.hpp"
#  include "ClassRegistry.hpp"
#  include "Climate.hpp"
#  include "EditorLayer.hpp"
#  include "HeightField.hpp"
#  include "MeshRenderer.hpp"
#  include "ObjectRotator.hpp"
#  include "PlayerController.hpp"
#  include "PostProcessingStack.hpp"
#  include "Prefab.hpp"
#  include "ProjectManager.hpp"
#  include "RadialBoundingVolume.hpp"
#  include "RenderLayer.hpp"
#  include "Scene.hpp"
#  include "Soil.hpp"
#  include "SorghumLayer.hpp"
#  include "Times.hpp"
#  include "Tree.hpp"
#  include "TreeModel.hpp"
#  include "TreeStructor.hpp"
#  include "WindowLayer.hpp"
#  include "pybind11/pybind11.h"
#  include "pybind11/stl/filesystem.h"
#  ifdef CUDA_MODULE_PLUGIN
#    include <CUDAModule.hpp>
#    include <RayTracerLayer.hpp>
#  endif

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
#  ifdef DATASET_GENERATION_PLUGIN
  PrivateComponentRegistration<SorghumPointCloudScanner>("SorghumPointCloudScanner");
#  endif
}

void push_layers(const bool enable_render_layer, const bool enable_window_layer, const bool enable_editor_layer) {
  if (enable_render_layer)
    Application::PushLayer<RenderLayer>("Render Layer");
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>("Window Layer");
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>("Editor Layer");
#  ifdef DIGITAL_AGRICULTURE_PLUGIN
  Application::PushLayer<SorghumLayer>("Sorghum Layer");
#  endif
#  ifdef CUDA_MODULE_PLUGIN
  if (enable_render_layer)
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

  return resource_folder_path / "DigitalAgricultureProject" / "test.eveproj";
}

void engine_run_windowless(const bool use_gpu, const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a valid project!");
    return;
  }
  register_classes();
  push_layers(use_gpu, false, false);
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
  push_layers(true, true, false);
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
  push_layers(true, true, true);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
}

bool engine_loop() {
  return Application::Loop();
}

void engine_terminate() {
  Application::Terminate();
}

void sorghum_descriptor_to_mesh(const std::string& sorghum_descriptor_path,
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
  DatasetGenerator::GenerateMeshForSorghum(sorghum_descriptor, sorghum_mesh_generator_settings, mesh_output_path);
}

void sorghum_state_to_mesh(const std::string& sorghum_state_path,
                           const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                           const std::filesystem::path& mesh_output_path) {
  std::shared_ptr<SorghumState> sorghum_state;
  if (const auto path = std::filesystem::path(sorghum_state_path); path.is_absolute()) {
    sorghum_state = ProjectManager::CreateTemporaryAsset<SorghumState>();
    sorghum_state->Import(sorghum_state_path);
  } else {
    sorghum_state = std::dynamic_pointer_cast<SorghumState>(ProjectManager::GetOrCreateAsset(path));
  }
  if (!sorghum_state) {
    EVOENGINE_ERROR("Failed to import sorghum state!")
    return;
  }
  DatasetGenerator::GenerateMeshForSorghum(sorghum_state, sorghum_mesh_generator_settings, mesh_output_path);
}

void sorghum_descriptor_to_point_cloud(bool use_gpu, const std::string& sorghum_descriptor_path,
                                       const SorghumPointCloudPointSettings& point_settings,
                                       const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                       const bool avoid_occlusion, const bool generate_ground,
                                       const std::filesystem::path& point_cloud_output_path) {
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
  capture_settings->use_gpu = use_gpu;
  DatasetGenerator::GeneratePointCloudForSorghum(sorghum_descriptor, point_settings, capture_settings,
                                                 sorghum_mesh_generator_settings, avoid_occlusion, generate_ground,
                                                 point_cloud_output_path);
}

void sorghum_state_to_point_cloud(bool use_gpu, const std::string& sorghum_state_path,
                                  const SorghumPointCloudPointSettings& point_settings,
                                  const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                  const bool avoid_occlusion, const bool generate_ground,
                                  const std::filesystem::path& point_cloud_output_path) {
  std::shared_ptr<SorghumState> sorghum_state;
  if (const auto path = std::filesystem::path(sorghum_state_path); path.is_absolute()) {
    sorghum_state = ProjectManager::CreateTemporaryAsset<SorghumState>();
    sorghum_state->Import(sorghum_state_path);
  } else {
    sorghum_state = std::dynamic_pointer_cast<SorghumState>(ProjectManager::GetOrCreateAsset(path));
  }
  if (!sorghum_state) {
    EVOENGINE_ERROR("Failed to import sorghum state!")
    return;
  }
  const auto capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  capture_settings->step = glm::vec2(0.005f);  // Smaller -> more points.
  capture_settings->scanner_angles = {30, 60};
  capture_settings->output_spline_info = true;
  capture_settings->use_gpu = use_gpu;
  DatasetGenerator::GeneratePointCloudForSorghum(sorghum_state, point_settings, capture_settings,
                                                 sorghum_mesh_generator_settings, avoid_occlusion, generate_ground,
                                                 point_cloud_output_path);
}

void sorghum_descriptor_to_mesh_and_point_cloud(bool use_gpu, const std::string& sorghum_descriptor_path,
                                                const SorghumPointCloudPointSettings& point_settings,
                                                const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                bool avoid_occlusion, const bool generate_ground,
                                                const std::filesystem::path& mesh_output_path,
                                                const std::filesystem::path& point_cloud_output_path) {
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
  capture_settings->use_gpu = use_gpu;
  DatasetGenerator::GenerateMeshAndPointCloudForSorghum(sorghum_descriptor, point_settings, capture_settings,
                                                        sorghum_mesh_generator_settings, avoid_occlusion,
                                                        generate_ground, mesh_output_path, point_cloud_output_path);
}

void sorghum_state_to_mesh_and_point_cloud(bool use_gpu, const std::string& sorghum_state_path,
                                           const SorghumPointCloudPointSettings& point_settings,
                                           const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                           bool avoid_occlusion, const bool generate_ground,
                                           const std::filesystem::path& mesh_output_path,
                                           const std::filesystem::path& point_cloud_output_path) {
  std::shared_ptr<SorghumState> sorghum_state;
  if (const auto path = std::filesystem::path(sorghum_state_path); path.is_absolute()) {
    sorghum_state = ProjectManager::CreateTemporaryAsset<SorghumState>();
    sorghum_state->Import(sorghum_state_path);
  } else {
    sorghum_state = std::dynamic_pointer_cast<SorghumState>(ProjectManager::GetOrCreateAsset(path));
  }
  if (!sorghum_state) {
    EVOENGINE_ERROR("Failed to import sorghum descriptor!")
    return;
  }
  const auto capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  capture_settings->step = glm::vec2(0.005f);  // Smaller -> more points.
  capture_settings->scanner_angles = {30, 60};
  capture_settings->output_spline_info = true;
  capture_settings->use_gpu = use_gpu;
  DatasetGenerator::GenerateMeshAndPointCloudForSorghum(sorghum_state, point_settings, capture_settings,
                                                        sorghum_mesh_generator_settings, avoid_occlusion,
                                                        generate_ground, mesh_output_path, point_cloud_output_path);
}

PYBIND11_MODULE(PyDigitalAgriculture, m) {
  m.doc() = "PyDigitalAgriculture";  // optional module docstring

  py::class_<SorghumMeshGeneratorSettings>(m, "SorghumMeshGeneratorSettings")
      .def(py::init<>())
      .def_readwrite("enable_panicle", &SorghumMeshGeneratorSettings::enable_panicle)
      .def_readwrite("enable_stem", &SorghumMeshGeneratorSettings::enable_stem)
      .def_readwrite("enable_leaves", &SorghumMeshGeneratorSettings::enable_leaves)
      .def_readwrite("enable_leaf_sheath", &SorghumMeshGeneratorSettings::enable_leaf_sheath)
      .def_readwrite("single_leaf_index", &SorghumMeshGeneratorSettings::single_leaf_index)
      .def_readwrite("bottom_face", &SorghumMeshGeneratorSettings::bottom_face)
      .def_readwrite("leaf_separated", &SorghumMeshGeneratorSettings::leaf_separated)
      .def_readwrite("leaf_thickness", &SorghumMeshGeneratorSettings::leaf_thickness);

  py::class_<SorghumPointCloudPointSettings>(m, "SorghumPointCloudPointSettings")
      .def(py::init<>())
      .def_readwrite("variance", &SorghumPointCloudPointSettings::variance)
      .def_readwrite("ball_rand_radius", &SorghumPointCloudPointSettings::ball_rand_radius)
      .def_readwrite("type_index", &SorghumPointCloudPointSettings::type_index)
      .def_readwrite("instance_index", &SorghumPointCloudPointSettings::instance_index)
      .def_readwrite("leaf_index", &SorghumPointCloudPointSettings::leaf_index)
      .def_readwrite("bounding_box_limit", &SorghumPointCloudPointSettings::bounding_box_limit);

  m.def("get_default_project_path", &get_default_project_path, "Get default project path");
  m.def("engine_run_windowless", &engine_run_windowless, "Start Project (Windowless)");
  m.def("engine_run", &engine_run, "Start Project (No Editor)");
  m.def("engine_run_with_editor", &engine_run_with_editor, "Start Project (with Editor)");
  m.def("engine_loop", &engine_loop, "Loop Application");
  m.def("engine_terminate", &engine_terminate, "Terminate Application");

  m.def("sorghum_descriptor_to_mesh", &sorghum_descriptor_to_mesh, "Create a sorghum and generate mesh");
  m.def("sorghum_descriptor_to_point_cloud", &sorghum_descriptor_to_point_cloud,
        "Create a sorghum and generate point cloud");
  m.def("sorghum_descriptor_to_mesh_and_point_cloud", &sorghum_descriptor_to_mesh_and_point_cloud,
        "Create a sorghum and generate mesh and point cloud");

  m.def("sorghum_state_to_mesh", &sorghum_state_to_mesh, "Create a sorghum and generate mesh");
  m.def("sorghum_state_to_point_cloud", &sorghum_state_to_point_cloud, "Create a sorghum and generate point cloud");
  m.def("sorghum_state_to_mesh_and_point_cloud", &sorghum_state_to_mesh_and_point_cloud,
        "Create a sorghum and generate mesh and point cloud");
}
#endif