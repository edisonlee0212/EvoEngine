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

void push_layers(const bool use_gpu, const bool enable_window_layer, const bool enable_editor_layer) {
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>("Window Layer");
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>("Editor Layer");
#  ifdef DIGITAL_AGRICULTURE_PLUGIN
  Application::PushLayer<SorghumLayer>("Sorghum Layer");
#  endif
#  ifdef CUDA_MODULE_PLUGIN
  if (use_gpu)
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

void generate_sorghum_data(const bool use_gpu, const SorghumGantryCaptureSettings& capture_settings,
                           DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters) {
  const auto gantry_capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  *gantry_capture_settings = capture_settings;
  gantry_capture_settings->grid_distance = {2.f, 2.f};
  gantry_capture_settings->grid_size.x = gantry_capture_settings->grid_size.y = 1;
  data_generation_parameters.point_cloud_capture_settings = gantry_capture_settings;
  if (use_gpu) {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::OptiX;
  } else {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Cpu;
  }
  DatasetGenerator::GenerateDataForSorghum(data_generation_parameters);
}

void generate_sorghum_grid_data(const bool use_gpu, const SorghumGantryCaptureSettings& capture_settings,
                                const SorghumGrid& sorghum_grid,
                                DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters) {
  const auto gantry_capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  *gantry_capture_settings = capture_settings;
  gantry_capture_settings->grid_distance = glm::vec2(sorghum_grid.grid_distance_x, sorghum_grid.grid_distance_y);
  gantry_capture_settings->grid_size.x = sorghum_grid.grid_size_x;
  gantry_capture_settings->grid_size.y = sorghum_grid.grid_size_y;
  data_generation_parameters.point_cloud_capture_settings = gantry_capture_settings;
  if (use_gpu) {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::OptiX;
  } else {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Cpu;
  }
  DatasetGenerator::GenerateDataForSorghumGrid(sorghum_grid, data_generation_parameters);
}

void generate_sorghum_field_data(const bool use_gpu, const SorghumGantryCaptureSettings& capture_settings,
                                 int grid_size, float grid_distance, const std::filesystem::path& sorghum_field_path,
                                 DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters) {
  const auto gantry_capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  *gantry_capture_settings = capture_settings;
  gantry_capture_settings->grid_distance = glm::vec2(grid_size);
  gantry_capture_settings->grid_size.x = grid_distance;
  gantry_capture_settings->grid_size.y = grid_distance;
  data_generation_parameters.point_cloud_capture_settings = gantry_capture_settings;
  std::shared_ptr<SorghumField> sorghum_field{};
  if (sorghum_field_path.is_relative()) {
    const auto absolute_path = ProjectManager::GetAssetsFolderPath() / sorghum_field_path;
    if (std::filesystem::exists(absolute_path)) {
      sorghum_field = std::dynamic_pointer_cast<SorghumField>(ProjectManager::GetOrCreateAsset(sorghum_field_path));
    } else {
      EVOENGINE_ERROR("Sorghum Field doesn't exist!")
      return;
    }
  } else {
    if (ProjectManager::IsInAssetsFolder(sorghum_field_path)) {
      sorghum_field = std::dynamic_pointer_cast<SorghumField>(
          ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(sorghum_field_path)));
    } else {
      EVOENGINE_ERROR("Sorghum Field doesn't exist!")
      return;
    }
  }
  if (use_gpu) {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::OptiX;
  } else {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Cpu;
  }
  DatasetGenerator::GenerateDataForSorghumField(sorghum_field, data_generation_parameters);
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

  py::class_<DatasetGenerator::SorghumDataGenerationParameters>(m, "SorghumDataGenerationParameters")
      .def(py::init<>())
      .def_readwrite("sorghum_path", &DatasetGenerator::SorghumDataGenerationParameters::sorghum_path)

      .def_readwrite("export_point_cloud", &DatasetGenerator::SorghumDataGenerationParameters::export_point_cloud)
      .def_readwrite("export_mesh", &DatasetGenerator::SorghumDataGenerationParameters::export_mesh)

      .def_readwrite("generate_ground_mesh", &DatasetGenerator::SorghumDataGenerationParameters::generate_ground_mesh)
      .def_readwrite("avoid_occlusion", &DatasetGenerator::SorghumDataGenerationParameters::avoid_occlusion)

      .def_readwrite("sorghum_point_cloud_point_settings",
                     &DatasetGenerator::SorghumDataGenerationParameters::sorghum_point_cloud_point_settings)
      .def_readwrite("sorghum_mesh_generator_settings",
                     &DatasetGenerator::SorghumDataGenerationParameters::sorghum_mesh_generator_settings)

      .def_readwrite("seed", &DatasetGenerator::SorghumDataGenerationParameters::seed)
      .def_readwrite("output_folder", &DatasetGenerator::SorghumDataGenerationParameters::output_folder)
      .def_readwrite("output_file_name", &DatasetGenerator::SorghumDataGenerationParameters::output_file_name);

  py::class_<SorghumGantryCaptureSettings>(m, "SorghumGantryCaptureSettings")
      .def(py::init<>())
      .def_readwrite("bounding_box_size", &SorghumGantryCaptureSettings::bounding_box_size)
      .def_readwrite("step", &SorghumGantryCaptureSettings::step)
      .def_readwrite("output_spline_info", &SorghumGantryCaptureSettings::output_spline_info)
      .def_readwrite("sample_height", &SorghumGantryCaptureSettings::sample_height);

  py::class_<SorghumGrid>(m, "SorghumGrid")
      .def(py::init<>())
      .def_readwrite("grid_distance_x", &SorghumGrid::grid_distance_x)
      .def_readwrite("grid_distance_y", &SorghumGrid::grid_distance_y)
      .def_readwrite("position_offset_mean", &SorghumGrid::position_offset_mean)
      .def_readwrite("position_offset_variance", &SorghumGrid::position_offset_variance)
      .def_readwrite("rotation_variance_xz", &SorghumGrid::rotation_variance_xz)
      .def_readwrite("rotation_variance_y", &SorghumGrid::rotation_variance_y)
      .def_readwrite("grid_size_x", &SorghumGrid::grid_size_x)
      .def_readwrite("grid_size_y", &SorghumGrid::grid_size_y);

  m.def("get_default_project_path", &get_default_project_path, "Get default project path");
  m.def("engine_run_windowless", &engine_run_windowless, "Start Project (Windowless)");
  m.def("engine_run", &engine_run, "Start Project (No Editor)");
  m.def("engine_run_with_editor", &engine_run_with_editor, "Start Project (with Editor)");
  m.def("engine_loop", &engine_loop, "Loop Application");
  m.def("engine_terminate", &engine_terminate, "Terminate Application");

  m.def("generate_sorghum_data", &generate_sorghum_data, "Generate data point for single sorghum");
  m.def("generate_sorghum_grid_data", &generate_sorghum_grid_data, "Generate data point for a grid of sorghums");
  m.def("generate_sorghum_field_data", &generate_sorghum_field_data, "Generate data point for a sorghum field");
}
#endif