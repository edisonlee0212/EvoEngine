#include "Application.hpp"
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"

#include "RenderLayer.hpp"

#ifdef DIGITAL_AGRICULTURE_PLUGIN

#  include "SorghumLayer.hpp"
#  include "MaizeLayer.hpp"
using namespace digital_agriculture_plugin;
#endif
#include "WindowLayer.hpp"

#ifdef DATASET_GENERATION_PLUGIN
#  include <SorghumPointCloudScanner.hpp>
#  include <TasselPointCloudScanner.hpp>
#  include <TreePointCloudScanner.hpp>
#  include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#endif

#ifdef ECOSYSLAB_PLUGIN
#  include "Soil.hpp"
#endif

using namespace evo_engine;

void register_classes() {
#ifdef DATASET_GENERATION_PLUGIN
  PrivateComponentRegistration<SorghumPointCloudScanner>("SorghumPointCloudScanner");
  PrivateComponentRegistration<TasselPointCloudScanner>("TasselPointCloudScanner");
#endif
}

void run_with_editor(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();

  Application::PushLayer<RenderLayer>("Render Layer");
  Application::PushLayer<WindowLayer>("Window Layer");

  Application::PushLayer<EditorLayer>("Editor Layer");
#ifdef DIGITAL_AGRICULTURE_PLUGIN
  Application::PushLayer<SorghumLayer>("Sorghum Layer");
  Application::PushLayer<MaizeLayer>("Maize Layer");
#endif

  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene =
      std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset("DigitalAgriculture.evescene"));
  Application::Attach(new_scene);
  Application::Start();
}

void run_windowless(const PointCloudCaptureSettings::CaptureMode capture_mode,
                    const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();
  switch (capture_mode) {
    case PointCloudCaptureSettings::CaptureMode::Cpu: {
#ifdef DIGITAL_AGRICULTURE_PLUGIN
      Application::PushLayer<SorghumLayer>("Sorghum Layer");
#endif
    } break;
    case PointCloudCaptureSettings::CaptureMode::Gpu:
      Application::PushLayer<RenderLayer>("Render Layer");
#ifdef DIGITAL_AGRICULTURE_PLUGIN
      Application::PushLayer<SorghumLayer>("Sorghum Layer");
      Application::PushLayer<MaizeLayer>("Maize Layer");
#endif
      break;
  }
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene =
      std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset("DigitalAgriculture.evescene"));
  Application::Attach(new_scene);
  Application::Start();
}

void sorghum_field_point_cloud(const uint32_t output_size, int grid_size, float grid_distance, const float random_shift,
                               const float variance,
                               const std::shared_ptr<SorghumGantryCaptureSettings>& sorghum_gantry_capture_settings,
                               const std::filesystem::path& sorghum_generator_path,
                               const std::filesystem::path& output_folder) {
#ifndef DIGITAL_AGRICULTURE_PLUGIN
  throw std::runtime_error("DigitalAgriculture plugin missing!");
#endif
#ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#endif

  std::filesystem::create_directories(output_folder);

  sorghum_gantry_capture_settings->grid_size = {grid_size, grid_size};
  sorghum_gantry_capture_settings->grid_distance = {grid_distance, grid_distance};
  SorghumGrid sorghum_grid{};
  sorghum_grid.grid_size.x = sorghum_grid.grid_size.y = grid_size;
  sorghum_grid.grid_distance.x = sorghum_grid.grid_distance.y = grid_distance;
  sorghum_grid.position_offset_mean = random_shift;
  sorghum_grid.position_offset_variance = variance;

  DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters{};
  data_generation_parameters.export_point_cloud = true;
  data_generation_parameters.output_folder = output_folder;
  data_generation_parameters.point_cloud_capture_settings = sorghum_gantry_capture_settings;
  data_generation_parameters.sorghum_point_cloud_point_settings.ball_rand_radius = 0.005f;
  data_generation_parameters.sorghum_point_cloud_point_settings.variance = 0.0f;
  data_generation_parameters.sorghum_point_cloud_point_settings.instance_index = true;
  data_generation_parameters.sorghum_point_cloud_point_settings.type_index = true;
  data_generation_parameters.sorghum_point_cloud_point_settings.leaf_index = true;
  data_generation_parameters.sorghum_mesh_generator_settings.leaf_separated = true;

  int index = 0;
  const auto sorghum_field = AssetManager::CreateTemporaryAsset<SorghumField>();
  const auto scene = Application::GetActiveScene();

  for (int i = 0; i < output_size; i++) {
    const std::string prefix = "SorghumField_" + std::to_string(i);
    const auto seed = i * grid_size * grid_size;
    DatasetGenerator::ApplySorghumGrid(sorghum_field, sorghum_generator_path, sorghum_grid);
    const auto sorghum_field_entity = DatasetGenerator::CreateSorghumEntity(sorghum_field, seed);
    data_generation_parameters.output_file_name = prefix;
    DatasetGenerator::GenerateDataForAllSorghums(data_generation_parameters);
    scene->DeleteEntity(sorghum_field_entity);
    index++;
  }
}
void sorghum_point_cloud(const uint32_t output_size, const bool avoid_occlusion, const bool generate_ground,
                         const std::shared_ptr<SorghumGantryCaptureSettings>& sorghum_gantry_capture_settings,
                         const std::filesystem::path& sorghum_generator_path,
                         const std::filesystem::path& output_folder) {
#ifndef DIGITAL_AGRICULTURE_PLUGIN
  throw std::runtime_error("DigitalAgriculture plugin missing!");
#endif
#ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#endif
  sorghum_gantry_capture_settings->grid_size = {1, 1};
  sorghum_gantry_capture_settings->grid_distance = {2.0, 2.0};

  DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters{};
  data_generation_parameters.export_point_cloud = true;
  data_generation_parameters.output_folder = output_folder;
  data_generation_parameters.point_cloud_capture_settings = sorghum_gantry_capture_settings;
  data_generation_parameters.sorghum_point_cloud_point_settings.ball_rand_radius = 0.005f;
  data_generation_parameters.sorghum_point_cloud_point_settings.variance = 0.0f;
  data_generation_parameters.sorghum_point_cloud_point_settings.instance_index = true;
  data_generation_parameters.sorghum_point_cloud_point_settings.type_index = true;
  data_generation_parameters.sorghum_point_cloud_point_settings.leaf_index = true;
  data_generation_parameters.sorghum_mesh_generator_settings.leaf_separated = true;
  data_generation_parameters.avoid_occlusion = avoid_occlusion;
  data_generation_parameters.generate_ground_mesh = generate_ground;
  int index = 0;
  const auto scene = Application::GetActiveScene();

  for (int i = 0; i < output_size; i++) {
    std::string name = "Sorghum_" + std::to_string(i);
    const std::string prefix = "Sorghum_" + std::to_string(i);
    const auto sorghum_entity = DatasetGenerator::CreateSorghumEntity(sorghum_generator_path, i);
    data_generation_parameters.output_file_name = prefix;
    DatasetGenerator::GenerateDataForSorghum(sorghum_entity, data_generation_parameters);
    scene->DeleteEntity(sorghum_entity);
    index++;
  }
}

void sorghum_mesh_point_cloud(const uint32_t output_size, const bool avoid_occlusion, const bool generate_ground,
                              const std::shared_ptr<SorghumGantryCaptureSettings>& sorghum_gantry_capture_settings,
                              const std::filesystem::path& sorghum_generator_path,
                              const std::filesystem::path& output_folder) {
#ifndef DIGITAL_AGRICULTURE_PLUGIN
  throw std::runtime_error("DigitalAgriculture plugin missing!");
#endif
#ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#endif
  sorghum_gantry_capture_settings->grid_size = {1, 1};
  sorghum_gantry_capture_settings->grid_distance = {2.0, 2.0};
  DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters{};
  data_generation_parameters.export_mesh = true;
  data_generation_parameters.export_point_cloud = true;
  data_generation_parameters.output_folder = output_folder;
  data_generation_parameters.point_cloud_capture_settings = sorghum_gantry_capture_settings;
  data_generation_parameters.sorghum_point_cloud_point_settings.ball_rand_radius = 0.005f;
  data_generation_parameters.sorghum_point_cloud_point_settings.variance = 0.0f;
  data_generation_parameters.sorghum_point_cloud_point_settings.instance_index = true;
  data_generation_parameters.sorghum_point_cloud_point_settings.type_index = true;
  data_generation_parameters.sorghum_point_cloud_point_settings.leaf_index = true;
  data_generation_parameters.sorghum_mesh_generator_settings.leaf_separated = true;
  data_generation_parameters.avoid_occlusion = avoid_occlusion;
  data_generation_parameters.generate_ground_mesh = generate_ground;
  int index = 0;
  const auto scene = Application::GetActiveScene();

  for (int i = 0; i < output_size; i++) {
    std::string name = "Sorghum_" + std::to_string(i);
    const std::string prefix = "Sorghum_" + std::to_string(i);
    const auto sorghum_entity = DatasetGenerator::CreateSorghumEntity(sorghum_generator_path, i);
    data_generation_parameters.output_file_name = prefix;
    DatasetGenerator::GenerateDataForSorghum(sorghum_entity, data_generation_parameters);
    scene->DeleteEntity(sorghum_entity);
    index++;
  }
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

  const auto capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  capture_settings->step = 0.005f;  // Smaller -> more points.
  capture_settings->scanner_angles = {30};
  capture_settings->output_spline_info = true;
  capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Gpu;

  run_windowless(capture_settings->capture_mode, project_path);
  const auto sg_relative_path = std::filesystem::path("SorghumGenerator") / "Random.sg";
  const auto output_folder_path = std::filesystem::current_path() / "SorghumData";
  sorghum_field_point_cloud(1, 8, 0.75f, 0, 0, capture_settings, sg_relative_path, output_folder_path);
  sorghum_mesh_point_cloud(1, true, false, capture_settings, sg_relative_path, output_folder_path);

  EVOENGINE_LOG("Generation Finished!")

  // Open File Explorer for generated files.
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  const auto folder_path = output_folder_path.string();
  ShellExecuteA(nullptr, "open", folder_path.c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
#endif
  Application::Terminate();
}
