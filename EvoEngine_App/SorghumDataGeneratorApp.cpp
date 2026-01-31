#include "Application.hpp"
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"

#include "RenderLayer.hpp"
#include "Sorghum.hpp"

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
#  include "Soil.hpp"
#endif

using namespace evo_engine;

void register_classes() {
#ifdef DATASET_GENERATION_PLUGIN
  PrivateComponentRegistration<SorghumPointCloudScanner>("SorghumPointCloudScanner");
#endif
}

void run_with_editor(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();

  Application::PushLayer<RenderLayer>("Render Layer");
#ifdef CUDA_MODULE_PLUGIN
  Application::PushLayer<RayTracerLayer>("Ray Tracer Layer");
#endif
  Application::PushLayer<WindowLayer>("Window Layer");

  Application::PushLayer<EditorLayer>("Editor Layer");
#ifdef DIGITAL_AGRICULTURE_PLUGIN
  Application::PushLayer<SorghumLayer>("Sorghum Layer");
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
    case PointCloudCaptureSettings::CaptureMode::OptiX: {
#ifdef DIGITAL_AGRICULTURE_PLUGIN
      Application::PushLayer<SorghumLayer>("Sorghum Layer");
#endif
#ifdef CUDA_MODULE_PLUGIN
      Application::PushLayer<RayTracerLayer>("Ray Tracer Layer");
#endif
    } break;
    case PointCloudCaptureSettings::CaptureMode::Cpu: {
#ifdef DIGITAL_AGRICULTURE_PLUGIN
      Application::PushLayer<SorghumLayer>("Sorghum Layer");
#endif
    } break;
    case PointCloudCaptureSettings::CaptureMode::GpuCompute:
      Application::PushLayer<RenderLayer>("Render Layer");
#ifdef DIGITAL_AGRICULTURE_PLUGIN
      Application::PushLayer<SorghumLayer>("Sorghum Layer");
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
  data_generation_parameters.sorghum_mesh_generator_settings.leaf_separated = false;
  data_generation_parameters.avoid_occlusion = avoid_occlusion;
  data_generation_parameters.generate_ground_mesh = generate_ground;
  int index = 0;
  const auto scene = Application::GetActiveScene();

  const bool save_temporary_sorghum_descriptors = false;

  const bool save_skeleton = true;
  const int start_index = 0; 

  for (int i = start_index; i < output_size+start_index; i++) {
    std::cout << "generating Sorghum_" << std::to_string(i) << std::endl;
    std::string name = "Sorghum_" + std::to_string(i);
    const std::string prefix = "Sorghum_" + std::to_string(i);
    const auto sorghum_entity = DatasetGenerator::CreateSorghumEntity(sorghum_generator_path, i);
    data_generation_parameters.output_file_name = prefix;
    DatasetGenerator::GenerateDataForSorghum(sorghum_entity, data_generation_parameters);

    if (save_temporary_sorghum_descriptors) {
      

      // export the content of the temporary asset 
      auto temporary_sorghum_descriptor = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock()->sorghum_descriptor.Get<SorghumDescriptor>();

      temporary_sorghum_descriptor->Export(data_generation_parameters.output_folder /
                                           (data_generation_parameters.output_file_name + ".sorghum"));
    }

    if (save_skeleton) {
      auto temporary_sorghum_descriptor =
          scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock()->sorghum_descriptor.Get<SorghumDescriptor>();
      auto save_path =
          data_generation_parameters.output_folder / (data_generation_parameters.output_file_name + "_spline.yml");
      try {
        std::filesystem::path yaml_path = save_path;
        yaml_path.replace_extension(".yml");
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "Sorghums" << YAML::BeginSeq;

        if (scene->IsEntityValid(sorghum_entity)) {

          out << YAML::BeginMap;
          {
            out << YAML::Key << "Instance Index" << YAML::Value << sorghum_entity.GetIndex();
            out << YAML::Key << "Leaves" << YAML::BeginSeq;

            // get the tilt angle of the stem
            auto stem_direction = normalize(temporary_sorghum_descriptor->stem.spline.segments[0].front);

            glm::vec3 center_point(0,0,0);
            for (const auto& leaf : temporary_sorghum_descriptor->leaves) {
              SorghumSpline leaf_spline;
              leaf_spline.segments = leaf.spline.segments;
              auto stem_part = leaf_spline.GetStemPart();
              auto point = stem_part[0].position;
              point.y = 0;
              center_point += point;
            }

            center_point /= temporary_sorghum_descriptor->leaves.size();
            std::cout << "center point: "<< center_point.x << "," << center_point.y << "," << center_point.z << "," << std::endl;

            for (const auto& leaf : temporary_sorghum_descriptor->leaves) {
              out << YAML::BeginMap;

              SorghumSpline leaf_spline;
              leaf_spline.segments = leaf.spline.segments;

              
              auto segments = leaf_spline.RebuildFixedSizeSegments(64);

              auto stem_part = leaf_spline.GetStemPart();

              auto stem_part_length = stem_part.size();

              auto leaf_part = leaf_spline.GetLeafPart();


              /////////////////////////////////////////
              // extend leaf to the center of the coordinate 
              auto p1 = leaf_part[0].position;
              auto p2 = leaf_part[1].position;
              auto c = center_point;
              auto d = stem_direction;

              auto u = p1 - p2;
              auto w = p2 - c;

              float a = glm::dot(u, u);
              float b = glm::dot(u, d);
              float c2 = glm::dot(d, d);
              float e = glm::dot(u, w);
              float f = glm::dot(d, w);

              float denom = a * c2 - b * b;
              float s = 0.0f, t = 0.0f;

              if (denom > 1e-6f) {  
                s = (b * f - c2 * e) / denom;
                t = (a * f - b * e) / denom;
              }


              glm::vec3 intersection = p2 + s * u;

              SorghumSplineSegment seg;
              seg.position = intersection;
              seg.theta = leaf_part[0].theta;
              seg.up = leaf_part[0].up;
              seg.front = leaf_part[0].front;
              leaf_part.insert(leaf_part.begin(), seg);



              
              for (int j = 0; j < stem_part_length; j++) {
                auto& segment = stem_part[j];
                auto y_target = segment.position.y;

                auto t = (y_target - c.y) / d.y;

                auto p = c + t * d;

                segment.position.x = p.x;
                segment.position.z = p.z;

              }
              ////////////////////////////////

              stem_part.insert(stem_part.end(), leaf_part.begin(), leaf_part.end());

              SorghumSpline spline;
              spline.segments = stem_part;

              segments = spline.RebuildFixedSizeSegments(64);

              std::vector<glm::vec3> points(segments.size());
              std::vector<glm::vec3> left_points(segments.size());
              std::vector<glm::vec3> right_points(segments.size());

              for (uint32_t node_index = 0; node_index < points.size(); node_index++) {
                const auto& segment = segments[node_index];
                points[node_index] = segment.position;
                left_points[node_index] = segment.GetLeafPoint(-segment.theta);
                right_points[node_index] = segment.GetLeafPoint(segment.theta);
              }

              out << YAML::Key << "Leaf Index" << YAML::Value << leaf.index + 1;

              out << YAML::Key << "Center Points" << YAML::Value << YAML::BeginSeq;
              for (const auto& p : points)
                out << YAML::Flow << YAML::BeginSeq << p.x << p.y << p.z << YAML::EndSeq;
              out << YAML::EndSeq;

              out << YAML::Key << "Left Points" << YAML::Value << YAML::BeginSeq;
              for (const auto& p : left_points)
                out << YAML::Flow << YAML::BeginSeq << p.x << p.y << p.z << YAML::EndSeq;
              out << YAML::EndSeq;

              out << YAML::Key << "Right Points" << YAML::Value << YAML::BeginSeq;
              for (const auto& p : right_points)
                out << YAML::Flow << YAML::BeginSeq << p.x << p.y << p.z << YAML::EndSeq;
              out << YAML::EndSeq;
              out << YAML::EndMap;
            }
            out << YAML::EndSeq;
          }
          out << YAML::EndMap;
        }
        
        
        out << YAML::EndSeq;
        out << YAML::EndMap;
        std::ofstream output_file(yaml_path.string());
        output_file << out.c_str();
        output_file.flush();
      } catch (const std::exception& e) {
        EVOENGINE_ERROR("Failed to save!");
      }
    }
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
  capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::OptiX;

  run_windowless(capture_settings->capture_mode, project_path);
  const auto sg_relative_path = std::filesystem::path("SorghumGenerator") / "Random.sg";
  //const auto sg_relative_path = std::filesystem::path("SorghumGenerator") / "Sample0.sorghum";
  const auto output_folder_path = std::filesystem::path("E:/SorghumData");
  //sorghum_field_point_cloud(1, 8, 0.75f, 0, 0, capture_settings, sg_relative_path, output_folder_path);
  sorghum_mesh_point_cloud(500, true, false, capture_settings, sg_relative_path, output_folder_path);


  EVOENGINE_LOG("Generation Finished!")

  // Open File Explorer for generated files.
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  const auto folder_path = output_folder_path.string();
  ShellExecuteA(nullptr, "open", folder_path.c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
#endif
  Application::Terminate();
}
