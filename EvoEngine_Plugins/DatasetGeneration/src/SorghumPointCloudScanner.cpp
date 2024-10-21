#include "SorghumPointCloudScanner.hpp"
#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <CUDAModule.hpp>
#  include <OptiXRayTracer.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "EcoSysLabLayer.hpp"
#include "Sorghum.hpp"
#include "Tinyply.hpp"
#include "TreePointCloudScanner.hpp"
using namespace tinyply;
using namespace digital_agriculture_plugin;
using namespace dataset_generation_plugin;
bool SorghumPointCloudPointSettings::OnInspect() {
  return false;
}

void SorghumPointCloudPointSettings::Save(const std::string& name, YAML::Emitter& out) const {
}

void SorghumPointCloudPointSettings::Load(const std::string& name, const YAML::Node& in) {
}

bool SorghumPointCloudGridCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragInt2("Grid size", &grid_size.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat("Grid distance", &grid_distance, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &step, 0.01f, 0.0f, 0.5f))
    changed = true;
  return changed;
}

void SorghumPointCloudGridCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  const glm::vec2 start_point = glm::vec2((static_cast<float>(grid_size.x) * 0.5f - 0.5f) * grid_distance,
                                          (static_cast<float>(grid_size.y) * 0.5f - 0.5f) * grid_distance);

  const int y_step_size = grid_size.y * grid_distance / step;
  const int x_step_size = grid_size.x * grid_distance / step;

  point_cloud_samples.resize((grid_size.x * y_step_size + grid_size.y * x_step_size) * drone_sample);
  unsigned start_index = 0;
  for (int i = 0; i < grid_size.x; i++) {
    float x = i * grid_distance;
    for (int step = 0; step < y_step_size; step++) {
      float z = step * step;
      const glm::vec3 center = glm::vec3{x, drone_height, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(drone_sample, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[drone_sample * (i * y_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }
  start_index += grid_size.x * y_step_size * drone_sample;
  for (int i = 0; i < grid_size.y; i++) {
    float z = i * grid_distance;
    for (int step = 0; step < x_step_size; step++) {
      float x = step * step;
      const glm::vec3 center = glm::vec3{x, drone_height, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(drone_sample, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[start_index + drone_sample * (i * x_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }
}

bool SorghumPointCloudGridCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  return glm::abs(sample.m_hitInfo.position.x) < bounding_box_size &&
         glm::abs(sample.m_hitInfo.position.z) < bounding_box_size;
}

bool SorghumGantryCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragInt2("Grid size", &grid_size.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat2("Grid distance", &grid_distance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat2("Step", &step.x, 0.00001f, 0.0f, 0.5f))
    changed = true;

  return changed;
}

void SorghumGantryCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  const glm::vec2 start_point = glm::vec2((grid_size.x) * grid_distance.x, (grid_size.y) * grid_distance.y) * 0.5f;
  const int x_step_size = static_cast<int>(grid_size.x * grid_distance.x / step.x);
  const int y_step_size = static_cast<int>(grid_size.y * grid_distance.y / step.y);

  point_cloud_samples.resize(y_step_size * x_step_size * 2 * scanner_angles.size());
  constexpr auto front = glm::vec3(0, -1, 0);
  const float roll_angle = glm::linearRand(0, 360);
  const auto up = glm::vec3(glm::sin(glm::radians(roll_angle)), 0, glm::cos(glm::radians(roll_angle)));
  Jobs::RunParallelFor(y_step_size * x_step_size, [&](unsigned i) {
    const auto x = i / y_step_size;
    const auto y = i % y_step_size;
    const glm::vec3 center = glm::vec3{step.x * x, 0.f, step.y * y} - glm::vec3(start_point.x, 0, start_point.y);
    for (int angle_index = 0; angle_index < scanner_angles.size(); angle_index++) {
      auto& sample1 = point_cloud_samples[i * scanner_angles.size() + angle_index];
      const auto& scanner_angle = scanner_angles[angle_index];
      sample1.direction = glm::normalize(glm::rotate(front, glm::radians(scanner_angle), up));
      sample1.start = center - sample1.direction * (sample_height / glm::cos(glm::radians(scanner_angle)));

      auto& sample2 = point_cloud_samples[y_step_size * x_step_size * scanner_angles.size() +
                                          i * scanner_angles.size() + angle_index];
      sample2.direction = glm::normalize(glm::rotate(front, glm::radians(-scanner_angle), up));
      sample2.start = center - sample2.direction * (sample_height / glm::cos(glm::radians(scanner_angle)));
    }
  });
}

bool SorghumGantryCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  return glm::abs(sample.m_hitInfo.position.x) < bounding_box_size &&
         glm::abs(sample.m_hitInfo.position.z) < bounding_box_size;
}

void SorghumPointCloudScanner::Scan(const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                       std::vector<glm::vec3>& points, std::vector<int>& leaf_indices,
                                       std::vector<int>& instance_indices, std::vector<int>& type_indices) const {
#ifdef OPTIX_RAY_TRACER_PLUGIN
  const auto digital_agriculture_layer = Application::GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Soil> soil;
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  Bound plant_bound{};
  std::unordered_map<Handle, Handle> leaf_mesh_renderer_handles, stem_mesh_renderer_handles,
      panicle_mesh_renderer_handles;
  const auto scene = GetScene();
  const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
  if (sorghum_entities == nullptr) {
    EVOENGINE_ERROR("No sorghums!");
    return;
  }

  for (const auto& sorghum_entity : *sorghum_entities) {
    if (scene->IsEntityValid(sorghum_entity)) {
      scene->ForEachChild(sorghum_entity, [&](const Entity child) {
        if (scene->GetEntityName(child) == "Leaf Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
          const auto leaf_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
          leaf_mesh_renderer_handles.insert({leaf_mesh_renderer->GetHandle(), sorghum_entity.GetIndex()});

          const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
          const auto mesh = leaf_mesh_renderer->mesh.Get<Mesh>();
          plant_bound.min =
              glm::min(plant_bound.min, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plant_bound.max =
              glm::max(plant_bound.max, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        } else if (scene->GetEntityName(child) == "Stem Mesh" && scene->HasPrivateComponent<Particles>(child)) {
          const auto stem_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
          stem_mesh_renderer_handles.insert({stem_mesh_renderer->GetHandle(), sorghum_entity.GetIndex()});

          const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
          const auto mesh = stem_mesh_renderer->mesh.Get<Mesh>();
          plant_bound.min =
              glm::min(plant_bound.min, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plant_bound.max =
              glm::max(plant_bound.max, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        } else if (scene->GetEntityName(child) == "Panicle Strands" &&
                   scene->HasPrivateComponent<StrandsRenderer>(child)) {
          const auto panicle_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
          panicle_mesh_renderer_handles.insert({panicle_mesh_renderer->GetHandle(), sorghum_entity.GetIndex()});

          const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
          const auto mesh = panicle_mesh_renderer->mesh.Get<Mesh>();
          plant_bound.min =
              glm::min(plant_bound.min, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plant_bound.max =
              glm::max(plant_bound.max, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        }
      });
    }
  }

  Handle ground_mesh_renderer_handle = 0;
  if (soil) {
    if (auto soil_entity = soil->GetOwner(); scene->IsEntityValid(soil_entity)) {
      scene->ForEachChild(soil_entity, [&](Entity child) {
        if (scene->GetEntityName(child) == "Ground Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
          ground_mesh_renderer_handle = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->GetHandle();
        }
      });
    }
  }
  std::vector<PointCloudSample> pc_samples;
  capture_settings->GenerateSamples(pc_samples);
  CudaModule::SamplePointCloud(Application::GetLayer<RayTracerLayer>()->environment_properties, pc_samples);

  glm::vec3 left_offset = glm::linearRand(-left_random_offset, left_random_offset);
  glm::vec3 right_offset = glm::linearRand(-right_random_offset, right_random_offset);
  for (int sample_index = 0; sample_index < pc_samples.size(); sample_index++) {
    const auto& sample = pc_samples.at(sample_index);
    if (!sample.m_hit)
      continue;
    if (!capture_settings->SampleFilter(sample))
      continue;
    auto& position = sample.m_hitInfo.position;
    if (position.x < (plant_bound.min.x - sorghum_point_cloud_point_settings.bounding_box_limit) ||
        position.y < (plant_bound.min.y - sorghum_point_cloud_point_settings.bounding_box_limit) ||
        position.z < (plant_bound.min.z - sorghum_point_cloud_point_settings.bounding_box_limit) ||
        position.x > (plant_bound.max.x + sorghum_point_cloud_point_settings.bounding_box_limit) ||
        position.y > (plant_bound.max.y + sorghum_point_cloud_point_settings.bounding_box_limit) ||
        position.z > (plant_bound.max.z + sorghum_point_cloud_point_settings.bounding_box_limit))
      continue;
    auto ball_rand = glm::vec3(0.0f);
    if (sorghum_point_cloud_point_settings.ball_rand_radius > 0.0f) {
      ball_rand = glm::ballRand(sorghum_point_cloud_point_settings.ball_rand_radius);
    }
    const auto distance = glm::distance(sample.m_hitInfo.position, sample.start);

    points.emplace_back(sample.m_hitInfo.position +
                        distance * glm::vec3(glm::gaussRand(0.0f, sorghum_point_cloud_point_settings.variance),
                                             glm::gaussRand(0.0f, sorghum_point_cloud_point_settings.variance),
                                             glm::gaussRand(0.0f, sorghum_point_cloud_point_settings.variance)) +
                        ball_rand + (sample_index >= pc_samples.size() / 2 ? left_offset : right_offset));

    if (sorghum_point_cloud_point_settings.leaf_index) {
      leaf_indices.emplace_back(glm::floatBitsToUint(sample.m_hitInfo.data.x));
    }

    auto leaf_search = leaf_mesh_renderer_handles.find(sample.m_handle);
    auto stem_search = stem_mesh_renderer_handles.find(sample.m_handle);
    auto panicle_search = panicle_mesh_renderer_handles.find(sample.m_handle);
    if (sorghum_point_cloud_point_settings.instance_index) {
      if (leaf_search != leaf_mesh_renderer_handles.end()) {
        instance_indices.emplace_back(leaf_search->second);
      } else if (stem_search != stem_mesh_renderer_handles.end()) {
        instance_indices.emplace_back(stem_search->second);
      } else if (panicle_search != panicle_mesh_renderer_handles.end()) {
        instance_indices.emplace_back(panicle_search->second);
      } else {
        instance_indices.emplace_back(0);
      }
    }

    if (sorghum_point_cloud_point_settings.type_index) {
      if (leaf_search != leaf_mesh_renderer_handles.end()) {
        type_indices.emplace_back(0);
      } else if (stem_search != stem_mesh_renderer_handles.end()) {
        type_indices.emplace_back(1);
      } else if (panicle_search != panicle_mesh_renderer_handles.end()) {
        type_indices.emplace_back(2);
      } else if (sample.m_handle == ground_mesh_renderer_handle) {
        type_indices.emplace_back(3);
      } else {
        type_indices.emplace_back(-1);
      }
    }
  }
#endif
}

void SorghumPointCloudScanner::SavePointCloud(const std::filesystem::path& save_path,
                                              const std::vector<glm::vec3>& points,
                                              const std::vector<int>& leaf_indices,
                                              const std::vector<int>& instance_indices,
                                              const std::vector<int>& type_indices) const {
  std::filebuf fb_binary;
  fb_binary.open(save_path.string(), std::ios::out | std::ios::binary);
  std::ostream ostream(&fb_binary);
  if (ostream.fail())
    throw std::runtime_error("failed to open " + save_path.string());

  PlyFile cube_file;
  cube_file.add_properties_to_element("vertex", {"x", "y", "z"}, Type::FLOAT32, points.size(),
                                      static_cast<const uint8_t*>(static_cast<const void*>(points.data())),
                                      Type::INVALID, 0);

  if (sorghum_point_cloud_point_settings.type_index)
    cube_file.add_properties_to_element("type_index", {"type_index"}, Type::INT32, type_indices.size(),
                                        static_cast<const uint8_t*>(static_cast<const void*>(type_indices.data())), Type::INVALID, 0);

  if (sorghum_point_cloud_point_settings.instance_index) {
    cube_file.add_properties_to_element("instance_index", {"instance_index"}, Type::INT32, instance_indices.size(),
                                        static_cast<const uint8_t*>(static_cast<const void*>(instance_indices.data())), Type::INVALID, 0);
  }

  if (sorghum_point_cloud_point_settings.leaf_index) {
    cube_file.add_properties_to_element("leaf_index", {"leaf_index"}, Type::INT32, leaf_indices.size(),
                                        static_cast<const uint8_t*>(static_cast<const void*>(leaf_indices.data())), Type::INVALID, 0);
  }
  // Write a binary file
  cube_file.write(ostream, true);
}

void SorghumPointCloudScanner::WriteSplineInfo(const std::filesystem::path& save_path,
                                               const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
  if (sorghum_entities == nullptr) {
    EVOENGINE_ERROR("No sorghums!");
    return;
  }
  try {
    std::filesystem::path yaml_path = save_path;
    yaml_path.replace_extension(".yml");
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "Sorghums" << YAML::BeginSeq;
    for (const auto& sorghum_entity : *sorghum_entities) {
      if (scene->IsEntityValid(sorghum_entity)) {
        if (capture_settings->output_spline_info) {
          const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
          const auto sorghum_descriptor = sorghum->sorghum_descriptor.Get<SorghumDescriptor>();
          out << YAML::BeginMap;
          {
            out << YAML::Key << "Instance Index" << YAML::Value << sorghum_entity.GetIndex();
            out << YAML::Key << "Leaves" << YAML::BeginSeq;
            for (const auto& leaf : sorghum_descriptor->leaves) {
              out << YAML::BeginMap;
              std::vector<glm::vec3> points(capture_settings->spline_subdivision_count);
              SorghumSpline leaf_part;
              leaf_part.segments = leaf.spline.GetLeafPart();
              const auto segments = leaf_part.RebuildFixedSizeSegments(capture_settings->spline_subdivision_count);
              for (uint32_t node_index = 0; node_index < capture_settings->spline_subdivision_count; node_index++) {
                points[node_index] = segments[node_index].position;
              }
              out << YAML::Key << "Leaf Index" << YAML::Value << leaf.index + 1;
              Serialization::SerializeVector("Center Points", points, out);
              out << YAML::EndMap;
            }
            out << YAML::EndSeq;
          }
          out << YAML::EndMap;
        }
      }
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

void SorghumPointCloudScanner::Capture(const std::filesystem::path& save_path,
                                       const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const {
#ifdef OPTIX_RAY_TRACER_PLUGIN
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
  if (sorghum_entities == nullptr) {
    EVOENGINE_ERROR("No sorghums!");
    return;
  }
  std::vector<glm::vec3> points;
  std::vector<int> leaf_indices;
  std::vector<int> instance_indices;
  std::vector<int> type_indices;

  Scan(capture_settings, points, leaf_indices, instance_indices, type_indices);
  SavePointCloud(save_path, points, leaf_indices, instance_indices, type_indices);
  
  if (capture_settings->output_spline_info) {
    WriteSplineInfo(save_path, capture_settings);
  }
#endif
}

bool SorghumPointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Grid Capture")) {
    static std::shared_ptr<TreePointCloudGridCaptureSettings> capture_settings =
        std::make_shared<TreePointCloudGridCaptureSettings>();
    capture_settings->OnInspect();
    FileUtils::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          Capture(path, capture_settings);
        },
        false);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Point settings")) {
    if (sorghum_point_cloud_point_settings.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

void SorghumPointCloudScanner::OnDestroy() {
  sorghum_point_cloud_point_settings = {};
}

void SorghumPointCloudScanner::Serialize(YAML::Emitter& out) const {
  sorghum_point_cloud_point_settings.Save("sorghum_point_cloud_point_settings", out);
}

void SorghumPointCloudScanner::Deserialize(const YAML::Node& in) {
  sorghum_point_cloud_point_settings.Load("sorghum_point_cloud_point_settings", in);
}
