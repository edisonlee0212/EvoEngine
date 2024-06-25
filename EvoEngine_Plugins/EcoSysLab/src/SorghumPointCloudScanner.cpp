#include "SorghumPointCloudScanner.hpp"
#ifdef BUILD_WITH_RAYTRACER
#  include <CUDAModule.hpp>
#  include <RayTracer.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "EcoSysLabLayer.hpp"
#include "Sorghum.hpp"
#include "Tinyply.hpp"
#include "TreePointCloudScanner.hpp"
using namespace tinyply;
using namespace eco_sys_lab;

bool SorghumPointCloudPointSettings::OnInspect() {
  return false;
}

void SorghumPointCloudPointSettings::Save(const std::string& name, YAML::Emitter& out) const {
}

void SorghumPointCloudPointSettings::Load(const std::string& name, const YAML::Node& in) {
}

bool SorghumPointCloudGridCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragInt2("Grid size", &m_gridSize.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat("Grid distance", &m_gridDistance, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &m_step, 0.01f, 0.0f, 0.5f))
    changed = true;
  return changed;
}

void SorghumPointCloudGridCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  const glm::vec2 start_point = glm::vec2((static_cast<float>(m_gridSize.x) * 0.5f - 0.5f) * m_gridDistance,
                                         (static_cast<float>(m_gridSize.y) * 0.5f - 0.5f) * m_gridDistance);

  const int y_step_size = m_gridSize.y * m_gridDistance / m_step;
  const int x_step_size = m_gridSize.x * m_gridDistance / m_step;

  point_cloud_samples.resize((m_gridSize.x * y_step_size + m_gridSize.y * x_step_size) * m_droneSample);
  unsigned start_index = 0;
  for (int i = 0; i < m_gridSize.x; i++) {
    float x = i * m_gridDistance;
    for (int step = 0; step < y_step_size; step++) {
      float z = step * m_step;
      const glm::vec3 center = glm::vec3{x, m_droneHeight, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(m_droneSample, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[m_droneSample * (i * y_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }
  start_index += m_gridSize.x * y_step_size * m_droneSample;
  for (int i = 0; i < m_gridSize.y; i++) {
    float z = i * m_gridDistance;
    for (int step = 0; step < x_step_size; step++) {
      float x = step * m_step;
      const glm::vec3 center = glm::vec3{x, m_droneHeight, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(m_droneSample, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[start_index + m_droneSample * (i * x_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }
}

bool SorghumPointCloudGridCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  return glm::abs(sample.m_hitInfo.position.x) < m_boundingBoxSize &&
         glm::abs(sample.m_hitInfo.position.z) < m_boundingBoxSize;
}

bool SorghumGantryCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragInt2("Grid size", &m_gridSize.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat2("Grid distance", &m_gridDistance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat2("Step", &m_step.x, 0.00001f, 0.0f, 0.5f))
    changed = true;

  return changed;
}

void SorghumGantryCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  const glm::vec2 start_point = glm::vec2((m_gridSize.x) * m_gridDistance.x, (m_gridSize.y) * m_gridDistance.y) * 0.5f;
  const int x_step_size = static_cast<int>(m_gridSize.x * m_gridDistance.x / m_step.x);
  const int y_step_size = static_cast<int>(m_gridSize.y * m_gridDistance.y / m_step.y);

  point_cloud_samples.resize(y_step_size * x_step_size * 2);
  constexpr auto front = glm::vec3(0, -1, 0);
  const float roll_angle = glm::linearRand(0, 360);
  const auto up = glm::vec3(glm::sin(glm::radians(roll_angle)), 0, glm::cos(glm::radians(roll_angle)));
  Jobs::RunParallelFor(y_step_size * x_step_size, [&](unsigned i) {
    const auto x = i / y_step_size;
    const auto y = i % y_step_size;
    const glm::vec3 center = glm::vec3{m_step.x * x, 0.f, m_step.y * y} - glm::vec3(start_point.x, 0, start_point.y);

    auto& sample1 = point_cloud_samples[i];
    sample1.direction = glm::normalize(glm::rotate(front, glm::radians(m_scannerAngle), up));
    sample1.start = center - sample1.direction * (m_sampleHeight / glm::cos(glm::radians(m_scannerAngle)));

    auto& sample2 = point_cloud_samples[y_step_size * x_step_size + i];
    sample2.direction = glm::normalize(glm::rotate(front, glm::radians(-m_scannerAngle), up));
    sample2.start = center - sample2.direction * (m_sampleHeight / glm::cos(glm::radians(m_scannerAngle)));
  });
}

bool SorghumGantryCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  return glm::abs(sample.m_hitInfo.position.x) < m_boundingBoxSize &&
         glm::abs(sample.m_hitInfo.position.z) < m_boundingBoxSize;
}

void SorghumPointCloudScanner::Capture(const std::filesystem::path& save_path,
                                       const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const {
#ifdef BUILD_WITH_RAYTRACER
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Soil> soil;
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  if (!soil) {
    EVOENGINE_ERROR("No soil!");
    return;
  }
  Bound plant_bound{};
  std::unordered_map<Handle, Handle> leaf_mesh_renderer_handles, stem_mesh_renderer_handles, panicle_mesh_renderer_handles;
  const auto scene = GetScene();
  const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
  if (sorghum_entities == nullptr) {
    EVOENGINE_ERROR("No sorghums!");
    return;
  }
  for (const auto& sorghum_entity : *sorghum_entities) {
    if (scene->IsEntityValid(sorghum_entity)) {
      scene->ForEachChild(sorghum_entity, [&](Entity child) {
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
  if (auto soil_entity = soil->GetOwner(); scene->IsEntityValid(soil_entity)) {
    scene->ForEachChild(soil_entity, [&](Entity child) {
      if (scene->GetEntityName(child) == "Ground Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
        ground_mesh_renderer_handle = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->GetHandle();
      }
    });
  }

  std::vector<PointCloudSample> pc_samples;
  capture_settings->GenerateSamples(pc_samples);
  CudaModule::SamplePointCloud(Application::GetLayer<RayTracerLayer>()->environment_properties, pc_samples);

  std::vector<glm::vec3> points;
  std::vector<int> leaf_index;
  std::vector<int> instance_index;
  std::vector<int> type_index;
  glm::vec3 left_offset = glm::linearRand(-m_leftRandomOffset, m_leftRandomOffset);
  glm::vec3 right_offset = glm::linearRand(-m_rightRandomOffset, m_rightRandomOffset);
  for (int sample_index = 0; sample_index < pc_samples.size(); sample_index++) {
    const auto& sample = pc_samples.at(sample_index);
    if (!sample.m_hit)
      continue;
    if (!capture_settings->SampleFilter(sample))
      continue;
    auto& position = sample.m_hitInfo.position;
    if (position.x < (plant_bound.min.x - m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
        position.y < (plant_bound.min.y - m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
        position.z < (plant_bound.min.z - m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
        position.x > (plant_bound.max.x + m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
        position.y > (plant_bound.max.y + m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
        position.z > (plant_bound.max.z + m_sorghumPointCloudPointSettings.m_boundingBoxLimit))
      continue;
    auto ball_rand = glm::vec3(0.0f);
    if (m_sorghumPointCloudPointSettings.m_ballRandRadius > 0.0f) {
      ball_rand = glm::ballRand(m_sorghumPointCloudPointSettings.m_ballRandRadius);
    }
    const auto distance = glm::distance(sample.m_hitInfo.position, sample.m_start);

    points.emplace_back(sample.m_hitInfo.position +
                        distance * glm::vec3(glm::gaussRand(0.0f, m_sorghumPointCloudPointSettings.m_variance),
                                             glm::gaussRand(0.0f, m_sorghumPointCloudPointSettings.m_variance),
                                             glm::gaussRand(0.0f, m_sorghumPointCloudPointSettings.m_variance)) +
                        ball_rand + (sample_index >= pc_samples.size() / 2 ? left_offset : right_offset));

    if (m_sorghumPointCloudPointSettings.m_leafIndex) {
      leaf_index.emplace_back(static_cast<int>(sample.m_hitInfo.data.x + 0.1f));
    }

    auto leaf_search = leaf_mesh_renderer_handles.find(sample.m_handle);
    auto stem_search = stem_mesh_renderer_handles.find(sample.m_handle);
    auto panicle_search = panicle_mesh_renderer_handles.find(sample.m_handle);
    if (m_sorghumPointCloudPointSettings.m_instanceIndex) {
      if (leaf_search != leaf_mesh_renderer_handles.end()) {
        instance_index.emplace_back(leaf_search->second);
      } else if (stem_search != stem_mesh_renderer_handles.end()) {
        instance_index.emplace_back(stem_search->second);
      } else if (panicle_search != panicle_mesh_renderer_handles.end()) {
        instance_index.emplace_back(panicle_search->second);
      } else {
        instance_index.emplace_back(0);
      }
    }

    if (m_sorghumPointCloudPointSettings.m_typeIndex) {
      if (leaf_search != leaf_mesh_renderer_handles.end()) {
        type_index.emplace_back(0);
      } else if (stem_search != stem_mesh_renderer_handles.end()) {
        type_index.emplace_back(1);
      } else if (panicle_search != panicle_mesh_renderer_handles.end()) {
        type_index.emplace_back(2);
      } else if (sample.m_handle == ground_mesh_renderer_handle) {
        type_index.emplace_back(3);
      } else {
        type_index.emplace_back(-1);
      }
    }
  }
  std::filebuf fb_binary;
  fb_binary.open(save_path.string(), std::ios::out | std::ios::binary);
  std::ostream ostream(&fb_binary);
  if (ostream.fail())
    throw std::runtime_error("failed to open " + save_path.string());

  PlyFile cube_file;
  cube_file.add_properties_to_element("vertex", {"x", "y", "z"}, Type::FLOAT32, points.size(),
                                      reinterpret_cast<uint8_t*>(points.data()), Type::INVALID, 0);

  if (m_sorghumPointCloudPointSettings.m_typeIndex)
    cube_file.add_properties_to_element("type_index", {"type_index"}, Type::INT32, type_index.size(),
                                        reinterpret_cast<uint8_t*>(type_index.data()), Type::INVALID, 0);

  if (m_sorghumPointCloudPointSettings.m_instanceIndex) {
    cube_file.add_properties_to_element("instance_index", {"instance_index"}, Type::INT32, instance_index.size(),
                                        reinterpret_cast<uint8_t*>(instance_index.data()), Type::INVALID, 0);
  }

  if (m_sorghumPointCloudPointSettings.m_leafIndex) {
    cube_file.add_properties_to_element("leaf_index", {"leaf_index"}, Type::INT32, leaf_index.size(),
                                        reinterpret_cast<uint8_t*>(leaf_index.data()), Type::INVALID, 0);
  }

  // Write a binary file
  cube_file.write(ostream, true);
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
    if (m_sorghumPointCloudPointSettings.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

void SorghumPointCloudScanner::OnDestroy() {
  m_sorghumPointCloudPointSettings = {};
}

void SorghumPointCloudScanner::Serialize(YAML::Emitter& out) const {
  m_sorghumPointCloudPointSettings.Save("m_sorghumPointCloudPointSettings", out);
}

void SorghumPointCloudScanner::Deserialize(const YAML::Node& in) {
  m_sorghumPointCloudPointSettings.Load("m_sorghumPointCloudPointSettings", in);
}
