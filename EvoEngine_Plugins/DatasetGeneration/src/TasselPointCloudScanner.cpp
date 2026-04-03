#include "TasselPointCloudScanner.hpp"

#include "CpuRayTracer.hpp"
#include "MaizeTassel.hpp"
#include "Particles.hpp"
#include "PointCloud.hpp"
#include "Tinyply.hpp"

using namespace l_system_plugin;
using namespace dataset_generation_plugin;

// ---------------------------------------------------------------------------
// TasselPointCloudPointSettings
// ---------------------------------------------------------------------------

bool TasselPointCloudPointSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat("Variance", &variance, 0.001f, 0.0f, 0.1f))
    changed = true;
  if (ImGui::DragFloat("Ball Rand Radius", &ball_rand_radius, 0.001f, 0.0f, 0.1f))
    changed = true;
  if (ImGui::Checkbox("Type Index", &type_index))
    changed = true;
  if (ImGui::Checkbox("Instance Index", &instance_index))
    changed = true;
  if (ImGui::DragFloat("Bounding Box Limit", &bounding_box_limit, 0.1f, 0.0f, 10.0f))
    changed = true;
  return changed;
}

void TasselPointCloudPointSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;
  out << YAML::Key << "variance" << YAML::Value << variance;
  out << YAML::Key << "ball_rand_radius" << YAML::Value << ball_rand_radius;
  out << YAML::Key << "type_index" << YAML::Value << type_index;
  out << YAML::Key << "instance_index" << YAML::Value << instance_index;
  out << YAML::Key << "bounding_box_limit" << YAML::Value << bounding_box_limit;
  out << YAML::EndMap;
}

void TasselPointCloudPointSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& n = in[name];
    if (n["variance"]) variance = n["variance"].as<float>();
    if (n["ball_rand_radius"]) ball_rand_radius = n["ball_rand_radius"].as<float>();
    if (n["type_index"]) type_index = n["type_index"].as<bool>();
    if (n["instance_index"]) instance_index = n["instance_index"].as<bool>();
    if (n["bounding_box_limit"]) bounding_box_limit = n["bounding_box_limit"].as<float>();
  }
}

// ---------------------------------------------------------------------------
// TasselPointCloudGridCaptureSettings
// ---------------------------------------------------------------------------

bool TasselPointCloudGridCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragInt2("Grid size", &grid_size.x, 1, 1, 100))
    changed = true;
  if (ImGui::DragFloat("Grid distance", &grid_distance, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &step, 0.001f, 0.0f, 0.5f))
    changed = true;
  if (ImGui::DragInt("Samples per step", &samples_per_step, 1, 1, 4096))
    changed = true;
  if (ImGui::DragFloat("Sample height", &sample_height, 0.1f, 0.0f, 10.0f))
    changed = true;
  return changed;
}

void TasselPointCloudGridCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  // Hemisphere sampling from above — suitable for small tassel objects.
  const glm::vec2 start_point = glm::vec2((static_cast<float>(grid_size.x) * 0.5f - 0.5f) * grid_distance,
                                          (static_cast<float>(grid_size.y) * 0.5f - 0.5f) * grid_distance);

  const int x_step_size = static_cast<int>(grid_size.x * grid_distance / step);
  const int y_step_size = static_cast<int>(grid_size.y * grid_distance / step);

  point_cloud_samples.resize(x_step_size * y_step_size * samples_per_step);

  Jobs::RunParallelFor(x_step_size * y_step_size, [&](unsigned i) {
    const auto x = static_cast<int>(i) / y_step_size;
    const auto y = static_cast<int>(i) % y_step_size;
    const glm::vec3 center =
        glm::vec3{step * x, sample_height, step * y} - glm::vec3(start_point.x, 0, start_point.y);
    for (int s = 0; s < samples_per_step; s++) {
      auto& sample = point_cloud_samples[i * samples_per_step + s];
      sample.direction = glm::sphericalRand(1.0f);
      sample.direction.y = -glm::abs(sample.direction.y);
      sample.start = center;
    }
  });
}

bool TasselPointCloudGridCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  return glm::abs(sample.hit_info.position.x) < bounding_box_size &&
         glm::abs(sample.hit_info.position.y) < bounding_box_size &&
         glm::abs(sample.hit_info.position.z) < bounding_box_size;
}

// ---------------------------------------------------------------------------
// TasselPointCloudScanner
// ---------------------------------------------------------------------------

void TasselPointCloudScanner::Scan(const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                   std::vector<glm::vec3>& points, std::vector<int>& instance_indices,
                                   std::vector<int>& type_indices) const {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  const auto scene = GetScene();

  Bound plant_bound{};

  // Handle maps: handle -> instance entity index
  std::unordered_map<Handle, Handle> stem_mesh_renderer_handles;
  std::unordered_map<Handle, Handle> spikelet_particle_handles;

  const std::vector<Entity>* tassel_entities = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
  if (tassel_entities == nullptr) {
    EVOENGINE_ERROR("No MaizeTassel entities found!");
    return;
  }

  for (const auto& tassel_entity : *tassel_entities) {
    if (!scene->IsEntityValid(tassel_entity))
      continue;
    scene->ForEachChild(tassel_entity, [&](const Entity child) {
      const auto name = scene->GetEntityName(child);
      if (name == "Tassel Stem Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
        stem_mesh_renderer_handles.insert({mesh_renderer->GetHandle(), tassel_entity.GetIndex()});

        const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
        const auto mesh = mesh_renderer->mesh.Get<Mesh>();
        if (mesh) {
          plant_bound.min =
              glm::min(plant_bound.min, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plant_bound.max =
              glm::max(plant_bound.max, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        }
      } else if (name == "Tassel Spikelets" && scene->HasPrivateComponent<Particles>(child)) {
        // Particles are rendered as instanced meshes; they have a MeshRenderer-like handle for ray tracing.
        const auto particles = scene->GetOrSetPrivateComponent<Particles>(child).lock();
        spikelet_particle_handles.insert({particles->GetHandle(), tassel_entity.GetIndex()});

        // Expand bounds to cover particle positions.
        const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (particle_info_list) {
          const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
          const auto& infos = particle_info_list->PeekParticleInfoList();
          for (const auto& info : infos) {
            glm::vec3 pos = glm::vec3(global_transform.value * info.instance_matrix.value * glm::vec4(0, 0, 0, 1));
            plant_bound.min = glm::min(plant_bound.min, pos - glm::vec3(0.1f));
            plant_bound.max = glm::max(plant_bound.max, pos + glm::vec3(0.1f));
          }
        }
      }
    });
  }

  std::vector<PointCloudSample> pc_samples;
  capture_settings->GenerateSamples(pc_samples);

  switch (capture_settings->capture_mode) {
    case PointCloudCaptureSettings::CaptureMode::Cpu: {
      std::shared_ptr<RenderInstanceStorage> render_instances{};
      if (render_layer) {
        render_instances = render_layer->GetCurrentRenderInstanceStorage();
      }
      if (!render_instances) {
        render_instances = std::make_shared<RenderInstanceStorage>();
        Bound world_bound;
        render_instances->BuildFromScene({}, Application::GetActiveScene(), world_bound);
      }
      CpuRayTracer cpu_ray_tracer;
      cpu_ray_tracer.Initialize(
          render_instances,
          [&](uint32_t, const std::shared_ptr<Mesh>&) {},
          [&](const uint32_t, const Entity&) {});
      cpu_ray_tracer.SamplePointCloud(pc_samples);
    } break;
    case PointCloudCaptureSettings::CaptureMode::Gpu: {
      PointCloud::SampleCurrentScene(pc_samples);
    } break;
  }

  for (size_t sample_index = 0; sample_index < pc_samples.size(); sample_index++) {
    const auto& sample = pc_samples[sample_index];
    if (sample.hit_count == 0)
      continue;
    if (!capture_settings->SampleFilter(sample))
      continue;

    auto& position = sample.hit_info.position;
    if (position.x < (plant_bound.min.x - tassel_point_cloud_point_settings.bounding_box_limit) ||
        position.y < (plant_bound.min.y - tassel_point_cloud_point_settings.bounding_box_limit) ||
        position.z < (plant_bound.min.z - tassel_point_cloud_point_settings.bounding_box_limit) ||
        position.x > (plant_bound.max.x + tassel_point_cloud_point_settings.bounding_box_limit) ||
        position.y > (plant_bound.max.y + tassel_point_cloud_point_settings.bounding_box_limit) ||
        position.z > (plant_bound.max.z + tassel_point_cloud_point_settings.bounding_box_limit))
      continue;

    auto ball_rand = glm::vec3(0.0f);
    if (tassel_point_cloud_point_settings.ball_rand_radius > 0.0f) {
      ball_rand = glm::ballRand(tassel_point_cloud_point_settings.ball_rand_radius);
    }
    const auto distance = glm::distance(sample.hit_info.position, sample.start);

    points.emplace_back(sample.hit_info.position +
                        distance * glm::vec3(glm::gaussRand(0.0f, tassel_point_cloud_point_settings.variance),
                                             glm::gaussRand(0.0f, tassel_point_cloud_point_settings.variance),
                                             glm::gaussRand(0.0f, tassel_point_cloud_point_settings.variance)) +
                        ball_rand);

    // Instance index
    auto stem_search = stem_mesh_renderer_handles.find(sample.handle);
    auto spikelet_search = spikelet_particle_handles.find(sample.handle);

    if (tassel_point_cloud_point_settings.instance_index) {
      if (stem_search != stem_mesh_renderer_handles.end()) {
        instance_indices.emplace_back(stem_search->second);
      } else if (spikelet_search != spikelet_particle_handles.end()) {
        instance_indices.emplace_back(spikelet_search->second);
      } else {
        instance_indices.emplace_back(0);
      }
    }

    // Type index: 0 = stem, 1 = spikelet, -1 = unknown
    if (tassel_point_cloud_point_settings.type_index) {
      if (stem_search != stem_mesh_renderer_handles.end()) {
        type_indices.emplace_back(0);
      } else if (spikelet_search != spikelet_particle_handles.end()) {
        type_indices.emplace_back(1);
      } else {
        type_indices.emplace_back(-1);
      }
    }
  }
}

void TasselPointCloudScanner::SavePointCloud(const std::filesystem::path& save_path,
                                             const std::vector<glm::vec3>& points,
                                             const std::vector<int>& instance_indices,
                                             const std::vector<int>& type_indices) const {
  std::filebuf fb_binary;
  fb_binary.open(save_path.string(), std::ios::out | std::ios::binary);
  std::ostream ostream(&fb_binary);
  if (ostream.fail())
    throw std::runtime_error("failed to open " + save_path.string());

  tinyply::PlyFile ply_file;
  ply_file.add_properties_to_element("vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, points.size(),
                                     static_cast<const uint8_t*>(static_cast<const void*>(points.data())),
                                     tinyply::Type::INVALID, 0);

  if (tassel_point_cloud_point_settings.type_index)
    ply_file.add_properties_to_element("type_index", {"type_index"}, tinyply::Type::INT32, type_indices.size(),
                                       static_cast<const uint8_t*>(static_cast<const void*>(type_indices.data())),
                                       tinyply::Type::INVALID, 0);

  if (tassel_point_cloud_point_settings.instance_index)
    ply_file.add_properties_to_element(
        "instance_index", {"instance_index"}, tinyply::Type::INT32, instance_indices.size(),
        static_cast<const uint8_t*>(static_cast<const void*>(instance_indices.data())), tinyply::Type::INVALID, 0);

  ply_file.write(ostream, true);
}

void TasselPointCloudScanner::Capture(const std::filesystem::path& save_path,
                                      const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* tassel_entities = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
  if (tassel_entities == nullptr) {
    EVOENGINE_ERROR("No MaizeTassel entities found!");
    return;
  }
  std::vector<glm::vec3> points;
  std::vector<int> instance_indices;
  std::vector<int> type_indices;

  Scan(capture_settings, points, instance_indices, type_indices);
  SavePointCloud(save_path, points, instance_indices, type_indices);
}

bool TasselPointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Grid Capture")) {
    static std::shared_ptr<TasselPointCloudGridCaptureSettings> capture_settings =
        std::make_shared<TasselPointCloudGridCaptureSettings>();
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
    if (tassel_point_cloud_point_settings.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

void TasselPointCloudScanner::OnDestroy() {
  tassel_point_cloud_point_settings = {};
}

void TasselPointCloudScanner::Serialize(YAML::Emitter& out) const {
  tassel_point_cloud_point_settings.Save("tassel_point_cloud_point_settings", out);
}

void TasselPointCloudScanner::Deserialize(const YAML::Node& in) {
  tassel_point_cloud_point_settings.Load("tassel_point_cloud_point_settings", in);
}
