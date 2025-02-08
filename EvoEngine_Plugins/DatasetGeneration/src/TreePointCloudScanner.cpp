#include "TreePointCloudScanner.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include <CUDAModule.hpp>
#  include <OptiXRayTracer.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "CpuRayTracer.hpp"
#include "EcoSysLabLayer.hpp"
#include "Soil.hpp"
#include "Tinyply.hpp"
using namespace eco_sys_lab_plugin;
using namespace dataset_generation_plugin;
#pragma region Settings
void TreePointCloudPointSettings::OnInspect() {
  ImGui::DragFloat("Point variance", &variance, 0.01f);
  ImGui::DragFloat("Point uniform random radius", &ball_rand_radius, 0.01f);
  ImGui::DragFloat("Bounding box offset", &bounding_box_limit, 0.01f);
  ImGui::Checkbox("Type Index", &type_index);
  ImGui::Checkbox("Instance Index", &instance_index);
  ImGui::Checkbox("Branch Index", &branch_index);
  ImGui::Checkbox("Tree Part Index", &tree_part_index);
  ImGui::Checkbox("Line Index", &line_index);
  ImGui::Checkbox("Internode Index", &internode_index);
}

void TreePointCloudPointSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "variance" << YAML::Value << variance;
  out << YAML::Key << "ball_rand_radius" << YAML::Value << ball_rand_radius;
  out << YAML::Key << "type_index" << YAML::Value << type_index;
  out << YAML::Key << "instance_index" << YAML::Value << instance_index;
  out << YAML::Key << "branch_index" << YAML::Value << branch_index;
  out << YAML::Key << "tree_part_index" << YAML::Value << tree_part_index;
  out << YAML::Key << "line_index" << YAML::Value << line_index;
  out << YAML::Key << "internode_index" << YAML::Value << internode_index;
  out << YAML::Key << "bounding_box_limit" << YAML::Value << bounding_box_limit;
  out << YAML::EndMap;
}

void TreePointCloudPointSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    auto& cd = in[name];
    if (cd["variance"])
      variance = cd["variance"].as<float>();
    if (cd["ball_rand_radius"])
      ball_rand_radius = cd["ball_rand_radius"].as<float>();
    if (cd["type_index"])
      type_index = cd["type_index"].as<bool>();
    if (cd["instance_index"])
      instance_index = cd["instance_index"].as<bool>();
    if (cd["branch_index"])
      branch_index = cd["branch_index"].as<bool>();
    if (cd["tree_part_index"])
      tree_part_index = cd["tree_part_index"].as<bool>();
    if (cd["line_index"])
      line_index = cd["line_index"].as<bool>();
    if (cd["internode_index"])
      internode_index = cd["internode_index"].as<bool>();
    if (cd["bounding_box_limit"])
      bounding_box_limit = cd["bounding_box_limit"].as<float>();
  }
}

bool TreePointCloudCircularCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat("Distance to focus point", &distance_from_trees, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Height to ground", &capture_height, 0.01f))
    changed = true;
  ImGui::Separator();
  ImGui::Text("Rotation:");
  if (ImGui::DragInt3("Pitch Angle Start/Step/End", &pitch_angle_start, 1))
    changed = true;
  if (ImGui::DragInt3("Turn Angle Start/Step/End", &turn_angle_start, 1))
    changed = true;
  ImGui::Separator();
  ImGui::Text("Camera Settings:");
  if (ImGui::DragFloat("FOV", &camera_fov))
    changed = true;
  if (ImGui::DragInt("Resolution", &scan_resolution))
    changed = true;
  if (ImGui::DragFloat("Max Depth", &max_capture_depth))
    changed = true;
  return changed;
}

void TreePointCloudCircularCaptureSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;

  out << YAML::Key << "pitch_angle_start" << YAML::Value << pitch_angle_start;
  out << YAML::Key << "pitch_angle_step" << YAML::Value << pitch_angle_step;
  out << YAML::Key << "pitch_angle_end" << YAML::Value << pitch_angle_end;
  out << YAML::Key << "turn_angle_start" << YAML::Value << turn_angle_start;
  out << YAML::Key << "turn_angle_step" << YAML::Value << turn_angle_step;
  out << YAML::Key << "turn_angle_end" << YAML::Value << turn_angle_end;
  out << YAML::Key << "distance_from_trees" << YAML::Value << distance_from_trees;
  out << YAML::Key << "capture_height" << YAML::Value << capture_height;
  out << YAML::Key << "camera_fov" << YAML::Value << camera_fov;
  out << YAML::Key << "resolution_" << YAML::Value << scan_resolution;
  out << YAML::Key << "max_capture_depth" << YAML::Value << max_capture_depth;
  out << YAML::EndMap;
}

void TreePointCloudCircularCaptureSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    auto& cd = in[name];

    if (cd["pitch_angle_start"])
      pitch_angle_start = cd["pitch_angle_start"].as<int>();
    if (cd["pitch_angle_step"])
      pitch_angle_step = cd["pitch_angle_step"].as<int>();
    if (cd["pitch_angle_end"])
      pitch_angle_end = cd["pitch_angle_end"].as<int>();
    if (cd["turn_angle_start"])
      turn_angle_start = cd["turn_angle_start"].as<int>();
    if (cd["turn_angle_step"])
      turn_angle_step = cd["turn_angle_step"].as<int>();
    if (cd["turn_angle_end"])
      turn_angle_end = cd["turn_angle_end"].as<int>();
    if (cd["distance_from_trees"])
      distance_from_trees = cd["distance_from_trees"].as<float>();
    if (cd["capture_height"])
      capture_height = cd["capture_height"].as<float>();
    if (cd["camera_fov"])
      camera_fov = cd["camera_fov"].as<float>();
    if (cd["resolution_"])
      scan_resolution = cd["resolution_"].as<int>();
    if (cd["max_capture_depth"])
      max_capture_depth = cd["max_capture_depth"].as<float>();
  }
}

GlobalTransform TreePointCloudCircularCaptureSettings::GetTransform(const glm::vec2& focus_point,
                                                                    const float turn_angle,
                                                                    const float pitch_angle) const {
  GlobalTransform camera_global_transform;
  const glm::vec3 camera_position = glm::vec3(glm::sin(glm::radians(turn_angle)) * distance_from_trees, capture_height,
                                              glm::cos(glm::radians(turn_angle)) * distance_from_trees);
  const glm::vec3 camera_direction = glm::vec3(glm::sin(glm::radians(turn_angle)) * distance_from_trees,
                                               distance_from_trees * glm::sin(glm::radians(pitch_angle)),
                                               glm::cos(glm::radians(turn_angle)) * distance_from_trees);
  camera_global_transform.SetPosition(camera_position + glm::vec3(focus_point.x, 0, focus_point.y));
  camera_global_transform.SetRotation(glm::quatLookAt(glm::normalize(-camera_direction), glm::vec3(0, 1, 0)));
  return camera_global_transform;
}

void TreePointCloudCircularCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  int counter = 0;
  for (int turn_angle = turn_angle_start; turn_angle < turn_angle_end; turn_angle += turn_angle_step) {
    for (int pitch_angle = pitch_angle_start; pitch_angle < pitch_angle_end; pitch_angle += pitch_angle_step) {
      point_cloud_samples.resize((counter + 1) * scan_resolution * scan_resolution);
      auto scanner_global_transform =
          GetTransform(glm::vec2(camera_focus_point.x, camera_focus_point.y), turn_angle, pitch_angle);
      auto front = scanner_global_transform.GetRotation() * glm::vec3(0, 0, -1);
      auto up = scanner_global_transform.GetRotation() * glm::vec3(0, 1, 0);
      auto left = scanner_global_transform.GetRotation() * glm::vec3(1, 0, 0);
      auto position = scanner_global_transform.GetPosition();
      std::vector<std::shared_future<void>> results;
      Jobs::RunParallelFor(scan_resolution * scan_resolution, [&](const unsigned i) {
        const float x = i % scan_resolution;
        const float y = i / scan_resolution;
        const float x_angle = (x - scan_resolution / 2.0f + glm::linearRand(-0.5f, 0.5f)) /
                              static_cast<float>(scan_resolution) * camera_fov / 2.0f;
        const float y_angle = (y - scan_resolution / 2.0f + glm::linearRand(-0.5f, 0.5f)) /
                              static_cast<float>(scan_resolution) * camera_fov / 2.0f;
        auto& sample = point_cloud_samples[counter * scan_resolution * scan_resolution + i];
        sample.direction =
            glm::normalize(glm::rotate(glm::rotate(front, glm::radians(x_angle), left), glm::radians(y_angle), up));
        sample.start = position;
      });
      counter++;
    }
  }
}

bool TreePointCloudGridCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat("Max size", &bounding_box_size, 0.1f, 0.f, 999.f))
    changed = true;
  if (ImGui::DragInt2("Grid size", &grid_size.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat2("Grid distance", &grid_distance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &step, 0.01f, 0.0f, 0.5f))
    changed = true;
  if (ImGui::DragInt("Sample", &ground_sample_size, 1, 1, INT_MAX))
    changed = true;
  return changed;
}

void TreePointCloudGridCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  const auto start_point = glm::vec2((static_cast<float>(grid_size.x) * 0.5f - 0.5f) * grid_distance.x,
                                     (static_cast<float>(grid_size.y) * 0.5f - 0.5f) * grid_distance.y);

  const int y_step_size = grid_size.y * grid_distance.y / step;
  const int x_step_size = grid_size.x * grid_distance.x / step;

  point_cloud_samples.resize((grid_size.x * y_step_size + grid_size.y * x_step_size) *
                             (ground_sample_size + drone_sample_size));
  unsigned start_index = 0;
  for (int i = 0; i < grid_size.x; i++) {
    const float x = i * grid_distance.x;
    for (int step = 0; step < y_step_size; step++) {
      const float z = step * step;
      const glm::vec3 center = glm::vec3{x, ground_sample_height, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(ground_sample_size, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[ground_sample_size * (i * y_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        if (glm::linearRand(0.0f, 1.0f) > 0.3f) {
          sample.direction.y = glm::abs(sample.direction.y);
        } else {
          sample.direction.y = -glm::abs(sample.direction.y);
        }
        sample.start = center;
      });
    }
  }

  start_index += grid_size.x * y_step_size * ground_sample_size;
  for (int i = 0; i < grid_size.y; i++) {
    const float z = i * grid_distance.y;
    for (int step = 0; step < x_step_size; step++) {
      const float x = step * step;
      const glm::vec3 center = glm::vec3{x, ground_sample_height, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(ground_sample_size, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[start_index + ground_sample_size * (i * x_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        if (glm::linearRand(0.0f, 1.0f) > 0.3f) {
          sample.direction.y = glm::abs(sample.direction.y);
        } else {
          sample.direction.y = -glm::abs(sample.direction.y);
        }
        sample.start = center;
      });
    }
  }

  start_index += grid_size.y * x_step_size * ground_sample_size;
  for (int i = 0; i < grid_size.x; i++) {
    const float x = i * grid_distance.x;
    for (int step = 0; step < y_step_size; step++) {
      const float z = step * step;
      const glm::vec3 center = glm::vec3{x, drone_sample_height, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(drone_sample_size, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[drone_sample_size * (i * y_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }

  start_index += grid_size.x * y_step_size * drone_sample_size;
  for (int i = 0; i < grid_size.y; i++) {
    const float z = i * grid_distance.y;
    for (int step = 0; step < x_step_size; step++) {
      const float x = step * step;
      const glm::vec3 center = glm::vec3{x, drone_sample_height, z} - glm::vec3(start_point.x, 0, start_point.y);
      Jobs::RunParallelFor(drone_sample_size, [&](const unsigned sample_index) {
        auto& sample = point_cloud_samples[start_index + drone_sample_size * (i * x_step_size + step) + sample_index];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }
}

bool TreePointCloudGridCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  if (bounding_box_size == 0.f)
    return true;
  return glm::abs(sample.hit_info.position.x) < bounding_box_size &&
         glm::abs(sample.hit_info.position.z) < bounding_box_size;
}
#pragma endregion

void TreePointCloudScanner::Capture(const TreeMeshGeneratorSettings& mesh_generator_settings,
                                    const std::filesystem::path& save_path,
                                    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Soil> soil;
  const auto soil_candidate = EcoSysLabLayer::FindSoil();
  if (!soil_candidate.expired())
    soil = soil_candidate.lock();
  if (!soil) {
    EVOENGINE_ERROR("No soil!");
    return;
  }
  std::unordered_map<Handle, Handle> branch_mesh_renderer_handles, foliage_mesh_renderer_handles;
  Bound plant_bound{};
  auto scene = GetScene();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (tree_entities == nullptr) {
    EVOENGINE_ERROR("No trees!");
    return;
  }
  for (const auto& tree_entity : *tree_entities) {
    if (scene->IsEntityValid(tree_entity)) {
      // auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      // auto copyPath = savePath;
      // tree->ExportTreeParts(ecoSysLabLayer->meshGeneratorSettings, copyPath.replace_extension(".yml"));

      scene->ForEachChild(tree_entity, [&](Entity child) {
        if (scene->GetEntityName(child) == "Branch Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
          const auto branch_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
          branch_mesh_renderer_handles.insert({branch_mesh_renderer->GetHandle(), tree_entity.GetIndex()});

          const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
          const auto mesh = branch_mesh_renderer->mesh.Get<Mesh>();
          plant_bound.min =
              glm::min(plant_bound.min, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plant_bound.max =
              glm::max(plant_bound.max, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        } else if (scene->GetEntityName(child) == "Foliage Mesh" && scene->HasPrivateComponent<Particles>(child)) {
          const auto foliage_mesh_renderer = scene->GetOrSetPrivateComponent<Particles>(child).lock();
          foliage_mesh_renderer_handles.insert({foliage_mesh_renderer->GetHandle(), tree_entity.GetIndex()});

          const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
          const auto mesh = foliage_mesh_renderer->mesh.Get<Mesh>();
          plant_bound.min =
              glm::min(plant_bound.min, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plant_bound.max =
              glm::max(plant_bound.max, glm::vec3(global_transform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        } else if (scene->GetEntityName(child) == "Twig Strands" &&
                   scene->HasPrivateComponent<StrandsRenderer>(child)) {
          const auto twig_strands_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(child).lock();
          branch_mesh_renderer_handles.insert({twig_strands_renderer->GetHandle(), tree_entity.GetIndex()});

          const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
          const auto strands = twig_strands_renderer->strands.Get<Strands>();
          plant_bound.min =
              glm::min(plant_bound.min, glm::vec3(global_transform.value * glm::vec4(strands->GetBound().min, 1.0f)));
          plant_bound.max =
              glm::max(plant_bound.max, glm::vec3(global_transform.value * glm::vec4(strands->GetBound().max, 1.0f)));
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
  switch (capture_settings->capture_mode) {
    case PointCloudCaptureSettings::CaptureMode::OptiX: {
#ifdef CUDA_MODULE_PLUGIN
      CudaModule::SamplePointCloud(Application::GetLayer<RayTracerLayer>()->environment_properties, pc_samples);
#else
      EVOENGINE_ERROR("Missing CudaModule plugin!")
#endif
    } break;
    case PointCloudCaptureSettings::CaptureMode::Cpu: {
      /**
       * You may take a look at render instances, to see what it contains. RenderLayer will prepare a RenderInstance
       * every frame that contains all needed information for rendering everything for current scene. It's used in
       * rasterization rendering, and here we also use it for ray tracing. It also detects updates of the scene, like
       * transformation, mesh, material changes.
       */
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
      /**
       * During this step, the cpu_ray_tracer will scan all MeshRendereres in the scene, and establish TLAS and BLAS
       * based on them.
       */
      cpu_ray_tracer.Initialize(
          render_instances,
          [&](uint32_t, const std::shared_ptr<Mesh>&) {

          },
          [&](const uint32_t node_index, const Entity& entity) {

          });
      cpu_ray_tracer.SamplePointCloud(pc_samples);
    } break;
    case PointCloudCaptureSettings::CaptureMode::GpuCompute: {
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
      /**
       * During this step, the cpu_ray_tracer will scan all MeshRendereres in the scene, and establish TLAS and BLAS
       * based on them.
       */
      cpu_ray_tracer.Initialize(
          render_instances,
          [&](uint32_t, const std::shared_ptr<Mesh>&) {

          },
          [&](const uint32_t node_index, const Entity& entity) {

          });
      /**
       * The cpu_ray_tracer will aggregate and flatten TLAS and BLAS so from its hierarcal structure to vectors so we
       * can use it on GPU.
       */
      auto aggregate_scene = cpu_ray_tracer.Aggregate();
      /**
       * Upload prepared data to GPU, these data will be linked to the compute pipeline via Descriptors (collectively
       * DescriptorSet) so we can read them in shader. You may take a look at its implementation to see how easy to send
       * data to GPU.
       */
      aggregate_scene.InitializeBuffers();
      aggregate_scene.SamplePointCloudGpu(cpu_ray_tracer, pc_samples);
    } break;
  }
  std::vector<glm::vec3> points;

  std::vector<int> internode_index;
  std::vector<int> branch_index;
  std::vector<int> tree_part_index;
  std::vector<int> tree_part_type_index;
  std::vector<int> line_index;
  std::vector<int> instance_index;
  std::vector<int> type_index;

  for (const auto& sample : pc_samples) {
    if (!sample.hit)
      continue;
    if (!capture_settings->SampleFilter(sample))
      continue;
    auto& position = sample.hit_info.position;
    if (position.x < (plant_bound.min.x - point_settings.bounding_box_limit) ||
        position.y < (plant_bound.min.y - point_settings.bounding_box_limit) ||
        position.z < (plant_bound.min.z - point_settings.bounding_box_limit) ||
        position.x > (plant_bound.max.x + point_settings.bounding_box_limit) ||
        position.y > (plant_bound.max.y + point_settings.bounding_box_limit) ||
        position.z > (plant_bound.max.z + point_settings.bounding_box_limit))
      continue;
    auto ball_rand = glm::vec3(0.0f);
    if (point_settings.ball_rand_radius > 0.0f) {
      ball_rand = glm::ballRand(point_settings.ball_rand_radius);
    }
    const auto distance = glm::distance(sample.hit_info.position, sample.start);
    points.emplace_back(sample.hit_info.position +
                        distance * glm::vec3(glm::gaussRand(0.0f, point_settings.variance),
                                             glm::gaussRand(0.0f, point_settings.variance),
                                             glm::gaussRand(0.0f, point_settings.variance)) +
                        ball_rand);

    if (point_settings.internode_index) {
      internode_index.emplace_back(static_cast<int>(sample.hit_info.data.x + 0.1f));
    }
    if (point_settings.branch_index) {
      branch_index.emplace_back(static_cast<int>(sample.hit_info.data.y + 0.1f));
    }
    if (point_settings.line_index) {
      line_index.emplace_back(static_cast<int>(sample.hit_info.data.z + 0.1f));
    }
    if (point_settings.tree_part_index) {
      tree_part_index.emplace_back(static_cast<int>(sample.hit_info.data2.x + 0.1f));
    }
    if (point_settings.tree_part_type_index) {
      tree_part_type_index.emplace_back(static_cast<int>(sample.hit_info.data2.y + 0.1f));
    }
    auto branch_search = branch_mesh_renderer_handles.find(sample.handle);
    auto foliage_search = foliage_mesh_renderer_handles.find(sample.handle);
    if (point_settings.instance_index) {
      if (branch_search != branch_mesh_renderer_handles.end()) {
        instance_index.emplace_back(branch_search->second);
      } else if (foliage_search != foliage_mesh_renderer_handles.end()) {
        instance_index.emplace_back(foliage_search->second);
      } else {
        instance_index.emplace_back(0);
      }
    }

    if (point_settings.type_index) {
      if (branch_search != branch_mesh_renderer_handles.end()) {
        type_index.emplace_back(0);
      } else if (foliage_search != foliage_mesh_renderer_handles.end()) {
        type_index.emplace_back(1);
      } else if (sample.handle == ground_mesh_renderer_handle) {
        type_index.emplace_back(2);
      } else {
        type_index.emplace_back(-1);
      }
    }
  }
  std::filebuf fb_binary;
  fb_binary.open(save_path.string(), std::ios::out | std::ios::binary);
  std::ostream out_stream_binary(&fb_binary);
  if (out_stream_binary.fail())
    throw std::runtime_error("failed to open " + save_path.string());

  tinyply::PlyFile cube_file;
  cube_file.add_properties_to_element("vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, points.size(),
                                      reinterpret_cast<uint8_t*>(points.data()), tinyply::Type::INVALID, 0);

  if (point_settings.type_index)
    cube_file.add_properties_to_element("type_index", {"type_index"}, tinyply::Type::INT32, type_index.size(),
                                        reinterpret_cast<uint8_t*>(type_index.data()), tinyply::Type::INVALID, 0);

  if (point_settings.instance_index) {
    cube_file.add_properties_to_element("instance_index", {"instance_index"}, tinyply::Type::INT32,
                                        instance_index.size(), reinterpret_cast<uint8_t*>(instance_index.data()),
                                        tinyply::Type::INVALID, 0);
  }
  if (point_settings.branch_index) {
    cube_file.add_properties_to_element("branch_index", {"branch_index"}, tinyply::Type::INT32, branch_index.size(),
                                        reinterpret_cast<uint8_t*>(branch_index.data()), tinyply::Type::INVALID, 0);
  }
  if (point_settings.tree_part_index) {
    cube_file.add_properties_to_element("tree_part_index", {"tree_part_index"}, tinyply::Type::INT32,
                                        tree_part_index.size(), reinterpret_cast<uint8_t*>(tree_part_index.data()),
                                        tinyply::Type::INVALID, 0);
  }
  if (point_settings.tree_part_type_index) {
    cube_file.add_properties_to_element(
        "tree_part_type_index", {"tree_part_type_index"}, tinyply::Type::INT32, tree_part_type_index.size(),
        reinterpret_cast<uint8_t*>(tree_part_type_index.data()), tinyply::Type::INVALID, 0);
  }
  if (point_settings.line_index) {
    cube_file.add_properties_to_element("line_index", {"line_index"}, tinyply::Type::INT32, line_index.size(),
                                        reinterpret_cast<uint8_t*>(line_index.data()), tinyply::Type::INVALID, 0);
  }
  if (point_settings.internode_index) {
    cube_file.add_properties_to_element("internode_index", {"internode_index"}, tinyply::Type::INT32,
                                        internode_index.size(), reinterpret_cast<uint8_t*>(internode_index.data()),
                                        tinyply::Type::INVALID, 0);
  }
  // Write a binary file
  cube_file.write(out_stream_binary, true);

  if (point_settings.tree_part_index) {
    try {
      std::filesystem::path yaml_path = save_path;
      yaml_path.replace_extension(".yml");
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "Forest" << YAML::BeginSeq;
      for (const auto& tree_entity : *tree_entities) {
        if (!scene->IsEntityValid(tree_entity))
          continue;
        const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        if (!tree->generate_mesh)
          continue;
        out << YAML::BeginMap;
        out << YAML::Key << "II" << YAML::Value << tree_entity.GetIndex();
        const auto gt = scene->GetDataComponent<GlobalTransform>(tree_entity);
        out << YAML::Key << "P" << YAML::Value << gt.GetPosition();
        tree->ExportTreeParts(mesh_generator_settings, out);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
      std::ofstream output_file(yaml_path.string());
      output_file << out.c_str();
      output_file.flush();
    } catch (const std::exception& e) {
      EVOENGINE_ERROR("Failed to save: " + std::string(e.what()));
    }
  }
}

bool TreePointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  if (ImGui::TreeNodeEx("Circular Capture")) {
    static auto capture_settings = std::make_shared<TreePointCloudCircularCaptureSettings>();
    capture_settings->OnInspect();
    FileUtils::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          Capture(eco_sys_lab_layer->mesh_generator_settings, path, capture_settings);
        },
        false);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Grid Capture")) {
    static auto capture_settings = std::make_shared<TreePointCloudGridCaptureSettings>();
    capture_settings->OnInspect();
    FileUtils::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          Capture(eco_sys_lab_layer->mesh_generator_settings, path, capture_settings);
        },
        false);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Point settings")) {
    point_settings.OnInspect();
    ImGui::TreePop();
  }

  return changed;
}

void TreePointCloudScanner::OnDestroy() {
  point_settings = {};
}

void TreePointCloudScanner::Serialize(YAML::Emitter& out) const {
  point_settings.Save("point_settings", out);
}

void TreePointCloudScanner::Deserialize(const YAML::Node& in) {
  point_settings.Load("point_settings", in);
}
