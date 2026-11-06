#include "CudaSerializationAdapters.hpp"

#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"
#include "RayTracerLayer.hpp"

using namespace evo_engine;

namespace {
constexpr size_t kMaximumVirtualParPoints = 1000000;

glm::vec3 IlluminationHeatColor(const float value) {
  const float t = glm::clamp(value, 0.0f, 1.0f);
  if (t < 1.0f / 3.0f)
    return glm::mix(glm::vec3(0.04f, 0.08f, 0.35f), glm::vec3(0.0f, 0.75f, 1.0f), t * 3.0f);
  if (t < 2.0f / 3.0f)
    return glm::mix(glm::vec3(0.0f, 0.75f, 1.0f), glm::vec3(1.0f, 0.9f, 0.0f), (t - 1.0f / 3.0f) * 3.0f);
  return glm::mix(glm::vec3(1.0f, 0.9f, 0.0f), glm::vec3(0.9f, 0.05f, 0.02f), (t - 2.0f / 3.0f) * 3.0f);
}
}  // namespace

void VoxelIlluminationEstimator::Estimate() {
  if (!IsValidIlluminationVoxelGrid(volume_size, voxel_resolution, samples_per_voxel))
    return;
  const size_t voxel_count = static_cast<size_t>(voxel_resolution.x) * voxel_resolution.y * voxel_resolution.z;
  if (voxel_count > kMaximumVirtualParPoints / samples_per_voxel)
    return;
  const auto points = GenerateIlluminationVoxelPoints(volume_size, voxel_resolution, samples_per_voxel, seed);
  if (points.empty()) {
    voxels_.clear();
    visualization_dirty_ = true;
    return;
  }

  const auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner());
  const glm::vec3 sensor_normal = glm::normalize(global_transform.GetRotation() * glm::vec3(0.0f, 1.0f, 0.0f));
  std::vector<IlluminationSampler<glm::vec3>> samplers(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    const glm::vec3 position = global_transform.value * glm::vec4(points[i].position, 1.0f);
    auto& sampler = samplers[i];
    sampler.v_0.position = sampler.v_1.position = sampler.v_2.position = position;
    sampler.v_0.normal = sampler.v_1.normal = sampler.v_2.normal = sensor_normal;
    sampler.front_face = true;
    sampler.back_face = false;
    sampler.direction = glm::vec3(0.0f);
    sampler.energy = glm::vec3(0.0f);
  }

  const auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  CudaModule::EstimateIlluminationRayTracing(ray_tracer_layer->environment_properties, ray_properties, samplers, seed,
                                             push_normal_distance, ray_tracer_layer->GetIlluminationClouds());
  std::vector<glm::vec3> energies(samplers.size());
  for (size_t i = 0; i < samplers.size(); i++)
    energies[i] = samplers[i].energy;
  voxels_ = AggregateIlluminationVoxels(points, energies, volume_size, voxel_resolution);
  visualization_dirty_ = true;
}

void VoxelIlluminationEstimator::RebuildVisualization() {
  if (!voxel_info_list_)
    voxel_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  float observed_maximum = 0.0f;
  for (const auto& voxel : voxels_)
    observed_maximum = glm::max(observed_maximum, voxel.GetIntensity());
  const float range_minimum = automatic_intensity_range ? 0.0f : minimum_intensity;
  const float range_maximum = automatic_intensity_range ? observed_maximum : maximum_intensity;
  const float range = glm::max(range_maximum - range_minimum, glm::epsilon<float>());
  const glm::vec3 voxel_size = volume_size / glm::vec3(voxel_resolution);
  std::vector<ParticleInfo> particle_infos(voxels_.size());
  for (size_t i = 0; i < voxels_.size(); i++) {
    particle_infos[i].instance_matrix.value = glm::translate(voxels_[i].center) * glm::scale(voxel_size * voxel_fill);
    particle_infos[i].instance_color =
        glm::vec4(IlluminationHeatColor((voxels_[i].GetIntensity() - range_minimum) / range), voxel_opacity);
  }
  voxel_info_list_->SetParticleInfos(particle_infos);
  visualization_dirty_ = false;
}

bool VoxelIlluminationEstimator::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  bool sampling_changed = false;
  if (ImGui::DragFloat3("Volume size", &volume_size.x, 0.1f, 0.001f, 1000.0f)) {
    volume_size = glm::max(volume_size, glm::vec3(0.001f));
    sampling_changed = true;
  }
  if (ImGui::DragInt3("Voxel resolution", &voxel_resolution.x, 1.0f, 1, 256)) {
    voxel_resolution = glm::clamp(voxel_resolution, glm::ivec3(1), glm::ivec3(256));
    sampling_changed = true;
  }
  if (ImGui::DragInt("Samples per voxel", &samples_per_voxel, 1.0f, 1, 1024)) {
    samples_per_voxel = glm::clamp(samples_per_voxel, 1, 1024);
    sampling_changed = true;
  }
  sampling_changed |= ImGui::DragInt("Seed", &seed);
  sampling_changed |= ImGui::DragFloat("Normal distance", &push_normal_distance, 0.0001f, -1.0f, 1.0f);
  sampling_changed |= ImGui::DragInt("Rays per point", &ray_properties.samples, 1.0f, 1, 100000);
  sampling_changed |= ImGui::DragInt("Bounces", &ray_properties.bounces, 1.0f, 0, 64);
  volume_size = glm::max(volume_size, glm::vec3(0.001f));
  voxel_resolution = glm::clamp(voxel_resolution, glm::ivec3(1), glm::ivec3(256));
  samples_per_voxel = glm::clamp(samples_per_voxel, 1, 1024);
  ray_properties.samples = glm::max(ray_properties.samples, 1);
  ray_properties.bounces = glm::max(ray_properties.bounces, 0);
  if (sampling_changed) {
    voxels_.clear();
    visualization_dirty_ = true;
    changed = true;
  }

  const size_t voxel_count = static_cast<size_t>(voxel_resolution.x) * voxel_resolution.y * voxel_resolution.z;
  const size_t point_count = voxel_count * samples_per_voxel;
  ImGui::Text("Virtual PAR points: %llu", static_cast<unsigned long long>(point_count));
  ImGui::TextUnformatted("Sensor direction: local +Y");
  ImGui::BeginDisabled(point_count > kMaximumVirtualParPoints);
  if (ImGui::Button("Estimate illumination")) {
    Estimate();
    show_voxels = true;
    changed = true;
  }
  ImGui::EndDisabled();
  if (point_count > kMaximumVirtualParPoints)
    ImGui::TextUnformatted("Reduce the grid below 1,000,000 virtual PAR points.");

  if (ImGui::TreeNode("Visualization")) {
    ImGui::Checkbox("Show voxels", &show_voxels);
    ImGui::Checkbox("Show sampling volume", &show_sampling_volume);
    if (ImGui::SliderFloat("Voxel fill", &voxel_fill, 0.05f, 1.0f))
      visualization_dirty_ = true;
    if (ImGui::SliderFloat("Voxel opacity", &voxel_opacity, 0.05f, 1.0f))
      visualization_dirty_ = true;
    if (ImGui::Checkbox("Automatic intensity range", &automatic_intensity_range))
      visualization_dirty_ = true;
    if (!automatic_intensity_range) {
      if (ImGui::DragFloat("Minimum intensity", &minimum_intensity, 0.01f, 0.0f))
        visualization_dirty_ = true;
      if (ImGui::DragFloat("Maximum intensity", &maximum_intensity, 0.01f, minimum_intensity))
        visualization_dirty_ = true;
      maximum_intensity = glm::max(maximum_intensity, minimum_intensity + glm::epsilon<float>());
    }
    ImGui::Checkbox("Depth test", &depth_test);
    ImGui::TreePop();
  }
  ImGui::Text("Estimated voxels: %llu", static_cast<unsigned long long>(voxels_.size()));
  if (voxels_.empty())
    ImGui::TextUnformatted("No result: click Estimate illumination.");

  const auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner());
  if (show_sampling_volume) {
    GizmoSettings volume_gizmo_settings{};
    volume_gizmo_settings.draw_settings.blending = true;
    volume_gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
    volume_gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
    volume_gizmo_settings.draw_settings.line_width = 2.0f;
    editor_layer->DrawGizmoCube(glm::vec4(1.0f, 1.0f, 1.0f, 0.6f), global_transform.value * glm::scale(volume_size),
                                1.0f, volume_gizmo_settings);
  }
  if (show_voxels && !voxels_.empty()) {
    if (visualization_dirty_) {
      RebuildVisualization();
    } else {
      GizmoSettings gizmo_settings{};
      gizmo_settings.draw_settings.blending = true;
      gizmo_settings.depth_test = depth_test;
      gizmo_settings.depth_write = false;
      editor_layer->DrawGizmoCubes(voxel_info_list_, global_transform.value, 1.0f, gizmo_settings);
    }
  }
  return changed;
}

const std::vector<IlluminationVoxel>& VoxelIlluminationEstimator::PeekVoxels() const {
  return voxels_;
}

void evo_engine::SerializeVoxelIlluminationEstimator(YAML::Emitter& out, const VoxelIlluminationEstimator& target) {
  out << YAML::Key << "volume_size" << YAML::Value << target.volume_size;
  out << YAML::Key << "voxel_resolution" << YAML::Value << target.voxel_resolution;
  out << YAML::Key << "samples_per_voxel" << YAML::Value << target.samples_per_voxel;
  out << YAML::Key << "seed" << YAML::Value << target.seed;
  out << YAML::Key << "push_normal_distance" << YAML::Value << target.push_normal_distance;
  out << YAML::Key << "ray_samples" << YAML::Value << target.ray_properties.samples;
  out << YAML::Key << "ray_bounces" << YAML::Value << target.ray_properties.bounces;
  out << YAML::Key << "show_voxels" << YAML::Value << target.show_voxels;
  out << YAML::Key << "show_sampling_volume" << YAML::Value << target.show_sampling_volume;
  out << YAML::Key << "automatic_intensity_range" << YAML::Value << target.automatic_intensity_range;
  out << YAML::Key << "depth_test" << YAML::Value << target.depth_test;
  out << YAML::Key << "voxel_fill" << YAML::Value << target.voxel_fill;
  out << YAML::Key << "voxel_opacity" << YAML::Value << target.voxel_opacity;
  out << YAML::Key << "minimum_intensity" << YAML::Value << target.minimum_intensity;
  out << YAML::Key << "maximum_intensity" << YAML::Value << target.maximum_intensity;
}

void evo_engine::DeserializeVoxelIlluminationEstimator(const YAML::Node& in, VoxelIlluminationEstimator& target) {
  if (in["volume_size"])
    target.volume_size = in["volume_size"].as<glm::vec3>();
  if (in["voxel_resolution"])
    target.voxel_resolution = in["voxel_resolution"].as<glm::ivec3>();
  if (in["samples_per_voxel"])
    target.samples_per_voxel = in["samples_per_voxel"].as<int>();
  if (in["seed"])
    target.seed = in["seed"].as<int>();
  if (in["push_normal_distance"])
    target.push_normal_distance = in["push_normal_distance"].as<float>();
  if (in["ray_samples"])
    target.ray_properties.samples = in["ray_samples"].as<int>();
  if (in["ray_bounces"])
    target.ray_properties.bounces = in["ray_bounces"].as<int>();
  if (in["show_voxels"])
    target.show_voxels = in["show_voxels"].as<bool>();
  if (in["show_sampling_volume"])
    target.show_sampling_volume = in["show_sampling_volume"].as<bool>();
  if (in["automatic_intensity_range"])
    target.automatic_intensity_range = in["automatic_intensity_range"].as<bool>();
  if (in["depth_test"])
    target.depth_test = in["depth_test"].as<bool>();
  if (in["voxel_fill"])
    target.voxel_fill = in["voxel_fill"].as<float>();
  if (in["voxel_opacity"])
    target.voxel_opacity = in["voxel_opacity"].as<float>();
  if (in["minimum_intensity"])
    target.minimum_intensity = in["minimum_intensity"].as<float>();
  if (in["maximum_intensity"])
    target.maximum_intensity = in["maximum_intensity"].as<float>();
}
