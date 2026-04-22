#include "TasselPointCloudScanner.hpp"

#include "AssetManager.hpp"
#include "CpuRayTracer.hpp"
#include "MaizeTassel.hpp"
#include "Particles.hpp"
#include "PointCloud.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Tinyply.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <glm/gtc/matrix_transform.hpp>
#include <limits>
#include <random>
#include <sstream>

using namespace l_system_plugin;
using namespace dataset_generation_plugin;

namespace {
constexpr uint32_t kAngularNoiseSeedSalt = 0xA11CE5EDu;
constexpr uint32_t kPointNoiseSeedSalt = 0x51DE5EEDu;
constexpr uint32_t kBeamShuffleSeedSalt = 0xB34F1234u;
constexpr uint32_t kScanSettingsSeedSalt = 0xC0A151EDu;
constexpr uint32_t kVisualSettingsSeedSalt = 0x715AA5E1u;
constexpr uint32_t kOneShotRepeatSeedSalt = 0x0AE50001u;

uint32_t MixSeed(const int seed, const uint32_t salt) {
  uint32_t x = static_cast<uint32_t>(seed) ^ salt;
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

int AdvanceSeedWithWrap(const int seed) {
  if (seed == std::numeric_limits<int>::max()) {
    return std::numeric_limits<int>::min();
  }
  return seed + 1;
}

float SampleGaussian(std::mt19937& rng, const float sigma) {
  if (sigma <= 0.0f) {
    return 0.0f;
  }
  std::normal_distribution<float> dist(0.0f, sigma);
  return dist(rng);
}

glm::vec3 SampleBall(std::mt19937& rng, const float radius) {
  if (radius <= 0.0f) {
    return glm::vec3(0.0f);
  }

  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int attempt = 0; attempt < 32; attempt++) {
    const glm::vec3 p(dist(rng), dist(rng), dist(rng));
    const float len_sq = glm::dot(p, p);
    if (len_sq <= 1.0f) {
      return p * radius;
    }
  }

  return glm::vec3(0.0f);
}

double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

void ExpandBoundWithTransformedMeshBound(Bound& plant_bound, const glm::mat4& transform,
                                         const Bound& local_bound, bool& has_plant_bound) {
  std::vector<glm::vec3> corners;
  local_bound.PopulateCorners(corners);
  for (const auto& corner : corners) {
    const auto world_corner = glm::vec3(transform * glm::vec4(corner, 1.0f));
    plant_bound.min = glm::min(plant_bound.min, world_corner);
    plant_bound.max = glm::max(plant_bound.max, world_corner);
  }
  has_plant_bound = true;
}

float BoundMaxAbsComponent(const Bound& bound) {
  const glm::vec3 abs_min = glm::abs(bound.min);
  const glm::vec3 abs_max = glm::abs(bound.max);
  return std::max(
      std::max(std::max(abs_min.x, abs_min.y), abs_min.z),
      std::max(std::max(abs_max.x, abs_max.y), abs_max.z));
}

const char* ScanModeName(const TasselScanMode mode) {
  switch (mode) {
    case TasselScanMode::Hemisphere:
      return "Hemisphere";
    case TasselScanMode::Gantry:
      return "Gantry";
    case TasselScanMode::Circular:
      return "Circular";
  }
  return "Unknown";
}

std::filesystem::path ResolveDefaultTasselScannerDescriptorPath() {
  const std::array<std::filesystem::path, 4> resource_candidates = {
      std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/") /
          "New TasselPointCloudScannerDescriptor.tscan",
      std::filesystem::path("./DigitalAgricultureProject/Assets/") /
          "New TasselPointCloudScannerDescriptor.tscan",
      std::filesystem::path("./DatasetGenerationResources/Defaults/") /
          "New TasselPointCloudScannerDescriptor.tscan",
      std::filesystem::path("./EvoEngine_Plugins/DatasetGeneration/Internals/DatasetGenerationResources/Defaults/") /
          "New TasselPointCloudScannerDescriptor.tscan"};

  for (const auto& relative_candidate : resource_candidates) {
    const auto absolute_candidate = std::filesystem::absolute(relative_candidate);
    if (std::filesystem::exists(absolute_candidate)) {
      return absolute_candidate;
    }
  }

  const auto assets_folder = ProjectManager::GetAssetsFolderPath();
  if (!assets_folder.empty()) {
    const std::array<std::filesystem::path, 2> project_asset_candidates = {
        std::filesystem::path("DatasetGeneration") / "New TasselPointCloudScannerDescriptor.tscan",
        "New TasselPointCloudScannerDescriptor.tscan"};
    for (const auto& relative_candidate : project_asset_candidates) {
      const auto absolute_candidate = assets_folder / relative_candidate;
      if (std::filesystem::exists(absolute_candidate)) {
        return absolute_candidate;
      }
    }
  }

  return {};
}

std::filesystem::path ResolveWritableTasselScannerDescriptorDefaultsPath() {
  if (const auto existing = ResolveDefaultTasselScannerDescriptorPath(); !existing.empty()) {
    return existing;
  }

  return std::filesystem::absolute(
      std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/") /
      "New TasselPointCloudScannerDescriptor.tscan");
}

bool SaveTasselScannerDescriptorDefaultsToFile(const TasselPointCloudScannerDescriptor& descriptor,
                                               const std::filesystem::path& file_path) {
  if (file_path.empty()) {
    return false;
  }

  try {
    std::filesystem::create_directories(file_path.parent_path());

    YAML::Emitter out;
    out << YAML::BeginMap;
    descriptor.Serialize(out);
    out << YAML::EndMap;

    std::ofstream stream(file_path.string(), std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
      EVOENGINE_WARNING("Failed to open scanner defaults file for writing: " + file_path.string());
      return false;
    }

    stream << out.c_str();
    stream.flush();
    if (!stream.good()) {
      EVOENGINE_WARNING("Failed while writing scanner defaults file: " + file_path.string());
      return false;
    }

    return true;
  } catch (const std::exception& e) {
    EVOENGINE_WARNING("Failed to save TasselPointCloudScannerDescriptor defaults to " + file_path.string() + ": " +
                      std::string(e.what()));
    return false;
  }
}

bool LoadTasselScannerDescriptorDefaultsFromFile(TasselPointCloudScannerDescriptor& descriptor,
                                                 const std::filesystem::path& file_path) {
  if (file_path.empty() || !std::filesystem::exists(file_path)) {
    return false;
  }

  try {
    const std::ifstream stream(file_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node defaults = YAML::Load(string_stream.str());
    if (!defaults || !defaults.IsMap()) {
      return false;
    }
    descriptor.Deserialize(defaults);
    return true;
  } catch (const std::exception& e) {
    EVOENGINE_WARNING("Failed to load TasselPointCloudScannerDescriptor defaults from " + file_path.string() + ": " +
                      std::string(e.what()));
    return false;
  }
}

void AutoScaleTasselCaptureSettingsToPlantBound(
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    const Bound& plant_bound) {
  const auto tassel_capture_settings =
      std::dynamic_pointer_cast<TasselPointCloudGridCaptureSettings>(capture_settings);
  if (!tassel_capture_settings) {
    return;
  }

  const glm::vec3 extent = glm::max(plant_bound.max - plant_bound.min, glm::vec3(1e-4f));
  const glm::vec3 plant_center = 0.5f * (plant_bound.min + plant_bound.max);
  const float height = extent.y;
  const float horizontal_radius = 0.5f * std::max(extent.x, extent.z);
  const float max_abs_position = BoundMaxAbsComponent(plant_bound);

  tassel_capture_settings->scan_center = glm::vec3(plant_center.x, 0.0f, plant_center.z);
  tassel_capture_settings->look_target_height = plant_center.y;

  // Ensure world-space sample filter bounds always include the full plant regardless of unit scale.
  const float required_bbox_half_extent = max_abs_position + std::max(1.0f, 0.35f * height);
  tassel_capture_settings->bounding_box_size =
      std::max(tassel_capture_settings->bounding_box_size, required_bbox_half_extent);

  // Place scanners using geometry-derived scale to avoid keeping only the basal region.
  const float required_sample_height = plant_bound.max.y + 0.30f * height;
  tassel_capture_settings->sample_height =
      std::max(tassel_capture_settings->sample_height, required_sample_height);

  if (tassel_capture_settings->scan_mode == TasselScanMode::Circular) {
    const float required_scanner_distance =
        horizontal_radius + 0.55f * height + 1.0f;
    tassel_capture_settings->scanner_distance =
        std::max(tassel_capture_settings->scanner_distance, required_scanner_distance);
  }
}

bool CollectTasselPlantBound(const std::shared_ptr<Scene>& scene, Bound& plant_bound, bool& has_plant_bound) {
  has_plant_bound = false;
  plant_bound = Bound{};

  const std::vector<Entity>* tassel_entities = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
  if (tassel_entities == nullptr) {
    return false;
  }

  for (const auto& tassel_entity : *tassel_entities) {
    if (!scene->IsEntityValid(tassel_entity)) {
      continue;
    }

    scene->ForEachChild(tassel_entity, [&](const Entity child) {
      const auto name = scene->GetEntityName(child);
      if (name == "Tassel Internodes" && scene->HasPrivateComponent<Particles>(child)) {
        const auto particles = scene->GetOrSetPrivateComponent<Particles>(child).lock();
        if (!particles) {
          return;
        }

        const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
        const auto mesh = particles->mesh.Get<Mesh>();
        if (!mesh) {
          return;
        }

        const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (particle_info_list && !particle_info_list->PeekParticleInfoList().empty()) {
          const auto& infos = particle_info_list->PeekParticleInfoList();
          for (const auto& info : infos) {
            ExpandBoundWithTransformedMeshBound(
                plant_bound, global_transform.value * info.instance_matrix.value,
                mesh->GetBound(), has_plant_bound);
          }
        } else {
          ExpandBoundWithTransformedMeshBound(
              plant_bound, global_transform.value, mesh->GetBound(), has_plant_bound);
        }
      } else if (name == "Tassel Stem Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
        if (!mesh_renderer) {
          return;
        }

        const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
        const auto mesh = mesh_renderer->mesh.Get<Mesh>();
        if (mesh) {
          ExpandBoundWithTransformedMeshBound(
              plant_bound, global_transform.value, mesh->GetBound(), has_plant_bound);
        }
      } else if (name == "Tassel Spikelets" && scene->HasPrivateComponent<Particles>(child)) {
        const auto particles = scene->GetOrSetPrivateComponent<Particles>(child).lock();
        if (!particles) {
          return;
        }

        const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
        const auto mesh = particles->mesh.Get<Mesh>();
        if (!mesh) {
          return;
        }

        const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (particle_info_list && !particle_info_list->PeekParticleInfoList().empty()) {
          const auto& infos = particle_info_list->PeekParticleInfoList();
          for (const auto& info : infos) {
            ExpandBoundWithTransformedMeshBound(
                plant_bound, global_transform.value * info.instance_matrix.value,
                mesh->GetBound(), has_plant_bound);
          }
        } else {
          ExpandBoundWithTransformedMeshBound(
              plant_bound, global_transform.value, mesh->GetBound(), has_plant_bound);
        }
      }
    });
  }

  return true;
}

bool PointInsideExpandedBound(const glm::vec3& point, const Bound& plant_bound, float expand) {
  return point.x >= (plant_bound.min.x - expand) &&
         point.y >= (plant_bound.min.y - expand) &&
         point.z >= (plant_bound.min.z - expand) &&
         point.x <= (plant_bound.max.x + expand) &&
         point.y <= (plant_bound.max.y + expand) &&
         point.z <= (plant_bound.max.z + expand);
}

glm::vec4 HeatColorFromAge(float age_seconds, float hot_seconds, float warm_seconds, float cool_seconds) {
  const float hot = std::max(0.0001f, hot_seconds);
  const float warm = std::max(0.0001f, warm_seconds);
  const float cool = std::max(0.0001f, cool_seconds);
  const float total = hot + warm + cool;

  if (age_seconds <= hot) {
    return glm::vec4(1.0f, 0.12f, 0.08f, 1.0f);
  }

  if (age_seconds <= hot + warm) {
    const float t = glm::clamp((age_seconds - hot) / warm, 0.0f, 1.0f);
    return glm::mix(glm::vec4(1.0f, 0.62f, 0.10f, 1.0f),
                    glm::vec4(1.0f, 0.95f, 0.22f, 1.0f), t);
  }

  if (age_seconds <= total) {
    const float t = glm::clamp((age_seconds - hot - warm) / cool, 0.0f, 1.0f);
    return glm::mix(glm::vec4(1.0f, 0.95f, 0.22f, 1.0f),
                    glm::vec4(0.0f, 0.0f, 0.0f, 1.0f), t);
  }

  return glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

void ShowItemHoverDescription(const char* description) {
  if (description == nullptr || description[0] == '\0') {
    return;
  }

  if (!ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort | ImGuiHoveredFlags_AllowWhenDisabled)) {
    return;
  }

  ImGui::BeginTooltip();
  ImGui::PushTextWrapPos(ImGui::GetFontSize() * 40.0f);
  ImGui::TextUnformatted(description);
  ImGui::PopTextWrapPos();
  ImGui::EndTooltip();
}

float SampleFloatMeanDeviation(std::mt19937& rng, const float mean, const float deviation) {
  if (deviation <= 0.0f) {
    return mean;
  }
  std::normal_distribution<float> dist(mean, deviation);
  return dist(rng);
}

int SampleIntMeanDeviation(std::mt19937& rng, const int mean, const int deviation) {
  if (deviation <= 0) {
    return mean;
  }
  std::normal_distribution<float> dist(static_cast<float>(mean), static_cast<float>(deviation));
  return static_cast<int>(std::lround(dist(rng)));
}

glm::vec3 SampleVec3MeanDeviation(std::mt19937& rng, const glm::vec3& mean, const glm::vec3& deviation) {
  return glm::vec3(
      SampleFloatMeanDeviation(rng, mean.x, std::max(0.0f, deviation.x)),
      SampleFloatMeanDeviation(rng, mean.y, std::max(0.0f, deviation.y)),
      SampleFloatMeanDeviation(rng, mean.z, std::max(0.0f, deviation.z)));
}

bool BeginMeanDeviationTable(const char* table_id) {
  if (!ImGui::BeginTable(table_id,
                         3,
                         ImGuiTableFlags_SizingStretchProp |
                         ImGuiTableFlags_BordersInnerV |
                         ImGuiTableFlags_RowBg)) {
    return false;
  }
  ImGui::TableSetupColumn("Parameter", ImGuiTableColumnFlags_WidthStretch, 1.9f);
  ImGui::TableSetupColumn("Mean", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("Deviation", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableHeadersRow();
  return true;
}

bool InspectFloatMeanDeviation(const char* label, float& mean, float& deviation, const float speed,
                               const float mean_min, const float mean_max,
                               const float deviation_min, const float deviation_max,
                               const char* tip) {
  bool changed = false;
  ImGui::PushID(label);

  ImGui::TableNextRow();

  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ShowItemHoverDescription(tip);

  ImGui::TableSetColumnIndex(1);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::DragFloat("##mean", &mean, speed, mean_min, mean_max)) {
    changed = true;
  }
  ShowItemHoverDescription(tip);

  ImGui::TableSetColumnIndex(2);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::DragFloat("##deviation", &deviation, speed, deviation_min, deviation_max)) {
    changed = true;
  }
  ShowItemHoverDescription(tip);

  ImGui::PopID();
  return changed;
}

bool InspectIntMeanDeviation(const char* label, int& mean, int& deviation, const float speed,
                             const int mean_min, const int mean_max,
                             const int deviation_min, const int deviation_max,
                             const char* tip) {
  bool changed = false;
  ImGui::PushID(label);

  ImGui::TableNextRow();

  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ShowItemHoverDescription(tip);

  ImGui::TableSetColumnIndex(1);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::DragInt("##mean", &mean, speed, mean_min, mean_max)) {
    changed = true;
  }
  ShowItemHoverDescription(tip);

  ImGui::TableSetColumnIndex(2);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::DragInt("##deviation", &deviation, speed, deviation_min, deviation_max)) {
    changed = true;
  }
  ShowItemHoverDescription(tip);

  ImGui::PopID();
  return changed;
}

bool InspectVec3MeanDeviation(const char* label, glm::vec3& mean, glm::vec3& deviation, const float speed,
                              const float mean_min, const float mean_max,
                              const float deviation_min, const float deviation_max,
                              const char* tip) {
  bool changed = false;
  ImGui::PushID(label);

  ImGui::TableNextRow();

  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ShowItemHoverDescription(tip);

  ImGui::TableSetColumnIndex(1);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::DragFloat3("##mean", &mean.x, speed, mean_min, mean_max)) {
    changed = true;
  }
  ShowItemHoverDescription(tip);

  ImGui::TableSetColumnIndex(2);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::DragFloat3("##deviation", &deviation.x, speed, deviation_min, deviation_max)) {
    changed = true;
  }
  ShowItemHoverDescription(tip);

  ImGui::PopID();
  return changed;
}

void SaveFloatMeanDeviation(YAML::Emitter& out, const char* key, const float mean, const float deviation) {
  out << YAML::Key << key << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "mean" << YAML::Value << mean;
  out << YAML::Key << "deviation" << YAML::Value << deviation;
  out << YAML::EndMap;
}

void SaveIntMeanDeviation(YAML::Emitter& out, const char* key, const int mean, const int deviation) {
  out << YAML::Key << key << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "mean" << YAML::Value << mean;
  out << YAML::Key << "deviation" << YAML::Value << deviation;
  out << YAML::EndMap;
}

void SaveVec3MeanDeviation(YAML::Emitter& out, const char* key, const glm::vec3& mean, const glm::vec3& deviation) {
  out << YAML::Key << key << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "mean" << YAML::Value << mean;
  out << YAML::Key << "deviation" << YAML::Value << deviation;
  out << YAML::EndMap;
}

void LoadFloatMeanDeviation(const YAML::Node& root, const char* key, float& mean, float& deviation) {
  if (!root[key]) {
    return;
  }
  const auto& node = root[key];
  if (!node.IsMap()) {
    return;
  }
  if (node["mean"]) {
    mean = node["mean"].as<float>();
  }
  if (node["deviation"]) {
    deviation = node["deviation"].as<float>();
  }
}

void LoadIntMeanDeviation(const YAML::Node& root, const char* key, int& mean, int& deviation) {
  if (!root[key]) {
    return;
  }
  const auto& node = root[key];
  if (!node.IsMap()) {
    return;
  }
  if (node["mean"]) {
    mean = node["mean"].as<int>();
  }
  if (node["deviation"]) {
    deviation = node["deviation"].as<int>();
  }
}

void LoadVec3MeanDeviation(const YAML::Node& root, const char* key, glm::vec3& mean, glm::vec3& deviation) {
  if (!root[key]) {
    return;
  }
  const auto& node = root[key];
  if (!node.IsMap()) {
    return;
  }
  if (node["mean"]) {
    mean = node["mean"].as<glm::vec3>();
  }
  if (node["deviation"]) {
    deviation = node["deviation"].as<glm::vec3>();
  }
}

TasselPointCloudPointSettings SamplePointSettingsForScan(const TasselPointCloudPointSettings& input,
                                                         std::mt19937& rng) {
  TasselPointCloudPointSettings sampled = input;
  sampled.distance_sigma_scale = std::max(0.0f, SampleFloatMeanDeviation(
      rng, input.distance_sigma_scale, input.distance_sigma_scale_deviation));
  sampled.hit_ball_jitter_radius = std::max(0.0f, SampleFloatMeanDeviation(
      rng, input.hit_ball_jitter_radius, input.hit_ball_jitter_radius_deviation));
  sampled.range_noise_base_sigma = std::max(0.0f, SampleFloatMeanDeviation(
      rng, input.range_noise_base_sigma, input.range_noise_base_sigma_deviation));
  sampled.range_noise_scale = std::max(0.0f, SampleFloatMeanDeviation(
      rng, input.range_noise_scale, input.range_noise_scale_deviation));
  sampled.dropout_probability = glm::clamp(
      SampleFloatMeanDeviation(rng, input.dropout_probability, input.dropout_probability_deviation), 0.0f, 1.0f);
  sampled.angular_noise_sigma = std::max(0.0f, SampleFloatMeanDeviation(
      rng, input.angular_noise_sigma, input.angular_noise_sigma_deviation));
  sampled.min_range = std::max(0.0f, SampleFloatMeanDeviation(rng, input.min_range, input.min_range_deviation));
  sampled.max_range = std::max(0.0f, SampleFloatMeanDeviation(rng, input.max_range, input.max_range_deviation));
  if (sampled.max_range > 0.0f && sampled.max_range < sampled.min_range) {
    std::swap(sampled.max_range, sampled.min_range);
  }
  sampled.bounding_box_limit = std::max(0.0f, SampleFloatMeanDeviation(
      rng, input.bounding_box_limit, input.bounding_box_limit_deviation));
  return sampled;
}

TasselPointCloudGridCaptureSettings SampleCaptureSettingsForScan(
    const TasselPointCloudGridCaptureSettings& input, std::mt19937& rng) {
  TasselPointCloudGridCaptureSettings sampled = input;

  sampled.bounding_box_size = std::max(0.001f, SampleFloatMeanDeviation(
      rng, input.bounding_box_size, input.bounding_box_size_deviation));
  sampled.grid_size.x = std::max(1, SampleIntMeanDeviation(rng, input.grid_size.x, input.grid_size_deviation.x));
  sampled.grid_size.y = std::max(1, SampleIntMeanDeviation(rng, input.grid_size.y, input.grid_size_deviation.y));
  sampled.grid_distance = std::max(0.0001f, SampleFloatMeanDeviation(
      rng, input.grid_distance, input.grid_distance_deviation));
  sampled.step = std::max(0.0001f, SampleFloatMeanDeviation(rng, input.step, input.step_deviation));
  sampled.samples_per_step = std::max(1, SampleIntMeanDeviation(
      rng, input.samples_per_step, input.samples_per_step_deviation));
  sampled.sample_height = SampleFloatMeanDeviation(rng, input.sample_height, input.sample_height_deviation);
  sampled.scan_center = SampleVec3MeanDeviation(rng, input.scan_center, input.scan_center_deviation);
  sampled.look_target_height = SampleFloatMeanDeviation(
      rng, input.look_target_height, input.look_target_height_deviation);

  sampled.scanner_angles.clear();
  sampled.scanner_angle_deviations.clear();
  const size_t angle_count = std::max(input.scanner_angles.size(), input.scanner_angle_deviations.size());
  if (angle_count == 0) {
    sampled.scanner_angles.push_back(30.0f);
    sampled.scanner_angle_deviations.push_back(0.0f);
  } else {
    sampled.scanner_angles.reserve(angle_count);
    sampled.scanner_angle_deviations.reserve(angle_count);
    for (size_t i = 0; i < angle_count; i++) {
      const float mean = i < input.scanner_angles.size() ? input.scanner_angles[i] : 30.0f;
      const float deviation = i < input.scanner_angle_deviations.size() ? input.scanner_angle_deviations[i] : 0.0f;
      sampled.scanner_angles.push_back(glm::clamp(SampleFloatMeanDeviation(rng, mean, std::max(0.0f, deviation)),
                                                  -89.0f, 89.0f));
      sampled.scanner_angle_deviations.push_back(0.0f);
    }
  }

  sampled.scanner_distance = std::max(0.01f, SampleFloatMeanDeviation(
      rng, input.scanner_distance, input.scanner_distance_deviation));
  sampled.pitch_angle_start = glm::clamp(
      SampleIntMeanDeviation(rng, input.pitch_angle_start, input.pitch_angle_start_deviation), -89, 89);
  sampled.pitch_angle_end = glm::clamp(
      SampleIntMeanDeviation(rng, input.pitch_angle_end, input.pitch_angle_end_deviation), -89, 89);
  sampled.pitch_angle_step = std::max(
      1, SampleIntMeanDeviation(rng, input.pitch_angle_step, input.pitch_angle_step_deviation));
  sampled.turn_angle_start = glm::clamp(
      SampleIntMeanDeviation(rng, input.turn_angle_start, input.turn_angle_start_deviation), 0, 360);
  sampled.turn_angle_end = glm::clamp(
      SampleIntMeanDeviation(rng, input.turn_angle_end, input.turn_angle_end_deviation), 0, 360);
  sampled.turn_angle_step = std::max(
      1, SampleIntMeanDeviation(rng, input.turn_angle_step, input.turn_angle_step_deviation));
  sampled.fov = glm::clamp(SampleFloatMeanDeviation(rng, input.fov, input.fov_deviation), 1.0f, 179.0f);
  sampled.scan_resolution = std::max(
      2, SampleIntMeanDeviation(rng, input.scan_resolution, input.scan_resolution_deviation));

  if (sampled.pitch_angle_end <= sampled.pitch_angle_start) {
    sampled.pitch_angle_end = std::min(89, sampled.pitch_angle_start + 1);
  }
  if (sampled.turn_angle_end <= sampled.turn_angle_start) {
    sampled.turn_angle_end = std::min(360, sampled.turn_angle_start + 1);
  }
  return sampled;
}

struct SampledVisualScanSettings {
  float playback_speed = 1.0f;
  float view_dwell_seconds = 0.08f;
  float transition_seconds = 0.03f;
  int beam_stride = 32;
  int max_beams_per_view = 2048;
  float beam_width = 0.003f;
  float beam_alpha = 0.75f;
  float beam_speed = 6.0f;
  float beam_fire_fraction = 0.35f;
  float beam_max_length = 1.5f;
  int max_heat_points = 20000;
  int max_hits_per_view = 1024;
  float point_size = 0.012f;
  float scanner_size = 0.03f;
  float hot_seconds = 0.15f;
  float warm_seconds = 0.35f;
  float cool_seconds = 1.25f;
};

SampledVisualScanSettings SampleVisualScanSettings(const TasselPointCloudScannerDescriptor& descriptor) {
  std::mt19937 rng(MixSeed(descriptor.scan_seed, kVisualSettingsSeedSalt));
  SampledVisualScanSettings sampled;
  sampled.playback_speed = std::max(0.01f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_playback_speed, descriptor.visual_scan_playback_speed_deviation));
  sampled.view_dwell_seconds = std::max(0.01f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_view_dwell_seconds, descriptor.visual_scan_view_dwell_seconds_deviation));
  sampled.transition_seconds = std::max(0.0f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_transition_seconds, descriptor.visual_scan_transition_seconds_deviation));
  sampled.beam_stride = std::max(1, SampleIntMeanDeviation(
      rng, descriptor.visual_scan_beam_stride, descriptor.visual_scan_beam_stride_deviation));
  sampled.max_beams_per_view = std::max(1, SampleIntMeanDeviation(
      rng, descriptor.visual_scan_max_beams_per_view, descriptor.visual_scan_max_beams_per_view_deviation));
  sampled.beam_width = std::max(0.0001f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_beam_width, descriptor.visual_scan_beam_width_deviation));
  sampled.beam_alpha = glm::clamp(SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_beam_alpha, descriptor.visual_scan_beam_alpha_deviation), 0.0f, 1.0f);
  sampled.beam_speed = std::max(0.001f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_beam_speed, descriptor.visual_scan_beam_speed_deviation));
  sampled.beam_fire_fraction = glm::clamp(SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_beam_fire_fraction, descriptor.visual_scan_beam_fire_fraction_deviation), 0.0f,
      1.0f);
  sampled.beam_max_length = std::max(0.01f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_beam_max_length, descriptor.visual_scan_beam_max_length_deviation));
  sampled.max_heat_points = std::max(1, SampleIntMeanDeviation(
      rng, descriptor.visual_scan_max_heat_points, descriptor.visual_scan_max_heat_points_deviation));
  sampled.max_hits_per_view = std::max(1, SampleIntMeanDeviation(
      rng, descriptor.visual_scan_max_hits_per_view, descriptor.visual_scan_max_hits_per_view_deviation));
  sampled.point_size = std::max(0.0001f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_point_size, descriptor.visual_scan_point_size_deviation));
  sampled.scanner_size = std::max(0.0001f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_scanner_size, descriptor.visual_scan_scanner_size_deviation));
  sampled.hot_seconds = std::max(0.01f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_hot_seconds, descriptor.visual_scan_hot_seconds_deviation));
  sampled.warm_seconds = std::max(0.01f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_warm_seconds, descriptor.visual_scan_warm_seconds_deviation));
  sampled.cool_seconds = std::max(0.01f, SampleFloatMeanDeviation(
      rng, descriptor.visual_scan_cool_seconds, descriptor.visual_scan_cool_seconds_deviation));
  return sampled;
}

}

// ---------------------------------------------------------------------------
// TasselPointCloudPointSettings
// ---------------------------------------------------------------------------

bool TasselPointCloudPointSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNodeEx("Distance/Hit Jitter", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (BeginMeanDeviationTable("PointDistanceHitJitterTable")) {
      changed |= InspectFloatMeanDeviation(
          "Distance Sigma Scale", distance_sigma_scale, distance_sigma_scale_deviation,
          0.001f, 0.0f, 0.1f, 0.0f, 0.1f,
          "Gaussian perturbation scale applied relative to ray distance.");
      changed |= InspectFloatMeanDeviation(
          "Hit Ball Jitter Radius", hit_ball_jitter_radius, hit_ball_jitter_radius_deviation,
          0.001f, 0.0f, 0.1f, 0.0f, 0.1f,
          "Uniform random offset radius sampled in a unit ball around each hit point.");
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("LIDAR Noise Model")) {
    if (BeginMeanDeviationTable("PointLidarNoiseModelTable")) {
      changed |= InspectFloatMeanDeviation(
          "Range Noise Base Sigma", range_noise_base_sigma, range_noise_base_sigma_deviation,
          0.0001f, 0.0f, 0.1f, 0.0f, 0.1f,
          "Base standard deviation for range noise, independent of distance.");
      changed |= InspectFloatMeanDeviation(
          "Range Noise Scale", range_noise_scale, range_noise_scale_deviation,
          0.0001f, 0.0f, 0.1f, 0.0f, 0.1f,
          "Distance-dependent range-noise scale. Sigma = base + scale * distance.");
      changed |= InspectFloatMeanDeviation(
          "Dropout Probability", dropout_probability, dropout_probability_deviation,
          0.01f, 0.0f, 1.0f, 0.0f, 1.0f,
          "Per-ray chance that a valid return is dropped.");
      changed |= InspectFloatMeanDeviation(
          "Angular Noise Sigma (rad)", angular_noise_sigma, angular_noise_sigma_deviation,
          0.0001f, 0.0f, 0.1f, 0.0f, 0.1f,
          "Angular jitter standard deviation (radians) applied to ray direction.");
      changed |= InspectFloatMeanDeviation(
          "Min Range", min_range, min_range_deviation,
          0.01f, 0.0f, 100.0f, 0.0f, 100.0f,
          "Discard hits closer than this distance from scanner origin.");
      changed |= InspectFloatMeanDeviation(
          "Max Range (0=inf)", max_range, max_range_deviation,
          0.1f, 0.0f, 1000.0f, 0.0f, 1000.0f,
          "Discard hits beyond this distance. 0 means no upper limit.");
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("Color Output", &color_output))
    changed = true;
  ShowItemHoverDescription("Write per-point RGB hit color into point cloud output.");
  if (ImGui::Checkbox("Type Index", &type_index))
    changed = true;
  ShowItemHoverDescription("Store semantic type index per point (stem/spikelet). ");
  if (ImGui::Checkbox("Instance Index", &instance_index))
    changed = true;
  ShowItemHoverDescription("Store per-instance identifier for source geometry.");
  if (BeginMeanDeviationTable("PointBoundsTable")) {
    changed |= InspectFloatMeanDeviation(
        "Bounding Box Limit", bounding_box_limit, bounding_box_limit_deviation,
        0.1f, 0.0f, 10.0f, 0.0f, 10.0f,
        "Extra world-space expansion around detected plant bounds used when filtering accepted hits.");
    ImGui::EndTable();
  }

  distance_sigma_scale = std::max(0.0f, distance_sigma_scale);
  distance_sigma_scale_deviation = std::max(0.0f, distance_sigma_scale_deviation);
  hit_ball_jitter_radius = std::max(0.0f, hit_ball_jitter_radius);
  hit_ball_jitter_radius_deviation = std::max(0.0f, hit_ball_jitter_radius_deviation);
  range_noise_base_sigma = std::max(0.0f, range_noise_base_sigma);
  range_noise_base_sigma_deviation = std::max(0.0f, range_noise_base_sigma_deviation);
  range_noise_scale = std::max(0.0f, range_noise_scale);
  range_noise_scale_deviation = std::max(0.0f, range_noise_scale_deviation);
  dropout_probability = glm::clamp(dropout_probability, 0.0f, 1.0f);
  dropout_probability_deviation = std::max(0.0f, dropout_probability_deviation);
  angular_noise_sigma = std::max(0.0f, angular_noise_sigma);
  angular_noise_sigma_deviation = std::max(0.0f, angular_noise_sigma_deviation);
  min_range = std::max(0.0f, min_range);
  min_range_deviation = std::max(0.0f, min_range_deviation);
  max_range = std::max(0.0f, max_range);
  max_range_deviation = std::max(0.0f, max_range_deviation);
  bounding_box_limit = std::max(0.0f, bounding_box_limit);
  bounding_box_limit_deviation = std::max(0.0f, bounding_box_limit_deviation);
  return changed;
}

void TasselPointCloudPointSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;
  SaveFloatMeanDeviation(out, "distance_sigma_scale", distance_sigma_scale, distance_sigma_scale_deviation);
  SaveFloatMeanDeviation(out, "hit_ball_jitter_radius", hit_ball_jitter_radius, hit_ball_jitter_radius_deviation);
  SaveFloatMeanDeviation(out, "range_noise_base_sigma", range_noise_base_sigma, range_noise_base_sigma_deviation);
  SaveFloatMeanDeviation(out, "range_noise_scale", range_noise_scale, range_noise_scale_deviation);
  SaveFloatMeanDeviation(out, "dropout_probability", dropout_probability, dropout_probability_deviation);
  SaveFloatMeanDeviation(out, "angular_noise_sigma", angular_noise_sigma, angular_noise_sigma_deviation);
  SaveFloatMeanDeviation(out, "min_range", min_range, min_range_deviation);
  SaveFloatMeanDeviation(out, "max_range", max_range, max_range_deviation);
  out << YAML::Key << "color_output" << YAML::Value << color_output;
  out << YAML::Key << "type_index" << YAML::Value << type_index;
  out << YAML::Key << "instance_index" << YAML::Value << instance_index;
  SaveFloatMeanDeviation(out, "bounding_box_limit", bounding_box_limit, bounding_box_limit_deviation);
  out << YAML::EndMap;
}

void TasselPointCloudPointSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& n = in[name];
    LoadFloatMeanDeviation(n, "distance_sigma_scale", distance_sigma_scale, distance_sigma_scale_deviation);
    LoadFloatMeanDeviation(n, "hit_ball_jitter_radius", hit_ball_jitter_radius, hit_ball_jitter_radius_deviation);
    LoadFloatMeanDeviation(n, "range_noise_base_sigma", range_noise_base_sigma, range_noise_base_sigma_deviation);
    LoadFloatMeanDeviation(n, "range_noise_scale", range_noise_scale, range_noise_scale_deviation);
    LoadFloatMeanDeviation(n, "dropout_probability", dropout_probability, dropout_probability_deviation);
    LoadFloatMeanDeviation(n, "angular_noise_sigma", angular_noise_sigma, angular_noise_sigma_deviation);
    LoadFloatMeanDeviation(n, "min_range", min_range, min_range_deviation);
    LoadFloatMeanDeviation(n, "max_range", max_range, max_range_deviation);
    if (n["color_output"]) color_output = n["color_output"].as<bool>();
    if (n["type_index"]) type_index = n["type_index"].as<bool>();
    if (n["instance_index"]) instance_index = n["instance_index"].as<bool>();
    LoadFloatMeanDeviation(n, "bounding_box_limit", bounding_box_limit, bounding_box_limit_deviation);

    distance_sigma_scale = std::max(0.0f, distance_sigma_scale);
    distance_sigma_scale_deviation = std::max(0.0f, distance_sigma_scale_deviation);
    hit_ball_jitter_radius = std::max(0.0f, hit_ball_jitter_radius);
    hit_ball_jitter_radius_deviation = std::max(0.0f, hit_ball_jitter_radius_deviation);
    range_noise_base_sigma = std::max(0.0f, range_noise_base_sigma);
    range_noise_base_sigma_deviation = std::max(0.0f, range_noise_base_sigma_deviation);
    range_noise_scale = std::max(0.0f, range_noise_scale);
    range_noise_scale_deviation = std::max(0.0f, range_noise_scale_deviation);
    dropout_probability = glm::clamp(dropout_probability, 0.0f, 1.0f);
    dropout_probability_deviation = std::max(0.0f, dropout_probability_deviation);
    angular_noise_sigma = std::max(0.0f, angular_noise_sigma);
    angular_noise_sigma_deviation = std::max(0.0f, angular_noise_sigma_deviation);
    min_range = std::max(0.0f, min_range);
    min_range_deviation = std::max(0.0f, min_range_deviation);
    max_range = std::max(0.0f, max_range);
    max_range_deviation = std::max(0.0f, max_range_deviation);
    bounding_box_limit = std::max(0.0f, bounding_box_limit);
    bounding_box_limit_deviation = std::max(0.0f, bounding_box_limit_deviation);
  }
}

// ---------------------------------------------------------------------------
// TasselPointCloudGridCaptureSettings
// ---------------------------------------------------------------------------

bool TasselPointCloudGridCaptureSettings::OnInspect() {
  bool changed = false;

  const char* mode_names[] = {"Hemisphere", "Gantry", "Circular"};
  int mode_int = static_cast<int>(scan_mode);
  if (ImGui::Combo("Scan Mode", &mode_int, mode_names, IM_ARRAYSIZE(mode_names))) {
    scan_mode = static_cast<TasselScanMode>(mode_int);
    changed = true;
  }
  ShowItemHoverDescription(
      "Selects scanner path generation: Hemisphere (random downward rays), Gantry (tilted sweeps), or Circular (camera-like orbital views).");

  if (BeginMeanDeviationTable("CaptureCommonTable")) {
    changed |= InspectIntMeanDeviation(
        "Grid Size X", grid_size.x, grid_size_deviation.x,
        1.0f, 1, 100, 0, 100,
        "Number of scanner positions along X in the scan area.");
    changed |= InspectIntMeanDeviation(
        "Grid Size Z", grid_size.y, grid_size_deviation.y,
        1.0f, 1, 100, 0, 100,
        "Number of scanner positions along Z in the scan area.");
    changed |= InspectFloatMeanDeviation(
        "Grid Distance", grid_distance, grid_distance_deviation,
        0.1f, 0.0f, 100.0f, 0.0f, 100.0f,
        "World-space spacing between neighboring grid centers.");
    changed |= InspectFloatMeanDeviation(
        "Step", step, step_deviation,
        0.001f, 0.0f, 0.5f, 0.0f, 0.5f,
        "Sub-grid sampling step in world units.");
    changed |= InspectFloatMeanDeviation(
        "Sample Height", sample_height, sample_height_deviation,
        0.1f, 0.0f, 10.0f, 0.0f, 10.0f,
        "Base scanner Y height in world units before per-mode offsets are applied.");
    changed |= InspectVec3MeanDeviation(
        "Scan Center", scan_center, scan_center_deviation,
        0.01f, -1000.0f, 1000.0f, 0.0f, 1000.0f,
        "World-space scanner center offset. Deviation is per-axis.");
    changed |= InspectFloatMeanDeviation(
        "Look Target Height", look_target_height, look_target_height_deviation,
        0.01f, -1000.0f, 1000.0f, 0.0f, 1000.0f,
        "World-space Y target used by circular mode look-at.");
    changed |= InspectFloatMeanDeviation(
        "Bounding Box Size", bounding_box_size, bounding_box_size_deviation,
        0.01f, 0.0f, 1000.0f, 0.0f, 1000.0f,
        "Half extent for sample-space bound filtering.");
    ImGui::EndTable();
  }

  if (scan_mode == TasselScanMode::Hemisphere) {
    if (BeginMeanDeviationTable("CaptureHemisphereTable")) {
      changed |= InspectIntMeanDeviation(
          "Samples per Step", samples_per_step, samples_per_step_deviation,
          1.0f, 1, 4096, 0, 4096,
          "Rays spawned per scanner origin in Hemisphere mode.");
      ImGui::EndTable();
    }
  } else if (scan_mode == TasselScanMode::Gantry) {
    // Show scanner_angles list (mean/deviation per angle entry).
    const bool scanner_angles_open = ImGui::TreeNodeEx("Scanner Angles");
    ShowItemHoverDescription(
        "Tilt angles (degrees from vertical) used by Gantry passes. Each angle generates forward and backward sweeps.");
    if (scanner_angles_open) {
      const size_t angle_count = std::max(scanner_angles.size(), scanner_angle_deviations.size());
      if (angle_count == 0) {
        scanner_angles.push_back(30.0f);
        scanner_angle_deviations.push_back(0.0f);
      }
      scanner_angles.resize(std::max<size_t>(1, angle_count), 30.0f);
      scanner_angle_deviations.resize(std::max<size_t>(1, angle_count), 0.0f);
      int remove_index = -1;
      if (BeginMeanDeviationTable("CaptureGantryAnglesTable")) {
        for (size_t i = 0; i < scanner_angles.size(); i++) {
          ImGui::PushID(static_cast<int>(i));

          ImGui::TableNextRow();

          ImGui::TableSetColumnIndex(0);
          ImGui::AlignTextToFramePadding();
          const std::string angle_label = "Angle " + std::to_string(i + 1);
          ImGui::TextUnformatted(angle_label.c_str());
          ShowItemHoverDescription(
              "Per-pass gantry tilt angle in degrees. 0 looks straight down; larger magnitudes increase obliqueness.");
          if (scanner_angles.size() > 1) {
            ImGui::SameLine();
            if (ImGui::SmallButton("-")) {
              remove_index = static_cast<int>(i);
            }
            ShowItemHoverDescription("Remove this gantry angle entry.");
          }

          ImGui::TableSetColumnIndex(1);
          if (ImGui::DragFloat("##mean", &scanner_angles[i], 1.0f, -90.0f, 90.0f)) {
            changed = true;
          }
          ShowItemHoverDescription(
              "Per-pass gantry tilt angle in degrees. 0 looks straight down; larger magnitudes increase obliqueness.");

          ImGui::TableSetColumnIndex(2);
          if (ImGui::DragFloat("##deviation", &scanner_angle_deviations[i], 1.0f, 0.0f, 90.0f)) {
            changed = true;
          }
          ShowItemHoverDescription(
              "Per-pass gantry tilt angle deviation (degrees). Sampled each scan run.");

          ImGui::PopID();
        }
        ImGui::EndTable();
      }

      if (remove_index >= 0) {
        scanner_angles.erase(scanner_angles.begin() + static_cast<ptrdiff_t>(remove_index));
        scanner_angle_deviations.erase(scanner_angle_deviations.begin() + static_cast<ptrdiff_t>(remove_index));
        changed = true;
      }
      if (ImGui::SmallButton("+ Add Angle")) {
        scanner_angles.push_back(30.f);
        scanner_angle_deviations.push_back(0.0f);
        changed = true;
      }
      ShowItemHoverDescription("Add another gantry angle entry.");
      ImGui::TreePop();
    }
  } else if (scan_mode == TasselScanMode::Circular) {
    if (BeginMeanDeviationTable("CaptureCircularTable")) {
      changed |= InspectFloatMeanDeviation(
          "Scanner Distance", scanner_distance, scanner_distance_deviation,
          0.01f, 0.01f, 10.0f, 0.0f, 10.0f,
          "Orbit radius from scan center in world units for Circular mode.");
      changed |= InspectIntMeanDeviation(
          "Pitch Start", pitch_angle_start, pitch_angle_start_deviation,
          1.0f, -90, 90, 0, 90,
          "Starting elevation angle (degrees) for Circular mode.");
      changed |= InspectIntMeanDeviation(
          "Pitch End", pitch_angle_end, pitch_angle_end_deviation,
          1.0f, -90, 90, 0, 90,
          "Ending elevation angle (degrees) for Circular mode.");
      changed |= InspectIntMeanDeviation(
          "Pitch Step", pitch_angle_step, pitch_angle_step_deviation,
          1.0f, 1, 90, 0, 90,
          "Elevation increment (degrees) between pitch samples.");
      changed |= InspectIntMeanDeviation(
          "Turn Start", turn_angle_start, turn_angle_start_deviation,
          1.0f, 0, 360, 0, 360,
          "Starting azimuth angle (degrees) for Circular mode.");
      changed |= InspectIntMeanDeviation(
          "Turn End", turn_angle_end, turn_angle_end_deviation,
          1.0f, 0, 360, 0, 360,
          "Ending azimuth angle (degrees) for Circular mode.");
      changed |= InspectIntMeanDeviation(
          "Turn Step", turn_angle_step, turn_angle_step_deviation,
          1.0f, 1, 360, 0, 360,
          "Azimuth increment (degrees) between circular viewpoints.");
      changed |= InspectFloatMeanDeviation(
          "FOV", fov, fov_deviation,
          1.0f, 1.0f, 180.0f, 0.0f, 180.0f,
          "Virtual camera field-of-view in degrees used to emit rays.");
      changed |= InspectIntMeanDeviation(
          "Scan Resolution", scan_resolution, scan_resolution_deviation,
          1.0f, 2, 4096, 0, 4096,
          "Per-view ray grid resolution (scan_resolution x scan_resolution).");
      ImGui::EndTable();
    }
  }

  bounding_box_size = std::max(0.001f, bounding_box_size);
  bounding_box_size_deviation = std::max(0.0f, bounding_box_size_deviation);
  grid_size = glm::max(grid_size, glm::ivec2(1));
  grid_size_deviation = glm::max(grid_size_deviation, glm::ivec2(0));
  grid_distance = std::max(0.0001f, grid_distance);
  grid_distance_deviation = std::max(0.0f, grid_distance_deviation);
  step = std::max(0.0001f, step);
  step_deviation = std::max(0.0f, step_deviation);
  samples_per_step = std::max(1, samples_per_step);
  samples_per_step_deviation = std::max(0, samples_per_step_deviation);
  sample_height_deviation = std::max(0.0f, sample_height_deviation);
  scan_center_deviation = glm::max(scan_center_deviation, glm::vec3(0.0f));
  look_target_height_deviation = std::max(0.0f, look_target_height_deviation);
  scanner_distance = std::max(0.01f, scanner_distance);
  scanner_distance_deviation = std::max(0.0f, scanner_distance_deviation);
  pitch_angle_start = glm::clamp(pitch_angle_start, -90, 90);
  pitch_angle_end = glm::clamp(pitch_angle_end, -90, 90);
  pitch_angle_start_deviation = std::max(0, pitch_angle_start_deviation);
  pitch_angle_end_deviation = std::max(0, pitch_angle_end_deviation);
  pitch_angle_step = std::max(1, pitch_angle_step);
  pitch_angle_step_deviation = std::max(0, pitch_angle_step_deviation);
  turn_angle_start = glm::clamp(turn_angle_start, 0, 360);
  turn_angle_end = glm::clamp(turn_angle_end, 0, 360);
  turn_angle_start_deviation = std::max(0, turn_angle_start_deviation);
  turn_angle_end_deviation = std::max(0, turn_angle_end_deviation);
  turn_angle_step = std::max(1, turn_angle_step);
  turn_angle_step_deviation = std::max(0, turn_angle_step_deviation);
  fov = glm::clamp(fov, 1.0f, 180.0f);
  fov_deviation = std::max(0.0f, fov_deviation);
  scan_resolution = std::max(2, scan_resolution);
  scan_resolution_deviation = std::max(0, scan_resolution_deviation);
  if (pitch_angle_end <= pitch_angle_start) {
    pitch_angle_end = std::min(90, pitch_angle_start + 1);
  }
  if (turn_angle_end <= turn_angle_start) {
    turn_angle_end = std::min(360, turn_angle_start + 1);
  }
  scanner_angle_deviations.resize(scanner_angles.size(), 0.0f);
  for (size_t i = 0; i < scanner_angles.size(); i++) {
    scanner_angles[i] = glm::clamp(scanner_angles[i], -90.0f, 90.0f);
    scanner_angle_deviations[i] = std::max(0.0f, scanner_angle_deviations[i]);
  }

  return changed;
}

void TasselPointCloudGridCaptureSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "capture_mode" << YAML::Value << static_cast<int>(capture_mode);
  out << YAML::Key << "output_spline_info" << YAML::Value << output_spline_info;
  out << YAML::Key << "spline_subdivision_count" << YAML::Value << spline_subdivision_count;
  SaveFloatMeanDeviation(out, "bounding_box_size", bounding_box_size, bounding_box_size_deviation);
  out << YAML::Key << "grid_size" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "mean" << YAML::Value << grid_size;
  out << YAML::Key << "deviation" << YAML::Value << grid_size_deviation;
  out << YAML::EndMap;
  SaveFloatMeanDeviation(out, "grid_distance", grid_distance, grid_distance_deviation);
  SaveFloatMeanDeviation(out, "step", step, step_deviation);
  SaveIntMeanDeviation(out, "samples_per_step", samples_per_step, samples_per_step_deviation);
  SaveFloatMeanDeviation(out, "sample_height", sample_height, sample_height_deviation);
  SaveVec3MeanDeviation(out, "scan_center", scan_center, scan_center_deviation);
  SaveFloatMeanDeviation(out, "look_target_height", look_target_height, look_target_height_deviation);
  out << YAML::Key << "scan_mode" << YAML::Value << static_cast<int>(scan_mode);
  out << YAML::Key << "scanner_angles" << YAML::Value << YAML::BeginSeq;
  {
    const size_t angle_count = std::max(scanner_angles.size(), scanner_angle_deviations.size());
    for (size_t i = 0; i < angle_count; i++) {
      const float angle_mean = i < scanner_angles.size() ? scanner_angles[i] : 30.0f;
      const float angle_deviation = i < scanner_angle_deviations.size() ? scanner_angle_deviations[i] : 0.0f;
      out << YAML::BeginMap;
      out << YAML::Key << "mean" << YAML::Value << angle_mean;
      out << YAML::Key << "deviation" << YAML::Value << angle_deviation;
      out << YAML::EndMap;
    }
  }
  out << YAML::EndSeq;
  SaveFloatMeanDeviation(out, "scanner_distance", scanner_distance, scanner_distance_deviation);
  SaveIntMeanDeviation(out, "pitch_angle_start", pitch_angle_start, pitch_angle_start_deviation);
  SaveIntMeanDeviation(out, "pitch_angle_end", pitch_angle_end, pitch_angle_end_deviation);
  SaveIntMeanDeviation(out, "pitch_angle_step", pitch_angle_step, pitch_angle_step_deviation);
  SaveIntMeanDeviation(out, "turn_angle_start", turn_angle_start, turn_angle_start_deviation);
  SaveIntMeanDeviation(out, "turn_angle_end", turn_angle_end, turn_angle_end_deviation);
  SaveIntMeanDeviation(out, "turn_angle_step", turn_angle_step, turn_angle_step_deviation);
  SaveFloatMeanDeviation(out, "fov", fov, fov_deviation);
  SaveIntMeanDeviation(out, "scan_resolution", scan_resolution, scan_resolution_deviation);
  out << YAML::EndMap;
}

void TasselPointCloudGridCaptureSettings::Load(const std::string& name, const YAML::Node& in) {
  if (!in[name]) {
    return;
  }
  const auto& n = in[name];
  if (n["capture_mode"]) {
    const auto mode = n["capture_mode"].as<int>();
    if (mode == static_cast<int>(CaptureMode::Cpu) || mode == static_cast<int>(CaptureMode::Gpu)) {
      capture_mode = static_cast<CaptureMode>(mode);
    }
  }
  if (n["output_spline_info"]) output_spline_info = n["output_spline_info"].as<bool>();
  if (n["spline_subdivision_count"]) spline_subdivision_count = n["spline_subdivision_count"].as<uint32_t>();
  LoadFloatMeanDeviation(n, "bounding_box_size", bounding_box_size, bounding_box_size_deviation);
  if (n["grid_size"] && n["grid_size"].IsMap()) {
    const auto& grid_node = n["grid_size"];
    if (grid_node["mean"]) {
      grid_size = grid_node["mean"].as<glm::ivec2>();
    }
    if (grid_node["deviation"]) {
      grid_size_deviation = grid_node["deviation"].as<glm::ivec2>();
    }
  }
  LoadFloatMeanDeviation(n, "grid_distance", grid_distance, grid_distance_deviation);
  LoadFloatMeanDeviation(n, "step", step, step_deviation);
  LoadIntMeanDeviation(n, "samples_per_step", samples_per_step, samples_per_step_deviation);
  LoadFloatMeanDeviation(n, "sample_height", sample_height, sample_height_deviation);
  LoadVec3MeanDeviation(n, "scan_center", scan_center, scan_center_deviation);
  LoadFloatMeanDeviation(n, "look_target_height", look_target_height, look_target_height_deviation);
  if (n["scan_mode"]) {
    const auto mode = n["scan_mode"].as<int>();
    if (mode >= static_cast<int>(TasselScanMode::Hemisphere) && mode <= static_cast<int>(TasselScanMode::Circular)) {
      scan_mode = static_cast<TasselScanMode>(mode);
    }
  }
  if (n["scanner_angles"] && n["scanner_angles"].IsSequence()) {
    scanner_angles.clear();
    scanner_angle_deviations.clear();
    for (const auto& angle_node : n["scanner_angles"]) {
      if (!angle_node.IsMap()) {
        continue;
      }
      const float mean = angle_node["mean"] ? angle_node["mean"].as<float>() : 30.0f;
      const float deviation = angle_node["deviation"] ? angle_node["deviation"].as<float>() : 0.0f;
      scanner_angles.push_back(mean);
      scanner_angle_deviations.push_back(deviation);
    }
  }
  LoadFloatMeanDeviation(n, "scanner_distance", scanner_distance, scanner_distance_deviation);
  LoadIntMeanDeviation(n, "pitch_angle_start", pitch_angle_start, pitch_angle_start_deviation);
  LoadIntMeanDeviation(n, "pitch_angle_end", pitch_angle_end, pitch_angle_end_deviation);
  LoadIntMeanDeviation(n, "pitch_angle_step", pitch_angle_step, pitch_angle_step_deviation);
  LoadIntMeanDeviation(n, "turn_angle_start", turn_angle_start, turn_angle_start_deviation);
  LoadIntMeanDeviation(n, "turn_angle_end", turn_angle_end, turn_angle_end_deviation);
  LoadIntMeanDeviation(n, "turn_angle_step", turn_angle_step, turn_angle_step_deviation);
  LoadFloatMeanDeviation(n, "fov", fov, fov_deviation);
  LoadIntMeanDeviation(n, "scan_resolution", scan_resolution, scan_resolution_deviation);

  grid_size = glm::max(grid_size, glm::ivec2(1));
  grid_size_deviation = glm::max(grid_size_deviation, glm::ivec2(0));
  bounding_box_size = std::max(0.001f, bounding_box_size);
  bounding_box_size_deviation = std::max(0.0f, bounding_box_size_deviation);
  grid_distance = std::max(0.0001f, grid_distance);
  grid_distance_deviation = std::max(0.0f, grid_distance_deviation);
  step = std::max(0.0001f, step);
  step_deviation = std::max(0.0f, step_deviation);
  samples_per_step = std::max(1, samples_per_step);
  samples_per_step_deviation = std::max(0, samples_per_step_deviation);
  sample_height_deviation = std::max(0.0f, sample_height_deviation);
  scan_center_deviation = glm::max(scan_center_deviation, glm::vec3(0.0f));
  look_target_height_deviation = std::max(0.0f, look_target_height_deviation);
  scanner_distance = std::max(0.01f, scanner_distance);
  scanner_distance_deviation = std::max(0.0f, scanner_distance_deviation);
  pitch_angle_start = glm::clamp(pitch_angle_start, -90, 90);
  pitch_angle_end = glm::clamp(pitch_angle_end, -90, 90);
  pitch_angle_start_deviation = std::max(0, pitch_angle_start_deviation);
  pitch_angle_end_deviation = std::max(0, pitch_angle_end_deviation);
  pitch_angle_step = std::max(1, pitch_angle_step);
  pitch_angle_step_deviation = std::max(0, pitch_angle_step_deviation);
  turn_angle_start = glm::clamp(turn_angle_start, 0, 360);
  turn_angle_end = glm::clamp(turn_angle_end, 0, 360);
  turn_angle_start_deviation = std::max(0, turn_angle_start_deviation);
  turn_angle_end_deviation = std::max(0, turn_angle_end_deviation);
  turn_angle_step = std::max(1, turn_angle_step);
  turn_angle_step_deviation = std::max(0, turn_angle_step_deviation);
  fov = glm::clamp(fov, 1.0f, 180.0f);
  fov_deviation = std::max(0.0f, fov_deviation);
  scan_resolution = std::max(2, scan_resolution);
  scan_resolution_deviation = std::max(0, scan_resolution_deviation);
  if (pitch_angle_end <= pitch_angle_start) {
    pitch_angle_end = std::min(90, pitch_angle_start + 1);
  }
  if (turn_angle_end <= turn_angle_start) {
    turn_angle_end = std::min(360, turn_angle_start + 1);
  }
  if (scanner_angles.empty()) {
    scanner_angles.push_back(30.f);
  }
  scanner_angle_deviations.resize(scanner_angles.size(), 0.0f);
  for (size_t i = 0; i < scanner_angles.size(); i++) {
    scanner_angles[i] = glm::clamp(scanner_angles[i], -90.0f, 90.0f);
    scanner_angle_deviations[i] = std::max(0.0f, scanner_angle_deviations[i]);
  }
}

void TasselPointCloudGridCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) {
  switch (scan_mode) {
    case TasselScanMode::Hemisphere:
      GenerateHemisphereSamples(point_cloud_samples);
      break;
    case TasselScanMode::Gantry:
      GenerateGantrySamples(point_cloud_samples);
      break;
    case TasselScanMode::Circular:
      GenerateCircularSamples(point_cloud_samples);
      break;
  }
}

void TasselPointCloudGridCaptureSettings::GenerateHemisphereSamples(
    std::vector<PointCloudSample>& point_cloud_samples) const {
  const glm::vec2 start_point = glm::vec2((static_cast<float>(grid_size.x) * 0.5f - 0.5f) * grid_distance,
                                          (static_cast<float>(grid_size.y) * 0.5f - 0.5f) * grid_distance);

  const int x_step_size = static_cast<int>(grid_size.x * grid_distance / step);
  const int y_step_size = static_cast<int>(grid_size.y * grid_distance / step);

  point_cloud_samples.resize(x_step_size * y_step_size * samples_per_step);

  Jobs::RunParallelFor(x_step_size * y_step_size, [&](unsigned i) {
    const auto x = static_cast<int>(i) / y_step_size;
    const auto y = static_cast<int>(i) % y_step_size;
    const glm::vec3 center =
        scan_center + glm::vec3{step * x, sample_height, step * y} - glm::vec3(start_point.x, 0, start_point.y);
    for (int s = 0; s < samples_per_step; s++) {
      auto& sample = point_cloud_samples[i * samples_per_step + s];
      sample.direction = glm::sphericalRand(1.0f);
      sample.direction.y = -glm::abs(sample.direction.y);
      sample.start = center;
    }
  });
}

void TasselPointCloudGridCaptureSettings::GenerateGantrySamples(
    std::vector<PointCloudSample>& point_cloud_samples) const {
  const glm::vec2 start_point = glm::vec2((static_cast<float>(grid_size.x) * 0.5f - 0.5f) * grid_distance,
                                          (static_cast<float>(grid_size.y) * 0.5f - 0.5f) * grid_distance);

  const int x_step_size = static_cast<int>(grid_size.x * grid_distance / step);
  const int z_step_size = static_cast<int>(grid_size.y * grid_distance / step);
  const int angles_count = static_cast<int>(scanner_angles.size());
  // Two passes per angle (forward/backward along X).
  const int rays_per_position = angles_count * 2;

  point_cloud_samples.resize(x_step_size * z_step_size * rays_per_position);

  Jobs::RunParallelFor(x_step_size * z_step_size, [&](unsigned i) {
    const auto xi = static_cast<int>(i) / z_step_size;
    const auto zi = static_cast<int>(i) % z_step_size;
    const glm::vec3 position =
        scan_center + glm::vec3{step * xi, sample_height, step * zi} - glm::vec3(start_point.x, 0, start_point.y);
    for (int a = 0; a < angles_count; a++) {
      const float angle_rad = glm::radians(scanner_angles[a]);
      // Forward pass.
      {
        auto& sample = point_cloud_samples[i * rays_per_position + a * 2];
        sample.start = position;
        sample.direction = glm::normalize(glm::vec3(glm::sin(angle_rad), -glm::cos(angle_rad), 0.0f));
      }
      // Backward pass.
      {
        auto& sample = point_cloud_samples[i * rays_per_position + a * 2 + 1];
        sample.start = position;
        sample.direction = glm::normalize(glm::vec3(-glm::sin(angle_rad), -glm::cos(angle_rad), 0.0f));
      }
    }
  });
}

void TasselPointCloudGridCaptureSettings::GenerateCircularSamples(
    std::vector<PointCloudSample>& point_cloud_samples) const {
  std::vector<PointCloudSample> all_samples;

  for (int pitch = pitch_angle_start; pitch <= pitch_angle_end; pitch += std::max(1, pitch_angle_step)) {
    for (int turn = turn_angle_start; turn < turn_angle_end; turn += std::max(1, turn_angle_step)) {
      const float pitch_rad = glm::radians(static_cast<float>(pitch));
      const float turn_rad = glm::radians(static_cast<float>(turn));

        const glm::vec3 scanner_pos = scan_center +
                      glm::vec3(scanner_distance * glm::cos(pitch_rad) * glm::sin(turn_rad),
                            scanner_distance * glm::sin(pitch_rad) + sample_height,
                            scanner_distance * glm::cos(pitch_rad) * glm::cos(turn_rad));
        const float look_target_y =
          look_target_height == 0.0f ? sample_height * 0.5f : look_target_height;
        const glm::vec3 look_target(scan_center.x, look_target_y, scan_center.z);
        const glm::vec3 look_dir = glm::normalize(look_target - scanner_pos);
      const glm::vec3 right = glm::normalize(glm::cross(look_dir, glm::vec3(0, 1, 0)));
      const glm::vec3 up = glm::normalize(glm::cross(right, look_dir));

      const float half_fov = glm::radians(fov * 0.5f);
      for (int sx = 0; sx < scan_resolution; sx++) {
        for (int sy = 0; sy < scan_resolution; sy++) {
          const float u = (static_cast<float>(sx) / static_cast<float>(scan_resolution - 1) - 0.5f) * 2.0f;
          const float v = (static_cast<float>(sy) / static_cast<float>(scan_resolution - 1) - 0.5f) * 2.0f;

          PointCloudSample sample{};
          sample.start = scanner_pos;
          sample.direction = glm::normalize(look_dir + glm::tan(half_fov) * (u * right + v * up));
          all_samples.emplace_back(sample);
        }
      }
    }
  }

  point_cloud_samples = std::move(all_samples);
}

bool TasselPointCloudGridCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  return glm::abs(sample.hit_info.position.x) < bounding_box_size &&
         glm::abs(sample.hit_info.position.y) < bounding_box_size &&
         glm::abs(sample.hit_info.position.z) < bounding_box_size;
}

// ---------------------------------------------------------------------------
// TasselPointCloudScannerDescriptor
// ---------------------------------------------------------------------------

TasselPointCloudScannerDescriptor::TasselPointCloudScannerDescriptor() {
  const auto defaults_path = ResolveDefaultTasselScannerDescriptorPath();
  if (!LoadTasselScannerDescriptorDefaultsFromFile(*this, defaults_path)) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      EVOENGINE_WARNING(
          "TasselPointCloudScannerDescriptor defaults file not found or invalid. Using inline member defaults.");
    }
  }
}

bool TasselPointCloudScannerDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  const auto capture_from_selected_scanner = [&](const bool emit_warning) -> bool {
    if (!editor_layer) {
      if (emit_warning) {
        EVOENGINE_WARNING("Capture failed. No editor layer available.");
      }
      return false;
    }

    const auto scene = Application::GetActiveScene();
    if (!scene) {
      if (emit_warning) {
        EVOENGINE_WARNING("Capture failed. No active scene.");
      }
      return false;
    }

    const auto selected_entity = editor_layer->GetSelectedEntity();
    if (!scene->IsEntityValid(selected_entity) || !scene->HasPrivateComponent<TasselPointCloudScanner>(selected_entity)) {
      if (emit_warning) {
        EVOENGINE_WARNING("Capture failed. Select a TasselPointCloudScanner entity.");
      }
      return false;
    }

    const auto scanner = scene->GetOrSetPrivateComponent<TasselPointCloudScanner>(selected_entity).lock();
    if (!scanner) {
      if (emit_warning) {
        EVOENGINE_WARNING("Capture failed. Unable to access selected TasselPointCloudScanner component.");
      }
      return false;
    }

    TasselPointCloudPointSettings captured_point_settings;
    TasselPointCloudGridCaptureSettings captured_capture_settings;
    if (!scanner->TryGetActiveSettings(captured_point_settings, captured_capture_settings)) {
      if (emit_warning) {
        EVOENGINE_WARNING("Capture failed. Selected scanner has no active descriptor settings.");
      }
      return false;
    }

    point_settings = captured_point_settings;
    capture_settings = captured_capture_settings;
    changed = true;
    EVOENGINE_LOG("Captured settings from selected TasselPointCloudScanner into descriptor: " + GetTitle());
    return true;
  };

  if (ImGui::Button("Capture From Selected Scanner")) {
    capture_from_selected_scanner(true);
  }
  ShowItemHoverDescription(
      "Copy active point and capture settings from the selected TasselPointCloudScanner instance into this descriptor.");

  ImGui::SameLine();
  if (ImGui::Button("Overwrite Scanner Defaults")) {
    const bool captured_from_scanner = capture_from_selected_scanner(false);
    if (captured_from_scanner) {
      EVOENGINE_LOG("Using selected scanner settings for defaults overwrite.");
    }

    const auto defaults_path = ResolveWritableTasselScannerDescriptorDefaultsPath();
    if (SaveTasselScannerDescriptorDefaultsToFile(*this, defaults_path)) {
      EVOENGINE_LOG("TasselPointCloudScannerDescriptor defaults overwritten: " + defaults_path.string());
    } else {
      EVOENGINE_WARNING("Failed to overwrite TasselPointCloudScannerDescriptor defaults.");
    }
  }
  if (ImGui::IsItemHovered()) {
    const auto defaults_path = ResolveWritableTasselScannerDescriptorDefaultsPath();
    const std::string tip =
        "Write new defaults for future TasselPointCloudScannerDescriptor assets. If a TasselPointCloudScanner is selected, its active settings are captured first.\nPath: " +
        defaults_path.string();
    ImGui::SetTooltip("%s", tip.c_str());
  }

  if (ImGui::TreeNodeEx("Capture", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (capture_settings.OnInspect()) {
      changed = true;
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Execution", ImGuiTreeNodeFlags_DefaultOpen)) {
    int mode = static_cast<int>(capture_settings.capture_mode);
    if (ImGui::Combo("Capture Backend", &mode, "CPU\0GPU Ray Tracing\0")) {
      capture_settings.capture_mode = static_cast<PointCloudCaptureSettings::CaptureMode>(mode);
      changed = true;
    }
    ShowItemHoverDescription(
        "Selects tracing backend used for capture and playback precompute.");
    if (ImGui::DragInt("Scan Seed", &scan_seed, 1.0f, std::numeric_limits<int>::min(), std::numeric_limits<int>::max())) {
      changed = true;
    }
    ShowItemHoverDescription("Deterministic scanner seed used by angular jitter, dropout/noise, and beam ordering.");
    if (gpu_only_mode) {
      gpu_only_mode = false;
      changed = true;
    }

    ImGui::TextUnformatted(capture_settings.capture_mode == PointCloudCaptureSettings::CaptureMode::Gpu
                               ? "Capture backend: GPU Ray Tracing"
                               : "Capture backend: CPU");
    ImGui::TreePop();
  }

  ImGui::TextUnformatted("In-editor scan playback controls live on the scanner private component.");

  if (ImGui::TreeNodeEx("Point Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (point_settings.OnInspect()) {
      changed = true;
    }
    ImGui::TreePop();
  }

  visual_scan_playback_speed = std::max(0.01f, visual_scan_playback_speed);
  visual_scan_playback_speed_deviation = std::max(0.0f, visual_scan_playback_speed_deviation);
  visual_scan_view_dwell_seconds = std::max(0.01f, visual_scan_view_dwell_seconds);
  visual_scan_view_dwell_seconds_deviation = std::max(0.0f, visual_scan_view_dwell_seconds_deviation);
  visual_scan_transition_seconds = std::max(0.0f, visual_scan_transition_seconds);
  visual_scan_transition_seconds_deviation = std::max(0.0f, visual_scan_transition_seconds_deviation);
  visual_scan_beam_stride = std::max(1, visual_scan_beam_stride);
  visual_scan_beam_stride_deviation = std::max(0, visual_scan_beam_stride_deviation);
  visual_scan_max_beams_per_view = std::max(1, visual_scan_max_beams_per_view);
  visual_scan_max_beams_per_view_deviation = std::max(0, visual_scan_max_beams_per_view_deviation);
  visual_scan_beam_width = std::max(0.0001f, visual_scan_beam_width);
  visual_scan_beam_width_deviation = std::max(0.0f, visual_scan_beam_width_deviation);
  visual_scan_beam_alpha = glm::clamp(visual_scan_beam_alpha, 0.0f, 1.0f);
  visual_scan_beam_alpha_deviation = std::max(0.0f, visual_scan_beam_alpha_deviation);
  visual_scan_beam_speed = std::max(0.001f, visual_scan_beam_speed);
  visual_scan_beam_speed_deviation = std::max(0.0f, visual_scan_beam_speed_deviation);
  visual_scan_beam_fire_fraction = glm::clamp(visual_scan_beam_fire_fraction, 0.0f, 1.0f);
  visual_scan_beam_fire_fraction_deviation = std::max(0.0f, visual_scan_beam_fire_fraction_deviation);
  visual_scan_beam_max_length = std::max(0.01f, visual_scan_beam_max_length);
  visual_scan_beam_max_length_deviation = std::max(0.0f, visual_scan_beam_max_length_deviation);
  visual_scan_max_heat_points = std::max(1, visual_scan_max_heat_points);
  visual_scan_max_heat_points_deviation = std::max(0, visual_scan_max_heat_points_deviation);
  visual_scan_max_hits_per_view = std::max(1, visual_scan_max_hits_per_view);
  visual_scan_max_hits_per_view_deviation = std::max(0, visual_scan_max_hits_per_view_deviation);
  visual_scan_point_size = std::max(0.0001f, visual_scan_point_size);
  visual_scan_point_size_deviation = std::max(0.0f, visual_scan_point_size_deviation);
  visual_scan_scanner_size = std::max(0.0001f, visual_scan_scanner_size);
  visual_scan_scanner_size_deviation = std::max(0.0f, visual_scan_scanner_size_deviation);
  visual_scan_hot_seconds = std::max(0.01f, visual_scan_hot_seconds);
  visual_scan_hot_seconds_deviation = std::max(0.0f, visual_scan_hot_seconds_deviation);
  visual_scan_warm_seconds = std::max(0.01f, visual_scan_warm_seconds);
  visual_scan_warm_seconds_deviation = std::max(0.0f, visual_scan_warm_seconds_deviation);
  visual_scan_cool_seconds = std::max(0.01f, visual_scan_cool_seconds);
  visual_scan_cool_seconds_deviation = std::max(0.0f, visual_scan_cool_seconds_deviation);
  return changed;
}

void TasselPointCloudScannerDescriptor::Serialize(YAML::Emitter& out) const {
  point_settings.Save("point_settings", out);
  capture_settings.Save("capture_settings", out);
  out << YAML::Key << "scan_seed" << YAML::Value << scan_seed;
  out << YAML::Key << "gpu_only_mode" << YAML::Value << false;

  out << YAML::Key << "visual_scan_interactive_enabled" << YAML::Value << visual_scan_interactive_enabled;
  out << YAML::Key << "visual_scan_show_scanner" << YAML::Value << visual_scan_show_scanner;
  out << YAML::Key << "visual_scan_show_beams" << YAML::Value << visual_scan_show_beams;
  out << YAML::Key << "visual_scan_show_heat_points" << YAML::Value << visual_scan_show_heat_points;
  out << YAML::Key << "visual_scan_depth_test_beams" << YAML::Value << visual_scan_depth_test_beams;
  out << YAML::Key << "visual_scan_depth_test_heat_points" << YAML::Value << visual_scan_depth_test_heat_points;
  out << YAML::Key << "visual_scan_interpolate_motion" << YAML::Value << visual_scan_interpolate_motion;
  out << YAML::Key << "visual_scan_keep_points_after_finish" << YAML::Value << visual_scan_keep_points_after_finish;

  SaveFloatMeanDeviation(out, "visual_scan_playback_speed", visual_scan_playback_speed,
                         visual_scan_playback_speed_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_view_dwell_seconds", visual_scan_view_dwell_seconds,
                         visual_scan_view_dwell_seconds_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_transition_seconds", visual_scan_transition_seconds,
                         visual_scan_transition_seconds_deviation);
  SaveIntMeanDeviation(out, "visual_scan_beam_stride", visual_scan_beam_stride,
                       visual_scan_beam_stride_deviation);
  SaveIntMeanDeviation(out, "visual_scan_max_beams_per_view", visual_scan_max_beams_per_view,
                       visual_scan_max_beams_per_view_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_width", visual_scan_beam_width,
                         visual_scan_beam_width_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_alpha", visual_scan_beam_alpha,
                         visual_scan_beam_alpha_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_speed", visual_scan_beam_speed,
                         visual_scan_beam_speed_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_fire_fraction", visual_scan_beam_fire_fraction,
                         visual_scan_beam_fire_fraction_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_max_length", visual_scan_beam_max_length,
                         visual_scan_beam_max_length_deviation);
  SaveIntMeanDeviation(out, "visual_scan_max_heat_points", visual_scan_max_heat_points,
                       visual_scan_max_heat_points_deviation);
  SaveIntMeanDeviation(out, "visual_scan_max_hits_per_view", visual_scan_max_hits_per_view,
                       visual_scan_max_hits_per_view_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_point_size", visual_scan_point_size,
                         visual_scan_point_size_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_scanner_size", visual_scan_scanner_size,
                         visual_scan_scanner_size_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_hot_seconds", visual_scan_hot_seconds,
                         visual_scan_hot_seconds_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_warm_seconds", visual_scan_warm_seconds,
                         visual_scan_warm_seconds_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_cool_seconds", visual_scan_cool_seconds,
                         visual_scan_cool_seconds_deviation);
}

void TasselPointCloudScannerDescriptor::Deserialize(const YAML::Node& in) {
  point_settings.Load("point_settings", in);
  capture_settings.Load("capture_settings", in);
  if (in["scan_seed"]) {
    scan_seed = in["scan_seed"].as<int>();
  }

  if (in["visual_scan_interactive_enabled"]) {
    visual_scan_interactive_enabled = in["visual_scan_interactive_enabled"].as<bool>();
  }
  if (in["visual_scan_show_scanner"]) {
    visual_scan_show_scanner = in["visual_scan_show_scanner"].as<bool>();
  }
  if (in["visual_scan_show_beams"]) {
    visual_scan_show_beams = in["visual_scan_show_beams"].as<bool>();
  }
  if (in["visual_scan_show_heat_points"]) {
    visual_scan_show_heat_points = in["visual_scan_show_heat_points"].as<bool>();
  }
  if (in["visual_scan_depth_test_beams"]) {
    visual_scan_depth_test_beams = in["visual_scan_depth_test_beams"].as<bool>();
  }
  if (in["visual_scan_depth_test_heat_points"]) {
    visual_scan_depth_test_heat_points = in["visual_scan_depth_test_heat_points"].as<bool>();
  }
  if (in["visual_scan_interpolate_motion"]) {
    visual_scan_interpolate_motion = in["visual_scan_interpolate_motion"].as<bool>();
  }
  if (in["visual_scan_keep_points_after_finish"]) {
    visual_scan_keep_points_after_finish = in["visual_scan_keep_points_after_finish"].as<bool>();
  }
  LoadFloatMeanDeviation(in, "visual_scan_playback_speed",
                         visual_scan_playback_speed, visual_scan_playback_speed_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_view_dwell_seconds",
                         visual_scan_view_dwell_seconds, visual_scan_view_dwell_seconds_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_transition_seconds",
                         visual_scan_transition_seconds, visual_scan_transition_seconds_deviation);
  LoadIntMeanDeviation(in, "visual_scan_beam_stride",
                       visual_scan_beam_stride, visual_scan_beam_stride_deviation);
  LoadIntMeanDeviation(in, "visual_scan_max_beams_per_view",
                       visual_scan_max_beams_per_view, visual_scan_max_beams_per_view_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_beam_width",
                         visual_scan_beam_width, visual_scan_beam_width_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_beam_alpha",
                         visual_scan_beam_alpha, visual_scan_beam_alpha_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_beam_speed",
                         visual_scan_beam_speed, visual_scan_beam_speed_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_beam_fire_fraction",
                         visual_scan_beam_fire_fraction, visual_scan_beam_fire_fraction_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_beam_max_length",
                         visual_scan_beam_max_length, visual_scan_beam_max_length_deviation);
  LoadIntMeanDeviation(in, "visual_scan_max_heat_points",
                       visual_scan_max_heat_points, visual_scan_max_heat_points_deviation);
  LoadIntMeanDeviation(in, "visual_scan_max_hits_per_view",
                       visual_scan_max_hits_per_view, visual_scan_max_hits_per_view_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_point_size",
                         visual_scan_point_size, visual_scan_point_size_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_scanner_size",
                         visual_scan_scanner_size, visual_scan_scanner_size_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_hot_seconds",
                         visual_scan_hot_seconds, visual_scan_hot_seconds_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_warm_seconds",
                         visual_scan_warm_seconds, visual_scan_warm_seconds_deviation);
  LoadFloatMeanDeviation(in, "visual_scan_cool_seconds",
                         visual_scan_cool_seconds, visual_scan_cool_seconds_deviation);
  visual_scan_playback_speed = std::max(0.01f, visual_scan_playback_speed);
  visual_scan_view_dwell_seconds = std::max(0.01f, visual_scan_view_dwell_seconds);
  visual_scan_transition_seconds = std::max(0.0f, visual_scan_transition_seconds);
  visual_scan_beam_stride = std::max(1, visual_scan_beam_stride);
  visual_scan_max_beams_per_view = std::max(1, visual_scan_max_beams_per_view);
  visual_scan_beam_width = std::max(0.0001f, visual_scan_beam_width);
  visual_scan_beam_alpha = glm::clamp(visual_scan_beam_alpha, 0.0f, 1.0f);
  visual_scan_beam_speed = std::max(0.001f, visual_scan_beam_speed);
  visual_scan_beam_fire_fraction = glm::clamp(visual_scan_beam_fire_fraction, 0.0f, 1.0f);
  visual_scan_beam_max_length = std::max(0.01f, visual_scan_beam_max_length);
  visual_scan_max_heat_points = std::max(1, visual_scan_max_heat_points);
  visual_scan_max_hits_per_view = std::max(1, visual_scan_max_hits_per_view);
  visual_scan_point_size = std::max(0.0001f, visual_scan_point_size);
  visual_scan_scanner_size = std::max(0.0001f, visual_scan_scanner_size);
  visual_scan_hot_seconds = std::max(0.01f, visual_scan_hot_seconds);
  visual_scan_warm_seconds = std::max(0.01f, visual_scan_warm_seconds);
  visual_scan_cool_seconds = std::max(0.01f, visual_scan_cool_seconds);
  visual_scan_keep_points_after_finish = true;
  gpu_only_mode = false;
}

// ---------------------------------------------------------------------------
// TasselPointCloudScanner
// ---------------------------------------------------------------------------

void TasselPointCloudScanner::SetScannerDescriptor(
    const std::shared_ptr<TasselPointCloudScannerDescriptor>& descriptor) {
  scanner_descriptor_ref_ = descriptor;
  ClearVisualScanRuntimeState(true, true);
  visual_scan_playback_state_ = VisualPlaybackState::Idle;
  visual_scan_last_tick_seconds_ = -1.0;
  one_shot_last_trigger_seconds_ = -1.0;
}

bool TasselPointCloudScanner::TryGetActiveSettings(TasselPointCloudPointSettings& point_settings,
                                                   TasselPointCloudGridCaptureSettings& capture_settings) const {
  AssetRef descriptor_ref_copy = scanner_descriptor_ref_;
  const auto active_descriptor = descriptor_ref_copy.Get<TasselPointCloudScannerDescriptor>();
  if (!active_descriptor) {
    EVOENGINE_ERROR("Tassel point cloud scanner has no active descriptor for baseline extraction.");
    return false;
  }

  point_settings = active_descriptor->point_settings;
  capture_settings = active_descriptor->capture_settings;
  return true;
}

bool TasselPointCloudScanner::ExecuteDeterministicScan(
    const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor,
    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
    ScanExecutionResult& out) const {
  out = ScanExecutionResult{};

  const auto scene = GetScene();
  if (!scene) {
    EVOENGINE_ERROR("No active scene for tassel point cloud scanning.");
    return false;
  }

  if (!active_descriptor) {
    EVOENGINE_ERROR("Tassel point cloud scanner requires a TasselPointCloudScannerDescriptor asset.");
    return false;
  }

  std::mt19937 settings_rng(MixSeed(active_descriptor->scan_seed, kScanSettingsSeedSalt));
  const auto sampled_point_settings = SamplePointSettingsForScan(active_descriptor->point_settings, settings_rng);

  std::shared_ptr<PointCloudCaptureSettings> active_capture_settings = capture_settings;
  if (!active_capture_settings) {
    active_capture_settings =
        std::make_shared<TasselPointCloudGridCaptureSettings>(active_descriptor->capture_settings);
  }
  if (const auto capture_settings_with_distribution =
          std::dynamic_pointer_cast<TasselPointCloudGridCaptureSettings>(active_capture_settings)) {
    active_capture_settings = std::make_shared<TasselPointCloudGridCaptureSettings>(
        SampleCaptureSettingsForScan(*capture_settings_with_distribution, settings_rng));
  }

  Bound plant_bound{};
  bool has_plant_bound = false;

  std::unordered_map<Handle, Handle> stem_particle_handles;
  std::unordered_map<Handle, Handle> spikelet_particle_handles;

  const std::vector<Entity>* tassel_entities = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
  if (tassel_entities == nullptr) {
    EVOENGINE_ERROR("No MaizeTassel entities found.");
    return false;
  }

  for (const auto& tassel_entity : *tassel_entities) {
    if (!scene->IsEntityValid(tassel_entity)) {
      continue;
    }

    scene->ForEachChild(tassel_entity, [&](const Entity child) {
      const auto name = scene->GetEntityName(child);
      if (name == "Tassel Internodes" && scene->HasPrivateComponent<Particles>(child)) {
        const auto particles = scene->GetOrSetPrivateComponent<Particles>(child).lock();
        if (!particles) {
          return;
        }
        stem_particle_handles.insert({particles->GetHandle(), tassel_entity.GetIndex()});

        const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
        const auto mesh = particles->mesh.Get<Mesh>();
        if (mesh) {
          const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
          if (particle_info_list && !particle_info_list->PeekParticleInfoList().empty()) {
            const auto& infos = particle_info_list->PeekParticleInfoList();
            for (const auto& info : infos) {
              ExpandBoundWithTransformedMeshBound(
                  plant_bound, global_transform.value * info.instance_matrix.value,
                  mesh->GetBound(), has_plant_bound);
            }
          } else {
            ExpandBoundWithTransformedMeshBound(
                plant_bound, global_transform.value, mesh->GetBound(), has_plant_bound);
          }
        }
      } else if (name == "Tassel Stem Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
        if (!mesh_renderer) {
          return;
        }
        stem_particle_handles.insert({mesh_renderer->GetHandle(), tassel_entity.GetIndex()});

        const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
        const auto mesh = mesh_renderer->mesh.Get<Mesh>();
        if (mesh) {
          ExpandBoundWithTransformedMeshBound(
              plant_bound, global_transform.value, mesh->GetBound(), has_plant_bound);
        }
      } else if (name == "Tassel Spikelets" && scene->HasPrivateComponent<Particles>(child)) {
        const auto particles = scene->GetOrSetPrivateComponent<Particles>(child).lock();
        if (!particles) {
          return;
        }
        spikelet_particle_handles.insert({particles->GetHandle(), tassel_entity.GetIndex()});

        const auto global_transform = scene->GetDataComponent<GlobalTransform>(child);
        const auto mesh = particles->mesh.Get<Mesh>();
        if (mesh) {
          const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
          if (particle_info_list && !particle_info_list->PeekParticleInfoList().empty()) {
            const auto& infos = particle_info_list->PeekParticleInfoList();
            for (const auto& info : infos) {
              ExpandBoundWithTransformedMeshBound(
                  plant_bound, global_transform.value * info.instance_matrix.value,
                  mesh->GetBound(), has_plant_bound);
            }
          } else {
            ExpandBoundWithTransformedMeshBound(
                plant_bound, global_transform.value, mesh->GetBound(), has_plant_bound);
          }
        }
      }
    });
  }

  if (has_plant_bound) {
    AutoScaleTasselCaptureSettingsToPlantBound(active_capture_settings, plant_bound);
  }

  out.samples.clear();
  active_capture_settings->GenerateSamples(out.samples);
  if (out.samples.empty()) {
    EVOENGINE_ERROR("Tassel point cloud capture generated zero ray samples before tracing.");
    return false;
  }

  const auto& settings = sampled_point_settings;
  if (settings.angular_noise_sigma > 0.0f) {
    std::mt19937 angular_rng(MixSeed(active_descriptor->scan_seed, kAngularNoiseSeedSalt));
    std::normal_distribution<float> angular_dist(0.0f, settings.angular_noise_sigma);
    for (auto& sample : out.samples) {
      glm::vec3 tangent;
      if (glm::abs(sample.direction.y) < 0.999f) {
        tangent = glm::normalize(glm::cross(sample.direction, glm::vec3(0, 1, 0)));
      } else {
        tangent = glm::normalize(glm::cross(sample.direction, glm::vec3(1, 0, 0)));
      }
      const glm::vec3 bitangent = glm::cross(sample.direction, tangent);
      sample.direction = glm::normalize(
          sample.direction + angular_dist(angular_rng) * tangent + angular_dist(angular_rng) * bitangent);
    }
  }

  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (active_capture_settings->capture_mode == PointCloudCaptureSettings::CaptureMode::Gpu) {
    if (!Platform::RayTracingEnabled()) {
      EVOENGINE_ERROR("Tassel point cloud GPU capture preflight failed: ray tracing is disabled.");
      return false;
    }
    if (!render_layer) {
      EVOENGINE_ERROR("Tassel point cloud GPU capture preflight failed: RenderLayer is unavailable.");
      return false;
    }
  }

  switch (active_capture_settings->capture_mode) {
    case PointCloudCaptureSettings::CaptureMode::Cpu: {
      std::shared_ptr<RenderInstanceStorage> render_instances{};
      if (render_layer) {
        render_instances = render_layer->GetCurrentRenderInstanceStorage();
      }
      if (!render_instances) {
        render_instances = std::make_shared<RenderInstanceStorage>();
        Bound world_bound;
        render_instances->BuildFromScene({}, scene, world_bound);
      }
      CpuRayTracer cpu_ray_tracer;
      cpu_ray_tracer.Initialize(
          render_instances,
          [&](uint32_t, const std::shared_ptr<Mesh>&) {

          },
          [&](const uint32_t, const Entity&) {

          });
      cpu_ray_tracer.SamplePointCloud(out.samples);
    } break;
    case PointCloudCaptureSettings::CaptureMode::Gpu:
      PointCloud::SampleCurrentScene(out.samples);
      break;
  }

  const bool has_ray_hits = std::any_of(out.samples.begin(), out.samples.end(),
                                        [](const PointCloudSample& sample) {
                                          return sample.hit_count > 0;
                                        });
  if (!has_ray_hits) {
    EVOENGINE_ERROR("Tassel point cloud capture returned zero hits after tracing.");
    return false;
  }

  out.sample_kept.assign(out.samples.size(), static_cast<uint8_t>(0));
  out.sample_points.assign(out.samples.size(), glm::vec3(0.0f));
  out.points.clear();
  out.instance_indices.clear();
  out.type_indices.clear();
  out.colors.clear();

  std::mt19937 noise_rng(MixSeed(active_descriptor->scan_seed, kPointNoiseSeedSalt));
  std::uniform_real_distribution<float> dropout_dist(0.0f, 1.0f);

  size_t ray_hit_samples = 0;
  size_t sample_filter_rejected = 0;
  size_t range_filter_rejected = 0;
  size_t dropout_rejected = 0;
  size_t bound_rejected = 0;

  for (size_t sample_index = 0; sample_index < out.samples.size(); sample_index++) {
    const auto& sample = out.samples[sample_index];
    if (sample.hit_count == 0) {
      continue;
    }
    ray_hit_samples++;

    if (!active_capture_settings->SampleFilter(sample)) {
      sample_filter_rejected++;
      continue;
    }

    const float distance = glm::distance(sample.hit_info.position, sample.start);
    if (settings.min_range > 0.0f && distance < settings.min_range) {
      range_filter_rejected++;
      continue;
    }
    if (settings.max_range > 0.0f && distance > settings.max_range) {
      range_filter_rejected++;
      continue;
    }

    if (settings.dropout_probability > 0.0f && dropout_dist(noise_rng) < settings.dropout_probability) {
      dropout_rejected++;
      continue;
    }

    if (has_plant_bound &&
        !PointInsideExpandedBound(sample.hit_info.position, plant_bound, settings.bounding_box_limit)) {
      bound_rejected++;
      continue;
    }

    const glm::vec3 ball_rand = SampleBall(noise_rng, settings.hit_ball_jitter_radius);

    glm::vec3 range_noise(0.0f);
    if (settings.range_noise_base_sigma > 0.0f || settings.range_noise_scale > 0.0f) {
      const float sigma = settings.range_noise_base_sigma + settings.range_noise_scale * distance;
      range_noise = glm::vec3(
          SampleGaussian(noise_rng, sigma),
          SampleGaussian(noise_rng, sigma),
          SampleGaussian(noise_rng, sigma));
    }

    glm::vec3 distance_scaled_noise(0.0f);
    if (settings.distance_sigma_scale > 0.0f) {
      distance_scaled_noise = distance * glm::vec3(
          SampleGaussian(noise_rng, settings.distance_sigma_scale),
          SampleGaussian(noise_rng, settings.distance_sigma_scale),
          SampleGaussian(noise_rng, settings.distance_sigma_scale));
    }

    const glm::vec3 final_point = sample.hit_info.position + distance_scaled_noise + range_noise + ball_rand;
    out.sample_kept[sample_index] = static_cast<uint8_t>(1);
    out.sample_points[sample_index] = final_point;
    out.points.emplace_back(final_point);

    if (settings.color_output) {
      out.colors.emplace_back(glm::vec3(sample.hit_info.color));
    }

    const auto stem_search = stem_particle_handles.find(sample.handle);
    const auto spikelet_search = spikelet_particle_handles.find(sample.handle);

    if (settings.instance_index) {
      if (stem_search != stem_particle_handles.end()) {
        out.instance_indices.emplace_back(static_cast<int>(stem_search->second));
      } else if (spikelet_search != spikelet_particle_handles.end()) {
        out.instance_indices.emplace_back(static_cast<int>(spikelet_search->second));
      } else {
        out.instance_indices.emplace_back(0);
      }
    }

    if (settings.type_index) {
      if (stem_search != stem_particle_handles.end()) {
        out.type_indices.emplace_back(0);
      } else if (spikelet_search != spikelet_particle_handles.end()) {
        out.type_indices.emplace_back(1);
      } else {
        out.type_indices.emplace_back(-1);
      }
    }
  }

  if (out.points.empty()) {
    const auto post_filter_rejected =
        sample_filter_rejected + range_filter_rejected + dropout_rejected + bound_rejected;
    std::string active_scan_mode = "Unknown";
    float active_sample_filter_bbox = 0.0f;
    if (const auto active_tassel_capture_settings =
            std::dynamic_pointer_cast<TasselPointCloudGridCaptureSettings>(active_capture_settings)) {
      active_scan_mode = ScanModeName(active_tassel_capture_settings->scan_mode);
      active_sample_filter_bbox = active_tassel_capture_settings->bounding_box_size;
    }
    EVOENGINE_ERROR(
        "Tassel capture produced zero kept points after filtering. "
        "samples=" + std::to_string(out.samples.size()) +
        ", hit_samples=" + std::to_string(ray_hit_samples) +
        ", sample_filter_rejected=" + std::to_string(sample_filter_rejected) +
        ", range_filter_rejected=" + std::to_string(range_filter_rejected) +
        ", dropout_rejected=" + std::to_string(dropout_rejected) +
        ", bound_rejected=" + std::to_string(bound_rejected) +
        ", filtered_out=" + std::to_string(post_filter_rejected) +
        ", scan_mode=" + active_scan_mode +
        ", sample_filter_bbox=" + std::to_string(active_sample_filter_bbox));
  }

  return true;
}

void TasselPointCloudScanner::RunScanVisualization(
    const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor) {
  if (!active_descriptor) {
    return;
  }

  const auto sampled_visual_settings = SampleVisualScanSettings(*active_descriptor);

  if (active_descriptor->visual_scan_interactive_enabled) {
    StartVisualScanPlayback(active_descriptor);
    return;
  }

  const double build_start_seconds = GetSteadyTimeSeconds();
  ScanExecutionResult execution{};
  if (!ExecuteDeterministicScan(active_descriptor, nullptr, execution)) {
    visual_scan_last_build_seconds_ = GetSteadyTimeSeconds() - build_start_seconds;
    return;
  }

  StopVisualScanPlayback(false);
  ClearVisualScanRuntimeState(true, true);

  const float hot_seconds = sampled_visual_settings.hot_seconds;
  const float warm_seconds = sampled_visual_settings.warm_seconds;
  const float cool_seconds = sampled_visual_settings.cool_seconds;
  const float cooled_age = hot_seconds + warm_seconds + cool_seconds + 0.01f;

  for (const auto& point : execution.points) {
    visual_scan_heat_points_.push_back(VisualHeatPoint{point, cooled_age});
  }
  visual_scan_total_injected_hits_ = execution.points.size();

  const size_t max_heat_points = static_cast<size_t>(sampled_visual_settings.max_heat_points);
  while (visual_scan_heat_points_.size() > max_heat_points) {
    visual_scan_heat_points_.pop_front();
  }

  if (!execution.samples.empty()) {
    visual_scan_scanner_position_ = execution.samples.back().start;
  }

  visual_scan_playback_state_ = VisualPlaybackState::Finished;
  visual_scan_last_build_seconds_ = GetSteadyTimeSeconds() - build_start_seconds;
}

void TasselPointCloudScanner::ClearVisualScanRuntimeState(const bool clear_batches,
                                                          const bool clear_heat_points) {
  if (clear_batches) {
    visual_scan_batches_.clear();
    visual_scan_active_beam_indices_.clear();
    visual_scan_active_beam_fire_times_.clear();
    visual_scan_active_view_index_ = 0;
    visual_scan_view_elapsed_seconds_ = 0.0;
    visual_scan_current_view_injected_ = false;
    visual_scan_last_rendered_beams_ = 0;
  }

  if (clear_heat_points) {
    visual_scan_heat_points_.clear();
    visual_scan_last_rendered_heat_points_ = 0;
    visual_scan_total_injected_hits_ = 0;
  }

  if (const auto beam_particle_info = visual_scan_beam_particle_info_ref_.Get<ParticleInfoList>()) {
    beam_particle_info->SetParticleInfos(std::vector<ParticleInfo>{});
  }
  visual_scan_last_rendered_beams_ = 0;
  if (clear_heat_points) {
    if (const auto heat_particle_info = visual_scan_heat_particle_info_ref_.Get<ParticleInfoList>()) {
      heat_particle_info->SetParticleInfos(std::vector<ParticleInfo>{});
    }
  }

  if (clear_batches || clear_heat_points) {
    visual_scan_scanner_position_ = glm::vec3(0.0f);
  }
}

void TasselPointCloudScanner::StopVisualScanPlayback(const bool clear_heat_points) {
  visual_scan_playback_state_ = clear_heat_points ? VisualPlaybackState::Idle : VisualPlaybackState::Finished;
  visual_scan_view_elapsed_seconds_ = 0.0;
  visual_scan_current_view_injected_ = false;
  visual_scan_last_tick_seconds_ = -1.0;
  ClearVisualScanRuntimeState(false, clear_heat_points);
}

bool TasselPointCloudScanner::BuildVisualScanBatches(
    const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor) {
  ScanExecutionResult execution{};
  if (!ExecuteDeterministicScan(active_descriptor, nullptr, execution)) {
    EVOENGINE_ERROR("Visual scan playback failed during deterministic scan execution.")
    return false;
  }
  if (execution.samples.empty()) {
    EVOENGINE_ERROR("Visual scan playback failed: generated zero ray samples.")
    return false;
  }

  visual_scan_batches_.clear();
  visual_scan_batches_.reserve(512);

  const auto sampled_visual_settings = SampleVisualScanSettings(*active_descriptor);

  const size_t beam_stride = static_cast<size_t>(sampled_visual_settings.beam_stride);
  const size_t max_beams_per_view = static_cast<size_t>(sampled_visual_settings.max_beams_per_view);
  const float beam_max_length = sampled_visual_settings.beam_max_length;
  const float position_group_epsilon = 1e-6f;

  size_t start_index = 0;
  while (start_index < execution.samples.size()) {
    size_t end_index = start_index + 1;
    const glm::vec3 scanner_origin = execution.samples[start_index].start;
    while (end_index < execution.samples.size()) {
      if (glm::length(execution.samples[end_index].start - scanner_origin) > position_group_epsilon) {
        break;
      }
      end_index++;
    }

    VisualScanViewBatch view_batch;
    view_batch.scanner_origin = scanner_origin;

    size_t rendered_beams = 0;
    for (size_t sample_index = start_index;
         sample_index < end_index && rendered_beams < max_beams_per_view;
         sample_index += beam_stride) {
      const auto& sample = execution.samples[sample_index];
      glm::vec3 end_point = sample.start + sample.direction * beam_max_length;
      if (sample.hit_count > 0) {
        end_point = sample.hit_info.position;
      }
      const glm::vec3 delta = end_point - sample.start;
      const float length = glm::length(delta);
      if (length > beam_max_length && length > 1e-6f) {
        end_point = sample.start + (delta / length) * beam_max_length;
      }
      view_batch.beam_starts.emplace_back(sample.start);
      view_batch.beam_ends.emplace_back(end_point);
      rendered_beams++;
    }

    for (size_t sample_index = start_index; sample_index < end_index; sample_index++) {
      if (execution.sample_kept[sample_index] == static_cast<uint8_t>(0)) {
        continue;
      }
      view_batch.accepted_hits.emplace_back(execution.sample_points[sample_index]);
    }

    visual_scan_batches_.emplace_back(std::move(view_batch));
    start_index = end_index;
  }

  if (visual_scan_batches_.empty()) {
    EVOENGINE_ERROR("Visual scan playback failed: no viewpoint batches were built.")
    return false;
  }

  if (!visual_scan_beam_particle_info_ref_.Get<ParticleInfoList>()) {
    visual_scan_beam_particle_info_ref_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }
  if (!visual_scan_heat_particle_info_ref_.Get<ParticleInfoList>()) {
    visual_scan_heat_particle_info_ref_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }

  return true;
}

void TasselPointCloudScanner::StartVisualScanPlayback(
    const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor) {
  const double build_start_seconds = GetSteadyTimeSeconds();
  ClearVisualScanRuntimeState(true, true);

  if (!BuildVisualScanBatches(active_descriptor)) {
    visual_scan_playback_state_ = VisualPlaybackState::Idle;
    visual_scan_last_build_seconds_ = GetSteadyTimeSeconds() - build_start_seconds;
    return;
  }

  visual_scan_playback_state_ = VisualPlaybackState::Playing;
  visual_scan_active_view_index_ = 0;
  visual_scan_view_elapsed_seconds_ = 0.0;
  visual_scan_current_view_injected_ = false;
  visual_scan_last_tick_seconds_ = -1.0;
  visual_scan_scanner_position_ = visual_scan_batches_.front().scanner_origin;
  visual_scan_last_build_seconds_ = GetSteadyTimeSeconds() - build_start_seconds;
}

void TasselPointCloudScanner::TickVisualScanPlayback(
    const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor,
    const double delta_seconds) {
  if (visual_scan_playback_state_ != VisualPlaybackState::Playing || visual_scan_batches_.empty()) {
    return;
  }

  const auto sampled_visual_settings = SampleVisualScanSettings(*active_descriptor);
  const double scaled_delta = std::max(0.0, delta_seconds) *
                              static_cast<double>(sampled_visual_settings.playback_speed);
  const float dwell_seconds = sampled_visual_settings.view_dwell_seconds;
  const float transition_seconds = sampled_visual_settings.transition_seconds;
  const float cycle_duration = std::max(0.01f, dwell_seconds + transition_seconds);

  if (!visual_scan_current_view_injected_) {
    const auto& current_view = visual_scan_batches_[visual_scan_active_view_index_];
    for (const auto& hit : current_view.accepted_hits) {
      visual_scan_heat_points_.push_back(VisualHeatPoint{hit, 0.0f});
    }
    visual_scan_total_injected_hits_ += current_view.accepted_hits.size();

    visual_scan_active_beam_indices_.clear();
    visual_scan_active_beam_fire_times_.clear();

    const size_t total_beams = current_view.beam_starts.size();
    const float beam_fire_fraction = sampled_visual_settings.beam_fire_fraction;
    size_t target_fire_count = static_cast<size_t>(beam_fire_fraction * static_cast<float>(total_beams));
    if (beam_fire_fraction > 0.0f && target_fire_count == 0 && total_beams > 0) {
      target_fire_count = 1;
    }
    target_fire_count = std::min(total_beams, target_fire_count);

    if (target_fire_count > 0) {
      std::vector<size_t> shuffled_indices(total_beams);
      for (size_t i = 0; i < total_beams; i++) {
        shuffled_indices[i] = i;
      }

      std::mt19937 beam_rng(MixSeed(active_descriptor->scan_seed + static_cast<int>(visual_scan_active_view_index_),
                                    kBeamShuffleSeedSalt));
      std::shuffle(shuffled_indices.begin(), shuffled_indices.end(), beam_rng);
      shuffled_indices.resize(target_fire_count);

      visual_scan_active_beam_indices_ = std::move(shuffled_indices);
      visual_scan_active_beam_fire_times_.resize(target_fire_count, 0.0f);
      if (target_fire_count > 1 && dwell_seconds > 0.0f) {
        const float launch_spacing = dwell_seconds / static_cast<float>(target_fire_count);
        for (size_t i = 0; i < target_fire_count; i++) {
          visual_scan_active_beam_fire_times_[i] = launch_spacing * static_cast<float>(i);
        }
      }
    }

    visual_scan_current_view_injected_ = true;
  }

  const size_t max_heat_points = static_cast<size_t>(sampled_visual_settings.max_heat_points);
  while (visual_scan_heat_points_.size() > max_heat_points) {
    visual_scan_heat_points_.pop_front();
  }

  const float hot_seconds = sampled_visual_settings.hot_seconds;
  const float warm_seconds = sampled_visual_settings.warm_seconds;
  const float cool_seconds = sampled_visual_settings.cool_seconds;

  for (auto& heat_point : visual_scan_heat_points_) {
    heat_point.age_seconds += static_cast<float>(scaled_delta);
  }

  visual_scan_view_elapsed_seconds_ += scaled_delta;
  while (visual_scan_view_elapsed_seconds_ >= cycle_duration) {
    visual_scan_view_elapsed_seconds_ -= cycle_duration;
    if (visual_scan_active_view_index_ + 1 < visual_scan_batches_.size()) {
      visual_scan_active_view_index_++;
      visual_scan_current_view_injected_ = false;
      visual_scan_active_beam_indices_.clear();
      visual_scan_active_beam_fire_times_.clear();
    } else {
      visual_scan_playback_state_ = VisualPlaybackState::Finished;
      visual_scan_active_beam_indices_.clear();
      visual_scan_active_beam_fire_times_.clear();
      break;
    }
  }

  if (visual_scan_playback_state_ == VisualPlaybackState::Playing) {
    const auto& current_view = visual_scan_batches_[visual_scan_active_view_index_];
    visual_scan_scanner_position_ = current_view.scanner_origin;
    if (active_descriptor->visual_scan_interpolate_motion && transition_seconds > 0.0f &&
        visual_scan_active_view_index_ + 1 < visual_scan_batches_.size() &&
        visual_scan_view_elapsed_seconds_ > static_cast<double>(dwell_seconds)) {
      const float t = glm::clamp(
          static_cast<float>((visual_scan_view_elapsed_seconds_ - dwell_seconds) / transition_seconds), 0.0f, 1.0f);
      const auto& next_view = visual_scan_batches_[visual_scan_active_view_index_ + 1];
      visual_scan_scanner_position_ = glm::mix(current_view.scanner_origin, next_view.scanner_origin, t);
    }
  } else {
    visual_scan_scanner_position_ = visual_scan_batches_.back().scanner_origin;
  }

  if (const auto beam_particle_info = visual_scan_beam_particle_info_ref_.Get<ParticleInfoList>()) {
    if (active_descriptor->visual_scan_show_beams &&
        visual_scan_playback_state_ == VisualPlaybackState::Playing &&
        visual_scan_active_view_index_ < visual_scan_batches_.size() &&
        !visual_scan_active_beam_indices_.empty()) {
      const auto& current_view = visual_scan_batches_[visual_scan_active_view_index_];

      std::vector<glm::vec3> animated_beam_starts;
      std::vector<glm::vec3> animated_beam_ends;
      animated_beam_starts.reserve(visual_scan_active_beam_indices_.size());
      animated_beam_ends.reserve(visual_scan_active_beam_indices_.size());

      const float beam_speed = sampled_visual_settings.beam_speed;
      const float beam_alpha = sampled_visual_settings.beam_alpha;
      const float view_time_seconds = static_cast<float>(visual_scan_view_elapsed_seconds_);

      if (beam_alpha > 0.0f && view_time_seconds <= dwell_seconds) {
        for (size_t i = 0; i < visual_scan_active_beam_indices_.size(); i++) {
          const size_t beam_index = visual_scan_active_beam_indices_[i];
          if (beam_index >= current_view.beam_starts.size() ||
              beam_index >= current_view.beam_ends.size()) {
            continue;
          }

          const float launch_time = i < visual_scan_active_beam_fire_times_.size()
                                        ? visual_scan_active_beam_fire_times_[i]
                                        : 0.0f;
          if (view_time_seconds < launch_time) {
            continue;
          }

          const glm::vec3 start = current_view.beam_starts[beam_index];
          const glm::vec3 final_end = current_view.beam_ends[beam_index];
          const glm::vec3 delta = final_end - start;
          const float beam_length = glm::length(delta);
          if (beam_length <= 1e-6f) {
            continue;
          }

          const float beam_elapsed_seconds = view_time_seconds - launch_time;
          const float travel_time = beam_length / beam_speed;
          if (beam_elapsed_seconds >= travel_time) {
            continue;
          }

          const float travel_distance = std::min(beam_length, beam_elapsed_seconds * beam_speed);
          const glm::vec3 animated_end = start + (delta / beam_length) * travel_distance;

          animated_beam_starts.emplace_back(start);
          animated_beam_ends.emplace_back(animated_end);
        }
      }

      if (!animated_beam_starts.empty()) {
        beam_particle_info->ApplyConnections(animated_beam_starts, animated_beam_ends,
                                             glm::vec4(1.0f, 0.22f, 0.10f, beam_alpha),
                                             sampled_visual_settings.beam_width);
        visual_scan_last_rendered_beams_ = animated_beam_starts.size();
      } else {
        beam_particle_info->SetParticleInfos(std::vector<ParticleInfo>{});
        visual_scan_last_rendered_beams_ = 0;
      }
    } else {
      beam_particle_info->SetParticleInfos(std::vector<ParticleInfo>{});
      visual_scan_last_rendered_beams_ = 0;
    }
  }

  if (const auto heat_particle_info = visual_scan_heat_particle_info_ref_.Get<ParticleInfoList>()) {
    std::vector<ParticleInfo> heat_infos;
    heat_infos.reserve(visual_scan_heat_points_.size());
    const float point_size = sampled_visual_settings.point_size;
    for (const auto& heat_point : visual_scan_heat_points_) {
      ParticleInfo info;
      info.instance_matrix.value = glm::translate(heat_point.position) * glm::scale(glm::vec3(point_size));
      info.instance_color = HeatColorFromAge(heat_point.age_seconds, hot_seconds, warm_seconds, cool_seconds);
      if (info.instance_color.a <= 0.0f) {
        continue;
      }
      heat_infos.emplace_back(info);
    }
    heat_particle_info->SetParticleInfos(heat_infos);
    visual_scan_last_rendered_heat_points_ = heat_infos.size();
  }
}

void TasselPointCloudScanner::RenderVisualScanPlayback(
    const std::shared_ptr<EditorLayer>& editor_layer,
    const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor) {
  if (!editor_layer) {
    return;
  }

  const auto sampled_visual_settings = SampleVisualScanSettings(*active_descriptor);

  GizmoSettings beam_gizmo_settings{};
  beam_gizmo_settings.depth_test = active_descriptor->visual_scan_depth_test_beams;
  beam_gizmo_settings.depth_write = active_descriptor->visual_scan_depth_test_beams;

  GizmoSettings heat_gizmo_settings{};
  heat_gizmo_settings.depth_test = active_descriptor->visual_scan_depth_test_heat_points;
  heat_gizmo_settings.depth_write = active_descriptor->visual_scan_depth_test_heat_points;

  if (active_descriptor->visual_scan_show_beams) {
    if (const auto beam_particle_info = visual_scan_beam_particle_info_ref_.Get<ParticleInfoList>()) {
      if (visual_scan_last_rendered_beams_ > 0) {
        editor_layer->DrawGizmoCylinders(beam_particle_info, glm::mat4(1.0f), 1.0f, beam_gizmo_settings);
      }
    }
  }

  if (active_descriptor->visual_scan_show_heat_points) {
    if (const auto heat_particle_info = visual_scan_heat_particle_info_ref_.Get<ParticleInfoList>()) {
      if (visual_scan_last_rendered_heat_points_ > 0) {
        editor_layer->DrawGizmoSpheres(heat_particle_info, glm::mat4(1.0f), 1.0f, heat_gizmo_settings);
      }
    }
  }

  if (active_descriptor->visual_scan_show_scanner &&
      (visual_scan_playback_state_ == VisualPlaybackState::Playing ||
       visual_scan_playback_state_ == VisualPlaybackState::Finished)) {
    editor_layer->DrawGizmoSphere(glm::vec4(0.10f, 0.88f, 1.0f, 1.0f),
                                  glm::translate(glm::mat4(1.0f), visual_scan_scanner_position_),
                                  sampled_visual_settings.scanner_size);
  }
}

void TasselPointCloudScanner::Update() {
  if (!Application::IsPlaying()) {
    return;
  }

  if (visual_scan_playback_state_ == VisualPlaybackState::Idle) {
    return;
  }

  const auto active_descriptor = scanner_descriptor_ref_.Get<TasselPointCloudScannerDescriptor>();
  if (!active_descriptor) {
    return;
  }

  const double now_seconds = GetSteadyTimeSeconds();
  double delta_seconds = 0.0;
  if (visual_scan_last_tick_seconds_ >= 0.0) {
    delta_seconds = std::max(0.0, now_seconds - visual_scan_last_tick_seconds_);
  }
  visual_scan_last_tick_seconds_ = now_seconds;

  if (visual_scan_playback_state_ == VisualPlaybackState::Playing) {
    TickVisualScanPlayback(active_descriptor, delta_seconds);
  }

  RenderVisualScanPlayback(Application::GetLayer<EditorLayer>(), active_descriptor);
}

void TasselPointCloudScanner::Scan(const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                   std::vector<glm::vec3>& points, std::vector<int>& instance_indices,
                                   std::vector<int>& type_indices,
                                   std::vector<glm::vec3>& colors) const {
  points.clear();
  instance_indices.clear();
  type_indices.clear();
  colors.clear();

  AssetRef descriptor_ref_copy = scanner_descriptor_ref_;
  const auto active_descriptor = descriptor_ref_copy.Get<TasselPointCloudScannerDescriptor>();
  ScanExecutionResult execution{};
  if (!ExecuteDeterministicScan(active_descriptor, capture_settings, execution)) {
    return;
  }

  points = std::move(execution.points);
  instance_indices = std::move(execution.instance_indices);
  type_indices = std::move(execution.type_indices);
  colors = std::move(execution.colors);
}

void TasselPointCloudScanner::SavePointCloud(const std::filesystem::path& save_path,
                                             const std::vector<glm::vec3>& points,
                                             const std::vector<int>& instance_indices,
                                             const std::vector<int>& type_indices,
                                             const std::vector<glm::vec3>& colors) const {
  std::filebuf fb_binary;
  fb_binary.open(save_path.string(), std::ios::out | std::ios::binary);
  std::ostream ostream(&fb_binary);
  if (ostream.fail())
    throw std::runtime_error("failed to open " + save_path.string());

  tinyply::PlyFile ply_file;
  ply_file.add_properties_to_element("vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, points.size(),
                                     static_cast<const uint8_t*>(static_cast<const void*>(points.data())),
                                     tinyply::Type::INVALID, 0);

  if (!colors.empty())
    ply_file.add_properties_to_element("vertex", {"red", "green", "blue"}, tinyply::Type::FLOAT32, colors.size(),
                                       static_cast<const uint8_t*>(static_cast<const void*>(colors.data())),
                                       tinyply::Type::INVALID, 0);

  if (!type_indices.empty())
    ply_file.add_properties_to_element("type_index", {"type_index"}, tinyply::Type::INT32, type_indices.size(),
                                       static_cast<const uint8_t*>(static_cast<const void*>(type_indices.data())),
                                       tinyply::Type::INVALID, 0);

  if (!instance_indices.empty())
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
  std::vector<glm::vec3> colors;

  Scan(capture_settings, points, instance_indices, type_indices, colors);
  if (points.empty()) {
    EVOENGINE_ERROR("Tassel capture aborted: scan produced no points.");
    return;
  }
  SavePointCloud(save_path, points, instance_indices, type_indices, colors);
}

bool TasselPointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  const auto reset_scan_runtime = [&]() {
    visual_scan_last_tick_seconds_ = -1.0;
    one_shot_last_trigger_seconds_ = -1.0;
  };

  const auto clear_scan_runtime = [&]() {
    StopVisualScanPlayback(true);
    ClearVisualScanRuntimeState(true, true);
    reset_scan_runtime();
  };

  if (editor_layer &&
      editor_layer->DragAndDropButton<TasselPointCloudScannerDescriptor>(
          scanner_descriptor_ref_, "Scanner Descriptor")) {
    changed = true;
    clear_scan_runtime();
  }

  auto active_descriptor = scanner_descriptor_ref_.Get<TasselPointCloudScannerDescriptor>();
  if (!active_descriptor) {
    const auto descriptor = AssetManager::CreateTemporaryAsset<TasselPointCloudScannerDescriptor>();
    scanner_descriptor_ref_ = descriptor;
    active_descriptor = descriptor;
    changed = true;
    clear_scan_runtime();
  }

  if (!active_descriptor) {
    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f),
                       "Unable to allocate scanner descriptor.");
    return changed;
  }

  ImGui::TextColored(ImVec4(0.60f, 0.90f, 0.62f, 1.0f),
                     "Descriptor-driven scanner settings active.");

  const auto capture_current_settings_to_descriptor = [&]() -> bool {
    TasselPointCloudPointSettings captured_point_settings;
    TasselPointCloudGridCaptureSettings captured_capture_settings;
    if (!TryGetActiveSettings(captured_point_settings, captured_capture_settings)) {
      EVOENGINE_WARNING("Capture failed. Scanner has no active descriptor settings.");
      return false;
    }

    active_descriptor->point_settings = captured_point_settings;
    active_descriptor->capture_settings = captured_capture_settings;
    changed = true;
    EVOENGINE_LOG("Captured current scanner settings into active descriptor: " + active_descriptor->GetTitle());
    return true;
  };

  if (ImGui::Button("Capture Current Settings to Descriptor")) {
    capture_current_settings_to_descriptor();
  }
  ShowItemHoverDescription(
      "Copy this scanner entity's active point and capture settings into the currently bound descriptor.");

  ImGui::SameLine();
  if (ImGui::Button("Overwrite Scanner Defaults")) {
    const bool captured_from_scanner = capture_current_settings_to_descriptor();
    if (captured_from_scanner) {
      EVOENGINE_LOG("Using current scanner settings for defaults overwrite.");
    }

    const auto defaults_path = ResolveWritableTasselScannerDescriptorDefaultsPath();
    if (SaveTasselScannerDescriptorDefaultsToFile(*active_descriptor, defaults_path)) {
      EVOENGINE_LOG("TasselPointCloudScannerDescriptor defaults overwritten: " + defaults_path.string());
    } else {
      EVOENGINE_WARNING("Failed to overwrite TasselPointCloudScannerDescriptor defaults.");
    }
  }
  if (ImGui::IsItemHovered()) {
    const auto defaults_path = ResolveWritableTasselScannerDescriptorDefaultsPath();
    const std::string tip =
        "Write new defaults for future TasselPointCloudScannerDescriptor assets. "
        "Current scanner settings are captured first.\nPath: " +
        defaults_path.string();
    ImGui::SetTooltip("%s", tip.c_str());
  }

  const char* scan_mode_names[] = {"One-shot scan", "Animated scan"};
  int scan_mode_index = static_cast<int>(private_scan_mode_);
  if (ImGui::Combo("Scan Mode", &scan_mode_index, scan_mode_names, IM_ARRAYSIZE(scan_mode_names))) {
    private_scan_mode_ = static_cast<PrivateScanMode>(scan_mode_index);
    reset_scan_runtime();
    StopVisualScanPlayback(false);
    changed = true;
  }
  ShowItemHoverDescription(
      "Choose whether the scanner runs a single fast scan injection or the animated playback path.");
  if (ImGui::Checkbox("Advance Seed Per Playback", &advance_seed_per_playback_)) {
    changed = true;
  }
  ShowItemHoverDescription(
      "When enabled, each playback trigger increments descriptor Scan Seed before running the scan.");

  const auto run_scan_for_current_mode = [&]() {
    if (advance_seed_per_playback_) {
      active_descriptor->scan_seed = AdvanceSeedWithWrap(active_descriptor->scan_seed);
      changed = true;
    }
    const bool previous_interactive_setting = active_descriptor->visual_scan_interactive_enabled;
    active_descriptor->visual_scan_interactive_enabled = private_scan_mode_ == PrivateScanMode::Animated;
    RunScanVisualization(active_descriptor);
    active_descriptor->visual_scan_interactive_enabled = previous_interactive_setting;
    one_shot_last_trigger_seconds_ = GetSteadyTimeSeconds();
  };

  if (private_scan_mode_ == PrivateScanMode::OneShot) {
    ImGui::TextUnformatted("One-shot uses the fast playback-backed scan renderer without the removed live preview loop.");

    if (ImGui::Button("Run Scan")) {
      run_scan_for_current_mode();
    }
    ShowItemHoverDescription("Execute one fast scan and render the resulting point set immediately.");
    ImGui::SameLine();
    if (ImGui::Button("Clear Scan")) {
      clear_scan_runtime();
    }
    ShowItemHoverDescription("Clear the current rendered scan points and beam state.");

    if (BeginMeanDeviationTable("OneShotRepeatTable")) {
      if (InspectFloatMeanDeviation("One-shot Hz (0=off)", one_shot_repeat_hz_, one_shot_repeat_hz_deviation_,
                                    0.1f, 0.0f, 60.0f, 0.0f, 60.0f,
                                    "Optional auto-repeat frequency while this inspector is open. Set mean to 0 to disable periodic scans.")) {
        one_shot_repeat_hz_ = std::max(0.0f, one_shot_repeat_hz_);
        one_shot_repeat_hz_deviation_ = std::max(0.0f, one_shot_repeat_hz_deviation_);
        reset_scan_runtime();
        changed = true;
      }
      ImGui::EndTable();
    }
  } else {
    ImGui::TextUnformatted("Animated scan reuses the same deterministic capture but plays the visual scan back over time.");

    if (ImGui::Button("Run Scan")) {
      run_scan_for_current_mode();
    }
    ShowItemHoverDescription("Run the animated visual scan playback using the fast deterministic point set.");
    ImGui::SameLine();
    if (ImGui::Button("Stop Scan")) {
      StopVisualScanPlayback(false);
      reset_scan_runtime();
    }
    ShowItemHoverDescription("Stop the animated playback and keep the currently accumulated points.");
    ImGui::SameLine();
    if (ImGui::Button("Clear Scan")) {
      clear_scan_runtime();
    }
    ShowItemHoverDescription("Stop playback and clear beams, points, and playback state.");
  }

  if (ImGui::TreeNodeEx("Visual Scan Playback", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("Show Scanner", &active_descriptor->visual_scan_show_scanner)) {
      changed = true;
    }
    ShowItemHoverDescription("Draw a marker sphere at the active scanner position.");
    if (ImGui::Checkbox("Show Beams", &active_descriptor->visual_scan_show_beams)) {
      changed = true;
    }
    ShowItemHoverDescription("Draw animated laser beams for the currently active scanner view.");
    if (ImGui::Checkbox("Show Heat Points", &active_descriptor->visual_scan_show_heat_points)) {
      changed = true;
    }
    ShowItemHoverDescription("Show accumulated hit points with hot-to-cool fading.");
    if (ImGui::Checkbox("Depth Test Beams", &active_descriptor->visual_scan_depth_test_beams)) {
      changed = true;
    }
    ShowItemHoverDescription("Enable depth test/write for beam gizmos so occluded beams are hidden by geometry.");
    if (ImGui::Checkbox("Depth Test Heat Points", &active_descriptor->visual_scan_depth_test_heat_points)) {
      changed = true;
    }
    ShowItemHoverDescription("Enable depth test/write for heat-point gizmos so occluded points are hidden by geometry.");
    if (ImGui::Checkbox("Interpolate Motion", &active_descriptor->visual_scan_interpolate_motion)) {
      changed = true;
    }
    ShowItemHoverDescription("Blend scanner motion between view origins during transitions.");
    if (!active_descriptor->visual_scan_keep_points_after_finish) {
      active_descriptor->visual_scan_keep_points_after_finish = true;
      changed = true;
    }
    ImGui::TextUnformatted("Keep Points After Finish: Always On");
    ShowItemHoverDescription("Heat points persist after playback and are removed only by Clear Scan or max-cap eviction.");
    if (BeginMeanDeviationTable("VisualScanPlaybackDistributionTable")) {
      changed |= InspectFloatMeanDeviation(
        "Playback Speed", active_descriptor->visual_scan_playback_speed,
        active_descriptor->visual_scan_playback_speed_deviation,
        0.01f, 0.01f, 8.0f, 0.0f, 8.0f,
        "Global playback time scale. Higher values run the scan faster.");
      changed |= InspectFloatMeanDeviation(
        "View Dwell (s)", active_descriptor->visual_scan_view_dwell_seconds,
        active_descriptor->visual_scan_view_dwell_seconds_deviation,
        0.005f, 0.01f, 5.0f, 0.0f, 5.0f,
        "Time spent at each scanner view while launching beams.");
      changed |= InspectFloatMeanDeviation(
        "Transition (s)", active_descriptor->visual_scan_transition_seconds,
        active_descriptor->visual_scan_transition_seconds_deviation,
        0.005f, 0.0f, 5.0f, 0.0f, 5.0f,
        "Travel time from current scanner view to the next view.");
      changed |= InspectIntMeanDeviation(
        "Beam Stride", active_descriptor->visual_scan_beam_stride,
        active_descriptor->visual_scan_beam_stride_deviation,
        1.0f, 1, 2048, 0, 2048,
        "Beam resolution control: 1 uses every traced ray. Larger values draw a sparser beam set.");
      changed |= InspectIntMeanDeviation(
        "Max Beams / View", active_descriptor->visual_scan_max_beams_per_view,
        active_descriptor->visual_scan_max_beams_per_view_deviation,
        1.0f, 1, 200000, 0, 200000,
        "Hard cap on rendered beams per scanner view after stride filtering.");
      changed |= InspectFloatMeanDeviation(
        "Beam Width", active_descriptor->visual_scan_beam_width,
        active_descriptor->visual_scan_beam_width_deviation,
        0.0005f, 0.0005f, 0.5f, 0.0f, 0.5f,
        "Cylinder thickness for rendered beams.");
      changed |= InspectFloatMeanDeviation(
        "Beam Alpha", active_descriptor->visual_scan_beam_alpha,
        active_descriptor->visual_scan_beam_alpha_deviation,
        0.01f, 0.0f, 1.0f, 0.0f, 1.0f,
        "Beam transparency: 0 is invisible, 1 is fully opaque.");
      changed |= InspectFloatMeanDeviation(
        "Beam Speed", active_descriptor->visual_scan_beam_speed,
        active_descriptor->visual_scan_beam_speed_deviation,
        0.05f, 0.001f, 500.0f, 0.0f, 500.0f,
        "Beam tip travel speed in world units per second.");
      changed |= InspectFloatMeanDeviation(
        "Beam Fire Fraction", active_descriptor->visual_scan_beam_fire_fraction,
        active_descriptor->visual_scan_beam_fire_fraction_deviation,
        0.01f, 0.0f, 1.0f, 0.0f, 1.0f,
        "Randomized fraction of candidate beams launched each view.");
      changed |= InspectFloatMeanDeviation(
        "Beam Max Length", active_descriptor->visual_scan_beam_max_length,
        active_descriptor->visual_scan_beam_max_length_deviation,
        0.01f, 0.01f, 100.0f, 0.0f, 100.0f,
        "Fallback beam length when a ray does not hit geometry.");
      changed |= InspectIntMeanDeviation(
        "Max Heat Points", active_descriptor->visual_scan_max_heat_points,
        active_descriptor->visual_scan_max_heat_points_deviation,
        1.0f, 1, 2000000, 0, 2000000,
        "Maximum number of fading hit points kept in memory and rendered.");
      changed |= InspectIntMeanDeviation(
        "Max Hits / View [deprecated]", active_descriptor->visual_scan_max_hits_per_view,
        active_descriptor->visual_scan_max_hits_per_view_deviation,
        1.0f, 1, 1000000, 0, 1000000,
        "Deprecated: playback now injects the full deterministic kept-hit set for parity with Run Scan and Capture.");
      changed |= InspectFloatMeanDeviation(
        "Heat Point Size", active_descriptor->visual_scan_point_size,
        active_descriptor->visual_scan_point_size_deviation,
        0.0005f, 0.0005f, 1.0f, 0.0f, 1.0f,
        "Sphere size used for rendered hit points.");
      changed |= InspectFloatMeanDeviation(
        "Scanner Size", active_descriptor->visual_scan_scanner_size,
        active_descriptor->visual_scan_scanner_size_deviation,
        0.0005f, 0.0005f, 1.0f, 0.0f, 1.0f,
        "Sphere size used for the scanner marker.");
      changed |= InspectFloatMeanDeviation(
        "Hot Duration (s)", active_descriptor->visual_scan_hot_seconds,
        active_descriptor->visual_scan_hot_seconds_deviation,
        0.01f, 0.01f, 10.0f, 0.0f, 10.0f,
        "Initial bright-red lifetime for each hit point.");
      changed |= InspectFloatMeanDeviation(
        "Warm Duration (s)", active_descriptor->visual_scan_warm_seconds,
        active_descriptor->visual_scan_warm_seconds_deviation,
        0.01f, 0.01f, 10.0f, 0.0f, 10.0f,
        "Mid-phase yellow/orange lifetime for each hit point.");
      changed |= InspectFloatMeanDeviation(
        "Cool Duration (s)", active_descriptor->visual_scan_cool_seconds,
        active_descriptor->visual_scan_cool_seconds_deviation,
        0.01f, 0.01f, 30.0f, 0.0f, 30.0f,
        "Final transition duration before each hit point settles to persistent black.");
      ImGui::EndTable();
    }

    const char* playback_state_name = "Unknown";
    switch (visual_scan_playback_state_) {
      case VisualPlaybackState::Idle:
        playback_state_name = "Idle";
        break;
      case VisualPlaybackState::Playing:
        playback_state_name = "Playing";
        break;
      case VisualPlaybackState::Finished:
        playback_state_name = "Finished";
        break;
    }

    ImGui::Separator();
    ImGui::Text("Playback State: %s", playback_state_name);
    ImGui::Text("View Batch: %llu / %llu",
                static_cast<unsigned long long>(visual_scan_batches_.empty()
                                                    ? 0
                                                    : (visual_scan_active_view_index_ + 1)),
                static_cast<unsigned long long>(visual_scan_batches_.size()));
    ImGui::Text("Rendered Beams: %llu", static_cast<unsigned long long>(visual_scan_last_rendered_beams_));
    ImGui::Text("Rendered Heat Points: %llu",
                static_cast<unsigned long long>(visual_scan_last_rendered_heat_points_));
    ImGui::Text("Injected Hits: %llu", static_cast<unsigned long long>(visual_scan_total_injected_hits_));
    ImGui::Text("Last Build Time (s): %.3f", visual_scan_last_build_seconds_);
    ImGui::TreePop();
  }

  active_descriptor->visual_scan_playback_speed = std::max(0.01f, active_descriptor->visual_scan_playback_speed);
    active_descriptor->visual_scan_playback_speed_deviation =
      std::max(0.0f, active_descriptor->visual_scan_playback_speed_deviation);
  active_descriptor->visual_scan_view_dwell_seconds =
      std::max(0.01f, active_descriptor->visual_scan_view_dwell_seconds);
    active_descriptor->visual_scan_view_dwell_seconds_deviation =
      std::max(0.0f, active_descriptor->visual_scan_view_dwell_seconds_deviation);
  active_descriptor->visual_scan_transition_seconds =
      std::max(0.0f, active_descriptor->visual_scan_transition_seconds);
    active_descriptor->visual_scan_transition_seconds_deviation =
      std::max(0.0f, active_descriptor->visual_scan_transition_seconds_deviation);
  active_descriptor->visual_scan_beam_stride = std::max(1, active_descriptor->visual_scan_beam_stride);
    active_descriptor->visual_scan_beam_stride_deviation =
      std::max(0, active_descriptor->visual_scan_beam_stride_deviation);
  active_descriptor->visual_scan_max_beams_per_view =
      std::max(1, active_descriptor->visual_scan_max_beams_per_view);
    active_descriptor->visual_scan_max_beams_per_view_deviation =
      std::max(0, active_descriptor->visual_scan_max_beams_per_view_deviation);
  active_descriptor->visual_scan_beam_width = std::max(0.0001f, active_descriptor->visual_scan_beam_width);
    active_descriptor->visual_scan_beam_width_deviation =
      std::max(0.0f, active_descriptor->visual_scan_beam_width_deviation);
  active_descriptor->visual_scan_beam_alpha = glm::clamp(active_descriptor->visual_scan_beam_alpha, 0.0f, 1.0f);
    active_descriptor->visual_scan_beam_alpha_deviation =
      std::max(0.0f, active_descriptor->visual_scan_beam_alpha_deviation);
  active_descriptor->visual_scan_beam_speed = std::max(0.001f, active_descriptor->visual_scan_beam_speed);
    active_descriptor->visual_scan_beam_speed_deviation =
      std::max(0.0f, active_descriptor->visual_scan_beam_speed_deviation);
  active_descriptor->visual_scan_beam_fire_fraction =
      glm::clamp(active_descriptor->visual_scan_beam_fire_fraction, 0.0f, 1.0f);
    active_descriptor->visual_scan_beam_fire_fraction_deviation =
      std::max(0.0f, active_descriptor->visual_scan_beam_fire_fraction_deviation);
  active_descriptor->visual_scan_beam_max_length = std::max(0.01f, active_descriptor->visual_scan_beam_max_length);
    active_descriptor->visual_scan_beam_max_length_deviation =
      std::max(0.0f, active_descriptor->visual_scan_beam_max_length_deviation);
  active_descriptor->visual_scan_max_heat_points = std::max(1, active_descriptor->visual_scan_max_heat_points);
    active_descriptor->visual_scan_max_heat_points_deviation =
      std::max(0, active_descriptor->visual_scan_max_heat_points_deviation);
  active_descriptor->visual_scan_max_hits_per_view = std::max(1, active_descriptor->visual_scan_max_hits_per_view);
    active_descriptor->visual_scan_max_hits_per_view_deviation =
      std::max(0, active_descriptor->visual_scan_max_hits_per_view_deviation);
  active_descriptor->visual_scan_point_size = std::max(0.0001f, active_descriptor->visual_scan_point_size);
    active_descriptor->visual_scan_point_size_deviation =
      std::max(0.0f, active_descriptor->visual_scan_point_size_deviation);
  active_descriptor->visual_scan_scanner_size = std::max(0.0001f, active_descriptor->visual_scan_scanner_size);
    active_descriptor->visual_scan_scanner_size_deviation =
      std::max(0.0f, active_descriptor->visual_scan_scanner_size_deviation);
  active_descriptor->visual_scan_hot_seconds = std::max(0.01f, active_descriptor->visual_scan_hot_seconds);
    active_descriptor->visual_scan_hot_seconds_deviation =
      std::max(0.0f, active_descriptor->visual_scan_hot_seconds_deviation);
  active_descriptor->visual_scan_warm_seconds = std::max(0.01f, active_descriptor->visual_scan_warm_seconds);
    active_descriptor->visual_scan_warm_seconds_deviation =
      std::max(0.0f, active_descriptor->visual_scan_warm_seconds_deviation);
  active_descriptor->visual_scan_cool_seconds = std::max(0.01f, active_descriptor->visual_scan_cool_seconds);
    active_descriptor->visual_scan_cool_seconds_deviation =
      std::max(0.0f, active_descriptor->visual_scan_cool_seconds_deviation);

  one_shot_repeat_hz_ = std::max(0.0f, one_shot_repeat_hz_);
  one_shot_repeat_hz_deviation_ = std::max(0.0f, one_shot_repeat_hz_deviation_);
  if (private_scan_mode_ == PrivateScanMode::OneShot && one_shot_repeat_hz_ > 0.0f) {
    std::mt19937 one_shot_repeat_rng(MixSeed(
        active_descriptor->scan_seed + static_cast<int>(visual_scan_total_injected_hits_ & 0x7FFFFFFFu),
        kOneShotRepeatSeedSalt));
    const float sampled_repeat_hz = SampleFloatMeanDeviation(
        one_shot_repeat_rng, one_shot_repeat_hz_, one_shot_repeat_hz_deviation_);
    const float effective_repeat_hz = sampled_repeat_hz > 0.0f ? sampled_repeat_hz : one_shot_repeat_hz_;
    if (effective_repeat_hz > 0.0f) {
      const double now_seconds = GetSteadyTimeSeconds();
      const double period = 1.0 / static_cast<double>(std::max(0.1f, effective_repeat_hz));
      if (one_shot_last_trigger_seconds_ < 0.0 ||
          (now_seconds - one_shot_last_trigger_seconds_) >= period) {
        run_scan_for_current_mode();
      }
    }
  }

  if (!Application::IsPlaying() &&
      visual_scan_playback_state_ == VisualPlaybackState::Playing) {
    const double now_seconds = GetSteadyTimeSeconds();
    double delta_seconds = 0.0;
    if (visual_scan_last_tick_seconds_ >= 0.0) {
      delta_seconds = std::max(0.0, now_seconds - visual_scan_last_tick_seconds_);
    }
    visual_scan_last_tick_seconds_ = now_seconds;
    TickVisualScanPlayback(active_descriptor, delta_seconds);
  }

  if (!Application::IsPlaying()) {
    RenderVisualScanPlayback(editor_layer, active_descriptor);
  }

  return changed;
}

void TasselPointCloudScanner::OnDestroy() {
  scanner_descriptor_ref_.Clear();
  visual_scan_beam_particle_info_ref_.Clear();
  visual_scan_heat_particle_info_ref_.Clear();
  visual_scan_playback_state_ = VisualPlaybackState::Idle;
  visual_scan_last_tick_seconds_ = -1.0;
  visual_scan_last_build_seconds_ = 0.0;
  one_shot_last_trigger_seconds_ = -1.0;
  ClearVisualScanRuntimeState(true, true);
}

void TasselPointCloudScanner::Serialize(YAML::Emitter& out) const {
  scanner_descriptor_ref_.Save("scanner_descriptor_ref", out);

  AssetRef descriptor_ref_copy = scanner_descriptor_ref_;
  const auto active_descriptor = descriptor_ref_copy.Get<TasselPointCloudScannerDescriptor>();
  if (!active_descriptor) {
    return;
  }

  // Persist a complete component-local backup of scanner settings so scene save/load
  // round-trips every field even if the descriptor asset cannot be resolved later.
  active_descriptor->point_settings.Save("point_settings", out);
  active_descriptor->capture_settings.Save("capture_settings", out);
  active_descriptor->point_settings.Save("point_settings_backup", out);
  active_descriptor->capture_settings.Save("capture_settings_backup", out);

  out << YAML::Key << "gpu_only_mode" << YAML::Value << false;
  out << YAML::Key << "scan_seed" << YAML::Value << active_descriptor->scan_seed;
  out << YAML::Key << "private_scan_mode" << YAML::Value << static_cast<int>(private_scan_mode_);
  SaveFloatMeanDeviation(out, "one_shot_repeat_hz", one_shot_repeat_hz_, one_shot_repeat_hz_deviation_);
  out << YAML::Key << "advance_seed_per_playback" << YAML::Value << advance_seed_per_playback_;

  out << YAML::Key << "visual_scan_interactive_enabled" << YAML::Value
    << active_descriptor->visual_scan_interactive_enabled;
  out << YAML::Key << "visual_scan_show_scanner" << YAML::Value << active_descriptor->visual_scan_show_scanner;
  out << YAML::Key << "visual_scan_show_beams" << YAML::Value << active_descriptor->visual_scan_show_beams;
  out << YAML::Key << "visual_scan_show_heat_points" << YAML::Value
    << active_descriptor->visual_scan_show_heat_points;
  out << YAML::Key << "visual_scan_depth_test_beams" << YAML::Value
    << active_descriptor->visual_scan_depth_test_beams;
  out << YAML::Key << "visual_scan_depth_test_heat_points" << YAML::Value
    << active_descriptor->visual_scan_depth_test_heat_points;
  out << YAML::Key << "visual_scan_interpolate_motion" << YAML::Value
    << active_descriptor->visual_scan_interpolate_motion;
  out << YAML::Key << "visual_scan_keep_points_after_finish" << YAML::Value
    << active_descriptor->visual_scan_keep_points_after_finish;
  SaveFloatMeanDeviation(out, "visual_scan_playback_speed", active_descriptor->visual_scan_playback_speed,
                         active_descriptor->visual_scan_playback_speed_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_view_dwell_seconds", active_descriptor->visual_scan_view_dwell_seconds,
                         active_descriptor->visual_scan_view_dwell_seconds_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_transition_seconds", active_descriptor->visual_scan_transition_seconds,
                         active_descriptor->visual_scan_transition_seconds_deviation);
  SaveIntMeanDeviation(out, "visual_scan_beam_stride", active_descriptor->visual_scan_beam_stride,
                       active_descriptor->visual_scan_beam_stride_deviation);
  SaveIntMeanDeviation(out, "visual_scan_max_beams_per_view", active_descriptor->visual_scan_max_beams_per_view,
                       active_descriptor->visual_scan_max_beams_per_view_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_width", active_descriptor->visual_scan_beam_width,
                         active_descriptor->visual_scan_beam_width_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_alpha", active_descriptor->visual_scan_beam_alpha,
                         active_descriptor->visual_scan_beam_alpha_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_speed", active_descriptor->visual_scan_beam_speed,
                         active_descriptor->visual_scan_beam_speed_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_fire_fraction", active_descriptor->visual_scan_beam_fire_fraction,
                         active_descriptor->visual_scan_beam_fire_fraction_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_beam_max_length", active_descriptor->visual_scan_beam_max_length,
                         active_descriptor->visual_scan_beam_max_length_deviation);
  SaveIntMeanDeviation(out, "visual_scan_max_heat_points", active_descriptor->visual_scan_max_heat_points,
                       active_descriptor->visual_scan_max_heat_points_deviation);
  SaveIntMeanDeviation(out, "visual_scan_max_hits_per_view", active_descriptor->visual_scan_max_hits_per_view,
                       active_descriptor->visual_scan_max_hits_per_view_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_point_size", active_descriptor->visual_scan_point_size,
                         active_descriptor->visual_scan_point_size_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_scanner_size", active_descriptor->visual_scan_scanner_size,
                         active_descriptor->visual_scan_scanner_size_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_hot_seconds", active_descriptor->visual_scan_hot_seconds,
                         active_descriptor->visual_scan_hot_seconds_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_warm_seconds", active_descriptor->visual_scan_warm_seconds,
                         active_descriptor->visual_scan_warm_seconds_deviation);
  SaveFloatMeanDeviation(out, "visual_scan_cool_seconds", active_descriptor->visual_scan_cool_seconds,
                         active_descriptor->visual_scan_cool_seconds_deviation);
}

void TasselPointCloudScanner::Deserialize(const YAML::Node& in) {
  scanner_descriptor_ref_.Load("scanner_descriptor_ref", in);

  auto active_descriptor = scanner_descriptor_ref_.Get<TasselPointCloudScannerDescriptor>();
  if (active_descriptor) {
    // When scene-local scanner settings are present, apply them even if descriptor
    // reference resolves, so all fields round-trip through scene save/load.
    active_descriptor->Deserialize(in);
  }

  if (in["private_scan_mode"]) {
    const int loaded_mode = in["private_scan_mode"].as<int>();
    private_scan_mode_ = loaded_mode == static_cast<int>(PrivateScanMode::Animated)
                             ? PrivateScanMode::Animated
                             : PrivateScanMode::OneShot;
  } else if (active_descriptor && active_descriptor->visual_scan_interactive_enabled) {
    private_scan_mode_ = PrivateScanMode::Animated;
  } else {
    private_scan_mode_ = PrivateScanMode::OneShot;
  }

  one_shot_repeat_hz_ = 0.0f;
  one_shot_repeat_hz_deviation_ = 0.0f;
  LoadFloatMeanDeviation(in, "one_shot_repeat_hz", one_shot_repeat_hz_, one_shot_repeat_hz_deviation_);
  one_shot_repeat_hz_ = std::max(0.0f, one_shot_repeat_hz_);
  one_shot_repeat_hz_deviation_ = std::max(0.0f, one_shot_repeat_hz_deviation_);

  if (in["advance_seed_per_playback"]) {
    advance_seed_per_playback_ = in["advance_seed_per_playback"].as<bool>();
  } else {
    advance_seed_per_playback_ = false;
  }

  if (!active_descriptor) {
    const auto descriptor = AssetManager::CreateTemporaryAsset<TasselPointCloudScannerDescriptor>();

    // Migrate component-local scanner settings into a required descriptor asset.
    descriptor->point_settings.Load("point_settings_backup", in);
    descriptor->capture_settings.Load("capture_settings_backup", in);
    if (in["scan_seed"]) {
      descriptor->scan_seed = in["scan_seed"].as<int>();
    }
    if (in["visual_scan_interactive_enabled"]) {
      descriptor->visual_scan_interactive_enabled = in["visual_scan_interactive_enabled"].as<bool>();
    }
    if (in["visual_scan_show_scanner"]) {
      descriptor->visual_scan_show_scanner = in["visual_scan_show_scanner"].as<bool>();
    }
    if (in["visual_scan_show_beams"]) {
      descriptor->visual_scan_show_beams = in["visual_scan_show_beams"].as<bool>();
    }
    if (in["visual_scan_show_heat_points"]) {
      descriptor->visual_scan_show_heat_points = in["visual_scan_show_heat_points"].as<bool>();
    }
    if (in["visual_scan_depth_test_beams"]) {
      descriptor->visual_scan_depth_test_beams = in["visual_scan_depth_test_beams"].as<bool>();
    }
    if (in["visual_scan_depth_test_heat_points"]) {
      descriptor->visual_scan_depth_test_heat_points = in["visual_scan_depth_test_heat_points"].as<bool>();
    }
    if (in["visual_scan_interpolate_motion"]) {
      descriptor->visual_scan_interpolate_motion = in["visual_scan_interpolate_motion"].as<bool>();
    }
    if (in["visual_scan_keep_points_after_finish"]) {
      descriptor->visual_scan_keep_points_after_finish = in["visual_scan_keep_points_after_finish"].as<bool>();
    }
    LoadFloatMeanDeviation(in, "visual_scan_playback_speed", descriptor->visual_scan_playback_speed,
                           descriptor->visual_scan_playback_speed_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_view_dwell_seconds", descriptor->visual_scan_view_dwell_seconds,
                           descriptor->visual_scan_view_dwell_seconds_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_transition_seconds", descriptor->visual_scan_transition_seconds,
                           descriptor->visual_scan_transition_seconds_deviation);
    LoadIntMeanDeviation(in, "visual_scan_beam_stride", descriptor->visual_scan_beam_stride,
                         descriptor->visual_scan_beam_stride_deviation);
    LoadIntMeanDeviation(in, "visual_scan_max_beams_per_view", descriptor->visual_scan_max_beams_per_view,
                         descriptor->visual_scan_max_beams_per_view_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_beam_width", descriptor->visual_scan_beam_width,
                           descriptor->visual_scan_beam_width_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_beam_alpha", descriptor->visual_scan_beam_alpha,
                           descriptor->visual_scan_beam_alpha_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_beam_speed", descriptor->visual_scan_beam_speed,
                           descriptor->visual_scan_beam_speed_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_beam_fire_fraction", descriptor->visual_scan_beam_fire_fraction,
                           descriptor->visual_scan_beam_fire_fraction_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_beam_max_length", descriptor->visual_scan_beam_max_length,
                           descriptor->visual_scan_beam_max_length_deviation);
    LoadIntMeanDeviation(in, "visual_scan_max_heat_points", descriptor->visual_scan_max_heat_points,
                         descriptor->visual_scan_max_heat_points_deviation);
    LoadIntMeanDeviation(in, "visual_scan_max_hits_per_view", descriptor->visual_scan_max_hits_per_view,
                         descriptor->visual_scan_max_hits_per_view_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_point_size", descriptor->visual_scan_point_size,
                           descriptor->visual_scan_point_size_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_scanner_size", descriptor->visual_scan_scanner_size,
                           descriptor->visual_scan_scanner_size_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_hot_seconds", descriptor->visual_scan_hot_seconds,
                           descriptor->visual_scan_hot_seconds_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_warm_seconds", descriptor->visual_scan_warm_seconds,
                           descriptor->visual_scan_warm_seconds_deviation);
    LoadFloatMeanDeviation(in, "visual_scan_cool_seconds", descriptor->visual_scan_cool_seconds,
                           descriptor->visual_scan_cool_seconds_deviation);
    descriptor->visual_scan_playback_speed = std::max(0.01f, descriptor->visual_scan_playback_speed);
    descriptor->visual_scan_playback_speed_deviation =
      std::max(0.0f, descriptor->visual_scan_playback_speed_deviation);
    descriptor->visual_scan_view_dwell_seconds = std::max(0.01f, descriptor->visual_scan_view_dwell_seconds);
    descriptor->visual_scan_view_dwell_seconds_deviation =
      std::max(0.0f, descriptor->visual_scan_view_dwell_seconds_deviation);
    descriptor->visual_scan_transition_seconds = std::max(0.0f, descriptor->visual_scan_transition_seconds);
    descriptor->visual_scan_transition_seconds_deviation =
      std::max(0.0f, descriptor->visual_scan_transition_seconds_deviation);
    descriptor->visual_scan_beam_stride = std::max(1, descriptor->visual_scan_beam_stride);
    descriptor->visual_scan_beam_stride_deviation =
      std::max(0, descriptor->visual_scan_beam_stride_deviation);
    descriptor->visual_scan_max_beams_per_view = std::max(1, descriptor->visual_scan_max_beams_per_view);
    descriptor->visual_scan_max_beams_per_view_deviation =
      std::max(0, descriptor->visual_scan_max_beams_per_view_deviation);
    descriptor->visual_scan_beam_width = std::max(0.0001f, descriptor->visual_scan_beam_width);
    descriptor->visual_scan_beam_width_deviation =
      std::max(0.0f, descriptor->visual_scan_beam_width_deviation);
    descriptor->visual_scan_beam_alpha = glm::clamp(descriptor->visual_scan_beam_alpha, 0.0f, 1.0f);
    descriptor->visual_scan_beam_alpha_deviation =
      std::max(0.0f, descriptor->visual_scan_beam_alpha_deviation);
    descriptor->visual_scan_beam_speed = std::max(0.001f, descriptor->visual_scan_beam_speed);
    descriptor->visual_scan_beam_speed_deviation =
      std::max(0.0f, descriptor->visual_scan_beam_speed_deviation);
    descriptor->visual_scan_beam_fire_fraction =
      glm::clamp(descriptor->visual_scan_beam_fire_fraction, 0.0f, 1.0f);
    descriptor->visual_scan_beam_fire_fraction_deviation =
      std::max(0.0f, descriptor->visual_scan_beam_fire_fraction_deviation);
    descriptor->visual_scan_beam_max_length = std::max(0.01f, descriptor->visual_scan_beam_max_length);
    descriptor->visual_scan_beam_max_length_deviation =
      std::max(0.0f, descriptor->visual_scan_beam_max_length_deviation);
    descriptor->visual_scan_max_heat_points = std::max(1, descriptor->visual_scan_max_heat_points);
    descriptor->visual_scan_max_heat_points_deviation =
      std::max(0, descriptor->visual_scan_max_heat_points_deviation);
    descriptor->visual_scan_max_hits_per_view = std::max(1, descriptor->visual_scan_max_hits_per_view);
    descriptor->visual_scan_max_hits_per_view_deviation =
      std::max(0, descriptor->visual_scan_max_hits_per_view_deviation);
    descriptor->visual_scan_point_size = std::max(0.0001f, descriptor->visual_scan_point_size);
    descriptor->visual_scan_point_size_deviation =
      std::max(0.0f, descriptor->visual_scan_point_size_deviation);
    descriptor->visual_scan_scanner_size = std::max(0.0001f, descriptor->visual_scan_scanner_size);
    descriptor->visual_scan_scanner_size_deviation =
      std::max(0.0f, descriptor->visual_scan_scanner_size_deviation);
    descriptor->visual_scan_hot_seconds = std::max(0.01f, descriptor->visual_scan_hot_seconds);
    descriptor->visual_scan_hot_seconds_deviation =
      std::max(0.0f, descriptor->visual_scan_hot_seconds_deviation);
    descriptor->visual_scan_warm_seconds = std::max(0.01f, descriptor->visual_scan_warm_seconds);
    descriptor->visual_scan_warm_seconds_deviation =
      std::max(0.0f, descriptor->visual_scan_warm_seconds_deviation);
    descriptor->visual_scan_cool_seconds = std::max(0.01f, descriptor->visual_scan_cool_seconds);
    descriptor->visual_scan_cool_seconds_deviation =
      std::max(0.0f, descriptor->visual_scan_cool_seconds_deviation);
    descriptor->visual_scan_keep_points_after_finish = true;
    descriptor->gpu_only_mode = false;

    scanner_descriptor_ref_ = descriptor;
    active_descriptor = descriptor;

    if (in["point_settings_backup"] || in["capture_settings_backup"] ||
      in["gpu_only_mode"] || in["scan_seed"] ||
        in["visual_scan_interactive_enabled"] || in["visual_scan_show_scanner"] ||
      in["visual_scan_show_beams"] || in["visual_scan_show_heat_points"] ||
      in["visual_scan_depth_test_beams"] || in["visual_scan_depth_test_heat_points"] ||
        in["visual_scan_interpolate_motion"] || in["visual_scan_keep_points_after_finish"] ||
        in["visual_scan_playback_speed"] || in["visual_scan_view_dwell_seconds"] ||
        in["visual_scan_transition_seconds"] || in["visual_scan_beam_stride"] ||
        in["visual_scan_max_beams_per_view"] || in["visual_scan_beam_width"] ||
        in["visual_scan_beam_alpha"] || in["visual_scan_beam_speed"] ||
        in["visual_scan_beam_fire_fraction"] ||
        in["visual_scan_beam_max_length"] || in["visual_scan_max_heat_points"] ||
        in["visual_scan_max_hits_per_view"] || in["visual_scan_point_size"] ||
        in["visual_scan_scanner_size"] || in["visual_scan_hot_seconds"] ||
        in["visual_scan_warm_seconds"] || in["visual_scan_cool_seconds"]) {
      descriptor->SetUnsaved();
    }
  }

  if (active_descriptor) {
    bool descriptor_changed = false;
    if (active_descriptor->gpu_only_mode) {
      active_descriptor->gpu_only_mode = false;
      descriptor_changed = true;
    }
    if (descriptor_changed) {
      active_descriptor->SetUnsaved();
    }
  }

  one_shot_last_trigger_seconds_ = -1.0;
  one_shot_repeat_hz_ = std::max(0.0f, one_shot_repeat_hz_);
  one_shot_repeat_hz_deviation_ = std::max(0.0f, one_shot_repeat_hz_deviation_);

  visual_scan_playback_state_ = VisualPlaybackState::Idle;
  visual_scan_last_tick_seconds_ = -1.0;
  visual_scan_last_build_seconds_ = 0.0;
  ClearVisualScanRuntimeState(true, true);
}

void TasselPointCloudScanner::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(scanner_descriptor_ref_);
}
