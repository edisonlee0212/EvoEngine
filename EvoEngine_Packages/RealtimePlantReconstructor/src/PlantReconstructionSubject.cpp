#include "PlantReconstructionSubject.hpp"

#include "AssetManager.hpp"
#include "Jobs.hpp"
#include "Scene.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <unordered_map>

using namespace realtime_plant_reconstructor;

namespace {
void WritePath(YAML::Emitter& out, const char* key, const std::filesystem::path& path) {
  out << YAML::Key << key << YAML::Value << path.string();
}

std::filesystem::path ReadPath(const YAML::Node& in, const char* key) {
  const auto node = in[key];
  return node && node.IsScalar() ? std::filesystem::path(node.as<std::string>()) : std::filesystem::path();
}

std::string ReadString(const YAML::Node& in, const char* key, const std::string& fallback = {}) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<std::string>() : fallback;
}

float ReadFloat(const YAML::Node& in, const char* key, const float fallback) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<float>() : fallback;
}

int ReadInt(const YAML::Node& in, const char* key, const int fallback) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<int>() : fallback;
}

bool ReadBool(const YAML::Node& in, const char* key, const bool fallback) {
  const auto node = in[key];
  return node && node.IsScalar() ? node.as<bool>() : fallback;
}

struct GridKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GridKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridKeyHash {
  size_t operator()(const GridKey& key) const {
    size_t seed = 1469598103934665603ull;
    const auto mix = [&](const int value) {
      seed ^= static_cast<size_t>(value) + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
  }
};

GridKey MakeGridKey(const glm::dvec3& position, const double cell_size) {
  return {static_cast<int>(std::floor(position.x / cell_size)), static_cast<int>(std::floor(position.y / cell_size)),
          static_cast<int>(std::floor(position.z / cell_size))};
}

double SquaredDistance(const glm::dvec3& lhs, const glm::dvec3& rhs) {
  const auto delta = lhs - rhs;
  return glm::dot(delta, delta);
}

glm::vec4 DensityColor(const float value, const float opacity) {
  const auto t = glm::clamp(value, 0.0f, 1.0f);
  const glm::vec3 low(0.08f, 0.22f, 0.36f);
  const glm::vec3 mid(0.10f, 0.70f, 0.48f);
  const glm::vec3 high(0.95f, 0.72f, 0.16f);
  const auto color = t < 0.5f ? glm::mix(low, mid, t * 2.0f) : glm::mix(mid, high, (t - 0.5f) * 2.0f);
  return glm::vec4(color, glm::clamp(opacity, 0.0f, 1.0f));
}
}  // namespace

void PlantReconstructionSubject::OnCreate() {
  SeedRawPipelineStage();
  MarkPreviewDirty();
}

void PlantReconstructionSubject::OnDestroy() {
  loaded_point_cloud_.reset();
  preview_particles_.reset();
  neighborhood_field_ = {};
}

void PlantReconstructionSubject::SetPointCloudAsset(const std::shared_ptr<PointCloud>& point_cloud,
                                                    const ImportMetadata& metadata) {
  import_metadata_ = metadata;
  loaded_point_cloud_ = point_cloud;
  if (point_cloud) {
    point_cloud_asset_handle_ = point_cloud->GetHandle();
    point_cloud_asset_type_name_ = point_cloud->GetTypeName();
    total_point_count_ = point_cloud->positions.size();
    min_bound_ = point_cloud->GetMinBound();
    max_bound_ = point_cloud->GetMaxBound();
    status_ = "Loaded.";
  } else {
    point_cloud_asset_handle_ = Handle(0);
    point_cloud_asset_type_name_ = "PointCloud";
    total_point_count_ = 0;
    displayed_point_count_ = 0;
    min_bound_ = glm::dvec3(0.0);
    max_bound_ = glm::dvec3(0.0);
    status_ = "No point cloud.";
  }
  SeedRawPipelineStage();
  MarkNeighborhoodDirty();
  MarkPreviewDirty();
}

void PlantReconstructionSubject::DrawEditorPreview(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (!show_preview || !editor_layer) {
    return;
  }
  if (preview_dirty_ && !RebuildPreview()) {
    return;
  }
  if (!preview_particles_ || preview_particles_->PeekParticleInfoList().empty()) {
    return;
  }
  const auto scene = GetScene();
  if (!scene || !scene->IsEntityValid(GetOwner())) {
    return;
  }

  GizmoSettings settings{};
  settings.draw_settings.blending = true;
  settings.depth_test = true;
  settings.depth_write = false;
  editor_layer->DrawGizmoSpheres(preview_particles_, scene->GetDataComponent<GlobalTransform>(GetOwner()).value, 1.0f,
                                 settings);
}

void PlantReconstructionSubject::MarkPreviewDirty() {
  preview_dirty_ = true;
}

void PlantReconstructionSubject::MarkNeighborhoodDirty() {
  neighborhood_field_ = {};
  neighborhood_dirty_ = true;
  neighborhood_status_ = "Dirty.";
  for (auto& stage : pipeline_stages_) {
    if (stage.id == 2 || stage.operator_key == "NeighborhoodField.CpuPreviewKnn") {
      stage.status = "Dirty";
      stage.settings_hash = "k=" + std::to_string(neighborhood_k) + ";max=" + std::to_string(max_neighborhood_points);
      stage.output_hash.clear();
    }
  }
  if (show_neighborhood_density) {
    MarkPreviewDirty();
  }
}

bool PlantReconstructionSubject::EnsurePointCloudLoaded() {
  if (loaded_point_cloud_) {
    return true;
  }
  if (point_cloud_asset_handle_.GetValue() == 0) {
    status_ = "No point-cloud asset handle.";
    return false;
  }
  try {
    loaded_point_cloud_ = AssetManager::GetAsset<PointCloud>(point_cloud_asset_handle_);
  } catch (const std::exception& error) {
    status_ = std::string("Failed to load point-cloud asset: ") + error.what();
    return false;
  }
  if (!loaded_point_cloud_) {
    status_ = "Asset is not a PointCloud.";
    return false;
  }
  total_point_count_ = loaded_point_cloud_->positions.size();
  min_bound_ = loaded_point_cloud_->GetMinBound();
  max_bound_ = loaded_point_cloud_->GetMaxBound();
  status_ = "Loaded.";
  return true;
}

bool PlantReconstructionSubject::RebuildPreview() {
  if (!EnsurePointCloudLoaded()) {
    return false;
  }
  if (!preview_particles_) {
    preview_particles_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }
  const auto total = loaded_point_cloud_->positions.size();
  if (total == 0) {
    preview_particles_->SetParticleInfos({});
    displayed_point_count_ = 0;
    preview_dirty_ = false;
    status_ = "Loaded empty point cloud.";
    return true;
  }

  const bool use_density =
      show_neighborhood_density && neighborhood_field_.valid && !neighborhood_field_.points.empty();
  const auto max_points = static_cast<size_t>(std::max(1, max_preview_points));
  const auto stride = total > max_points ? (total + max_points - 1) / max_points : size_t{1};
  displayed_point_count_ = use_density ? neighborhood_field_.points.size() : (total + stride - 1) / stride;

  const float base_size_mm = import_metadata_.view_point_size_mm > 0.0f
                                 ? import_metadata_.view_point_size_mm * point_size_multiplier
                                 : point_size_mm;
  const float point_size_m = std::max(0.0001f, base_size_mm * 0.001f);
  const glm::vec4 uniform_color(point_color, glm::clamp(point_opacity, 0.0f, 1.0f));
  const bool use_colors = use_ply_colors && loaded_point_cloud_->colors.size() == total;

  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(displayed_point_count_);
  for (size_t i = 0; i < displayed_point_count_; ++i) {
    const auto point_index = use_density ? std::min(neighborhood_field_.points[i].source_point_index, total - 1)
                                         : std::min(i * stride, total - 1);
    const auto position = loaded_point_cloud_->positions[point_index] + loaded_point_cloud_->offset;
    auto& info = particle_infos[i];
    info.instance_matrix.SetPosition(glm::vec3(position));
    info.instance_matrix.SetScale(glm::vec3(point_size_m));
    if (use_density) {
      info.instance_color = DensityColor(neighborhood_field_.points[i].normalized_density, point_opacity);
    } else if (use_colors) {
      info.instance_color = glm::clamp(loaded_point_cloud_->colors[point_index], glm::vec4(0.0f), glm::vec4(1.0f));
      info.instance_color.w = glm::clamp(info.instance_color.w * point_opacity, 0.0f, 1.0f);
    } else {
      info.instance_color = uniform_color;
    }
  }
  preview_particles_->SetParticleInfos(particle_infos);
  preview_dirty_ = false;
  status_ = use_density ? "Neighborhood density preview ready." : "Preview ready.";
  return true;
}

bool PlantReconstructionSubject::BuildNeighborhoodField() {
  if (!EnsurePointCloudLoaded()) {
    neighborhood_status_ = status_;
    return false;
  }
  const auto total = loaded_point_cloud_->positions.size();
  if (total < 2) {
    neighborhood_field_ = {};
    neighborhood_field_.source_point_count = total;
    neighborhood_field_.valid = true;
    neighborhood_dirty_ = false;
    neighborhood_status_ = "Point cloud has fewer than two points.";
    UpsertNeighborhoodStage("Ready");
    MarkPreviewDirty();
    return true;
  }

  const auto max_points =
      std::min(total, static_cast<size_t>(std::max(2, std::min(max_neighborhood_points, max_preview_points))));
  const auto stride = total > max_points ? (total + max_points - 1) / max_points : size_t{1};
  const auto sample_count = (total + stride - 1) / stride;
  const auto k = std::clamp(neighborhood_k, 1, std::min(64, static_cast<int>(sample_count - 1)));

  std::vector<size_t> source_indices(sample_count);
  std::vector<glm::dvec3> points(sample_count);
  glm::dvec3 sample_min(std::numeric_limits<double>::max());
  glm::dvec3 sample_max(std::numeric_limits<double>::lowest());
  for (size_t i = 0; i < sample_count; ++i) {
    const auto point_index = std::min(i * stride, total - 1);
    const auto position = loaded_point_cloud_->positions[point_index] + loaded_point_cloud_->offset;
    source_indices[i] = point_index;
    points[i] = position;
    sample_min = glm::min(sample_min, position);
    sample_max = glm::max(sample_max, position);
  }

  const auto extent = sample_max - sample_min;
  const auto diagonal = std::sqrt(glm::dot(extent, extent));
  const auto cell_size = std::max(diagonal / std::cbrt(static_cast<double>(sample_count)), 1e-6);
  std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> grid;
  grid.reserve(sample_count * 2);
  for (size_t i = 0; i < sample_count; ++i) {
    grid[MakeGridKey(points[i], cell_size)].push_back(i);
  }

  NeighborhoodField field;
  field.points.resize(sample_count);
  field.source_point_count = total;
  field.stride = stride;
  field.k = k;

  const auto& grid_ref = grid;
  Jobs::RunParallelFor(sample_count, [&](const size_t i) {
    const auto center_key = MakeGridKey(points[i], cell_size);
    std::vector<double> distances;
    distances.reserve(static_cast<size_t>(k * 8));
    for (int radius = 1; radius <= 8; ++radius) {
      distances.clear();
      for (int z = center_key.z - radius; z <= center_key.z + radius; ++z) {
        for (int y = center_key.y - radius; y <= center_key.y + radius; ++y) {
          for (int x = center_key.x - radius; x <= center_key.x + radius; ++x) {
            const auto cell = grid_ref.find({x, y, z});
            if (cell == grid_ref.end()) {
              continue;
            }
            for (const auto candidate_index : cell->second) {
              if (candidate_index != i) {
                distances.push_back(SquaredDistance(points[i], points[candidate_index]));
              }
            }
          }
        }
      }
      if (distances.size() >= static_cast<size_t>(k) || radius == 8) {
        break;
      }
    }

    if (distances.empty()) {
      const auto fallback_stride = std::max<size_t>(1, sample_count / 4096);
      for (size_t candidate_index = 0; candidate_index < sample_count; candidate_index += fallback_stride) {
        if (candidate_index != i) {
          distances.push_back(SquaredDistance(points[i], points[candidate_index]));
        }
      }
    }

    auto& info = field.points[i];
    info.source_point_index = source_indices[i];
    if (!distances.empty()) {
      const auto rank = std::min(static_cast<size_t>(k), distances.size()) - 1;
      std::nth_element(distances.begin(), distances.begin() + static_cast<ptrdiff_t>(rank), distances.end());
      info.local_spacing = static_cast<float>(std::sqrt(std::max(distances[rank], 0.0)));
      info.density = 1.0f / std::max(info.local_spacing, 0.000001f);
    }
  });

  float spacing_sum = 0.0f;
  size_t valid_spacing_count = 0;
  field.min_spacing = std::numeric_limits<float>::max();
  field.max_spacing = 0.0f;
  for (const auto& point : field.points) {
    if (point.local_spacing <= 0.0f) {
      continue;
    }
    field.min_spacing = std::min(field.min_spacing, point.local_spacing);
    field.max_spacing = std::max(field.max_spacing, point.local_spacing);
    spacing_sum += point.local_spacing;
    ++valid_spacing_count;
  }
  if (valid_spacing_count == 0) {
    field.min_spacing = 0.0f;
  } else {
    field.mean_spacing = spacing_sum / static_cast<float>(valid_spacing_count);
  }

  const auto spacing_range = field.max_spacing - field.min_spacing;
  for (auto& point : field.points) {
    point.normalized_density =
        spacing_range > 0.0f ? 1.0f - glm::clamp((point.local_spacing - field.min_spacing) / spacing_range, 0.0f, 1.0f)
                             : 0.5f;
  }

  field.valid = true;
  neighborhood_field_ = std::move(field);
  neighborhood_dirty_ = false;
  neighborhood_status_ = "Ready.";
  UpsertNeighborhoodStage("Ready");
  MarkPreviewDirty();
  return true;
}

Handle PlantReconstructionSubject::GetPointCloudAssetHandle() const {
  return point_cloud_asset_handle_;
}

const std::string& PlantReconstructionSubject::GetPointCloudAssetTypeName() const {
  return point_cloud_asset_type_name_;
}

const PlantReconstructionSubject::ImportMetadata& PlantReconstructionSubject::GetImportMetadata() const {
  return import_metadata_;
}

const std::vector<PlantReconstructionSubject::StageRecord>& PlantReconstructionSubject::GetPipelineStages() const {
  return pipeline_stages_;
}

bool PlantReconstructionSubject::HasNeighborhoodField() const {
  return neighborhood_field_.valid && !neighborhood_field_.points.empty();
}

size_t PlantReconstructionSubject::GetNeighborhoodPointCount() const {
  return neighborhood_field_.points.size();
}

float PlantReconstructionSubject::GetNeighborhoodMinSpacing() const {
  return neighborhood_field_.min_spacing;
}

float PlantReconstructionSubject::GetNeighborhoodMeanSpacing() const {
  return neighborhood_field_.mean_spacing;
}

float PlantReconstructionSubject::GetNeighborhoodMaxSpacing() const {
  return neighborhood_field_.max_spacing;
}

const std::string& PlantReconstructionSubject::GetNeighborhoodStatus() const {
  return neighborhood_status_;
}

size_t PlantReconstructionSubject::GetTotalPointCount() const {
  return total_point_count_;
}

size_t PlantReconstructionSubject::GetDisplayedPointCount() const {
  return displayed_point_count_;
}

const glm::dvec3& PlantReconstructionSubject::GetMinBound() const {
  return min_bound_;
}

const glm::dvec3& PlantReconstructionSubject::GetMaxBound() const {
  return max_bound_;
}

bool PlantReconstructionSubject::GetLocalBound(Bound& bound) const {
  if (total_point_count_ == 0) {
    return false;
  }
  const auto offset = loaded_point_cloud_ ? loaded_point_cloud_->offset : glm::dvec3(0.0);
  const auto min_bound = min_bound_ + offset;
  const auto max_bound = max_bound_ + offset;
  bound.min = glm::vec3(min_bound);
  bound.max = glm::vec3(max_bound);
  return std::isfinite(bound.min.x) && std::isfinite(bound.min.y) && std::isfinite(bound.min.z) &&
         std::isfinite(bound.max.x) && std::isfinite(bound.max.y) && std::isfinite(bound.max.z) &&
         bound.min.x <= bound.max.x && bound.min.y <= bound.max.y && bound.min.z <= bound.max.z;
}

const std::string& PlantReconstructionSubject::GetStatus() const {
  return status_;
}

void PlantReconstructionSubject::SeedRawPipelineStage() {
  if (!pipeline_stages_.empty()) {
    return;
  }
  pipeline_stages_.push_back({});
}

void PlantReconstructionSubject::UpsertNeighborhoodStage(const std::string& status) {
  SeedRawPipelineStage();
  auto stage = std::find_if(pipeline_stages_.begin(), pipeline_stages_.end(), [](const auto& candidate) {
    return candidate.id == 2 || candidate.operator_key == "NeighborhoodField.CpuPreviewKnn";
  });
  if (stage == pipeline_stages_.end()) {
    PlantReconstructionSubject::StageRecord next;
    next.id = 2;
    next.display_name = "Neighborhood Field";
    next.operator_key = "NeighborhoodField.CpuPreviewKnn";
    next.input_stage_ids = {1};
    next.output_kind = "NeighborhoodField";
    stage = pipeline_stages_.insert(pipeline_stages_.end(), std::move(next));
  }
  stage->status = status;
  stage->settings_hash = "k=" + std::to_string(neighborhood_k) + ";max=" + std::to_string(max_neighborhood_points);
  stage->output_hash = neighborhood_field_.valid ? "points=" + std::to_string(neighborhood_field_.points.size()) : "";
  stage->visible = show_neighborhood_density;
}

void realtime_plant_reconstructor::SerializePlantReconstructionSubject(YAML::Emitter& out,
                                                                       const PlantReconstructionSubject& target) {
  out << YAML::Key << "point_cloud_asset_handle" << YAML::Value << target.point_cloud_asset_handle_.GetValue();
  out << YAML::Key << "point_cloud_asset_type_name" << YAML::Value << target.point_cloud_asset_type_name_;

  out << YAML::Key << "import_metadata" << YAML::Value << YAML::BeginMap;
  WritePath(out, "source_path", target.import_metadata_.source_path);
  WritePath(out, "manifest_path", target.import_metadata_.manifest_path);
  out << YAML::Key << "sample_name" << YAML::Value << target.import_metadata_.sample_name;
  out << YAML::Key << "source_name" << YAML::Value << target.import_metadata_.source_name;
  out << YAML::Key << "source_kind" << YAML::Value << target.import_metadata_.source_kind;
  out << YAML::Key << "role" << YAML::Value << target.import_metadata_.role;
  out << YAML::Key << "stage" << YAML::Value << target.import_metadata_.stage;
  out << YAML::Key << "view_point_size_mm" << YAML::Value << target.import_metadata_.view_point_size_mm;
  out << YAML::EndMap;

  out << YAML::Key << "display" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "show_preview" << YAML::Value << target.show_preview;
  out << YAML::Key << "point_size_mm" << YAML::Value << target.point_size_mm;
  out << YAML::Key << "point_size_multiplier" << YAML::Value << target.point_size_multiplier;
  out << YAML::Key << "point_opacity" << YAML::Value << target.point_opacity;
  out << YAML::Key << "point_color" << YAML::Value << target.point_color;
  out << YAML::Key << "max_preview_points" << YAML::Value << target.max_preview_points;
  out << YAML::Key << "use_ply_colors" << YAML::Value << target.use_ply_colors;
  out << YAML::EndMap;

  out << YAML::Key << "neighborhood" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "show_density" << YAML::Value << target.show_neighborhood_density;
  out << YAML::Key << "k" << YAML::Value << target.neighborhood_k;
  out << YAML::Key << "max_points" << YAML::Value << target.max_neighborhood_points;
  out << YAML::EndMap;

  out << YAML::Key << "stats" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "total_point_count" << YAML::Value << target.total_point_count_;
  out << YAML::Key << "min_bound" << YAML::Value << target.min_bound_;
  out << YAML::Key << "max_bound" << YAML::Value << target.max_bound_;
  out << YAML::EndMap;

  out << YAML::Key << "pipeline_stages" << YAML::Value << YAML::BeginSeq;
  for (const auto& stage : target.pipeline_stages_) {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << stage.id;
    out << YAML::Key << "display_name" << YAML::Value << stage.display_name;
    out << YAML::Key << "operator_key" << YAML::Value << stage.operator_key;
    out << YAML::Key << "input_stage_ids" << YAML::Value << YAML::BeginSeq;
    for (const auto input_stage_id : stage.input_stage_ids) {
      out << input_stage_id;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "output_kind" << YAML::Value << stage.output_kind;
    out << YAML::Key << "status" << YAML::Value << stage.status;
    out << YAML::Key << "settings_hash" << YAML::Value << stage.settings_hash;
    out << YAML::Key << "output_hash" << YAML::Value << stage.output_hash;
    out << YAML::Key << "visible" << YAML::Value << stage.visible;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void realtime_plant_reconstructor::DeserializePlantReconstructionSubject(const YAML::Node& in,
                                                                         PlantReconstructionSubject& target) {
  if (const auto handle = in["point_cloud_asset_handle"]) {
    target.point_cloud_asset_handle_ = Handle(handle.as<uint64_t>());
  }
  target.point_cloud_asset_type_name_ = ReadString(in, "point_cloud_asset_type_name", "PointCloud");

  if (const auto metadata = in["import_metadata"]) {
    target.import_metadata_.source_path = ReadPath(metadata, "source_path");
    target.import_metadata_.manifest_path = ReadPath(metadata, "manifest_path");
    target.import_metadata_.sample_name = ReadString(metadata, "sample_name");
    target.import_metadata_.source_name = ReadString(metadata, "source_name");
    target.import_metadata_.source_kind = ReadString(metadata, "source_kind");
    target.import_metadata_.role = ReadString(metadata, "role");
    target.import_metadata_.stage = ReadString(metadata, "stage");
    target.import_metadata_.view_point_size_mm = ReadFloat(metadata, "view_point_size_mm", 0.0f);
  }

  if (const auto display = in["display"]) {
    target.show_preview = ReadBool(display, "show_preview", target.show_preview);
    target.point_size_mm = ReadFloat(display, "point_size_mm", target.point_size_mm);
    target.point_size_multiplier = ReadFloat(display, "point_size_multiplier", target.point_size_multiplier);
    target.point_opacity = ReadFloat(display, "point_opacity", target.point_opacity);
    if (display["point_color"]) {
      target.point_color = display["point_color"].as<glm::vec3>();
    }
    target.max_preview_points = ReadInt(display, "max_preview_points", target.max_preview_points);
    target.use_ply_colors = ReadBool(display, "use_ply_colors", target.use_ply_colors);
  }

  if (const auto neighborhood = in["neighborhood"]) {
    target.show_neighborhood_density = ReadBool(neighborhood, "show_density", target.show_neighborhood_density);
    target.neighborhood_k = ReadInt(neighborhood, "k", target.neighborhood_k);
    target.max_neighborhood_points = ReadInt(neighborhood, "max_points", target.max_neighborhood_points);
  }

  if (const auto stats = in["stats"]) {
    if (stats["total_point_count"]) {
      target.total_point_count_ = stats["total_point_count"].as<size_t>();
    }
    if (stats["min_bound"]) {
      target.min_bound_ = stats["min_bound"].as<glm::dvec3>();
    }
    if (stats["max_bound"]) {
      target.max_bound_ = stats["max_bound"].as<glm::dvec3>();
    }
  }

  target.pipeline_stages_.clear();
  if (const auto stages = in["pipeline_stages"]) {
    for (const auto& in_stage : stages) {
      PlantReconstructionSubject::StageRecord stage;
      stage.id = ReadInt(in_stage, "id", stage.id);
      stage.display_name = ReadString(in_stage, "display_name", stage.display_name);
      stage.operator_key = ReadString(in_stage, "operator_key", stage.operator_key);
      if (const auto inputs = in_stage["input_stage_ids"]) {
        for (const auto& input : inputs) {
          stage.input_stage_ids.push_back(input.as<int>());
        }
      }
      stage.output_kind = ReadString(in_stage, "output_kind", stage.output_kind);
      stage.status = ReadString(in_stage, "status", stage.status);
      stage.settings_hash = ReadString(in_stage, "settings_hash");
      stage.output_hash = ReadString(in_stage, "output_hash");
      stage.visible = ReadBool(in_stage, "visible", stage.visible);
      target.pipeline_stages_.push_back(stage);
    }
  }
  target.SeedRawPipelineStage();
  target.loaded_point_cloud_.reset();
  target.preview_particles_.reset();
  target.neighborhood_field_ = {};
  target.neighborhood_dirty_ = true;
  target.neighborhood_status_ = "Not built.";
  target.displayed_point_count_ = 0;
  target.status_ = "Point-cloud entity loaded. Preview not loaded.";
  target.MarkNeighborhoodDirty();
  target.MarkPreviewDirty();
}

bool realtime_plant_reconstructor::InspectPlantReconstructionSubject(InspectorContext&,
                                                                     PlantReconstructionSubject& subject) {
  bool changed = false;
  ImGui::TextWrapped("Status: %s", subject.GetStatus().c_str());
  ImGui::Text("Asset handle: %llu", static_cast<unsigned long long>(subject.GetPointCloudAssetHandle().GetValue()));
  ImGui::Text("Points: %zu / %zu", subject.GetDisplayedPointCount(), subject.GetTotalPointCount());
  ImGui::Text("Bounds min: %.3f, %.3f, %.3f", subject.GetMinBound().x, subject.GetMinBound().y,
              subject.GetMinBound().z);
  ImGui::Text("Bounds max: %.3f, %.3f, %.3f", subject.GetMaxBound().x, subject.GetMaxBound().y,
              subject.GetMaxBound().z);
  ImGui::TextWrapped("Source: %s", subject.GetImportMetadata().source_path.string().c_str());

  if (ImGui::Button("Load Preview")) {
    subject.MarkPreviewDirty();
    subject.RebuildPreview();
  }
  ImGui::SameLine();
  if (ImGui::Button("Build Neighborhoods")) {
    subject.show_neighborhood_density = true;
    if (!subject.BuildNeighborhoodField()) {
      subject.show_neighborhood_density = false;
    }
    subject.MarkPreviewDirty();
  }

  changed |= ImGui::Checkbox("Show Preview", &subject.show_preview);
  changed |= ImGui::DragFloat("Point Size (mm)", &subject.point_size_mm, 1.0f, 0.1f, 10000.0f);
  changed |= ImGui::DragFloat("Point Size Multiplier", &subject.point_size_multiplier, 0.01f, 0.0f, 10.0f);
  changed |= ImGui::DragFloat("Point Opacity", &subject.point_opacity, 0.01f, 0.0f, 1.0f);
  changed |= ImGui::ColorEdit3("Point Color", &subject.point_color.x);
  changed |= ImGui::Checkbox("Use PLY Colors", &subject.use_ply_colors);
  changed |= ImGui::DragInt("Max Preview Points", &subject.max_preview_points, 1000, 1000, 5000000);
  if (changed) {
    subject.MarkPreviewDirty();
  }

  if (ImGui::TreeNode("Neighborhood Field")) {
    ImGui::TextWrapped("Status: %s", subject.GetNeighborhoodStatus().c_str());
    bool view_changed = ImGui::Checkbox("Show Density Colors", &subject.show_neighborhood_density);
    bool settings_changed = false;
    settings_changed |= ImGui::DragInt("K", &subject.neighborhood_k, 1, 1, 64);
    settings_changed |= ImGui::DragInt("Max Neighborhood Points", &subject.max_neighborhood_points, 1000, 1000, 500000);
    if (settings_changed) {
      subject.MarkNeighborhoodDirty();
    }
    if (view_changed) {
      subject.MarkPreviewDirty();
    }
    changed |= view_changed || settings_changed;
    ImGui::Text("Points: %zu", subject.GetNeighborhoodPointCount());
    ImGui::Text("Spacing min/mean/max: %.5f / %.5f / %.5f", subject.GetNeighborhoodMinSpacing(),
                subject.GetNeighborhoodMeanSpacing(), subject.GetNeighborhoodMaxSpacing());
    if (!subject.HasNeighborhoodField()) {
      ImGui::TextWrapped("Build neighborhoods to enable density coloring.");
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Pipeline")) {
    for (const auto& stage : subject.GetPipelineStages()) {
      ImGui::Text("%d: %s", stage.id, stage.display_name.c_str());
      ImGui::Text("Operator: %s", stage.operator_key.c_str());
      ImGui::Text("Output: %s", stage.output_kind.c_str());
      ImGui::Text("Status: %s", stage.status.c_str());
    }
    ImGui::TreePop();
  }
  return changed;
}
