#pragma once

#include "EditorLayer.hpp"
#include "PointCloud.hpp"

#include "Bound.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace realtime_plant_reconstructor {
using namespace evo_engine;

class PlantReconstructionSubject final : public IPrivateComponent {
 public:
  struct ImportMetadata {
    std::filesystem::path source_path;
    std::filesystem::path manifest_path;
    std::string sample_name;
    std::string source_name;
    std::string source_kind;
    std::string role;
    std::string stage;
    float view_point_size_mm = 0.0f;
  };

  struct StageRecord {
    int id = 1;
    std::string display_name = "Raw Point Cloud";
    std::string operator_key = "RawPointCloud";
    std::vector<int> input_stage_ids;
    std::string output_kind = "PointCloudXYZ";
    std::string status = "Ready";
    std::string settings_hash;
    std::string output_hash;
    bool visible = true;
  };

  bool show_preview = true;
  float point_size_mm = 100.0f;
  float point_size_multiplier = 0.6f;
  float point_opacity = 1.0f;
  glm::vec3 point_color = glm::vec3(0.82f);
  int max_preview_points = 200000;
  bool use_ply_colors = false;
  bool show_neighborhood_density = false;
  int neighborhood_k = 8;
  int max_neighborhood_points = 50000;

  void OnCreate() override;
  void OnDestroy() override;

  void SetPointCloudAsset(const std::shared_ptr<PointCloud>& point_cloud, const ImportMetadata& metadata);
  void DrawEditorPreview(const std::shared_ptr<EditorLayer>& editor_layer);
  void MarkPreviewDirty();
  void MarkNeighborhoodDirty();
  bool EnsurePointCloudLoaded();
  bool RebuildPreview();
  bool BuildNeighborhoodField();

  [[nodiscard]] Handle GetPointCloudAssetHandle() const;
  [[nodiscard]] const std::string& GetPointCloudAssetTypeName() const;
  [[nodiscard]] const ImportMetadata& GetImportMetadata() const;
  [[nodiscard]] const std::vector<StageRecord>& GetPipelineStages() const;
  [[nodiscard]] bool HasNeighborhoodField() const;
  [[nodiscard]] size_t GetNeighborhoodPointCount() const;
  [[nodiscard]] float GetNeighborhoodMinSpacing() const;
  [[nodiscard]] float GetNeighborhoodMeanSpacing() const;
  [[nodiscard]] float GetNeighborhoodMaxSpacing() const;
  [[nodiscard]] const std::string& GetNeighborhoodStatus() const;
  [[nodiscard]] size_t GetTotalPointCount() const;
  [[nodiscard]] size_t GetDisplayedPointCount() const;
  [[nodiscard]] const glm::dvec3& GetMinBound() const;
  [[nodiscard]] const glm::dvec3& GetMaxBound() const;
  [[nodiscard]] bool GetLocalBound(Bound& bound) const;
  [[nodiscard]] const std::string& GetStatus() const;

  friend void SerializePlantReconstructionSubject(YAML::Emitter& out, const PlantReconstructionSubject& target);
  friend void DeserializePlantReconstructionSubject(const YAML::Node& in, PlantReconstructionSubject& target);

 private:
  struct NeighborhoodPointInfo {
    size_t source_point_index = 0;
    float local_spacing = 0.0f;
    float density = 0.0f;
    float normalized_density = 0.0f;
  };

  struct NeighborhoodField {
    std::vector<NeighborhoodPointInfo> points;
    size_t source_point_count = 0;
    size_t stride = 1;
    int k = 8;
    float min_spacing = 0.0f;
    float mean_spacing = 0.0f;
    float max_spacing = 0.0f;
    bool valid = false;
  };

  Handle point_cloud_asset_handle_ = Handle(0);
  std::string point_cloud_asset_type_name_ = "PointCloud";
  ImportMetadata import_metadata_{};
  std::vector<StageRecord> pipeline_stages_;

  std::shared_ptr<PointCloud> loaded_point_cloud_;
  std::shared_ptr<ParticleInfoList> preview_particles_;
  NeighborhoodField neighborhood_field_;
  bool preview_dirty_ = true;
  bool neighborhood_dirty_ = true;
  size_t total_point_count_ = 0;
  size_t displayed_point_count_ = 0;
  glm::dvec3 min_bound_ = glm::dvec3(0.0);
  glm::dvec3 max_bound_ = glm::dvec3(0.0);
  std::string status_ = "No point cloud.";
  std::string neighborhood_status_ = "Not built.";

  void SeedRawPipelineStage();
  void UpsertNeighborhoodStage(const std::string& status);
};

void SerializePlantReconstructionSubject(YAML::Emitter& out, const PlantReconstructionSubject& target);
void DeserializePlantReconstructionSubject(const YAML::Node& in, PlantReconstructionSubject& target);
bool InspectPlantReconstructionSubject(InspectorContext& context, PlantReconstructionSubject& subject);

}  // namespace realtime_plant_reconstructor
