#pragma once

#include "EditorLayer.hpp"
#include "PlantReconstructionSubject.hpp"
#include "PointCloud.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace realtime_plant_reconstructor {
using namespace evo_engine;

class PlantReconstructionLayer final : public ILayer {
 public:
  void OnCreate() override;
  void Update() override;
  [[nodiscard]] bool SupportsProjectStateSerialization() const override;
  void SerializeProjectState(YAML::Emitter& out) const override;
  void DeserializeProjectState(const YAML::Node& in) override;
  [[nodiscard]] bool SupportsLayerAutomationMode(const std::string& mode) const override;
  int RunLayerAutomation(const YAML::Node& config) override;
  [[nodiscard]] bool TryGetEntityEditorBound(const std::shared_ptr<Scene>& scene, const Entity& entity,
                                             Bound& bound) const override;
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  struct ImportResult {
    bool success = false;
    Entity entity;
    std::filesystem::path source_path;
    std::string sample_name;
    std::string message;
    size_t total_point_count = 0;
    size_t displayed_point_count = 0;
  };

  void RefreshInputFolders();
  [[nodiscard]] size_t GetCheckedInputFolderCount() const;
  [[nodiscard]] size_t GetCheckedInputCandidateCount() const;
  [[nodiscard]] const std::string& GetStatus() const;
  [[nodiscard]] ImportResult ImportRandomCheckedPointCloud(uint64_t seed, bool save_asset);

 private:
  struct PointCloudCandidate {
    std::filesystem::path path;
    std::filesystem::path input_folder_path;
    std::filesystem::path manifest_path;
    std::string sample_name;
    std::string source_name;
    std::string source_kind;
    std::string role;
    std::string stage;
    int priority = 0;
    float view_point_size_mm = 0.0f;
    std::filesystem::file_time_type modified_time{};
  };

  struct InputFolderInfo {
    std::filesystem::path path;
    std::string name;
    bool checked = true;
    size_t candidate_count = 0;
  };

  std::array<char, 512> scale_root_buffer_{};
  std::vector<PointCloudCandidate> candidates_;
  std::vector<InputFolderInfo> input_folders_;
  std::set<std::string> saved_checked_input_folders_;
  int selected_candidate_index_ = -1;
  bool input_folder_selection_loaded_ = false;

  float point_size_mm_ = 100.0f;
  float point_size_multiplier_ = 0.6f;
  float point_opacity_ = 1.0f;
  float point_color_[3] = {0.82f, 0.82f, 0.82f};
  int max_preview_points_ = 200000;
  bool use_ply_colors_ = false;
  bool show_preview_ = true;
  int neighborhood_k_ = 8;
  int max_neighborhood_points_ = 50000;
  bool layout_seeded_ = false;

  std::string status_ = "Ready.";

  [[nodiscard]] std::filesystem::path ScaleRoot() const;
  void SetScaleRoot(const std::filesystem::path& path);
  void ImportGrid();
  void DiscoverCandidates();
  void AddManifestCandidates(const std::filesystem::path& run_manifest_path);
  void AddInputCandidates(const std::filesystem::path& input_root);
  void AddInputFolderCandidates(const std::filesystem::path& input_folder_path,
                                std::vector<PointCloudCandidate>& candidates) const;
  [[nodiscard]] bool HasInputFolderTransform(const std::filesystem::path& input_folder_path) const;
  [[nodiscard]] std::vector<PointCloudCandidate> BuildCheckedInputCandidates() const;
  [[nodiscard]] ImportResult ImportCandidate(const PointCloudCandidate& candidate, bool save_asset);
  void DrawSubjectPreviews(const std::shared_ptr<EditorLayer>& editor_layer) const;
  void ApplyDefaultsToSubject(PlantReconstructionSubject& subject) const;
  void StoreCheckedInputFolderSelection();
  [[nodiscard]] std::string NormalizeInputFolderPath(const std::filesystem::path& path) const;
  [[nodiscard]] std::string CandidateEntityName(const PointCloudCandidate& candidate) const;
  [[nodiscard]] std::string CandidateLabel(const PointCloudCandidate& candidate) const;
};

}  // namespace realtime_plant_reconstructor
