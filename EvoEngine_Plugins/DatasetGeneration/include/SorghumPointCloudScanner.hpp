#pragma once

#include "PointCloudScannerUtils.hpp"
using namespace evo_engine;
namespace dataset_generation_plugin {
struct SorghumPointCloudPointSettings {
  float variance = 0.015f;
  float ball_rand_radius = 0.01f;

  bool type_index = true;
  bool instance_index = true;
  bool leaf_index = true;

  float bounding_box_limit = 2.f;

  bool OnInspect();
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
};

class SorghumPointCloudGridCaptureSettings : public PointCloudCaptureSettings {
 public:
  float bounding_box_size = 3.;

  glm::ivec2 grid_size = {5, 5};
  float grid_distance = 1.25f;
  float step = 0.01f;
  int drone_sample = 512;
  float drone_height = 2.5f;
  bool OnInspect() override;
  void GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) override;
  bool SampleFilter(const PointCloudSample& sample) override;
};

class SorghumGantryCaptureSettings : public PointCloudCaptureSettings {
 public:
  float bounding_box_size = 10.;

  glm::ivec2 grid_size = {5, 5};
  glm::vec2 grid_distance = {0.75f, 0.75f};
  glm::vec2 step = {0.0075f, 0.0075f};
  float sample_height = 2.5f;

  std::vector<float> scanner_angles = {30.f};

  bool OnInspect() override;
  void GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) override;
  bool SampleFilter(const PointCloudSample& sample) override;
};

class SorghumPointCloudScanner : public IPrivateComponent {
 public:
  glm::vec3 left_random_offset = glm::vec3(0.0f);
  glm::vec3 right_random_offset = glm::vec3(0.0f);
  SorghumPointCloudPointSettings sorghum_point_cloud_point_settings{};

  void Scan(const std::shared_ptr<PointCloudCaptureSettings>& capture_settings, std::vector<glm::vec3>& points,
               std::vector<int>& leaf_indices, std::vector<int>& instance_indices,
               std::vector<int>& type_indices) const;

  void SavePointCloud(const std::filesystem::path& save_path, const std::vector<glm::vec3>& points,
                      const std::vector<int>& leaf_indices, const std::vector<int>& instance_indices,
                      const std::vector<int>& type_indices) const;

  static void WriteSplineInfo(const std::filesystem::path& save_path,
                              const std::shared_ptr<PointCloudCaptureSettings>& capture_settings);
  void Capture(const std::filesystem::path& save_path,
               const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void OnDestroy() override;

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace dataset_generation_plugin