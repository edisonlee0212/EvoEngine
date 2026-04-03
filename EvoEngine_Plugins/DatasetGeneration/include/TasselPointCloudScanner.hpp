#pragma once

#include "PointCloudScannerUtils.hpp"

namespace dataset_generation_plugin {
using namespace evo_engine;

struct TasselPointCloudPointSettings {
  float variance = 0.015f;
  float ball_rand_radius = 0.01f;

  bool type_index = true;       // 0 = stem, 1 = spikelet
  bool instance_index = true;

  float bounding_box_limit = 1.f;

  bool OnInspect();
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
};

class TasselPointCloudGridCaptureSettings : public PointCloudCaptureSettings {
 public:
  float bounding_box_size = 1.f;

  glm::ivec2 grid_size = {1, 1};
  float grid_distance = 0.5f;
  float step = 0.005f;
  int samples_per_step = 256;
  float sample_height = 1.0f;

  bool OnInspect() override;
  void GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) override;
  bool SampleFilter(const PointCloudSample& sample) override;
};

class TasselPointCloudScanner : public IPrivateComponent {
 public:
  TasselPointCloudPointSettings tassel_point_cloud_point_settings{};

  void Scan(const std::shared_ptr<PointCloudCaptureSettings>& capture_settings, std::vector<glm::vec3>& points,
            std::vector<int>& instance_indices, std::vector<int>& type_indices) const;

  void SavePointCloud(const std::filesystem::path& save_path, const std::vector<glm::vec3>& points,
                      const std::vector<int>& instance_indices, const std::vector<int>& type_indices) const;

  void Capture(const std::filesystem::path& save_path,
               const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnDestroy() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace dataset_generation_plugin
