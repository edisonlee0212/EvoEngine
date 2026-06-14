#pragma once
#include "PointCloudScannerUtils.hpp"
#include "Tree.hpp"

namespace dataset_generation_package {
using namespace evo_engine;
using namespace eco_sys_lab_package;
struct TreePointCloudPointSettings {
  float variance = 0.015f;
  float ball_rand_radius = 0.005f;
  bool type_index = true;
  bool instance_index = true;
  bool branch_index = false;
  bool internode_index = false;
  bool line_index = false;
  bool tree_part_index = false;
  bool tree_part_type_index = false;

  float bounding_box_limit = 1.f;

  void DrawGui();

  void Save(const std::string& name, YAML::Emitter& out) const;

  void Load(const std::string& name, const YAML::Node& in);
};

class TreePointCloudCircularCaptureSettings : public PointCloudCaptureSettings {
 public:
  int pitch_angle_start = -20;
  int pitch_angle_step = 10;
  int pitch_angle_end = 60;
  int turn_angle_start = 0;
  int turn_angle_step = 10;
  int turn_angle_end = 360;
  float distance_from_trees = 5.0f;
  float capture_height = 1.5f;
  float camera_fov = 60;
  glm::vec2 camera_focus_point = {0, 0};
  int scan_resolution = 128;
  float max_capture_depth = 10;

  bool DrawGui() override;

  void Save(const std::string& name, YAML::Emitter& out) const override;

  void Load(const std::string& name, const YAML::Node& in) override;

  GlobalTransform GetTransform(const glm::vec2& focus_point, float turn_angle, float pitch_angle) const;
  void GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) override;
};

class TreePointCloudGridCaptureSettings : public PointCloudCaptureSettings {
 public:
  float bounding_box_size = 0.f;

  glm::ivec2 grid_size = {5, 5};
  glm::vec2 grid_distance = {1.25f, 1.25f};
  float step = 0.01f;
  int ground_sample_size = 512;
  float ground_sample_height = 1.0f;
  int drone_sample_size = 128;
  float drone_sample_height = 5.0f;
  bool DrawGui() override;
  void GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) override;
  bool SampleFilter(const PointCloudSample& sample) override;
};

class TreePointCloudScanner : public IPrivateComponent {
 public:
  TreePointCloudPointSettings point_settings;
  void Capture(const TreeMeshGeneratorSettings& mesh_generator_settings, const std::filesystem::path& save_path,
               const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const;

  void OnDestroy() override;
};
}  // namespace dataset_generation_package
