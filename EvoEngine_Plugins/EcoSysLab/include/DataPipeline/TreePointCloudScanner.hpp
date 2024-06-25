#pragma once
#include "PointCloudScannerUtils.hpp"
#include "Tree.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
struct TreePointCloudPointSettings {
  float m_variance = 0.015f;
  float m_ballRandRadius = 0.005f;
  bool m_typeIndex = true;
  bool m_instanceIndex = true;
  bool m_branchIndex = false;
  bool m_internodeIndex = false;
  bool m_lineIndex = false;
  bool m_treePartIndex = false;
  bool m_treePartTypeIndex = false;

  float m_boundingBoxLimit = 1.f;

  void OnInspect();

  void Save(const std::string& name, YAML::Emitter& out) const;

  void Load(const std::string& name, const YAML::Node& in);
};

class TreePointCloudCircularCaptureSettings : public PointCloudCaptureSettings {
 public:
  int m_pitchAngleStart = -20;
  int m_pitchAngleStep = 10;
  int m_pitchAngleEnd = 60;
  int m_turnAngleStart = 0;
  int m_turnAngleStep = 10;
  int m_turnAngleEnd = 360;
  float m_distance = 5.0f;
  float m_height = 1.5f;
  float m_fov = 60;
  glm::vec2 m_focusPoint = {0, 0};
  int m_resolution = 128;
  float m_cameraDepthMax = 10;

  bool OnInspect() override;

  void Save(const std::string& name, YAML::Emitter& out) const override;

  void Load(const std::string& name, const YAML::Node& in) override;

  GlobalTransform GetTransform(const glm::vec2& focusPoint, float turnAngle, float pitchAngle) const;
  void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) override;
};

class TreePointCloudGridCaptureSettings : public PointCloudCaptureSettings {
 public:
  float m_boundingBoxSize = 0.f;

  glm::ivec2 m_gridSize = {5, 5};
  glm::vec2 m_gridDistance = {1.25f, 1.25f};
  float m_step = 0.01f;
  int m_backpackSample = 512;
  float m_backpackHeight = 1.0f;
  int m_droneSample = 128;
  float m_droneHeight = 5.0f;
  bool OnInspect() override;
  void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) override;
  bool SampleFilter(const PointCloudSample& sample) override;
};

class TreePointCloudScanner : public IPrivateComponent {
 public:
  TreePointCloudPointSettings m_pointSettings;
  void Capture(const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::filesystem::path& savePath,
               const std::shared_ptr<PointCloudCaptureSettings>& captureSettings) const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  void OnDestroy() override;

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace eco_sys_lab