#pragma once

#include "Json.hpp"
#include "jsSetupConfigParser.hpp"
using namespace evo_engine;
namespace eco_sys_lab {

struct JoeScanProfile {
  float m_encoderValue = 0.f;
  std::vector<glm::vec2> m_points;
  std::vector<int> m_brightness;
};

class JoeScan : public IAsset {
 public:
  std::vector<JoeScanProfile> m_profiles;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
};

struct JoeScanScannerSettings {
  int m_step = 1;
};

class JoeScanScanner : public IPrivateComponent {
  std::shared_ptr<std::mutex> m_scannerMutex;
  bool m_scanEnabled = false;
  JobHandle m_scannerJob{};
  float m_scanTimeStep = 0.5f;

  std::unordered_map<int, JoeScanProfile> m_preservedProfiles;

  std::vector<glm::vec2> m_points;

 public:
  AssetRef m_config;
  AssetRef m_joeScan;
  void StopScanningProcess();
  void StartScanProcess(const JoeScanScannerSettings& settings);
  JoeScanScanner();
  static bool InitializeScanSystem(const std::shared_ptr<Json>& json, jsScanSystem& scanSystem,
                                   std::vector<jsScanHead>& scanHeads);
  static void FreeScanSystem(jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads);
  jsScanSystem m_scanSystem = 0;
  std::vector<jsScanHead> m_scanHeads;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void FixedUpdate() override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace eco_sys_lab
