#pragma once

#include "Json.hpp"
#include "jsSetupConfigParser.hpp"
#include "LogScan.hpp"
using namespace evo_engine;
namespace log_scanning_plugin {

struct JoeScanScannerSettings {
  int step = 1;
};

class JoeScanScanner : public IPrivateComponent {
  std::shared_ptr<std::mutex> scanner_mutex_;
  bool scan_enabled_ = false;
  JobHandle scanner_job_{};
  float scan_time_step_ = 0.5f;

  std::unordered_map<int, LogScanProfile> preserved_profiles_;

  std::vector<glm::vec2> points_;

 public:
  AssetRef config;
  AssetRef log_scan;
  void StopScanningProcess();
  void StartScanProcess(const JoeScanScannerSettings& settings);
  JoeScanScanner();
  static bool InitializeScanSystem(const std::shared_ptr<Json>& json, jsScanSystem& scan_system,
                                   std::vector<jsScanHead>& scan_heads);
  static void FreeScanSystem(jsScanSystem& scan_system, std::vector<jsScanHead>& scan_heads);
  jsScanSystem scan_system = 0;
  std::vector<jsScanHead> scan_heads;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void FixedUpdate() override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace eco_sys_lab_plugin
