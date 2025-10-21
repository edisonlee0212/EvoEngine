#pragma once

#include "Json.hpp"
#include "Prefab.hpp"

namespace log_scanning_plugin {
using namespace evo_engine;
struct JoeScanConfig {
  struct ScanHead {
    int id = -1;
    glm::vec2 shift = {0.0f, 0.0f};
    float roll = {0.0f};
  };
  std::vector<ScanHead> scan_heads;
  void Import(const std::shared_ptr<Json>& json);
  void PlacePrefabs(const std::shared_ptr<Prefab>& prefab) const;
};

struct LogScanProfile {
  float encoder_value = 0.f;
  std::vector<glm::vec2> points;
  std::vector<int> brightness;
};

class LogScan : public IAsset {
 public:
  std::vector<LogScanProfile> profiles;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Regularize();
  void Recenter();
};
}  // namespace log_scanning_plugin
