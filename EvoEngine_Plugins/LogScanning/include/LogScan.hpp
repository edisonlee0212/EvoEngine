#pragma once
using namespace evo_engine;
namespace log_scanning_plugin {
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

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

};
}