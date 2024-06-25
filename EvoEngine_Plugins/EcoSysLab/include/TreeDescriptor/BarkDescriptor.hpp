#pragma once

using namespace evo_engine;
namespace eco_sys_lab {
class BarkDescriptor : public IAsset {
 public:
  float bark_x_frequency = 3.0f;
  float bark_y_frequency = 5.0f;
  float bark_depth = 0.1f;

  float base_frequency = 1.0f;
  float base_max_distance = 1.f;
  float base_distance_decrease_factor = 2.f;
  float base_depth = .1f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  float GetValue(float xFactor, float distanceToRoot);

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace eco_sys_lab