#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {
class AnimationPlayer : public IPrivateComponent {
 public:
  bool auto_play = true;
  float auto_play_speed = 30.0f;
  void Update() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  // void Save(YAML::Emitter& out) override;
  // void Deserialize(const YAML::Node& in) override;
};
}  // namespace evo_engine
