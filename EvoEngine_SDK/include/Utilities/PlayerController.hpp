#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {
class PlayerController : public IPrivateComponent {
  float last_x_ = 0, last_y_ = 0, last_scroll_y_ = 0;
  bool start_mouse_ = false;
  float scene_camera_yaw_angle_ = -89;
  float scene_camera_pitch_angle_ = 0;

 public:
  float velocity = 20.0f;
  float sensitivity = 0.1f;
  void OnCreate() override;
  void LateUpdate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};
}  // namespace evo_engine
