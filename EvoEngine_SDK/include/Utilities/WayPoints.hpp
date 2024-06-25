#pragma once
#include "IPrivateComponent.hpp"
namespace evo_engine {
class WayPoints : public IPrivateComponent {
 public:
  enum class Mode { FixedTime, FixedVelocity } mode = Mode::FixedTime;

  float speed = 1.0f;
  std::vector<EntityRef> entities;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
};
}  // namespace evo_engine