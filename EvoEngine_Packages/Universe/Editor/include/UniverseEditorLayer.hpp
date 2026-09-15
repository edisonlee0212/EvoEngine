#pragma once
#include "ILayer.hpp"
#include "StarDemoCamera.hpp"
#include "UniverseViewHost.hpp"
namespace universe_package {
class UniverseEditorLayer final : public evo_engine::ILayer, public UniverseViewHost {
  StarDemoCameraOverride demo_camera_;
  void OnCreate() override;
  void OnDestroy() override;

 public:
  UniverseViewportInput GetViewportInput(const std::shared_ptr<evo_engine::Scene>& scene) const override;
  std::shared_ptr<evo_engine::Camera> GetCamera() const override;
  StarFollowCameraPose GetPose() const override;
  void Move(const StarFollowCameraPose& pose) override;
  void Rebase(const glm::dmat4& transform) override;
  void SetDemoCamera(bool enabled) override;
};
}  // namespace universe_package
