#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include "StarFollow.hpp"

namespace universe_package {
struct UniverseViewportInput {
  std::shared_ptr<evo_engine::Camera> camera;
  StarPickRequest request;
  std::string camera_name = "Main camera";
  uint64_t click_sequence = 0;
  uint64_t follow_toggle_sequence = 0;
};

class UniverseViewHost {
 public:
  virtual ~UniverseViewHost() = default;
  virtual UniverseViewportInput GetViewportInput(const std::shared_ptr<evo_engine::Scene>& scene) const = 0;
  virtual std::shared_ptr<evo_engine::Camera> GetCamera() const = 0;
  virtual StarFollowCameraPose GetPose() const = 0;
  virtual void Move(const StarFollowCameraPose& pose) = 0;
  virtual void Rebase(const glm::dmat4& transform) = 0;
  virtual void SetDemoCamera(bool enabled) = 0;
};
}  // namespace universe_package
