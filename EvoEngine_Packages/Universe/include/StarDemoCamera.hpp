#pragma once

#include "AssetRef.hpp"

namespace evo_engine {
class Camera;
class PostProcessingStack;
}  // namespace evo_engine

namespace universe_package {
struct StarDemoCameraOverride {
  StarDemoCameraOverride() = default;
  ~StarDemoCameraOverride();
  StarDemoCameraOverride(const StarDemoCameraOverride&) = delete;
  StarDemoCameraOverride& operator=(const StarDemoCameraOverride&) = delete;

  void Apply(const std::shared_ptr<evo_engine::Camera>& camera);
  void Restore();

 private:
  std::weak_ptr<evo_engine::Camera> camera_;
  evo_engine::AssetRef original_;
  std::shared_ptr<evo_engine::PostProcessingStack> override_;
};
}  // namespace universe_package
