#pragma once
#include "Cubemap.hpp"

namespace evo_engine {
class LightProbe final : public IAsset {
  std::shared_ptr<Cubemap> cubemap_;
  friend class RenderLayer;
  friend class Camera;

 public:
  void Initialize(uint32_t resolution = 32);
  void ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap);
  [[nodiscard]] std::shared_ptr<Cubemap> GetCubemap() const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine