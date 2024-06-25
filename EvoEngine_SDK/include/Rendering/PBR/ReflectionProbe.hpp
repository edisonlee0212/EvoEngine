#pragma once
#include "Cubemap.hpp"

namespace evo_engine {
class ReflectionProbe : public IAsset {
  std::shared_ptr<Cubemap> cubemap_;
  friend class RenderLayer;
  friend class Camera;
  std::vector<std::vector<std::shared_ptr<ImageView>>> mip_map_views_;

 public:
  void Initialize(uint32_t resolution = 512);
  [[nodiscard]] std::shared_ptr<Cubemap> GetCubemap() const;
  void ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine
