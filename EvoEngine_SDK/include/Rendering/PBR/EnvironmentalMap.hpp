#pragma once
#include "AssetRef.hpp"
#include "LightProbe.hpp"
#include "ReflectionProbe.hpp"
#include "RenderTexture.hpp"

namespace evo_engine {
class EnvironmentalMap final : public IAsset {
  friend class Graphics;
  friend class Camera;
  friend class Environment;
  friend class RenderLayer;
  friend class Resources;

 public:
  AssetRef light_probe;
  AssetRef reflection_probe;
  void ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap);
  void ConstructFromTexture2D(const std::shared_ptr<Texture2D>& target_texture_2d);
  void ConstructFromRenderTexture(const std::shared_ptr<RenderTexture>& target_render_texture);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine
