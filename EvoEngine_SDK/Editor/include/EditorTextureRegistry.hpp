#pragma once

#include <imgui.h>
#include <memory>
#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"
#include "GraphicsResources.hpp"

namespace evo_engine {
class Texture2D;
class RenderTexture;

class EVOENGINE_EDITOR_API EditorTextureRegistry {
 public:
  // IDs remain valid for the GPU work submitted in the current frame.
  static ImTextureID GetTextureId(const Texture2D& texture);
  static ImTextureID GetTextureId(const SampledImageResources& resources);
  static ImTextureID GetTextureId(const std::shared_ptr<Image>& image, const std::shared_ptr<ImageView>& view,
                                  const std::shared_ptr<Sampler>& sampler);
  static ImTextureID GetColorTextureId(RenderTexture& texture, uint32_t mip = 0);
  static ImTextureID GetDepthTextureId(RenderTexture& texture, uint32_t mip = 0);
  static void CollectGarbage();
};
}  // namespace evo_engine
