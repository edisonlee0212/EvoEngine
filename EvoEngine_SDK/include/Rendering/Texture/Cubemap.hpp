#pragma once
#include "Graphics.hpp"
#include "GraphicsResources.hpp"
#include "IAsset.hpp"
#include "Texture2D.hpp"

namespace evo_engine {
class CubemapStorage;
struct TextureStorageHandle;

class Cubemap final : public IAsset {
  friend class RenderLayer;

  friend class LightProbe;
  friend class ReflectionProbe;
  friend class TextureStorage;
  std::shared_ptr<TextureStorageHandle> texture_storage_handle_;

 public:
  struct EquirectangularToCubemapConstant {
    glm::mat4 projection_view = {};
    float preset_value = 0;
  };
  Cubemap();
  const CubemapStorage& PeekStorage() const;
  CubemapStorage& RefStorage() const;
  ~Cubemap() override;
  void Initialize(uint32_t resolution, uint32_t mip_levels = 1) const;
  [[nodiscard]] uint32_t GetTextureStorageIndex() const;
  void ConvertFromEquirectangularTexture(const std::shared_ptr<Texture2D>& target_texture) const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  [[nodiscard]] const std::shared_ptr<Image>& GetImage() const;
  [[nodiscard]] const std::shared_ptr<ImageView>& GetImageView() const;
  [[nodiscard]] const std::shared_ptr<Sampler>& GetSampler() const;
  [[nodiscard]] const std::vector<std::shared_ptr<ImageView>>& GetFaceViews() const;
};
}  // namespace evo_engine
