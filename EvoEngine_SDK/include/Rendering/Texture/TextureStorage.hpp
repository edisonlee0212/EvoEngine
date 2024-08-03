#pragma once
#include "GraphicsResources.hpp"
#include "ISingleton.hpp"

namespace evo_engine {
struct TextureStorageHandle {
  int value = 0;
};

class Texture2DStorage {
  friend class TextureStorage;
  friend class Cubemap;
  std::vector<glm::vec4> new_data_;
  glm::uvec2 new_resolution_{};
  void UploadData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution);

 public:
  bool pending_delete = false;

  std::shared_ptr<TextureStorageHandle> handle;

  std::shared_ptr<Image> image = {};
  std::shared_ptr<ImageView> image_view = {};
  std::shared_ptr<Sampler> sampler = {};
  ImTextureID im_texture_id = VK_NULL_HANDLE;

  [[nodiscard]] VkImageLayout GetLayout() const;
  [[nodiscard]] VkImage GetVkImage() const;
  [[nodiscard]] VkImageView GetVkImageView() const;
  [[nodiscard]] VkSampler GetVkSampler() const;
  [[nodiscard]] std::shared_ptr<Image> GetImage() const;
  void Initialize(const glm::uvec2& resolution);
  void SetDataImmediately(const std::vector<glm::vec4>& data, const glm::uvec2& resolution);
  void SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution);
  void UploadDataImmediately();
  void Clear();
};
class CubemapStorage {
 public:
  bool pending_delete = false;

  std::shared_ptr<TextureStorageHandle> handle;

  std::shared_ptr<Image> image = {};
  std::shared_ptr<ImageView> image_view = {};
  std::shared_ptr<Sampler> sampler = {};
  void Clear();
  std::vector<std::shared_ptr<ImageView>> face_views;
  std::vector<ImTextureID> im_texture_ids;
  void Initialize(uint32_t resolution, uint32_t mip_levels);
  [[nodiscard]] VkImageLayout GetLayout() const;
  [[nodiscard]] VkImage GetVkImage() const;
  [[nodiscard]] VkImageView GetVkImageView() const;
  [[nodiscard]] VkSampler GetVkSampler() const;
  [[nodiscard]] std::shared_ptr<Image> GetImage() const;
};
class TextureStorage final {
  EVOENGINE_SINGLETON_INSTANCE(TextureStorage)
  std::vector<Texture2DStorage> texture_2ds_;
  std::vector<CubemapStorage> cubemaps_;

  friend class RenderLayer;
  friend class Graphics;
  friend class Resources;
  static void DeviceSync();

 public:
  static const Texture2DStorage& PeekTexture2DStorage(const std::shared_ptr<TextureStorageHandle>& handle);
  static Texture2DStorage& RefTexture2DStorage(const std::shared_ptr<TextureStorageHandle>& handle);

  static const CubemapStorage& PeekCubemapStorage(const std::shared_ptr<TextureStorageHandle>& handle);
  static CubemapStorage& RefCubemapStorage(const std::shared_ptr<TextureStorageHandle>& handle);

  static void UnRegisterTexture2D(const std::shared_ptr<TextureStorageHandle>& handle);
  static void UnRegisterCubemap(const std::shared_ptr<TextureStorageHandle>& handle);
  static std::shared_ptr<TextureStorageHandle> RegisterTexture2D();
  static std::shared_ptr<TextureStorageHandle> RegisterCubemap();
  static void Initialize();
};
}  // namespace evo_engine
