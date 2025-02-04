
#pragma once
#include "GraphicsResources.hpp"
#include "ISingleton.hpp"

namespace evo_engine {

/**
 * @struct TextureStorageHandle
 * @brief Represents a handle for texture storage with a unique value.
 */
struct TextureStorageHandle {
  int value = 0;  ///< Unique identifier for the texture storage handle.
};

/**
 * @class Texture2DStorage
 * @brief Responsible for managing 2D textures and their associated GPU resources.
 */
class Texture2DStorage {
  friend class TextureStorage;
  friend class Cubemap;

  /**
   * @brief Stores new texture data to be uploaded to the GPU.
   */
  std::vector<glm::vec4> new_data_;

  /**
   * @brief Stores the resolution of the new texture data.
   */
  glm::uvec2 new_resolution_{};

  /**
   * @brief Uploads texture data to the GPU.
   * @param data The pixel data to upload.
   * @param resolution The resolution of the texture.
   */
  void UploadData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution);

 public:
  bool pending_delete = false;  ///< Indicates whether the storage is pending deletion.

  std::shared_ptr<TextureStorageHandle> handle;  ///< Handle associated with this texture storage.

  std::shared_ptr<Image> image = {};           ///< GPU image resource.
  std::shared_ptr<ImageView> image_view = {};  ///< GPU image view resource.
  std::shared_ptr<Sampler> sampler = {};       ///< GPU sampler resource.

  ImTextureID im_texture_id = VK_NULL_HANDLE;  ///< ImGui texture ID for rendering.

  /**
   * @brief Retrieves the Vulkan image layout of the texture.
   * @return The Vulkan image layout.
   */
  [[nodiscard]] VkImageLayout GetLayout() const;

  /**
   * @brief Retrieves the Vulkan image handle.
   * @return The Vulkan image handle.
   */
  [[nodiscard]] VkImage GetVkImage() const;

  /**
   * @brief Retrieves the Vulkan image view handle.
   * @return The Vulkan image view handle.
   */
  [[nodiscard]] VkImageView GetVkImageView() const;

  /**
   * @brief Retrieves the Vulkan sampler handle.
   * @return The Vulkan sampler handle.
   */
  [[nodiscard]] VkSampler GetVkSampler() const;

  /**
   * @brief Retrieves the GPU image resource.
   * @return A shared pointer to the GPU image.
   */
  [[nodiscard]] std::shared_ptr<Image> GetImage() const;

  /**
   * @brief Initializes the GPU resources for the texture with the given resolution.
   * @param resolution The resolution of the texture.
   */
  void Initialize(const glm::uvec2& resolution);

  /**
   * @brief Sets texture data and uploads it immediately to the GPU.
   * @param data The pixel data to set.
   * @param resolution The resolution of the texture.
   */
  void SetDataImmediately(const std::vector<glm::vec4>& data, const glm::uvec2& resolution);

  /**
   * @brief Sets texture data and queues it for upload during a batch process.
   * @param data The pixel data to set.
   * @param resolution The resolution of the texture.
   */
  void SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution);

  /**
   * @brief Immediately uploads any pending data to the GPU.
   */
  void UploadDataImmediately();

  /**
   * @brief Clears the texture resources and data.
   */
  void Clear();
};

/**
 * @class CubemapStorage
 * @brief Responsible for managing cubemap textures and their associated GPU resources.
 */
class CubemapStorage {
 public:
  bool pending_delete = false;  ///< Indicates whether the storage is pending deletion.

  std::shared_ptr<TextureStorageHandle> handle;  ///< Handle associated with this cubemap storage.

  std::shared_ptr<Image> image = {};           ///< GPU image resource.
  std::shared_ptr<ImageView> image_view = {};  ///< GPU image view resource.
  std::shared_ptr<Sampler> sampler = {};       ///< GPU sampler resource.

  /**
   * @brief Clears the cubemap resources and data.
   */
  void Clear();

  std::vector<std::shared_ptr<ImageView>> face_views;  ///< Image views for each face of the cubemap.
  std::vector<ImTextureID> im_texture_ids;             ///< ImGui texture IDs for each face of the cubemap.

  /**
   * @brief Initializes the GPU resources for the cubemap with the given resolution and mip levels.
   * @param resolution The resolution of the cubemap.
   * @param mip_levels The number of mip levels.
   */
  void Initialize(uint32_t resolution, uint32_t mip_levels);

  /**
   * @brief Retrieves the Vulkan image layout of the cubemap.
   * @return The Vulkan image layout.
   */
  [[nodiscard]] VkImageLayout GetLayout() const;

  /**
   * @brief Retrieves the Vulkan image handle of the cubemap.
   * @return The Vulkan image handle.
   */
  [[nodiscard]] VkImage GetVkImage() const;

  /**
   * @brief Retrieves the Vulkan image view handle of the cubemap.
   * @return The Vulkan image view handle.
   */
  [[nodiscard]] VkImageView GetVkImageView() const;

  /**
   * @brief Retrieves the Vulkan sampler handle of the cubemap.
   * @return The Vulkan sampler handle.
   */
  [[nodiscard]] VkSampler GetVkSampler() const;

  /**
   * @brief Retrieves the GPU image resource of the cubemap.
   * @return A shared pointer to the GPU image.
   */
  [[nodiscard]] std::shared_ptr<Image> GetImage() const;
};

/**
 * @class TextureStorage
 * @brief A singleton class that manages storage and access for all textures and cubemaps in the engine.
 */
class TextureStorage final {
  EVOENGINE_SINGLETON_INSTANCE(TextureStorage)

  /**
   * @brief Stores all 2D texture storages.
   */
  std::vector<Texture2DStorage> texture_2ds_;

  /**
   * @brief Stores all cubemap storages.
   */
  std::vector<CubemapStorage> cubemaps_;

  friend class RenderLayer;
  friend class Platform;
  friend class Resources;

  uint32_t version_ = 0;    ///< Current version of the texture storage.
  bool initialized = true;  ///< Indicates whether the storage has been initialized.

 public:
  /**
   * @brief Retrieves the current version of the texture storage.
   * @return The version as a 32-bit unsigned integer.
   */
  [[nodiscard]] static uint32_t GetVersion();

  /**
   * @brief Synchronizes the device to ensure all texture-related operations are complete.
   */
  static void DeviceSync();

  /**
   * @brief Binds a 2D texture to a specified descriptor set at the given binding.
   * @param descriptor_set The descriptor set to bind the texture to.
   * @param binding The binding index within the descriptor set.
   */
  static void BindTexture2DToDescriptorSet(const std::shared_ptr<DescriptorSet>& descriptor_set, uint32_t binding);

  /**
   * @brief Binds a cubemap to a specified descriptor set at the given binding.
   * @param descriptor_set The descriptor set to bind the cubemap to.
   * @param binding The binding index within the descriptor set.
   */
  static void BindCubemapToDescriptorSet(const std::shared_ptr<DescriptorSet>& descriptor_set, uint32_t binding);

  /**
   * @brief Provides a constant reference to a Texture2DStorage object using a handle.
   * @param handle The handle identifying the texture storage.
   * @return A constant reference to the Texture2DStorage object.
   */
  static const Texture2DStorage& PeekTexture2DStorage(const std::shared_ptr<TextureStorageHandle>& handle);

  /**
   * @brief Provides a mutable reference to a Texture2DStorage object using a handle.
   * @param handle The handle identifying the texture storage.
   * @return A mutable reference to the Texture2DStorage object.
   */
  static Texture2DStorage& RefTexture2DStorage(const std::shared_ptr<TextureStorageHandle>& handle);

  /**
   * @brief Provides a constant reference to a CubemapStorage object using a handle.
   * @param handle The handle identifying the cubemap storage.
   * @return A constant reference to the CubemapStorage object.
   */
  static const CubemapStorage& PeekCubemapStorage(const std::shared_ptr<TextureStorageHandle>& handle);

  /**
   * @brief Provides a mutable reference to a CubemapStorage object using a handle.
   * @param handle The handle identifying the cubemap storage.
   * @return A mutable reference to the CubemapStorage object.
   */
  static CubemapStorage& RefCubemapStorage(const std::shared_ptr<TextureStorageHandle>& handle);

  /**
   * @brief Unregisters a 2D texture from the storage using a handle.
   * @param handle The handle identifying the texture storage.
   */
  static void UnRegisterTexture2D(const std::shared_ptr<TextureStorageHandle>& handle);

  /**
   * @brief Unregisters a cubemap from the storage using a handle.
   * @param handle The handle identifying the cubemap storage.
   */
  static void UnRegisterCubemap(const std::shared_ptr<TextureStorageHandle>& handle);

  /**
   * @brief Registers a new 2D texture in the storage and returns its handle.
   * @return A shared pointer to the TextureStorageHandle of the newly registered texture.
   */
  static std::shared_ptr<TextureStorageHandle> RegisterTexture2D();

  /**
   * @brief Registers a new cubemap in the storage and returns its handle.
   * @return A shared pointer to the TextureStorageHandle of the newly registered cubemap.
   */
  static std::shared_ptr<TextureStorageHandle> RegisterCubemap();

  /**
   * @brief Initializes the texture storage system.
   */
  static void Initialize();

  /**
   * @brief Cleans up and releases resources used by the texture storage system.
   */
  static void OnDestroy();
};

}  // namespace evo_engine
