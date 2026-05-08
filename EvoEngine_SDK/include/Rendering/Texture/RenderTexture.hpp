
#pragma once
#include "GraphicsPipeline.hpp"
#include "GraphicsPipelineStates.hpp"
#include "GraphicsResources.hpp"

namespace evo_engine {

/**
 * @brief Structure for configuring the creation of a RenderTexture.
 */
struct RenderTextureCreateInfo {
  VkExtent3D extent = {1, 1, 1};                           /**< Extent of the texture (width, height, depth). */
  VkImageViewType image_view_type = VK_IMAGE_VIEW_TYPE_2D; /**< Type of the image view. */
  bool color = true; /**< Flag to indicate if the texture has a color attachment. */
  bool depth = true; /**< Flag to indicate if the texture has a depth attachment. */
};

/**
 * @brief A class representing a render texture with color and depth attachments.
 */
class RenderTexture {
  friend class Platform;
  friend class RenderLayer;
  friend class WindowLayer;

  std::shared_ptr<Image> color_image_ = {};                        /**< Pointer to the color image resource. */
  std::vector<std::shared_ptr<ImageView>> color_image_views_ = {}; /**< Views of the color image. */

  std::shared_ptr<Image> depth_image_ = {};                        /**< Pointer to the depth image resource. */
  std::vector<std::shared_ptr<ImageView>> depth_image_views_ = {}; /**< Views of the depth image. */

  VkExtent3D extent_;                               /**< Extent of the texture (width, height, depth). */
  VkImageViewType image_view_type_;                 /**< Type of the image view. */
  std::shared_ptr<Sampler> color_sampler_ = {};     /**< Sampler for the color texture. */
  std::shared_ptr<Sampler> depth_sampler_ = {};     /**< Sampler for the depth texture. */
  std::vector<ImTextureID> color_im_texture_ids_{}; /**< Color texture IDs for ImGui. */
  std::vector<ImTextureID> depth_im_texture_ids_{}; /**< Depth texture IDs for ImGui. */

  bool color_ = true; /**< Indicates if the texture has a color attachment. */
  bool depth_ = true; /**< Indicates if the texture has a depth attachment. */

  /**
   * @brief Initializes the render texture.
   * @param render_texture_create_info Information for creating the render texture.
   * @param mip_levels Number of mip levels.
   */
  void Initialize(const RenderTextureCreateInfo& render_texture_create_info, uint32_t mip_levels = 1);

  std::shared_ptr<DescriptorSet> depth_present_descriptor_set_; /**< Descriptor set for presenting the depth texture. */
  std::shared_ptr<DescriptorSet> color_present_descriptor_set_; /**< Descriptor set for presenting the color texture. */
  std::shared_ptr<DescriptorSet> storage_descriptor_set_;       /**< Descriptor set for storage. */

 public:
  /**
   * @brief Clears the render texture.
   * @param vk_command_buffer Vulkan command buffer used for clearing.
   */
  void Clear(VkCommandBuffer vk_command_buffer) const;

  /**
   * @brief Constructs a RenderTexture with the specified creation information.
   * @param render_texture_create_info Information for creating the render texture.
   */
  explicit RenderTexture(const RenderTextureCreateInfo& render_texture_create_info);

  /**
   * @brief Resizes the render texture.
   * @param extent New extent for the texture.
   * @param mip_level Number of mip levels.
   */
  void Resize(VkExtent3D extent, uint32_t mip_level = 1);

  /**
   * @brief Appends color attachment information for rendering.
   * @param attachment_infos Vector to append the attachment information to.
   * @param load_op Vulkan load operation for the color attachment.
   * @param store_op Vulkan store operation for the color attachment.
   * @param mip_level Mip level to use.
   */
  void AppendColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos, VkAttachmentLoadOp load_op,
                                  VkAttachmentStoreOp store_op, uint32_t mip_level = 0) const;

  /**
   * @brief Retrieves depth attachment information for rendering.
   * @param load_op Vulkan load operation for the depth attachment.
   * @param store_op Vulkan store operation for the depth attachment.
   * @param mip_level Mip level to use.
   * @return Depth attachment information.
   */
  [[nodiscard]] VkRenderingAttachmentInfo GetDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                 VkAttachmentStoreOp store_op,
                                                                 uint32_t mip_level = 0) const;

  /**
   * @brief Gets the extent of the texture.
   * @return Current extent of the render texture.
   */
  [[nodiscard]] VkExtent3D GetExtent() const;

  /**
   * @brief Gets the image view type of the texture.
   * @return Image view type of the render texture.
   */
  [[nodiscard]] VkImageViewType GetImageViewType() const;

  /**
   * @brief Gets the number of mip levels in the texture.
   * @return Number of mip levels.
   */
  [[nodiscard]] uint32_t GetMipLevels() const;

  /**
   * @brief Gets the color sampler of the texture.
   * @return Shared pointer to the color sampler.
   */
  [[nodiscard]] const std::shared_ptr<Sampler>& GetColorSampler() const;

  /**
   * @brief Gets the depth sampler of the texture.
   * @return Shared pointer to the depth sampler.
   */
  [[nodiscard]] const std::shared_ptr<Sampler>& GetDepthSampler() const;

  /**
   * @brief Gets the color image of the texture.
   * @return Shared pointer to the color image.
   */
  [[nodiscard]] const std::shared_ptr<Image>& GetColorImage();

  /**
   * @brief Gets the depth image of the texture.
   * @return Shared pointer to the depth image.
   */
  [[nodiscard]] const std::shared_ptr<Image>& GetDepthImage();

  /**
   * @brief Gets the color image view for a specific mip level.
   * @param mip_index Index of the mip level.
   * @return Shared pointer to the color image view for the given mip level.
   */
  [[nodiscard]] const std::shared_ptr<ImageView>& GetColorImageView(uint32_t mip_index = 0);

  /**
   * @brief Gets the depth image view for a specific mip level.
   * @param mip_index Index of the mip level.
   * @return Shared pointer to the depth image view for the given mip level.
   */
  [[nodiscard]] const std::shared_ptr<ImageView>& GetDepthImageView(uint32_t mip_index = 0);

  /**
   * @brief Renders the provided function using the render texture.
   * @param vk_command_buffer Vulkan command buffer used for rendering.
   * @param load_op Vulkan load operation for attachments.
   * @param store_op Vulkan store operation for attachments.
   * @param func Function to be rendered.
   * @param mip_level Mip level to render to.
   */
  void Render(VkCommandBuffer vk_command_buffer, VkAttachmentLoadOp load_op, VkAttachmentStoreOp store_op,
              const std::function<void()>& func, uint32_t mip_level = 0) const;

  /**
   * @brief Gets the ImGui texture ID for the color attachment at a specific mip level.
   * @param mip_index Index of the mip level.
   * @return ImGui texture ID for the color attachment.
   */
  [[nodiscard]] ImTextureID GetColorImTextureId(uint32_t mip_index = 0) const;

  /**
   * @brief Gets the ImGui texture ID for the depth attachment at a specific mip level.
   * @param mip_index Index of the mip level.
   * @return ImGui texture ID for the depth attachment.
   */
  [[nodiscard]] ImTextureID GetDepthImTextureId(uint32_t mip_index = 0) const;

  /**
   * @brief Applies the graphics pipeline states to the render texture.
   * @param global_pipeline_state Global pipeline state to apply.
   */
  void ApplyGraphicsPipelineStates(GraphicsPipelineStates& global_pipeline_state) const;

  /**
   * @brief Saves the contents of the render texture to a file.
   * @param path Path to save the file to.
   * @return True if the save operation was successful, false otherwise.
   */
  [[maybe_unused]] bool Save(const std::filesystem::path& path) const;

  /**
   * @brief Stores the render texture contents as a PNG file.
   * @param path Path to save the PNG file to.
   * @param resize_x Optional width to resize the texture to (-1 for no resizing).
   * @param resize_y Optional height to resize the texture to (-1 for no resizing).
   * @param compression_level Compression level for the PNG file (default: 8).
   */
  void StoreToPng(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1,
                  unsigned compression_level = 8) const;

  /**
   * @brief Stores the linear depth of the render texture as a PNG file.
   * @param path Path to save the PNG file to.
   * @param near_distance Near plane distance.
   * @param far_distance Far plane distance.
   * @param max_depth Maximum depth value.
   * @param resize_x Optional width to resize the texture to (-1 for no resizing).
   * @param resize_y Optional height to resize the texture to (-1 for no resizing).
   * @param compression_level Compression level for the PNG file (default: 8).
   */
  void StoreLinearDepthToPng(const std::filesystem::path& path, float near_distance, float far_distance,
                             float max_depth, int resize_x = -1, int resize_y = -1,
                             unsigned compression_level = 8) const;

  /**
   * @brief Stores the render texture contents as a JPG file.
   * @param path Path to save the JPG file to.
   * @param resize_x Optional width to resize the texture to (-1 for no resizing).
   * @param resize_y Optional height to resize the texture to (-1 for no resizing).
   * @param quality Quality of the JPG file (default: 100).
   */
  void StoreToJpg(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1,
                  unsigned quality = 100) const;

  /**
   * @brief Stores the render texture contents as an HDR file.
   * @param path Path to save the HDR file to.
   * @param resize_x Optional width to resize the texture to (-1 for no resizing).
   * @param resize_y Optional height to resize the texture to (-1 for no resizing).
   * @param quality Quality of the HDR file (default: 100).
   */
  void StoreToHdr(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1,
                  unsigned quality = 100) const;

  /**
   * @brief Gets the descriptor set for presenting the color texture.
   * @return Shared pointer to the color present descriptor set.
   */
  const std::shared_ptr<DescriptorSet>& GetColorPresentDescriptorSet() const;

  /**
   * @brief Gets the descriptor set for presenting the depth texture.
   * @return Shared pointer to the depth present descriptor set.
   */
  const std::shared_ptr<DescriptorSet>& GetDepthPresentDescriptorSet() const;

  /**
   * @brief Gets the descriptor set for storage.
   * @return Shared pointer to the storage descriptor set.
   */
  const std::shared_ptr<DescriptorSet>& GetStorageDescriptorSet() const;
};

}  // namespace evo_engine
