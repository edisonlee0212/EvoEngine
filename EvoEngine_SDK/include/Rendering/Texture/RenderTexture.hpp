#pragma once
#include "GraphicsPipeline.hpp"
#include "GraphicsPipelineStates.hpp"
#include "GraphicsResources.hpp"
namespace evo_engine {
struct RenderTextureCreateInfo {
  VkExtent3D extent = {1, 1, 1};
  VkImageViewType image_view_type = VK_IMAGE_VIEW_TYPE_2D;
  bool color = true;
  bool depth = true;
};

class RenderTexture {
  friend class Platform;
  friend class RenderLayer;
  std::shared_ptr<Image> color_image_ = {};
  std::vector<std::shared_ptr<ImageView>> color_image_views_ = {};

  std::shared_ptr<Image> depth_image_ = {};
  std::vector<std::shared_ptr<ImageView>> depth_image_views_ = {};
  std::vector<std::shared_ptr<ImageView>> debug_depth_image_views_ = {};
  VkExtent3D extent_;
  VkImageViewType image_view_type_;
  std::shared_ptr<Sampler> color_sampler_ = {};
  std::shared_ptr<Sampler> depth_sampler_ = {};
  std::vector<ImTextureID> color_im_texture_ids_{};
  std::vector<ImTextureID> depth_im_texture_ids_{};

  bool color_ = true;
  bool depth_ = true;
  void Initialize(const RenderTextureCreateInfo& render_texture_create_info, uint32_t mip_levels = 1);
  std::shared_ptr<DescriptorSet> depth_present_descriptor_set_;
  std::shared_ptr<DescriptorSet> color_present_descriptor_set_;
  std::shared_ptr<DescriptorSet> storage_descriptor_set_;

 public:
  inline static std::shared_ptr<DescriptorSetLayout> render_texture_storage_layout;
  inline static std::shared_ptr<DescriptorSetLayout> render_texture_present_layout;

  void Clear(VkCommandBuffer vk_command_buffer) const;
  explicit RenderTexture(const RenderTextureCreateInfo& render_texture_create_info);
  void Resize(VkExtent3D extent, uint32_t mip_level = 1);
  void AppendColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos, VkAttachmentLoadOp load_op,
                                  VkAttachmentStoreOp store_op, uint32_t mip_level = 0) const;
  [[nodiscard]] VkRenderingAttachmentInfo GetDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                 VkAttachmentStoreOp store_op,
                                                                 uint32_t mip_level = 0) const;
  [[nodiscard]] VkExtent3D GetExtent() const;
  [[nodiscard]] VkImageViewType GetImageViewType() const;
  [[nodiscard]] uint32_t GetMipLevels() const;
  [[nodiscard]] const std::shared_ptr<Sampler>& GetColorSampler() const;
  [[nodiscard]] const std::shared_ptr<Sampler>& GetDepthSampler() const;
  [[nodiscard]] const std::shared_ptr<Image>& GetColorImage();
  [[nodiscard]] const std::shared_ptr<Image>& GetDepthImage();
  [[nodiscard]] const std::shared_ptr<ImageView>& GetColorImageView(uint32_t mip_index = 0);
  [[nodiscard]] const std::shared_ptr<ImageView>& GetDepthImageView(uint32_t mip_index = 0);
  void Render(VkCommandBuffer vk_command_buffer, VkAttachmentLoadOp load_op, VkAttachmentStoreOp store_op,
              const std::function<void()>& func, uint32_t mip_level = 0) const;
  [[nodiscard]] ImTextureID GetColorImTextureId(uint32_t mip_index = 0) const;
  [[nodiscard]] ImTextureID GetDepthImTextureId(uint32_t mip_index = 0) const;
  void ApplyGraphicsPipelineStates(GraphicsPipelineStates& global_pipeline_state) const;
  [[maybe_unused]] bool Save(const std::filesystem::path& path) const;
  void StoreToPng(const std::string& path, int resize_x = -1, int resize_y = -1, unsigned compression_level = 8) const;
  void StoreToJpg(const std::string& path, int resize_x = -1, int resize_y = -1, unsigned quality = 100) const;
  void StoreToHdr(const std::string& path, int resize_x = -1, int resize_y = -1, unsigned quality = 100) const;

  const std::shared_ptr<DescriptorSet>& GetColorPresentDescriptorSet() const;
  const std::shared_ptr<DescriptorSet>& GetDepthPresentDescriptorSet() const;
  const std::shared_ptr<DescriptorSet>& GetStorageDescriptorSet() const;
};
}  // namespace evo_engine
