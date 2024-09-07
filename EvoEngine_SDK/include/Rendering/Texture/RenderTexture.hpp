#pragma once
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
  friend class Camera;
  std::shared_ptr<Image> color_image_ = {};
  std::shared_ptr<ImageView> color_image_view_ = {};

  std::shared_ptr<Image> depth_image_ = {};
  std::shared_ptr<ImageView> depth_image_view_ = {};

  VkExtent3D extent_;
  VkImageViewType image_view_type_;
  std::shared_ptr<Sampler> color_sampler_ = {};
  std::shared_ptr<Sampler> depth_sampler_ = {};
  ImTextureID color_im_texture_id_ = nullptr;

  bool color_ = true;
  bool depth_ = true;
  void Initialize(const RenderTextureCreateInfo& render_texture_create_info);
  std::shared_ptr<DescriptorSet> descriptor_set_;

 public:
  void Clear(VkCommandBuffer vk_command_buffer ) const;
  explicit RenderTexture(const RenderTextureCreateInfo& render_texture_create_info);
  void Resize(VkExtent3D extent);
  void AppendColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos, VkAttachmentLoadOp load_op,
                                  VkAttachmentStoreOp store_op) const;
  [[nodiscard]] VkRenderingAttachmentInfo GetDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                 VkAttachmentStoreOp store_op) const;
  [[nodiscard]] VkExtent3D GetExtent() const;
  [[nodiscard]] VkImageViewType GetImageViewType() const;

  [[nodiscard]] const std::shared_ptr<Sampler>& GetColorSampler() const;
  [[nodiscard]] const std::shared_ptr<Sampler>& GetDepthSampler() const;
  [[nodiscard]] const std::shared_ptr<Image>& GetColorImage();
  [[nodiscard]] const std::shared_ptr<Image>& GetDepthImage();
  [[nodiscard]] const std::shared_ptr<ImageView>& GetColorImageView();
  [[nodiscard]] const std::shared_ptr<ImageView>& GetDepthImageView();
  void BeginRendering(VkCommandBuffer vk_command_buffer , VkAttachmentLoadOp load_op,
                      VkAttachmentStoreOp store_op) const;
  void EndRendering(VkCommandBuffer vk_command_buffer ) const;
  [[nodiscard]] ImTextureID GetColorImTextureId() const;
  void ApplyGraphicsPipelineStates(GraphicsPipelineStates& global_pipeline_state) const;
  [[maybe_unused]] bool Save(const std::filesystem::path& path) const;
  void StoreToPng(const std::string& path, int resize_x = -1, int resize_y = -1, unsigned compression_level = 8) const;
  void StoreToJpg(const std::string& path, int resize_x = -1, int resize_y = -1, unsigned quality = 100) const;
  void StoreToHdr(const std::string& path, int resize_x = -1, int resize_y = -1, unsigned quality = 100) const;
};
}  // namespace evo_engine
