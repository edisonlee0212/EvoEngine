#include "RenderTexture.hpp"

#include "Application.hpp"
#include "Console.hpp"
#include "EditorLayer.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"

using namespace evo_engine;

void RenderTexture::Initialize(const RenderTextureCreateInfo& render_texture_create_info, uint32_t mip_levels) {
  if (!Platform::Initialized())
    return;
  color_image_views_.clear();
  color_image_.reset();
  color_sampler_.reset();
  depth_image_views_.clear();
  depth_image_.reset();
  depth_sampler_.reset();
  color_im_texture_ids_.clear();
  depth_im_texture_ids_.clear();
  int layer_count = render_texture_create_info.image_view_type == VK_IMAGE_VIEW_TYPE_CUBE ? 6 : 1;
  depth_ = render_texture_create_info.depth;
  color_ = render_texture_create_info.color;
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  switch (render_texture_create_info.image_view_type) {
    case VK_IMAGE_VIEW_TYPE_1D:
      image_info.imageType = VK_IMAGE_TYPE_1D;
      break;
    case VK_IMAGE_VIEW_TYPE_2D:
      image_info.imageType = VK_IMAGE_TYPE_2D;
      break;
    case VK_IMAGE_VIEW_TYPE_3D:
      image_info.imageType = VK_IMAGE_TYPE_3D;
      break;
    case VK_IMAGE_VIEW_TYPE_CUBE:
      image_info.imageType = VK_IMAGE_TYPE_2D;
      break;
    case VK_IMAGE_VIEW_TYPE_1D_ARRAY:
      image_info.imageType = VK_IMAGE_TYPE_1D;
      break;
    case VK_IMAGE_VIEW_TYPE_2D_ARRAY:
      image_info.imageType = VK_IMAGE_TYPE_2D;
      break;
    case VK_IMAGE_VIEW_TYPE_CUBE_ARRAY:
      image_info.imageType = VK_IMAGE_TYPE_2D;
      break;
    case VK_IMAGE_VIEW_TYPE_MAX_ENUM:
      EVOENGINE_ERROR("Wrong imageViewType!");
      break;
  }
  if (color_) {
    image_info.extent = render_texture_create_info.extent;
    image_info.mipLevels = mip_levels;
    image_info.arrayLayers = layer_count;
    image_info.format = Platform::Constants::render_texture_color;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                       VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    color_image_ = std::make_shared<Image>(image_info);
    Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
      if (mip_levels > 1) {
        color_image_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        color_image_->GenerateMipmaps(vk_command_buffer);
      }
      color_image_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    });

    for (unsigned int mip = 0; mip < mip_levels; ++mip) {
      VkImageViewCreateInfo view_info{};
      view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      view_info.image = color_image_->GetVkImage();
      view_info.viewType = render_texture_create_info.image_view_type;
      view_info.format = Platform::Constants::render_texture_color;
      view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      view_info.subresourceRange.baseMipLevel = mip;
      view_info.subresourceRange.levelCount = 1;
      view_info.subresourceRange.baseArrayLayer = 0;
      view_info.subresourceRange.layerCount = layer_count;
      color_image_views_.emplace_back(std::make_shared<ImageView>(view_info));
    }

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    color_sampler_ = std::make_shared<Sampler>(sampler_info);
    color_im_texture_ids_.resize(mip_levels);
    for (unsigned int mip = 0; mip < mip_levels; ++mip) {
      EditorLayer::UpdateTextureId(color_im_texture_ids_[mip], color_sampler_->GetVkSampler(),
                                   color_image_views_[mip]->GetVkImageView(), color_image_->GetLayout());
    }
  }

  if (depth_) {
    VkImageCreateInfo depth_info{};
    depth_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    depth_info.imageType = image_info.imageType;
    depth_info.extent = render_texture_create_info.extent;
    depth_info.mipLevels = 1;
    depth_info.arrayLayers = layer_count;
    depth_info.format = Platform::Constants::render_texture_depth;
    depth_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    depth_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depth_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                       VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    depth_info.samples = VK_SAMPLE_COUNT_1_BIT;
    depth_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    depth_image_ = std::make_shared<Image>(depth_info);
    Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
      depth_image_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    });

    for (unsigned int mip = 0; mip < mip_levels; ++mip) {
      VkImageViewCreateInfo depth_view_info{};
      depth_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      depth_view_info.image = depth_image_->GetVkImage();
      depth_view_info.viewType = render_texture_create_info.image_view_type;
      depth_view_info.format = Platform::Constants::render_texture_depth;
      depth_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
      depth_view_info.subresourceRange.baseMipLevel = mip;
      depth_view_info.subresourceRange.levelCount = 1;
      depth_view_info.subresourceRange.baseArrayLayer = 0;
      depth_view_info.subresourceRange.layerCount = layer_count;
      depth_image_views_.emplace_back(std::make_shared<ImageView>(depth_view_info));
    }

    VkSamplerCreateInfo depth_sampler_info{};
    depth_sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    depth_sampler_info.magFilter = VK_FILTER_LINEAR;
    depth_sampler_info.minFilter = VK_FILTER_LINEAR;
    depth_sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    depth_sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    depth_sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    depth_sampler_info.anisotropyEnable = VK_TRUE;
    depth_sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
    depth_sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    depth_sampler_info.unnormalizedCoordinates = VK_FALSE;
    depth_sampler_info.compareEnable = VK_FALSE;
    depth_sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    depth_sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    depth_sampler_ = std::make_shared<Sampler>(depth_sampler_info);
    depth_im_texture_ids_.resize(mip_levels);
    for (unsigned int mip = 0; mip < mip_levels; ++mip) {
      EditorLayer::UpdateTextureId(depth_im_texture_ids_[mip], depth_sampler_->GetVkSampler(),
                                   depth_image_views_[mip]->GetVkImageView(), depth_image_->GetLayout());
    }
  }
  extent_ = render_texture_create_info.extent;
  image_view_type_ = render_texture_create_info.image_view_type;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();

  if (color_) {
    color_present_descriptor_set_ =
        std::make_shared<DescriptorSet>(render_layer->GetRenderTexturePresentDescriptorSetLayout());
    VkDescriptorImageInfo present_info;
    present_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    present_info.imageView = color_image_views_[0]->GetVkImageView();
    present_info.sampler = color_sampler_->GetVkSampler();
    color_present_descriptor_set_->UpdateImageDescriptorBinding(0, present_info);

    storage_descriptor_set_ =
        std::make_shared<DescriptorSet>(render_layer->GetRenderTextureStorageDescriptorSetLayout());
    VkDescriptorImageInfo storage_info;
    storage_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    storage_info.imageView = color_image_views_[0]->GetVkImageView();
    storage_info.sampler = color_sampler_->GetVkSampler();
    storage_descriptor_set_->UpdateImageDescriptorBinding(0, storage_info);
  }

  if (depth_) {
    depth_present_descriptor_set_ =
        std::make_shared<DescriptorSet>(render_layer->GetRenderTexturePresentDescriptorSetLayout());
    VkDescriptorImageInfo present_info;
    present_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    present_info.imageView = depth_image_views_[0]->GetVkImageView();
    present_info.sampler = depth_sampler_->GetVkSampler();
    depth_present_descriptor_set_->UpdateImageDescriptorBinding(0, present_info);
  }
}

void RenderTexture::Clear(VkCommandBuffer vk_command_buffer) const {
  if (depth_) {
    const auto prev_depth_layout = depth_image_->GetLayout();
    depth_image_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    VkImageSubresourceRange depth_subresource_range{};
    depth_subresource_range.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    depth_subresource_range.baseMipLevel = 0;
    depth_subresource_range.levelCount = 1;
    depth_subresource_range.baseArrayLayer = 0;
    depth_subresource_range.layerCount = 1;
    VkClearDepthStencilValue depth_stencil_value{};
    depth_stencil_value = {1, 0};
    Platform::ClearDepthStencilImage(vk_command_buffer, *depth_image_, depth_stencil_value, 1,
                                     &depth_subresource_range);
    depth_image_->TransitImageLayout(vk_command_buffer, prev_depth_layout);
  }
  if (color_) {
    const auto prev_color_layout = color_image_->GetLayout();
    color_image_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    VkImageSubresourceRange color_subresource_range{};
    color_subresource_range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    color_subresource_range.baseMipLevel = 0;
    color_subresource_range.levelCount = 1;
    color_subresource_range.baseArrayLayer = 0;
    color_subresource_range.layerCount = 1;
    VkClearColorValue color_value{};
    color_value = {0, 0, 0, 1};
    Platform::ClearColorImage(vk_command_buffer, *color_image_, color_value, 1, &color_subresource_range);
    color_image_->TransitImageLayout(vk_command_buffer, prev_color_layout);
  }
}

RenderTexture::RenderTexture(const RenderTextureCreateInfo& render_texture_create_info) {
  Initialize(render_texture_create_info);
}
void RenderTexture::Resize(const VkExtent3D extent, const uint32_t mip_level) {
  if (extent.width == extent_.width && extent.height == extent_.height && extent.depth == extent_.depth)
    return;
  RenderTextureCreateInfo render_texture_create_info;
  render_texture_create_info.extent = extent;
  render_texture_create_info.color = color_;
  render_texture_create_info.depth = depth_;
  render_texture_create_info.image_view_type = image_view_type_;
  Initialize(render_texture_create_info, mip_level);
}

void RenderTexture::AppendColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos,
                                               const VkAttachmentLoadOp load_op, const VkAttachmentStoreOp store_op,
                                               const uint32_t mip_level) const {
  assert(color_);
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  attachment.clearValue.color = {0, 0, 0, 0};
  attachment.imageView = color_image_views_[mip_level]->GetVkImageView();
  attachment_infos.push_back(attachment);
}

VkRenderingAttachmentInfo RenderTexture::GetDepthAttachmentInfo(const VkAttachmentLoadOp load_op,
                                                                const VkAttachmentStoreOp store_op,
                                                                const uint32_t mip_level) const {
  assert(depth_);
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  attachment.clearValue.depthStencil = {1, 0};
  attachment.imageView = depth_image_views_[mip_level]->GetVkImageView();
  return attachment;
}

VkExtent3D RenderTexture::GetExtent() const {
  return extent_;
}

VkImageViewType RenderTexture::GetImageViewType() const {
  return image_view_type_;
}

uint32_t RenderTexture::GetMipLevels() const {
  return color_image_views_.size();
}

const std::shared_ptr<Sampler>& RenderTexture::GetColorSampler() const {
  assert(color_);
  return color_sampler_;
}

const std::shared_ptr<Sampler>& RenderTexture::GetDepthSampler() const {
  assert(depth_);
  return depth_sampler_;
}

const std::shared_ptr<Image>& RenderTexture::GetColorImage() {
  assert(color_);
  return color_image_;
}

const std::shared_ptr<Image>& RenderTexture::GetDepthImage() {
  assert(depth_);
  return depth_image_;
}

const std::shared_ptr<ImageView>& RenderTexture::GetColorImageView(const uint32_t mip_index) {
  assert(color_);
  return color_image_views_[mip_index];
}

const std::shared_ptr<ImageView>& RenderTexture::GetDepthImageView(const uint32_t mip_index) {
  assert(depth_);
  return depth_image_views_[mip_index];
}

void RenderTexture::Render(const VkCommandBuffer vk_command_buffer, const VkAttachmentLoadOp load_op,
                           const VkAttachmentStoreOp store_op, const std::function<void()>& func,
                           const uint32_t mip_level) const {
  if (depth_)
    depth_image_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
  if (color_)
    color_image_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
  VkRect2D render_area;
  render_area.offset = {0, 0};
  const auto mip_scale = glm::pow(0.5f, static_cast<float>(mip_level));
  render_area.extent.width = static_cast<uint32_t>(glm::max(1.0f, static_cast<float>(extent_.width) * mip_scale));
  render_area.extent.height = static_cast<uint32_t>(glm::max(1.0f, static_cast<float>(extent_.height) * mip_scale));
  VkRenderingInfo render_info{};
  VkRenderingAttachmentInfo depth_attachment;
  if (depth_) {
    depth_attachment = GetDepthAttachmentInfo(load_op, store_op, mip_level);
    render_info.pDepthAttachment = &depth_attachment;
  } else {
    render_info.pDepthAttachment = nullptr;
  }
  std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
  if (color_) {
    AppendColorAttachmentInfos(color_attachment_infos, load_op, store_op, mip_level);
    render_info.colorAttachmentCount = color_attachment_infos.size();
    render_info.pColorAttachments = color_attachment_infos.data();
  } else {
    render_info.colorAttachmentCount = 0;
    render_info.pColorAttachments = nullptr;
  }
  render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  render_info.renderArea = render_area;
  render_info.layerCount = 1;
  Platform::BeginRendering(vk_command_buffer, render_info);
  func();
  Platform::EndRendering(vk_command_buffer);
}

ImTextureID RenderTexture::GetColorImTextureId(const uint32_t mip_index) const {
  return color_im_texture_ids_[glm::clamp(mip_index, 0u, static_cast<uint32_t>(color_im_texture_ids_.size()) - 1)];
}

ImTextureID RenderTexture::GetDepthImTextureId(uint32_t mip_index) const {
  return depth_im_texture_ids_[glm::clamp(mip_index, 0u, static_cast<uint32_t>(depth_im_texture_ids_.size()) - 1)];
}

void RenderTexture::ApplyGraphicsPipelineStates(GraphicsPipelineStates& global_pipeline_state) const {
  VkViewport viewport;
  viewport.x = 0.0f;
  viewport.y = 0.0f;
  viewport.width = static_cast<float>(extent_.width);
  viewport.height = static_cast<float>(extent_.height);
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;

  VkRect2D scissor;
  scissor.offset = {0, 0};
  scissor.extent.width = extent_.width;
  scissor.extent.height = extent_.height;
  global_pipeline_state.view_port = viewport;
  global_pipeline_state.scissor = scissor;
  global_pipeline_state.color_blend_attachment_states.clear();
  if (color_)
    global_pipeline_state.color_blend_attachment_states.resize(1);
}

bool RenderTexture::Save(const std::filesystem::path& path) const {
  if (path.extension() == ".png") {
    StoreToPng(path.string());
  } else if (path.extension() == ".jpg") {
    StoreToJpg(path.string());
  } else if (path.extension() == ".hdr") {
    StoreToHdr(path.string());
  } else {
    EVOENGINE_ERROR("Not implemented!");
    return false;
  }
  return true;
}

void RenderTexture::GetRgbaChannelData(std::vector<glm::vec4>& dst) const {
  assert(color_);
  const auto resolution_x = color_image_->GetExtent().width;
  const auto resolution_y = color_image_->GetExtent().height;
  dst.resize(resolution_x * resolution_y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution_x * resolution_y);
  image_buffer.CopyFromImage(*color_image_);
  image_buffer.DownloadVector(dst, resolution_x * resolution_y);
}

void RenderTexture::StoreToPng(const std::filesystem::path& path, int resize_x, int resize_y,
                               unsigned compression_level) const {
  assert(color_);
  stbi_write_png_compression_level = compression_level;
  const auto resolution_x = color_image_->GetExtent().width;
  const auto resolution_y = color_image_->GetExtent().height;
  std::vector<glm::vec4> dst;
  GetRgbaChannelData(dst);
  std::vector<uint8_t> pixels;
  if (resize_x > 0 && resize_y > 0 && (resize_x != resolution_x || resize_y != resolution_y)) {
    constexpr size_t store_channels = 4;
    std::vector<float> res;
    res.resize(resize_x * resize_y * store_channels);
    stbir_resize_float_linear(&dst.front().x, resolution_x, resolution_y, 0, res.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(store_channels));
    pixels.resize(resize_x * resize_y * store_channels);
    for (int i = 0; i < resize_x * resize_y; i++) {
      pixels[i * store_channels] = glm::clamp<int>(int(255.9f * res[i * store_channels]), 0, 255);
      pixels[i * store_channels + 1] = glm::clamp<int>(int(255.9f * res[i * store_channels + 1]), 0, 255);
      pixels[i * store_channels + 2] = glm::clamp<int>(int(255.9f * res[i * store_channels + 2]), 0, 255);
      pixels[i * store_channels + 3] = 255;
    }
    stbi_flip_vertically_on_write(true);
    stbi_write_png(path.string().c_str(), resize_x, resize_y, store_channels, pixels.data(), 0);
  } else {
    constexpr size_t store_channels = 4;
    pixels.resize(resolution_x * resolution_y * store_channels);
    for (int i = 0; i < resolution_x * resolution_y; i++) {
      pixels[i * store_channels] = glm::clamp<int>(int(255.9f * dst[i].x), 0, 255);
      pixels[i * store_channels + 1] = glm::clamp<int>(int(255.9f * dst[i].y), 0, 255);
      pixels[i * store_channels + 2] = glm::clamp<int>(int(255.9f * dst[i].z), 0, 255);
      pixels[i * store_channels + 3] = 255;
    }
    stbi_flip_vertically_on_write(true);
    stbi_write_png(path.string().c_str(), resolution_x, resolution_y, store_channels, pixels.data(), 0);
  }
}

void RenderTexture::StoreLinearDepthToPng(const std::filesystem::path& path, float near_distance, float far_distance,
                                          float max_depth, int resize_x, int resize_y,
                                          unsigned compression_level) const {
  assert(color_);
  stbi_write_png_compression_level = compression_level;
  const auto resolution_x = depth_image_->GetExtent().width;
  const auto resolution_y = depth_image_->GetExtent().height;
  constexpr size_t store_channels = 4;
  std::vector<float> dst;
  dst.resize(resolution_x * resolution_y);
  // Retrieve image data here.
  Buffer image_buffer(sizeof(float) * resolution_x * resolution_y);
  image_buffer.CopyFromDepth(*depth_image_, 4);
  image_buffer.DownloadVector(dst, resolution_x * resolution_y);
  std::vector<uint8_t> pixels;

  const auto linearize_depth = [&](const float ndc_depth) {
    return near_distance * far_distance / (far_distance - ndc_depth * (far_distance - near_distance));
  };

  if (resize_x > 0 && resize_y > 0 && (resize_x != resolution_x || resize_y != resolution_y)) {
    std::vector<float> res;
    res.resize(resize_x * resize_y * store_channels);
    stbir_resize(dst.data(), resolution_x, resolution_y, 0, res.data(), resize_x, resize_y, 0, STBIR_1CHANNEL,
                 STBIR_TYPE_FLOAT, STBIR_EDGE_REFLECT, STBIR_FILTER_POINT_SAMPLE);
    pixels.resize(resize_x * resize_y * store_channels);
    for (int i = 0; i < resize_x * resize_y; i++) {
      const auto depth = glm::clamp<int>(static_cast<int>(255.9f * linearize_depth(res[i]) / max_depth), 0, 255);
      pixels[i * store_channels] = depth;
      pixels[i * store_channels + 1] = depth;
      pixels[i * store_channels + 2] = depth;
      if (store_channels == 4)
        pixels[i * store_channels + 3] = 255;
    }

    stbi_flip_vertically_on_write(true);
    stbi_write_png(path.string().c_str(), resize_x, resize_y, store_channels, pixels.data(), 0);
  } else {
    pixels.resize(resolution_x * resolution_y * store_channels);
    for (int i = 0; i < resolution_x * resolution_y; i++) {
      const auto depth = glm::clamp<int>(static_cast<int>(255.9f * linearize_depth(dst[i]) / max_depth), 0, 255);
      pixels[i * store_channels] = depth;
      pixels[i * store_channels + 1] = depth;
      pixels[i * store_channels + 2] = depth;
      if (store_channels == 4)
        pixels[i * store_channels + 3] = 255;
    }
    stbi_flip_vertically_on_write(true);
    stbi_write_png(path.string().c_str(), resolution_x, resolution_y, store_channels, pixels.data(), 0);
  }
}

void RenderTexture::StoreToJpg(const std::filesystem::path& path, int resize_x, int resize_y, unsigned quality) const {
  assert(color_);
  const auto resolution_x = color_image_->GetExtent().width;
  const auto resolution_y = color_image_->GetExtent().height;
  std::vector<float> dst;
  const size_t store_channels = 3;
  const size_t channels = 4;
  dst.resize(resolution_x * resolution_y * channels);
  // Retrieve image data here.
  // Retrieve image data here.
  Buffer image_buffer(sizeof(glm::vec4) * resolution_x * resolution_y);
  image_buffer.CopyFromImage(*color_image_);
  image_buffer.DownloadVector(dst, resolution_x * resolution_y * channels);
  std::vector<uint8_t> pixels;
  if (resize_x > 0 && resize_y > 0 && (resize_x != resolution_x || resize_y != resolution_y)) {
    std::vector<float> res;
    res.resize(resize_x * resize_y * store_channels);
    stbir_resize_float_linear(dst.data(), resolution_x, resolution_y, 0, res.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(store_channels));
    pixels.resize(resize_x * resize_y * store_channels);
    for (int i = 0; i < resize_x * resize_y; i++) {
      pixels[i * store_channels] = glm::clamp<int>(int(255.9f * res[i * channels]), 0, 255);
      pixels[i * store_channels + 1] = glm::clamp<int>(int(255.9f * res[i * channels + 1]), 0, 255);
      pixels[i * store_channels + 2] = glm::clamp<int>(int(255.9f * res[i * channels + 2]), 0, 255);
      if (store_channels == 4)
        pixels[i * store_channels + 3] = glm::clamp<int>(int(255.9f * res[i * channels + 3]), 0, 255);
    }
    stbi_flip_vertically_on_write(true);
    stbi_write_jpg(path.string().c_str(), resize_x, resize_y, store_channels, pixels.data(), quality);
  } else {
    pixels.resize(resolution_x * resolution_y * 3);
    for (int i = 0; i < resolution_x * resolution_y; i++) {
      pixels[i * store_channels] = glm::clamp<int>(int(255.9f * dst[i * channels]), 0, 255);
      pixels[i * store_channels + 1] = glm::clamp<int>(int(255.9f * dst[i * channels + 1]), 0, 255);
      pixels[i * store_channels + 2] = glm::clamp<int>(int(255.9f * dst[i * channels + 2]), 0, 255);
      if (store_channels == 4)
        pixels[i * store_channels + 3] = glm::clamp<int>(int(255.9f * dst[i * channels + 3]), 0, 255);
    }
    stbi_flip_vertically_on_write(true);
    stbi_write_jpg(path.string().c_str(), resolution_x, resolution_y, store_channels, pixels.data(), quality);
  }
}

void RenderTexture::StoreToHdr(const std::filesystem::path& path, int resize_x, int resize_y, unsigned quality) const {
  assert(color_);
  const auto resolution_x = color_image_->GetExtent().width;
  const auto resolution_y = color_image_->GetExtent().height;
  const size_t channels = 4;
  std::vector<float> dst;
  dst.resize(resolution_x * resolution_y * channels);
  Buffer image_buffer(sizeof(glm::vec4) * resolution_x * resolution_y);
  image_buffer.CopyFromImage(*color_image_);
  image_buffer.DownloadVector(dst, resolution_x * resolution_y * channels);
  // Retrieve image data here.

  stbi_flip_vertically_on_write(true);
  if (resize_x > 0 && resize_y > 0 && (resize_x != resolution_x || resize_y != resolution_y)) {
    std::vector<float> pixels;
    pixels.resize(resize_x * resize_y * channels);
    stbir_resize_float_linear(dst.data(), resolution_x, resolution_y, 0, pixels.data(), resize_x, resize_y, 0,
                              static_cast<stbir_pixel_layout>(channels));
    stbi_write_hdr(path.string().c_str(), resolution_x, resolution_y, channels, pixels.data());
  } else {
    stbi_write_hdr(path.string().c_str(), resolution_x, resolution_y, channels, dst.data());
  }
}

const std::shared_ptr<DescriptorSet>& RenderTexture::GetColorPresentDescriptorSet() const {
  return color_present_descriptor_set_;
}

const std::shared_ptr<DescriptorSet>& RenderTexture::GetDepthPresentDescriptorSet() const {
  return depth_present_descriptor_set_;
}

const std::shared_ptr<DescriptorSet>& RenderTexture::GetStorageDescriptorSet() const {
  return storage_descriptor_set_;
}
