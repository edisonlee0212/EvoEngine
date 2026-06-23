#include "RenderPasses/DdgiPassUtilities.hpp"

#include "Resources.hpp"
#include "Texture2D.hpp"

using namespace evo_engine;

VkDescriptorImageInfo evo_engine::CreateDdgiFallbackImageInfo() {
  VkDescriptorImageInfo image_info{};
  const auto missing_texture = Resources::GetInstance().GetMissingTexture();
  if (missing_texture) {
    image_info.imageLayout = missing_texture->GetLayout();
    image_info.imageView = missing_texture->GetVkImageView();
    image_info.sampler = missing_texture->GetVkSampler();
  }
  return image_info;
}

bool evo_engine::IsValidDescriptorImageInfo(const VkDescriptorImageInfo& image_info) {
  return image_info.imageView != VK_NULL_HANDLE && image_info.sampler != VK_NULL_HANDLE;
}
