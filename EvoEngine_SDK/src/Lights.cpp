#include "Lights.hpp"

#include "Application.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Serialization.hpp"
using namespace evo_engine;

bool SpotLight::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &cast_shadow))
    changed = false;
  if (ImGui::ColorEdit3("Color", &diffuse[0]))
    changed = false;
  if (ImGui::DragFloat("Intensity", &diffuse_brightness, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Bias", &bias, 0.001f, 0.0f, 999.0f))
    changed = false;

  if (ImGui::DragFloat("Constant", &constant, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Linear", &linear, 0.001f, 0, 1, "%.3f"))
    changed = false;
  if (ImGui::DragFloat("Quadratic", &quadratic, 0.001f, 0, 10, "%.4f"))
    changed = false;

  if (ImGui::DragFloat("Inner Degrees", &inner_degrees, 0.1f, 0.0f, outer_degrees))
    changed = false;
  if (ImGui::DragFloat("Outer Degrees", &outer_degrees, 0.1f, inner_degrees, 180.0f))
    changed = false;
  if (ImGui::DragFloat("Light Size", &light_size, 0.001f, 0.0f, 999.0f))
    changed = false;

  return changed;
}

void SpotLight::OnCreate() {
  SetEnabled(true);
}

void SpotLight::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "cast_shadow" << YAML::Value << cast_shadow;
  out << YAML::Key << "inner_degrees" << YAML::Value << inner_degrees;
  out << YAML::Key << "outer_degrees" << YAML::Value << outer_degrees;
  out << YAML::Key << "constant" << YAML::Value << constant;
  out << YAML::Key << "linear" << YAML::Value << linear;
  out << YAML::Key << "quadratic" << YAML::Value << quadratic;
  out << YAML::Key << "bias" << YAML::Value << bias;
  out << YAML::Key << "diffuse" << YAML::Value << diffuse;
  out << YAML::Key << "diffuse_brightness" << YAML::Value << diffuse_brightness;
  out << YAML::Key << "light_size" << YAML::Value << light_size;
}

void SpotLight::Deserialize(const YAML::Node& in) {
  cast_shadow = in["cast_shadow"].as<bool>();
  inner_degrees = in["inner_degrees"].as<float>();
  outer_degrees = in["outer_degrees"].as<float>();
  constant = in["constant"].as<float>();
  linear = in["linear"].as<float>();
  quadratic = in["quadratic"].as<float>();
  bias = in["bias"].as<float>();
  diffuse = in["diffuse"].as<glm::vec3>();
  diffuse_brightness = in["diffuse_brightness"].as<float>();
  light_size = in["light_size"].as<float>();
}

float PointLight::GetFarPlane() const {
  const float light_max = glm::max(glm::max(diffuse.x, diffuse.y), diffuse.z);
  return (-linear + glm::sqrt(linear * linear - 4 * quadratic * (constant - (256.0 / 5.0) * light_max))) /
         (2 * quadratic);
}

float SpotLight::GetFarPlane() const {
  const float light_max = glm::max(glm::max(diffuse.x, diffuse.y), diffuse.z);
  return (-linear + glm::sqrt(linear * linear - 4 * quadratic * (constant - (256.0 / 5.0) * light_max))) /
         (2 * quadratic);
}

bool PointLight::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &cast_shadow))
    changed = false;
  if (ImGui::ColorEdit3("Color", &diffuse[0]))
    changed = false;
  if (ImGui::DragFloat("Intensity", &diffuse_brightness, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Bias", &bias, 0.001f, 0.0f, 999.0f))
    changed = false;

  if (ImGui::DragFloat("Constant", &constant, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Linear", &linear, 0.0001f, 0, 1, "%.4f"))
    changed = false;
  if (ImGui::DragFloat("Quadratic", &quadratic, 0.00001f, 0, 10, "%.5f"))
    changed = false;

  if (ImGui::DragFloat("Light Size", &light_size, 0.001f, 0.0f, 999.0f))
    changed = false;

  return changed;
}

void PointLight::OnCreate() {
  SetEnabled(true);
}

void PointLight::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "cast_shadow" << YAML::Value << cast_shadow;
  out << YAML::Key << "constant" << YAML::Value << constant;
  out << YAML::Key << "linear" << YAML::Value << linear;
  out << YAML::Key << "quadratic" << YAML::Value << quadratic;
  out << YAML::Key << "bias" << YAML::Value << bias;
  out << YAML::Key << "diffuse" << YAML::Value << diffuse;
  out << YAML::Key << "diffuse_brightness" << YAML::Value << diffuse_brightness;
  out << YAML::Key << "light_size" << YAML::Value << light_size;
}

void PointLight::Deserialize(const YAML::Node& in) {
  cast_shadow = in["cast_shadow"].as<bool>();
  constant = in["constant"].as<float>();
  linear = in["linear"].as<float>();
  quadratic = in["quadratic"].as<float>();
  bias = in["bias"].as<float>();
  diffuse = in["diffuse"].as<glm::vec3>();
  diffuse_brightness = in["diffuse_brightness"].as<float>();
  light_size = in["light_size"].as<float>();
}

void DirectionalLight::OnCreate() {
  SetEnabled(true);
}

bool DirectionalLight::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &cast_shadow))
    changed = false;
  if (ImGui::ColorEdit3("Color", &diffuse[0]))
    changed = false;
  if (ImGui::DragFloat("Intensity", &diffuse_brightness, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Bias", &bias, 0.001f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Normal Offset", &normal_offset, 0.001f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Light Size", &light_size, 0.001f, 0.0f, 999.0f))
    changed = false;
  return changed;
}

void DirectionalLight::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "cast_shadow" << YAML::Value << cast_shadow;
  out << YAML::Key << "bias" << YAML::Value << bias;
  out << YAML::Key << "diffuse" << YAML::Value << diffuse;
  out << YAML::Key << "diffuse_brightness" << YAML::Value << diffuse_brightness;
  out << YAML::Key << "light_size" << YAML::Value << light_size;
  out << YAML::Key << "normal_offset" << YAML::Value << normal_offset;
}

void DirectionalLight::Deserialize(const YAML::Node& in) {
  cast_shadow = in["cast_shadow"].as<bool>();
  bias = in["bias"].as<float>();
  diffuse = in["diffuse"].as<glm::vec3>();
  diffuse_brightness = in["diffuse_brightness"].as<float>();
  light_size = in["light_size"].as<float>();
  normal_offset = in["normal_offset"].as<float>();
}
void DirectionalLight::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}

void PointLight::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}

void SpotLight::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}

void Lighting::Consume(glm::vec2 location, uint32_t resolution, uint32_t remaining_size,
                       std::vector<glm::uvec3>& results) {
  assert(resolution > 1);
  results.emplace_back(location.x, location.y, resolution / 2);
  results.emplace_back(location.x + resolution / 2, location.y, resolution / 2);
  results.emplace_back(location.x, location.y + resolution / 2, resolution / 2);
  results.emplace_back(location.x + resolution / 2, location.y + resolution / 2, resolution / 2);
  if (remaining_size > 4) {
    results.pop_back();
    Consume({location.x + resolution / 2, location.y + resolution / 2}, resolution / 2, remaining_size - 3, results);
  }
}

void Lighting::AllocateAtlas(uint32_t size, uint32_t max_resolution, std::vector<glm::uvec3>& results) {
  results.clear();
  if (size == 1) {
    results.emplace_back(0, 0, max_resolution);
  } else {
    results.emplace_back(0, 0, max_resolution / 2);
    results.emplace_back(max_resolution / 2, 0, max_resolution / 2);
    results.emplace_back(0, max_resolution / 2, max_resolution / 2);
    results.emplace_back(max_resolution / 2, max_resolution / 2, max_resolution / 2);
    if (size > 4) {
      results.pop_back();
      Consume({max_resolution / 2, max_resolution / 2}, max_resolution / 2, size - 3, results);
    }
  }
  results.resize(size);
}

Lighting::Lighting() {
  lighting_descriptor_set = std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("LIGHTING_LAYOUT"));
}

void Lighting::Initialize() {
  directional_shadow_map_sampler_.reset();
  directional_light_shadow_map_view_.reset();
  directional_light_shadow_map_.reset();
  directional_light_shadow_map_layered_views_.clear();
  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = Platform::Settings::directional_light_shadow_map_resolution;
    image_info.extent.height = Platform::Settings::directional_light_shadow_map_resolution;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 4;
    image_info.format = Platform::Constants::shadow_map;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    directional_light_shadow_map_ = std::make_shared<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = directional_light_shadow_map_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    view_info.format = Platform::Constants::shadow_map;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 4;

    directional_light_shadow_map_view_ = std::make_shared<ImageView>(view_info);

    for (int i = 0; i < 4; i++) {
      view_info.subresourceRange.baseArrayLayer = i;
      view_info.subresourceRange.layerCount = 1;
      directional_light_shadow_map_layered_views_.emplace_back(std::make_shared<ImageView>(view_info));
    }
    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    directional_shadow_map_sampler_ = std::make_shared<Sampler>(sampler_info);
  }

  point_light_shadow_map_sampler_.reset();
  point_light_shadow_map_view_.reset();
  point_light_shadow_map_.reset();
  point_light_shadow_map_layered_views_.clear();

  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = Platform::Settings::point_light_shadow_map_resolution;
    image_info.extent.height = Platform::Settings::point_light_shadow_map_resolution;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 6;
    image_info.format = Platform::Constants::shadow_map;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    point_light_shadow_map_ = std::make_shared<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = point_light_shadow_map_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    view_info.format = Platform::Constants::shadow_map;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 6;

    point_light_shadow_map_view_ = std::make_shared<ImageView>(view_info);

    for (int i = 0; i < 6; i++) {
      view_info.subresourceRange.baseArrayLayer = i;
      view_info.subresourceRange.layerCount = 1;
      point_light_shadow_map_layered_views_.emplace_back(std::make_shared<ImageView>(view_info));
    }

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    point_light_shadow_map_sampler_ = std::make_shared<Sampler>(sampler_info);
  }

  spot_light_shadow_map_sampler_.reset();
  spot_light_shadow_map_view_.reset();
  spot_light_shadow_map_.reset();

  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = Platform::Settings::spot_light_shadow_map_resolution;
    image_info.extent.height = Platform::Settings::spot_light_shadow_map_resolution;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = Platform::Constants::shadow_map;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    spot_light_shadow_map_ = std::make_shared<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = spot_light_shadow_map_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = Platform::Constants::shadow_map;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    spot_light_shadow_map_view_ = std::make_shared<ImageView>(view_info);

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    spot_light_shadow_map_sampler_ = std::make_shared<Sampler>(sampler_info);
  }

  {
    VkDescriptorImageInfo image_info{};
    auto render_layer = Application::GetLayer<RenderLayer>();
    image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    image_info.imageView = directional_light_shadow_map_view_->GetVkImageView();
    image_info.sampler = directional_shadow_map_sampler_->GetVkSampler();
    lighting_descriptor_set->UpdateImageDescriptorBinding(15, image_info);
    image_info.imageView = point_light_shadow_map_view_->GetVkImageView();
    image_info.sampler = point_light_shadow_map_sampler_->GetVkSampler();
    lighting_descriptor_set->UpdateImageDescriptorBinding(16, image_info);
    image_info.imageView = spot_light_shadow_map_view_->GetVkImageView();
    image_info.sampler = spot_light_shadow_map_sampler_->GetVkSampler();
    lighting_descriptor_set->UpdateImageDescriptorBinding(17, image_info);
  }
}

VkRenderingAttachmentInfo Lighting::GetDirectionalLightDepthAttachmentInfo(const VkAttachmentLoadOp load_op,
                                                                           const VkAttachmentStoreOp store_op) const {
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  attachment.clearValue.depthStencil.depth = 1.0f;
  attachment.imageView = directional_light_shadow_map_view_->GetVkImageView();
  return attachment;
}

VkRenderingAttachmentInfo Lighting::GetPointLightDepthAttachmentInfo(const VkAttachmentLoadOp load_op,
                                                                     const VkAttachmentStoreOp store_op) const {
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  attachment.clearValue.depthStencil.depth = 1.0f;
  attachment.imageView = point_light_shadow_map_view_->GetVkImageView();
  return attachment;
}

VkRenderingAttachmentInfo Lighting::GetLayeredDirectionalLightDepthAttachmentInfo(
    const uint32_t split, const VkAttachmentLoadOp load_op, const VkAttachmentStoreOp store_op) const {
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;
  attachment.clearValue.depthStencil.depth = 1.0f;
  attachment.imageView = directional_light_shadow_map_layered_views_[split]->GetVkImageView();
  return attachment;
}

VkRenderingAttachmentInfo Lighting::GetLayeredPointLightDepthAttachmentInfo(const uint32_t face,
                                                                            const VkAttachmentLoadOp load_op,
                                                                            const VkAttachmentStoreOp store_op) const {
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  attachment.clearValue.depthStencil.depth = 1.0f;
  attachment.imageView = point_light_shadow_map_layered_views_[face]->GetVkImageView();
  return attachment;
}

VkRenderingAttachmentInfo Lighting::GetSpotLightDepthAttachmentInfo(const VkAttachmentLoadOp load_op,
                                                                    const VkAttachmentStoreOp store_op) const {
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  attachment.clearValue.depthStencil.depth = 1.0f;
  attachment.imageView = spot_light_shadow_map_view_->GetVkImageView();
  return attachment;
}
