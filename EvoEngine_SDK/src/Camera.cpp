#include "Camera.hpp"
#include <algorithm>
#include <cctype>
#include <unordered_map>
#include "Application.hpp"
#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

namespace {
std::unordered_map<uint64_t, glm::mat4> previous_camera_projection_views;
struct CameraJitterState {
  glm::vec2 current = {};
  glm::vec2 previous = {};
  uint32_t frame_index = 0;
};
std::unordered_map<uint64_t, CameraJitterState> camera_jitter_states;

float Halton(uint32_t index, const uint32_t base) {
  float result = 0.0f;
  float fraction = 1.0f / static_cast<float>(base);
  while (index > 0) {
    result += static_cast<float>(index % base) * fraction;
    index /= base;
    fraction /= static_cast<float>(base);
  }
  return result;
}

std::string NormalizeRenderModeName(std::string value) {
  value.erase(std::remove_if(value.begin(), value.end(),
                             [](const char character) {
                               return character == '-' || character == '_' ||
                                      std::isspace(static_cast<unsigned char>(character));
                             }),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  return value;
}

void ReportCameraRenderModeFallback(const Camera::CameraRenderMode requested_mode,
                                    const Camera::CameraRenderMode fallback_mode) {
  static bool ray_tracing_fallback_reported = false;
  static bool ray_query_fallback_reported = false;
  if (requested_mode == fallback_mode) {
    return;
  }
  if (requested_mode == Camera::CameraRenderMode::RayTracing && !ray_tracing_fallback_reported) {
    EVOENGINE_WARNING(std::string("Camera render mode RayTracing is unavailable; falling back to ") +
                      Camera::GetCameraRenderModeName(fallback_mode) + ".")
    ray_tracing_fallback_reported = true;
  }
  if (requested_mode == Camera::CameraRenderMode::RayQuery && !ray_query_fallback_reported) {
    EVOENGINE_WARNING(std::string("Camera render mode RayQuery is unavailable; falling back to ") +
                      Camera::GetCameraRenderModeName(fallback_mode) + ".")
    ray_query_fallback_reported = true;
  }
}

void ReportShaderExecutionReorderingFallback(const CameraSettings::ShaderExecutionReorderingMode requested_mode) {
  static bool unsupported_reported = false;
  if (requested_mode == CameraSettings::ShaderExecutionReorderingMode::Disabled ||
      Platform::ShaderExecutionReorderingEnabled() || unsupported_reported) {
    return;
  }
  EVOENGINE_WARNING("Shader Execution Reordering is unavailable; using standard ray tracing scheduling.")
  unsupported_reported = true;
}

std::shared_ptr<ImageView> CreateGBufferImageView(const std::shared_ptr<Image>& image, const VkFormat format,
                                                  const VkComponentMapping components = VkComponentMapping{}) {
  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = format;
  view_info.components = components;
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  return std::make_unique<ImageView>(view_info);
}

void CreateGBufferAttachment(const VkExtent3D extent, const VkFormat format, std::shared_ptr<Image>& image,
                             std::shared_ptr<ImageView>& view) {
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent = extent;
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = format;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                     VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  image = std::make_unique<Image>(image_info);
  view = CreateGBufferImageView(image, format);
}

std::shared_ptr<Sampler> CreateGBufferSampler() {
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_NEAREST;
  sampler_info.minFilter = VK_FILTER_NEAREST;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
  sampler_info.anisotropyEnable = VK_TRUE;
  sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
  sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
  return std::make_unique<Sampler>(sampler_info);
}

void AppendGBufferAttachmentInfo(std::vector<VkRenderingAttachmentInfo>& attachment_infos,
                                 VkRenderingAttachmentInfo attachment, const std::shared_ptr<ImageView>& view) {
  attachment.clearValue = {0, 0, 0, 0};
  attachment.imageView = view->GetVkImageView();
  attachment_infos.push_back(attachment);
}
}  // namespace

const std::vector<std::string>& Camera::GetCameraRenderModeNames() {
  static const std::vector<std::string> render_mode_names{"Rasterization", "RayTracing", "RayQuery"};
  return render_mode_names;
}

const char* Camera::GetCameraRenderModeName(const CameraRenderMode mode) {
  const auto index = static_cast<uint32_t>(NormalizeCameraRenderMode(static_cast<uint32_t>(mode)));
  return GetCameraRenderModeNames()[index].c_str();
}

const std::vector<std::string>& Camera::GetShaderExecutionReorderingModeNames() {
  static const std::vector<std::string> mode_names{"Disabled", "Automatic", "Enabled"};
  return mode_names;
}

const char* Camera::GetShaderExecutionReorderingModeName(const CameraSettings::ShaderExecutionReorderingMode mode) {
  const auto index = static_cast<uint32_t>(NormalizeShaderExecutionReorderingMode(static_cast<uint32_t>(mode)));
  return GetShaderExecutionReorderingModeNames()[index].c_str();
}

CameraSettings::ShaderExecutionReorderingMode Camera::ParseShaderExecutionReorderingMode(
    const std::string& value, const CameraSettings::ShaderExecutionReorderingMode fallback) {
  const auto normalized = NormalizeRenderModeName(value);
  if (normalized == "0" || normalized == "off" || normalized == "disabled" || normalized == "disable") {
    return CameraSettings::ShaderExecutionReorderingMode::Disabled;
  }
  if (normalized == "1" || normalized == "auto" || normalized == "automatic") {
    return CameraSettings::ShaderExecutionReorderingMode::Automatic;
  }
  if (normalized == "2" || normalized == "on" || normalized == "enabled" || normalized == "enable") {
    return CameraSettings::ShaderExecutionReorderingMode::Enabled;
  }
  return fallback;
}

CameraSettings::ShaderExecutionReorderingMode Camera::NormalizeShaderExecutionReorderingMode(const uint32_t mode) {
  if (mode >= kShaderExecutionReorderingModeCount) {
    return CameraSettings::ShaderExecutionReorderingMode::Disabled;
  }
  return static_cast<CameraSettings::ShaderExecutionReorderingMode>(mode);
}

bool Camera::ResolveShaderExecutionReorderingEnabled(
    const CameraSettings::ShaderExecutionReorderingMode requested_mode) {
  if (requested_mode == CameraSettings::ShaderExecutionReorderingMode::Disabled) {
    return false;
  }
  ReportShaderExecutionReorderingFallback(requested_mode);
  return Platform::ShaderExecutionReorderingEnabled();
}

Camera::CameraRenderMode Camera::ParseCameraRenderMode(const std::string& value, const CameraRenderMode fallback) {
  const auto normalized = NormalizeRenderModeName(value);
  if (normalized == "0" || normalized == "raster" || normalized == "rasterization") {
    return CameraRenderMode::Rasterization;
  }
  if (normalized == "1" || normalized == "raytracing" || normalized == "pathtracing" || normalized == "pathtrace") {
    return CameraRenderMode::RayTracing;
  }
  if (normalized == "2" || normalized == "rayquery") {
    return CameraRenderMode::RayQuery;
  }
  return fallback;
}

Camera::CameraRenderMode Camera::NormalizeCameraRenderMode(const uint32_t mode) {
  if (mode >= kCameraRenderModeCount) {
    return CameraRenderMode::Rasterization;
  }
  return static_cast<CameraRenderMode>(mode);
}

bool Camera::IsRayCameraRenderMode(const CameraRenderMode mode) {
  return mode == CameraRenderMode::RayTracing || mode == CameraRenderMode::RayQuery;
}

Camera::CameraRenderMode Camera::ResolveCameraRenderMode(const CameraRenderMode requested_mode) {
  auto fallback_mode = requested_mode;
  if (requested_mode == CameraRenderMode::RayTracing && !Platform::RayTracingEnabled()) {
    fallback_mode = CameraRenderMode::Rasterization;
  } else if (requested_mode == CameraRenderMode::RayQuery && !Platform::RayQueryEnabled()) {
    fallback_mode = Platform::RayTracingEnabled() ? CameraRenderMode::RayTracing : CameraRenderMode::Rasterization;
  }
  ReportCameraRenderModeFallback(requested_mode, fallback_mode);
  return fallback_mode;
}

glm::vec3 CameraInfoBlock::Project(const glm::vec3& position) const {
  return projection * view * glm::vec4(position, 1.0f);
}

glm::vec3 CameraInfoBlock::UnProject(const glm::vec3& position) const {
  const glm::mat4 inverse = glm::inverse(projection * view);
  auto start = glm::vec4(position, 1.0f);
  start = inverse * start;
  return start / start.w;
}
bool CameraInfoBlock::operator!=(const CameraInfoBlock& other) const {
  const auto unjittered_projection_view = [](const CameraInfoBlock& block) {
    auto projection = block.projection;
    projection[2][0] -= block.jitter.x;
    projection[2][1] -= block.jitter.y;
    return projection * block.view;
  };
  if (unjittered_projection_view(*this) != unjittered_projection_view(other))
    return true;
  if (clear_color != other.clear_color)
    return true;
  if (resolution != other.resolution)
    return true;
  if (fade_ratio != other.fade_ratio)
    return true;
  if (fade_factor != other.fade_factor)
    return true;
  if (skybox_texture_index != other.skybox_texture_index)
    return true;
  if (environmental_prefiltered_index != other.environmental_prefiltered_index)
    return true;
  if (environmental_irradiance_texture_index != other.environmental_irradiance_texture_index)
    return true;
  if (camera_use_clear_color != other.camera_use_clear_color)
    return true;
  if (gamma != other.gamma)
    return true;
  if (sample_size != other.sample_size)
    return true;
  if (bounce != other.bounce)
    return true;
  if (firefly_clamp_enabled != other.firefly_clamp_enabled)
    return true;
  if (firefly_clamp_threshold != other.firefly_clamp_threshold)
    return true;
  if (auto_spp_enabled != other.auto_spp_enabled)
    return true;
  if (auto_spp_min_samples != other.auto_spp_min_samples)
    return true;
  if (auto_spp_max_samples != other.auto_spp_max_samples)
    return true;
  if (auto_spp_convergence_threshold != other.auto_spp_convergence_threshold)
    return true;
  return false;
}

void Camera::UpdateGBuffer() {
  if (!Platform::Initialized())
    return;
  g_buffer_base_color_ao_view_.reset();
  g_buffer_normal_roughness_view_.reset();
  g_buffer_pbr_flags_view_.reset();
  g_buffer_emissive_view_.reset();
  g_buffer_utility_view_.reset();

  g_buffer_base_color_ao_.reset();
  g_buffer_normal_roughness_.reset();
  g_buffer_pbr_flags_.reset();
  g_buffer_emissive_.reset();
  g_buffer_utility_.reset();

  g_buffer_sampler_ = CreateGBufferSampler();
  const auto g_buffer_attribute_format = Platform::Constants::g_buffer_attribute;
  CreateGBufferAttachment(render_texture_->GetExtent(), g_buffer_attribute_format, g_buffer_base_color_ao_,
                          g_buffer_base_color_ao_view_);
  CreateGBufferAttachment(render_texture_->GetExtent(), g_buffer_attribute_format, g_buffer_normal_roughness_,
                          g_buffer_normal_roughness_view_);
  CreateGBufferAttachment(render_texture_->GetExtent(), g_buffer_attribute_format, g_buffer_pbr_flags_,
                          g_buffer_pbr_flags_view_);
  CreateGBufferAttachment(render_texture_->GetExtent(), g_buffer_attribute_format, g_buffer_emissive_,
                          g_buffer_emissive_view_);
  CreateGBufferAttachment(render_texture_->GetExtent(), Platform::Constants::g_buffer_utility, g_buffer_utility_,
                          g_buffer_utility_view_);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  EditorLayer::UpdateTextureId(g_buffer_base_color_ao_im_texture_id_, g_buffer_sampler_->GetVkSampler(),
                               g_buffer_base_color_ao_view_->GetVkImageView(), g_buffer_base_color_ao_->GetLayout());
  EditorLayer::UpdateTextureId(g_buffer_normal_roughness_im_texture_id_, g_buffer_sampler_->GetVkSampler(),
                               g_buffer_normal_roughness_view_->GetVkImageView(),
                               g_buffer_normal_roughness_->GetLayout());
  EditorLayer::UpdateTextureId(g_buffer_pbr_flags_im_texture_id_, g_buffer_sampler_->GetVkSampler(),
                               g_buffer_pbr_flags_view_->GetVkImageView(), g_buffer_pbr_flags_->GetLayout());
  EditorLayer::UpdateTextureId(g_buffer_emissive_im_texture_id_, g_buffer_sampler_->GetVkSampler(),
                               g_buffer_emissive_view_->GetVkImageView(), g_buffer_emissive_->GetLayout());
  EditorLayer::UpdateTextureId(g_buffer_utility_im_texture_id_, g_buffer_sampler_->GetVkSampler(),
                               g_buffer_utility_view_->GetVkImageView(), g_buffer_utility_->GetLayout());
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    image_info.imageView = render_texture_->GetDepthImageView()->GetVkImageView();
    image_info.sampler = render_texture_->GetDepthSampler()->GetVkSampler();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(17, image_info);
    image_info.sampler = g_buffer_sampler_->GetVkSampler();
    image_info.imageView = g_buffer_base_color_ao_view_->GetVkImageView();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(20, image_info);
    image_info.imageView = g_buffer_normal_roughness_view_->GetVkImageView();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(21, image_info);
    image_info.imageView = g_buffer_pbr_flags_view_->GetVkImageView();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(22, image_info);
    image_info.imageView = g_buffer_emissive_view_->GetVkImageView();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(23, image_info);
    image_info.imageView = g_buffer_utility_view_->GetVkImageView();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(24, image_info);
  }
}

void Camera::TransitGBufferImageLayout(const VkCommandBuffer vk_command_buffer, VkImageLayout target_layout) const {
  g_buffer_base_color_ao_->TransitImageLayout(vk_command_buffer, target_layout);
  g_buffer_normal_roughness_->TransitImageLayout(vk_command_buffer, target_layout);
  g_buffer_pbr_flags_->TransitImageLayout(vk_command_buffer, target_layout);
  g_buffer_emissive_->TransitImageLayout(vk_command_buffer, target_layout);
  g_buffer_utility_->TransitImageLayout(vk_command_buffer, target_layout);
}
void Camera::UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform) {
  const auto rotation = global_transform.GetRotation();
  const auto position = global_transform.GetPosition();
  const glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  const glm::vec3 up = rotation * glm::vec3(0, 1, 0);
  const auto ratio = GetSizeRatio();

  camera_info_block.projection = glm::perspective(glm::radians(camera_settings.fov * 0.5f), ratio,
                                                  camera_settings.near_distance, camera_settings.far_distance);
  auto& jitter_state = camera_jitter_states[GetHandle().GetValue()];
  jitter_state.previous = jitter_state.current;
  jitter_state.current = {};
  const auto post_processing_stack = post_processing_stack_ref.Get<PostProcessingStack>();
  const bool taa_enabled = camera_render_mode == CameraRenderMode::Rasterization && post_processing_stack &&
                           post_processing_stack->enable_temporal_anti_aliasing &&
                           post_processing_stack->temporal_anti_aliasing;
  if (taa_enabled && size_.x != 0 && size_.y != 0) {
    const uint32_t sequence_index = jitter_state.frame_index % 16u + 1u;
    jitter_state.current = glm::vec2(Halton(sequence_index, 2u) - 0.5f, Halton(sequence_index, 3u) - 0.5f);
    jitter_state.current *= 2.0f / glm::vec2(size_);
    camera_info_block.projection[2][0] += jitter_state.current.x;
    camera_info_block.projection[2][1] += jitter_state.current.y;
    ++jitter_state.frame_index;
  } else {
    jitter_state.frame_index = 0;
  }
  camera_info_block.view = glm::lookAt(position, position + front, up);
  camera_info_block.projection_view = camera_info_block.projection * camera_info_block.view;
  camera_info_block.inverse_projection = glm::inverse(camera_info_block.projection);
  camera_info_block.inverse_view = glm::inverse(camera_info_block.view);
  camera_info_block.inverse_projection_view = glm::inverse(camera_info_block.projection * camera_info_block.view);
  const auto previous_projection_view = previous_camera_projection_views.find(GetHandle().GetValue());
  camera_info_block.previous_projection_view = previous_projection_view == previous_camera_projection_views.end()
                                                   ? camera_info_block.projection_view
                                                   : previous_projection_view->second;
  previous_camera_projection_views[GetHandle().GetValue()] = camera_info_block.projection_view;
  camera_info_block.clear_color =
      glm::vec4(glm::vec3(camera_settings.clear_color), camera_settings.background_intensity);
  camera_info_block.jitter = glm::vec4(jitter_state.current, jitter_state.previous);
  camera_info_block.resolution = size_;
  camera_info_block.fade_factor = camera_settings.fade_factor;
  camera_info_block.fade_ratio = camera_settings.fade_ratio;
  if (camera_settings.use_clear_color) {
    camera_info_block.camera_use_clear_color = 1;
  } else {
    camera_info_block.camera_use_clear_color = 0;
  }
  if (const auto camera_skybox = skybox.Get<Cubemap>()) {
    camera_info_block.skybox_texture_index = camera_skybox->GetTextureStorageIndex();
  } else {
    const auto default_cubemap = Resources::GetInstance().GetDefaultSkybox();
    camera_info_block.skybox_texture_index = default_cubemap->GetTextureStorageIndex();
  }
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    const auto camera_position = global_transform.GetPosition();
    auto scene = GetScene();
    if (!scene) {
      scene = ApplicationContext::Get().GetActiveScene();
    }
    std::shared_ptr<LightProbe> light_probe;
    std::shared_ptr<ReflectionProbe> reflection_probe;
    if (scene) {
      light_probe = scene->environment.GetLightProbe(camera_position);
      reflection_probe = scene->environment.GetReflectionProbe(camera_position);
    }
    if (!light_probe) {
      light_probe = Resources::GetInstance().GetDefaultEnvironmentalMap()->light_probe.Get<LightProbe>();
    }
    camera_info_block.environmental_irradiance_texture_index = light_probe->cubemap_->GetTextureStorageIndex();
    if (!reflection_probe) {
      reflection_probe = Resources::GetInstance().GetDefaultEnvironmentalMap()->reflection_probe.Get<ReflectionProbe>();
    }
    camera_info_block.environmental_prefiltered_index = reflection_probe->cubemap_->GetTextureStorageIndex();
  }

  camera_info_block.sample_size = camera_settings.sample_size;
  camera_info_block.bounce = camera_settings.bounce;
  camera_info_block.gamma = camera_settings.gamma;
  camera_info_block.firefly_clamp_enabled = camera_settings.firefly_clamp_enabled ? 1u : 0u;
  camera_info_block.firefly_clamp_threshold = camera_settings.firefly_clamp_threshold;
  const auto auto_spp_min_samples = static_cast<uint32_t>(glm::max(camera_settings.auto_spp_min_samples, 1));
  const auto auto_spp_max_samples =
      static_cast<uint32_t>(glm::max(camera_settings.auto_spp_max_samples, static_cast<int>(auto_spp_min_samples)));
  camera_info_block.auto_spp_enabled = camera_settings.auto_spp_enabled ? 1u : 0u;
  camera_info_block.auto_spp_min_samples = auto_spp_min_samples;
  camera_info_block.auto_spp_max_samples = auto_spp_max_samples;
  camera_info_block.auto_spp_convergence_threshold = glm::max(camera_settings.auto_spp_convergence_threshold, 0.0f);
}

void Camera::AppendGBufferColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos,
                                               const VkAttachmentLoadOp load_op,
                                               const VkAttachmentStoreOp store_op) const {
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_base_color_ao_view_);
  AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_normal_roughness_view_);
  AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_pbr_flags_view_);
  AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_emissive_view_);
  AppendGBufferAttachmentInfo(attachment_infos, attachment, g_buffer_utility_view_);
}

float Camera::GetSizeRatio() const {
  if (size_.x == 0 || size_.y == 0)
    return 0;
  return static_cast<float>(size_.x) / static_cast<float>(size_.y);
}

const std::shared_ptr<RenderTexture>& Camera::GetRenderTexture() const {
  return render_texture_;
}

glm::uvec2 Camera::GetSize() const {
  return size_;
}

uint32_t Camera::GetFrameCount() const {
  return frame_count_;
}

void Camera::Resize(const glm::uvec2& size) {
  if (size.x == 0 || size.y == 0)
    return;
  if (size.x > 16384 || size.y >= 16384)
    return;
  if (size_ == size)
    return;
  size_ = size;
  ResetFrameCount();
  if (render_texture_) {
    render_texture_->Resize({size_.x, size_.y, 1});
    UpdateGBuffer();
  }
}

void Camera::OnCreate() {
  size_ = glm::uvec2(1, 1);
  frame_count_ = 0;
  camera_settings = {};
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer || !Platform::Initialized()) {
    return;
  }
  RenderTextureCreateInfo render_texture_create_info{};
  render_texture_create_info.extent.width = size_.x;
  render_texture_create_info.extent.height = size_.y;
  render_texture_create_info.extent.depth = 1;
  render_texture_ = std::make_unique<RenderTexture>(render_texture_create_info);

  g_buffer_descriptor_set_ = std::make_shared<DescriptorSet>(render_layer->GetCameraGBufferDescriptorSetLayout());

  post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  UpdateGBuffer();
}

bool Camera::Rendered() const {
  return rendered_;
}

void Camera::SetRequireRendering(const bool value) {
  require_rendering_ = require_rendering_ || value;
}

void Camera::CalculatePlanes(std::vector<Plane>& planes, const glm::mat4& projection, const glm::mat4& view) {
  glm::mat4 combo_matrix = projection * glm::transpose(view);
  planes[0].a = combo_matrix[3][0] + combo_matrix[0][0];
  planes[0].b = combo_matrix[3][1] + combo_matrix[0][1];
  planes[0].c = combo_matrix[3][2] + combo_matrix[0][2];
  planes[0].d = combo_matrix[3][3] + combo_matrix[0][3];

  planes[1].a = combo_matrix[3][0] - combo_matrix[0][0];
  planes[1].b = combo_matrix[3][1] - combo_matrix[0][1];
  planes[1].c = combo_matrix[3][2] - combo_matrix[0][2];
  planes[1].d = combo_matrix[3][3] - combo_matrix[0][3];

  planes[2].a = combo_matrix[3][0] - combo_matrix[1][0];
  planes[2].b = combo_matrix[3][1] - combo_matrix[1][1];
  planes[2].c = combo_matrix[3][2] - combo_matrix[1][2];
  planes[2].d = combo_matrix[3][3] - combo_matrix[1][3];

  planes[3].a = combo_matrix[3][0] + combo_matrix[1][0];
  planes[3].b = combo_matrix[3][1] + combo_matrix[1][1];
  planes[3].c = combo_matrix[3][2] + combo_matrix[1][2];
  planes[3].d = combo_matrix[3][3] + combo_matrix[1][3];

  planes[4].a = combo_matrix[3][0] + combo_matrix[2][0];
  planes[4].b = combo_matrix[3][1] + combo_matrix[2][1];
  planes[4].c = combo_matrix[3][2] + combo_matrix[2][2];
  planes[4].d = combo_matrix[3][3] + combo_matrix[2][3];

  planes[5].a = combo_matrix[3][0] - combo_matrix[2][0];
  planes[5].b = combo_matrix[3][1] - combo_matrix[2][1];
  planes[5].c = combo_matrix[3][2] - combo_matrix[2][2];
  planes[5].d = combo_matrix[3][3] - combo_matrix[2][3];

  planes[0].Normalize();
  planes[1].Normalize();
  planes[2].Normalize();
  planes[3].Normalize();
  planes[4].Normalize();
  planes[5].Normalize();
}

void Camera::CalculateFrustumPoints(const std::shared_ptr<Camera>& camera_component, float near_plane, float far_plane,
                                    glm::vec3 camera_pos, glm::quat camera_rot, glm::vec3* points) {
  const glm::vec3 front = camera_rot * glm::vec3(0, 0, -1);
  const glm::vec3 right = camera_rot * glm::vec3(1, 0, 0);
  const glm::vec3 up = camera_rot * glm::vec3(0, 1, 0);
  const glm::vec3 near_center = front * near_plane;
  const glm::vec3 far_center = front * far_plane;

  const float e = tanf(glm::radians(camera_component->camera_settings.fov * 0.5f));
  const float near_ext_y = e * near_plane;
  const float near_ext_x = near_ext_y * camera_component->GetSizeRatio();
  const float far_ext_y = e * far_plane;
  const float far_ext_x = far_ext_y * camera_component->GetSizeRatio();

  points[0] = camera_pos + near_center - right * near_ext_x - up * near_ext_y;
  points[1] = camera_pos + near_center - right * near_ext_x + up * near_ext_y;
  points[2] = camera_pos + near_center + right * near_ext_x + up * near_ext_y;
  points[3] = camera_pos + near_center + right * near_ext_x - up * near_ext_y;
  points[4] = camera_pos + far_center - right * far_ext_x - up * far_ext_y;
  points[5] = camera_pos + far_center - right * far_ext_x + up * far_ext_y;
  points[6] = camera_pos + far_center + right * far_ext_x + up * far_ext_y;
  points[7] = camera_pos + far_center + right * far_ext_x - up * far_ext_y;
}

glm::quat Camera::ProcessMouseMovement(float yaw_angle, float pitch_angle, bool constrain_pitch) {
  // Make sure that when pitch is out of bounds, screen doesn't get flipped
  if (constrain_pitch) {
    if (pitch_angle > 89.0f)
      pitch_angle = 89.0f;
    if (pitch_angle < -89.0f)
      pitch_angle = -89.0f;
  }

  glm::vec3 front;
  front.x = cos(glm::radians(yaw_angle)) * cos(glm::radians(pitch_angle));
  front.y = sin(glm::radians(pitch_angle));
  front.z = sin(glm::radians(yaw_angle)) * cos(glm::radians(pitch_angle));
  front = glm::normalize(front);
  const glm::vec3 right = glm::normalize(glm::cross(
      front, glm::vec3(0.0f, 1.0f, 0.0f)));  // Normalize the vectors, because their length gets closer to 0 the more
  // you look up or down which results in slower movement.
  const glm::vec3 up = glm::normalize(glm::cross(right, front));
  return glm::quatLookAt(front, up);
}

void Camera::ReverseAngle(const glm::quat& rotation, float& pitch_angle, float& yaw_angle,
                          const bool& constrain_pitch) {
  const auto angle = glm::degrees(glm::eulerAngles(rotation));
  pitch_angle = angle.x;
  // yawAngle = glm::abs(angle.z) > 90.0f ? 90.0f - angle.y : -90.0f - angle.y;
  glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  front.y = 0;
  yaw_angle = glm::degrees(glm::acos(glm::dot(glm::vec3(0, 0, 1), glm::normalize(front))));
  if (constrain_pitch) {
    if (pitch_angle > 89.0f)
      pitch_angle = 89.0f;
    if (pitch_angle < -89.0f)
      pitch_angle = -89.0f;
  }
}
glm::mat4 Camera::GetProjection() const {
  return glm::perspective(glm::radians(camera_settings.fov * 0.5f), GetSizeRatio(), camera_settings.near_distance,
                          camera_settings.far_distance);
}

glm::vec3 Camera::GetMouseWorldPoint(GlobalTransform& ltw, glm::vec2 mouse_position) const {
  const float half_x = static_cast<float>(size_.x) / 2.0f;
  const float half_y = static_cast<float>(size_.y) / 2.0f;
  const auto start =
      glm::vec4(-1.0f * (mouse_position.x - half_x) / half_x, -1.0f * (mouse_position.y - half_y) / half_y, 0.0f, 1.0f);
  return start / start.w;
}

Ray Camera::ScreenPointToRay(GlobalTransform& ltw, glm::vec2 mouse_position) const {
  const auto position = ltw.GetPosition();
  const auto rotation = ltw.GetRotation();
  const glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  const glm::vec3 up = rotation * glm::vec3(0, 1, 0);
  const auto view = glm::lookAt(position, position + front, up);
  const glm::mat4 inv = glm::inverse(GetProjection() * view);
  const float half_x = static_cast<float>(size_.x) / 2.0f;
  const float half_y = static_cast<float>(size_.y) / 2.0f;
  const auto real_x = (mouse_position.x - half_x) / half_x;
  const auto real_y = (mouse_position.y - half_y) / half_y;
  if (glm::abs(real_x) > 1.0f || glm::abs(real_y) > 1.0f)
    return {glm::vec3(FLT_MAX), glm::vec3(FLT_MAX)};
  auto start = glm::vec4(real_x, -1 * real_y, -1, 1.0);
  auto end = glm::vec4(real_x, -1.0f * real_y, 1.0f, 1.0f);
  start = inv * start;
  end = inv * end;
  start /= start.w;
  end /= end.w;
  const glm::vec3 dir = glm::normalize(glm::vec3(end - start));
  return {glm::vec3(ltw.value[3]) + camera_settings.near_distance * dir,
          glm::vec3(ltw.value[3]) + camera_settings.far_distance * dir};
}

void Camera::OnDestroy() {
  post_processing_stack_ref.Clear();
  skybox.Clear();
}

void Camera::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(skybox);
  list.push_back(post_processing_stack_ref);
}

const std::shared_ptr<DescriptorSet>& Camera::GetGBufferDescriptorSet() const {
  return g_buffer_descriptor_set_;
}

const std::shared_ptr<Image>& Camera::GetGBufferUtilityImage() const {
  return g_buffer_utility_;
}

ImTextureID Camera::GetGBufferBaseColorAoImTextureId() const {
  return g_buffer_base_color_ao_im_texture_id_;
}

ImTextureID Camera::GetGBufferNormalRoughnessImTextureId() const {
  return g_buffer_normal_roughness_im_texture_id_;
}

ImTextureID Camera::GetGBufferPbrFlagsImTextureId() const {
  return g_buffer_pbr_flags_im_texture_id_;
}

ImTextureID Camera::GetGBufferEmissiveImTextureId() const {
  return g_buffer_emissive_im_texture_id_;
}

ImTextureID Camera::GetGBufferUtilityImTextureId() const {
  return g_buffer_utility_im_texture_id_;
}

void Camera::SetRendered() {
  rendered_ = true;
}
void Camera::ResetRenderState() {
  rendered_ = false;
  require_rendering_ = false;
}
void Camera::ResetFrameCount() {
  frame_count_ = 0;
}
