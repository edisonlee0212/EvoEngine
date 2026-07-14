#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderGraph.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
#include "SmaaAreaTex.h"
#include "SmaaSearchTex.h"
#include "WindowLayer.hpp"

#include <type_traits>
using namespace evo_engine;

namespace {
template <typename T>
uint64_t VulkanHandleIdentity(const T handle) {
  if constexpr (std::is_pointer_v<T>) {
    return reinterpret_cast<uintptr_t>(handle);
  } else {
    return static_cast<uint64_t>(handle);
  }
}

std::shared_ptr<Image> CreateSmaaLookupImage(const VkFormat format, const uint32_t width, const uint32_t height) {
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent = {width, height, 1};
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = format;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  return std::make_shared<Image>(image_info);
}

std::shared_ptr<ImageView> CreateSmaaLookupView(const std::shared_ptr<Image>& image) {
  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = image->GetFormat();
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  return std::make_shared<ImageView>(view_info, image);
}

std::shared_ptr<Sampler> CreateSmaaSampler() {
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.anisotropyEnable = VK_FALSE;
  sampler_info.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  sampler_info.minLod = 0.0f;
  sampler_info.maxLod = 1.0f;
  return std::make_shared<Sampler>(sampler_info);
}

void UploadSmaaLookupImage(const std::shared_ptr<Image>& image, const unsigned char* bytes, const size_t byte_size) {
  Buffer staging_buffer(byte_size, false);
  staging_buffer.UploadData(byte_size, bytes);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    image->CopyFromBuffer(vk_command_buffer, staging_buffer.GetVkBuffer());
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

const char* GetSmaaPresetDefine(const size_t preset_index) {
  constexpr const char* defines[] = {"SMAA_PRESET_LOW", "SMAA_PRESET_MEDIUM", "SMAA_PRESET_HIGH", "SMAA_PRESET_ULTRA"};
  return defines[glm::min(preset_index, std::size(defines) - 1)];
}
}  // namespace

std::shared_ptr<DescriptorSet> PerFrameDescriptorSet::GetOrCreate(
    const std::shared_ptr<DescriptorSetLayout>& layout) const {
  slots_.resize(Platform::GetMaxFramesInFlight());
  auto& slot = slots_.at(Platform::GetCurrentFrameIndex());
  const auto frame_count = Platform::GetFrameCount();
  if (!slot.recorded || slot.frame_count != frame_count) {
    slot.frame_count = frame_count;
    slot.recorded = true;
    slot.duplicate_descriptor_sets.clear();
    if (!slot.descriptor_set) {
      slot.descriptor_set = std::make_shared<DescriptorSet>(layout);
    }
    return slot.descriptor_set;
  }
  return slot.duplicate_descriptor_sets.emplace_back(std::make_shared<DescriptorSet>(layout));
}

void PerFrameDescriptorSet::Retain(RenderGraphTransientResourceStore& transient_resources) const {
  for (const auto& slot : slots_) {
    transient_resources.RetainDescriptorSet(slot.descriptor_set);
    for (const auto& descriptor_set : slot.duplicate_descriptor_sets) {
      transient_resources.RetainDescriptorSet(descriptor_set);
    }
  }
}

void PerFrameDescriptorSet::AppendIdentities(std::vector<uint64_t>& identities) const {
  for (const auto& slot : slots_) {
    if (slot.descriptor_set) {
      identities.emplace_back(VulkanHandleIdentity(slot.descriptor_set->GetVkDescriptorSet()));
    }
    for (const auto& descriptor_set : slot.duplicate_descriptor_sets) {
      if (descriptor_set) {
        identities.emplace_back(VulkanHandleIdentity(descriptor_set->GetVkDescriptorSet()));
      }
    }
  }
}

void PerFrameDescriptorSet::Reset() {
  slots_.clear();
}

std::vector<std::shared_ptr<DescriptorSet>>& PerFrameDescriptorSetList::Get() {
  slots_.resize(Platform::GetMaxFramesInFlight());
  auto& slot = slots_.at(Platform::GetCurrentFrameIndex());
  const auto frame_count = Platform::GetFrameCount();
  if (!slot.recorded || slot.frame_count != frame_count) {
    slot.frame_count = frame_count;
    slot.recorded = true;
    slot.duplicate_descriptor_set_lists.clear();
    return slot.descriptor_sets;
  }
  return slot.duplicate_descriptor_set_lists.emplace_back();
}

void PerFrameDescriptorSetList::Retain(RenderGraphTransientResourceStore& transient_resources) const {
  for (const auto& slot : slots_) {
    for (const auto& descriptor_set : slot.descriptor_sets) {
      transient_resources.RetainDescriptorSet(descriptor_set);
    }
    for (const auto& descriptor_sets : slot.duplicate_descriptor_set_lists) {
      for (const auto& descriptor_set : descriptor_sets) {
        transient_resources.RetainDescriptorSet(descriptor_set);
      }
    }
  }
}

void PerFrameDescriptorSetList::AppendIdentities(std::vector<uint64_t>& identities) const {
  for (const auto& slot : slots_) {
    for (const auto& descriptor_set : slot.descriptor_sets) {
      if (descriptor_set) {
        identities.emplace_back(VulkanHandleIdentity(descriptor_set->GetVkDescriptorSet()));
      }
    }
    for (const auto& descriptor_sets : slot.duplicate_descriptor_set_lists) {
      for (const auto& descriptor_set : descriptor_sets) {
        if (descriptor_set) {
          identities.emplace_back(VulkanHandleIdentity(descriptor_set->GetVkDescriptorSet()));
        }
      }
    }
  }
}

void PerFrameDescriptorSetList::Reset() {
  slots_.clear();
}

void PostProcessingCameraResources::ResetTemporalState() {
  anti_aliasing.history.valid = false;
  anti_aliasing.history.frame_index = 0;
  anti_aliasing.history.last_processed_frame = 0;
  current_jitter = {};
  previous_jitter = {};
  jitter_frame_index = 0;
  previous_matrices_valid = false;
  tone_mapping.auto_exposure_time_initialized = false;
  tone_mapping.luminance_reset_pending = true;
  tone_mapping.last_auto_exposure_time = 0.0;
  ++tone_mapping.auto_exposure_reset_count;
  ++temporal_reset_count;
}

void PostProcessingCameraResources::Retain(RenderGraphTransientResourceStore& transient_resources) const {
  transient_resources.RetainRenderTextureResources(stack.source_color_texture);
  transient_resources.RetainRenderTextureResources(stack.result_texture);
  transient_resources.RetainRenderTextureResources(stack.swap_texture);
  stack.blur_horizontal_descriptor_set.Retain(transient_resources);
  stack.blur_vertical_descriptor_set.Retain(transient_resources);
  ambient_occlusion.blur_horizontal_descriptor_set.Retain(transient_resources);
  ambient_occlusion.blur_vertical_descriptor_set.Retain(transient_resources);
  ambient_occlusion.combine_descriptor_set.Retain(transient_resources);
  ambient_occlusion.geometry_output_descriptor_set.Retain(transient_resources);
  anti_aliasing.copy_descriptor_set.Retain(transient_resources);
  anti_aliasing.resolve_descriptor_set.Retain(transient_resources);
  for (const auto& texture : anti_aliasing.history.textures) {
    transient_resources.RetainRenderTextureResources(texture);
  }
  for (const auto& texture : anti_aliasing.history.depth_textures) {
    transient_resources.RetainRenderTextureResources(texture);
  }
  transient_resources.RetainRenderTextureResources(anti_aliasing.smaa_edges_texture);
  transient_resources.RetainRenderTextureResources(anti_aliasing.smaa_blend_texture);
  anti_aliasing.smaa_prepare_descriptor_set.Retain(transient_resources);
  anti_aliasing.smaa_edge_descriptor_set.Retain(transient_resources);
  anti_aliasing.smaa_weight_descriptor_set.Retain(transient_resources);
  anti_aliasing.smaa_neighborhood_descriptor_set.Retain(transient_resources);
  screen_space_reflection.combine_descriptor_set.Retain(transient_resources);
  screen_space_reflection.reflect_output_descriptor_set.Retain(transient_resources);
  bloom.mix_descriptor_set.Retain(transient_resources);
  bloom.copy_descriptor_set.Retain(transient_resources);
  bloom.downsampling_descriptor_sets.Retain(transient_resources);
  bloom.upsampling_descriptor_sets.Retain(transient_resources);
  tone_mapping.auto_exposure_descriptor_set.Retain(transient_resources);
  transient_resources.RetainBuffer(tone_mapping.histogram_buffer);
  transient_resources.RetainBuffer(tone_mapping.luminance_buffer);
}

void AntiAliasing::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "algorithm" << YAML::Value << static_cast<int32_t>(algorithm);
  out << YAML::Key << "taa" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "preset" << YAML::Value << static_cast<int32_t>(taa.preset);
  out << YAML::Key << "variance_clipping_mode" << YAML::Value << static_cast<int32_t>(taa.variance_clipping_mode);
  out << YAML::Key << "history_color_mode" << YAML::Value << static_cast<int32_t>(taa.history_color_mode);
  out << YAML::Key << "variance_sample_count" << YAML::Value << taa.variance_sample_count;
  out << YAML::Key << "longest_velocity_sample_count" << YAML::Value << taa.longest_velocity_sample_count;
  out << YAML::Key << "use_ycocg" << YAML::Value << taa.use_ycocg;
  out << YAML::Key << "use_neighborhood_sampling" << YAML::Value << taa.use_neighborhood_sampling;
  out << YAML::Key << "use_bicubic_filter" << YAML::Value << taa.use_bicubic_filter;
  out << YAML::Key << "use_longest_velocity" << YAML::Value << taa.use_longest_velocity;
  out << YAML::Key << "use_depth_threshold" << YAML::Value << taa.use_depth_threshold;
  out << YAML::Key << "use_tgsm" << YAML::Value << taa.use_tgsm;
  out << YAML::Key << "use_fp16" << YAML::Value << taa.use_fp16;
  out << YAML::Key << "min_variance_gamma" << YAML::Value << taa.min_variance_gamma;
  out << YAML::Key << "max_variance_gamma" << YAML::Value << taa.max_variance_gamma;
  out << YAML::Key << "velocity_rejection_threshold" << YAML::Value << taa.velocity_rejection_threshold;
  out << YAML::Key << "depth_threshold" << YAML::Value << taa.depth_threshold;
  out << YAML::Key << "sharpen" << YAML::Value << taa.sharpen;
  out << YAML::EndMap;
  out << YAML::Key << "smaa" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "preset" << YAML::Value << static_cast<int32_t>(smaa.preset);
  out << YAML::EndMap;
}

void AntiAliasing::Deserialize(const YAML::Node& in) {
  algorithm = Algorithm::Smaa;
  taa = {};
  smaa = {};
  if (in["algorithm"] && in["algorithm"].as<int32_t>() == static_cast<int32_t>(Algorithm::Taa)) {
    algorithm = Algorithm::Taa;
  }
  if (const auto taa_node = in["taa"]) {
    TaaPreset loaded_preset = TaaPreset::BestQuality;
    if (taa_node["preset"]) {
      const auto value = taa_node["preset"].as<int32_t>();
      if (value >= static_cast<int32_t>(TaaPreset::BestQuality) && value <= static_cast<int32_t>(TaaPreset::Custom)) {
        loaded_preset = static_cast<TaaPreset>(value);
      }
    }
    ApplyTaaPreset(loaded_preset);
    if (taa_node["variance_clipping_mode"])
      taa.variance_clipping_mode = static_cast<VarianceClippingMode>(taa_node["variance_clipping_mode"].as<int32_t>());
    if (taa_node["history_color_mode"])
      taa.history_color_mode = static_cast<HistoryColorMode>(taa_node["history_color_mode"].as<int32_t>());
    if (taa_node["variance_sample_count"])
      taa.variance_sample_count = taa_node["variance_sample_count"].as<int>();
    if (taa_node["longest_velocity_sample_count"])
      taa.longest_velocity_sample_count = taa_node["longest_velocity_sample_count"].as<int>();
    if (taa_node["use_ycocg"])
      taa.use_ycocg = taa_node["use_ycocg"].as<bool>();
    if (taa_node["use_neighborhood_sampling"])
      taa.use_neighborhood_sampling = taa_node["use_neighborhood_sampling"].as<bool>();
    if (taa_node["use_bicubic_filter"])
      taa.use_bicubic_filter = taa_node["use_bicubic_filter"].as<bool>();
    if (taa_node["use_longest_velocity"])
      taa.use_longest_velocity = taa_node["use_longest_velocity"].as<bool>();
    if (taa_node["use_depth_threshold"])
      taa.use_depth_threshold = taa_node["use_depth_threshold"].as<bool>();
    if (taa_node["use_tgsm"])
      taa.use_tgsm = taa_node["use_tgsm"].as<bool>();
    if (taa_node["use_fp16"])
      taa.use_fp16 = taa_node["use_fp16"].as<bool>();
    if (taa_node["min_variance_gamma"])
      taa.min_variance_gamma = taa_node["min_variance_gamma"].as<float>();
    if (taa_node["max_variance_gamma"])
      taa.max_variance_gamma = taa_node["max_variance_gamma"].as<float>();
    if (taa_node["velocity_rejection_threshold"])
      taa.velocity_rejection_threshold = taa_node["velocity_rejection_threshold"].as<float>();
    if (taa_node["depth_threshold"])
      taa.depth_threshold = taa_node["depth_threshold"].as<float>();
    if (taa_node["sharpen"])
      taa.sharpen = taa_node["sharpen"].as<float>();
    taa.preset = loaded_preset;
  }
  if (const auto smaa_node = in["smaa"]; smaa_node && smaa_node["preset"]) {
    const auto value = smaa_node["preset"].as<int32_t>();
    if (value >= static_cast<int32_t>(SmaaPreset::Low) && value <= static_cast<int32_t>(SmaaPreset::Ultra)) {
      smaa.preset = static_cast<SmaaPreset>(value);
    }
  }
  NormalizeSettings();
}

void AntiAliasing::ApplyTaaPreset(const TaaPreset value) {
  taa.preset = value;
  if (value == TaaPreset::Custom) {
    return;
  }

  taa.history_color_mode = HistoryColorMode::ToneMapped;
  taa.use_tgsm = true;
  taa.use_fp16 = false;
  taa.min_variance_gamma = 0.75f;
  taa.max_variance_gamma = 2.0f;
  taa.velocity_rejection_threshold = 128.0f;
  taa.depth_threshold = 0.002f;
  taa.sharpen = 0.0f;
  taa.use_depth_threshold = true;
  taa.use_neighborhood_sampling = true;
  taa.use_ycocg = true;
  taa.use_bicubic_filter = true;
  taa.use_longest_velocity = true;
  taa.longest_velocity_sample_count = 9;

  switch (value) {
    case TaaPreset::BestQuality:
      taa.variance_clipping_mode = VarianceClippingMode::Intersection;
      taa.variance_sample_count = 9;
      break;
    case TaaPreset::HighQuality:
      taa.variance_clipping_mode = VarianceClippingMode::Clamp;
      taa.variance_sample_count = 5;
      break;
    case TaaPreset::Performance:
      taa.variance_clipping_mode = VarianceClippingMode::Clamp;
      taa.variance_sample_count = 5;
      taa.longest_velocity_sample_count = 5;
      taa.use_fp16 = true;
      taa.use_depth_threshold = false;
      taa.use_neighborhood_sampling = false;
      taa.use_ycocg = false;
      taa.use_bicubic_filter = false;
      taa.use_longest_velocity = false;
      break;
    case TaaPreset::Custom:
      break;
  }
  NormalizeSettings();
}

void AntiAliasing::NormalizeSettings() {
  if (algorithm != Algorithm::Taa && algorithm != Algorithm::Smaa) {
    algorithm = Algorithm::Smaa;
  }
  const auto variance_mode = static_cast<int32_t>(taa.variance_clipping_mode);
  if (variance_mode < static_cast<int32_t>(VarianceClippingMode::Disabled) ||
      variance_mode > static_cast<int32_t>(VarianceClippingMode::Intersection)) {
    taa.variance_clipping_mode = VarianceClippingMode::Intersection;
  }
  const auto color_mode = static_cast<int32_t>(taa.history_color_mode);
  if (color_mode < static_cast<int32_t>(HistoryColorMode::ToneMapped) ||
      color_mode > static_cast<int32_t>(HistoryColorMode::Linear)) {
    taa.history_color_mode = HistoryColorMode::ToneMapped;
  }
  taa.variance_sample_count = taa.variance_sample_count <= 5 ? 5 : 9;
  taa.longest_velocity_sample_count = taa.longest_velocity_sample_count <= 5 ? 5 : 9;
  taa.min_variance_gamma = glm::max(taa.min_variance_gamma, 0.0f);
  taa.max_variance_gamma = glm::max(taa.max_variance_gamma, taa.min_variance_gamma);
  taa.velocity_rejection_threshold = glm::max(taa.velocity_rejection_threshold, 1.0f);
  taa.depth_threshold = glm::max(taa.depth_threshold, 0.0f);
  taa.sharpen = glm::clamp(taa.sharpen, 0.0f, 1.0f);
  const auto smaa_preset = static_cast<int32_t>(smaa.preset);
  if (smaa_preset < static_cast<int32_t>(SmaaPreset::Low) || smaa_preset > static_cast<int32_t>(SmaaPreset::Ultra)) {
    smaa.preset = SmaaPreset::Ultra;
  }
  const auto smaa_debug_mode = static_cast<int32_t>(smaa.debug_mode);
  if (smaa_debug_mode < static_cast<int32_t>(SmaaDebugMode::None) ||
      smaa_debug_mode > static_cast<int32_t>(SmaaDebugMode::BlendWeights)) {
    smaa.debug_mode = SmaaDebugMode::None;
  }
}

void AntiAliasing::Process(const PostProcessingStack& post_processing_stack,
                           const std::shared_ptr<Camera>& target_camera,
                           PostProcessingExecutionContext& context) const {
  if (algorithm == Algorithm::Taa) {
    ProcessTaa(post_processing_stack, target_camera, context);
  } else {
    ProcessSmaa(post_processing_stack, target_camera, context);
  }
}

void AntiAliasing::ProcessTaa(const PostProcessingStack& post_processing_stack,
                              const std::shared_ptr<Camera>& target_camera,
                              PostProcessingExecutionContext& context) const {
  auto& camera_resources = context.camera.anti_aliasing;
  auto& renderer_resources = context.renderer.anti_aliasing;
  const bool effective_use_fp16 = taa.use_fp16 && Platform::GetInstance().GetCapabilities().support_shader_float16;
  const size_t resolve_pipeline_index = (taa.use_tgsm ? 1u : 0u) | (effective_use_fp16 ? 2u : 0u);
  const auto& copy_pipeline = renderer_resources.copy_pipeline;
  const auto& resolve_pipeline = renderer_resources.resolve_pipelines[resolve_pipeline_index];
  if (!copy_pipeline || !copy_pipeline->Initialized() || !resolve_pipeline || !resolve_pipeline->Initialized()) {
    return;
  }
  if (!context.motion_vectors_image_view) {
    camera_resources.history.valid = false;
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto size = target_camera->GetSize();
  const uint32_t current_frame_index = Platform::GetFrameCount();
  auto& history = camera_resources.history;
  if (history.size != size || !history.textures[0] || !history.textures[1] || !history.depth_textures[0] ||
      !history.depth_textures[1]) {
    RenderTextureCreateInfo create_info{};
    create_info.depth = false;
    create_info.extent = {size.x, size.y, 1};
    history.textures[0] = std::make_shared<RenderTexture>(create_info);
    history.textures[1] = std::make_shared<RenderTexture>(create_info);
    history.depth_textures[0] = std::make_shared<RenderTexture>(create_info);
    history.depth_textures[1] = std::make_shared<RenderTexture>(create_info);
    history.size = size;
    history.frame_index = 0;
    history.valid = false;
  }
  const uint32_t previous_history_index = history.frame_index % 2u;
  const uint32_t output_history_index = 1u - previous_history_index;
  const bool skipped_frame = history.valid && current_frame_index > history.last_processed_frame + 1u;
  const uint32_t camera_history_version = target_camera->GetTemporalHistoryVersion();
  const bool camera_history_reset = history.valid && history.camera_history_version != camera_history_version;
  const bool reject_camera_history = render_layer->RequiresCameraWideTemporalHistoryRejection();
  const bool history_valid = history.valid && !skipped_frame && !camera_history_reset && !reject_camera_history;

  const auto& copy_descriptor_set = camera_resources.copy_descriptor_set.GetOrCreate(renderer_resources.copy_layout);
  const auto& resolve_descriptor_set =
      camera_resources.resolve_descriptor_set.GetOrCreate(renderer_resources.resolve_layout);

  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    copy_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = context.camera.stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = context.camera.stack.source_color_texture->GetColorSampler()->GetVkSampler();
    copy_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = context.camera.stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = context.camera.stack.source_color_texture->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = history.textures[previous_history_index]->GetColorImageView()->GetVkImageView();
    image_info.sampler = history.textures[previous_history_index]->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    image_info.imageView = context.motion_vectors_image_view->GetVkImageView();
    image_info.sampler = context.camera.stack.source_color_texture->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = history.depth_textures[previous_history_index]->GetColorImageView()->GetVkImageView();
    image_info.sampler = history.depth_textures[previous_history_index]->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(3, image_info);
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(4, image_info);
    image_info.imageView = history.textures[output_history_index]->GetColorImageView()->GetVkImageView();
    image_info.sampler = history.textures[output_history_index]->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(5, image_info);
    image_info.imageView = history.depth_textures[output_history_index]->GetColorImageView()->GetVkImageView();
    image_info.sampler = history.depth_textures[output_history_index]->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(6, image_info);
  }

  TaaPushConstant push_constant{};
  push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  push_constant.history_valid = history_valid ? 1 : 0;
  push_constant.frame_index = static_cast<int32_t>(history.frame_index & 1u);
  push_constant.variance_clipping_mode = static_cast<int32_t>(taa.variance_clipping_mode);
  push_constant.variance_sample_count = taa.variance_sample_count;
  push_constant.use_ycocg = taa.use_ycocg ? 1 : 0;
  push_constant.use_neighborhood_sampling = taa.use_neighborhood_sampling ? 1 : 0;
  push_constant.use_bicubic_filter = taa.use_bicubic_filter ? 1 : 0;
  push_constant.use_longest_velocity = taa.use_longest_velocity ? 1 : 0;
  push_constant.longest_velocity_sample_count = taa.longest_velocity_sample_count;
  push_constant.use_depth_threshold = taa.use_depth_threshold ? 1 : 0;
  push_constant.history_color_mode = static_cast<int32_t>(taa.history_color_mode);
  push_constant.sharpen = glm::max(taa.sharpen, 0.0f);
  push_constant.debug_mode = static_cast<int32_t>(taa.debug_mode);
  push_constant.min_variance_gamma = taa.min_variance_gamma;
  push_constant.max_variance_gamma = taa.max_variance_gamma;
  push_constant.velocity_rejection_threshold = taa.velocity_rejection_threshold;
  push_constant.depth_threshold = taa.depth_threshold;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    context.camera.stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                   VK_IMAGE_LAYOUT_GENERAL);
    copy_pipeline->Bind(vk_command_buffer);
    copy_pipeline->BindDescriptorSet(vk_command_buffer, 0, copy_descriptor_set->GetVkDescriptorSet());
    copy_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);

    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    history.textures[previous_history_index]->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                  VK_IMAGE_LAYOUT_GENERAL);
    history.textures[output_history_index]->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                VK_IMAGE_LAYOUT_GENERAL);
    history.depth_textures[previous_history_index]->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                        VK_IMAGE_LAYOUT_GENERAL);
    history.depth_textures[output_history_index]->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                      VK_IMAGE_LAYOUT_GENERAL);
    resolve_pipeline->Bind(vk_command_buffer);
    resolve_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    resolve_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                        target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    resolve_pipeline->BindDescriptorSet(vk_command_buffer, 2, resolve_descriptor_set->GetVkDescriptorSet());
    resolve_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    resolve_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 8), Platform::DivUp(size.y, 8));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  history.valid = true;
  history.frame_index++;
  history.last_processed_frame = current_frame_index;
  history.camera_history_version = camera_history_version;
}

void AntiAliasing::BuildPipelines(PostProcessingRendererResources& resources, const bool force_rebuild) const {
  BuildTaaPipelines(resources, force_rebuild);
  BuildSmaaPipelines(resources, force_rebuild);
}

void AntiAliasing::BuildTaaPipelines(PostProcessingRendererResources& resources, const bool force_rebuild) const {
  static_cast<void>(force_rebuild);
  auto& renderer = resources.anti_aliasing;
  if (!renderer.copy_layout) {
    renderer.copy_layout = std::make_shared<DescriptorSetLayout>();
    renderer.copy_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.copy_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.copy_layout->Initialize();
  }
  if (!renderer.resolve_layout) {
    renderer.resolve_layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 4; ++binding) {
      renderer.resolve_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    }
    for (uint32_t binding = 4; binding < 7; ++binding) {
      renderer.resolve_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                     VK_SHADER_STAGE_COMPUTE_BIT, 0);
    }
    renderer.resolve_layout->Initialize();
  }
  if (!renderer.copy_pipeline) {
    renderer.copy_pipeline = std::make_shared<ComputePipeline>();
    renderer.copy_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/TAACopy.comp");
    renderer.copy_pipeline->descriptor_set_layouts.emplace_back(renderer.copy_layout);
    renderer.copy_pipeline->Initialize();
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  for (size_t index = 0; index < renderer.resolve_pipelines.size(); ++index) {
    if (renderer.resolve_pipelines[index]) {
      continue;
    }
    const bool use_tgsm = (index & 1u) != 0;
    const bool use_fp16 = (index & 2u) != 0 && Platform::GetInstance().GetCapabilities().support_shader_float16;
    const auto shader_defines = Platform::GetShaderGlobalDefines() + "\n#define EE_TAA_USE_TGSM " +
                                std::string(use_tgsm ? "1\n" : "0\n") + "#define EE_TAA_USE_FP16 " +
                                std::string(use_fp16 ? "1\n" : "0\n");
    auto& pipeline = renderer.resolve_pipelines[index];
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, shader_defines,
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/TAAResolve.comp");
    pipeline->descriptor_set_layouts.emplace_back(render_layer->GetPerFrameDescriptorSetLayout());
    pipeline->descriptor_set_layouts.emplace_back(render_layer->GetCameraGBufferDescriptorSetLayout());
    pipeline->descriptor_set_layouts.emplace_back(renderer.resolve_layout);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(TaaPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    pipeline->Initialize();
  }
}

void AntiAliasing::EnsureSmaaLookupTextures(PostProcessingRendererResources& resources) const {
  static_assert(AREATEX_WIDTH == 160 && AREATEX_HEIGHT == 560);
  static_assert(SEARCHTEX_WIDTH == 64 && SEARCHTEX_HEIGHT == 16);
  static_assert(sizeof(areaTexBytes) == AREATEX_SIZE);
  static_assert(sizeof(searchTexBytes) == SEARCHTEX_SIZE);
  auto& renderer = resources.anti_aliasing;
  if (renderer.smaa_area_image && renderer.smaa_search_image && renderer.smaa_area_view && renderer.smaa_search_view &&
      renderer.smaa_lookup_sampler) {
    return;
  }
  renderer.smaa_area_image = CreateSmaaLookupImage(VK_FORMAT_R8G8_UNORM, AREATEX_WIDTH, AREATEX_HEIGHT);
  renderer.smaa_search_image = CreateSmaaLookupImage(VK_FORMAT_R8_UNORM, SEARCHTEX_WIDTH, SEARCHTEX_HEIGHT);
  UploadSmaaLookupImage(renderer.smaa_area_image, areaTexBytes, sizeof(areaTexBytes));
  UploadSmaaLookupImage(renderer.smaa_search_image, searchTexBytes, sizeof(searchTexBytes));
  renderer.smaa_area_view = CreateSmaaLookupView(renderer.smaa_area_image);
  renderer.smaa_search_view = CreateSmaaLookupView(renderer.smaa_search_image);
  renderer.smaa_lookup_sampler = CreateSmaaSampler();
}

void AntiAliasing::EnsureSmaaTargets(const glm::uvec2& size, PostProcessingCameraResources& resources) const {
  auto& anti_aliasing = resources.anti_aliasing;
  if (anti_aliasing.smaa_size == size && anti_aliasing.smaa_edges_texture && anti_aliasing.smaa_blend_texture) {
    return;
  }
  RenderTextureCreateInfo create_info{};
  create_info.extent = {size.x, size.y, 1};
  create_info.color_format = VK_FORMAT_R8G8B8A8_UNORM;
  create_info.depth = false;
  anti_aliasing.smaa_edges_texture = std::make_shared<RenderTexture>(create_info);
  anti_aliasing.smaa_blend_texture = std::make_shared<RenderTexture>(create_info);
  anti_aliasing.smaa_size = size;
}

void AntiAliasing::BuildSmaaPipelines(PostProcessingRendererResources& resources, const bool force_rebuild) const {
  static_cast<void>(force_rebuild);
  EnsureSmaaLookupTextures(resources);
  auto& renderer = resources.anti_aliasing;
  if (!renderer.smaa_prepare_layout) {
    renderer.smaa_prepare_layout = std::make_shared<DescriptorSetLayout>();
    renderer.smaa_prepare_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                        VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.smaa_prepare_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                        VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.smaa_prepare_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                        VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.smaa_prepare_layout->Initialize();
  }
  if (!renderer.smaa_edge_layout) {
    renderer.smaa_edge_layout = std::make_shared<DescriptorSetLayout>();
    renderer.smaa_edge_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                     VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    renderer.smaa_edge_layout->Initialize();
  }
  if (!renderer.smaa_weight_layout) {
    renderer.smaa_weight_layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 3; ++binding) {
      renderer.smaa_weight_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                         VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    }
    renderer.smaa_weight_layout->Initialize();
  }
  if (!renderer.smaa_neighborhood_layout) {
    renderer.smaa_neighborhood_layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 3; ++binding) {
      renderer.smaa_neighborhood_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                               VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    }
    renderer.smaa_neighborhood_layout->Initialize();
  }
  if (!renderer.smaa_prepare_pipeline) {
    renderer.smaa_prepare_pipeline = std::make_shared<ComputePipeline>();
    renderer.smaa_prepare_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SMAAPrepare.comp");
    renderer.smaa_prepare_pipeline->descriptor_set_layouts.emplace_back(renderer.smaa_prepare_layout);
    auto& push_constant_range = renderer.smaa_prepare_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SmaaPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    renderer.smaa_prepare_pipeline->Initialize();
  }

  const auto create_pipeline = [&](const std::filesystem::path& vertex_path, const std::filesystem::path& fragment_path,
                                   const std::shared_ptr<DescriptorSetLayout>& descriptor_layout,
                                   const VkFormat color_format, const std::string& defines) {
    auto pipeline = std::make_shared<GraphicsPipeline>();
    pipeline->vertex_shader = Shader::CreateTemporary(ShaderType::Vertex, defines, vertex_path);
    pipeline->fragment_shader = Shader::CreateTemporary(ShaderType::Fragment, defines, fragment_path);
    pipeline->geometry_type = GeometryType::Mesh;
    pipeline->vertex_input_enabled = false;
    pipeline->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    pipeline->view_mask = 0;
    pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
    pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    pipeline->color_attachment_formats = {color_format};
    pipeline->descriptor_set_layouts.emplace_back(descriptor_layout);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SmaaPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pipeline->Initialize();
    pipeline->states.depth_test = false;
    pipeline->states.depth_write = false;
    pipeline->states.cull_mode = VK_CULL_MODE_NONE;
    return pipeline;
  };

  const auto shader_root = Resources::GetDefaultResourcesPath() / "Shaders/Graphics";
  for (size_t preset_index = 0; preset_index < renderer.smaa_edge_pipelines.size(); ++preset_index) {
    if (renderer.smaa_edge_pipelines[preset_index] && renderer.smaa_weight_pipelines[preset_index]) {
      continue;
    }
    const auto defines = Platform::GetShaderGlobalDefines() + "\n#define " + GetSmaaPresetDefine(preset_index) + "\n";
    renderer.smaa_edge_pipelines[preset_index] = create_pipeline(
        shader_root / "Vertex/PostProcessing/SMAAEdge.vert", shader_root / "Fragment/PostProcessing/SMAAEdge.frag",
        renderer.smaa_edge_layout, VK_FORMAT_R8G8B8A8_UNORM, defines);
    renderer.smaa_weight_pipelines[preset_index] =
        create_pipeline(shader_root / "Vertex/PostProcessing/SMAABlendWeight.vert",
                        shader_root / "Fragment/PostProcessing/SMAABlendWeight.frag", renderer.smaa_weight_layout,
                        VK_FORMAT_R8G8B8A8_UNORM, defines);
  }
  if (!renderer.smaa_neighborhood_pipeline) {
    renderer.smaa_neighborhood_pipeline = create_pipeline(
        shader_root / "Vertex/PostProcessing/SMAANeighborhood.vert",
        shader_root / "Fragment/PostProcessing/SMAANeighborhood.frag", renderer.smaa_neighborhood_layout,
        Platform::Constants::render_texture_color, Platform::GetShaderGlobalDefines());
  }
}

void AntiAliasing::ProcessSmaa(const PostProcessingStack& post_processing_stack,
                               const std::shared_ptr<Camera>& target_camera,
                               PostProcessingExecutionContext& context) const {
  auto& camera = context.camera.anti_aliasing;
  auto& renderer = context.renderer.anti_aliasing;
  const auto preset_index = static_cast<size_t>(smaa.preset);
  if (!renderer.smaa_prepare_pipeline || !renderer.smaa_prepare_pipeline->Initialized() ||
      !renderer.smaa_edge_pipelines[preset_index] || !renderer.smaa_edge_pipelines[preset_index]->Initialized() ||
      !renderer.smaa_weight_pipelines[preset_index] || !renderer.smaa_weight_pipelines[preset_index]->Initialized() ||
      !renderer.smaa_neighborhood_pipeline || !renderer.smaa_neighborhood_pipeline->Initialized()) {
    return;
  }
  const auto size = target_camera->GetSize();
  if (size.x == 0 || size.y == 0) {
    return;
  }
  EnsureSmaaTargets(size, context.camera);
  const auto& prepare_descriptor_set = camera.smaa_prepare_descriptor_set.GetOrCreate(renderer.smaa_prepare_layout);
  const auto& edge_descriptor_set = camera.smaa_edge_descriptor_set.GetOrCreate(renderer.smaa_edge_layout);
  const auto& weight_descriptor_set = camera.smaa_weight_descriptor_set.GetOrCreate(renderer.smaa_weight_layout);
  const auto& neighborhood_descriptor_set =
      camera.smaa_neighborhood_descriptor_set.GetOrCreate(renderer.smaa_neighborhood_layout);

  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
  image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
  prepare_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = context.camera.stack.source_color_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = VK_NULL_HANDLE;
  prepare_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = context.camera.stack.swap_texture->GetColorImageView()->GetVkImageView();
  prepare_descriptor_set->UpdateImageDescriptorBinding(2, image_info);

  image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  image_info.imageView = context.camera.stack.swap_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = context.camera.stack.swap_texture->GetColorSampler()->GetVkSampler();
  edge_descriptor_set->UpdateImageDescriptorBinding(0, image_info);

  image_info.imageView = camera.smaa_edges_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = camera.smaa_edges_texture->GetColorSampler()->GetVkSampler();
  weight_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = renderer.smaa_area_view->GetVkImageView();
  image_info.sampler = renderer.smaa_lookup_sampler->GetVkSampler();
  weight_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = renderer.smaa_search_view->GetVkImageView();
  weight_descriptor_set->UpdateImageDescriptorBinding(2, image_info);

  image_info.imageView = context.camera.stack.source_color_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = context.camera.stack.source_color_texture->GetColorSampler()->GetVkSampler();
  neighborhood_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = camera.smaa_blend_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = camera.smaa_blend_texture->GetColorSampler()->GetVkSampler();
  neighborhood_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = camera.smaa_edges_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = camera.smaa_edges_texture->GetColorSampler()->GetVkSampler();
  neighborhood_descriptor_set->UpdateImageDescriptorBinding(2, image_info);

  SmaaPushConstant push_constant{};
  push_constant.metrics = {1.0f / static_cast<float>(size.x), 1.0f / static_cast<float>(size.y),
                           static_cast<float>(size.x), static_cast<float>(size.y)};
  push_constant.tone_mapped = post_processing_stack.enable_tone_mapping ? 1 : 0;
  push_constant.debug_mode = static_cast<int32_t>(smaa.debug_mode);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const glm::ivec4 viewport{0, 0, static_cast<int>(size.x), static_cast<int>(size.y)};
    const auto render_fullscreen = [&](const std::shared_ptr<RenderTexture>& render_target,
                                       const std::shared_ptr<GraphicsPipeline>& pipeline,
                                       const std::shared_ptr<DescriptorSet>& descriptor_set) {
      std::vector<VkRenderingAttachmentInfo> attachments;
      render_target->AppendColorAttachmentInfos(attachments, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      VkRenderingInfo render_info{};
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea.extent = {size.x, size.y};
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = static_cast<uint32_t>(attachments.size());
      render_info.pColorAttachments = attachments.data();
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&] {
        pipeline->Bind(vk_command_buffer);
        const VkViewport vk_viewport = {static_cast<float>(viewport.x),
                                        static_cast<float>(viewport.y),
                                        static_cast<float>(viewport.z),
                                        static_cast<float>(viewport.w),
                                        0.0f,
                                        1.0f};
        const VkRect2D scissor = {{viewport.x, viewport.y},
                                  {static_cast<uint32_t>(viewport.z), static_cast<uint32_t>(viewport.w)}};
        vkCmdSetViewport(vk_command_buffer, 0, 1, &vk_viewport);
        vkCmdSetScissor(vk_command_buffer, 0, 1, &scissor);
        pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
        pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDraw(vk_command_buffer, 3, 1, 0, 0);
      });
    };

    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    context.camera.stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                   VK_IMAGE_LAYOUT_GENERAL);
    context.camera.stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    renderer.smaa_prepare_pipeline->Bind(vk_command_buffer);
    renderer.smaa_prepare_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                      prepare_descriptor_set->GetVkDescriptorSet());
    renderer.smaa_prepare_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    renderer.smaa_prepare_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 8), Platform::DivUp(size.y, 8));
    Platform::EverythingBarrier(vk_command_buffer);

    context.camera.stack.source_color_texture->GetColorImage()->TransitImageLayout(
        vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    context.camera.stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    camera.smaa_edges_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    render_fullscreen(camera.smaa_edges_texture, renderer.smaa_edge_pipelines[preset_index], edge_descriptor_set);

    camera.smaa_edges_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    camera.smaa_blend_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    render_fullscreen(camera.smaa_blend_texture, renderer.smaa_weight_pipelines[preset_index], weight_descriptor_set);

    camera.smaa_blend_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                           VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    render_fullscreen(target_camera->GetRenderTexture(), renderer.smaa_neighborhood_pipeline,
                      neighborhood_descriptor_set);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
  });
}

void Bloom::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "filter_radius" << YAML::Value << filter_radius;
  out << YAML::Key << "bloom_chain_length" << YAML::Value << bloom_chain_length;
}

void Bloom::Deserialize(const YAML::Node& in) {
  if (in["filter_radius"])
    filter_radius = in["filter_radius"].as<float>();
  if (in["bloom_chain_length"])
    bloom_chain_length = in["bloom_chain_length"].as<int>();
}

void Bloom::Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
                    PostProcessingExecutionContext& context) const {
  auto& stack = context.camera.stack;
  auto& camera = context.camera.bloom;
  auto& renderer = context.renderer.bloom;
  if (!renderer.copy_pipeline || !renderer.copy_pipeline->Initialized() || !renderer.downsampling_pipeline ||
      !renderer.downsampling_pipeline->Initialized() || !renderer.upsampling_pipeline ||
      !renderer.upsampling_pipeline->Initialized() || !renderer.mix_pipeline || !renderer.mix_pipeline->Initialized())
    return;

  const auto mip_levels = stack.result_texture->GetMipLevels();
  const auto base_extent = stack.result_texture->GetColorImage()->GetExtent();
  const auto target_size = target_camera->GetSize();
  const auto& copy_frame_descriptor_set = camera.copy_descriptor_set.GetOrCreate(renderer.copy_layout);
  const auto& mix_frame_descriptor_set = camera.mix_descriptor_set.GetOrCreate(renderer.mix_layout);
  auto& downsampling_frame_descriptor_sets = camera.downsampling_descriptor_sets.Get();
  auto& upsampling_frame_descriptor_sets = camera.upsampling_descriptor_sets.Get();
  const auto acquire_sampling_descriptor_set = [&](auto& descriptor_sets, const size_t index) {
    descriptor_sets.resize(std::max(descriptor_sets.size(), index + 1));
    auto& descriptor_set = descriptor_sets[index];
    if (!descriptor_set) {
      descriptor_set = std::make_shared<DescriptorSet>(renderer.sampling_layout);
    }
    return descriptor_set;
  };

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = stack.source_color_texture->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = stack.result_texture->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    renderer.copy_pipeline->Bind(vk_command_buffer);
    renderer.copy_pipeline->BindDescriptorSet(vk_command_buffer, 0, copy_frame_descriptor_set->GetVkDescriptorSet());
    ComputePushConstant push_constant;
    push_constant.resolution = target_size;
    renderer.copy_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    renderer.copy_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_size.x, 16),
                                     Platform::DivUp(target_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    for (int target_mip_level = 1; target_mip_level < glm::min(static_cast<int>(mip_levels), bloom_chain_length + 1);
         ++target_mip_level) {
      const auto current_descriptor_set =
          acquire_sampling_descriptor_set(downsampling_frame_descriptor_sets, target_mip_level - 1);

      VkDescriptorImageInfo descriptor_image_info;
      descriptor_image_info.imageView = stack.result_texture->GetColorImageView(target_mip_level - 1)->GetVkImageView();
      descriptor_image_info.imageLayout = stack.result_texture->GetColorImage()->GetLayout();
      descriptor_image_info.sampler = stack.result_texture->GetColorSampler()->GetVkSampler();
      current_descriptor_set->UpdateImageDescriptorBinding(0, descriptor_image_info);
      descriptor_image_info.imageView = stack.result_texture->GetColorImageView(target_mip_level)->GetVkImageView();
      current_descriptor_set->UpdateImageDescriptorBinding(1, descriptor_image_info);
      const float mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, target_mip_level);
      const float mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, target_mip_level);
      if (mip_width < 1.f || mip_height < 1.f)
        continue;
      const auto mip_extent_width = static_cast<uint32_t>(mip_width);
      const auto mip_extent_height = static_cast<uint32_t>(mip_height);
      renderer.downsampling_pipeline->Bind(vk_command_buffer);
      renderer.downsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                        current_descriptor_set->GetVkDescriptorSet());
      DownsamplingPushConstant push_constant;
      push_constant.mip_level = target_mip_level - 1;
      push_constant.source_resolution = {mip_width, mip_height};
      renderer.downsampling_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      renderer.downsampling_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(mip_extent_width, 16),
                                               Platform::DivUp(mip_extent_height, 16));
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    size_t descriptor_index = 0;
    for (int src_mip_level = glm::min(static_cast<int>(mip_levels), bloom_chain_length + 1) - 1; src_mip_level > 0;
         --src_mip_level) {
      const auto current_descriptor_set =
          acquire_sampling_descriptor_set(upsampling_frame_descriptor_sets, descriptor_index++);

      VkDescriptorImageInfo descriptor_image_info;
      descriptor_image_info.imageView = stack.result_texture->GetColorImageView(src_mip_level)->GetVkImageView();
      descriptor_image_info.imageLayout = stack.result_texture->GetColorImage()->GetLayout();
      descriptor_image_info.sampler = stack.result_texture->GetColorSampler()->GetVkSampler();
      current_descriptor_set->UpdateImageDescriptorBinding(0, descriptor_image_info);
      descriptor_image_info.imageView = stack.result_texture->GetColorImageView(src_mip_level - 1)->GetVkImageView();
      current_descriptor_set->UpdateImageDescriptorBinding(1, descriptor_image_info);

      const float prev_mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, src_mip_level);
      const float prev_mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, src_mip_level);
      if (prev_mip_width < 1.f || prev_mip_height < 1.f)
        continue;

      const float mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, src_mip_level - 1);
      const float mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, src_mip_level - 1);
      const auto mip_extent_width = static_cast<uint32_t>(mip_width);
      const auto mip_extent_height = static_cast<uint32_t>(mip_height);
      renderer.upsampling_pipeline->Bind(vk_command_buffer);
      renderer.upsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                      current_descriptor_set->GetVkDescriptorSet());
      UpsamplingPushConstant push_constant;
      push_constant.target_resolution = {mip_extent_width, mip_extent_height};
      push_constant.filter_radius = filter_radius;
      renderer.upsampling_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      renderer.upsampling_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(mip_extent_width, 16),
                                             Platform::DivUp(mip_extent_height, 16));
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = stack.source_color_texture->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = stack.result_texture->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    renderer.mix_pipeline->Bind(vk_command_buffer);
    renderer.mix_pipeline->BindDescriptorSet(vk_command_buffer, 0, mix_frame_descriptor_set->GetVkDescriptorSet());
    ComputePushConstant push_constant;
    push_constant.resolution = target_size;
    renderer.mix_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    renderer.mix_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_size.x, 16),
                                    Platform::DivUp(target_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void Bloom::BuildPipelines(PostProcessingRendererResources& resources, const bool force_rebuild) const {
  static_cast<void>(force_rebuild);
  auto& renderer = resources.bloom;
  if (!renderer.mix_layout) {
    renderer.mix_layout = std::make_shared<DescriptorSetLayout>();
    renderer.mix_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                               VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.mix_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                               VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.mix_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.mix_layout->Initialize();
  }
  if (!renderer.copy_layout) {
    renderer.copy_layout = std::make_shared<DescriptorSetLayout>();
    renderer.copy_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.copy_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.copy_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.copy_layout->Initialize();
  }
  if (!renderer.sampling_layout) {
    renderer.sampling_layout = std::make_shared<DescriptorSetLayout>();
    renderer.sampling_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                    VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.sampling_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                    0);
    renderer.sampling_layout->Initialize();
  }
  if (!renderer.downsampling_pipeline) {
    renderer.downsampling_pipeline = std::make_shared<ComputePipeline>();
    renderer.downsampling_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomDownsampling.comp");
    renderer.downsampling_pipeline->descriptor_set_layouts.emplace_back(renderer.sampling_layout);
    auto& downsampling_push_constant_range = renderer.downsampling_pipeline->push_constant_ranges.emplace_back();
    downsampling_push_constant_range.size = sizeof(DownsamplingPushConstant);
    downsampling_push_constant_range.offset = 0;
    downsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    renderer.downsampling_pipeline->Initialize();
  }
  if (!renderer.upsampling_pipeline) {
    renderer.upsampling_pipeline = std::make_shared<ComputePipeline>();
    renderer.upsampling_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomUpsampling.comp");
    renderer.upsampling_pipeline->descriptor_set_layouts.emplace_back(renderer.sampling_layout);
    auto& upsampling_push_constant_range = renderer.upsampling_pipeline->push_constant_ranges.emplace_back();
    upsampling_push_constant_range.size = sizeof(UpsamplingPushConstant);
    upsampling_push_constant_range.offset = 0;
    upsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    renderer.upsampling_pipeline->Initialize();
  }
  if (!renderer.copy_pipeline) {
    renderer.copy_pipeline = std::make_shared<ComputePipeline>();
    renderer.copy_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomCopy.comp");
    renderer.copy_pipeline->descriptor_set_layouts.emplace_back(renderer.copy_layout);
    auto& copy_push_constant_range = renderer.copy_pipeline->push_constant_ranges.emplace_back();
    copy_push_constant_range.size = sizeof(ComputePushConstant);
    copy_push_constant_range.offset = 0;
    copy_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    renderer.copy_pipeline->Initialize();
  }
  if (!renderer.mix_pipeline) {
    renderer.mix_pipeline = std::make_shared<ComputePipeline>();
    renderer.mix_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomMix.comp");
    renderer.mix_pipeline->descriptor_set_layouts.emplace_back(renderer.mix_layout);
    auto& mix_push_constant_range = renderer.mix_pipeline->push_constant_ranges.emplace_back();
    mix_push_constant_range.size = sizeof(ComputePushConstant);
    mix_push_constant_range.offset = 0;
    mix_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    renderer.mix_pipeline->Initialize();
  }
}

void PostProcessingStack::Resize(PostProcessingCameraResources& resources, const glm::uvec2& size) const {
  if (size.x == 0 || size.y == 0)
    return;
  if (size.x > 16384 || size.y >= 16384)
    return;
  auto& stack = resources.stack;
  if (size == stack.size && stack.source_color_texture && stack.result_texture && stack.swap_texture)
    return;
  const uint32_t mip_levels = static_cast<uint32_t>(std::floor(std::log2(std::max(size.x, size.y)))) + 1;
  RenderTextureCreateInfo create_info{};
  create_info.depth = false;
  create_info.extent = {size.x, size.y, 1};
  stack.source_color_texture = std::make_shared<RenderTexture>(create_info);
  stack.result_texture = std::make_shared<RenderTexture>(create_info, mip_levels);
  stack.swap_texture = std::make_shared<RenderTexture>(create_info);
  stack.size = size;
  ++stack.generation;
}

void PostProcessingStack::OnCreate() {
  ambient_occlusion = std::make_shared<AmbientOcclusion>();
  bloom = std::make_shared<Bloom>();
  screen_space_reflection = std::make_shared<ScreenSpaceReflection>();
  anti_aliasing = std::make_shared<AntiAliasing>();
  tone_mapping = std::make_shared<ToneMapping>();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (render_layer && render_layer->GetPostProcessingRendererResources() &&
      !ApplicationContext::Get().GetLayer<WindowLayer>()) {
    while (!BuildNextPipeline(*render_layer->GetPostProcessingRendererResources())) {
    }
  }

  enable_ambient_occlusion = true;
  enable_bloom = false;
  enable_screen_space_reflection = false;
  enable_anti_aliasing = true;
  enable_tone_mapping = true;
}

bool PostProcessingStack::BuildNextPipeline(PostProcessingRendererResources& resources) const {
  if (resources.pipelines_ready) {
    return true;
  }
  if (!ambient_occlusion || !bloom || !screen_space_reflection || !anti_aliasing || !tone_mapping) {
    return false;
  }
  switch (resources.pipeline_build_step) {
    case 0:
      if (!resources.stack.blur_layout) {
        resources.stack.blur_layout = std::make_shared<DescriptorSetLayout>();
        resources.stack.blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                           VK_SHADER_STAGE_COMPUTE_BIT, 0);
        resources.stack.blur_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                           VK_SHADER_STAGE_COMPUTE_BIT, 0);
        resources.stack.blur_layout->Initialize();
      }
      if (!resources.stack.blur_pipeline) {
        resources.stack.blur_pipeline = std::make_shared<ComputePipeline>();
        resources.stack.blur_pipeline->compute_shader =
            Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                    Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/Blur.comp");
        resources.stack.blur_pipeline->descriptor_set_layouts.emplace_back(resources.stack.blur_layout);
        auto& push_constant_range = resources.stack.blur_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(int) + sizeof(float) * 5;
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
        resources.stack.blur_pipeline->Initialize();
      }
      break;
    case 1:
      ambient_occlusion->BuildPipelines(resources);
      break;
    case 2:
      screen_space_reflection->BuildPipelines(resources);
      break;
    case 3:
      anti_aliasing->BuildPipelines(resources);
      break;
    case 4:
      bloom->BuildPipelines(resources);
      break;
    case 5:
      tone_mapping->BuildPipelines(resources);
      break;
    default:
      resources.pipelines_ready = true;
      return true;
  }
  ++resources.pipeline_build_step;
  resources.pipelines_ready = resources.pipeline_build_step > 5;
  return false;
}

void PostProcessingStack::Process(const std::shared_ptr<Camera>& target_camera,
                                  const std::shared_ptr<ImageView>& motion_vectors_image_view,
                                  const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process) {
  if (!target_camera) {
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer || !render_layer->GetPostProcessingRendererResources()) {
    return;
  }
  auto& renderer = *render_layer->GetPostProcessingRendererResources();
  if (!BuildNextPipeline(renderer)) {
    return;
  }
  auto stack_asset = target_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  auto& camera = target_camera->AcquirePostProcessingResources(stack_asset);
  Resize(camera, target_camera->GetSize());
  PostProcessingExecutionContext context{camera, renderer, motion_vectors_image_view};
  if (pre_process) {
    Platform::RecordCommandsMainQueue(pre_process);
  }

  if (enable_ambient_occlusion) {
    ambient_occlusion->Process(*this, target_camera, context);
  }
  if (enable_screen_space_reflection) {
    screen_space_reflection->Process(*this, target_camera, context);
  }
  if (enable_anti_aliasing && anti_aliasing->algorithm == AntiAliasing::Algorithm::Taa) {
    anti_aliasing->Process(*this, target_camera, context);
  } else {
    camera.anti_aliasing.history.valid = false;
  }
  if (enable_bloom) {
    bloom->Process(*this, target_camera, context);
  }

  if (enable_tone_mapping) {
    tone_mapping->Process(*this, target_camera, context);
  }
  if (enable_anti_aliasing && anti_aliasing->algorithm == AntiAliasing::Algorithm::Smaa) {
    anti_aliasing->Process(*this, target_camera, context);
  }
}

void PostProcessingStack::ProcessRayCamera(const std::shared_ptr<Camera>& target_camera,
                                           const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process) {
  if (!target_camera) {
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer || !render_layer->GetPostProcessingRendererResources()) {
    return;
  }
  auto& renderer = *render_layer->GetPostProcessingRendererResources();
  if (!BuildNextPipeline(renderer)) {
    return;
  }
  auto stack_asset = target_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  auto& camera = target_camera->AcquirePostProcessingResources(stack_asset);
  Resize(camera, target_camera->GetSize());
  PostProcessingExecutionContext context{camera, renderer, {}};
  if (pre_process) {
    Platform::RecordCommandsMainQueue(pre_process);
  }
  if (enable_bloom) {
    bloom->Process(*this, target_camera, context);
  }
  if (enable_tone_mapping) {
    tone_mapping->Process(*this, target_camera, context);
  }
}

void PostProcessingStack::GaussianBlur(const glm::uvec2& size, PostProcessingExecutionContext& context) const {
  struct PushConstant {
    int horizontal = false;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
  };
  const auto& blur_pipeline = context.renderer.stack.blur_pipeline;
  if (!blur_pipeline || !blur_pipeline->Initialized()) {
    return;
  }

  PushConstant push_constant{};
  const auto& horizontal_descriptor_set =
      context.camera.stack.blur_horizontal_descriptor_set.GetOrCreate(context.renderer.stack.blur_layout);
  const auto& vertical_descriptor_set =
      context.camera.stack.blur_vertical_descriptor_set.GetOrCreate(context.renderer.stack.blur_layout);

  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = context.camera.stack.result_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = context.camera.stack.result_texture->GetColorSampler()->GetVkSampler();
  horizontal_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = context.camera.stack.swap_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = context.camera.stack.swap_texture->GetColorSampler()->GetVkSampler();
  horizontal_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = context.camera.stack.swap_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = context.camera.stack.swap_texture->GetColorSampler()->GetVkSampler();
  vertical_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = context.camera.stack.result_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = context.camera.stack.result_texture->GetColorSampler()->GetVkSampler();
  vertical_descriptor_set->UpdateImageDescriptorBinding(1, image_info);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    context.camera.stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                             VK_IMAGE_LAYOUT_GENERAL);
    context.camera.stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

    blur_pipeline->Bind(vk_command_buffer);
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, horizontal_descriptor_set->GetVkDescriptorSet());
    push_constant.horizontal = true;
    blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);

    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, vertical_descriptor_set->GetVkDescriptorSet());
    push_constant.horizontal = false;
    blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}
