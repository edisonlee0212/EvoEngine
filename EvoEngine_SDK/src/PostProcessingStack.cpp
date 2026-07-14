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
using namespace evo_engine;

namespace {
template <typename T>
void HashCombine(size_t& seed, const T& value) {
  seed ^= std::hash<T>{}(value) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
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

void PerFrameDescriptorSetList::Reset() {
  slots_.clear();
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
  reset_history_ = true;
}

void AntiAliasing::ApplyTaaPreset(const TaaPreset value) {
  taa.preset = value;
  if (value == TaaPreset::Custom) {
    reset_history_ = true;
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
  reset_history_ = true;
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

size_t AntiAliasing::ComputeSettingsHash() const {
  size_t hash = 0;
  HashCombine(hash, static_cast<int32_t>(taa.preset));
  HashCombine(hash, static_cast<int32_t>(taa.variance_clipping_mode));
  HashCombine(hash, static_cast<int32_t>(taa.history_color_mode));
  HashCombine(hash, taa.variance_sample_count);
  HashCombine(hash, taa.longest_velocity_sample_count);
  HashCombine(hash, taa.use_ycocg);
  HashCombine(hash, taa.use_neighborhood_sampling);
  HashCombine(hash, taa.use_bicubic_filter);
  HashCombine(hash, taa.use_longest_velocity);
  HashCombine(hash, taa.use_depth_threshold);
  HashCombine(hash, taa.use_tgsm);
  HashCombine(hash, taa.use_fp16);
  HashCombine(hash, taa.min_variance_gamma);
  HashCombine(hash, taa.max_variance_gamma);
  HashCombine(hash, taa.velocity_rejection_threshold);
  HashCombine(hash, taa.depth_threshold);
  HashCombine(hash, taa.sharpen);
  return hash;
}

void AntiAliasing::Process(const PostProcessingStack& post_processing_stack,
                           const std::shared_ptr<Camera>& target_camera) {
  NormalizeSettings();
  if (algorithm == Algorithm::Taa) {
    ProcessTaa(post_processing_stack, target_camera);
  } else {
    ProcessSmaa(post_processing_stack, target_camera);
  }
}

void AntiAliasing::ProcessTaa(const PostProcessingStack& post_processing_stack,
                              const std::shared_ptr<Camera>& target_camera) {
  const bool effective_use_fp16 = taa.use_fp16 && Platform::GetInstance().GetCapabilities().support_shader_float16;
  if (!resolve_pipeline_configuration_valid_ || built_use_tgsm_ != taa.use_tgsm ||
      built_use_fp16_ != effective_use_fp16) {
    BuildTaaPipelines(true);
  }
  if (!copy_pipeline_ || !copy_pipeline_->Initialized() || !resolve_pipeline_ || !resolve_pipeline_->Initialized()) {
    return;
  }
  if (!post_processing_stack.motion_vectors_image_view) {
    ResetHistory(target_camera);
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto size = target_camera->GetSize();
  const uint64_t camera_handle = target_camera->GetHandle().GetValue();
  const uint32_t current_frame_index = Platform::GetFrameCount();
  PruneHistory(current_frame_index, camera_handle);
  auto& history = history_resources_[camera_handle];
  if (history.size != size || !history.textures[0] || !history.textures[1] || !history.depth_textures[0] ||
      !history.depth_textures[1]) {
    if (history.textures[0]) {
      Platform::WaitForFrameSubmissions("Required TAA History Resize Fence Wait");
    }
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
  const auto settings_hash = ComputeSettingsHash();
  const bool settings_changed = history.valid && history.settings_hash != settings_hash;
  const uint32_t camera_history_version = target_camera->GetTemporalHistoryVersion();
  const bool camera_history_reset = history.valid && history.camera_history_version != camera_history_version;
  const bool reject_camera_history = render_layer->RequiresCameraWideTemporalHistoryRejection();
  const bool history_valid = history.valid && !reset_history_ && !skipped_frame && !settings_changed &&
                             !camera_history_reset && !reject_camera_history;
  reset_history_ = false;

  const auto& copy_descriptor_set = copy_descriptor_set_.GetOrCreate(copy_layout_);
  const auto& resolve_descriptor_set = resolve_descriptor_set_.GetOrCreate(resolve_layout_);

  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    copy_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    copy_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = history.textures[previous_history_index]->GetColorImageView()->GetVkImageView();
    image_info.sampler = history.textures[previous_history_index]->GetColorSampler()->GetVkSampler();
    resolve_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    image_info.imageView = post_processing_stack.motion_vectors_image_view->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
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
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    copy_pipeline_->Bind(vk_command_buffer);
    copy_pipeline_->BindDescriptorSet(vk_command_buffer, 0, copy_descriptor_set->GetVkDescriptorSet());
    copy_pipeline_->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
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
    resolve_pipeline_->Bind(vk_command_buffer);
    resolve_pipeline_->BindDescriptorSet(vk_command_buffer, 0,
                                         render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    resolve_pipeline_->BindDescriptorSet(vk_command_buffer, 1,
                                         target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    resolve_pipeline_->BindDescriptorSet(vk_command_buffer, 2, resolve_descriptor_set->GetVkDescriptorSet());
    resolve_pipeline_->PushConstant(vk_command_buffer, 0, push_constant);
    resolve_pipeline_->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 8), Platform::DivUp(size.y, 8));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  history.valid = true;
  history.frame_index++;
  history.last_processed_frame = current_frame_index;
  history.camera_history_version = camera_history_version;
  history.settings_hash = settings_hash;
}

void AntiAliasing::ResetHistory(const std::shared_ptr<Camera>& target_camera) {
  if (!target_camera) {
    for (auto& history_pair : history_resources_) {
      history_pair.second.valid = false;
    }
    return;
  }
  if (const auto search = history_resources_.find(target_camera->GetHandle().GetValue());
      search != history_resources_.end()) {
    search->second.valid = false;
  }
}

void AntiAliasing::RetainRuntimeResources(const uint64_t camera_handle,
                                          RenderGraphTransientResourceStore& transient_resources) const {
  transient_resources.RetainRenderTextureResources(smaa_edges_texture_);
  transient_resources.RetainRenderTextureResources(smaa_blend_texture_);
  if (const auto search = history_resources_.find(camera_handle); search != history_resources_.end()) {
    for (const auto& texture : search->second.textures) {
      transient_resources.RetainRenderTextureResources(texture);
    }
    for (const auto& texture : search->second.depth_textures) {
      transient_resources.RetainRenderTextureResources(texture);
    }
  }
}

void AntiAliasing::PruneHistory(const uint32_t current_frame_index, const uint64_t active_camera_handle) {
  constexpr uint32_t kRetainFrameCount = 120u;
  for (auto iterator = history_resources_.begin(); iterator != history_resources_.end();) {
    const auto last_processed_frame = iterator->second.last_processed_frame;
    if (iterator->first != active_camera_handle && current_frame_index >= last_processed_frame &&
        current_frame_index - last_processed_frame > kRetainFrameCount) {
      iterator = history_resources_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

void AntiAliasing::BuildPipelines(const bool force_rebuild) {
  BuildTaaPipelines(force_rebuild);
  BuildSmaaPipelines(force_rebuild);
}

void AntiAliasing::BuildTaaPipelines(const bool force_rebuild) {
  if (force_rebuild && (copy_pipeline_ || resolve_pipeline_)) {
    Platform::WaitForFrameSubmissions("Required Post-Processing Pipeline Rebuild Fence Wait");
    copy_descriptor_set_.Reset();
    resolve_descriptor_set_.Reset();
  }
  if (force_rebuild || !copy_layout_) {
    copy_layout_ = std::make_shared<DescriptorSetLayout>();
    copy_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout_->Initialize();
  }
  if (force_rebuild || !resolve_layout_) {
    resolve_layout_ = std::make_shared<DescriptorSetLayout>();
    resolve_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0);
    resolve_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0);
    resolve_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0);
    resolve_layout_->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0);
    resolve_layout_->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout_->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout_->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout_->Initialize();
  }
  if (force_rebuild || !copy_pipeline_) {
    copy_pipeline_ = std::make_shared<ComputePipeline>();
    copy_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/TAACopy.comp");
    copy_pipeline_->descriptor_set_layouts.emplace_back(copy_layout_);
    copy_pipeline_->Initialize();
  }
  if (force_rebuild || !resolve_pipeline_) {
    resolve_pipeline_ = std::make_shared<ComputePipeline>();
    const bool effective_use_fp16 = taa.use_fp16 && Platform::GetInstance().GetCapabilities().support_shader_float16;
    const auto shader_defines = Platform::GetShaderGlobalDefines() + "\n#define EE_TAA_USE_TGSM " +
                                std::string(taa.use_tgsm ? "1\n" : "0\n") + "#define EE_TAA_USE_FP16 " +
                                std::string(effective_use_fp16 ? "1\n" : "0\n");
    resolve_pipeline_->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, shader_defines,
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/TAAResolve.comp");
    resolve_pipeline_->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    resolve_pipeline_->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    resolve_pipeline_->descriptor_set_layouts.emplace_back(resolve_layout_);
    auto& push_constant_range = resolve_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(TaaPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    resolve_pipeline_->Initialize();
    built_use_tgsm_ = taa.use_tgsm;
    built_use_fp16_ = effective_use_fp16;
    resolve_pipeline_configuration_valid_ = true;
  }
}

void AntiAliasing::EnsureSmaaLookupTextures() {
  static_assert(AREATEX_WIDTH == 160 && AREATEX_HEIGHT == 560);
  static_assert(SEARCHTEX_WIDTH == 64 && SEARCHTEX_HEIGHT == 16);
  static_assert(sizeof(areaTexBytes) == AREATEX_SIZE);
  static_assert(sizeof(searchTexBytes) == SEARCHTEX_SIZE);
  if (smaa_area_image_ && smaa_search_image_ && smaa_area_view_ && smaa_search_view_ && smaa_lookup_sampler_) {
    return;
  }
  smaa_area_image_ = CreateSmaaLookupImage(VK_FORMAT_R8G8_UNORM, AREATEX_WIDTH, AREATEX_HEIGHT);
  smaa_search_image_ = CreateSmaaLookupImage(VK_FORMAT_R8_UNORM, SEARCHTEX_WIDTH, SEARCHTEX_HEIGHT);
  UploadSmaaLookupImage(smaa_area_image_, areaTexBytes, sizeof(areaTexBytes));
  UploadSmaaLookupImage(smaa_search_image_, searchTexBytes, sizeof(searchTexBytes));
  smaa_area_view_ = CreateSmaaLookupView(smaa_area_image_);
  smaa_search_view_ = CreateSmaaLookupView(smaa_search_image_);
  smaa_lookup_sampler_ = CreateSmaaSampler();
}

void AntiAliasing::EnsureSmaaTargets(const glm::uvec2& size) {
  if (smaa_size_ == size && smaa_edges_texture_ && smaa_blend_texture_) {
    return;
  }
  if (smaa_edges_texture_) {
    Platform::WaitForFrameSubmissions("Required SMAA Resize Fence Wait");
  }
  RenderTextureCreateInfo create_info{};
  create_info.extent = {size.x, size.y, 1};
  create_info.color_format = VK_FORMAT_R8G8B8A8_UNORM;
  create_info.depth = false;
  smaa_edges_texture_ = std::make_shared<RenderTexture>(create_info);
  smaa_blend_texture_ = std::make_shared<RenderTexture>(create_info);
  smaa_size_ = size;
}

void AntiAliasing::BuildSmaaPipelines(const bool force_rebuild) {
  EnsureSmaaLookupTextures();
  if (force_rebuild) {
    smaa_prepare_descriptor_set_.Reset();
    smaa_edge_descriptor_set_.Reset();
    smaa_weight_descriptor_set_.Reset();
    smaa_neighborhood_descriptor_set_.Reset();
  }
  if (force_rebuild || !smaa_prepare_layout_) {
    smaa_prepare_layout_ = std::make_shared<DescriptorSetLayout>();
    smaa_prepare_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                VK_SHADER_STAGE_COMPUTE_BIT, 0);
    smaa_prepare_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    smaa_prepare_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    smaa_prepare_layout_->Initialize();
  }
  if (force_rebuild || !smaa_edge_layout_) {
    smaa_edge_layout_ = std::make_shared<DescriptorSetLayout>();
    smaa_edge_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                             0);
    smaa_edge_layout_->Initialize();
  }
  if (force_rebuild || !smaa_weight_layout_) {
    smaa_weight_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 3; ++binding) {
      smaa_weight_layout_->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                 VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    }
    smaa_weight_layout_->Initialize();
  }
  if (force_rebuild || !smaa_neighborhood_layout_) {
    smaa_neighborhood_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 3; ++binding) {
      smaa_neighborhood_layout_->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                       VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    }
    smaa_neighborhood_layout_->Initialize();
  }
  if (force_rebuild || !smaa_prepare_pipeline_) {
    smaa_prepare_pipeline_ = std::make_shared<ComputePipeline>();
    smaa_prepare_pipeline_->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SMAAPrepare.comp");
    smaa_prepare_pipeline_->descriptor_set_layouts.emplace_back(smaa_prepare_layout_);
    auto& push_constant_range = smaa_prepare_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SmaaPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    smaa_prepare_pipeline_->Initialize();
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
    return pipeline;
  };

  const auto shader_root = Resources::GetDefaultResourcesPath() / "Shaders/Graphics";
  for (size_t preset_index = 0; preset_index < smaa_edge_pipelines_.size(); ++preset_index) {
    if (!force_rebuild && smaa_edge_pipelines_[preset_index] && smaa_weight_pipelines_[preset_index]) {
      continue;
    }
    const auto defines = Platform::GetShaderGlobalDefines() + "\n#define " + GetSmaaPresetDefine(preset_index) + "\n";
    smaa_edge_pipelines_[preset_index] = create_pipeline(shader_root / "Vertex/PostProcessing/SMAAEdge.vert",
                                                         shader_root / "Fragment/PostProcessing/SMAAEdge.frag",
                                                         smaa_edge_layout_, VK_FORMAT_R8G8B8A8_UNORM, defines);
    smaa_weight_pipelines_[preset_index] = create_pipeline(shader_root / "Vertex/PostProcessing/SMAABlendWeight.vert",
                                                           shader_root / "Fragment/PostProcessing/SMAABlendWeight.frag",
                                                           smaa_weight_layout_, VK_FORMAT_R8G8B8A8_UNORM, defines);
  }
  if (force_rebuild || !smaa_neighborhood_pipeline_) {
    smaa_neighborhood_pipeline_ =
        create_pipeline(shader_root / "Vertex/PostProcessing/SMAANeighborhood.vert",
                        shader_root / "Fragment/PostProcessing/SMAANeighborhood.frag", smaa_neighborhood_layout_,
                        Platform::Constants::render_texture_color, Platform::GetShaderGlobalDefines());
  }
}

void AntiAliasing::ProcessSmaa(const PostProcessingStack& post_processing_stack,
                               const std::shared_ptr<Camera>& target_camera) {
  const auto preset_index = static_cast<size_t>(smaa.preset);
  if (!smaa_prepare_pipeline_ || !smaa_prepare_pipeline_->Initialized() || !smaa_edge_pipelines_[preset_index] ||
      !smaa_edge_pipelines_[preset_index]->Initialized() || !smaa_weight_pipelines_[preset_index] ||
      !smaa_weight_pipelines_[preset_index]->Initialized() || !smaa_neighborhood_pipeline_ ||
      !smaa_neighborhood_pipeline_->Initialized()) {
    return;
  }
  const auto size = target_camera->GetSize();
  if (size.x == 0 || size.y == 0) {
    return;
  }
  EnsureSmaaTargets(size);
  EnsureSmaaLookupTextures();
  const auto& prepare_descriptor_set = smaa_prepare_descriptor_set_.GetOrCreate(smaa_prepare_layout_);
  const auto& edge_descriptor_set = smaa_edge_descriptor_set_.GetOrCreate(smaa_edge_layout_);
  const auto& weight_descriptor_set = smaa_weight_descriptor_set_.GetOrCreate(smaa_weight_layout_);
  const auto& neighborhood_descriptor_set = smaa_neighborhood_descriptor_set_.GetOrCreate(smaa_neighborhood_layout_);

  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
  image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
  prepare_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = VK_NULL_HANDLE;
  prepare_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = post_processing_stack.swap_texture->GetColorImageView()->GetVkImageView();
  prepare_descriptor_set->UpdateImageDescriptorBinding(2, image_info);

  image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  image_info.imageView = post_processing_stack.swap_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = post_processing_stack.swap_texture->GetColorSampler()->GetVkSampler();
  edge_descriptor_set->UpdateImageDescriptorBinding(0, image_info);

  image_info.imageView = smaa_edges_texture_->GetColorImageView()->GetVkImageView();
  image_info.sampler = smaa_edges_texture_->GetColorSampler()->GetVkSampler();
  weight_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = smaa_area_view_->GetVkImageView();
  image_info.sampler = smaa_lookup_sampler_->GetVkSampler();
  weight_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = smaa_search_view_->GetVkImageView();
  weight_descriptor_set->UpdateImageDescriptorBinding(2, image_info);

  image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
  image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
  neighborhood_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  image_info.imageView = smaa_blend_texture_->GetColorImageView()->GetVkImageView();
  image_info.sampler = smaa_blend_texture_->GetColorSampler()->GetVkSampler();
  neighborhood_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = smaa_edges_texture_->GetColorImageView()->GetVkImageView();
  image_info.sampler = smaa_edges_texture_->GetColorSampler()->GetVkSampler();
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
        pipeline->states.ResetAllStates(1);
        pipeline->states.SetViewportScissor(viewport);
        pipeline->states.depth_test = false;
        pipeline->states.depth_write = false;
        pipeline->states.cull_mode = VK_CULL_MODE_NONE;
        pipeline->Bind(vk_command_buffer);
        pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
        pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDraw(vk_command_buffer, 3, 1, 0, 0);
      });
    };

    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    smaa_prepare_pipeline_->Bind(vk_command_buffer);
    smaa_prepare_pipeline_->BindDescriptorSet(vk_command_buffer, 0, prepare_descriptor_set->GetVkDescriptorSet());
    smaa_prepare_pipeline_->PushConstant(vk_command_buffer, 0, push_constant);
    smaa_prepare_pipeline_->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 8), Platform::DivUp(size.y, 8));
    Platform::EverythingBarrier(vk_command_buffer);

    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(
        vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    post_processing_stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    smaa_edges_texture_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    render_fullscreen(smaa_edges_texture_, smaa_edge_pipelines_[preset_index], edge_descriptor_set);

    smaa_edges_texture_->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                             VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    smaa_blend_texture_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    render_fullscreen(smaa_blend_texture_, smaa_weight_pipelines_[preset_index], weight_descriptor_set);

    smaa_blend_texture_->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                             VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                           VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    render_fullscreen(target_camera->GetRenderTexture(), smaa_neighborhood_pipeline_, neighborhood_descriptor_set);
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

void Bloom::Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) {
  if (!copy_pipeline || !copy_pipeline->Initialized() || !downsampling_pipeline ||
      !downsampling_pipeline->Initialized() || !upsampling_pipeline || !upsampling_pipeline->Initialized() ||
      !mix_pipeline || !mix_pipeline->Initialized())
    return;

  const auto mip_levels = post_processing_stack.result_texture->GetMipLevels();
  const auto base_extent = post_processing_stack.result_texture->GetColorImage()->GetExtent();
  const auto target_size = target_camera->GetSize();
  const auto& copy_frame_descriptor_set = copy_descriptor_set.GetOrCreate(copy_layout);
  const auto& mix_frame_descriptor_set = mix_descriptor_set.GetOrCreate(mix_layout);
  auto& downsampling_frame_descriptor_sets = downsampling_descriptor_set.Get();
  auto& upsampling_frame_descriptor_sets = upsampling_descriptor_set.Get();
  const auto acquire_sampling_descriptor_set = [&](auto& descriptor_sets, const size_t index) {
    descriptor_sets.resize(std::max(descriptor_sets.size(), index + 1));
    auto& descriptor_set = descriptor_sets[index];
    if (!descriptor_set) {
      descriptor_set = std::make_shared<DescriptorSet>(sampling_layout);
    }
    return descriptor_set;
  };

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    copy_pipeline->Bind(vk_command_buffer);
    copy_pipeline->BindDescriptorSet(vk_command_buffer, 0, copy_frame_descriptor_set->GetVkDescriptorSet());
    ComputePushConstant push_constant;
    push_constant.resolution = target_size;
    copy_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    copy_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_size.x, 16), Platform::DivUp(target_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    for (int target_mip_level = 1; target_mip_level < glm::min(static_cast<int>(mip_levels), bloom_chain_length + 1);
         ++target_mip_level) {
      const auto current_descriptor_set =
          acquire_sampling_descriptor_set(downsampling_frame_descriptor_sets, target_mip_level - 1);

      VkDescriptorImageInfo descriptor_image_info;
      descriptor_image_info.imageView =
          post_processing_stack.result_texture->GetColorImageView(target_mip_level - 1)->GetVkImageView();
      descriptor_image_info.imageLayout = post_processing_stack.result_texture->GetColorImage()->GetLayout();
      descriptor_image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
      current_descriptor_set->UpdateImageDescriptorBinding(0, descriptor_image_info);
      descriptor_image_info.imageView =
          post_processing_stack.result_texture->GetColorImageView(target_mip_level)->GetVkImageView();
      current_descriptor_set->UpdateImageDescriptorBinding(1, descriptor_image_info);
      const float mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, target_mip_level);
      const float mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, target_mip_level);
      if (mip_width < 1.f || mip_height < 1.f)
        continue;
      const auto mip_extent_width = static_cast<uint32_t>(mip_width);
      const auto mip_extent_height = static_cast<uint32_t>(mip_height);
      downsampling_pipeline->Bind(vk_command_buffer);
      downsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0, current_descriptor_set->GetVkDescriptorSet());
      DownsamplingPushConstant push_constant;
      push_constant.mip_level = target_mip_level - 1;
      push_constant.source_resolution = {mip_width, mip_height};
      downsampling_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      downsampling_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(mip_extent_width, 16),
                                      Platform::DivUp(mip_extent_height, 16));
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    size_t descriptor_index = 0;
    for (int src_mip_level = glm::min(static_cast<int>(mip_levels), bloom_chain_length + 1) - 1; src_mip_level > 0;
         --src_mip_level) {
      const auto current_descriptor_set =
          acquire_sampling_descriptor_set(upsampling_frame_descriptor_sets, descriptor_index++);

      VkDescriptorImageInfo descriptor_image_info;
      descriptor_image_info.imageView =
          post_processing_stack.result_texture->GetColorImageView(src_mip_level)->GetVkImageView();
      descriptor_image_info.imageLayout = post_processing_stack.result_texture->GetColorImage()->GetLayout();
      descriptor_image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
      current_descriptor_set->UpdateImageDescriptorBinding(0, descriptor_image_info);
      descriptor_image_info.imageView =
          post_processing_stack.result_texture->GetColorImageView(src_mip_level - 1)->GetVkImageView();
      current_descriptor_set->UpdateImageDescriptorBinding(1, descriptor_image_info);

      const float prev_mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, src_mip_level);
      const float prev_mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, src_mip_level);
      if (prev_mip_width < 1.f || prev_mip_height < 1.f)
        continue;

      const float mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, src_mip_level - 1);
      const float mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, src_mip_level - 1);
      const auto mip_extent_width = static_cast<uint32_t>(mip_width);
      const auto mip_extent_height = static_cast<uint32_t>(mip_height);
      upsampling_pipeline->Bind(vk_command_buffer);
      upsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0, current_descriptor_set->GetVkDescriptorSet());
      UpsamplingPushConstant push_constant;
      push_constant.target_resolution = {mip_extent_width, mip_extent_height};
      push_constant.filter_radius = filter_radius;
      upsampling_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      upsampling_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(mip_extent_width, 16),
                                    Platform::DivUp(mip_extent_height, 16));
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    mix_pipeline->Bind(vk_command_buffer);
    mix_pipeline->BindDescriptorSet(vk_command_buffer, 0, mix_frame_descriptor_set->GetVkDescriptorSet());
    ComputePushConstant push_constant;
    push_constant.resolution = target_size;
    mix_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    mix_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_size.x, 16), Platform::DivUp(target_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void Bloom::BuildPipelines(const bool force_rebuild) {
  if (force_rebuild && (copy_pipeline || mix_pipeline || downsampling_pipeline || upsampling_pipeline)) {
    Platform::WaitForFrameSubmissions("Required Post-Processing Pipeline Rebuild Fence Wait");
    copy_descriptor_set.Reset();
    mix_descriptor_set.Reset();
    downsampling_descriptor_set.Reset();
    upsampling_descriptor_set.Reset();
  }
  if (force_rebuild || !mix_layout) {
    mix_layout = std::make_shared<DescriptorSetLayout>();
    mix_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    mix_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    mix_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    mix_layout->Initialize();
  }
  if (force_rebuild || !copy_layout) {
    copy_layout = std::make_shared<DescriptorSetLayout>();
    copy_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->Initialize();
  }
  if (force_rebuild || !sampling_layout) {
    sampling_layout = std::make_shared<DescriptorSetLayout>();
    sampling_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0);
    sampling_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    sampling_layout->Initialize();
  }
  if (force_rebuild || !downsampling_pipeline) {
    downsampling_pipeline = std::make_shared<ComputePipeline>();
    downsampling_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomDownsampling.comp");
    downsampling_pipeline->descriptor_set_layouts.emplace_back(sampling_layout);
    auto& downsampling_push_constant_range = downsampling_pipeline->push_constant_ranges.emplace_back();
    downsampling_push_constant_range.size = sizeof(DownsamplingPushConstant);
    downsampling_push_constant_range.offset = 0;
    downsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    downsampling_pipeline->Initialize();
  }
  if (force_rebuild || !upsampling_pipeline) {
    upsampling_pipeline = std::make_shared<ComputePipeline>();
    upsampling_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomUpsampling.comp");
    upsampling_pipeline->descriptor_set_layouts.emplace_back(sampling_layout);
    auto& upsampling_push_constant_range = upsampling_pipeline->push_constant_ranges.emplace_back();
    upsampling_push_constant_range.size = sizeof(UpsamplingPushConstant);
    upsampling_push_constant_range.offset = 0;
    upsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    upsampling_pipeline->Initialize();
  }
  if (force_rebuild || !copy_pipeline) {
    copy_pipeline = std::make_shared<ComputePipeline>();
    copy_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomCopy.comp");
    copy_pipeline->descriptor_set_layouts.emplace_back(copy_layout);
    auto& copy_push_constant_range = copy_pipeline->push_constant_ranges.emplace_back();
    copy_push_constant_range.size = sizeof(ComputePushConstant);
    copy_push_constant_range.offset = 0;
    copy_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    copy_pipeline->Initialize();
  }
  if (force_rebuild || !mix_pipeline) {
    mix_pipeline = std::make_shared<ComputePipeline>();
    mix_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomMix.comp");
    mix_pipeline->descriptor_set_layouts.emplace_back(mix_layout);
    auto& mix_push_constant_range = mix_pipeline->push_constant_ranges.emplace_back();
    mix_push_constant_range.size = sizeof(ComputePushConstant);
    mix_push_constant_range.offset = 0;
    mix_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    mix_pipeline->Initialize();
  }
}

void PostProcessingStack::Resize(const glm::uvec2& size) {
  if (size.x == 0 || size.y == 0)
    return;
  if (size.x > 16384 || size.y >= 16384)
    return;
  if (size == current_size)
    return;
  if (current_size != glm::uvec2(1)) {
    Platform::WaitForFrameSubmissions("Required Post-Processing Resize Fence Wait");
  }
  current_size = size;
  const uint32_t mip_levels = static_cast<uint32_t>(std::floor(std::log2(std::max(size.x, size.y)))) + 1;
  source_color_texture->Resize({size.x, size.y, 1});
  result_texture->Resize({size.x, size.y, 1}, mip_levels);
  swap_texture->Resize({size.x, size.y, 1});
}

void PostProcessingStack::OnCreate() {
  if (!blur_layout) {
    blur_layout = std::make_shared<DescriptorSetLayout>();
    blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    blur_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    blur_layout->Initialize();
  }

  current_size = glm::uvec2(1);

  RenderTextureCreateInfo render_texture_create_info{};
  render_texture_create_info.depth = false;
  source_color_texture = std::make_unique<RenderTexture>(render_texture_create_info);
  result_texture = std::make_unique<RenderTexture>(render_texture_create_info);
  swap_texture = std::make_unique<RenderTexture>(render_texture_create_info);
  ambient_occlusion = std::make_shared<AmbientOcclusion>();
  bloom = std::make_shared<Bloom>();
  screen_space_reflection = std::make_shared<ScreenSpaceReflection>();
  anti_aliasing = std::make_shared<AntiAliasing>();
  tone_mapping = std::make_shared<ToneMapping>();
  pipeline_build_step_ = 0;
  pipelines_ready_ = false;
  if (!ApplicationContext::Get().GetLayer<WindowLayer>()) {
    while (!BuildNextPipeline()) {
    }
  }

  enable_ambient_occlusion = true;
  enable_bloom = false;
  enable_screen_space_reflection = false;
  enable_anti_aliasing = true;
  enable_tone_mapping = true;
}

bool PostProcessingStack::BuildNextPipeline() {
  if (pipelines_ready_) {
    return true;
  }
  if (!ambient_occlusion || !bloom || !screen_space_reflection || !anti_aliasing || !tone_mapping) {
    return false;
  }
  switch (pipeline_build_step_) {
    case 0:
      ambient_occlusion->BuildPipelines();
      break;
    case 1:
      screen_space_reflection->BuildPipelines();
      break;
    case 2:
      anti_aliasing->BuildPipelines();
      break;
    case 3:
      bloom->BuildPipelines();
      break;
    case 4:
      tone_mapping->BuildPipelines();
      break;
    default:
      pipelines_ready_ = true;
      return true;
  }
  ++pipeline_build_step_;
  pipelines_ready_ = pipeline_build_step_ > 4;
  return false;
}

void PostProcessingStack::Process(const std::shared_ptr<Camera>& target_camera,
                                  const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process) {
  if (!target_camera) {
    return;
  }
  if (!BuildNextPipeline()) {
    return;
  }
  Resize(target_camera->GetSize());
  const auto& horizontal_descriptor_set = blur_horizontal_descriptor_set.GetOrCreate(blur_layout);
  const auto& vertical_descriptor_set = blur_vertical_descriptor_set.GetOrCreate(blur_layout);
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = result_texture->GetColorSampler()->GetVkSampler();
    horizontal_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = swap_texture->GetColorSampler()->GetVkSampler();
    horizontal_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = swap_texture->GetColorSampler()->GetVkSampler();
    vertical_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = result_texture->GetColorSampler()->GetVkSampler();
    vertical_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  if (pre_process) {
    Platform::RecordCommandsMainQueue(pre_process);
  }

  if (enable_ambient_occlusion) {
    ambient_occlusion->Process(*this, target_camera);
  }
  if (enable_screen_space_reflection) {
    screen_space_reflection->Process(*this, target_camera);
  }
  if (enable_anti_aliasing && anti_aliasing->algorithm == AntiAliasing::Algorithm::Taa) {
    anti_aliasing->Process(*this, target_camera);
  } else {
    anti_aliasing->ResetHistory(target_camera);
  }
  if (enable_bloom) {
    bloom->Process(*this, target_camera);
  }

  if (enable_tone_mapping) {
    tone_mapping->Process(*this, target_camera);
  }
  if (enable_anti_aliasing && anti_aliasing->algorithm == AntiAliasing::Algorithm::Smaa) {
    anti_aliasing->Process(*this, target_camera);
  }
}

void PostProcessingStack::ProcessRayCamera(const std::shared_ptr<Camera>& target_camera,
                                           const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process) {
  if (!target_camera) {
    return;
  }
  if (!BuildNextPipeline()) {
    return;
  }
  Resize(target_camera->GetSize());
  if (pre_process) {
    Platform::RecordCommandsMainQueue(pre_process);
  }
  if (enable_bloom) {
    bloom->Process(*this, target_camera);
  }
  if (enable_tone_mapping) {
    tone_mapping->Process(*this, target_camera);
  }
}

void PostProcessingStack::GaussianBlur(const glm::uvec2& size) const {
  struct PushConstant {
    int horizontal = false;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
  };
  if (!blur_pipeline) {
    blur_pipeline = std::make_shared<ComputePipeline>();
    blur_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/Blur.comp");
    blur_pipeline->descriptor_set_layouts.emplace_back(blur_layout);
    auto& ssr_blur_pipeline_push_constant_range = blur_pipeline->push_constant_ranges.emplace_back();
    ssr_blur_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_blur_pipeline_push_constant_range.offset = 0;
    ssr_blur_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    blur_pipeline->Initialize();
  }

  PushConstant push_constant{};
  const auto& horizontal_descriptor_set = blur_horizontal_descriptor_set.GetOrCreate(blur_layout);
  const auto& vertical_descriptor_set = blur_vertical_descriptor_set.GetOrCreate(blur_layout);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

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
