#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;

void TemporalAntiAliasing::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "feedback" << YAML::Value << feedback;
  out << YAML::Key << "clamp_strength" << YAML::Value << clamp_strength;
  out << YAML::Key << "sharpen" << YAML::Value << sharpen;
}

void TemporalAntiAliasing::Deserialize(const YAML::Node& in) {
  if (in["feedback"])
    feedback = in["feedback"].as<float>();
  if (in["clamp_strength"])
    clamp_strength = in["clamp_strength"].as<float>();
  if (in["sharpen"])
    sharpen = in["sharpen"].as<float>();
}

void TemporalAntiAliasing::Process(const PostProcessingStack& post_processing_stack,
                                   const std::shared_ptr<Camera>& target_camera) {
  if (!copy_pipeline || !copy_pipeline->Initialized() || !resolve_pipeline || !resolve_pipeline->Initialized()) {
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
  const bool settings_changed =
      history.valid &&
      (history.feedback != feedback || history.clamp_strength != clamp_strength || history.sharpen != sharpen);
  const bool history_valid =
      history.valid && !reset_history && !skipped_frame && !settings_changed && target_camera->GetFrameCount() != 0u;
  reset_history = false;

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

  PushConstant push_constant{};
  push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  push_constant.history_valid = history_valid ? 1 : 0;
  push_constant.feedback = glm::clamp(feedback, 0.0f, 0.98f);
  push_constant.clamp_strength = glm::max(clamp_strength, 0.0f);
  push_constant.sharpen = glm::max(sharpen, 0.0f);
  push_constant.debug_mode = static_cast<int32_t>(debug_mode);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
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
    resolve_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  history.valid = true;
  history.frame_index++;
  history.last_processed_frame = current_frame_index;
  history.feedback = feedback;
  history.clamp_strength = clamp_strength;
  history.sharpen = sharpen;
}

void TemporalAntiAliasing::ResetHistory(const std::shared_ptr<Camera>& target_camera) {
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

void TemporalAntiAliasing::PruneHistory(const uint32_t current_frame_index, const uint64_t active_camera_handle) {
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

void TemporalAntiAliasing::BuildPipelines(const bool force_rebuild) {
  if (force_rebuild || !copy_layout) {
    copy_layout = std::make_shared<DescriptorSetLayout>();
    copy_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->Initialize();
  }
  if (force_rebuild || !resolve_layout) {
    resolve_layout = std::make_shared<DescriptorSetLayout>();
    resolve_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    resolve_layout->Initialize();
  }
  if (force_rebuild || !copy_descriptor_set) {
    copy_descriptor_set = std::make_shared<DescriptorSet>(copy_layout);
  }
  if (force_rebuild || !resolve_descriptor_set) {
    resolve_descriptor_set = std::make_shared<DescriptorSet>(resolve_layout);
  }
  if (force_rebuild || !copy_pipeline) {
    copy_pipeline = std::make_shared<ComputePipeline>();
    copy_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/TAACopy.comp");
    copy_pipeline->descriptor_set_layouts.emplace_back(copy_layout);
    copy_pipeline->Initialize();
  }
  if (force_rebuild || !resolve_pipeline) {
    resolve_pipeline = std::make_shared<ComputePipeline>();
    resolve_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/TAAResolve.comp");
    resolve_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    resolve_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    resolve_pipeline->descriptor_set_layouts.emplace_back(resolve_layout);
    auto& push_constant_range = resolve_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    resolve_pipeline->Initialize();
  }
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

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    copy_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    copy_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    copy_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    copy_pipeline->Bind(vk_command_buffer);
    copy_pipeline->BindDescriptorSet(vk_command_buffer, 0, copy_descriptor_set->GetVkDescriptorSet());
    ComputePushConstant push_constant;
    push_constant.resolution = target_size;
    copy_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    copy_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_size.x, 16), Platform::DivUp(target_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  downsampling_descriptor_set.clear();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    for (int target_mip_level = 1; target_mip_level < glm::min(static_cast<int>(mip_levels), bloom_chain_length + 1);
         ++target_mip_level) {
      const auto current_descriptor_set =
          downsampling_descriptor_set.emplace_back(std::make_shared<DescriptorSet>(sampling_layout));

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

  upsampling_descriptor_set.clear();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    for (int src_mip_level = glm::min(static_cast<int>(mip_levels), bloom_chain_length + 1) - 1; src_mip_level > 0;
         --src_mip_level) {
      const auto current_descriptor_set =
          upsampling_descriptor_set.emplace_back(std::make_shared<DescriptorSet>(sampling_layout));

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
    mix_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    mix_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    mix_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    mix_pipeline->Bind(vk_command_buffer);
    mix_pipeline->BindDescriptorSet(vk_command_buffer, 0, mix_descriptor_set->GetVkDescriptorSet());
    ComputePushConstant push_constant;
    push_constant.resolution = target_size;
    mix_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    mix_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_size.x, 16), Platform::DivUp(target_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void Bloom::BuildPipelines(const bool force_rebuild) {
  if (force_rebuild || !mix_layout) {
    mix_layout = std::make_shared<DescriptorSetLayout>();
    mix_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    mix_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    mix_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    mix_layout->Initialize();
  }
  if (force_rebuild || !mix_descriptor_set) {
    mix_descriptor_set = std::make_shared<DescriptorSet>(mix_layout);
  }
  if (force_rebuild || !copy_layout) {
    copy_layout = std::make_shared<DescriptorSetLayout>();
    copy_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    copy_layout->Initialize();
  }
  if (force_rebuild || !copy_descriptor_set) {
    copy_descriptor_set = std::make_shared<DescriptorSet>(copy_layout);
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

  if (!blur_horizontal_descriptor_set) {
    blur_horizontal_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
  }
  if (!blur_vertical_descriptor_set) {
    blur_vertical_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
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
  temporal_anti_aliasing = std::make_shared<TemporalAntiAliasing>();
  tone_mapping = std::make_shared<ToneMapping>();
  pipeline_build_step_ = 0;
  pipelines_ready_ = false;
  if (!ApplicationContext::Get().GetLayer<WindowLayer>()) {
    while (!BuildNextPipeline()) {
    }
  }

  enable_ambient_occlusion = true;
  enable_bloom = true;
  enable_screen_space_reflection = true;
  enable_temporal_anti_aliasing = true;
  enable_tone_mapping = true;
}

bool PostProcessingStack::BuildNextPipeline() {
  if (pipelines_ready_) {
    return true;
  }
  if (!ambient_occlusion || !bloom || !screen_space_reflection || !temporal_anti_aliasing || !tone_mapping) {
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
      temporal_anti_aliasing->BuildPipelines();
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
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = result_texture->GetColorSampler()->GetVkSampler();
    blur_horizontal_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = swap_texture->GetColorSampler()->GetVkSampler();
    blur_horizontal_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = swap_texture->GetColorSampler()->GetVkSampler();
    blur_vertical_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = result_texture->GetColorSampler()->GetVkSampler();
    blur_vertical_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
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
  if (enable_temporal_anti_aliasing) {
    temporal_anti_aliasing->Process(*this, target_camera);
  } else {
    temporal_anti_aliasing->ResetHistory(target_camera);
  }
  if (enable_bloom) {
    bloom->Process(*this, target_camera);
  }

  if (enable_tone_mapping) {
    tone_mapping->Process(*this, target_camera);
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

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

    blur_pipeline->Bind(vk_command_buffer);
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_horizontal_descriptor_set->GetVkDescriptorSet());
    push_constant.horizontal = true;
    blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);

    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_vertical_descriptor_set->GetVkDescriptorSet());
    push_constant.horizontal = false;
    blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}
