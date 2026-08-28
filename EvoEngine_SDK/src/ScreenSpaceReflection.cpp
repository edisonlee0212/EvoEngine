#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
using namespace evo_engine;

void ScreenSpaceReflection::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "max_distance" << YAML::Value << max_distance;
  out << YAML::Key << "distance_confidence" << YAML::Value << distance_confidence;
  out << YAML::Key << "max_iteration_count" << YAML::Value << max_iteration_count;
  out << YAML::Key << "binary_search_iteration_count" << YAML::Value << binary_search_iteration_count;
  out << YAML::Key << "thickness" << YAML::Value << thickness;
  out << YAML::Key << "start_bias" << YAML::Value << start_bias;
  out << YAML::Key << "blur" << YAML::Value << blur;
  out << YAML::Key << "temporal_stabilization" << YAML::Value << temporal_stabilization;
}

void ScreenSpaceReflection::Deserialize(const YAML::Node& in) {
  if (in["max_distance"])
    max_distance = in["max_distance"].as<float>();
  if (in["distance_confidence"])
    distance_confidence = in["distance_confidence"].as<float>();
  if (in["max_iteration_count"])
    max_iteration_count = in["max_iteration_count"].as<int>();
  if (in["binary_search_iteration_count"])
    binary_search_iteration_count = in["binary_search_iteration_count"].as<int>();
  else if (in["initial_steps"])
    binary_search_iteration_count = in["initial_steps"].as<int>();
  if (in["thickness"])
    thickness = in["thickness"].as<float>();
  if (in["start_bias"])
    start_bias = in["start_bias"].as<float>();
  if (in["blur"])
    blur = in["blur"].as<bool>();
  if (in["temporal_stabilization"])
    temporal_stabilization = in["temporal_stabilization"].as<bool>();
}

void ScreenSpaceReflection::Process(const PostProcessingStack& post_processing_stack,
                                    const std::shared_ptr<Camera>& target_camera,
                                    PostProcessingExecutionContext& context) const {
  const auto& source_color_texture = context.camera.stack.source_color_texture;
  const auto& result_texture = context.camera.stack.result_texture;
  auto& combine_descriptor_set = context.camera.screen_space_reflection.combine_descriptor_set;
  auto& reflect_output_descriptor_set = context.camera.screen_space_reflection.reflect_output_descriptor_set;
  auto& spatial_resolve_descriptor_set = context.camera.screen_space_reflection.spatial_resolve_descriptor_set;
  auto& temporal_descriptor_set = context.camera.screen_space_reflection.temporal_descriptor_set;
  const auto& combine_layout = context.renderer.screen_space_reflection.combine_layout;
  const auto& reflect_output_layout = context.renderer.screen_space_reflection.reflect_output_layout;
  const auto& spatial_resolve_layout = context.renderer.screen_space_reflection.spatial_resolve_layout;
  const auto& temporal_layout = context.renderer.screen_space_reflection.temporal_layout;
  const auto& reflect_pipeline = context.renderer.screen_space_reflection.reflect_pipeline;
  const auto& spatial_resolve_pipeline = context.renderer.screen_space_reflection.spatial_resolve_pipeline;
  const auto& temporal_pipeline = context.renderer.screen_space_reflection.temporal_pipeline;
  const auto& combine_pipeline = context.renderer.screen_space_reflection.combine_pipeline;
  if (!reflect_pipeline || !combine_pipeline || !reflect_pipeline->Initialized() || !combine_pipeline->Initialized())
    return;
  if (blur && (!spatial_resolve_pipeline || !spatial_resolve_pipeline->Initialized()))
    return;
  const bool temporal_enabled = temporal_stabilization && debug_mode == DebugMode::None &&
                                context.motion_vectors_image_view && temporal_pipeline &&
                                temporal_pipeline->Initialized();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto render_instances = render_layer->GetCurrentRenderInstanceStorage();
  if (!render_instances)
    return;
  const auto camera_index = render_instances->GetCameraIndex(target_camera->GetHandle());
  const auto raster_lighting_descriptor_set =
      render_layer->GetExistingRasterLightingTextureDescriptorSet(Platform::GetCurrentFrameIndex(), camera_index);
  const auto lighting_descriptor_set = RenderLayer::GetLightingDescriptorSet();
  if (!lighting_descriptor_set || !raster_lighting_descriptor_set)
    return;
  const auto& combine_frame_descriptor_set = combine_descriptor_set.GetOrCreate(combine_layout);
  const auto& reflect_output_frame_descriptor_set = reflect_output_descriptor_set.GetOrCreate(reflect_output_layout);
  const auto spatial_resolve_frame_descriptor_set =
      blur ? spatial_resolve_descriptor_set.GetOrCreate(spatial_resolve_layout) : nullptr;
  const auto temporal_frame_descriptor_set =
      temporal_enabled ? temporal_descriptor_set.GetOrCreate(temporal_layout) : nullptr;

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = source_color_texture->GetColorSampler()->GetVkSampler();
    combine_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = source_color_texture->GetColorSampler()->GetVkSampler();
    reflect_output_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = result_texture->GetColorSampler()->GetVkSampler();
    reflect_output_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  if (blur) {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = result_texture->GetColorSampler()->GetVkSampler();
    spatial_resolve_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = context.camera.stack.swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = context.camera.stack.swap_texture->GetColorSampler()->GetVkSampler();
    spatial_resolve_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  auto& temporal_resources = context.camera.screen_space_reflection;
  const auto current_reflection_texture = blur ? context.camera.stack.swap_texture : result_texture;
  const uint32_t history_read_index = temporal_resources.history_read_index;
  const uint32_t history_write_index = 1u - history_read_index;
  const auto final_reflection_texture =
      temporal_enabled ? temporal_resources.reflection_history[history_write_index] : current_reflection_texture;
  if (temporal_enabled) {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.sampler = current_reflection_texture->GetColorSampler()->GetVkSampler();
    image_info.imageView = current_reflection_texture->GetColorImageView()->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView =
        temporal_resources.reflection_history[history_read_index]->GetColorImageView()->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = context.motion_vectors_image_view->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
    image_info.imageView =
        temporal_resources.geometry_history[history_read_index]->GetColorImageView()->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(3, image_info);
    image_info.imageView =
        temporal_resources.material_history[history_read_index]->GetColorImageView()->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(4, image_info);
    image_info.imageView =
        temporal_resources.reflection_history[history_write_index]->GetColorImageView()->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(5, image_info);
    image_info.imageView =
        temporal_resources.geometry_history[history_write_index]->GetColorImageView()->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(6, image_info);
    image_info.imageView =
        temporal_resources.material_history[history_write_index]->GetColorImageView()->GetVkImageView();
    temporal_frame_descriptor_set->UpdateImageDescriptorBinding(7, image_info);
  }
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = final_reflection_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = final_reflection_texture->GetColorSampler()->GetVkSampler();
    combine_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  PushConstant push_constant;
  push_constant.max_distance = glm::max(max_distance, 0.01f);
  push_constant.max_iteration_count = glm::clamp(max_iteration_count, 1, 256);
  push_constant.distance_confidence = glm::max(distance_confidence, 0.0f);
  push_constant.binary_search_iteration_count = glm::clamp(binary_search_iteration_count, 0, 64);
  push_constant.thickness = glm::max(thickness, 0.0001f);
  push_constant.start_bias = glm::max(start_bias, 0.0f);
  push_constant.debug_mode = static_cast<int32_t>(debug_mode);
  push_constant.temporal_enabled = temporal_enabled ? 1 : 0;
  push_constant.temporal_history_valid = temporal_resources.history_valid ? 1 : 0;
  push_constant.camera_index = camera_index;
  RenderInstancePushConstant combine_push_constant;
  combine_push_constant.camera_index = camera_index;
  combine_push_constant.meshlet_culling_flags = static_cast<uint32_t>(debug_mode);
  const auto resolution = target_camera->GetSize();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    reflect_pipeline->Bind(vk_command_buffer);
    reflect_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    reflect_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                        target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    reflect_pipeline->BindDescriptorSet(
        vk_command_buffer, 2, target_camera->GetRenderTexture()->GetColorPresentDescriptorSet()->GetVkDescriptorSet());
    reflect_pipeline->BindDescriptorSet(vk_command_buffer, 3,
                                        reflect_output_frame_descriptor_set->GetVkDescriptorSet());
    reflect_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    reflect_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(resolution.x, 16), Platform::DivUp(resolution.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  if (blur) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
      result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      context.camera.stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                             VK_IMAGE_LAYOUT_GENERAL);
      spatial_resolve_pipeline->Bind(vk_command_buffer);
      spatial_resolve_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                  spatial_resolve_frame_descriptor_set->GetVkDescriptorSet());
      spatial_resolve_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                  target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
      spatial_resolve_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      spatial_resolve_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(resolution.x, 16),
                                         Platform::DivUp(resolution.y, 16));
      Platform::EverythingBarrier(vk_command_buffer);
    });
  }

  if (temporal_enabled) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
      current_reflection_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      context.motion_vectors_image_view->GetImage()->TransitImageLayout(vk_command_buffer,
                                                                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
      for (const auto& texture : temporal_resources.reflection_history)
        texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      for (const auto& texture : temporal_resources.geometry_history)
        texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      for (const auto& texture : temporal_resources.material_history)
        texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      temporal_pipeline->Bind(vk_command_buffer);
      temporal_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
      temporal_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
      temporal_pipeline->BindDescriptorSet(vk_command_buffer, 2, temporal_frame_descriptor_set->GetVkDescriptorSet());
      temporal_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      temporal_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(resolution.x, 16),
                                  Platform::DivUp(resolution.y, 16));
      Platform::EverythingBarrier(vk_command_buffer);
    });
    temporal_resources.history_read_index = history_write_index;
    temporal_resources.history_valid = true;
  } else {
    temporal_resources.history_valid = false;
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

    combine_pipeline->Bind(vk_command_buffer);
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 2, lighting_descriptor_set->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                        target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 3, raster_lighting_descriptor_set->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 4, combine_frame_descriptor_set->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(
        vk_command_buffer, 5, target_camera->GetRenderTexture()->GetStorageDescriptorSet()->GetVkDescriptorSet());
    combine_pipeline->PushConstant(vk_command_buffer, 0, combine_push_constant);
    combine_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(resolution.x, 16), Platform::DivUp(resolution.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void ScreenSpaceReflection::BuildPipelines(PostProcessingRendererResources& resources, const bool) const {
  constexpr bool force_rebuild = false;
  auto& combine_layout = resources.screen_space_reflection.combine_layout;
  auto& reflect_output_layout = resources.screen_space_reflection.reflect_output_layout;
  auto& spatial_resolve_layout = resources.screen_space_reflection.spatial_resolve_layout;
  auto& temporal_layout = resources.screen_space_reflection.temporal_layout;
  auto& reflect_pipeline = resources.screen_space_reflection.reflect_pipeline;
  auto& spatial_resolve_pipeline = resources.screen_space_reflection.spatial_resolve_pipeline;
  auto& temporal_pipeline = resources.screen_space_reflection.temporal_pipeline;
  auto& combine_pipeline = resources.screen_space_reflection.combine_pipeline;
  if (force_rebuild || !combine_layout) {
    combine_layout = std::make_shared<DescriptorSetLayout>();
    combine_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    combine_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    combine_layout->Initialize();
  }
  if (force_rebuild || !reflect_output_layout) {
    reflect_output_layout = std::make_shared<DescriptorSetLayout>();
    reflect_output_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    reflect_output_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    reflect_output_layout->Initialize();
  }
  if (force_rebuild || !spatial_resolve_layout) {
    spatial_resolve_layout = std::make_shared<DescriptorSetLayout>();
    spatial_resolve_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                  VK_SHADER_STAGE_COMPUTE_BIT, 0);
    spatial_resolve_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    spatial_resolve_layout->Initialize();
  }
  if (force_rebuild || !temporal_layout) {
    temporal_layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 5; ++binding)
      temporal_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                             VK_SHADER_STAGE_COMPUTE_BIT, 0);
    for (uint32_t binding = 5; binding < 8; ++binding)
      temporal_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    temporal_layout->Initialize();
  }
  if (force_rebuild || !reflect_pipeline) {
    reflect_pipeline = std::make_shared<ComputePipeline>();
    reflect_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SSRReflect.slang");
    reflect_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    reflect_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    reflect_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTexturePresentDescriptorSetLayout());
    reflect_pipeline->descriptor_set_layouts.emplace_back(reflect_output_layout);
    auto& ssr_reflect_pipeline_push_constant_range = reflect_pipeline->push_constant_ranges.emplace_back();
    ssr_reflect_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_reflect_pipeline_push_constant_range.offset = 0;
    ssr_reflect_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    reflect_pipeline->Initialize();
  }
  if (force_rebuild || !spatial_resolve_pipeline) {
    spatial_resolve_pipeline = std::make_shared<ComputePipeline>();
    spatial_resolve_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SSRSpatialResolve.slang");
    spatial_resolve_pipeline->descriptor_set_layouts.emplace_back(spatial_resolve_layout);
    spatial_resolve_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    auto& push_constant_range = spatial_resolve_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    spatial_resolve_pipeline->Initialize();
  }
  if (force_rebuild || !temporal_pipeline) {
    temporal_pipeline = std::make_shared<ComputePipeline>();
    temporal_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SSRTemporalResolve.slang");
    temporal_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    temporal_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    temporal_pipeline->descriptor_set_layouts.emplace_back(temporal_layout);
    auto& push_constant_range = temporal_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    temporal_pipeline->Initialize();
  }
  if (force_rebuild || !combine_pipeline) {
    combine_pipeline = std::make_shared<ComputePipeline>();
    combine_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SSRCombine.slang");
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetLightingDescriptorSetLayout());
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRasterLightingTextureDescriptorSetLayout());
    combine_pipeline->descriptor_set_layouts.emplace_back(combine_layout);
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTextureStorageDescriptorSetLayout());
    auto& ssr_combine_pipeline_push_constant_range = combine_pipeline->push_constant_ranges.emplace_back();
    ssr_combine_pipeline_push_constant_range.size = sizeof(RenderInstancePushConstant);
    ssr_combine_pipeline_push_constant_range.offset = 0;
    ssr_combine_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    combine_pipeline->Initialize();
  }
}
