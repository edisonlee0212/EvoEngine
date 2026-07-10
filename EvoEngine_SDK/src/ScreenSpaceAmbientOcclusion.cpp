#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
using namespace evo_engine;

void AmbientOcclusion::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "algorithm" << YAML::Value << static_cast<int>(algorithm);
  out << YAML::Key << "avoid_distance" << YAML::Value << avoid_distance;
  out << YAML::Key << "kernel_size" << YAML::Value << kernel_size;
  out << YAML::Key << "radius" << YAML::Value << radius;
  out << YAML::Key << "bias" << YAML::Value << bias;
  out << YAML::Key << "factor" << YAML::Value << factor;
  out << YAML::Key << "intensity" << YAML::Value << intensity;
  out << YAML::Key << "gtao_radius" << YAML::Value << gtao_radius;
  out << YAML::Key << "gtao_bias" << YAML::Value << gtao_bias;
  out << YAML::Key << "gtao_intensity" << YAML::Value << gtao_intensity;
  out << YAML::Key << "thickness" << YAML::Value << thickness;
  out << YAML::Key << "slice_count" << YAML::Value << slice_count;
  out << YAML::Key << "steps_per_slice" << YAML::Value << steps_per_slice;
  out << YAML::Key << "denoise_radius" << YAML::Value << denoise_radius;
}

void AmbientOcclusion::Deserialize(const YAML::Node& in) {
  if (in["algorithm"]) {
    const auto value = in["algorithm"].as<int>();
    algorithm = value == static_cast<int>(Algorithm::Ssao) ? Algorithm::Ssao : Algorithm::Gtao;
  }
  if (in["avoid_distance"])
    avoid_distance = in["avoid_distance"].as<float>();
  if (in["kernel_size"])
    kernel_size = in["kernel_size"].as<int>();
  if (in["radius"])
    radius = in["radius"].as<float>();
  if (in["bias"])
    bias = in["bias"].as<float>();
  if (in["factor"])
    factor = in["factor"].as<float>();
  if (in["intensity"])
    intensity = in["intensity"].as<float>();
  if (in["gtao_radius"])
    gtao_radius = in["gtao_radius"].as<float>();
  if (in["gtao_bias"])
    gtao_bias = in["gtao_bias"].as<float>();
  if (in["gtao_intensity"])
    gtao_intensity = in["gtao_intensity"].as<float>();
  if (in["thickness"])
    thickness = in["thickness"].as<float>();
  if (in["slice_count"])
    slice_count = in["slice_count"].as<int>();
  if (in["steps_per_slice"])
    steps_per_slice = in["steps_per_slice"].as<int>();
  if (in["denoise_radius"])
    denoise_radius = in["denoise_radius"].as<float>();
}

void AmbientOcclusion::Process(const PostProcessingStack& post_processing_stack,
                               const std::shared_ptr<Camera>& target_camera) {
  if (!geometry_pipeline || !geometry_pipeline->Initialized() || !blur_pipeline || !blur_pipeline->Initialized() ||
      !combine_pipeline || !combine_pipeline->Initialized())
    return;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    combine_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    combine_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    geometry_output_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    geometry_output_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    blur_horizontal_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.swap_texture->GetColorSampler()->GetVkSampler();
    blur_horizontal_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.swap_texture->GetColorSampler()->GetVkSampler();
    blur_vertical_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    blur_vertical_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  const auto size = target_camera->GetSize();
  PushConstant push_constant;
  push_constant.kernel_size = kernel_size;
  push_constant.radius = algorithm == Algorithm::Gtao ? gtao_radius : radius;
  push_constant.bias = algorithm == Algorithm::Gtao ? gtao_bias : bias;
  push_constant.factor = factor;
  push_constant.intensity = algorithm == Algorithm::Gtao ? gtao_intensity : intensity;
  push_constant.algorithm = static_cast<int>(algorithm);
  push_constant.slice_count = slice_count;
  push_constant.steps_per_slice = steps_per_slice;
  push_constant.thickness = thickness;
  push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    geometry_pipeline->Bind(vk_command_buffer);
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                         target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->BindDescriptorSet(
        vk_command_buffer, 2, target_camera->GetRenderTexture()->GetColorPresentDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 3, geometry_output_descriptor_set->GetVkDescriptorSet());
    geometry_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    geometry_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
  BlurPushConstant blur_push_constant{};
  blur_push_constant.avoid_distance = algorithm == Algorithm::Gtao ? denoise_radius : avoid_distance;
  blur_push_constant.camera_near = target_camera->camera_settings.near_distance;
  blur_push_constant.camera_far = target_camera->camera_settings.far_distance;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

    blur_pipeline->Bind(vk_command_buffer);
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_horizontal_descriptor_set->GetVkDescriptorSet());
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                     target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    blur_push_constant.horizontal = true;
    blur_pipeline->PushConstant(vk_command_buffer, 0, blur_push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);

    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_vertical_descriptor_set->GetVkDescriptorSet());
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                     target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    blur_push_constant.horizontal = false;
    blur_pipeline->PushConstant(vk_command_buffer, 0, blur_push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    combine_pipeline->Bind(vk_command_buffer);
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 0, combine_descriptor_set->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(
        vk_command_buffer, 1, target_camera->GetRenderTexture()->GetStorageDescriptorSet()->GetVkDescriptorSet());
    combine_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void AmbientOcclusion::BuildPipelines(const bool force_rebuild) {
  if (force_rebuild || !combine_layout) {
    combine_layout = std::make_shared<DescriptorSetLayout>();
    combine_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    combine_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    combine_layout->Initialize();
  }
  if (force_rebuild || !geometry_output_layout) {
    geometry_output_layout = std::make_shared<DescriptorSetLayout>();
    geometry_output_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    geometry_output_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    geometry_output_layout->Initialize();
  }
  if (force_rebuild || !geometry_pipeline) {
    geometry_pipeline = std::make_shared<ComputePipeline>();
    geometry_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/AmbientOcclusionGeometry.comp");
    geometry_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    geometry_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    geometry_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTexturePresentDescriptorSetLayout());
    geometry_pipeline->descriptor_set_layouts.emplace_back(geometry_output_layout);
    auto& ssr_reflect_pipeline_push_constant_range = geometry_pipeline->push_constant_ranges.emplace_back();
    ssr_reflect_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_reflect_pipeline_push_constant_range.offset = 0;
    ssr_reflect_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    geometry_pipeline->Initialize();
  }
  if (force_rebuild || !combine_pipeline) {
    combine_pipeline = std::make_shared<ComputePipeline>();
    combine_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/AmbientOcclusionCombine.comp");
    combine_pipeline->descriptor_set_layouts.emplace_back(combine_layout);
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTextureStorageDescriptorSetLayout());
    combine_pipeline->Initialize();
  }
  if (force_rebuild || !combine_descriptor_set) {
    combine_descriptor_set = std::make_shared<DescriptorSet>(combine_layout);
  }
  if (force_rebuild || !geometry_output_descriptor_set) {
    geometry_output_descriptor_set = std::make_shared<DescriptorSet>(geometry_output_layout);
  }

  if (force_rebuild || !blur_layout) {
    blur_layout = std::make_shared<DescriptorSetLayout>();
    blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    blur_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    blur_layout->Initialize();
  }

  if (force_rebuild || !blur_horizontal_descriptor_set) {
    blur_horizontal_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
  }
  if (force_rebuild || !blur_vertical_descriptor_set) {
    blur_vertical_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
  }

  if (force_rebuild || !blur_pipeline) {
    blur_pipeline = std::make_shared<ComputePipeline>();
    blur_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Resources::GetDefaultResourcesPath() /
                                                         "Shaders/Compute/PostProcessing/AmbientOcclusionBlur.comp");
    blur_pipeline->descriptor_set_layouts.emplace_back(blur_layout);
    blur_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    auto& ssr_blur_pipeline_push_constant_range = blur_pipeline->push_constant_ranges.emplace_back();
    ssr_blur_pipeline_push_constant_range.size = sizeof(BlurPushConstant);
    ssr_blur_pipeline_push_constant_range.offset = 0;
    ssr_blur_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    blur_pipeline->Initialize();
  }
}
