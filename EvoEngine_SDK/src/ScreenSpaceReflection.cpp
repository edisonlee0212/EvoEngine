#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
using namespace evo_engine;

void ScreenSpaceReflection::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "max_distance" << YAML::Value << max_distance;
  out << YAML::Key << "distance_confidence" << YAML::Value << distance_confidence;
  out << YAML::Key << "max_iteration_count" << YAML::Value << max_iteration_count;
  out << YAML::Key << "initial_steps" << YAML::Value << initial_steps;
  out << YAML::Key << "thickness" << YAML::Value << thickness;
  out << YAML::Key << "blur" << YAML::Value << blur;
}

void ScreenSpaceReflection::Deserialize(const YAML::Node& in) {
  if (in["max_distance"])
    max_distance = in["max_distance"].as<float>();
  if (in["distance_confidence"])
    distance_confidence = in["distance_confidence"].as<float>();
  if (in["max_iteration_count"])
    max_iteration_count = in["max_iteration_count"].as<int>();
  if (in["initial_steps"])
    initial_steps = in["initial_steps"].as<int>();
  if (in["thickness"])
    thickness = in["thickness"].as<float>();
  if (in["blur"])
    blur = in["blur"].as<bool>();
}

void ScreenSpaceReflection::Process(const PostProcessingStack& post_processing_stack,
                                    const std::shared_ptr<Camera>& target_camera) {
  if (!reflect_pipeline || !combine_pipeline || !reflect_pipeline->Initialized() || !combine_pipeline->Initialized())
    return;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto& combine_frame_descriptor_set = combine_descriptor_set.GetOrCreate(combine_layout);
  const auto& reflect_output_frame_descriptor_set = reflect_output_descriptor_set.GetOrCreate(reflect_output_layout);

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    combine_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    combine_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    reflect_output_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    reflect_output_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  PushConstant push_constant;
  push_constant.max_distance = max_distance;
  push_constant.max_iteration_count = max_iteration_count;
  push_constant.distance_confidence = distance_confidence;
  push_constant.initial_steps = initial_steps;
  push_constant.thickness = thickness;
  push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  const auto resolution = target_camera->GetSize();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
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

  post_processing_stack.GaussianBlur(target_camera->GetSize());

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);

    combine_pipeline->Bind(vk_command_buffer);
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 1, combine_frame_descriptor_set->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                        target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    combine_pipeline->BindDescriptorSet(
        vk_command_buffer, 3, target_camera->GetRenderTexture()->GetStorageDescriptorSet()->GetVkDescriptorSet());
    combine_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    combine_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(resolution.x, 16), Platform::DivUp(resolution.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void ScreenSpaceReflection::BuildPipelines(const bool force_rebuild) {
  if (force_rebuild && (reflect_pipeline || combine_pipeline)) {
    Platform::WaitForFrameSubmissions("Required Post-Processing Pipeline Rebuild Fence Wait");
    combine_descriptor_set.Reset();
    reflect_output_descriptor_set.Reset();
  }
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
  if (force_rebuild || !reflect_pipeline) {
    reflect_pipeline = std::make_shared<ComputePipeline>();
    reflect_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SSRReflect.comp");
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
  if (force_rebuild || !combine_pipeline) {
    combine_pipeline = std::make_shared<ComputePipeline>();
    combine_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SSRCombine.comp");
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    combine_pipeline->descriptor_set_layouts.emplace_back(combine_layout);
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTextureStorageDescriptorSetLayout());
    auto& ssr_combine_pipeline_push_constant_range = combine_pipeline->push_constant_ranges.emplace_back();
    ssr_combine_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_combine_pipeline_push_constant_range.offset = 0;
    ssr_combine_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    combine_pipeline->Initialize();
  }
}
