#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
using namespace evo_engine;

namespace {
std::shared_ptr<Sampler> CreateAmbientOcclusionSampler() {
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
  sampler_info.maxLod = 1.0f;
  return std::make_shared<Sampler>(sampler_info);
}

void ComputeWriteToReadBarrier(const VkCommandBuffer command_buffer, const std::shared_ptr<Image>& image) {
  VkImageMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
  barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  barrier.srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT;
  barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  barrier.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
  barrier.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
  barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = image->GetVkImage();
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  barrier.subresourceRange.levelCount = 1;
  barrier.subresourceRange.layerCount = 1;
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.imageMemoryBarrierCount = 1;
  dependency.pImageMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}
}  // namespace

void AmbientOcclusion::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "radius" << YAML::Value << radius;
  out << YAML::Key << "bias" << YAML::Value << bias;
  out << YAML::Key << "intensity" << YAML::Value << intensity;
  out << YAML::Key << "thickness" << YAML::Value << thickness;
  out << YAML::Key << "slice_count" << YAML::Value << slice_count;
  out << YAML::Key << "steps_per_slice" << YAML::Value << steps_per_slice;
  out << YAML::Key << "denoise_radius" << YAML::Value << denoise_radius;
}

void AmbientOcclusion::Deserialize(const YAML::Node& in) {
  if (in["radius"])
    radius = in["radius"].as<float>();
  if (in["bias"])
    bias = in["bias"].as<float>();
  if (in["intensity"])
    intensity = in["intensity"].as<float>();
  if (in["thickness"])
    thickness = in["thickness"].as<float>();
  if (in["slice_count"])
    slice_count = in["slice_count"].as<int>();
  if (in["steps_per_slice"])
    steps_per_slice = in["steps_per_slice"].as<int>();
  if (in["denoise_radius"])
    denoise_radius = in["denoise_radius"].as<float>();
}

void AmbientOcclusion::Process(const PostProcessingStack&, const std::shared_ptr<Camera>& target_camera,
                               PostProcessingExecutionContext& context) const {
  const auto& ambient_occlusion_view = context.ambient_occlusion_image_view;
  const auto& scratch_view = context.ambient_occlusion_scratch_image_view;
  auto& geometry_output_descriptor_set = context.camera.ambient_occlusion.geometry_output_descriptor_set;
  auto& blur_horizontal_descriptor_set = context.camera.ambient_occlusion.blur_horizontal_descriptor_set;
  auto& blur_vertical_descriptor_set = context.camera.ambient_occlusion.blur_vertical_descriptor_set;
  const auto& geometry_output_layout = context.renderer.ambient_occlusion.geometry_output_layout;
  const auto& blur_layout = context.renderer.ambient_occlusion.blur_layout;
  const auto& geometry_pipeline = context.renderer.ambient_occlusion.geometry_pipeline;
  const auto& blur_pipeline = context.renderer.ambient_occlusion.blur_pipeline;
  const auto& sampler = context.renderer.ambient_occlusion.sampler;
  if (!ambient_occlusion_view || !scratch_view || !sampler || !geometry_pipeline || !geometry_pipeline->Initialized() ||
      !blur_pipeline || !blur_pipeline->Initialized()) {
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto& geometry_output_frame_descriptor_set = geometry_output_descriptor_set.GetOrCreate(geometry_output_layout);
  const auto& blur_horizontal_frame_descriptor_set = blur_horizontal_descriptor_set.GetOrCreate(blur_layout);
  const auto& blur_vertical_frame_descriptor_set = blur_vertical_descriptor_set.GetOrCreate(blur_layout);
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = ambient_occlusion_view->GetVkImageView();
    geometry_output_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = ambient_occlusion_view->GetVkImageView();
    image_info.sampler = sampler->GetVkSampler();
    blur_horizontal_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = scratch_view->GetVkImageView();
    blur_horizontal_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = scratch_view->GetVkImageView();
    image_info.sampler = sampler->GetVkSampler();
    blur_vertical_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = ambient_occlusion_view->GetVkImageView();
    blur_vertical_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  const auto size = target_camera->GetSize();
  PushConstant push_constant;
  push_constant.radius = radius;
  push_constant.bias = bias;
  push_constant.intensity = intensity;
  push_constant.slice_count = slice_count;
  push_constant.steps_per_slice = steps_per_slice;
  push_constant.thickness = thickness;
  push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    geometry_pipeline->Bind(vk_command_buffer);
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                         target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                         geometry_output_frame_descriptor_set->GetVkDescriptorSet());
    geometry_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    geometry_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    ComputeWriteToReadBarrier(vk_command_buffer, ambient_occlusion_view->GetImage());

    BlurPushConstant blur_push_constant{};
    blur_push_constant.denoise_radius = denoise_radius;
    blur_push_constant.camera_near = target_camera->camera_settings.near_distance;
    blur_push_constant.camera_far = target_camera->camera_settings.far_distance;
    blur_pipeline->Bind(vk_command_buffer);
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_horizontal_frame_descriptor_set->GetVkDescriptorSet());
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                     target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    blur_push_constant.horizontal = true;
    blur_pipeline->PushConstant(vk_command_buffer, 0, blur_push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
    ComputeWriteToReadBarrier(vk_command_buffer, scratch_view->GetImage());

    blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_vertical_frame_descriptor_set->GetVkDescriptorSet());
    blur_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                     target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    blur_push_constant.horizontal = false;
    blur_pipeline->PushConstant(vk_command_buffer, 0, blur_push_constant);
    blur_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(size.x, 16), Platform::DivUp(size.y, 16));
  });
}

void AmbientOcclusion::BuildPipelines(PostProcessingRendererResources& resources, const bool force_rebuild) const {
  auto& geometry_output_layout = resources.ambient_occlusion.geometry_output_layout;
  auto& blur_layout = resources.ambient_occlusion.blur_layout;
  auto& geometry_pipeline = resources.ambient_occlusion.geometry_pipeline;
  auto& blur_pipeline = resources.ambient_occlusion.blur_pipeline;
  auto& sampler = resources.ambient_occlusion.sampler;
  if (force_rebuild || !geometry_output_layout) {
    geometry_output_layout = std::make_shared<DescriptorSetLayout>();
    geometry_output_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    geometry_output_layout->Initialize();
  }
  if (force_rebuild || !geometry_pipeline) {
    geometry_pipeline = std::make_shared<ComputePipeline>();
    geometry_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/GTAO.slang");
    geometry_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    geometry_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    geometry_pipeline->descriptor_set_layouts.emplace_back(geometry_output_layout);
    auto& ssr_reflect_pipeline_push_constant_range = geometry_pipeline->push_constant_ranges.emplace_back();
    ssr_reflect_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_reflect_pipeline_push_constant_range.offset = 0;
    ssr_reflect_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    geometry_pipeline->Initialize();
  }
  if (force_rebuild || !blur_layout) {
    blur_layout = std::make_shared<DescriptorSetLayout>();
    blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    blur_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    blur_layout->Initialize();
  }

  if (force_rebuild || !blur_pipeline) {
    blur_pipeline = std::make_shared<ComputePipeline>();
    blur_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Resources::GetDefaultResourcesPath() /
                                                         "Shaders/Compute/PostProcessing/AmbientOcclusionBlur.slang");
    blur_pipeline->descriptor_set_layouts.emplace_back(blur_layout);
    blur_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    auto& ssr_blur_pipeline_push_constant_range = blur_pipeline->push_constant_ranges.emplace_back();
    ssr_blur_pipeline_push_constant_range.size = sizeof(BlurPushConstant);
    ssr_blur_pipeline_push_constant_range.offset = 0;
    ssr_blur_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    blur_pipeline->Initialize();
  }
  if (force_rebuild || !sampler) {
    sampler = CreateAmbientOcclusionSampler();
  }
}
