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
  PushConstant push_constant;
  push_constant.max_distance = max_distance;
  push_constant.max_iteration_count = max_iteration_count;
  push_constant.distance_confidence = distance_confidence;
  push_constant.initial_steps = initial_steps;
  push_constant.thickness = thickness;
  push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  const auto mesh = Resources::GetInstance().GetTexturePassThroughQuad();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = target_camera->GetSize().x;
    render_area.extent.height = target_camera->GetSize().y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(target_camera->GetSize().x);
    viewport.height = static_cast<float>(target_camera->GetSize().y);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = target_camera->GetSize().x;
    scissor.extent.height = target_camera->GetSize().y;
#pragma endregion
    GeometryStorage::BindVertices(vk_command_buffer);

    std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
    VkRenderingInfo render_info2{};
    render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info2.renderArea = render_area;
    render_info2.layerCount = 1;
    render_info2.pDepthAttachment = VK_NULL_HANDLE;

    // Input texture
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    // Attachments
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
    post_processing_stack.result_texture->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();

    Platform::BeginRendering(vk_command_buffer, render_info2);
    reflect_pipeline->states.depth_test = false;
    reflect_pipeline->states.color_blend_attachment_states.clear();
    reflect_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
    for (auto& i : reflect_pipeline->states.color_blend_attachment_states) {
      i.colorWriteMask =
          VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
      i.blendEnable = VK_FALSE;
    }
    reflect_pipeline->Bind(vk_command_buffer);
    reflect_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    reflect_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                        target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    reflect_pipeline->BindDescriptorSet(
        vk_command_buffer, 2, target_camera->GetRenderTexture()->GetColorPresentDescriptorSet()->GetVkDescriptorSet());
    reflect_pipeline->states.view_port = viewport;
    reflect_pipeline->states.scissor = scissor;

    reflect_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    mesh->DrawIndexed(vk_command_buffer, reflect_pipeline->states, 1);
    Platform::EndRendering(vk_command_buffer);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  post_processing_stack.GaussianBlur(target_camera->GetSize());

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = target_camera->GetSize().x;
    render_area.extent.height = target_camera->GetSize().y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(target_camera->GetSize().x);
    viewport.height = static_cast<float>(target_camera->GetSize().y);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = target_camera->GetSize().x;
    scissor.extent.height = target_camera->GetSize().y;
#pragma endregion
    GeometryStorage::BindVertices(vk_command_buffer);

    std::vector<VkRenderingAttachmentInfo> color_attachment_infos{};
    VkRenderingInfo render_info2{};
    render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info2.renderArea = render_area;
    render_info2.layerCount = 1;
    render_info2.pDepthAttachment = VK_NULL_HANDLE;

    // Input texture
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    // Attachments
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);

    target_camera->GetRenderTexture()->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();
    {
      Platform::BeginRendering(vk_command_buffer, render_info2);
      combine_pipeline->states.depth_test = false;
      combine_pipeline->states.color_blend_attachment_states.clear();
      combine_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
      for (auto& i : combine_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      combine_pipeline->Bind(vk_command_buffer);
      combine_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                          render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
      combine_pipeline->BindDescriptorSet(vk_command_buffer, 1, combine_descriptor_set->GetVkDescriptorSet());
      combine_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                          target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
      combine_pipeline->states.view_port = viewport;
      combine_pipeline->states.scissor = scissor;
      combine_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      mesh->DrawIndexed(vk_command_buffer, combine_pipeline->states, 1);
      Platform::EndRendering(vk_command_buffer);
    }
  });
}

void ScreenSpaceReflection::BuildPipelines(const bool force_rebuild) {
  if (force_rebuild || !combine_layout) {
    combine_layout = std::make_shared<DescriptorSetLayout>();
    combine_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
    combine_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
    combine_layout->Initialize();
  }
  if (force_rebuild || !reflect_pipeline) {
    reflect_pipeline = std::make_shared<GraphicsPipeline>();
    reflect_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    reflect_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/SSRReflect.frag");
    reflect_pipeline->geometry_type = GeometryType::Mesh;
    reflect_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    reflect_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    reflect_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTexturePresentDescriptorSetLayout());
    reflect_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
    reflect_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    reflect_pipeline->color_attachment_formats = {2, Platform::Constants::render_texture_color};
    auto& ssr_reflect_pipeline_push_constant_range = reflect_pipeline->push_constant_ranges.emplace_back();
    ssr_reflect_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_reflect_pipeline_push_constant_range.offset = 0;
    ssr_reflect_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    reflect_pipeline->Initialize();
  }
  if (force_rebuild || !combine_pipeline) {
    combine_pipeline = std::make_shared<GraphicsPipeline>();
    combine_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    combine_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/SSRCombine.frag");
    combine_pipeline->geometry_type = GeometryType::Mesh;
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    combine_pipeline->descriptor_set_layouts.emplace_back(combine_layout);
    combine_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetCameraGBufferDescriptorSetLayout());
    combine_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
    combine_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    combine_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    auto& ssr_combine_pipeline_push_constant_range = combine_pipeline->push_constant_ranges.emplace_back();
    ssr_combine_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_combine_pipeline_push_constant_range.offset = 0;
    ssr_combine_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    combine_pipeline->Initialize();
  }
  if (force_rebuild || !combine_descriptor_set) {
    combine_descriptor_set = std::make_shared<DescriptorSet>(combine_layout);
  }
}
