#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
using namespace evo_engine;

bool Bloom::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Filter radius", &filter_radius, 0.0001f, 0.0001f, .1f)) {
    changed = true;
  }
  if (ImGui::DragInt("Chain length", &bloom_chain_length, 1, 0, 10)) {
    changed = true;
  }
  if (ImGui::Button("Rebuild pipelines")) {
    BuildPipelines();
  }
  return changed;
}

void Bloom::Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) {
  if (!copy_pipeline || !copy_pipeline->Initialized() || !downsampling_pipeline ||
      !downsampling_pipeline->Initialized() || !upsampling_pipeline || !upsampling_pipeline->Initialized() ||
      !mix_pipeline || !mix_pipeline->Initialized())
    return;

  const auto render_layer = Application::GetLayer<RenderLayer>();
  const auto mip_levels = post_processing_stack.result_texture->GetMipLevels();
  const auto base_extent = post_processing_stack.result_texture->GetColorImage()->GetExtent();
  const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = target_camera->GetSize().x;
    render_area.extent.height = target_camera->GetSize().y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = target_camera->GetSize().x;
    viewport.height = target_camera->GetSize().y;
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

    vkCmdBeginRendering(vk_command_buffer, &render_info2);
    copy_pipeline->states.depth_test = false;
    copy_pipeline->states.color_blend_attachment_states.clear();
    copy_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
    for (auto& i : copy_pipeline->states.color_blend_attachment_states) {
      i.colorWriteMask =
          VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
      i.blendEnable = VK_FALSE;
    }
    copy_pipeline->Bind(vk_command_buffer);
    copy_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                     render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    copy_pipeline->BindDescriptorSet(
        vk_command_buffer, 1, target_camera->GetRenderTexture()->GetColorPresentDescriptorSet()->GetVkDescriptorSet());
    copy_pipeline->states.view_port = viewport;
    copy_pipeline->states.scissor = scissor;
    mesh->DrawIndexed(vk_command_buffer, copy_pipeline->states, 1);
    vkCmdEndRendering(vk_command_buffer);
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
      const float mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, target_mip_level);
      const float mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, target_mip_level);
      if (mip_width < 1.f || mip_height < 1.f)
        continue;
#pragma region Viewport and scissor
      VkViewport viewport;
      viewport.x = 0.0f;
      viewport.y = 0.0f;
      viewport.width = mip_width;
      viewport.height = mip_height;
      viewport.minDepth = 0.0f;
      viewport.maxDepth = 1.0f;

      VkRect2D scissor;
      scissor.offset = {0, 0};
      scissor.extent.width = mip_width;
      scissor.extent.height = mip_height;
      downsampling_pipeline->states.view_port = viewport;
      downsampling_pipeline->states.scissor = scissor;
#pragma endregion
      GeometryStorage::BindVertices(vk_command_buffer);
      post_processing_stack.result_texture->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE,
          [&]() {
            downsampling_pipeline->states.cull_mode = VK_CULL_MODE_NONE;
            downsampling_pipeline->states.color_blend_attachment_states.clear();
            downsampling_pipeline->states.color_blend_attachment_states.resize(1);
            for (auto& i : downsampling_pipeline->states.color_blend_attachment_states) {
              i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                                 VK_COLOR_COMPONENT_A_BIT;
              i.blendEnable = VK_FALSE;
            }
            downsampling_pipeline->Bind(vk_command_buffer);
            downsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                     render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            downsampling_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                     current_descriptor_set->GetVkDescriptorSet());
            DownsamplingPushConstant push_constant;
            push_constant.mip_level = target_mip_level - 1;
            push_constant.source_resolution = {mip_width, mip_height};
            downsampling_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
            mesh->DrawIndexed(vk_command_buffer, downsampling_pipeline->states, 1);
          },
          target_mip_level);
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

      const float prev_mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, src_mip_level);
      const float prev_mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, src_mip_level);
      if (prev_mip_width < 1.f || prev_mip_height < 1.f)
        continue;

      const float mip_width = static_cast<float>(base_extent.width) * glm::pow(0.5f, src_mip_level - 1);
      const float mip_height = static_cast<float>(base_extent.height) * glm::pow(0.5f, src_mip_level - 1);

#pragma region Viewport and scissor
      VkViewport viewport;
      viewport.x = 0.0f;
      viewport.y = 0.0f;
      viewport.width = mip_width;
      viewport.height = mip_height;
      viewport.minDepth = 0.0f;
      viewport.maxDepth = 1.0f;

      VkRect2D scissor;
      scissor.offset = {0, 0};
      scissor.extent.width = mip_width;
      scissor.extent.height = mip_height;
      upsampling_pipeline->states.view_port = viewport;
      upsampling_pipeline->states.scissor = scissor;
#pragma endregion
      GeometryStorage::BindVertices(vk_command_buffer);
      post_processing_stack.result_texture->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE,
          [&]() {
            upsampling_pipeline->states.cull_mode = VK_CULL_MODE_NONE;
            upsampling_pipeline->states.color_blend_attachment_states.clear();
            upsampling_pipeline->states.color_blend_attachment_states.resize(1);
            for (auto& i : upsampling_pipeline->states.color_blend_attachment_states) {
              i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                                 VK_COLOR_COMPONENT_A_BIT;
              i.blendEnable = VK_FALSE;
            }
            upsampling_pipeline->Bind(vk_command_buffer);
            upsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                   render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            upsampling_pipeline->BindDescriptorSet(vk_command_buffer, 1, current_descriptor_set->GetVkDescriptorSet());

            UpsamplingPushConstant push_constant;
            push_constant.filter_radius = filter_radius;
            upsampling_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
            mesh->DrawIndexed(vk_command_buffer, upsampling_pipeline->states, 1);
          },
          src_mip_level - 1);
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
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = target_camera->GetSize().x;
    render_area.extent.height = target_camera->GetSize().y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = target_camera->GetSize().x;
    viewport.height = target_camera->GetSize().y;
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
      vkCmdBeginRendering(vk_command_buffer, &render_info2);
      mix_pipeline->states.depth_test = false;
      mix_pipeline->states.color_blend_attachment_states.clear();
      mix_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
      for (auto& i : mix_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      mix_pipeline->Bind(vk_command_buffer);
      mix_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                      render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
      mix_pipeline->BindDescriptorSet(vk_command_buffer, 1, mix_descriptor_set->GetVkDescriptorSet());
      mix_pipeline->states.view_port = viewport;
      mix_pipeline->states.scissor = scissor;
      mesh->DrawIndexed(vk_command_buffer, mix_pipeline->states, 1);
      vkCmdEndRendering(vk_command_buffer);
    }
  });
}

void Bloom::BuildPipelines() {
  mix_layout = std::make_shared<DescriptorSetLayout>();
  mix_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  mix_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  mix_layout->Initialize();

  if (!mix_descriptor_set) {
    mix_descriptor_set = std::make_shared<DescriptorSet>(mix_layout);
  }

  sampling_layout = std::make_shared<DescriptorSetLayout>();
  sampling_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  sampling_layout->Initialize();

  downsampling_pipeline = std::make_shared<GraphicsPipeline>();
  downsampling_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                      "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  downsampling_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/BloomDownsampling.frag");

  downsampling_pipeline->geometry_type = GeometryType::Mesh;
  downsampling_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  downsampling_pipeline->descriptor_set_layouts.emplace_back(sampling_layout);
  downsampling_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  downsampling_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  downsampling_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
  auto& downsampling_push_constant_range = downsampling_pipeline->push_constant_ranges.emplace_back();
  downsampling_push_constant_range.size = sizeof(DownsamplingPushConstant);
  downsampling_push_constant_range.offset = 0;
  downsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  downsampling_pipeline->Initialize();

  upsampling_pipeline = std::make_shared<GraphicsPipeline>();
  upsampling_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                      "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  upsampling_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/BloomUpsampling.frag");

  upsampling_pipeline->geometry_type = GeometryType::Mesh;
  upsampling_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  upsampling_pipeline->descriptor_set_layouts.emplace_back(sampling_layout);
  upsampling_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  upsampling_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  upsampling_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
  auto& upsampling_push_constant_range = upsampling_pipeline->push_constant_ranges.emplace_back();
  upsampling_push_constant_range.size = sizeof(UpsamplingPushConstant);
  upsampling_push_constant_range.offset = 0;
  upsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  upsampling_pipeline->Initialize();

  copy_pipeline = std::make_shared<GraphicsPipeline>();
  copy_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                      "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  copy_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/BloomCopy.frag");
  copy_pipeline->geometry_type = GeometryType::Mesh;
  copy_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  copy_pipeline->descriptor_set_layouts.emplace_back(RenderTexture::render_texture_present_layout);
  copy_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  copy_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  copy_pipeline->color_attachment_formats = {2, Platform::Constants::render_texture_color};
  copy_pipeline->Initialize();

  mix_pipeline = std::make_shared<GraphicsPipeline>();
  mix_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                      "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  mix_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/BloomMix.frag");
  mix_pipeline->geometry_type = GeometryType::Mesh;
  mix_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  mix_pipeline->descriptor_set_layouts.emplace_back(mix_layout);
  mix_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  mix_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  mix_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
  mix_pipeline->Initialize();
}

void PostProcessingStack::Resize(const glm::uvec2& size) const {
  if (size.x == 0 || size.y == 0)
    return;
  if (size.x > 16384 || size.y >= 16384)
    return;
  const uint32_t mip_levels = static_cast<uint32_t>(std::floor(std::log2(std::max(size.x, size.y)))) + 1;
  source_color_texture->Resize({size.x, size.y, 1});
  result_texture->Resize({size.x, size.y, 1}, mip_levels);
  swap_texture->Resize({size.x, size.y, 1});
}

void PostProcessingStack::OnCreate() {
  if (!blur_layout) {
    blur_layout = std::make_shared<DescriptorSetLayout>();
    blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    blur_layout->Initialize();
  }

  if (!blur_horizontal_descriptor_set) {
    blur_horizontal_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
  }
  if (!blur_vertical_descriptor_set) {
    blur_vertical_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
  }

  RenderTextureCreateInfo render_texture_create_info{};
  render_texture_create_info.depth = false;
  source_color_texture = std::make_unique<RenderTexture>(render_texture_create_info);
  result_texture = std::make_unique<RenderTexture>(render_texture_create_info);
  swap_texture = std::make_unique<RenderTexture>(render_texture_create_info);
  screen_space_ambient_occlusion = std::make_shared<ScreenSpaceAmbientOcclusion>();
  bloom = std::make_shared<Bloom>();
  screen_space_reflection = std::make_shared<ScreenSpaceReflection>();

  screen_space_ambient_occlusion->BuildPipelines();
  bloom->BuildPipelines();
  screen_space_reflection->BuildPipelines();
}

bool PostProcessingStack::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::Checkbox("SSAO", &enable_screen_space_ambient_occlusion))
    changed = true;

  if (enable_screen_space_ambient_occlusion && ImGui::TreeNodeEx("SSAO", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (screen_space_ambient_occlusion->OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("Bloom", &enable_bloom))
    changed = true;

  if (enable_bloom && ImGui::TreeNodeEx("Bloom", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (bloom->OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("SSR", &enable_screen_space_reflection))
    changed = true;

  if (enable_screen_space_reflection && ImGui::TreeNodeEx("SSR", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (screen_space_reflection->OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Debug")) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
    auto initial_size = ImVec2(source_color_texture->GetExtent().width * debug_scale,
                               source_color_texture->GetExtent().height * debug_scale);
    if (ImGui::TreeNode("Source")) {
      ImGui::Image(source_color_texture->GetColorImTextureId(), initial_size, ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Result")) {
      ImGui::Image(
          result_texture->GetColorImTextureId(),
          ImVec2(result_texture->GetExtent().width * debug_scale, result_texture->GetExtent().height * debug_scale),
          ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Mipmaps")) {
      const auto mip_levels = result_texture->GetMipLevels();
      for (uint32_t mip_level = 1; mip_level < mip_levels; mip_level++) {
        initial_size /= 2.f;
        ImGui::Image(result_texture->GetColorImTextureId(mip_level), initial_size, ImVec2(0, 1), ImVec2(1, 0));
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Swap")) {
      ImGui::Image(
          swap_texture->GetColorImTextureId(),
          ImVec2(swap_texture->GetExtent().width * debug_scale, swap_texture->GetExtent().height * debug_scale),
          ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  return changed;
}

void PostProcessingStack::Process(const std::shared_ptr<Camera>& target_camera) const {
  const auto render_layer = Application::GetLayer<RenderLayer>();

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = result_texture->GetColorSampler()->GetVkSampler();
    blur_horizontal_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = swap_texture->GetColorSampler()->GetVkSampler();
    blur_vertical_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }

  if (enable_screen_space_ambient_occlusion) {
    screen_space_ambient_occlusion->Process(*this, target_camera);
  }
  if (enable_bloom) {
    bloom->Process(*this, target_camera);
  }
  if (enable_screen_space_reflection) {
    screen_space_reflection->Process(*this, target_camera);
  }
}

void PostProcessingStack::GaussianBlur(const glm::uvec2& size) const {
  struct PushConstant {
    int horizontal = false;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
  };
  if (!blur_pipeline) {
    blur_pipeline = std::make_shared<GraphicsPipeline>();
    blur_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    blur_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                                          "Shaders/Graphics/Fragment/PostProcessing/Blur.frag");
    blur_pipeline->geometry_type = GeometryType::Mesh;
    blur_pipeline->descriptor_set_layouts.emplace_back(blur_layout);
    blur_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
    blur_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    blur_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    auto& ssr_blur_pipeline_push_constant_range = blur_pipeline->push_constant_ranges.emplace_back();
    ssr_blur_pipeline_push_constant_range.size = sizeof(PushConstant);
    ssr_blur_pipeline_push_constant_range.offset = 0;
    ssr_blur_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    blur_pipeline->Initialize();
  }

  const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");

  PushConstant push_constant{};

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = size.x;
    render_area.extent.height = size.y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = size.x;
    viewport.height = size.y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = size.x;
    scissor.extent.height = size.y;
#pragma endregion
    GeometryStorage::BindVertices(vk_command_buffer);
    std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
    color_attachment_infos.clear();
    VkRenderingInfo render_info2{};
    render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info2.renderArea = render_area;
    render_info2.layerCount = 1;
    render_info2.pDepthAttachment = VK_NULL_HANDLE;
    result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

    swap_texture->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                             VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();

    {
      vkCmdBeginRendering(vk_command_buffer, &render_info2);
      blur_pipeline->states.depth_test = false;
      blur_pipeline->states.color_blend_attachment_states.clear();
      blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
      for (auto& i : blur_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      blur_pipeline->Bind(vk_command_buffer);
      blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_horizontal_descriptor_set->GetVkDescriptorSet());
      blur_pipeline->states.view_port = viewport;
      blur_pipeline->states.scissor = scissor;
      push_constant.horizontal = true;
      blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      mesh->DrawIndexed(vk_command_buffer, blur_pipeline->states, 1);
      vkCmdEndRendering(vk_command_buffer);
    }

    color_attachment_infos.clear();
    result_texture->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                               VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();
    {
      vkCmdBeginRendering(vk_command_buffer, &render_info2);
      blur_pipeline->states.depth_test = false;
      blur_pipeline->states.color_blend_attachment_states.clear();
      blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
      for (auto& i : blur_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      blur_pipeline->Bind(vk_command_buffer);
      blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_vertical_descriptor_set->GetVkDescriptorSet());
      blur_pipeline->states.view_port = viewport;
      blur_pipeline->states.scissor = scissor;
      push_constant.horizontal = false;
      blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      mesh->DrawIndexed(vk_command_buffer, blur_pipeline->states, 1);
      vkCmdEndRendering(vk_command_buffer);
    }
  });
}
