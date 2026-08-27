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

VkFormat ResolveBloomFormat() {
  constexpr VkFormatFeatureFlags2 required_features = VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_BIT |
                                                      VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_FILTER_LINEAR_BIT |
                                                      VK_FORMAT_FEATURE_2_STORAGE_IMAGE_BIT;
  const auto properties = Platform::GetPhysicalDeviceFormatProperties(VK_FORMAT_R16G16B16A16_SFLOAT);
  return (properties.optimalTilingFeatures & required_features) == required_features ? VK_FORMAT_R16G16B16A16_SFLOAT
                                                                                     : VK_FORMAT_R32G32B32A32_SFLOAT;
}

glm::uvec2 BloomMipSize(const glm::uvec2 base_size, const uint32_t mip_level) {
  return glm::max(base_size >> mip_level, glm::uvec2(1));
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

void PerFrameDescriptorSetList::Reset() {
  slots_.clear();
}

void PostProcessingCameraResources::ResetTemporalState() {
  previous_inverse_projection = glm::mat4(1.0f);
  previous_inverse_view = glm::mat4(1.0f);
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
  ambient_occlusion.geometry_output_descriptor_set.Retain(transient_resources);
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
  out << YAML::Key << "preset" << YAML::Value << static_cast<int32_t>(preset);
}

void AntiAliasing::Deserialize(const YAML::Node& in) {
  *this = AntiAliasing{};
  if (in["preset"]) {
    preset = static_cast<Preset>(in["preset"].as<int32_t>());
  }
  NormalizeSettings();
}

void AntiAliasing::NormalizeSettings() {
  const auto value = static_cast<int32_t>(preset);
  if (value < static_cast<int32_t>(Preset::Low) || value > static_cast<int32_t>(Preset::Ultra)) {
    preset = Preset::Ultra;
  }
  const auto debug = static_cast<int32_t>(debug_mode);
  if (debug < static_cast<int32_t>(DebugMode::None) || debug > static_cast<int32_t>(DebugMode::BlendWeights)) {
    debug_mode = DebugMode::None;
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

void AntiAliasing::BuildPipelines(PostProcessingRendererResources& resources, const bool force_rebuild) const {
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
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/SMAAPrepare.slang");
    renderer.smaa_prepare_pipeline->descriptor_set_layouts.emplace_back(renderer.smaa_prepare_layout);
    auto& push_constant_range = renderer.smaa_prepare_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
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
    push_constant_range.size = sizeof(PushConstant);
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
        shader_root / "Vertex/PostProcessing/SMAAEdge.slang", shader_root / "Fragment/PostProcessing/SMAAEdge.slang",
        renderer.smaa_edge_layout, VK_FORMAT_R8G8B8A8_UNORM, defines);
    renderer.smaa_weight_pipelines[preset_index] =
        create_pipeline(shader_root / "Vertex/PostProcessing/SMAABlendWeight.slang",
                        shader_root / "Fragment/PostProcessing/SMAABlendWeight.slang", renderer.smaa_weight_layout,
                        VK_FORMAT_R8G8B8A8_UNORM, defines);
  }
  if (!renderer.smaa_neighborhood_pipeline) {
    renderer.smaa_neighborhood_pipeline = create_pipeline(
        shader_root / "Vertex/PostProcessing/SMAANeighborhood.slang",
        shader_root / "Fragment/PostProcessing/SMAANeighborhood.slang", renderer.smaa_neighborhood_layout,
        Platform::Constants::render_texture_color, Platform::GetShaderGlobalDefines());
  }
}

void AntiAliasing::Process(const PostProcessingStack& post_processing_stack,
                           const std::shared_ptr<Camera>& target_camera,
                           PostProcessingExecutionContext& context) const {
  auto& camera = context.camera.anti_aliasing;
  auto& renderer = context.renderer.anti_aliasing;
  const auto preset_index = static_cast<size_t>(preset);
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

  PushConstant push_constant{};
  push_constant.metrics = {1.0f / static_cast<float>(size.x), 1.0f / static_cast<float>(size.y),
                           static_cast<float>(size.x), static_cast<float>(size.y)};
  push_constant.tone_mapped = post_processing_stack.enable_tone_mapping ? 1 : 0;
  push_constant.debug_mode = static_cast<int32_t>(debug_mode);

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
  out << YAML::Key << "threshold" << YAML::Value << threshold;
  out << YAML::Key << "knee" << YAML::Value << knee;
  out << YAML::Key << "intensity" << YAML::Value << intensity;
}

void Bloom::Deserialize(const YAML::Node& in) {
  if (in["filter_radius"])
    filter_radius = in["filter_radius"].as<float>();
  if (in["threshold"])
    threshold = in["threshold"].as<float>();
  if (in["knee"])
    knee = in["knee"].as<float>();
  if (in["intensity"])
    intensity = in["intensity"].as<float>();
}

void Bloom::Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
                    PostProcessingExecutionContext& context) const {
  auto& stack = context.camera.stack;
  auto& camera = context.camera.bloom;
  auto& renderer = context.renderer.bloom;
  const auto record_commands = [&](const std::function<void(VkCommandBuffer)>& action) {
    if (context.record_commands) {
      context.record_commands(action);
    } else {
      Platform::RecordCommandsMainQueue(action);
    }
  };
  if (!renderer.copy_pipeline || !renderer.copy_pipeline->Initialized() || !renderer.downsampling_pipeline ||
      !renderer.downsampling_pipeline->Initialized() || !renderer.upsampling_pipeline ||
      !renderer.upsampling_pipeline->Initialized() || !renderer.mix_pipeline || !renderer.mix_pipeline->Initialized())
    return;

  if (!camera.downsample_texture_a || !camera.downsample_texture_b || !camera.upsample_texture) {
    return;
  }
  const auto mip_levels = camera.downsample_texture_a->GetMipLevels();
  const uint32_t processed_mip_count = mip_levels > 2 ? mip_levels - 2 : 1;
  const auto bloom_size = camera.size;
  const auto target_size = target_camera->GetSize();
  const auto& copy_frame_descriptor_set = camera.copy_descriptor_set.GetOrCreate(renderer.copy_layout);
  const auto& mix_frame_descriptor_set = camera.mix_descriptor_set.GetOrCreate(renderer.mix_layout);
  auto& downsampling_frame_descriptor_sets = camera.downsampling_descriptor_sets.Get();
  auto& upsampling_frame_descriptor_sets = camera.upsampling_descriptor_sets.Get();
  const auto acquire_sampling_descriptor_set = [&](auto& descriptor_sets, const size_t index,
                                                   const std::shared_ptr<DescriptorSetLayout>& layout) {
    descriptor_sets.resize(std::max(descriptor_sets.size(), index + 1));
    auto& descriptor_set = descriptor_sets[index];
    if (!descriptor_set) {
      descriptor_set = std::make_shared<DescriptorSet>(layout);
    }
    return descriptor_set;
  };

  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = stack.source_color_texture->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = camera.downsample_texture_a->GetColorImageView(0)->GetVkImageView();
    image_info.sampler = camera.downsample_texture_a->GetColorSampler()->GetVkSampler();
    copy_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }

  record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto target_image = target_camera->GetRenderTexture()->GetColorImage();
    const auto source_image = stack.source_color_texture->GetColorImage();
    target_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    source_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    VkImageCopy copy_region{};
    copy_region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_region.srcSubresource.layerCount = 1;
    copy_region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_region.dstSubresource.layerCount = 1;
    copy_region.extent = {target_size.x, target_size.y, 1};
    vkCmdCopyImage(vk_command_buffer, target_image->GetVkImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                   source_image->GetVkImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy_region);
    target_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    source_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    camera.downsample_texture_a->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    camera.downsample_texture_b->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    camera.upsample_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    renderer.copy_pipeline->Bind(vk_command_buffer);
    renderer.copy_pipeline->BindDescriptorSet(vk_command_buffer, 0, copy_frame_descriptor_set->GetVkDescriptorSet());
    PrefilterPushConstant push_constant;
    push_constant.source_resolution = target_size;
    push_constant.target_resolution = bloom_size;
    push_constant.threshold = threshold;
    push_constant.knee = knee;
    renderer.copy_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    renderer.copy_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(bloom_size.x, 16),
                                     Platform::DivUp(bloom_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);

    size_t descriptor_index = 0;
    for (uint32_t target_mip = 1; target_mip < processed_mip_count; ++target_mip) {
      const auto source_size = BloomMipSize(bloom_size, target_mip - 1);
      const auto destination_size = BloomMipSize(bloom_size, target_mip);
      const std::array<std::shared_ptr<RenderTexture>, 2> inputs = {camera.downsample_texture_a,
                                                                    camera.downsample_texture_b};
      const std::array<std::shared_ptr<RenderTexture>, 2> outputs = {camera.downsample_texture_b,
                                                                     camera.downsample_texture_a};
      for (size_t pass = 0; pass < inputs.size(); ++pass) {
        const auto descriptor_set = acquire_sampling_descriptor_set(downsampling_frame_descriptor_sets,
                                                                    descriptor_index++, renderer.sampling_layout);
        VkDescriptorImageInfo image_info{};
        image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
        image_info.imageView =
            inputs[pass]->GetColorImageView(pass == 0 ? target_mip - 1 : target_mip)->GetVkImageView();
        image_info.sampler = inputs[pass]->GetColorSampler()->GetVkSampler();
        descriptor_set->UpdateImageDescriptorBinding(0, image_info);
        image_info.imageView = outputs[pass]->GetColorImageView(target_mip)->GetVkImageView();
        image_info.sampler = outputs[pass]->GetColorSampler()->GetVkSampler();
        descriptor_set->UpdateImageDescriptorBinding(1, image_info);
        renderer.downsampling_pipeline->Bind(vk_command_buffer);
        renderer.downsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
        DownsamplingPushConstant downsampling;
        downsampling.source_resolution = pass == 0 ? source_size : destination_size;
        downsampling.target_resolution = destination_size;
        downsampling.apply_karis = target_mip == 1 && pass == 0 ? 1 : 0;
        renderer.downsampling_pipeline->PushConstant(vk_command_buffer, 0, downsampling);
        renderer.downsampling_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(destination_size.x, 16),
                                                 Platform::DivUp(destination_size.y, 16));
        Platform::EverythingBarrier(vk_command_buffer);
      }
    }

    descriptor_index = 0;
    for (uint32_t target_mip = processed_mip_count - 1; target_mip-- > 0;) {
      const auto target_mip_size = BloomMipSize(bloom_size, target_mip);
      const auto source_mip_size = BloomMipSize(bloom_size, target_mip + 1);
      const auto descriptor_set = acquire_sampling_descriptor_set(upsampling_frame_descriptor_sets, descriptor_index++,
                                                                  renderer.upsampling_layout);
      VkDescriptorImageInfo image_info{};
      image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
      image_info.imageView = camera.downsample_texture_a->GetColorImageView(target_mip)->GetVkImageView();
      image_info.sampler = camera.downsample_texture_a->GetColorSampler()->GetVkSampler();
      descriptor_set->UpdateImageDescriptorBinding(0, image_info);
      const auto bloom_source =
          target_mip == processed_mip_count - 2 ? camera.downsample_texture_a : camera.upsample_texture;
      image_info.imageView = bloom_source->GetColorImageView(target_mip + 1)->GetVkImageView();
      image_info.sampler = bloom_source->GetColorSampler()->GetVkSampler();
      descriptor_set->UpdateImageDescriptorBinding(1, image_info);
      image_info.imageView = camera.upsample_texture->GetColorImageView(target_mip)->GetVkImageView();
      image_info.sampler = camera.upsample_texture->GetColorSampler()->GetVkSampler();
      descriptor_set->UpdateImageDescriptorBinding(2, image_info);
      renderer.upsampling_pipeline->Bind(vk_command_buffer);
      renderer.upsampling_pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
      UpsamplingPushConstant upsampling;
      upsampling.source_resolution = source_mip_size;
      upsampling.target_resolution = target_mip_size;
      upsampling.filter_radius = filter_radius;
      renderer.upsampling_pipeline->PushConstant(vk_command_buffer, 0, upsampling);
      renderer.upsampling_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_mip_size.x, 16),
                                             Platform::DivUp(target_mip_size.y, 16));
      Platform::EverythingBarrier(vk_command_buffer);
    }

    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = stack.source_color_texture->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    const auto bloom_result = processed_mip_count > 1 ? camera.upsample_texture : camera.downsample_texture_a;
    image_info.imageView = bloom_result->GetColorImageView(0)->GetVkImageView();
    image_info.sampler = bloom_result->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    mix_frame_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
    target_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    renderer.mix_pipeline->Bind(vk_command_buffer);
    renderer.mix_pipeline->BindDescriptorSet(vk_command_buffer, 0, mix_frame_descriptor_set->GetVkDescriptorSet());
    MixPushConstant mix_push_constant;
    mix_push_constant.resolution = target_size;
    mix_push_constant.bloom_resolution = bloom_size;
    mix_push_constant.intensity = intensity;
    renderer.mix_pipeline->PushConstant(vk_command_buffer, 0, mix_push_constant);
    renderer.mix_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(target_size.x, 16),
                                    Platform::DivUp(target_size.y, 16));
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void Bloom::BuildPipelines(PostProcessingRendererResources& resources, const bool force_rebuild) const {
  auto& renderer = resources.bloom;
  const auto format = ResolveBloomFormat();
  if (force_rebuild || renderer.format != format) {
    renderer.downsampling_pipeline.reset();
    renderer.upsampling_pipeline.reset();
    renderer.copy_pipeline.reset();
    renderer.mix_pipeline.reset();
    renderer.format = format;
  }
  auto bloom_shader_header = Platform::GetShaderGlobalDefines();
  if (format == VK_FORMAT_R16G16B16A16_SFLOAT) {
    bloom_shader_header += "\n#define EE_BLOOM_STORAGE_FORMAT_RGBA16F 1\n";
  }
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
  if (!renderer.upsampling_layout) {
    renderer.upsampling_layout = std::make_shared<DescriptorSetLayout>();
    renderer.upsampling_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                      VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.upsampling_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                      VK_SHADER_STAGE_COMPUTE_BIT, 0);
    renderer.upsampling_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                      0);
    renderer.upsampling_layout->Initialize();
  }
  if (!renderer.downsampling_pipeline) {
    renderer.downsampling_pipeline = std::make_shared<ComputePipeline>();
    renderer.downsampling_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, bloom_shader_header,
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomDownsampling.slang");
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
        ShaderType::Compute, bloom_shader_header,
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomUpsampling.slang");
    renderer.upsampling_pipeline->descriptor_set_layouts.emplace_back(renderer.upsampling_layout);
    auto& upsampling_push_constant_range = renderer.upsampling_pipeline->push_constant_ranges.emplace_back();
    upsampling_push_constant_range.size = sizeof(UpsamplingPushConstant);
    upsampling_push_constant_range.offset = 0;
    upsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    renderer.upsampling_pipeline->Initialize();
  }
  if (!renderer.copy_pipeline) {
    renderer.copy_pipeline = std::make_shared<ComputePipeline>();
    renderer.copy_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, bloom_shader_header,
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomCopy.slang");
    renderer.copy_pipeline->descriptor_set_layouts.emplace_back(renderer.copy_layout);
    auto& copy_push_constant_range = renderer.copy_pipeline->push_constant_ranges.emplace_back();
    copy_push_constant_range.size = sizeof(PrefilterPushConstant);
    copy_push_constant_range.offset = 0;
    copy_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    renderer.copy_pipeline->Initialize();
  }
  if (!renderer.mix_pipeline) {
    renderer.mix_pipeline = std::make_shared<ComputePipeline>();
    renderer.mix_pipeline->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/BloomMix.slang");
    renderer.mix_pipeline->descriptor_set_layouts.emplace_back(renderer.mix_layout);
    auto& mix_push_constant_range = renderer.mix_pipeline->push_constant_ranges.emplace_back();
    mix_push_constant_range.size = sizeof(MixPushConstant);
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
  auto& bloom = resources.bloom;
  const auto bloom_size = (size + glm::uvec2(1)) / 2u;
  const auto bloom_format = ResolveBloomFormat();
  if (size == stack.size && stack.source_color_texture && stack.result_texture && stack.swap_texture &&
      bloom.size == bloom_size && bloom.format == bloom_format && bloom.downsample_texture_a &&
      bloom.downsample_texture_b && bloom.upsample_texture)
    return;
  if (size != stack.size || !stack.source_color_texture || !stack.result_texture || !stack.swap_texture) {
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

  const uint32_t bloom_mip_levels =
      static_cast<uint32_t>(std::floor(std::log2(std::min(bloom_size.x, bloom_size.y)))) + 1;
  RenderTextureCreateInfo bloom_create_info{};
  bloom_create_info.depth = false;
  bloom_create_info.extent = {bloom_size.x, bloom_size.y, 1};
  bloom_create_info.color_format = bloom_format;
  bloom.downsample_texture_a = std::make_shared<RenderTexture>(bloom_create_info, bloom_mip_levels);
  bloom.downsample_texture_b = std::make_shared<RenderTexture>(bloom_create_info, bloom_mip_levels);
  bloom.upsample_texture = std::make_shared<RenderTexture>(bloom_create_info, bloom_mip_levels);
  bloom.size = bloom_size;
  bloom.format = bloom_format;
}

void PostProcessingStack::OnCreate() {
  ApplyDefaultSettings();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (render_layer && render_layer->GetPostProcessingRendererResources() &&
      !ApplicationContext::Get().GetLayer<WindowLayer>()) {
    while (!BuildNextPipeline(*render_layer->GetPostProcessingRendererResources())) {
    }
  }
}

void PostProcessingStack::ApplyDefaultSettings() {
  ambient_occlusion = std::make_shared<AmbientOcclusion>();
  bloom = std::make_shared<Bloom>();
  screen_space_reflection = std::make_shared<ScreenSpaceReflection>();
  anti_aliasing = std::make_shared<AntiAliasing>();
  tone_mapping = std::make_shared<ToneMapping>();
  enable_ambient_occlusion = true;
  enable_bloom = true;
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
                                    Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/Blur.slang");
        resources.stack.blur_pipeline->descriptor_set_layouts.emplace_back(resources.stack.blur_layout);
        auto& push_constant_range = resources.stack.blur_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(BlurPushConstant);
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
  PostProcessingExecutionContext context{camera, renderer};
  if (pre_process) {
    Platform::RecordCommandsMainQueue(pre_process);
  }

  if (enable_screen_space_reflection) {
    screen_space_reflection->Process(*this, target_camera, context);
  }
  if (enable_bloom) {
    bloom->Process(*this, target_camera, context);
  }

  if (enable_tone_mapping) {
    tone_mapping->Process(*this, target_camera, context);
  }
  if (enable_anti_aliasing) {
    anti_aliasing->Process(*this, target_camera, context);
  }
}

void PostProcessingStack::ProcessBloomAndToneMappingImmediately(const std::shared_ptr<Camera>& target_camera) {
  if (!target_camera || !bloom || !tone_mapping) {
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer || !render_layer->GetPostProcessingRendererResources()) {
    return;
  }
  auto& renderer = *render_layer->GetPostProcessingRendererResources();
  bloom->BuildPipelines(renderer);
  tone_mapping->BuildPipelines(renderer);
  auto stack_asset = target_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  auto& camera = target_camera->AcquirePostProcessingResources(stack_asset);
  Resize(camera, target_camera->GetSize());
  PostProcessingExecutionContext context{camera, renderer};
  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    context.record_commands = [&](const std::function<void(VkCommandBuffer)>& action) {
      action(command_buffer);
    };
    bloom->Process(*this, target_camera, context);
    tone_mapping->Process(*this, target_camera, context);
  });
}

void PostProcessingStack::ProcessAmbientOcclusion(
    const std::shared_ptr<Camera>& target_camera, const std::shared_ptr<ImageView>& ambient_occlusion_image_view,
    const std::shared_ptr<ImageView>& scratch_image_view,
    const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process) {
  if (!target_camera || !enable_ambient_occlusion || !ambient_occlusion || !ambient_occlusion_image_view ||
      !scratch_image_view) {
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer || !render_layer->GetPostProcessingRendererResources()) {
    return;
  }
  auto& renderer = *render_layer->GetPostProcessingRendererResources();
  ambient_occlusion->BuildPipelines(renderer);
  auto stack_asset = target_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  auto& camera = target_camera->AcquirePostProcessingResources(stack_asset);
  PostProcessingExecutionContext context{camera, renderer, ambient_occlusion_image_view, scratch_image_view};
  if (pre_process) {
    Platform::RecordCommandsMainQueue(pre_process);
  }
  ambient_occlusion->Process(*this, target_camera, context);
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
  PostProcessingExecutionContext context{camera, renderer};
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
  const auto& blur_pipeline = context.renderer.stack.blur_pipeline;
  if (!blur_pipeline || !blur_pipeline->Initialized()) {
    return;
  }

  BlurPushConstant push_constant{};
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
