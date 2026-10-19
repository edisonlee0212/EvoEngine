#include "GiSupportCache.hpp"
#include <stdexcept>
#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "Cubemap.hpp"
#include "HddagiCamera.hpp"
#include "Platform.hpp"
#include "RayTracingPipeline.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

using namespace evo_engine;
namespace {
std::shared_ptr<Buffer> MakeSupportBuffer(VkDeviceSize bytes, VkBufferUsageFlags usage) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = bytes;
  info.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  return std::make_shared<Buffer>(info);
}

}  // namespace
std::shared_ptr<DescriptorSetLayout> evo_engine::CreateGiSupportLayout(bool ray_tracing) {
  auto layout = std::make_shared<DescriptorSetLayout>();
  const VkShaderStageFlags stages =
      VK_SHADER_STAGE_COMPUTE_BIT |
      (ray_tracing ? VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR : 0u);
  for (uint32_t binding :
       {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 9u, 10u, 11u, 12u, 13u, 14u, 16u, 19u, 20u, 21u, 23u, 24u, 25u}) {
    const auto type = binding == 0                    ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER
                      : binding <= 3 || binding == 23 ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
                      : binding == 9                  ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE
                      : binding == 16                 ? VK_DESCRIPTOR_TYPE_SAMPLER
                                                      : VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    layout->PushDescriptorBinding(binding, type, stages, 0);
  }
  layout->Initialize();
  return layout;
}
GiSupportCache::GiSupportCache(const std::shared_ptr<DescriptorSetLayout>& per_frame,
                               const std::shared_ptr<DescriptorSetLayout>& ray_tracing,
                               const std::shared_ptr<DescriptorSetLayout>& lighting, bool hddagi,
                               std::vector<std::shared_ptr<DescriptorSetLayout>> deferred_layouts)
    : hddagi_(hddagi) {
  probes_ = MakeSupportBuffer(Capacity * ProbeBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  requests_ = MakeSupportBuffer(Capacity * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  pending_ = MakeSupportBuffer(Capacity * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  rays_ = MakeSupportBuffer(Capacity * Rays * sizeof(glm::vec4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  layout_ = CreateGiSupportLayout(!hddagi);
  VkSamplerCreateInfo sampler_info{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
  sampler_info.minFilter = sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
  sampler_info.addressModeU = sampler_info.addressModeV = sampler_info.addressModeW =
      VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.maxLod = VK_LOD_CLAMP_NONE;
  sampler_ = std::make_shared<Sampler>(sampler_info);
  for (int mode : {0, 1, 2, 3, 4, 5}) {
    if (!hddagi && mode == 2)
      continue;
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->descriptor_set_layouts = {per_frame, layout_, lighting};
    const auto defines =
        "#define GI_SUPPORT_MODE " + std::to_string(mode) + "\n" + (hddagi ? "#define GI_SUPPORT_HDDAGI\n" : "");
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, defines, Resources::GetDefaultResourcesPath() / "Shaders/Compute/GiSupport.slang");
    pipeline->Initialize();
    if (!pipeline->Initialized())
      throw std::runtime_error("GI support compute pipeline failed");
    pipelines_[mode] = pipeline;
  }
  if (!deferred_layouts.empty()) {
    deferred_layouts.push_back(layout_);
    deferred_pipeline_ = std::make_shared<ComputePipeline>();
    deferred_pipeline_->descriptor_set_layouts = std::move(deferred_layouts);
    deferred_pipeline_->push_constant_ranges.push_back(
        {VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(RenderInstancePushConstant)});
    deferred_pipeline_->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, std::string("#define EE_GI_SUPPORT\n") + (hddagi ? "#define EE_AUTOMATIC_HDDAGI\n" : ""),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/DeferredComputeLighting.slang");
    deferred_pipeline_->Initialize();
    if (!deferred_pipeline_->Initialized())
      throw std::runtime_error("GI support deferred pipeline failed");
  }
  if (!hddagi) {
    trace_ = std::make_shared<RayTracingPipeline>();
    const auto root = Resources::GetDefaultResourcesPath() / "Shaders/RayTracing";
    const auto defines = Platform::GetShaderGlobalDefines();
    trace_->raygen_shader = Shader::CreateTemporary(ShaderType::RayGen, defines, root / "RayGen/GiSupport.slang");
    trace_->miss_shader = Shader::CreateTemporary(ShaderType::Miss, defines, root / "Miss/DDGIProbeTrace.slang");
    trace_->closest_hit_shader = Shader::CreateTemporary(ShaderType::ClosestHit, defines + "#define GI_SUPPORT_TRACE\n",
                                                         root / "ClosestHit/DDGIProbeTrace.slang");
    trace_->any_hit_shader = Shader::CreateTemporary(ShaderType::AnyHit, defines, root / "AnyHit/DDGIProbeTrace.slang");
    trace_->descriptor_set_layouts = {per_frame, ray_tracing, lighting, layout_};
    trace_->push_constant_ranges.push_back({VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
                                                VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR,
                                            0, sizeof(DdgiProbeRayTracingPushConstant)});
    trace_->Initialize();
    if (!trace_->Initialized())
      throw std::runtime_error("GI support ray pipeline failed");
  }
}
uint64_t GiSupportCache::AllocationBytes() const {
  return probes_->GetSize() + requests_->GetSize() + pending_->GetSize() + rays_->GetSize() +
         (mask_ ? mask_->GetExtent().width * uint64_t(mask_->GetExtent().height) : 0);
}
void GiSupportCache::Execute(const RenderGraphExecutionContext& context,
                             const DeferredComputeLightingPass::Parameters& inputs,
                             const std::shared_ptr<const HddagiRuntime>& runtime,
                             const std::shared_ptr<DescriptorSet>& ray_tracing,
                             const DdgiProbeRayTracingPushConstant& trace, uint64_t lighting_signature) {
  const auto& field = runtime->resources;
  const auto viewport = inputs.camera->GetSize();
  if (!mask_ || mask_->GetExtent().width != viewport.x || mask_->GetExtent().height != viewport.y) {
    // A replaced mask is retained by the submission that references its descriptor set.
    VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    info.imageType = VK_IMAGE_TYPE_2D;
    info.format = VK_FORMAT_R8_UINT;
    info.extent = {viewport.x, viewport.y, 1};
    info.mipLevels = info.arrayLayers = 1;
    info.samples = VK_SAMPLE_COUNT_1_BIT;
    info.tiling = VK_IMAGE_TILING_OPTIMAL;
    info.usage = VK_IMAGE_USAGE_STORAGE_BIT;
    mask_ = std::make_shared<Image>(info);
    VkImageViewCreateInfo view{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    view.image = mask_->GetVkImage();
    view.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view.format = info.format;
    view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    mask_view_ = std::make_shared<ImageView>(view, mask_);
  }
  uint64_t signature = lighting_signature ^ runtime->content_version;
  const auto mix = [&](const void* bytes, size_t size) {
    const auto* data = static_cast<const uint8_t*>(bytes);
    for (size_t i = 0; i < size; ++i)
      signature = (signature ^ data[i]) * 1099511628211ull;
  };
  for (const auto& cascade : field->light_inputs)
    for (const auto& lights : cascade)
      mix(lights.data(), lights.size() * sizeof(SdfgiLight));
  const auto& sky = field->sky_input;
  mix(&runtime->probes.base_probe_distance, sizeof(runtime->probes.base_probe_distance));
  mix(&sky.constant_color, sizeof(sky.constant_color));
  mix(&runtime->settings.read_sky_light, sizeof(runtime->settings.read_sky_light));
  mix(&runtime->settings.bounce_feedback, sizeof(runtime->settings.bounce_feedback));
  mix(&sky.map_id, sizeof(sky.map_id));
  mix(&sky.map_version, sizeof(sky.map_version));
  mix(&sky.cubemap_version, sizeof(sky.cubemap_version));
  mix(&sky.color, sizeof(sky.color));
  mix(&sky.gamma, sizeof(sky.gamma));
  mix(&sky.rotation, sizeof(sky.rotation));
  mix(&sky.energy, sizeof(sky.energy));
  if (field_.lock() != field || signature_ != signature) {
    ++generation_;
    field_ = field;
    signature_ = signature;
    reset_pending_ = true;
  }
  reset_pending_ |= region_version_ != runtime->region_version;
  region_version_ = runtime->region_version;
  content_version_ = runtime->content_version;
  GiSupportParams params;
  params.field = BuildHddagiGatherData(runtime->probes, runtime->settings, runtime->cascades,
                                       runtime->frame.anchor.world_position);
  params.viewport = viewport;
  params.camera_index = inputs.camera_index;
  params.frame = uint32_t(Platform::GetFrameCount());
  params.generation = generation_;
  params.sky_color_energy = glm::vec4(sky.color, sky.energy);
  params.sky_parameters = {sky.rotation, 1.0f / sky.gamma,
                           field->settings.read_sky_light ? (sky.constant_color ? 1.0f : 2.0f) : 0.0f,
                           sky.cubemap ? float(sky.cubemap->GetTextureStorageIndex()) : 0.0f};
  auto frame = std::make_shared<Frame>();
  frame->uniform = MakeSupportBuffer(sizeof(params), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
  BufferUploadBatch upload;
  upload.Add(frame->uniform, params, {BufferUploadUsage::Uniform});
  upload.Record(frame->uploads);
  frame->set = std::make_shared<DescriptorSet>(layout_);
  for (const auto& [binding, buffer] : {std::pair{0u, frame->uniform}, std::pair{1u, probes_}, std::pair{2u, requests_},
                                        std::pair{3u, rays_}, std::pair{23u, pending_}})
    frame->set->UpdateBufferDescriptorBinding(binding, buffer);
  std::vector<VkRenderingAttachmentInfo> attachments;
  inputs.camera->AppendGBufferColorAttachmentInfos(attachments, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                   VK_ATTACHMENT_STORE_OP_STORE);
  const auto bind = [&](uint32_t binding, VkImageView view) {
    VkDescriptorImageInfo image{};
    image.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image.imageView = view;
    frame->set->UpdateImageDescriptorBinding(binding, image);
  };
  bind(4, attachments[1].imageView);
  bind(5, inputs.camera->GetRenderTexture()->GetDepthImageView()->GetVkImageView());
  bind(6, attachments[0].imageView);
  bind(7, attachments[2].imageView);
  bind(21, attachments[3].imageView);
  bind(24, attachments[4].imageView);
  bind(9, mask_view_->GetVkImageView());
  for (const auto& [binding, name] : {std::pair{10u, "VoxelBits"}, std::pair{11u, "Regions"},
                                      std::pair{13u, "Occlusion0"}, std::pair{14u, "Occlusion1"}})
    bind(binding, field->images.at(name).sampled->GetVkImageView());
  if (hddagi_) {
    const auto& surface = field->camera_images.at(inputs.camera->GetHandle().GetValue())->images.at("Surface");
    bind(25, surface.sampled->GetVkImageView());
    for (const auto& [binding, name] :
         {std::pair{12u, "Light"}, std::pair{19u, "FilteredDiffuse"}, std::pair{20u, "Specular"}})
      bind(binding, field->images.at(name).sampled->GetVkImageView());
  }
  VkDescriptorImageInfo sampler{};
  sampler.sampler = sampler_->GetVkSampler();
  frame->set->UpdateImageDescriptorBinding(16, sampler);
  frames_.resize(Platform::GetMaxFramesInFlight());
  frames_[Platform::GetCurrentFrameIndex()] = frame;
  published_ = frame;
  auto& retained = *inputs.transient_resources;
  retained.RetainBuffer(frame->uniform);
  retained.RetainDescriptorSet(frame->set);
  retained.RetainImage(mask_);
  retained.RetainImageView(mask_view_);
  inputs.record_commands([&](VkCommandBuffer command) {
    ApplyGraphResourceBarriers(command, context);
    const RenderPassGpuTimestampScope timing(command, context, inputs.camera->GetHandle().GetValue(),
                                             uint64_t(inputs.camera_index));
    Platform::EverythingBarrier(command);
    mask_->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
    if (!initialized_) {
      vkCmdFillBuffer(command, probes_->GetVkBuffer(), 0, VK_WHOLE_SIZE, 0);
      initialized_ = true;
    }
    vkCmdFillBuffer(command, requests_->GetVkBuffer(), 0, VK_WHOLE_SIZE, 0xffffffffu);
    if (reset_pending_)
      vkCmdFillBuffer(command, pending_->GetVkBuffer(), 0, VK_WHOLE_SIZE, 0xffffffffu);
    reset_pending_ = false;
    Platform::EverythingBarrier(command);
    const auto dispatch = [&](int mode, uint32_t x, uint32_t y) {
      auto& pipeline = pipelines_.at(mode);
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 0, inputs.per_frame_descriptor_set->GetVkDescriptorSet());
      pipeline->BindDescriptorSet(command, 1, frame->set->GetVkDescriptorSet());
      pipeline->BindDescriptorSet(command, 2, inputs.lighting_descriptor_set->GetVkDescriptorSet());
      pipeline->Dispatch(command, x, y);
      Platform::EverythingBarrier(command);
    };
    dispatch(0, Platform::DivUp(viewport.x, 16), Platform::DivUp(viewport.y, 16));
    dispatch(5, Platform::DivUp(viewport.x, 16), Platform::DivUp(viewport.y, 16));
    dispatch(4, Capacity / 64, 1);
    vkCmdFillBuffer(command, pending_->GetVkBuffer(), 0, VK_WHOLE_SIZE, 0xffffffffu);
    Platform::EverythingBarrier(command);
    dispatch(1, Capacity / 64, 1);
    if (hddagi_)
      dispatch(2, Capacity * Rays / 64, 1);
    else {
      trace_->Bind(command);
      trace_->BindDescriptorSet(command, 0, inputs.per_frame_descriptor_set->GetVkDescriptorSet());
      trace_->BindDescriptorSet(command, 1, ray_tracing->GetVkDescriptorSet());
      trace_->BindDescriptorSet(command, 2, inputs.lighting_descriptor_set->GetVkDescriptorSet());
      trace_->BindDescriptorSet(command, 3, frame->set->GetVkDescriptorSet());
      auto support_trace = trace;
      support_trace.trace_parameters.z = glm::uintBitsToFloat(0u);
      support_trace.probe_counts_and_ray_count.w = Rays;
      trace_->PushConstant(command, 0, support_trace);
      trace_->Trace(command, Capacity * Rays, 1, 1);
      Platform::EverythingBarrier(command);
    }
    dispatch(3, Capacity / 64, 1);
    ApplyGraphResourceReleaseBarriers(command, context, RenderPassQueue::Graphics);
  });
}

std::shared_ptr<DescriptorSet> GiSupportCache::GetCurrentSet() const {
  return frames_.at(Platform::GetCurrentFrameIndex())->set;
}
std::shared_ptr<DescriptorSet> GiSupportCache::GetPublishedSet(const HddagiRuntime& runtime) const {
  return hddagi_ && published_ && field_.lock() == runtime.resources && content_version_ == runtime.content_version
             ? published_->set
             : nullptr;
}

std::shared_ptr<Buffer> GiSupportCache::GetPublishedUniform() const {
  return published_ ? published_->uniform : nullptr;
}
