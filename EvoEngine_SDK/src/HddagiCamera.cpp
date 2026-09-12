#include "HddagiCamera.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"
#include "SdfgiResources.hpp"
#include "Shader.hpp"

using namespace evo_engine;

std::shared_ptr<HddagiRuntime> evo_engine::SnapshotHddagiCapture(const std::shared_ptr<const HddagiRuntime>& source,
                                                                 const bool immediate) {
  if (!source || !source->resources || !source->resources->transport_ready ||
      source->resources->transport_failure_flags || !source->frame.failure.empty() || !source->voxel_failure.empty() ||
      !source->transport_failure.empty())
    return {};
  if (immediate)
    Platform::WaitForFrameSubmissions("HDDAGI immediate capture snapshot");
  // An immediate call cannot consume a field whose current scene-frame commands have not been submitted yet.
  if (immediate && source->resources->submission &&
      source->resources->submission->status != FrameSubmissionState::Status::Submitted)
    return {};
  try {
    auto snapshot = std::make_shared<HddagiRuntime>();
    snapshot->probes = source->probes;
    snapshot->settings = source->settings;
    snapshot->frame = source->frame;
    snapshot->cascades = source->cascades;
    snapshot->contributors = source->contributors;
    snapshot->resources = std::make_shared<HddagiResources>();
    const auto target = snapshot->resources;
    target->capture_source = source->resources;
    target->probes = source->probes;
    target->settings = source->settings;
    target->transport_ready = target->transport_recorded = true;
    target->transport_generation = static_cast<uint32_t>(source->resources->last_voxel_frame + 1);
    target->linear_sampler = source->resources->linear_sampler;
    target->camera_pipelines = source->resources->camera_pipelines;
    for (const auto name : {"FilteredDiffuse", "Specular", "Occlusion0", "Occlusion1", "VoxelBits", "Regions", "Light",
                            "Disocclusion", "LightNeighbors"})
      target->images[name] = source->resources->images.at(name);
    for (const auto name : {"FilteredDiffuse", "Specular", "Occlusion0", "Occlusion1"}) {
      auto& texture = target->images.at(name);
      const auto& r = texture.requirement;
      VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
      info.flags = VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT | VK_IMAGE_CREATE_EXTENDED_USAGE_BIT;
      info.imageType = r.type;
      info.format = r.storage_format;
      info.extent = r.extent;
      info.mipLevels = 1;
      info.arrayLayers = r.layers;
      info.samples = VK_SAMPLE_COUNT_1_BIT;
      info.tiling = VK_IMAGE_TILING_OPTIMAL;
      info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                   VK_IMAGE_USAGE_TRANSFER_DST_BIT;
      const std::array formats{r.storage_format, r.sampled_format};
      VkImageFormatListCreateInfo format_info{VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO};
      format_info.viewFormatCount = r.storage_format == r.sampled_format ? 1 : 2;
      format_info.pViewFormats = formats.data();
      info.pNext = &format_info;
      texture.image = std::make_shared<Image>(info);
      const auto view = [&](VkFormat format, VkImageUsageFlags usage) {
        VkImageViewUsageCreateInfo view_usage{VK_STRUCTURE_TYPE_IMAGE_VIEW_USAGE_CREATE_INFO};
        view_usage.usage = usage;
        VkImageViewCreateInfo v{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        v.pNext = &view_usage;
        v.image = texture.image->GetVkImage();
        v.viewType = r.type == VK_IMAGE_TYPE_3D ? VK_IMAGE_VIEW_TYPE_3D : VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        v.format = format;
        v.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, r.layers};
        return std::make_shared<ImageView>(v, texture.image);
      };
      texture.storage = view(r.storage_format, VK_IMAGE_USAGE_STORAGE_BIT);
      texture.sampled = view(r.sampled_format, VK_IMAGE_USAGE_SAMPLED_BIT);
      target->allocation_bytes += texture.image->GetVmaAllocationInfo().size;
    }
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = 16;
    info.usage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    target->buffers["Status"] = std::make_shared<Buffer>(info);
    const auto copy = [source, target](VkCommandBuffer command) {
      source->resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                     VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
      for (const auto name : {"FilteredDiffuse", "Specular", "Occlusion0", "Occlusion1"}) {
        const auto& from = source->resources->images.at(name);
        const auto& to = target->images.at(name);
        to.image->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
        VkImageCopy region{};
        region.srcSubresource = region.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, from.requirement.layers};
        region.extent = from.requirement.extent;
        vkCmdCopyImage(command, from.image->GetVkImage(), VK_IMAGE_LAYOUT_GENERAL, to.image->GetVkImage(),
                       VK_IMAGE_LAYOUT_GENERAL, 1, &region);
      }
      const VkBufferCopy region{0, 0, 16};
      vkCmdCopyBuffer(command, source->resources->buffers.at("Status")->GetVkBuffer(),
                      target->buffers.at("Status")->GetVkBuffer(), 1, &region);
      target->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                          VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    };
    if (immediate) {
      Platform::ImmediateSubmit(copy);
    } else {
      Platform::RecordCommandsMainQueue(copy);
    }
    return snapshot;
  } catch (const std::exception& error) {
    EVOENGINE_ERROR("HDDAGI reflection capture snapshot unavailable: " + std::string(error.what()));
    return {};
  }
}

uint64_t HddagiCameraFrame::AllocationBytes() const {
  uint64_t bytes = uploads.GetAllocationBytes();
  for (const auto& [name, buffer] : buffers)
    bytes += buffer->GetVmaAllocationInfo().size;
  return bytes;
}

std::shared_ptr<HddagiCameraFrame> HddagiCameraFrame::Create(
    HddagiRuntime& runtime, RenderInstanceStorage& instances,
    const std::vector<std::shared_ptr<DescriptorSetLayout>>& host_layouts, const glm::uvec2 viewport,
    const uint64_t camera_id, const bool reflection_capture) {
  auto& field = *runtime.resources;
  auto frame = std::make_shared<HddagiCameraFrame>();
  frame->filter_reflections = field.settings.filter_reflections && !reflection_capture;
  const auto layout = BuildHddagiCameraLayout(viewport);
  const auto max_dimension = Platform::GetSelectedPhysicalDevice()->properties.limits.maxImageDimension2D;
  if (layout.viewport.x > max_dimension || layout.viewport.y > max_dimension)
    throw std::invalid_argument("HDDAGI camera dimensions exceed the device limit");
  frame->params = {layout.viewport,
                   layout.gi,
                   layout.pixel_stride,
                   reflection_capture ? 1u : 0u,
                   layout.reflection_filter_radius,
                   0};
  frame->metadata =
      BuildHddagiGatherData(field.probes, field.settings, runtime.cascades, runtime.frame.anchor.world_position);
  if (field.camera_pipelines.empty()) {
    auto descriptors = std::make_shared<DescriptorSetLayout>();
    for (uint32_t i = 0; i < 26; ++i) {
      const auto type = i == 0 || i == 21                            ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER
                        : i == 5                                     ? VK_DESCRIPTOR_TYPE_SAMPLER
                        : i == 11 || i == 20                         ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
                        : i == 12 || (i >= 14 && i <= 16) || i >= 24 ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE
                                                                     : VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
      descriptors->PushDescriptorBinding(i, type, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    }
    descriptors->Initialize();
    auto layouts = host_layouts;
    layouts.push_back(descriptors);
    if (const auto failure =
            SdfgiResources::ValidateDescriptorLimits(layouts, Platform::GetSelectedPhysicalDevice()->properties.limits);
        !failure.empty())
      throw std::runtime_error(failure);
    std::map<std::string, std::shared_ptr<ComputePipeline>> pipelines;
    for (const auto& [name, shader] :
         std::map<std::string, std::string>{{"Surface", "HddagiSurface"},
                                            {"Gather", "HddagiCameraGather"},
                                            {"Deferred", "DeferredComputeLighting"},
                                            {"FilterHorizontal", "HddagiReflectionFilter"},
                                            {"FilterVertical", "HddagiReflectionFilter"}}) {
      auto pipeline = std::make_shared<ComputePipeline>();
      pipeline->descriptor_set_layouts = layouts;
      pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(RenderInstancePushConstant)});
      pipeline->compute_shader =
          Shader::CreateTemporary(ShaderType::Compute,
                                  name == "Deferred"         ? "#define EE_AUTOMATIC_HDDAGI\n"
                                  : name == "FilterVertical" ? "#define MODE_VERTICAL\n"
                                                             : "",
                                  Resources::GetDefaultResourcesPath() / ("Shaders/Compute/" + shader + ".slang"));
      pipeline->Initialize();
      if (!pipeline->Initialized())
        throw std::runtime_error("HDDAGI camera pipeline failed: " + name);
      pipelines[name] = pipeline;
    }
    field.camera_pipelines = std::move(pipelines);
  }
  frame->pipelines = field.camera_pipelines;
  auto& camera_images = field.camera_images[camera_id];
  if (!camera_images || camera_images->layout.viewport != layout.viewport || camera_images->layout.gi != layout.gi) {
    auto candidate = std::make_shared<HddagiCameraImages>();
    candidate->layout = layout;
    for (const auto name : {"Surface", "Diffuse", "Specular", "Blend", "SpecularScratch", "BlendScratch"}) {
      const std::string key(name);
      const auto size = key == "Surface" ? layout.viewport : layout.gi;
      const auto storage_format = key == "Surface"         ? VK_FORMAT_R16G16B16A16_SFLOAT
                                  : key.find("Blend") == 0 ? VK_FORMAT_R8G8_UNORM
                                                           : VK_FORMAT_R32_UINT;
      const auto sampled_format =
          key == "Diffuse" || key.find("Specular") == 0 ? VK_FORMAT_E5B9G9R9_UFLOAT_PACK32 : storage_format;
      VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
      info.flags = VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT | VK_IMAGE_CREATE_EXTENDED_USAGE_BIT;
      info.imageType = VK_IMAGE_TYPE_2D;
      info.format = storage_format;
      info.extent = {size.x, size.y, 1};
      info.mipLevels = info.arrayLayers = 1;
      info.samples = VK_SAMPLE_COUNT_1_BIT;
      info.tiling = VK_IMAGE_TILING_OPTIMAL;
      info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                   VK_IMAGE_USAGE_TRANSFER_DST_BIT;
      const std::array formats{storage_format, sampled_format};
      VkImageFormatListCreateInfo formats_info{VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO};
      formats_info.viewFormatCount = storage_format == sampled_format ? 1 : 2;
      formats_info.pViewFormats = formats.data();
      info.pNext = &formats_info;
      auto image = std::make_shared<Image>(info);
      const auto view = [&](VkFormat format, VkImageUsageFlags usage) {
        VkImageViewUsageCreateInfo view_usage{VK_STRUCTURE_TYPE_IMAGE_VIEW_USAGE_CREATE_INFO};
        view_usage.usage = usage;
        VkImageViewCreateInfo view_info{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        view_info.pNext = &view_usage;
        view_info.image = image->GetVkImage();
        view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        view_info.format = format;
        view_info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        return std::make_shared<ImageView>(view_info, image);
      };
      HddagiImage texture;
      texture.requirement = {key, storage_format, sampled_format, VK_IMAGE_TYPE_2D, info.extent};
      texture.image = image;
      texture.storage = view(storage_format, VK_IMAGE_USAGE_STORAGE_BIT);
      texture.sampled = view(sampled_format, VK_IMAGE_USAGE_SAMPLED_BIT);
      if (!image->GetVkImage() || !texture.storage->GetVkImageView() || !texture.sampled->GetVkImageView())
        throw std::runtime_error("HDDAGI camera image allocation failed: " + key);
      candidate->images.emplace(key, std::move(texture));
    }
    camera_images = std::move(candidate);
  }
  frame->images = camera_images;
  const auto buffer = [&](const std::string& name, size_t size, VkBufferUsageFlags usage) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = size;
    info.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    auto result = std::make_shared<Buffer>(info);
    frame->buffers[name] = result;
    return result;
  };
  frame->input_uploads.Add(buffer("Gather", sizeof(frame->metadata), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT),
                           frame->metadata, {BufferUploadUsage::Uniform});
  frame->input_uploads.Add(buffer("Params", sizeof(frame->params), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT), frame->params,
                           {BufferUploadUsage::Uniform});
  auto& classification = frame->receiver_classification;
  classification.assign(std::max(size_t(1), instances.GetInstanceInfoBlocks().size()), 1);
  for (size_t i = 0; i < instances.GetInstanceInfoBlocks().size(); ++i) {
    const SdfgiContributorId id{instances.GetInstanceRendererHandle(i).GetValue(),
                                instances.GetInstanceEntityHandle(i).GetValue()};
    classification[i] = runtime.contributors.entries.count(id) == 0;
  }
  frame->input_uploads.AddVector(buffer("Receivers", classification.size() * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT),
                                 classification);
  frame->set = std::make_shared<DescriptorSet>(frame->pipelines.at("Gather")->descriptor_set_layouts[5]);
  frame->set->UpdateBufferDescriptorBinding(0, frame->buffers.at("Gather"));
  frame->set->UpdateBufferDescriptorBinding(11, field.buffers.at("Status"));
  frame->set->UpdateBufferDescriptorBinding(20, frame->buffers.at("Receivers"));
  frame->set->UpdateBufferDescriptorBinding(21, frame->buffers.at("Params"));
  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  for (const auto& [binding, name] : std::map<uint32_t, std::string>{{1, "FilteredDiffuse"},
                                                                     {2, "Specular"},
                                                                     {3, "Occlusion0"},
                                                                     {4, "Occlusion1"},
                                                                     {6, "VoxelBits"},
                                                                     {7, "Regions"},
                                                                     {8, "Light"},
                                                                     {9, "Disocclusion"},
                                                                     {10, "LightNeighbors"}}) {
    image_info.imageView = field.images.at(name).sampled->GetVkImageView();
    frame->set->UpdateImageDescriptorBinding(binding, image_info);
  }
  for (const auto& [binding, name] : std::map<uint32_t, std::string>{{12, "Surface"},
                                                                     {13, "Surface"},
                                                                     {14, "Diffuse"},
                                                                     {15, "Specular"},
                                                                     {16, "Blend"},
                                                                     {17, "Diffuse"},
                                                                     {18, "Specular"},
                                                                     {19, "Blend"},
                                                                     {22, "SpecularScratch"},
                                                                     {23, "BlendScratch"},
                                                                     {24, "SpecularScratch"},
                                                                     {25, "BlendScratch"}}) {
    const auto& texture = frame->images->images.at(name);
    image_info.imageView =
        (binding == 12 || (binding >= 14 && binding <= 16) || binding >= 24 ? texture.storage : texture.sampled)
            ->GetVkImageView();
    frame->set->UpdateImageDescriptorBinding(binding, image_info);
  }
  image_info.imageView = VK_NULL_HANDLE;
  image_info.sampler = field.linear_sampler->GetVkSampler();
  frame->set->UpdateImageDescriptorBinding(5, image_info);
  return frame;
}

void HddagiCameraFrame::ImportCamera(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                     const HddagiResources& field) const {
  field.Import(graph, registry);
  for (const auto& [name, texture] : images->images) {
    RenderResourceDescriptor descriptor;
    descriptor.name = "Camera.HDDAGI." + name;
    descriptor.type = RenderResourceType::Image;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.dimensions = {
        RenderResourceSizeMode::Absolute, texture.requirement.extent.width, texture.requirement.extent.height, 1, 1, 1};
    descriptor.format_name = std::to_string(texture.requirement.storage_format);
    graph.AddResource(descriptor);
    registry.BindImage(descriptor.name, texture.image);
  }
}

void HddagiCameraFrame::AddPasses(RenderGraph& graph, const std::shared_ptr<HddagiResources>& field,
                                  const std::function<DeferredComputeLightingPass::Parameters()>& parameters,
                                  const std::string& dependency) {
  std::string previous = dependency;
  std::vector<std::string> passes{"Surface", "Gather"};
  if (filter_reflections) {
    passes.push_back("FilterHorizontal");
    passes.push_back("FilterVertical");
  }
  for (const auto& name : passes) {
    const bool surface = name == "Surface";
    auto descriptor = DeferredComputeLightingPass::CreateDescriptor(false, false);
    descriptor.resources.erase(std::remove_if(descriptor.resources.begin(), descriptor.resources.end(),
                                              [](const auto& access) {
                                                return access.resource_name == RenderResourceNames::camera_color ||
                                                       access.resource_name ==
                                                           RenderResourceNames::lighting_directional_shadow_map;
                                              }),
                               descriptor.resources.end());
    for (auto& access : descriptor.resources)
      access.usage = RenderResourceUsage::Read;
    if (name == "Gather")
      for (const auto image : {"FilteredDiffuse", "Specular", "Occlusion0", "Occlusion1", "VoxelBits", "Regions",
                               "Light", "Disocclusion", "LightNeighbors", "Status"})
        descriptor.resources.push_back(
            {"Frame.HDDAGI." + std::string(image), RenderResourceUsage::Read, RenderResourceState::General});
    descriptor.name = "HddagiCamera" + std::string(name);
    descriptor.profiler_display_name = "HDDAGI " + std::string(name);
    descriptor.dependencies = {previous};
    previous = descriptor.name;
    descriptor.resources.push_back({"Camera.HDDAGI.Surface",
                                    surface ? RenderResourceUsage::Write : RenderResourceUsage::Read,
                                    RenderResourceState::General});
    if (name == "Gather")
      for (const auto image : {"Diffuse", "Specular", "Blend"})
        descriptor.resources.push_back(
            {"Camera.HDDAGI." + std::string(image), RenderResourceUsage::Write, RenderResourceState::General});
    if (name.find("Filter") == 0) {
      const bool vertical = name == "FilterVertical";
      for (const auto image : {"Specular", "Blend", "SpecularScratch", "BlendScratch"}) {
        const bool scratch = std::string(image).find("Scratch") != std::string::npos;
        descriptor.resources.push_back({"Camera.HDDAGI." + std::string(image),
                                        scratch != vertical ? RenderResourceUsage::Write : RenderResourceUsage::Read,
                                        RenderResourceState::General});
      }
    }
    graph.AddPass(descriptor, [frame = shared_from_this(), field, parameters, surface,
                               name](const RenderGraphExecutionContext& context) {
      if (!frame->uploaded) {
        frame->input_uploads.Record(frame->uploads);
        frame->uploaded = true;
      }
      auto inputs = parameters();
      inputs.pipeline = frame->pipelines.at(name);
      inputs.hddagi_resources = field;
      inputs.hddagi_descriptor_set = frame->set;
      inputs.dispatch_size = surface ? frame->params.viewport : frame->params.gi_size;
      DeferredComputeLightingPass::Execute(context, inputs);
    });
  }
}
