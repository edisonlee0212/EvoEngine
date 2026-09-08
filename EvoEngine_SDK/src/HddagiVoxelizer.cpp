// Godot HDDAGI raster/region ordering, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt.
#include "HddagiVoxelizer.hpp"
#include <set>
#include "HddagiTypes.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

using namespace evo_engine;

std::shared_ptr<HddagiVoxelFrame> HddagiVoxelFrame::Create(HddagiResources& resources,
                                                           const std::shared_ptr<DescriptorSetLayout>& host_layout,
                                                           const SdfgiContributorRegistry& contributors,
                                                           const std::vector<SdfgiCascade>& cascades,
                                                           const std::vector<SdfgiPendingRegion>& pending,
                                                           const uint32_t version) {
  HddagiUpdatePlan plan;
  plan.cascades = cascades;
  plan.scroll.resize(cascades.size(), glm::ivec3(0));
  for (const auto& region : pending) {
    plan.regions.push_back({region, region, region});
    if (region.offset == glm::ivec3(0) && region.size == cascades[region.cascade].size)
      plan.full_cascades |= 1u << region.cascade;
  }
  plan.reset_history_cascades = plan.full_cascades;
  return Create(resources, host_layout, contributors, plan, version);
}

std::shared_ptr<HddagiVoxelFrame> HddagiVoxelFrame::Create(HddagiResources& resources,
                                                           const std::shared_ptr<DescriptorSetLayout>& host_layout,
                                                           const SdfgiContributorRegistry& contributors,
                                                           const HddagiUpdatePlan& plan, const uint32_t version) {
  if (version == 0 || version > UINT16_MAX)
    throw std::invalid_argument("HDDAGI region version must be nonzero and fit 16 bits");
  const auto& cascades = plan.cascades;
  std::vector<SdfgiPendingRegion> pending;
  for (const auto& entry : plan.regions) {
    const auto& region = entry.core;
    if (region.size == glm::ivec3(0)) {
      pending.push_back(entry.raster);
      continue;
    }
    if (region.cascade >= cascades.size() || glm::any(glm::lessThan(region.offset, glm::ivec3(0))) ||
        glm::any(glm::lessThanEqual(region.size, glm::ivec3(0))) ||
        glm::any(glm::greaterThan(region.offset + region.size, cascades[region.cascade].size)) ||
        glm::any(glm::notEqual(region.offset % 8, glm::ivec3(0))) ||
        glm::any(glm::notEqual(region.size % 8, glm::ivec3(0))))
      throw std::invalid_argument("HDDAGI hierarchy updates must cover aligned whole regions");
    pending.push_back(entry.raster);
  }
  auto frame = std::make_shared<HddagiVoxelFrame>();
  frame->update_plan = plan;
  frame->version = version;
  for (const auto& entry : plan.regions) {
    frame->update_bounds.push_back({entry.light.offset, entry.light.cascade, entry.light.offset + entry.light.size, 0});
    frame->written_cascades |= 1u << entry.core.cascade;
  }
  frame->scene_frame = Platform::GetFrameCount();
  VkBufferCreateInfo readback_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  readback_info.size = resources.probes.cascade_count * 20;
  readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo readback_allocation{};
  readback_allocation.usage = VMA_MEMORY_USAGE_AUTO;
  readback_allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  readback_allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  frame->status_readback = std::make_shared<Buffer>(readback_info, readback_allocation);
  frame->raster = resources.voxel_pipeline;
  frame->region_store = resources.region_pipeline;
  if (!frame->raster || !frame->region_store) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    const auto raster_stages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, raster_stages, 0);
    for (uint32_t i = 1; i <= 4; ++i)
      layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    layout->Initialize();
    const auto failure = SdfgiResources::ValidateDescriptorLimits(
        {host_layout, layout}, Platform::GetSelectedPhysicalDevice()->properties.limits);
    if (!failure.empty())
      throw std::runtime_error(failure);
    const auto root = Resources::GetDefaultResourcesPath() / "Shaders";
    frame->raster = std::make_shared<GraphicsPipeline>();
    auto& pipeline = *frame->raster;
    pipeline.vertex_input_enabled = true;
    pipeline.view_mask = 0;
    pipeline.depth_attachment_format = pipeline.stencil_attachment_format = VK_FORMAT_UNDEFINED;
    pipeline.descriptor_set_layouts = {host_layout, layout};
    pipeline.push_constant_ranges.push_back({raster_stages, 0, sizeof(SdfgiVoxelPushConstant)});
    pipeline.vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, "", root / "Graphics/Vertex/SDFGI/SdfgiVoxelize.slang");
    pipeline.fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, "", root / "Graphics/Fragment/HDDAGI/HddagiVoxelize.slang");
    pipeline.Initialize();
    if (!pipeline.Initialized())
      throw std::runtime_error("HDDAGI raster pipeline creation failed");
    auto region_layout = std::make_shared<DescriptorSetLayout>();
    region_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    for (uint32_t i = 1; i <= 3; ++i)
      region_layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    region_layout->Initialize();
    frame->region_store = std::make_shared<ComputePipeline>();
    auto& compute = *frame->region_store;
    compute.descriptor_set_layouts = {region_layout};
    compute.push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiRegionParams)});
    compute.compute_shader = Shader::CreateTemporary(ShaderType::Compute, "", root / "Compute/HddagiRegionStore.slang");
    compute.Initialize();
    if (!compute.Initialized())
      throw std::runtime_error("HDDAGI region pipeline creation failed");
    resources.voxel_pipeline = frame->raster;
    resources.region_pipeline = frame->region_store;
  }
  const auto& layout = frame->raster->descriptor_set_layouts[1];
  const auto& region_layout = frame->region_store->descriptor_set_layouts[0];
  frame->inputs = SdfgiVoxelFrame::CreateRasterInputs(
      frame->raster, layout,
      {resources.images.at("Albedo").storage, resources.images.at("Emission").storage,
       resources.images.at("EmissionAniso").storage, resources.images.at("NormalBits").storage},
      SdfgiYMultiplier(static_cast<SdfgiSettings::VerticalScale>(resources.probes.vertical_scale)), contributors,
      cascades, pending);
  VkBufferCreateInfo bounds_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  bounds_info.size = std::max<size_t>(1, frame->update_bounds.size()) * sizeof(HddagiUpdateBounds);
  bounds_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  const auto bounds_buffer = std::make_shared<Buffer>(bounds_info);
  frame->inputs->buffers.emplace("UpdateBounds", bounds_buffer);
  if (!frame->update_bounds.empty())
    frame->inputs->input_uploads.AddVector(bounds_buffer, frame->update_bounds);
  frame->region_set = std::make_shared<DescriptorSet>(region_layout);
  uint32_t binding = 0;
  for (const auto name : {"NormalBits", "VoxelBits", "Regions", "Versions"}) {
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    info.imageView =
        (binding == 0 ? resources.images.at(name).sampled : resources.images.at(name).storage)->GetVkImageView();
    frame->region_set->UpdateImageDescriptorBinding(binding++, info);
  }
  if (!resources.light_store_pipeline) {
    auto light_layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 12; ++binding) {
      const auto type = binding < 4 || binding == 6 || binding == 7 ? VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE
                        : (binding < 6 || binding == 11)            ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
                                                                    : VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
      light_layout->PushDescriptorBinding(binding, type, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    }
    light_layout->Initialize();
    const auto failure = SdfgiResources::ValidateDescriptorLimits(
        {light_layout}, Platform::GetSelectedPhysicalDevice()->properties.limits);
    if (!failure.empty())
      throw std::runtime_error(failure);
    auto compute = std::make_shared<ComputePipeline>();
    compute->descriptor_set_layouts = {light_layout};
    compute->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiLightStoreParams)});
    compute->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, "", Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiLightStore.slang");
    compute->Initialize();
    if (!compute->Initialized())
      throw std::runtime_error("HDDAGI light-store pipeline creation failed");
    resources.light_store_pipeline = compute;
  }
  frame->light_store = resources.light_store_pipeline;
  for (uint32_t cascade = 0; cascade < resources.probes.cascade_count; ++cascade) {
    auto set = std::make_shared<DescriptorSet>(frame->light_store->descriptor_set_layouts[0]);
    const char* names[]{"Albedo",     "Emission",   "EmissionAniso", "NormalBits",     "",     "",
                        "Occlusion0", "Occlusion1", "Disocclusion",  "LightNeighbors", "Light"};
    for (uint32_t binding = 0; binding < 11; ++binding) {
      if (binding == 4 || binding == 5) {
        set->UpdateBufferDescriptorBinding(
            binding, resources.buffers.at(std::string(binding == 4 ? "DispatchSpare" : "ProcessSpare") +
                                          std::to_string(cascade)));
      } else {
        VkDescriptorImageInfo info{};
        info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
        const auto& image = resources.images.at(names[binding]);
        info.imageView = (binding < 8 ? image.sampled : image.storage)->GetVkImageView();
        set->UpdateImageDescriptorBinding(binding, info);
      }
    }
    set->UpdateBufferDescriptorBinding(11, bounds_buffer);
    frame->light_sets.push_back(std::move(set));
  }
  if (!resources.light_scroll_pipeline) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 5; ++binding)
      layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
    auto compute = std::make_shared<ComputePipeline>();
    compute->descriptor_set_layouts = {layout};
    compute->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiScrollParams)});
    compute->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, "", Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiLightScroll.slang");
    compute->Initialize();
    if (!compute->Initialized())
      throw std::runtime_error("HDDAGI light-scroll pipeline creation failed");
    resources.light_scroll_pipeline = compute;
  }
  frame->light_scroll = resources.light_scroll_pipeline;
  for (uint32_t cascade = 0; cascade < resources.probes.cascade_count; ++cascade) {
    auto set = std::make_shared<DescriptorSet>(frame->light_scroll->descriptor_set_layouts[0]);
    uint32_t binding = 0;
    for (const auto name : {"Dispatch", "Process", "DispatchSpare", "ProcessSpare"})
      set->UpdateBufferDescriptorBinding(binding++, resources.buffers.at(std::string(name) + std::to_string(cascade)));
    set->UpdateBufferDescriptorBinding(4, bounds_buffer);
    frame->scroll_sets.push_back(std::move(set));
  }
  if (!resources.reset_probes_pipeline) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 10; ++binding)
      layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
    auto compute = std::make_shared<ComputePipeline>();
    compute->descriptor_set_layouts = {layout};
    compute->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiResetParams)});
    compute->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, "", Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiResetProbes.slang");
    compute->Initialize();
    if (!compute->Initialized())
      throw std::runtime_error("HDDAGI probe-reset pipeline creation failed");
    resources.reset_probes_pipeline = compute;
  }
  frame->reset_probes = resources.reset_probes_pipeline;
  if (!resources.occlusion_pipeline) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->descriptor_set_layouts = {layout};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiOcclusionParams)});
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, "", Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiOcclusion.slang");
    pipeline->Initialize();
    if (!pipeline->Initialized())
      throw std::runtime_error("HDDAGI occlusion pipeline creation failed");
    resources.occlusion_pipeline = std::move(pipeline);
  }
  frame->occlusion = resources.occlusion_pipeline;
  if (!resources.metadata_pipeline) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 7; ++binding)
      layout->PushDescriptorBinding(binding,
                                    binding < 3    ? VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE
                                    : binding == 3 ? VK_DESCRIPTOR_TYPE_SAMPLER
                                                   : VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                    VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->descriptor_set_layouts = {layout};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiMetadataParams)});
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, "", Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiProbeMetadata.slang");
    pipeline->Initialize();
    if (!pipeline->Initialized())
      throw std::runtime_error("HDDAGI probe-metadata pipeline creation failed");
    resources.metadata_pipeline = std::move(pipeline);
  }
  if (!resources.linear_sampler) {
    VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sampler.minFilter = sampler.magFilter = VK_FILTER_LINEAR;
    sampler.addressModeU = sampler.addressModeV = sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    resources.linear_sampler = std::make_shared<Sampler>(sampler);
  }
  frame->sampler = resources.linear_sampler;
  frame->metadata = resources.metadata_pipeline;
  frame->metadata_set = std::make_shared<DescriptorSet>(frame->metadata->descriptor_set_layouts[0]);
  const char* metadata_names[]{"Regions", "Occlusion0", "Occlusion1", "", "Neighbors", "Proximity", "CameraVisibility"};
  for (uint32_t binding = 0; binding < 7; ++binding) {
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    if (binding == 3)
      info.sampler = frame->sampler->GetVkSampler();
    else {
      const auto& image = resources.images.at(metadata_names[binding]);
      info.imageView = (binding < 3 ? image.sampled : image.storage)->GetVkImageView();
    }
    frame->metadata_set->UpdateImageDescriptorBinding(binding, info);
  }
  for (uint32_t plane = 0; plane < 2; ++plane) {
    auto& set = frame->occlusion_sets[plane];
    set = std::make_shared<DescriptorSet>(frame->occlusion->descriptor_set_layouts[0]);
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    info.imageView = resources.images.at("NormalBits").sampled->GetVkImageView();
    set->UpdateImageDescriptorBinding(0, info);
    info.imageView = resources.images.at("Occlusion" + std::to_string(plane)).storage->GetVkImageView();
    set->UpdateImageDescriptorBinding(1, info);
  }
  frame->reset_set = std::make_shared<DescriptorSet>(frame->reset_probes->descriptor_set_layouts[0]);
  uint32_t reset_binding = 0;
  for (const auto name : {"History", "HistorySum", "Diffuse", "Specular", "FilteredDiffuse", "Ambient", "Neighbors",
                          "ProcessFrame", "Proximity", "CameraVisibility"}) {
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    info.imageView = resources.images.at(name).storage->GetVkImageView();
    frame->reset_set->UpdateImageDescriptorBinding(reset_binding++, info);
  }
  return frame;
}

void HddagiVoxelFrame::AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                 const std::shared_ptr<HddagiResources>& resources) {
  std::vector<RenderResourceAccess> reads;
  const auto import_buffer = [&](const std::string& name, const std::shared_ptr<Buffer>& buffer) {
    if (!buffer)
      return;
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.HDDAGI.Voxel." + name;
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = buffer->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, buffer);
    reads.push_back({descriptor.name, RenderResourceUsage::Read, RenderResourceState::General});
  };
  for (const auto& [name, buffer] : inputs->buffers)
    import_buffer(name, buffer);
  RenderPassDescriptor upload{"HddagiVoxelInputs", RenderPassQueue::Graphics, RenderPassScope::Frame};
  if (!resources->initialization_recorded)
    upload.dependencies = {"HddagiInitialize"};
  for (const auto& read : reads)
    upload.resources.push_back(
        {read.resource_name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  graph.AddPass(upload, [frame = shared_from_this()](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      ApplyGraphResourceBarriers(command, context);
    });
    frame->inputs->input_uploads.Record(frame->inputs->uploads);
  });
  import_buffer("Vertices", inputs->vertex_buffer);
  import_buffer("Indices", inputs->index_buffer);
  std::set<const Image*> imported;
  for (const auto& texture : inputs->textures) {
    if (!texture.image || !imported.insert(texture.image.get()).second)
      continue;
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.HDDAGI.Voxel.Texture" + std::to_string(imported.size());
    descriptor.type = RenderResourceType::Image;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    graph.AddResource(descriptor);
    registry.BindImage(descriptor.name, texture.image);
    reads.push_back({descriptor.name, RenderResourceUsage::Read,
                     texture.image->GetLayout() == VK_IMAGE_LAYOUT_GENERAL ? RenderResourceState::General
                                                                           : RenderResourceState::ShaderRead});
  }
  RenderPassDescriptor invalidate{"HddagiInvalidate", RenderPassQueue::Graphics, RenderPassScope::Frame};
  invalidate.dependencies = {upload.name};
  for (const auto name : {"HitCache", "HitVersions", "History", "HistorySum", "Diffuse", "Specular", "FilteredDiffuse",
                          "Ambient", "Neighbors", "ProcessFrame", "Proximity", "CameraVisibility"})
    invalidate.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Write, RenderResourceState::General});
  graph.AddPass(invalidate, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      for (const auto name : {"HitCache", "HitVersions"}) {
        const auto& image = resources->images.at(name);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, image.requirement.layers};
        Platform::ClearColorImage(command, *image.image, VkClearColorValue{}, 1, &range);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      frame->reset_probes->Bind(command);
      frame->reset_probes->BindDescriptorSet(command, 0, frame->reset_set->GetVkDescriptorSet());
      for (uint32_t cascade = 0; cascade < resources->probes.cascade_count; ++cascade) {
        if (!((frame->written_cascades | frame->update_plan.reset_history_cascades) & (1u << cascade)))
          continue;
        const auto& field = frame->inputs->cascades[cascade];
        HddagiResetParams params;
        params.probe_size = resources->probes.ProbeSize();
        params.cascade = cascade;
        params.region_offset = (field.position - field.size / 2) / 8;
        params.history_size = resources->settings.history_size;
        params.scroll = frame->update_plan.scroll[cascade];
        params.reset_all = (frame->update_plan.reset_history_cascades >> cascade) & 1u;
        frame->reset_probes->PushConstant(command, 0, params);
        frame->reset_probes->Dispatch(command, (params.probe_size.x + 3) / 4, (params.probe_size.y + 3) / 4,
                                      (params.probe_size.z + 3) / 4);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
  });
  std::string previous = invalidate.name;
  for (uint32_t cascade = 0; cascade < inputs->cascades.size(); ++cascade) {
    if (std::none_of(inputs->regions.begin(), inputs->regions.end(), [cascade](const auto& r) {
          return r.pending.cascade == cascade;
        }))
      continue;
    RenderPassDescriptor pass{"HddagiVoxelCascade" + std::to_string(cascade), RenderPassQueue::Graphics,
                              RenderPassScope::Frame};
    pass.dependencies = {previous};
    previous = pass.name;
    pass.profiler_group = RenderPassProfilerGroup::FramePreparation;
    pass.profiler_display_name = "HDDAGI Field Update";
    pass.resources = reads;
    for (const auto name : {"Albedo", "Emission", "EmissionAniso", "NormalBits", "VoxelBits", "Regions", "Versions",
                            "Disocclusion", "LightNeighbors", "Light"})
      pass.resources.push_back(
          {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Write, RenderResourceState::General});
    for (const auto name : {"ProcessSpare", "DispatchSpare"})
      pass.resources.push_back({"Frame.HDDAGI." + std::string(name) + std::to_string(cascade),
                                RenderResourceUsage::Write, RenderResourceState::General});
    for (const auto name : {"Process", "Dispatch"})
      pass.resources.push_back({"Frame.HDDAGI." + std::string(name) + std::to_string(cascade),
                                RenderResourceUsage::Read, RenderResourceState::General});
    for (const auto name : {"Occlusion0", "Occlusion1"})
      pass.resources.push_back(
          {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
    graph.AddPass(pass, [frame = shared_from_this(), resources, cascade](const RenderGraphExecutionContext& context) {
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
        const RenderPassGpuTimestampScope timing(command, context);
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        ApplyGraphResourceBarriers(command, context);
        const auto dispatch = resources->buffers.at("DispatchSpare" + std::to_string(cascade));
        dispatch->Fill(command, 0, dispatch->GetSize(), 0);
        const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        for (const auto name : {"Albedo", "Emission", "EmissionAniso", "NormalBits"})
          Platform::ClearColorImage(command, *resources->images.at(name).image, VkClearColorValue{}, 1, &range);
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT |
                                   VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_2_INDEX_READ_BIT);
        for (const auto& region : frame->inputs->regions) {
          if (region.pending.cascade != cascade)
            continue;
          for (uint32_t axis = 0; axis < 3; ++axis) {
            frame->inputs->RecordAxis(command, *frame->raster, region, axis);
            resources->OrderAccess(command,
                                   VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
          }
        }
        if (!(frame->update_plan.full_cascades & (1u << cascade))) {
          resources->OrderAccess(
              command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT,
              VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT);
          frame->light_scroll->Bind(command);
          frame->light_scroll->BindDescriptorSet(command, 0, frame->scroll_sets[cascade]->GetVkDescriptorSet());
          HddagiScrollParams params;
          params.grid = frame->inputs->cascades[cascade].size;
          params.capacity = resources->light_cell_capacity;
          params.scroll = frame->update_plan.scroll[cascade];
          params.cascade = cascade;
          params.region_count = frame->update_bounds.size();
          frame->light_scroll->PushConstant(command, 0, params);
          frame->light_scroll->DispatchIndirect(command, *resources->buffers.at("Dispatch" + std::to_string(cascade)));
          resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                 VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        }
        frame->region_store->Bind(command);
        frame->region_store->BindDescriptorSet(command, 0, frame->region_set->GetVkDescriptorSet());
        for (const auto& entry : frame->update_plan.regions) {
          const auto& region = entry.core;
          if (region.cascade != cascade || region.size == glm::ivec3(0))
            continue;
          HddagiRegionParams params;
          params.grid = frame->inputs->cascades[cascade].size;
          params.cascade = cascade;
          params.offset = region.offset;
          params.version = frame->version;
          params.region_world_offset = (frame->inputs->cascades[cascade].position - params.grid / 2) / 8;
          frame->region_store->PushConstant(command, 0, params);
          frame->region_store->Dispatch(command, region.size.x / 8, region.size.y / 8, region.size.z / 8);
        }
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        frame->occlusion->Bind(command);
        for (uint32_t plane = 0; plane < 2; ++plane) {
          frame->occlusion->BindDescriptorSet(command, 0, frame->occlusion_sets[plane]->GetVkDescriptorSet());
          for (const auto& entry : frame->update_plan.regions) {
            const auto& region = entry.core;
            if (region.cascade != cascade || region.size == glm::ivec3(0))
              continue;
            HddagiOcclusionParams params;
            params.grid = frame->inputs->cascades[cascade].size;
            params.cascade = cascade;
            params.offset = region.offset;
            // Reference host offsets (-4, 0) combine with Vulkan RGBA4's high-to-low component order.
            params.layer_offset = (int32_t(plane) - 1) * 4;
            params.region_world_offset = (frame->inputs->cascades[cascade].position - params.grid / 2) / 8;
            frame->occlusion->PushConstant(command, 0, params);
            frame->occlusion->Dispatch(command, region.size.x / 8, region.size.y / 8, region.size.z / 8);
          }
        }
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        frame->light_store->Bind(command);
        frame->light_store->BindDescriptorSet(command, 0, frame->light_sets[cascade]->GetVkDescriptorSet());
        for (uint32_t region_index = 0; region_index < frame->update_plan.regions.size(); ++region_index) {
          const auto& region = frame->update_plan.regions[region_index].light;
          if (region.cascade != cascade)
            continue;
          HddagiLightStoreParams params;
          params.grid = frame->inputs->cascades[cascade].size;
          params.capacity = resources->light_cell_capacity;
          params.offset = region.offset;
          params.limit = params.offset + region.size;
          params.cascade = cascade;
          params.region_index = region_index;
          params.region_world_offset = (frame->inputs->cascades[cascade].position - params.grid / 2) / 8;
          frame->light_store->PushConstant(command, 0, params);
          frame->light_store->Dispatch(command, (region.size.x + 3) / 4, (region.size.y + 3) / 4,
                                       (region.size.z + 3) / 4);
          resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                 VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        }
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                               VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
      });
    });
  }
  RenderPassDescriptor metadata_pass{"HddagiProbeMetadata", RenderPassQueue::Graphics, RenderPassScope::Frame};
  metadata_pass.dependencies = {previous};
  for (const auto name : {"Regions", "Occlusion0", "Occlusion1"})
    metadata_pass.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Read, RenderResourceState::General});
  for (const auto name : {"Neighbors", "Proximity", "CameraVisibility"})
    metadata_pass.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Write, RenderResourceState::General});
  graph.AddPass(metadata_pass, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      frame->metadata->Bind(command);
      frame->metadata->BindDescriptorSet(command, 0, frame->metadata_set->GetVkDescriptorSet());
      for (uint32_t c = 0; c < resources->probes.cascade_count; ++c) {
        if (!(frame->written_cascades & (1u << c)))
          continue;
        HddagiMetadataParams params;
        params.grid = frame->inputs->cascades[c].size;
        params.cascade = c;
        params.region_offset = (frame->inputs->cascades[c].position - params.grid / 2) / 8;
        params.cascade_count = resources->probes.cascade_count;
        params.probe_size = resources->probes.ProbeSize();
        frame->metadata->PushConstant(command, 0, params);
        frame->metadata->Dispatch(command, (params.probe_size.x + 3) / 4, (params.probe_size.y + 3) / 4,
                                  (params.probe_size.z + 3) / 4);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
  });
  previous = metadata_pass.name;
  RenderResourceDescriptor descriptor;
  descriptor.name = "Frame.HDDAGI.StatusReadback";
  descriptor.type = RenderResourceType::Buffer;
  descriptor.lifetime = RenderResourceLifetime::Persistent;
  descriptor.byte_size = status_readback->GetSize();
  graph.AddResource(descriptor);
  registry.BindBuffer(descriptor.name, status_readback);
  RenderPassDescriptor complete{"HddagiVoxelComplete", RenderPassQueue::Graphics, RenderPassScope::Frame};
  complete.dependencies = {previous};
  complete.resources.push_back(
      {descriptor.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  for (uint32_t cascade = 0; cascade < resources->probes.cascade_count; ++cascade)
    complete.resources.push_back({"Frame.HDDAGI." +
                                      std::string(written_cascades & (1u << cascade) ? "DispatchSpare" : "Dispatch") +
                                      std::to_string(cascade),
                                  RenderResourceUsage::Read, RenderResourceState::General});
  graph.AddPass(complete, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                             VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
      for (uint32_t cascade = 0; cascade < resources->probes.cascade_count; ++cascade) {
        const VkBufferCopy copy{0, cascade * 20u, 20};
        vkCmdCopyBuffer(command,
                        resources->buffers
                            .at(std::string(frame->written_cascades & (1u << cascade) ? "DispatchSpare" : "Dispatch") +
                                std::to_string(cascade))
                            ->GetVkBuffer(),
                        frame->status_readback->GetVkBuffer(), 1, &copy);
      }
      Platform::BufferMemoryBarrier(command, *frame->status_readback, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_HOST_BIT,
                                    VK_ACCESS_2_HOST_READ_BIT);
    });
    for (uint32_t cascade = 0; cascade < resources->probes.cascade_count; ++cascade)
      if (frame->written_cascades & (1u << cascade))
        for (const auto name : {"Process", "Dispatch"})
          std::swap(resources->buffers.at(std::string(name) + std::to_string(cascade)),
                    resources->buffers.at(std::string(name) + "Spare" + std::to_string(cascade)));
    frame->status_recorded = resources->voxelization_recorded = true;
  });
}

void HddagiVoxelFrame::ReadStatusAfterFence(HddagiResources& resources) const {
  if (!status_recorded || (resources.last_status_frame != UINT64_MAX && scene_frame < resources.last_status_frame))
    return;
  std::vector<uint32_t> data;
  status_readback->DownloadVector(data, resources.probes.cascade_count * 5);
  resources.light_cell_counts.resize(resources.probes.cascade_count);
  resources.failure_flags = 0;
  for (uint32_t cascade = 0; cascade < resources.probes.cascade_count; ++cascade) {
    resources.light_cell_counts[cascade] = data[cascade * 5 + 3];
    resources.failure_flags |= data[cascade * 5 + 4];
  }
  resources.last_status_frame = scene_frame;
}

uint64_t HddagiVoxelFrame::AllocationBytes() const {
  return inputs->AllocationBytes() + status_readback->GetVmaAllocationInfo().size;
}
