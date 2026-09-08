// Godot HDDAGI probe integration host contract, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt.
#include "HddagiProbe.hpp"
#include "Cubemap.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"
#include "SdfgiResources.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"

using namespace evo_engine;

uint64_t HddagiProbeFrame::AllocationBytes() const {
  return cascade_buffer->GetVmaAllocationInfo().size + uploads.GetAllocationBytes() +
         status_readback->GetVmaAllocationInfo().size;
}

void HddagiProbeFrame::ReadStatusAfterFence(HddagiResources& resources) const {
  if (!status_recorded ||
      (resources.last_transport_status_frame != UINT64_MAX && scene_frame < resources.last_transport_status_frame))
    return;
  std::array<uint32_t, 4> status{};
  status_readback->Download(status);
  resources.transport_generation = status[0];
  resources.transport_failure_flags = status[1];
  resources.transport_ready = status[2] != 0 && status[1] == 0;
  resources.last_transport_status_frame = scene_frame;
}

std::shared_ptr<HddagiProbeFrame> HddagiProbeFrame::Create(HddagiResources& resources,
                                                           const std::vector<SdfgiCascade>& cascade_inputs,
                                                           const SdfgiSkyInput& sky, const uint32_t scene_frame,
                                                           const bool force_update, const uint32_t written_cascades) {
  auto frame = std::make_shared<HddagiProbeFrame>();
  frame->scene_frame = scene_frame;
  frame->written_cascades = written_cascades;
  VkBufferCreateInfo readback_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  readback_info.size = 16;
  readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO;
  allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  frame->status_readback = std::make_shared<Buffer>(readback_info, allocation);
  if (!resources.transport_status_pipelines[0]) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t i = 0; i < 2; ++i)
      layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
    std::array<std::shared_ptr<ComputePipeline>, 2> pipelines;
    for (uint32_t i = 0; i < 2; ++i) {
      auto pipeline = std::make_shared<ComputePipeline>();
      pipeline->descriptor_set_layouts = {layout};
      pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, 16});
      pipeline->compute_shader =
          Shader::CreateTemporary(ShaderType::Compute, i == 0 ? "#define MODE_BEGIN\n" : "",
                                  Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiTransportStatus.slang");
      pipeline->Initialize();
      if (!pipeline->Initialized())
        throw std::runtime_error("HDDAGI transport-status pipeline creation failed");
      pipelines[i] = std::move(pipeline);
    }
    resources.transport_status_pipelines = std::move(pipelines);
  }
  frame->status_pipelines = resources.transport_status_pipelines;
  for (uint32_t c = 0; c < resources.probes.cascade_count; ++c) {
    auto set = std::make_shared<DescriptorSet>(frame->status_pipelines[0]->descriptor_set_layouts[0]);
    set->UpdateBufferDescriptorBinding(
        0, resources.buffers.at(std::string(written_cascades & (1u << c) ? "DispatchSpare" : "Dispatch") +
                                std::to_string(c)));
    set->UpdateBufferDescriptorBinding(1, resources.buffers.at("Status"));
    frame->status_sets.push_back(std::move(set));
  }
  if (!resources.integrate_pipeline) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 17; ++binding) {
      const auto type = binding == 16                   ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
                        : binding == 2                  ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER
                        : binding == 15                 ? VK_DESCRIPTOR_TYPE_SAMPLER
                        : binding >= 5 && binding <= 12 ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE
                                                        : VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
      layout->PushDescriptorBinding(binding, type, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    }
    layout->Initialize();
    if (const auto failure = SdfgiResources::ValidateDescriptorLimits(
            {layout}, Platform::GetSelectedPhysicalDevice()->properties.limits);
        !failure.empty())
      throw std::runtime_error(failure);
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->descriptor_set_layouts = {layout};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiIntegrateParams)});
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, "", Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiIntegrate.slang");
    pipeline->Initialize();
    if (!pipeline->Initialized())
      throw std::runtime_error("HDDAGI integration pipeline creation failed");
    VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sampler.minFilter = sampler.magFilter = VK_FILTER_LINEAR;
    sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampler.addressModeU = sampler.addressModeV = sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler.maxLod = VK_LOD_CLAMP_NONE;
    resources.mip_sampler = std::make_shared<Sampler>(sampler);
    resources.integrate_pipeline = std::move(pipeline);
  }
  frame->pipeline = resources.integrate_pipeline;
  if (!resources.filter_pipeline) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 4; ++binding)
      layout->PushDescriptorBinding(binding,
                                    binding < 3 ? VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE : VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                    VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
    auto pipeline = std::make_shared<ComputePipeline>();
    pipeline->descriptor_set_layouts = {layout};
    pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiFilterParams)});
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, "", Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiFilterProbes.slang");
    pipeline->Initialize();
    if (!pipeline->Initialized())
      throw std::runtime_error("HDDAGI probe-filter pipeline creation failed");
    resources.filter_pipeline = std::move(pipeline);
  }
  frame->filter_pipeline = resources.filter_pipeline;
  frame->filter_set = std::make_shared<DescriptorSet>(frame->filter_pipeline->descriptor_set_layouts[0]);
  uint32_t filter_binding = 0;
  for (const auto name : {"Diffuse", "Neighbors", "Proximity", "FilteredDiffuse"}) {
    const auto& image = resources.images.at(name);
    VkDescriptorImageInfo info{};
    info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    info.imageView = (filter_binding == 3 ? image.storage : image.sampled)->GetVkImageView();
    frame->filter_set->UpdateImageDescriptorBinding(filter_binding++, info);
  }
  frame->sampler = resources.mip_sampler;
  frame->set = std::make_shared<DescriptorSet>(frame->pipeline->descriptor_set_layouts[0]);
  frame->set->UpdateBufferDescriptorBinding(16, resources.buffers.at("Status"));
  auto& params = frame->params;
  params.probe_size = resources.probes.ProbeSize();
  params.grid = (params.probe_size - 1) * 8;
  params.cascade_count = resources.probes.cascade_count;
  params.history_size = resources.settings.history_size;
  params.ray_bias = resources.settings.probe_bias;
  params.y_mult = SdfgiYMultiplier(resources.probes.vertical_scale);
  params.global_frame = scene_frame;
  const auto& previous_sky = resources.sky_input;
  const bool sky_changed =
      sky.cubemap != previous_sky.cubemap || sky.map_id != previous_sky.map_id ||
      sky.map_version != previous_sky.map_version || sky.cubemap_version != previous_sky.cubemap_version ||
      sky.constant_color != previous_sky.constant_color || sky.color != previous_sky.color ||
      sky.gamma != previous_sky.gamma || sky.rotation != previous_sky.rotation || sky.energy != previous_sky.energy;
  const uint32_t force_frames = force_update || sky_changed ? 4 : resources.force_probe_frames;
  params.force_update = force_frames != 0;
  frame->force_frames_remaining = force_frames ? force_frames - 1 : 0;
  frame->sky = sky;
  if (resources.settings.read_sky_light) {
    params.sky_mode = sky.constant_color ? 1 : 2;
    params.sky_color = sky.color;
    params.sky_energy = sky.energy;
    params.sky_rotation = sky.rotation;
    if (!sky.constant_color) {
      VkDescriptorImageInfo info{};
      if (!sky.cubemap ||
          !TextureStorage::TryGetCubemapDescriptorImageInfo(sky.cubemap->GetTextureStorageIndex(), info))
        throw std::runtime_error("HDDAGI scene sky radiance cubemap is not ready");
      if (!std::isfinite(sky.gamma) || sky.gamma <= 0)
        throw std::runtime_error("HDDAGI scene sky gamma must be positive and finite");
      params.sky_inverse_gamma = 1 / sky.gamma;
      frame->sky_image = sky.cubemap->PeekStorage().image;
      frame->sky_view = sky.cubemap->PeekStorage().image_view;
      frame->set->UpdateImageDescriptorBinding(14, info);
    }
  }
  for (uint32_t c = 0; c < cascade_inputs.size(); ++c) {
    const auto& source = cascade_inputs[c];
    frame->cascades.data[c].offset = glm::vec3(source.position - source.size / 2) * source.cell_size;
    frame->cascades.data[c].to_cell = 1 / source.cell_size;
    frame->cascades.data[c].region_world_offset = (source.position - source.size / 2) / 8;
  }
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = sizeof(frame->cascades);
  info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  frame->cascade_buffer = std::make_shared<Buffer>(info);
  frame->input_uploads.Add(frame->cascade_buffer, frame->cascades, {BufferUploadUsage::Uniform});
  frame->set->UpdateBufferDescriptorBinding(2, frame->cascade_buffer);
  const char* names[]{"VoxelBits",    "Regions",   "",           "Light",    "Versions", "HitCache",
                      "HitVersions",  "History",   "HistorySum", "Specular", "Diffuse",  "Ambient",
                      "ProcessFrame", "Proximity", "BlackSky"};
  for (uint32_t binding = 0; binding < 15; ++binding) {
    if (binding == 2 || (binding == 14 && frame->sky_view))
      continue;
    const auto& image = resources.images.at(names[binding]);
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = (binding >= 5 && binding <= 12 ? image.storage : image.sampled)->GetVkImageView();
    frame->set->UpdateImageDescriptorBinding(binding, image_info);
  }
  VkDescriptorImageInfo sampler_info{};
  sampler_info.sampler = frame->sampler->GetVkSampler();
  frame->set->UpdateImageDescriptorBinding(15, sampler_info);
  return frame;
}

void HddagiProbeFrame::AddBeginPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                    const std::shared_ptr<HddagiResources>& resources, const std::string& dependency) {
  RenderPassDescriptor pass{"HddagiTransportBegin", RenderPassQueue::Graphics, RenderPassScope::Frame};
  if (!dependency.empty())
    pass.dependencies = {dependency};
  pass.resources.push_back({"Frame.HDDAGI.Status", RenderResourceUsage::ReadWrite, RenderResourceState::General});
  for (uint32_t c = 0; c < resources->probes.cascade_count; ++c)
    pass.resources.push_back(
        {"Frame.HDDAGI." + std::string(written_cascades & (1u << c) ? "DispatchSpare" : "Dispatch") + std::to_string(c),
         RenderResourceUsage::Read, RenderResourceState::General});
  graph.AddPass(pass, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      ApplyGraphResourceBarriers(command, context);
      frame->status_pipelines[0]->Bind(command);
      for (uint32_t c = 0; c < resources->probes.cascade_count; ++c) {
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        frame->status_pipelines[0]->BindDescriptorSet(command, 0, frame->status_sets[c]->GetVkDescriptorSet());
        frame->status_pipelines[0]->PushConstant(
            command, 0, glm::uvec4(c, resources->light_cell_capacity, uint32_t(frame->scene_frame + 1), 0));
        frame->status_pipelines[0]->Dispatch(command, 1, 1, 1);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
  });
}

void HddagiProbeFrame::AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                 const std::shared_ptr<HddagiResources>& resources, const std::string& dependency) {
  RenderResourceDescriptor descriptor;
  descriptor.name = "Frame.HDDAGI.Probe.Cascades";
  descriptor.type = RenderResourceType::Buffer;
  descriptor.lifetime = RenderResourceLifetime::Persistent;
  descriptor.byte_size = cascade_buffer->GetSize();
  graph.AddResource(descriptor);
  registry.BindBuffer(descriptor.name, cascade_buffer);
  RenderPassDescriptor upload{"HddagiProbeInputs", RenderPassQueue::Graphics, RenderPassScope::Frame};
  if (!dependency.empty())
    upload.dependencies = {dependency};
  upload.resources.push_back(
      {descriptor.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  graph.AddPass(upload, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      ApplyGraphResourceBarriers(command, context);
    });
    frame->input_uploads.Record(frame->uploads);
  });
  RenderPassDescriptor pass{"HddagiIntegrate", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.profiler_group = RenderPassProfilerGroup::FramePreparation;
  pass.profiler_display_name = "HDDAGI Integrate";
  pass.dependencies = {upload.name};
  pass.resources.push_back({"Frame.HDDAGI.Status", RenderResourceUsage::ReadWrite, RenderResourceState::General});
  pass.resources.push_back({descriptor.name, RenderResourceUsage::Read, RenderResourceState::General});
  for (const auto name : {"VoxelBits", "Regions", "Versions", "Light", "Proximity", "BlackSky"})
    pass.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Read, RenderResourceState::General});
  for (const auto name :
       {"HitCache", "HitVersions", "History", "HistorySum", "Specular", "Diffuse", "Ambient", "ProcessFrame"})
    pass.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
  if (sky_image) {
    RenderResourceDescriptor image;
    image.name = "Frame.HDDAGI.Probe.Sky";
    image.type = RenderResourceType::Image;
    image.lifetime = RenderResourceLifetime::Persistent;
    graph.AddResource(image);
    registry.BindImage(image.name, sky_image);
    pass.resources.push_back({image.name, RenderResourceUsage::Read,
                              sky_image->GetLayout() == VK_IMAGE_LAYOUT_GENERAL ? RenderResourceState::General
                                                                                : RenderResourceState::ShaderRead});
  }
  graph.AddPass(pass, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      frame->pipeline->Bind(command);
      frame->pipeline->BindDescriptorSet(command, 0, frame->set->GetVkDescriptorSet());
      auto params = frame->params;
      for (uint32_t c = 0; c < params.cascade_count; ++c) {
        params.cascade = c;
        params.world_offset = frame->cascades.data[c].region_world_offset;
        frame->pipeline->PushConstant(command, 0, params);
        frame->pipeline->Dispatch(command, params.probe_size.x, params.probe_size.y * params.probe_size.z, 1);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
  });
  RenderPassDescriptor filter{"HddagiFilterProbes", RenderPassQueue::Graphics, RenderPassScope::Frame};
  filter.profiler_group = RenderPassProfilerGroup::FramePreparation;
  filter.profiler_display_name = "HDDAGI Filter Probes";
  filter.dependencies = {pass.name};
  for (const auto name : {"Diffuse", "Neighbors", "Proximity"})
    filter.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Read, RenderResourceState::General});
  filter.resources.push_back(
      {"Frame.HDDAGI.FilteredDiffuse", RenderResourceUsage::Write, RenderResourceState::General});
  graph.AddPass(filter, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      frame->filter_pipeline->Bind(command);
      frame->filter_pipeline->BindDescriptorSet(command, 0, frame->filter_set->GetVkDescriptorSet());
      HddagiFilterParams params;
      params.probe_size = resources->probes.ProbeSize();
      params.enabled = resources->settings.filter_probes;
      for (uint32_t c = 0; c < resources->probes.cascade_count; ++c) {
        params.cascade = c;
        params.region_offset = frame->cascades.data[c].region_world_offset;
        frame->filter_pipeline->PushConstant(command, 0, params);
        frame->filter_pipeline->Dispatch(command, params.probe_size.x, params.probe_size.y * params.probe_size.z, 1);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
  });
  descriptor.name = "Frame.HDDAGI.TransportReadback";
  descriptor.byte_size = 16;
  graph.AddResource(descriptor);
  registry.BindBuffer(descriptor.name, status_readback);
  RenderPassDescriptor complete{"HddagiTransportComplete", RenderPassQueue::Graphics, RenderPassScope::Frame};
  complete.dependencies = {filter.name};
  complete.resources.push_back({"Frame.HDDAGI.Status", RenderResourceUsage::ReadWrite, RenderResourceState::General});
  complete.resources.push_back(
      {descriptor.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  graph.AddPass(complete, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      frame->status_pipelines[1]->Bind(command);
      frame->status_pipelines[1]->BindDescriptorSet(command, 0, frame->status_sets[0]->GetVkDescriptorSet());
      frame->status_pipelines[1]->PushConstant(command, 0, glm::uvec4(0));
      frame->status_pipelines[1]->Dispatch(command, 1, 1, 1);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                             VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
      const VkBufferCopy copy{0, 0, 16};
      vkCmdCopyBuffer(command, resources->buffers.at("Status")->GetVkBuffer(), frame->status_readback->GetVkBuffer(), 1,
                      &copy);
      Platform::BufferMemoryBarrier(command, *frame->status_readback, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_HOST_BIT,
                                    VK_ACCESS_2_HOST_READ_BIT);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
    frame->status_recorded = true;
    resources->transport_recorded = true;
    resources->sky_input = frame->sky;
    resources->force_probe_frames = frame->force_frames_remaining;
  });
}
