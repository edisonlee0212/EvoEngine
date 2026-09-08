// Godot HDDAGI static/dynamic direct-light ordering, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Static radiance is separate from immutable source emission for editable geometry.
#include "HddagiLight.hpp"
#include <cstring>
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

using namespace evo_engine;

uint64_t HddagiLightFrame::AllocationBytes() const {
  uint64_t bytes = uploads.GetAllocationBytes();
  for (const auto& [name, buffer] : buffers)
    bytes += buffer->GetVmaAllocationInfo().size;
  return bytes;
}

std::shared_ptr<HddagiLightFrame> HddagiLightFrame::Create(HddagiResources& resources,
                                                           const std::vector<SdfgiCascade>& cascade_inputs,
                                                           const std::vector<SdfgiLightInput>& inputs,
                                                           const uint32_t scene_frame, const uint32_t written) {
  auto frame = std::make_shared<HddagiLightFrame>();
  frame->scene_frame = scene_frame;
  frame->written_cascades = written;
  frame->bounce_feedback = resources.transport_recorded ? resources.settings.bounce_feedback : 0;
  if (!resources.direct_pipelines[0]) {
    auto layout = std::make_shared<DescriptorSetLayout>();
    const std::array types{
        VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,  VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,  VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,  VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
        VK_DESCRIPTOR_TYPE_SAMPLER,        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};
    for (uint32_t binding = 0; binding < types.size(); ++binding)
      layout->PushDescriptorBinding(binding, types[binding], VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
    if (const auto failure = SdfgiResources::ValidateDescriptorLimits(
            {layout}, Platform::GetSelectedPhysicalDevice()->properties.limits);
        !failure.empty())
      throw std::runtime_error(failure);
    std::array<std::shared_ptr<ComputePipeline>, 2> pipelines;
    for (uint32_t kind = 0; kind < 2; ++kind) {
      auto pipeline = std::make_shared<ComputePipeline>();
      pipeline->descriptor_set_layouts = {layout};
      pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(HddagiDirectParams)});
      pipeline->compute_shader =
          Shader::CreateTemporary(ShaderType::Compute, kind == 0 ? "#define MODE_PROCESS_STATIC\n" : "",
                                  Resources::GetDefaultResourcesPath() / "Shaders/Compute/HddagiDirectLight.slang");
      pipeline->Initialize();
      if (!pipeline->Initialized())
        throw std::runtime_error("HDDAGI direct-light pipeline creation failed");
      pipelines[kind] = std::move(pipeline);
    }
    VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sampler.magFilter = sampler.minFilter = VK_FILTER_LINEAR;
    sampler.addressModeU = sampler.addressModeV = sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    if (!resources.linear_sampler)
      resources.linear_sampler = std::make_shared<Sampler>(sampler);
    resources.direct_pipelines = std::move(pipelines);
  }
  frame->pipelines = resources.direct_pipelines;
  frame->sampler = resources.linear_sampler;
  const auto buffer = [&](const std::string& name, const size_t bytes, const VkBufferUsageFlags usage) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = bytes;
    info.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    auto result = std::make_shared<Buffer>(info);
    frame->buffers.emplace(name, result);
    return result;
  };
  for (uint32_t c = 0; c < cascade_inputs.size(); ++c) {
    const auto& input = cascade_inputs[c];
    frame->cascades.data[c].offset = glm::vec3(input.position - input.size / 2) * input.cell_size;
    frame->cascades.data[c].to_cell = 1.0f / input.cell_size;
    frame->cascades.data[c].region_world_offset = (input.position - input.size / 2) / 8;
    frame->lights.push_back(BuildSdfgiCascadeLights(inputs, input, c, SdfgiYMultiplier(resources.probes.vertical_scale),
                                                    resources.probes.cascade_count));
    const auto& lights = frame->lights.back();
    for (uint32_t kind = 0; kind < 2; ++kind) {
      const auto& selected = lights.data[kind];
      const bool changed = c >= resources.light_inputs.size() ||
                           selected.size() != resources.light_inputs[c][kind].size() ||
                           (!selected.empty() && std::memcmp(selected.data(), resources.light_inputs[c][kind].data(),
                                                             selected.size() * sizeof(SdfgiLight)) != 0);
      frame->lighting_changed |= changed;
      if (written || changed) {
        if (kind == 0)
          frame->refresh_static |= 1u << c;
        frame->full_dynamic |= 1u << c;
      }
      const auto light_buffer =
          buffer("Lights" + std::to_string(c) + "." + std::to_string(kind),
                 std::max(size_t(1), selected.size()) * sizeof(SdfgiLight), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
      frame->input_uploads.AddVector(light_buffer, selected);
    }
    frame->process.push_back(
        resources.buffers.at(std::string(written & (1u << c) ? "ProcessSpare" : "Process") + std::to_string(c)));
    frame->dispatch.push_back(
        resources.buffers.at(std::string(written & (1u << c) ? "DispatchSpare" : "Dispatch") + std::to_string(c)));
  }
  const auto cascade_buffer = buffer("Cascades", sizeof(frame->cascades), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
  frame->input_uploads.Add(cascade_buffer, frame->cascades, {BufferUploadUsage::Uniform});
  frame->sets.resize(cascade_inputs.size());
  for (uint32_t c = 0; c < cascade_inputs.size(); ++c)
    for (uint32_t kind = 0; kind < 2; ++kind) {
      auto& set = frame->sets[c][kind];
      set = std::make_shared<DescriptorSet>(frame->pipelines[kind]->descriptor_set_layouts[0]);
      set->UpdateBufferDescriptorBinding(2, cascade_buffer);
      set->UpdateBufferDescriptorBinding(3, frame->dispatch[c]);
      set->UpdateBufferDescriptorBinding(4, frame->process[c]);
      set->UpdateBufferDescriptorBinding(5,
                                         frame->buffers.at("Lights" + std::to_string(c) + "." + std::to_string(kind)));
      set->UpdateBufferDescriptorBinding(10, resources.buffers.at("Status"));
      for (const auto& [binding, name] : std::map<uint32_t, std::string>{
               {0, "VoxelBits"}, {1, "Regions"}, {6, "Light"}, {7, "StaticLight"}, {8, "FilteredDiffuse"}}) {
        VkDescriptorImageInfo info{};
        info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
        const auto& image = resources.images.at(name);
        info.imageView = (binding == 6 || binding == 7 ? image.storage : image.sampled)->GetVkImageView();
        set->UpdateImageDescriptorBinding(binding, info);
      }
      VkDescriptorImageInfo info{};
      info.sampler = frame->sampler->GetVkSampler();
      set->UpdateImageDescriptorBinding(9, info);
    }
  return frame;
}

void HddagiLightFrame::AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                 const std::shared_ptr<HddagiResources>& resources, const std::string& dependency) {
  RenderPassDescriptor upload{"HddagiLightInputs", RenderPassQueue::Graphics, RenderPassScope::Frame};
  if (!dependency.empty())
    upload.dependencies = {dependency};
  for (const auto& [name, buffer] : buffers) {
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.HDDAGI.Direct." + name;
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = buffer->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, buffer);
    upload.resources.push_back(
        {descriptor.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  }
  graph.AddPass(upload, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      ApplyGraphResourceBarriers(command, context);
    });
    frame->input_uploads.Record(frame->uploads);
  });
  RenderPassDescriptor pass{"HddagiDirectLight", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.profiler_group = RenderPassProfilerGroup::FramePreparation;
  pass.profiler_display_name = "HDDAGI Direct Light";
  pass.dependencies = {upload.name};
  pass.resources.push_back({"Frame.HDDAGI.Status", RenderResourceUsage::ReadWrite, RenderResourceState::General});
  for (const auto& [name, buffer] : buffers)
    pass.resources.push_back({"Frame.HDDAGI.Direct." + name, RenderResourceUsage::Read, RenderResourceState::General});
  for (const auto name : {"VoxelBits", "Regions", "FilteredDiffuse"})
    pass.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::Read, RenderResourceState::General});
  for (const auto name : {"Light", "StaticLight"})
    pass.resources.push_back(
        {"Frame.HDDAGI." + std::string(name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
  for (uint32_t c = 0; c < lights.size(); ++c) {
    const std::string suffix = std::string(written_cascades & (1u << c) ? "Spare" : "") + std::to_string(c);
    pass.resources.push_back(
        {"Frame.HDDAGI.Process" + suffix, RenderResourceUsage::ReadWrite, RenderResourceState::General});
    pass.resources.push_back(
        {"Frame.HDDAGI.Dispatch" + suffix, RenderResourceUsage::Read, RenderResourceState::General});
  }
  graph.AddPass(pass, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      ApplyGraphResourceBarriers(command, context);
      HddagiDirectParams params;
      params.grid = (resources->probes.ProbeSize() - 1) * 8;
      params.probe_size = resources->probes.ProbeSize();
      params.cascade_count = resources->probes.cascade_count;
      params.capacity = resources->light_cell_capacity;
      params.y_mult = SdfgiYMultiplier(resources->probes.vertical_scale);
      params.bounce_feedback = frame->bounce_feedback;
      for (uint32_t c = 0; c < frame->lights.size(); ++c)
        for (uint32_t kind = 0; kind < 2; ++kind) {
          resources->OrderAccess(
              command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT,
              VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT);
          const auto& pipeline = frame->pipelines[kind];
          pipeline->Bind(command);
          pipeline->BindDescriptorSet(command, 0, frame->sets[c][kind]->GetVkDescriptorSet());
          params.cascade = c;
          params.light_count = frame->lights[c].data[kind].size();
          params.refresh_static = (frame->refresh_static >> c) & 1;
          params.process_increment = frame->full_dynamic & (1u << c) ? 1 : resources->settings.light_update_frames;
          params.process_offset = frame->scene_frame % params.process_increment;
          pipeline->PushConstant(command, 0, params);
          pipeline->DispatchIndirect(command, *frame->dispatch[c]);
        }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                             VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
    });
    resources->light_inputs.clear();
    resources->light_overflow.clear();
    for (const auto& lights : frame->lights) {
      resources->light_inputs.push_back(lights.data);
      resources->light_overflow.push_back(lights.overflow);
    }
  });
}
