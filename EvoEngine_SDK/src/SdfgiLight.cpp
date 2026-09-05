// Direct-light sequencing adapted from Godot gi.cpp::SDFGI::{render_static_lights,update_light},
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiLight.hpp"

#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <stb_image_write.h>
#include <cstring>

using namespace evo_engine;

namespace {
std::string CascadeName(const uint32_t c, const std::string& name) {
  return "Cascade" + std::to_string(c) + "." + name;
}
std::string FrameName(const uint32_t f, const std::string& name) {
  return "Frame" + std::to_string(f) + "." + name;
}
bool SameLights(const std::vector<SdfgiLight>& a, const std::vector<SdfgiLight>& b) {
  return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(SdfgiLight)) == 0);
}
}  // namespace

SdfgiLightDebug::SdfgiLightDebug(const uint32_t index, const uint32_t plane) : cascade(index), slice(plane) {
  for (uint32_t i = 0; i < planes.size(); ++i) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = 3 * 128 * 128 * (i == 2 ? 2 : 4);
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    planes[i] = std::make_shared<Buffer>(info, allocation);
  }
}

void SdfgiLightDebug::AddPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                              const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency) {
  RenderPassDescriptor pass{"SdfgiLightDebug", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.dependencies = {dependency};
  for (const auto* name : {"Light", "Aniso0", "Aniso1"})
    pass.resources.push_back(
        {"Frame.SDFGI." + CascadeName(cascade, name), RenderResourceUsage::Read, RenderResourceState::General});
  for (uint32_t i = 0; i < planes.size(); ++i) {
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.SDFGI.LightDebug" + std::to_string(i);
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = planes[i]->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, planes[i]);
    pass.resources.push_back(
        {descriptor.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  }
  graph.AddPass(pass, [snapshot = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
      const char* names[]{"Light", "Aniso0", "Aniso1"};
      for (uint32_t i = 0; i < snapshot->planes.size(); ++i) {
        std::array<VkBufferImageCopy, 3> copies{};
        for (uint32_t axis = 0; axis < 3; ++axis) {
          auto& copy = copies[axis];
          copy.bufferOffset = axis * 128 * 128 * (i == 2 ? 2 : 4);
          copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
          copy.imageExtent = {128, 128, 128};
          if (axis == 0) {
            copy.imageOffset.x = snapshot->slice;
            copy.imageExtent.width = 1;
          } else if (axis == 1) {
            copy.imageOffset.y = snapshot->slice;
            copy.imageExtent.height = 1;
          } else {
            copy.imageOffset.z = snapshot->slice;
            copy.imageExtent.depth = 1;
          }
        }
        vkCmdCopyImageToBuffer(
            command, resources->textures.at(CascadeName(snapshot->cascade, names[i])).image->GetVkImage(),
            VK_IMAGE_LAYOUT_GENERAL, snapshot->planes[i]->GetVkBuffer(), copies.size(), copies.data());
      }
      VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
      barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
      barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
      barrier.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
      barrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
      VkDependencyInfo dependency_info{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
      dependency_info.memoryBarrierCount = 1;
      dependency_info.pMemoryBarriers = &barrier;
      vkCmdPipelineBarrier2(command, &dependency_info);
    });
    snapshot->recorded = true;
  });
}

void SdfgiLightDebug::StoreToPng(const std::filesystem::path& path) const {
  if (!recorded)
    throw std::runtime_error("SDFGI lighting diagnostic has not been recorded");
  Platform::WaitForFrameSubmissions("SDFGI explicit light-volume diagnostic readback");
  std::vector<uint32_t> light;
  std::vector<uint8_t> aniso0, aniso1;
  planes[0]->DownloadVector(light, 3 * 128 * 128);
  planes[1]->DownloadVector(aniso0, 3 * 128 * 128 * 4);
  planes[2]->DownloadVector(aniso1, 3 * 128 * 128 * 2);
  std::vector<uint8_t> pixels(2560 * 1440 * 4, 255);
  for (int y = 0; y < 1440; ++y)
    for (int x = 0; x < 2560; ++x) {
      const uint32_t axis = y / 480, column = x / 365;
      const int u = x % 365 - 54, v = y % 480 - 112;
      glm::vec3 color(6.0f / 255.0f);
      if (column < 7 && u >= 0 && v >= 0 && u < 256 && v < 256) {
        const uint32_t right = u / 2, up = 127 - v / 2;
        const uint32_t offset = axis * 128 * 128 + (axis == 1 ? up + right * 128 : right + up * 128);
        const uint32_t packed = light[offset];
        color = glm::vec3(packed & 511, (packed >> 9) & 511, (packed >> 18) & 511) *
                std::ldexp(1.0f, static_cast<int>(packed >> 27) - 24);
        if (column != 0)
          color *= (column <= 4 ? aniso0[offset * 4 + column - 1] : aniso1[offset * 2 + column - 5]) / 255.0f;
        color = glm::pow(color / (1.0f + color), glm::vec3(1.0f / 2.2f));
      }
      for (uint32_t channel = 0; channel < 3; ++channel)
        pixels[(y * 2560 + x) * 4 + channel] = static_cast<uint8_t>(glm::clamp(color[channel], 0.0f, 1.0f) * 255);
    }
  stbi_flip_vertically_on_write(false);
  if (!stbi_write_png(path.string().c_str(), 2560, 1440, 4, pixels.data(), 2560 * 4))
    throw std::runtime_error("Could not write SDFGI lighting diagnostic PNG");
}

uint64_t SdfgiLightDebug::AllocationBytes() const {
  uint64_t bytes = 0;
  for (const auto& plane : planes)
    bytes += plane->GetVmaAllocationInfo().size;
  return bytes;
}

SdfgiCascadeLights evo_engine::BuildSdfgiCascadeLights(const std::vector<SdfgiLightInput>& inputs,
                                                       const SdfgiCascade& cascade, const uint32_t index,
                                                       const float y_mult) {
  SdfgiCascadeLights result;
  std::array<std::vector<SdfgiLightInput>, 2> selected;
  // Preserve the reference cascade-AABB comparison before Y-adjusting uploaded light positions.
  const Bound bounds{glm::vec3(cascade.position - glm::ivec3(64)) * cascade.cell_size,
                     glm::vec3(cascade.position + glm::ivec3(64)) * cascade.cell_size};
  for (const auto& input : inputs) {
    const bool directional = input.type == SdfgiLightInput::Type::Directional;
    if (!directional && (index > 2 || glm::any(glm::greaterThanEqual(input.world_bounds.min, bounds.max)) ||
                         glm::any(glm::lessThanEqual(input.world_bounds.max, bounds.min))))
      continue;
    selected[directional || input.dynamic ? 1 : 0].push_back(input);
  }
  for (uint32_t kind = 0; kind < 2; ++kind) {
    result.overflow[kind] = BoundSdfgiLightList(selected[kind], kind != 0);
    for (const auto& input : selected[kind]) {
      SdfgiLight light{};
      const auto position = input.position * glm::vec3(1, y_mult, 1);
      const auto direction = input.type == SdfgiLightInput::Type::Directional
                                 ? glm::normalize(input.direction * glm::vec3(1, y_mult, 1))
                                 : input.direction;
      for (uint32_t axis = 0; axis < 3; ++axis) {
        light.color[axis] = input.color[axis];
        light.position[axis] = position[axis];
        light.direction[axis] = direction[axis];
        light.host_photometry[axis] = input.attenuation[axis];
      }
      light.energy = 1;
      light.has_shadow = input.casts_shadow;
      light.type = static_cast<uint32_t>(input.type);
      light.cos_spot_angle = input.cos_outer;
      light.host_photometry[3] = input.cos_inner;
      light.radius = input.range;
      result.data[kind].push_back(light);
    }
  }
  return result;
}

std::shared_ptr<SdfgiLightFrame> SdfgiLightFrame::Create(const SdfgiResources& resources,
                                                         const std::vector<SdfgiCascade>& cascade_inputs,
                                                         const std::vector<SdfgiLightInput>& inputs,
                                                         const uint32_t frame_number, const uint32_t rebuilt) {
  auto frame = std::make_shared<SdfgiLightFrame>();
  frame->frame_slot = Platform::GetCurrentFrameIndex();
  frame->scene_frame = frame_number;
  frame->rebuilt_cascades = rebuilt;
  frame->settings = resources.settings;
  frame->bounce_feedback = resources.transport_recorded ? resources.settings.bounce_feedback : 0;
  frame->cascades = BuildSdfgiCascadeBlock(cascade_inputs);
  frame->input_uploads.Add(resources.buffers.at(FrameName(frame->frame_slot, "Cascades")).buffer, frame->cascades,
                           {BufferUploadUsage::Uniform});
  frame->lights.resize(cascade_inputs.size());
  for (uint32_t c = 0; c < cascade_inputs.size(); ++c) {
    auto& lights = frame->lights[c];
    lights = BuildSdfgiCascadeLights(inputs, cascade_inputs[c], c, SdfgiYMultiplier(resources.settings.vertical_scale));
    if ((rebuilt & (1u << c)) || c >= resources.static_light_inputs.size() ||
        !SameLights(lights.data[0], resources.static_light_inputs[c]) || !resources.light_failure.empty())
      frame->static_refresh |= 1u << c;
    for (uint32_t kind = 0; kind < 2; ++kind)
      frame->input_uploads.AddVector(
          resources.buffers
              .at(FrameName(frame->frame_slot, CascadeName(c, kind == 0 ? "StaticLights" : "DynamicLights")))
              .buffer,
          lights.data[kind]);
  }
  frame->full_dynamic = rebuilt | frame->static_refresh;
  return frame;
}

void SdfgiLightFrame::AddPasses(RenderGraph& graph, const std::shared_ptr<SdfgiResources>& resources,
                                const std::string& dependency) {
  RenderPassDescriptor upload{"SdfgiLightInputs", RenderPassQueue::Graphics, RenderPassScope::Frame};
  upload.dependencies = {dependency};
  upload.resources.push_back({"Frame.SDFGI." + FrameName(frame_slot, "Cascades"), RenderResourceUsage::Write,
                              RenderResourceState::TransferDestinationGeneral});
  for (uint32_t c = 0; c < lights.size(); ++c)
    for (const auto* kind : {"StaticLights", "DynamicLights"})
      upload.resources.push_back({"Frame.SDFGI." + FrameName(frame_slot, CascadeName(c, kind)),
                                  RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  graph.AddPass(upload, [frame = shared_from_this()](const RenderGraphExecutionContext&) {
    frame->input_uploads.Record(frame->uploads);
  });
  RenderPassDescriptor pass{"SdfgiDirectLight", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.dependencies = {upload.name};
  pass.profiler_group = RenderPassProfilerGroup::FramePreparation;
  pass.profiler_display_name = "SDFGI Direct Light";
  for (const auto& access : upload.resources)
    pass.resources.push_back({access.resource_name, RenderResourceUsage::Read, RenderResourceState::General});
  for (const auto* name : {"Atlas", "Occlusion", "Status"})
    pass.resources.push_back(
        {"Frame.SDFGI." + std::string(name), RenderResourceUsage::Read, RenderResourceState::General});
  for (uint32_t c = 0; c < lights.size(); ++c) {
    for (const auto* name : {"Sdf", "Dispatch", "Indirect"})
      pass.resources.push_back(
          {"Frame.SDFGI." + CascadeName(c, name), RenderResourceUsage::Read, RenderResourceState::General});
    for (const auto* name : {"SolidCells", "UnlitCells", "Light", "Aniso0", "Aniso1"})
      pass.resources.push_back(
          {"Frame.SDFGI." + CascadeName(c, name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
  }
  graph.AddPass(pass, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    bool overflow = false;
    for (const auto& lights : frame->lights)
      overflow |= lights.overflow[0] != 0 || lights.overflow[1] != 0;
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                             VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
      ApplyGraphResourceBarriers(command, context);
      const VkBufferCopy copy{0, 0, sizeof(SdfgiSolidCell) * kSdfgiSolidCellCapacity};
      if (overflow)
        return;
      const auto timing =
          Platform::BeginGpuTimestampScope(command, {"SdfgiDirectLight", "SDFGI Direct Light", "SDFGI"});
      for (uint32_t c = 0; c < frame->lights.size(); ++c)
        if (frame->static_refresh & ~frame->rebuilt_cascades & (1u << c))
          vkCmdCopyBuffer(command, resources->buffers.at(CascadeName(c, "UnlitCells")).buffer->GetVkBuffer(),
                          resources->buffers.at(CascadeName(c, "SolidCells")).buffer->GetVkBuffer(), 1, &copy);
      resources->OrderAccess(
          command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT,
          VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT);
      SdfgiDirectLightPushConstant params{};
      for (auto& size : params.grid_size)
        size = 128;
      params.max_cascades = frame->settings.cascade_count;
      params.probe_axis_size = 17;
      params.y_mult = SdfgiYMultiplier(frame->settings.vertical_scale);
      params.use_occlusion = frame->settings.use_occlusion;
      for (uint32_t kind = 0; kind < 2; ++kind) {
        const auto& pipeline = resources->pipelines.at(kind == 0 ? "DirectLightSTATIC" : "DirectLightDYNAMIC");
        pipeline->Bind(command);
        for (uint32_t c = 0; c < frame->lights.size(); ++c) {
          params.cascade = c;
          params.light_count = frame->lights[c].data[kind].size();
          if (kind == 0 && (!(frame->static_refresh & (1u << c)) || params.light_count == 0))
            continue;
          params.process_increment =
              kind == 0 || (frame->full_dynamic & (1u << c)) ? 1 : frame->settings.light_update_frames;
          params.process_offset = frame->scene_frame % params.process_increment;
          params.bounce_feedback = kind == 0 ? 0 : frame->bounce_feedback;
          pipeline->PushConstant(command, 0, params);
          pipeline->BindDescriptorSet(
              command, 0,
              resources->sets
                  .at(FrameName(frame->frame_slot, CascadeName(c, kind == 0 ? "StaticLights" : "DynamicLights")))
                  ->GetVkDescriptorSet());
          pipeline->DispatchIndirect(command, *resources->buffers.at(CascadeName(c, "Indirect")).buffer);
        }
        resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      }
      Platform::EndGpuTimestampScope(command, timing);
    });
    resources->light_failure = overflow ? "SDFGI light-list capacity overflow" : "";
    resources->lighting_recorded = !overflow;
    resources->static_light_inputs.resize(frame->lights.size());
    if (!overflow)
      for (uint32_t c = 0; c < frame->lights.size(); ++c)
        resources->static_light_inputs[c] = frame->lights[c].data[0];
  });
}
