// Full-grid PROCESS followed by all-cascade STORE, from Godot gi.cpp::SDFGI::{update_probes,store_probes},
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiProbe.hpp"

#include "Cubemap.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "TextureStorage.hpp"

#include <stb_image_write.h>

using namespace evo_engine;

namespace {
std::string CascadeName(const uint32_t c, const std::string& name) {
  return "Cascade" + std::to_string(c) + "." + name;
}
std::string FrameName(const uint32_t f, const std::string& name) {
  return "Frame" + std::to_string(f) + "." + name;
}
}  // namespace

SdfgiProbeDebug::SdfgiProbeDebug(const SdfgiSettings& settings, const uint32_t index, const uint32_t selected_probe)
    : cascade(index),
      probe(selected_probe),
      cascade_count(settings.cascade_count),
      history_size(settings.history_size),
      probe_axis(settings.ProbeSize().x),
      columns(probe_axis * probe_axis) {
  const VkDeviceSize sizes[]{uint64_t(columns) * 8 * 136 * 2 * cascade_count * 4, uint64_t(columns) * 272 * 16,
                             16ull * history_size * 8, sizeof(SdfgiFieldStatus)};
  for (uint32_t i = 0; i < data.size(); ++i) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = sizes[i];
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    data[i] = std::make_shared<Buffer>(info, allocation);
  }
}

void SdfgiProbeDebug::AddPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                              const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency) {
  RenderPassDescriptor pass{"SdfgiProbeDebug", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.dependencies = {dependency};
  for (const auto& name :
       {std::string("Atlas"), CascadeName(cascade, "Average"), CascadeName(cascade, "History"), std::string("Status")})
    pass.resources.push_back({"Frame.SDFGI." + name, RenderResourceUsage::Read, RenderResourceState::General});
  for (uint32_t i = 0; i < data.size(); ++i) {
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.SDFGI.ProbeDebug" + std::to_string(i);
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = data[i]->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, data[i]);
    pass.resources.push_back(
        {descriptor.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  }
  graph.AddPass(pass, [snapshot = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      ApplyGraphResourceBarriers(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
      const std::string names[]{"Atlas", CascadeName(snapshot->cascade, "Average"),
                                CascadeName(snapshot->cascade, "History")};
      for (uint32_t i = 0; i < 3; ++i) {
        VkBufferImageCopy copy{};
        copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0,
                                 i == 0   ? 2 * snapshot->cascade_count
                                 : i == 2 ? snapshot->history_size
                                          : 1};
        copy.imageExtent = i == 0   ? VkExtent3D{snapshot->columns * 8, 136, 1}
                           : i == 1 ? VkExtent3D{snapshot->columns, 272, 1}
                                    : VkExtent3D{1, 16, 1};
        if (i == 2)
          copy.imageOffset = {static_cast<int>(snapshot->probe % snapshot->columns),
                              static_cast<int>(snapshot->probe / snapshot->columns * 16), 0};
        vkCmdCopyImageToBuffer(command, resources->textures.at(names[i]).image->GetVkImage(), VK_IMAGE_LAYOUT_GENERAL,
                               snapshot->data[i]->GetVkBuffer(), 1, &copy);
      }
      const VkBufferCopy copy{0, 0, sizeof(SdfgiFieldStatus)};
      vkCmdCopyBuffer(command, resources->buffers.at("Status").buffer->GetVkBuffer(), snapshot->data[3]->GetVkBuffer(),
                      1, &copy);
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

SdfgiFieldStatus SdfgiProbeDebug::ReadStatus() const {
  if (!recorded)
    throw std::runtime_error("SDFGI probe diagnostic has not been recorded");
  Platform::WaitForFrameSubmissions("SDFGI explicit probe diagnostic readback");
  SdfgiFieldStatus status;
  data[3]->Download(status);
  return status;
}

void SdfgiProbeDebug::StoreToPng(const std::filesystem::path& path) const {
  if (ReadStatus().failure_flags)
    throw std::runtime_error("SDFGI probe diagnostic contains a failed field");
  std::vector<uint32_t> atlas;
  std::vector<glm::ivec4> average;
  std::vector<int16_t> history;
  data[0]->DownloadVector(atlas, columns * 8 * 136 * 2 * cascade_count);
  data[1]->DownloadVector(average, columns * 272);
  data[2]->DownloadVector(history, 16 * history_size * 4);
  std::vector<uint8_t> pixels(2560 * 1440 * 4, 255);
  const auto tonemap = [](const glm::vec3 color) {
    return glm::pow(glm::max(color, glm::vec3(0)) / (1.0f + glm::max(color, glm::vec3(0))), glm::vec3(1.0f / 2.2f));
  };
  const auto signed_color = [](const glm::vec3 value) {
    return 0.5f + 0.5f * value / (1.0f + glm::abs(value));
  };
  for (int y = 0; y < 1440; ++y)
    for (int x = 0; x < 2560; ++x) {
      glm::vec3 color(6.0f / 255.0f);
      for (uint32_t kind = 0; kind < 2; ++kind) {
        const int u = x - 64 - kind * 1280, v = y - 64;
        if (u >= 0 && u < 1156 && v >= 0 && v < static_cast<int>(cascade_count * 68)) {
          const uint32_t packed = atlas[(kind * cascade_count + v / 68) * columns * 8 * 136 +
                                        (v % 68 * 2) * columns * 8 + u * columns * 8 / 1156];
          color = tonemap(glm::vec3(packed & 511, (packed >> 9) & 511, (packed >> 18) & 511) *
                          std::ldexp(1.0f, static_cast<int>(packed >> 27) - 24));
        }
      }
      if (x >= 64 && x < 642 && y >= 760 && y < 1304)
        color = signed_color(glm::vec3(average[(y - 760) / 2 * columns + (x - 64) * columns / 578]) /
                             (history_size * 1024.0f));
      if (x >= 780 && x < 1292 && y >= 800 && y < static_cast<int>(800 + history_size * 16)) {
        const uint32_t index = ((y - 800) / 16 * 16 + (x - 780) / 32) * 4;
        color = signed_color(glm::vec3(history[index], history[index + 1], history[index + 2]) / 1024.0f);
      }
      if (x >= 1450 && x < 1926 && y >= 800 && y < 1276) {
        const uint32_t index =
            probe / columns * 16 * columns + (y - 800) * probe_axis / 476 * probe_axis + (x - 1450) * probe_axis / 476;
        color = tonemap(glm::vec3(average[index]) / (history_size * 1024.0f) * 0.88622f);
      }
      for (uint32_t channel = 0; channel < 3; ++channel)
        pixels[(y * 2560 + x) * 4 + channel] = static_cast<uint8_t>(glm::clamp(color[channel], 0.0f, 1.0f) * 255);
    }
  stbi_flip_vertically_on_write(false);
  if (!stbi_write_png(path.string().c_str(), 2560, 1440, 4, pixels.data(), 2560 * 4))
    throw std::runtime_error("Could not write SDFGI probe diagnostic PNG");
}

uint64_t SdfgiProbeDebug::AllocationBytes() const {
  uint64_t bytes = 0;
  for (const auto& buffer : data)
    bytes += buffer->GetVmaAllocationInfo().size;
  return bytes;
}

std::shared_ptr<SdfgiProbeFrame> SdfgiProbeFrame::Create(const SdfgiResources& resources,
                                                         const std::vector<SdfgiCascade>& cascades,
                                                         const SdfgiSkyInput& sky, const uint32_t scene_frame) {
  auto frame = std::make_shared<SdfgiProbeFrame>();
  frame->frame_slot = Platform::GetCurrentFrameIndex();
  frame->scene_frame = scene_frame;
  frame->sky = sky;
  frame->sky_set = resources.sets.at("Sky");
  SdfgiIntegratePushConstant params{};
  for (uint32_t axis = 0; axis < 3; ++axis)
    params.grid_size[axis] = resources.settings.GridSize()[axis];
  params.max_cascades = resources.settings.cascade_count;
  params.probe_axis_size = resources.settings.ProbeSize().x;
  params.history_index = resources.transport_pass % resources.settings.history_size;
  params.history_size = resources.settings.history_size;
  params.ray_count = resources.settings.ray_count;
  params.ray_bias = resources.settings.probe_bias;
  params.pad = resources.debug_seed;
  params.image_size[0] = params.probe_axis_size * params.probe_axis_size;
  params.image_size[1] = 17;
  params.y_mult = SdfgiYMultiplier(resources.settings.vertical_scale);
  if (resources.settings.read_sky_light) {
    params.sky_energy = sky.energy;
    if (sky.constant_color) {
      params.sky_flags = 1;
      for (uint32_t axis = 0; axis < 3; ++axis)
        params.sky_color_or_orientation[axis] = sky.color[axis];
    } else {
      VkDescriptorImageInfo image_info{};
      if (!sky.cubemap ||
          !TextureStorage::TryGetCubemapDescriptorImageInfo(sky.cubemap->GetTextureStorageIndex(), image_info))
        throw std::runtime_error("SDFGI scene sky radiance cubemap is not ready");
      if (!std::isfinite(sky.gamma) || sky.gamma <= 0)
        throw std::runtime_error("SDFGI scene sky gamma must be positive and finite");
      const auto& storage = sky.cubemap->PeekStorage();
      frame->sky_image = storage.image;
      frame->sky_view = storage.image_view;
      frame->sky_set = std::make_shared<DescriptorSet>(resources.layouts[static_cast<size_t>(SdfgiLayout::Sky)]);
      frame->sky_set->UpdateImageDescriptorBinding(0, image_info);
      VkDescriptorImageInfo sampler_info{};
      sampler_info.sampler = resources.mip_sampler->GetVkSampler();
      frame->sky_set->UpdateImageDescriptorBinding(1, sampler_info);
      const auto orientation = glm::angleAxis(-sky.rotation, glm::vec3(0, 1, 0));
      params.sky_flags = 2 | (orientation.w < 0 ? 0 : 4);
      for (uint32_t axis = 0; axis < 3; ++axis)
        params.sky_color_or_orientation[axis] = orientation[axis];
      params.sky_lod_inverse_gamma[0] = std::min(2u, sky.cubemap->GetMipLevels() - 1);
      params.sky_lod_inverse_gamma[1] = 1 / sky.gamma;
    }
  }
  for (uint32_t c = 0; c < cascades.size(); ++c) {
    params.cascade = c;
    for (uint32_t axis = 0; axis < 3; ++axis)
      params.world_offset[axis] = cascades[c].position[axis] / 8;
    frame->constants.push_back(params);
  }
  return frame;
}

void SdfgiProbeFrame::AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency) {
  RenderPassDescriptor process{"SdfgiProbeProcess", RenderPassQueue::Graphics, RenderPassScope::Frame};
  process.dependencies = {dependency};
  process.profiler_group = RenderPassProfilerGroup::FramePreparation;
  process.profiler_display_name = "SDFGI Probe Process";
  process.resources.push_back(
      {"Frame.SDFGI." + FrameName(frame_slot, "Cascades"), RenderResourceUsage::Read, RenderResourceState::General});
  process.resources.push_back({"Frame.SDFGI.Status", RenderResourceUsage::Read, RenderResourceState::General});
  if (sky_image) {
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.SDFGI.SceneSky";
    descriptor.type = RenderResourceType::Image;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    graph.AddResource(descriptor);
    registry.BindImage(descriptor.name, sky_image);
    process.resources.push_back({descriptor.name, RenderResourceUsage::Read,
                                 sky_image->GetLayout() == VK_IMAGE_LAYOUT_GENERAL ? RenderResourceState::General
                                                                                   : RenderResourceState::ShaderRead});
  }
  for (uint32_t c = 0; c < constants.size(); ++c) {
    for (const auto* name : {"Sdf", "Light", "Aniso0", "Aniso1"})
      process.resources.push_back(
          {"Frame.SDFGI." + CascadeName(c, name), RenderResourceUsage::Read, RenderResourceState::General});
    for (const auto* name : {"History", "Average"})
      process.resources.push_back(
          {"Frame.SDFGI." + CascadeName(c, name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
  }
  graph.AddPass(process, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    if (!resources->lighting_recorded || !resources->light_failure.empty())
      return;
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      ApplyGraphResourceBarriers(command, context);
      const auto timing =
          Platform::BeginGpuTimestampScope(command, {"SdfgiProbeProcess", "SDFGI Probe Process", "SDFGI"});
      const auto& pipeline = resources->pipelines.at("IntegratePROCESS");
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 1, frame->sky_set->GetVkDescriptorSet());
      for (const auto& params : frame->constants) {
        pipeline->BindDescriptorSet(
            command, 0,
            resources->sets.at(FrameName(frame->frame_slot, CascadeName(params.cascade, "Integrate")))
                ->GetVkDescriptorSet());
        pipeline->PushConstant(command, 0, params);
        pipeline->Dispatch(command, (params.image_size[0] + 7) / 8, (params.image_size[1] + 7) / 8);
      }
      Platform::EndGpuTimestampScope(command, timing);
    });
  });
  RenderPassDescriptor store{"SdfgiProbeStore", RenderPassQueue::Graphics, RenderPassScope::Frame};
  store.dependencies = {process.name};
  store.profiler_group = RenderPassProfilerGroup::FramePreparation;
  store.profiler_display_name = "SDFGI Probe Store";
  store.resources = {{"Frame.SDFGI.Atlas", RenderResourceUsage::Write, RenderResourceState::General},
                     {"Frame.SDFGI.Status", RenderResourceUsage::ReadWrite, RenderResourceState::General}};
  for (uint32_t c = 0; c < constants.size(); ++c)
    store.resources.push_back(
        {"Frame.SDFGI." + CascadeName(c, "Average"), RenderResourceUsage::Read, RenderResourceState::General});
  graph.AddPass(store, [frame = shared_from_this(), resources](const RenderGraphExecutionContext& context) {
    if (!resources->lighting_recorded || !resources->light_failure.empty()) {
      resources->transport_recorded = false;
      return;
    }
    const uint32_t generation = resources->transport_pass + 1;
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      ApplyGraphResourceBarriers(command, context);
      const auto timing = Platform::BeginGpuTimestampScope(command, {"SdfgiProbeStore", "SDFGI Probe Store", "SDFGI"});
      const auto& pipeline = resources->pipelines.at("IntegrateSTORE");
      pipeline->Bind(command);
      pipeline->BindDescriptorSet(command, 1, resources->sets.at("Sky")->GetVkDescriptorSet());
      for (auto params : frame->constants) {
        params.history_index = generation % params.history_size;
        params.image_size[0] *= 6;
        params.image_size[1] *= 6;
        pipeline->BindDescriptorSet(
            command, 0,
            resources->sets.at(FrameName(frame->frame_slot, CascadeName(params.cascade, "Integrate")))
                ->GetVkDescriptorSet());
        pipeline->PushConstant(command, 0, params);
        pipeline->Dispatch(command, (params.image_size[0] + 7) / 8, (params.image_size[1] + 7) / 8);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
      vkCmdUpdateBuffer(command, resources->buffers.at("Status").buffer->GetVkBuffer(),
                        offsetof(SdfgiFieldStatus, generation), 4, &generation);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
      Platform::EndGpuTimestampScope(command, timing);
    });
    resources->transport_pass = generation;
    resources->transport_recorded = true;
    resources->transport_failure.clear();
  });
}
