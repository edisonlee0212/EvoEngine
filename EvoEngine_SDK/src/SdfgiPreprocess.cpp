// Pass sequence adapted from Godot gi.cpp::SDFGI::render_region,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiPreprocess.hpp"
#include "SdfgiScene.hpp"

#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <stb_image_write.h>
#include <cstring>
#include <glm/gtc/round.hpp>

using namespace evo_engine;

std::vector<uint32_t> evo_engine::SdfgiJumpFloodSteps(const glm::ivec3 grid) {
  std::vector<uint32_t> steps;
  for (uint32_t step = glm::ceilPowerOfTwo(std::max({grid.x, grid.y, grid.z}) / 2) / 2; step; step /= 2)
    steps.push_back(step);
  return steps;
}

glm::uvec3 evo_engine::SdfgiJumpFloodGroups(const glm::uvec3 size, const uint32_t step) {
  // Each group covers one residue modulo step, with eight cells per axis.
  return ((size + 8 * step - 1u) / (8 * step)) * step;
}

namespace {
std::string CascadeName(const uint32_t cascade, const std::string& name) {
  return "Cascade" + std::to_string(cascade) + "." + name;
}
constexpr auto kComputeAccess = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
void ClearLighting(const VkCommandBuffer command, const SdfgiResources& resources, const uint32_t cascade) {
  const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  for (const auto name : {"Light", "Aniso0", "Aniso1"})
    Platform::ClearColorImage(command, *resources.textures.at(CascadeName(cascade, name)).image, VkClearColorValue{}, 1,
                              &range);
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT,
                        kComputeAccess | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT);
}
}  // namespace

SdfgiPreprocessDebug::SdfgiPreprocessDebug(const uint32_t cascade_index, const uint32_t slice_index,
                                           const glm::ivec3 grid)
    : cascade(cascade_index), slice(slice_index), slices{grid} {
  if (slice >= slices.SliceCount())
    throw std::invalid_argument("SDFGI preprocessing slice exceeds the smallest grid dimension");
  for (size_t i = 0; i < planes.size(); ++i) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = slices.Total() * (i == 0 ? 1 : 4);
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo allocation{};
    allocation.usage = VMA_MEMORY_USAGE_AUTO;
    allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    planes[i] = std::make_shared<Buffer>(info, allocation);
  }
}

void SdfgiPreprocessDebug::AddPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                   const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency) {
  RenderPassDescriptor pass{"SdfgiPreprocessDebug", RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.dependencies = {dependency};
  for (const auto& name : {CascadeName(cascade, "Sdf"), std::string("Occlusion")})
    pass.resources.push_back({"Frame.SDFGI." + name, RenderResourceUsage::Read, RenderResourceState::General});
  for (size_t i = 0; i < planes.size(); ++i) {
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.SDFGI.PreprocessDebug" + std::to_string(i);
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
      for (size_t i = 0; i < 2; ++i) {
        std::vector<VkBufferImageCopy> copies;
        const uint32_t halves = i == 0 ? 1 : 2;
        for (uint32_t axis = 0; axis < 3; ++axis)
          for (uint32_t half = 0; half < halves; ++half) {
            VkBufferImageCopy copy{};
            copy.bufferOffset =
                (snapshot->slices.Offset(axis) * halves + half * snapshot->slices.Count(axis)) * (i == 0 ? 1 : 2);
            copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            copy.imageOffset = {static_cast<int32_t>(half * snapshot->slices.grid.x), 0,
                                i == 0 ? 0 : static_cast<int32_t>(snapshot->cascade * snapshot->slices.grid.z)};
            copy.imageExtent = {uint32_t(snapshot->slices.grid.x), uint32_t(snapshot->slices.grid.y),
                                uint32_t(snapshot->slices.grid.z)};
            if (axis == 0) {
              copy.imageOffset.x += snapshot->slice;
              copy.imageExtent.width = 1;
            } else if (axis == 1) {
              copy.imageOffset.y = snapshot->slice;
              copy.imageExtent.height = 1;
            } else {
              copy.imageOffset.z += snapshot->slice;
              copy.imageExtent.depth = 1;
            }
            copies.push_back(copy);
          }
        const auto& name = i == 0 ? CascadeName(snapshot->cascade, "Sdf") : std::string("Occlusion");
        vkCmdCopyImageToBuffer(command, resources->textures.at(name).image->GetVkImage(), VK_IMAGE_LAYOUT_GENERAL,
                               snapshot->planes[i]->GetVkBuffer(), copies.size(), copies.data());
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

void SdfgiPreprocessDebug::StoreToPng(const std::filesystem::path& path) const {
  if (!recorded)
    throw std::runtime_error("SDFGI preprocessing diagnostic has not been recorded");
  Platform::WaitForFrameSubmissions("SDFGI explicit SDF/occlusion diagnostic readback");
  std::vector<uint8_t> sdf;
  std::vector<uint16_t> occlusion;
  planes[0]->DownloadVector(sdf, slices.Total());
  planes[1]->DownloadVector(occlusion, 2 * slices.Total());
  std::vector<uint8_t> pixels(2560 * 1440 * 4, 255);
  for (int y = 0; y < 1440; ++y)
    for (int x = 0; x < 2560; ++x) {
      const uint32_t axis = y / 480;
      const uint32_t column = x / 284;
      const int u = x % 284 - 14, v = y % 480 - 112;
      uint8_t value = 6;
      if (column < 9 && u >= 0 && v >= 0 && u < 256 && v < 256) {
        const uint32_t offset = slices.Pixel(axis, u, v, 256);
        if (column == 0)
          value = static_cast<uint8_t>(std::min(255u, sdf[slices.Offset(axis) + offset] * 8u));
        else {
          const uint32_t channel = column - 1;
          const auto packed = occlusion[slices.Offset(axis) * 2 + (channel / 4) * slices.Count(axis) + offset];
          value = ((packed >> (12 - 4 * (channel % 4))) & 15) * 17;
        }
      }
      const auto index = (y * 2560 + x) * 4;
      pixels[index] = pixels[index + 1] = pixels[index + 2] = value;
    }
  stbi_flip_vertically_on_write(false);
  if (!stbi_write_png(path.string().c_str(), 2560, 1440, 4, pixels.data(), 2560 * 4))
    throw std::runtime_error("Could not write SDFGI preprocessing diagnostic PNG");
}

uint64_t SdfgiPreprocessDebug::AllocationBytes() const {
  return planes[0]->GetVmaAllocationInfo().size + planes[1]->GetVmaAllocationInfo().size;
}

SdfgiPreprocessReadback::SdfgiPreprocessReadback(const uint32_t count) : cascade_count(count) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = count * sizeof(SdfgiDispatchData) + sizeof(SdfgiFieldStatus);
  info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO;
  allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  buffer = std::make_shared<Buffer>(info, allocation);
}

void SdfgiPreprocessReadback::ReadAfterFrameFence(SdfgiResources& resources) {
  if (consumed || !recorded_cascades || scene_frame < resources.minimum_readback_frame)
    return;
  std::vector<SdfgiDispatchData> records;
  static_assert(sizeof(SdfgiDispatchData) == sizeof(SdfgiFieldStatus));
  buffer->DownloadVector(records, cascade_count + 1);
  resources.solid_cell_dispatch.resize(cascade_count);
  for (uint32_t cascade = 0; cascade < cascade_count; ++cascade)
    if (recorded_cascades & (1u << cascade))
      resources.solid_cell_dispatch[cascade] = records[cascade];
  std::memcpy(&resources.preprocess_status, &records.back(), sizeof(SdfgiFieldStatus));
  resources.preprocess_status_available = true;
  consumed = true;
}

void evo_engine::RecordSdfgiScroll(const VkCommandBuffer command, const SdfgiResources& resources,
                                   const uint32_t cascade, const glm::ivec3 cascade_position, const glm::ivec3 scroll,
                                   const uint32_t frame_slot) {
  SdfgiPreprocessPushConstant params{};
  params.grid_size = resources.settings.GridSize().x;
  params.grid_size_y = resources.settings.voxel_count_y;
  params.cascade = cascade;
  for (uint32_t axis = 0; axis < 3; ++axis)
    params.scroll[axis] = scroll[axis];
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT,
                        kComputeAccess | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT);
  const auto& voxels = resources.pipelines.at("Scroll");
  voxels->Bind(command);
  voxels->BindDescriptorSet(command, 0, resources.sets.at(CascadeName(cascade, "Scroll"))->GetVkDescriptorSet());
  voxels->PushConstant(command, 0, params);
  vkCmdDispatchIndirect(command, resources.buffers.at(CascadeName(cascade, "Indirect")).buffer->GetVkBuffer(), 0);
  const auto& occlusion = resources.pipelines.at("ScrollOcclusion");
  occlusion->Bind(command);
  occlusion->BindDescriptorSet(command, 0, resources.sets.at("ScrollOcclusion")->GetVkDescriptorSet());
  occlusion->PushConstant(command, 0, params);
  const glm::ivec3 groups = (resources.settings.GridSize() - glm::abs(scroll) + 3) / 4;
  occlusion->Dispatch(command, groups.x, groups.y, groups.z);

  SdfgiIntegratePushConstant probes{};
  for (uint32_t axis = 0; axis < 3; ++axis) {
    probes.grid_size[axis] = resources.settings.GridSize()[axis];
    probes.scroll[axis] = scroll[axis] / static_cast<int>(resources.settings.probe_spacing_cells);
    probes.world_offset[axis] = cascade_position[axis] / static_cast<int>(resources.settings.probe_spacing_cells);
  }
  probes.max_cascades = resources.settings.cascade_count;
  probes.cascade = cascade;
  probes.probe_axis_size = resources.settings.ProbeSize().x;
  probes.history_size = resources.settings.history_size;
  probes.image_size[0] = resources.textures.at("Ambient").requirement.extent.width;
  probes.image_size[1] = resources.textures.at("Ambient").requirement.extent.height;
  probes.y_mult = SdfgiYMultiplier(resources.settings.vertical_scale);
  const auto dispatch = [&](const char* name) {
    resources.OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, kComputeAccess);
    const auto& pipeline = resources.pipelines.at(name);
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(
        command, 0,
        resources.sets.at("Frame" + std::to_string(frame_slot) + "." + CascadeName(cascade, "Integrate"))
            ->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(command, 1, resources.sets.at("Sky")->GetVkDescriptorSet());
    pipeline->PushConstant(command, 0, probes);
    pipeline->Dispatch(command, (probes.image_size[0] + 7) / 8, (probes.image_size[1] + 7) / 8);
  };
  dispatch("IntegrateSCROLL");
  dispatch("IntegrateSCROLL_STORE");
  if (resources.settings.bounce_feedback > 0) {
    probes.image_size[0] *= 6;
    probes.image_size[1] *= 6;
    dispatch("IntegrateSTORE");
  }
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, kComputeAccess);
}

void evo_engine::RecordSdfgiPreprocess(const VkCommandBuffer command, const SdfgiResources& resources,
                                       const uint32_t cascade, const glm::ivec3 cascade_position,
                                       const glm::ivec3 scroll) {
  SdfgiPreprocessPushConstant params{};
  const auto grid = resources.settings.GridSize();
  params.grid_size = grid.x / 2;
  params.grid_size_y = grid.y / 2;
  params.cascade = cascade;
  for (uint32_t axis = 0; axis < 3; ++axis)
    params.scroll[axis] = scroll[axis];
  const auto barrier = [&]() {
    resources.OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, kComputeAccess);
  };
  const auto dispatch = [&](const char* pipeline_name, const std::string& set_name, const glm::uvec3 groups) {
    const auto& pipeline = resources.pipelines.at(pipeline_name);
    pipeline->Bind(command);
    pipeline->BindDescriptorSet(command, 0, resources.sets.at(set_name)->GetVkDescriptorSet());
    pipeline->PushConstant(command, 0, params);
    pipeline->Dispatch(command, groups.x, groups.y, groups.z);
  };
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
  vkCmdFillBuffer(command, resources.buffers.at(CascadeName(cascade, "Dispatch")).buffer->GetVkBuffer(), 0, 16, 0);
  barrier();
  dispatch("InitializeHalf", "InitializeHalf", glm::uvec3(grid / 8));
  barrier();
  params.half_size = 1;
  uint32_t source = 0;
  for (const uint32_t step : SdfgiJumpFloodSteps(grid)) {
    params.step_size = step;
    const bool optimized = step <= 8;
    dispatch(optimized ? "JumpFloodOptimized" : "JumpFlood", "JumpFloodHalf" + std::to_string(source),
             optimized ? SdfgiJumpFloodGroups(glm::uvec3(grid / 2), step) : glm::uvec3(grid / 8));
    barrier();
    source = 1 - source;
  }
  params.grid_size = resources.settings.GridSize().x;
  params.grid_size_y = grid.y;
  dispatch("Upscale", "Upscale", glm::uvec3(grid / 4));
  barrier();
  params.half_size = 0;
  params.step_size = 1;
  dispatch("JumpFloodOptimized", "JumpFlood0", glm::uvec3(grid / 8));
  barrier();
  const int spacing = resources.settings.probe_spacing_cells;
  const glm::ivec3 parity = (cascade_position / spacing) & glm::ivec3(1);
  for (uint32_t i = 0; i < 8; ++i) {
    const glm::ivec3 offset = glm::ivec3(i & 1, (i >> 1) & 1, (i >> 2) & 1) ^ parity;
    for (int axis = 0; axis < 3; ++axis)
      params.probe_offset[axis] = offset[axis];
    params.occlusion_index = i;
    dispatch("Occlusion", "Occlusion", glm::uvec3(grid / (2 * spacing) + 1 - offset));
  }
  barrier();
  dispatch("Store", CascadeName(cascade, "Store"), glm::uvec3(grid / 4));
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                        VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
  const VkBufferCopy copy{0, 0, sizeof(SdfgiDispatchData)};
  vkCmdCopyBuffer(command, resources.buffers.at(CascadeName(cascade, "Dispatch")).buffer->GetVkBuffer(),
                  resources.buffers.at(CascadeName(cascade, "Indirect")).buffer->GetVkBuffer(), 1, &copy);
  const VkBufferCopy seed_copy{0, 0, sizeof(SdfgiSolidCell) * resources.settings.SolidCellCapacity()};
  vkCmdCopyBuffer(command, resources.buffers.at(CascadeName(cascade, "SolidCells")).buffer->GetVkBuffer(),
                  resources.buffers.at(CascadeName(cascade, "UnlitCells")).buffer->GetVkBuffer(), 1, &seed_copy);
  ClearLighting(command, resources, cascade);
}

void evo_engine::RecordSdfgiPayloadRefresh(const VkCommandBuffer command, const SdfgiResources& resources,
                                           const uint32_t cascade) {
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT,
                        kComputeAccess | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT);
  const auto& pipeline = resources.pipelines.at("PayloadRefresh");
  pipeline->Bind(command);
  pipeline->BindDescriptorSet(command, 0,
                              resources.sets.at(CascadeName(cascade, "PayloadRefresh"))->GetVkDescriptorSet());
  pipeline->DispatchIndirect(command, *resources.buffers.at(CascadeName(cascade, "Indirect")).buffer);
  resources.OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                        VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
  const VkBufferCopy copy{0, 0, sizeof(SdfgiSolidCell) * resources.settings.SolidCellCapacity()};
  vkCmdCopyBuffer(command, resources.buffers.at(CascadeName(cascade, "UnlitCells")).buffer->GetVkBuffer(),
                  resources.buffers.at(CascadeName(cascade, "SolidCells")).buffer->GetVkBuffer(), 1, &copy);
  ClearLighting(command, resources, cascade);
}

std::string evo_engine::AddSdfgiPreprocessPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                               const std::shared_ptr<SdfgiResources>& resources,
                                               const std::shared_ptr<SdfgiPreprocessReadback>& readback,
                                               const uint32_t cascade, const glm::ivec3 cascade_position,
                                               const std::string& dependency, const glm::ivec3 scroll,
                                               const bool payload_only) {
  RenderResourceDescriptor diagnostic;
  diagnostic.name = "Frame.SDFGI.PreprocessReadback";
  diagnostic.type = RenderResourceType::Buffer;
  diagnostic.lifetime = RenderResourceLifetime::Persistent;
  diagnostic.byte_size = readback->buffer->GetSize();
  graph.AddResource(diagnostic);
  registry.BindBuffer(diagnostic.name, readback->buffer);
  RenderPassDescriptor pass{
      std::string(payload_only ? "SdfgiPayloadCascade" : "SdfgiPreprocessCascade") + std::to_string(cascade),
      RenderPassQueue::Graphics, RenderPassScope::Frame};
  pass.dependencies = {dependency};
  pass.profiler_group = RenderPassProfilerGroup::FramePreparation;
  pass.profiler_display_name = payload_only ? "SDFGI Payload Refresh" : "SDFGI Preprocess";
  for (const auto name : {"Albedo", "Emission", "EmissionAniso", "Facing"})
    pass.resources.push_back(
        {"Frame.SDFGI." + std::string(name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
  for (const auto name : {"JumpFlood0", "JumpFlood1", "JumpFloodHalf0", "JumpFloodHalf1", "Occlusion", "Status"})
    if (!payload_only || std::string(name) == "Status")
      pass.resources.push_back(
          {"Frame.SDFGI." + std::string(name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
  for (uint32_t i = 0; !payload_only && i < 8; ++i)
    pass.resources.push_back({"Frame.SDFGI.OcclusionScratch" + std::to_string(i), RenderResourceUsage::ReadWrite,
                              RenderResourceState::General});
  for (const auto name : {"Sdf", "SolidCells", "UnlitCells", "Dispatch", "Indirect", "Light", "Aniso0", "Aniso1"})
    if (!payload_only || std::string(name) != "Sdf")
      pass.resources.push_back(
          {"Frame.SDFGI." + CascadeName(cascade, name), RenderResourceUsage::ReadWrite, RenderResourceState::General});
  pass.resources.push_back(
      {diagnostic.name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  const auto frame_slot = Platform::GetCurrentFrameIndex();
  if (scroll != glm::ivec3(0)) {
    pass.resources.push_back({"Frame.SDFGI.Frame" + std::to_string(frame_slot) + ".Cascades", RenderResourceUsage::Read,
                              RenderResourceState::General});
    for (const auto& name : {std::string("HistoryScroll"), std::string("AverageScroll"), std::string("Atlas"),
                             CascadeName(cascade, "History"), CascadeName(cascade, "Average")})
      pass.resources.push_back({"Frame.SDFGI." + name, RenderResourceUsage::ReadWrite, RenderResourceState::General});
    if (cascade + 1 < resources->settings.cascade_count)
      pass.resources.push_back({"Frame.SDFGI." + CascadeName(cascade + 1, "Average"), RenderResourceUsage::Read,
                                RenderResourceState::General});
  }
  graph.AddPass(pass, [resources, readback, cascade, cascade_position, scroll, frame_slot,
                       payload_only](const RenderGraphExecutionContext& context) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command) {
      const RenderPassGpuTimestampScope timing(command, context);
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, kComputeAccess);
      ApplyGraphResourceBarriers(command, context);
      if (payload_only)
        RecordSdfgiPayloadRefresh(command, *resources, cascade);
      else {
        if (scroll != glm::ivec3(0))
          RecordSdfgiScroll(command, *resources, cascade, cascade_position, scroll, frame_slot);
        RecordSdfgiPreprocess(command, *resources, cascade, cascade_position, scroll);
      }
      resources->OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                             VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
      VkBufferCopy copy{0, cascade * sizeof(SdfgiDispatchData), sizeof(SdfgiDispatchData)};
      vkCmdCopyBuffer(command, resources->buffers.at(CascadeName(cascade, "Dispatch")).buffer->GetVkBuffer(),
                      readback->buffer->GetVkBuffer(), 1, &copy);
      copy.dstOffset = readback->cascade_count * sizeof(SdfgiDispatchData);
      vkCmdCopyBuffer(command, resources->buffers.at("Status").buffer->GetVkBuffer(), readback->buffer->GetVkBuffer(),
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
    readback->recorded_cascades |= 1u << cascade;
    resources->preprocessed_cascades |= 1u << cascade;
    if (payload_only)
      ++resources->payload_update_count;
    else
      ++resources->geometry_update_count;
    if (resources->preprocessed_cascades == (1u << resources->settings.cascade_count) - 1)
      resources->preprocess_failure.clear();
  });
  return pass.name;
}
