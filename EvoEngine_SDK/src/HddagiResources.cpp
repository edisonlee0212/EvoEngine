// Godot HDDAGI::create layouts, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. EvoEngine owns validation and allocation lifetime.
#include "HddagiResources.hpp"
#include "HddagiLight.hpp"
#include "HddagiProbe.hpp"
#include "HddagiVoxelizer.hpp"

#include <stb_image_write.h>
#include <array>
#include <stdexcept>
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

void HddagiResources::CaptureToPng(const std::filesystem::path& path, const std::string& image_name,
                                   const uint32_t layer, const uint32_t z_slice) const {
  const bool radiance = image_name == "Light" || image_name == "StaticLight" || image_name == "Diffuse" ||
                        image_name == "FilteredDiffuse" || image_name == "Specular" || image_name == "History";
  const bool occlusion = image_name == "Occlusion0" || image_name == "Occlusion1";
  if (!radiance && !occlusion && image_name != "ProcessFrame" && image_name != "HistorySum" &&
      image_name != "Proximity")
    throw std::invalid_argument("Unsupported HDDAGI diagnostic image");
  const auto& texture = images.at(image_name);
  const auto& requirement = texture.requirement;
  if (!transport_recorded || layer >= requirement.layers || z_slice >= requirement.extent.depth)
    throw std::invalid_argument("HDDAGI diagnostic generation or slice unavailable");
  Platform::WaitForFrameSubmissions("HDDAGI explicit transport diagnostic readback");
  const uint32_t count = requirement.extent.width * requirement.extent.height;
  const uint32_t stride = occlusion ? 2 : image_name == "Proximity" ? 1 : 4;
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = count * stride;
  info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO;
  allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  allocation.requiredFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  Buffer readback(info, allocation);
  VkBufferImageCopy copy{};
  copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, layer, 1};
  copy.imageOffset.z = z_slice;
  copy.imageExtent = {requirement.extent.width, requirement.extent.height, 1};
  readback.CopyFromImage(*texture.image, copy);
  std::vector<uint8_t> raw;
  readback.DownloadVector(raw, count * stride);
  std::vector<uint8_t> pixels(count * 4, 255);
  for (uint32_t i = 0; i < count; ++i) {
    uint32_t packed = 0;
    for (uint32_t byte = 0; byte < stride; ++byte)
      packed |= uint32_t(raw[i * stride + byte]) << (byte * 8);
    glm::vec3 color;
    if (radiance) {
      color =
          glm::vec3(packed & 511, (packed >> 9) & 511, (packed >> 18) & 511) * std::ldexp(1.0f, int(packed >> 27) - 24);
      color = glm::pow(color / (1.0f + color), glm::vec3(1.0f / 2.2f));
    } else if (occlusion) {
      color = glm::vec3((packed >> 12) & 15, (packed >> 8) & 15, (packed >> 4) & 15) / 15.0f;
      pixels[i * 4 + 3] = (packed & 15) * 17;
    } else {
      const float value = image_name == "HistorySum"     ? packed / float(16384 * settings.history_size)
                          : image_name == "ProcessFrame" ? (packed & 0xfffffff) / float(settings.history_size)
                                                         : packed / 255.0f;
      color = glm::vec3(value);
    }
    for (uint32_t channel = 0; channel < 3; ++channel)
      pixels[i * 4 + channel] = uint8_t(glm::clamp(color[channel], 0.0f, 1.0f) * 255);
  }
  stbi_flip_vertically_on_write(false);
  if (!stbi_write_png(path.string().c_str(), requirement.extent.width, requirement.extent.height, 4, pixels.data(),
                      requirement.extent.width * 4))
    throw std::runtime_error("Could not write HDDAGI diagnostic PNG");
}

namespace {
constexpr VkImageUsageFlags kUsage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                     VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
constexpr uint64_t kTemporalLimit = uint64_t{4} << 30;

uint32_t LightCellCapacity(const GiProbeSettings& p) {
  return (p.probe_count_x - 1) * (p.probe_count_y - 1) * (p.probe_count_x - 1) * 256;
}

VkBufferCreateInfo BufferInfo(const uint64_t bytes) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = bytes;
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT |
               VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  return info;
}

VkImageCreateInfo ImageInfo(const HddagiImageRequirement& r) {
  VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
  info.flags = r.storage_format == r.sampled_format
                   ? 0
                   : VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT | VK_IMAGE_CREATE_EXTENDED_USAGE_BIT;
  info.imageType = r.type;
  if (r.cube)
    info.flags |= VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
  info.format = r.storage_format;
  info.extent = r.extent;
  info.mipLevels = 1;
  info.arrayLayers = r.layers;
  info.samples = VK_SAMPLE_COUNT_1_BIT;
  info.tiling = VK_IMAGE_TILING_OPTIMAL;
  info.usage = kUsage;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  return info;
}
}  // namespace

uint64_t HddagiResources::AllocationBytes() const {
  uint64_t bytes = allocation_bytes;
  for (const auto& [name, buffer] : buffers)
    bytes += buffer->GetVmaAllocationInfo().size;
  for (const auto& frame : voxel_frames)
    if (frame)
      bytes += frame->AllocationBytes();
  for (const auto& frame : light_frames)
    if (frame)
      bytes += frame->AllocationBytes();
  for (const auto& frame : probe_frames)
    if (frame)
      bytes += frame->AllocationBytes();
  return bytes;
}

void HddagiResources::Import(RenderGraph& graph, RenderGraphResourceRegistry& registry) const {
  for (const auto& [name, texture] : images) {
    const auto& r = texture.requirement;
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.HDDAGI." + name;
    descriptor.type = RenderResourceType::Image;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.dimensions = {
        RenderResourceSizeMode::Absolute, r.extent.width, r.extent.height, r.extent.depth, r.layers, 1};
    descriptor.format_name = std::to_string(r.storage_format);
    descriptor.byte_size = texture.image->GetVmaAllocationInfo().size;
    graph.AddResource(descriptor);
    registry.BindImage(descriptor.name, texture.image);
  }
  for (const auto& [name, buffer] : buffers) {
    RenderResourceDescriptor descriptor;
    descriptor.name = "Frame.HDDAGI." + name;
    descriptor.type = RenderResourceType::Buffer;
    descriptor.lifetime = RenderResourceLifetime::Persistent;
    descriptor.byte_size = buffer->GetSize();
    graph.AddResource(descriptor);
    registry.BindBuffer(descriptor.name, buffer);
  }
}

RenderPassDescriptor HddagiResources::ClearDescriptor() const {
  RenderPassDescriptor result{"HddagiInitialize", RenderPassQueue::Graphics, RenderPassScope::Frame};
  result.profiler_group = RenderPassProfilerGroup::FramePreparation;
  result.profiler_display_name = "HDDAGI Initialize";
  for (const auto& [name, texture] : images)
    result.resources.push_back(
        {"Frame.HDDAGI." + name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  for (const auto& [name, buffer] : buffers)
    result.resources.push_back(
        {"Frame.HDDAGI." + name, RenderResourceUsage::Write, RenderResourceState::TransferDestinationGeneral});
  return result;
}

void HddagiResources::OrderAccess(const VkCommandBuffer command, const VkPipelineStageFlags2 stages,
                                  const VkAccessFlags2 access) const {
  VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
  barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
  barrier.srcAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
  barrier.dstStageMask = stages;
  barrier.dstAccessMask = access;
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.memoryBarrierCount = 1;
  dependency.pMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(command, &dependency);
}

void HddagiResources::Clear(const VkCommandBuffer command, const RenderGraphExecutionContext& context) {
  OrderAccess(command, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
  ApplyGraphResourceBarriers(command, context);
  const VkClearColorValue zero{};
  for (const auto& [name, texture] : images) {
    const VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, texture.requirement.layers};
    Platform::ClearColorImage(command, *texture.image, zero, 1, &range);
  }
  for (const auto& [name, buffer] : buffers)
    buffer->Fill(command, 0, buffer->GetSize(), 0);
  OrderAccess(command, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
              VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
  initialization_recorded = true;
}

uint64_t evo_engine::HddagiLogicalTemporalBytes(const GiProbeSettings& p, const HddagiSettings& s) {
  if (!s.Validate(p).empty())
    return UINT64_MAX;
  return uint64_t{p.probe_count_x} * p.probe_count_y * p.probe_count_x * p.cascade_count * 25 *
         (10 * s.history_size + 12);
}

std::vector<HddagiImageRequirement> evo_engine::GetHddagiImageRequirements(const GiProbeSettings& p,
                                                                           const HddagiSettings& s) {
  if (!s.Validate(p).empty())
    return {};
  const uint32_t x = (p.probe_count_x - 1) * 8, y = (p.probe_count_y - 1) * 8, c = p.cascade_count;
  std::vector<HddagiImageRequirement> result;
  const auto volume = [&](const char* name, VkFormat format, VkExtent3D extent, VkFormat sampled = VK_FORMAT_UNDEFINED,
                          bool atomic = false) {
    result.push_back({name, format, sampled == VK_FORMAT_UNDEFINED ? format : sampled, VK_IMAGE_TYPE_3D, extent, 1,
                      false, sampled != VK_FORMAT_UNDEFINED, atomic});
  };
  volume("VoxelBits", VK_FORMAT_R32G32_UINT, {x / 4, c * y / 4, x / 4});
  volume("Regions", VK_FORMAT_R8_UINT, {x / 8, c * y / 8, x / 8});
  volume("Versions", VK_FORMAT_R16_UINT, {x / 8, c * y / 8, x / 8});
  volume("Light", VK_FORMAT_R32_UINT, {x, c * y, x}, VK_FORMAT_E5B9G9R9_UFLOAT_PACK32);
  volume("StaticLight", VK_FORMAT_R32_UINT, {x, c * y, x});
  volume("Disocclusion", VK_FORMAT_R8_UINT, {x, c * y, x});
  volume("LightNeighbors", VK_FORMAT_R32_UINT, {x, c * y, x});
  volume("Albedo", VK_FORMAT_R16_UINT, {x / 2, y / 2, x * 3});
  volume("NormalBits", VK_FORMAT_R32_UINT, {x, y, x}, VK_FORMAT_UNDEFINED, true);
  volume("Emission", VK_FORMAT_R32_UINT, {x / 2, y / 2, x / 2});
  volume("EmissionAniso", VK_FORMAT_R32_UINT, {x / 2, y / 2, x / 2});
  for (const auto name : {"Occlusion0", "Occlusion1"})
    volume(name, VK_FORMAT_R16_UINT, {x + 2, c * (y + 2), x + 2}, VK_FORMAT_R4G4B4A4_UNORM_PACK16);
  const auto probes = [&](const char* name, VkFormat format, uint32_t tile, uint32_t layers, bool temporal = false,
                          VkFormat sampled = VK_FORMAT_UNDEFINED, uint32_t width = 1) {
    result.push_back({name,
                      format,
                      sampled == VK_FORMAT_UNDEFINED ? format : sampled,
                      VK_IMAGE_TYPE_2D,
                      {width * tile * p.probe_count_x, tile * p.probe_count_y * p.probe_count_x, 1},
                      layers,
                      temporal,
                      sampled != VK_FORMAT_UNDEFINED,
                      false});
  };
  for (const auto name : {"Diffuse", "Specular", "FilteredDiffuse"})
    probes(name, VK_FORMAT_R32_UINT, 7, c, false, VK_FORMAT_E5B9G9R9_UFLOAT_PACK32);
  probes("HitCache", VK_FORMAT_R32_UINT, 5, c * s.history_size, true);
  probes("HitVersions", VK_FORMAT_R16_UINT, 5, c * s.history_size, true);
  probes("History", VK_FORMAT_R32_UINT, 5, c * s.history_size, true);
  probes("HistorySum", VK_FORMAT_R32_UINT, 5, c, true, VK_FORMAT_UNDEFINED, 3);
  probes("Ambient", VK_FORMAT_R16G16B16A16_SFLOAT, 1, c);
  probes("Neighbors", VK_FORMAT_R32_UINT, 1, c);
  probes("ProcessFrame", VK_FORMAT_R32_UINT, 1, c);
  probes("Proximity", VK_FORMAT_R8_UNORM, 1, c);
  probes("CameraVisibility", VK_FORMAT_R8_UNORM, 1, c);
  result.push_back({"BlackSky",
                    VK_FORMAT_R16G16B16A16_SFLOAT,
                    VK_FORMAT_R16G16B16A16_SFLOAT,
                    VK_IMAGE_TYPE_2D,
                    {1, 1, 1},
                    6,
                    false,
                    true,
                    false,
                    true});
  return result;
}

HddagiCapabilityReport evo_engine::QueryHddagiCapabilities(const GiProbeSettings& p, const HddagiSettings& s) {
  HddagiCapabilityReport report;
  report.failure = s.Validate(p);
  if (!report.failure.empty())
    return report;
  if (!Platform::Initialized()) {
    report.failure = "HDDAGI requires an initialized Vulkan platform";
    return report;
  }
  const auto& device = Platform::GetSelectedPhysicalDevice();
  report.device_name = device->properties.deviceName;
  const auto& limits = device->properties.limits;
  if (!device->features.fragmentStoresAndAtomics || !device->features.shaderStorageImageExtendedFormats ||
      limits.maxComputeWorkGroupInvocations < 512 || limits.maxComputeWorkGroupSize[0] < 64 ||
      limits.maxComputeWorkGroupSize[1] < 8 || limits.maxComputeWorkGroupSize[2] < 8 ||
      limits.maxComputeSharedMemorySize < 12292 || limits.maxPerStageDescriptorStorageImages < 16 ||
      limits.maxDescriptorSetStorageImages < 16 || limits.maxBoundDescriptorSets < 6) {
    report.failure = "HDDAGI shader feature/workgroup/descriptor limits are unavailable";
    return report;
  }
  for (const auto& r : GetHddagiImageRequirements(p, s)) {
    const auto storage = Platform::GetPhysicalDeviceFormatProperties(r.storage_format).optimalTilingFeatures;
    const auto sampled = Platform::GetPhysicalDeviceFormatProperties(r.sampled_format).optimalTilingFeatures;
    const VkFormatFeatureFlags2 storage_required =
        VK_FORMAT_FEATURE_2_STORAGE_IMAGE_BIT | VK_FORMAT_FEATURE_2_TRANSFER_SRC_BIT |
        VK_FORMAT_FEATURE_2_TRANSFER_DST_BIT | (r.atomic ? VK_FORMAT_FEATURE_2_STORAGE_IMAGE_ATOMIC_BIT : 0);
    const VkFormatFeatureFlags2 sampled_required =
        VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_BIT | (r.filtered ? VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_FILTER_LINEAR_BIT : 0);
    auto info = ImageInfo(r);
    const std::array formats{r.storage_format, r.sampled_format};
    VkImageFormatListCreateInfo format_list{VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO};
    format_list.viewFormatCount = r.storage_format == r.sampled_format ? 1 : 2;
    format_list.pViewFormats = formats.data();
    info.pNext = &format_list;
    VkPhysicalDeviceImageFormatInfo2 query{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_FORMAT_INFO_2};
    query.pNext = &format_list;
    query.format = info.format;
    query.type = info.imageType;
    query.tiling = info.tiling;
    query.usage = info.usage;
    query.flags = info.flags;
    VkImageFormatProperties2 properties{VK_STRUCTURE_TYPE_IMAGE_FORMAT_PROPERTIES_2};
    const auto result = vkGetPhysicalDeviceImageFormatProperties2(device->vk_physical_device, &query, &properties);
    const auto& max = properties.imageFormatProperties;
    if (result != VK_SUCCESS || (storage & storage_required) != storage_required ||
        (sampled & sampled_required) != sampled_required || max.maxExtent.width < r.extent.width ||
        max.maxExtent.height < r.extent.height || max.maxExtent.depth < r.extent.depth ||
        max.maxArrayLayers < r.layers) {
      report.failure = "HDDAGI unsupported image format or dimensions: " + r.name;
      return report;
    }
    VkImage image = VK_NULL_HANDLE;
    if (vkCreateImage(Platform::GetVkDevice(), &info, nullptr, &image) != VK_SUCCESS) {
      report.failure = "HDDAGI could not query image memory: " + r.name;
      return report;
    }
    VkMemoryRequirements memory{};
    vkGetImageMemoryRequirements(Platform::GetVkDevice(), image, &memory);
    vkDestroyImage(Platform::GetVkDevice(), image, nullptr);
    report.image_bytes += memory.size;
    if (r.temporal)
      report.temporal_bytes += memory.size;
    if (memory.size > max.maxResourceSize || report.temporal_bytes >= kTemporalLimit) {
      report.failure = "HDDAGI image or temporal memory limit exceeded";
      return report;
    }
  }
  for (const auto bytes : {uint64_t(LightCellCapacity(p)) * 16, uint64_t(20), uint64_t(16)}) {
    if (bytes > limits.maxStorageBufferRange) {
      report.failure = "HDDAGI light payload exceeds the storage-buffer range";
      return report;
    }
    const auto info = BufferInfo(bytes);
    VkBuffer buffer = VK_NULL_HANDLE;
    if (vkCreateBuffer(Platform::GetVkDevice(), &info, nullptr, &buffer) != VK_SUCCESS) {
      report.failure = "HDDAGI could not query buffer memory";
      return report;
    }
    VkMemoryRequirements memory{};
    vkGetBufferMemoryRequirements(Platform::GetVkDevice(), buffer, &memory);
    vkDestroyBuffer(Platform::GetVkDevice(), buffer, nullptr);
    report.buffer_bytes += memory.size * (bytes == 16 ? 1 : p.cascade_count * 2);
  }
  return report;
}

std::shared_ptr<HddagiResources> HddagiResources::TryCreate(const GiProbeSettings& probes,
                                                            const HddagiSettings& settings, std::string& failure,
                                                            const uint32_t fail_after_allocations) {
  failure = QueryHddagiCapabilities(probes, settings).failure;
  if (!failure.empty())
    return {};
  try {
    auto result = std::make_shared<HddagiResources>();
    result->probes = probes;
    result->settings = settings;
    for (const auto& r : GetHddagiImageRequirements(probes, settings)) {
      if (result->images.size() == fail_after_allocations)
        throw std::runtime_error("Injected partial allocation failure");
      auto info = ImageInfo(r);
      const std::array formats{r.storage_format, r.sampled_format};
      VkImageFormatListCreateInfo format_list{VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO};
      format_list.viewFormatCount = r.storage_format == r.sampled_format ? 1 : 2;
      format_list.pViewFormats = formats.data();
      info.pNext = &format_list;
      auto image = std::make_shared<Image>(info);
      if (!image->GetVkImage())
        throw std::runtime_error("Invalid image allocation: " + r.name);
      const auto view = [&](VkFormat format, VkImageUsageFlags usage) {
        VkImageViewUsageCreateInfo view_usage{VK_STRUCTURE_TYPE_IMAGE_VIEW_USAGE_CREATE_INFO};
        view_usage.usage = usage;
        VkImageViewCreateInfo v{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        v.pNext = &view_usage;
        v.image = image->GetVkImage();
        v.viewType = r.type == VK_IMAGE_TYPE_3D                      ? VK_IMAGE_VIEW_TYPE_3D
                     : r.cube && usage == VK_IMAGE_USAGE_SAMPLED_BIT ? VK_IMAGE_VIEW_TYPE_CUBE
                                                                     : VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        v.format = format;
        v.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, r.layers};
        return std::make_shared<ImageView>(v, image);
      };
      auto storage = view(r.storage_format, VK_IMAGE_USAGE_STORAGE_BIT);
      auto sampled = view(r.sampled_format, VK_IMAGE_USAGE_SAMPLED_BIT);
      if (!image->GetVkImage() || !storage->GetVkImageView() || !sampled->GetVkImageView())
        throw std::runtime_error("Invalid image allocation: " + r.name);
      result->allocation_bytes += image->GetVmaAllocationInfo().size;
      if (r.temporal)
        result->temporal_bytes += image->GetVmaAllocationInfo().size;
      result->images.emplace(r.name, HddagiImage{r, image, storage, sampled});
    }
    result->light_cell_capacity = LightCellCapacity(probes);
    if (result->images.size() == fail_after_allocations)
      throw std::runtime_error("Injected partial allocation failure");
    result->buffers.emplace("Status", std::make_shared<Buffer>(BufferInfo(16)));
    for (uint32_t cascade = 0; cascade < probes.cascade_count; ++cascade)
      for (const auto name : {"Process", "Dispatch", "ProcessSpare", "DispatchSpare"}) {
        if (result->images.size() + result->buffers.size() == fail_after_allocations)
          throw std::runtime_error("Injected partial allocation failure");
        auto info =
            BufferInfo(std::string(name).find("Process") == 0 ? uint64_t(result->light_cell_capacity) * 16 : 20);
        auto buffer = std::make_shared<Buffer>(info);
        result->buffers.emplace(std::string(name) + std::to_string(cascade), std::move(buffer));
      }
    if (result->temporal_bytes >= kTemporalLimit)
      throw std::runtime_error("HDDAGI allocated temporal storage must remain below 4 GiB");
    return result;
  } catch (const std::exception& error) {
    failure = std::string("HDDAGI allocation failed: ") + error.what();
    return {};
  }
}
