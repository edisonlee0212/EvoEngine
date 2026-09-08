// Resource requirements follow Godot environment/gi.cpp and the SDFGI shaders at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiCapabilities.hpp"

#include <algorithm>
#include <array>
#include <sstream>

#include "Platform.hpp"
#include "SdfgiProbeLayout.hpp"
#include "SdfgiSettings.hpp"
#include "SdfgiTypes.hpp"

using namespace evo_engine;

namespace {
constexpr VkImageUsageFlags kImageUsage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                          VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
}

VkImageCreateFlags SdfgiImageRequirement::CreateFlags() const {
  return storage_format == sampled_format ? 0 : VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT | VK_IMAGE_CREATE_EXTENDED_USAGE_BIT;
}

std::vector<SdfgiImageRequirement> evo_engine::GetSdfgiImageRequirements(
    const uint32_t cascade_count, const uint32_t history_size, const uint32_t voxel_count_x,
    const uint32_t voxel_count_y, const uint32_t probe_spacing_cells, const uint32_t max_image_dimension) {
  if (cascade_count < 1 || cascade_count > 8 || history_size < 5 || history_size > 30 || history_size % 5 != 0) {
    return {};
  }
  SdfgiSettings settings;
  settings.voxel_count_x = voxel_count_x;
  settings.voxel_count_y = voxel_count_y;
  settings.probe_spacing_cells = probe_spacing_cells;
  if (!settings.Validate().empty())
    return {};
  const uint32_t horizontal = voxel_count_x;
  const auto layout = SdfgiProbeLayout::Create(voxel_count_x, voxel_count_y, probe_spacing_cells, max_image_dimension);
  if (!layout.columns)
    return {};
  const uint32_t rows = layout.rows;
  const uint32_t columns = layout.columns;
  return {
      {"albedo",
       VK_FORMAT_R16_UINT,
       VK_FORMAT_R16_UINT,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       2},
      {"emission",
       VK_FORMAT_R32_UINT,
       VK_FORMAT_R32_UINT,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       4},
      {"facing",
       VK_FORMAT_R32_UINT,
       VK_FORMAT_R32_UINT,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       4,
       false,
       true},
      {"jump_flood",
       VK_FORMAT_R8G8B8A8_UINT,
       VK_FORMAT_R8G8B8A8_UINT,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       4},
      {"jump_flood_half",
       VK_FORMAT_R8G8B8A8_UINT,
       VK_FORMAT_R8G8B8A8_UINT,
       VK_IMAGE_TYPE_3D,
       {horizontal / 2, voxel_count_y / 2, horizontal / 2},
       1,
       4},
      {"sdf_and_occlusion_scratch",
       VK_FORMAT_R8_UNORM,
       VK_FORMAT_R8_UNORM,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       1,
       true},
      {"radiance",
       VK_FORMAT_R32_UINT,
       VK_FORMAT_E5B9G9R9_UFLOAT_PACK32,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       4,
       true},
      {"anisotropy_0",
       VK_FORMAT_R8G8B8A8_UNORM,
       VK_FORMAT_R8G8B8A8_UNORM,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       4,
       true},
      {"anisotropy_1",
       VK_FORMAT_R8G8_UNORM,
       VK_FORMAT_R8G8_UNORM,
       VK_IMAGE_TYPE_3D,
       {horizontal, voxel_count_y, horizontal},
       1,
       2,
       true},
      {"occlusion",
       VK_FORMAT_R16_UINT,
       VK_FORMAT_R4G4B4A4_UNORM_PACK16,
       VK_IMAGE_TYPE_3D,
       {2 * horizontal, voxel_count_y, horizontal * cascade_count},
       1,
       2,
       true},
      {"history",
       VK_FORMAT_R16G16B16A16_SINT,
       VK_FORMAT_R16G16B16A16_SINT,
       VK_IMAGE_TYPE_2D,
       {columns, rows * 16, 1},
       history_size,
       8},
      {"average",
       VK_FORMAT_R32G32B32A32_SINT,
       VK_FORMAT_R32G32B32A32_SINT,
       VK_IMAGE_TYPE_2D,
       {columns, rows * 16, 1},
       1,
       16},
      {"probe_atlas",
       VK_FORMAT_R32_UINT,
       VK_FORMAT_E5B9G9R9_UFLOAT_PACK32,
       VK_IMAGE_TYPE_2D,
       {columns * 8, rows * 8, 1},
       2 * cascade_count,
       4,
       true},
      {"ambient",
       VK_FORMAT_R16G16B16A16_SFLOAT,
       VK_FORMAT_R16G16B16A16_SFLOAT,
       VK_IMAGE_TYPE_2D,
       {columns, rows, 1},
       cascade_count,
       8},
  };
}

std::vector<SdfgiCapabilityCheck> evo_engine::EvaluateSdfgiDeviceLimits(const VkPhysicalDeviceFeatures& features,
                                                                        const VkPhysicalDeviceLimits& limits,
                                                                        const uint32_t voxel_count_x,
                                                                        const uint32_t voxel_count_y) {
  std::vector<SdfgiCapabilityCheck> checks{
      {"fragmentStoresAndAtomics", features.fragmentStoresAndAtomics == VK_TRUE},
      {"shaderStorageImageExtendedFormats", features.shaderStorageImageExtendedFormats == VK_TRUE},
      {"shaderSampledImageArrayDynamicIndexing", features.shaderSampledImageArrayDynamicIndexing == VK_TRUE},
      {"shaderStorageImageArrayDynamicIndexing", features.shaderStorageImageArrayDynamicIndexing == VK_TRUE},
  };
  const auto require = [&](const char* name, const uint64_t available, const uint64_t required) {
    checks.push_back(
        {std::string(name) + " required=" + std::to_string(required) + " available=" + std::to_string(available),
         available >= required});
  };
  require("maxBoundDescriptorSets", limits.maxBoundDescriptorSets, 6);
  require("maxPushConstantsSize", limits.maxPushConstantsSize, 112);
  require("maxUniformBufferRange", limits.maxUniformBufferRange, 512);
  SdfgiSettings settings;
  settings.voxel_count_x = voxel_count_x;
  settings.voxel_count_y = voxel_count_y;
  if (const auto failure = settings.Validate(); !failure.empty())
    return {{failure, false}};
  require("maxStorageBufferRange", limits.maxStorageBufferRange, settings.SolidCellCapacity() * sizeof(SdfgiSolidCell));
  require("maxComputeWorkGroupInvocations", limits.maxComputeWorkGroupInvocations, 512);
  require("maxComputeWorkGroupSize.x", limits.maxComputeWorkGroupSize[0], 64);
  require("maxComputeWorkGroupSize.y", limits.maxComputeWorkGroupSize[1], 8);
  require("maxComputeWorkGroupSize.z", limits.maxComputeWorkGroupSize[2], 8);
  require("maxComputeWorkGroupCount.x", limits.maxComputeWorkGroupCount[0], (settings.SolidCellCapacity() + 63) / 64);
  require("maxComputeWorkGroupCount.y", limits.maxComputeWorkGroupCount[1], settings.GridSize().y / 4);
  require("maxComputeWorkGroupCount.z", limits.maxComputeWorkGroupCount[2], settings.GridSize().z / 4);
  require("maxComputeSharedMemorySize", limits.maxComputeSharedMemorySize, 16000);
  require("maxPerStageDescriptorStorageImages", limits.maxPerStageDescriptorStorageImages, 15);
  require("maxDescriptorSetStorageImages", limits.maxDescriptorSetStorageImages, 15);
  require("maxPerStageDescriptorSampledImages", limits.maxPerStageDescriptorSampledImages, 33);
  require("maxDescriptorSetSampledImages", limits.maxDescriptorSetSampledImages, 33);
  require("maxPerStageDescriptorSamplers", limits.maxPerStageDescriptorSamplers, 2);
  require("maxDescriptorSetSamplers", limits.maxDescriptorSetSamplers, 2);
  require("maxPerStageDescriptorStorageBuffers", limits.maxPerStageDescriptorStorageBuffers, 5);
  require("maxDescriptorSetStorageBuffers", limits.maxDescriptorSetStorageBuffers, 5);
  require("maxPerStageResources", limits.maxPerStageResources, 48);
  return checks;
}

bool evo_engine::SupportsSdfgiImage(const SdfgiImageRequirement& requirement,
                                    const VkFormatFeatureFlags2 storage_features,
                                    const VkFormatFeatureFlags2 sampled_features, const VkResult query_result,
                                    const VkImageFormatProperties& properties) {
  const VkFormatFeatureFlags2 storage_required =
      VK_FORMAT_FEATURE_2_STORAGE_IMAGE_BIT | VK_FORMAT_FEATURE_2_TRANSFER_SRC_BIT |
      VK_FORMAT_FEATURE_2_TRANSFER_DST_BIT | (requirement.atomic ? VK_FORMAT_FEATURE_2_STORAGE_IMAGE_ATOMIC_BIT : 0);
  const VkFormatFeatureFlags2 sampled_required =
      VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_BIT |
      (requirement.linear_filter ? VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_FILTER_LINEAR_BIT : 0);
  const uint64_t bytes = uint64_t(requirement.extent.width) * requirement.extent.height * requirement.extent.depth *
                         requirement.layers * requirement.bytes_per_texel;
  return query_result == VK_SUCCESS && (storage_features & storage_required) == storage_required &&
         (sampled_features & sampled_required) == sampled_required &&
         properties.maxExtent.width >= requirement.extent.width &&
         properties.maxExtent.height >= requirement.extent.height &&
         properties.maxExtent.depth >= requirement.extent.depth && properties.maxArrayLayers >= requirement.layers &&
         properties.maxMipLevels >= 1 && (properties.sampleCounts & VK_SAMPLE_COUNT_1_BIT) != 0 &&
         properties.maxResourceSize >= bytes;
}

bool SdfgiCapabilityReport::Supported() const {
  return !checks.empty() && std::all_of(checks.begin(), checks.end(), [](const auto& check) {
    return check.supported;
  });
}

std::string SdfgiCapabilityReport::ToString() const {
  std::ostringstream stream;
  stream << "SDFGI preflight supported=" << Supported() << " gpu=\"" << device_name << "\" driver=" << driver_version
         << " ray_tracing=" << ray_tracing_enabled << " ray_query=" << ray_query_enabled
         << " blas=" << acceleration_structures_enabled << " tlas=" << acceleration_structures_enabled;
  for (const auto& check : checks) {
    if (!check.supported) {
      stream << "\n  unavailable: " << check.name;
    }
  }
  return stream.str();
}

SdfgiCapabilityReport evo_engine::QuerySdfgiCapabilities(const uint32_t cascade_count, const uint32_t history_size,
                                                         const uint32_t voxel_count_x, const uint32_t voxel_count_y,
                                                         const uint32_t probe_spacing_cells,
                                                         const uint64_t other_history_bytes) {
  SdfgiCapabilityReport report;
  const auto max_dimension =
      Platform::Initialized() ? Platform::GetSelectedPhysicalDevice()->properties.limits.maxImageDimension2D : 16384u;
  const auto requirements = GetSdfgiImageRequirements(cascade_count, history_size, voxel_count_x, voxel_count_y,
                                                      probe_spacing_cells, max_dimension);
  const auto layout = SdfgiProbeLayout::Create(voxel_count_x, voxel_count_y, probe_spacing_cells, max_dimension);
  uint64_t history_bytes = 0;
  report.checks.push_back(
      {"Combined GI history must be below 4 GiB", layout.HistoryBytes(cascade_count, history_size, history_bytes) &&
                                                      GiHistoryBudget{other_history_bytes}.CanAdd(history_bytes)});
  report.checks.push_back(
      {"configuration: cascades 1..8, history 5..30 step 5, X/Y voxels 64..256 step 16", !requirements.empty()});
  report.checks.push_back({"Vulkan platform initialized", Platform::Initialized()});
  if (!Platform::Initialized()) {
    return report;
  }
  const auto& device = Platform::GetSelectedPhysicalDevice();
  report.device_name = device->properties.deviceName;
  report.driver_version = device->properties.driverVersion;
  report.ray_tracing_enabled = Platform::RayTracingEnabled();
  report.ray_query_enabled = Platform::RayQueryEnabled();
  report.acceleration_structures_enabled = Platform::RayAccelerationStructureEnabled();
  if (requirements.empty()) {
    return report;
  }
  const auto device_checks =
      EvaluateSdfgiDeviceLimits(device->features, device->properties.limits, voxel_count_x, voxel_count_y);
  report.checks.insert(report.checks.end(), device_checks.begin(), device_checks.end());
  for (const auto& requirement : requirements) {
    const std::array formats{requirement.storage_format, requirement.sampled_format};
    VkImageFormatListCreateInfo format_list{VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO};
    format_list.viewFormatCount = requirement.storage_format == requirement.sampled_format ? 1 : 2;
    format_list.pViewFormats = formats.data();
    VkPhysicalDeviceImageFormatInfo2 info{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_FORMAT_INFO_2};
    info.pNext = &format_list;
    info.format = requirement.storage_format;
    info.type = requirement.type;
    info.tiling = VK_IMAGE_TILING_OPTIMAL;
    info.usage = kImageUsage;
    info.flags = requirement.CreateFlags();
    VkImageFormatProperties2 properties{VK_STRUCTURE_TYPE_IMAGE_FORMAT_PROPERTIES_2};
    const auto result = vkGetPhysicalDeviceImageFormatProperties2(device->vk_physical_device, &info, &properties);
    const auto storage_features = Platform::GetPhysicalDeviceFormatProperties(requirement.storage_format);
    const auto sampled_features = Platform::GetPhysicalDeviceFormatProperties(requirement.sampled_format);
    report.checks.push_back(
        {std::string("image ") + requirement.name + " format=" + std::to_string(requirement.storage_format) +
             " sampled=" + std::to_string(requirement.sampled_format) + " query=" + std::to_string(result),
         SupportsSdfgiImage(requirement, storage_features.optimalTilingFeatures, sampled_features.optimalTilingFeatures,
                            result, properties.imageFormatProperties)});
  }
  if (report.Supported()) {
    GiHistoryBudget budget{other_history_bytes};
    bool fits = true;
    for (const size_t index : {size_t{10}, size_t{11}}) {
      const auto& requirement = requirements[index];
      VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
      info.imageType = requirement.type;
      info.format = requirement.storage_format;
      info.extent = requirement.extent;
      info.mipLevels = 1;
      info.arrayLayers = requirement.layers;
      info.samples = VK_SAMPLE_COUNT_1_BIT;
      info.tiling = VK_IMAGE_TILING_OPTIMAL;
      info.usage = kImageUsage;
      VkImage image = VK_NULL_HANDLE;
      if (vkCreateImage(Platform::GetVkDevice(), &info, nullptr, &image) != VK_SUCCESS) {
        fits = false;
        break;
      }
      VkMemoryRequirements memory{};
      vkGetImageMemoryRequirements(Platform::GetVkDevice(), image, &memory);
      vkDestroyImage(Platform::GetVkDevice(), image, nullptr);
      uint64_t bytes = memory.size;
      fits = fits && MultiplyGiHistoryBytes(bytes, uint64_t{cascade_count} + 1) && budget.Add(bytes);
    }
    report.checks.push_back({"Device-padded combined GI history must be below 4 GiB", fits});
  }
  return report;
}
