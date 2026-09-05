// Resource requirements follow Godot environment/gi.cpp and the SDFGI shaders at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiCapabilities.hpp"

#include <algorithm>
#include <array>
#include <sstream>

#include "Platform.hpp"

using namespace evo_engine;

namespace {
constexpr VkImageUsageFlags kImageUsage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                          VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
}

VkImageCreateFlags SdfgiImageRequirement::CreateFlags() const {
  return storage_format == sampled_format ? 0 : VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT | VK_IMAGE_CREATE_EXTENDED_USAGE_BIT;
}

std::vector<SdfgiImageRequirement> evo_engine::GetSdfgiImageRequirements(const uint32_t cascade_count,
                                                                         const uint32_t history_size) {
  if (cascade_count < 1 || cascade_count > 8 || history_size < 5 || history_size > 30 || history_size % 5 != 0) {
    return {};
  }
  return {
      {"albedo", VK_FORMAT_R16_UINT, VK_FORMAT_R16_UINT, VK_IMAGE_TYPE_3D, {128, 128, 128}, 1, 2},
      {"emission", VK_FORMAT_R32_UINT, VK_FORMAT_R32_UINT, VK_IMAGE_TYPE_3D, {128, 128, 128}, 1, 4},
      {"facing", VK_FORMAT_R32_UINT, VK_FORMAT_R32_UINT, VK_IMAGE_TYPE_3D, {128, 128, 128}, 1, 4, false, true},
      {"jump_flood", VK_FORMAT_R8G8B8A8_UINT, VK_FORMAT_R8G8B8A8_UINT, VK_IMAGE_TYPE_3D, {128, 128, 128}, 1, 4},
      {"jump_flood_half", VK_FORMAT_R8G8B8A8_UINT, VK_FORMAT_R8G8B8A8_UINT, VK_IMAGE_TYPE_3D, {64, 64, 64}, 1, 4},
      {"sdf_and_occlusion_scratch",
       VK_FORMAT_R8_UNORM,
       VK_FORMAT_R8_UNORM,
       VK_IMAGE_TYPE_3D,
       {128, 128, 128},
       1,
       1,
       true},
      {"radiance", VK_FORMAT_R32_UINT, VK_FORMAT_E5B9G9R9_UFLOAT_PACK32, VK_IMAGE_TYPE_3D, {128, 128, 128}, 1, 4, true},
      {"anisotropy_0",
       VK_FORMAT_R8G8B8A8_UNORM,
       VK_FORMAT_R8G8B8A8_UNORM,
       VK_IMAGE_TYPE_3D,
       {128, 128, 128},
       1,
       4,
       true},
      {"anisotropy_1", VK_FORMAT_R8G8_UNORM, VK_FORMAT_R8G8_UNORM, VK_IMAGE_TYPE_3D, {128, 128, 128}, 1, 2, true},
      {"occlusion",
       VK_FORMAT_R16_UINT,
       VK_FORMAT_R4G4B4A4_UNORM_PACK16,
       VK_IMAGE_TYPE_3D,
       {256, 128, 128 * cascade_count},
       1,
       2,
       true},
      {"history",
       VK_FORMAT_R16G16B16A16_SINT,
       VK_FORMAT_R16G16B16A16_SINT,
       VK_IMAGE_TYPE_2D,
       {289, 272, 1},
       history_size,
       8},
      {"average", VK_FORMAT_R32G32B32A32_SINT, VK_FORMAT_R32G32B32A32_SINT, VK_IMAGE_TYPE_2D, {289, 272, 1}, 1, 16},
      {"probe_atlas",
       VK_FORMAT_R32_UINT,
       VK_FORMAT_E5B9G9R9_UFLOAT_PACK32,
       VK_IMAGE_TYPE_2D,
       {2312, 136, 1},
       2 * cascade_count,
       4,
       true},
      {"ambient",
       VK_FORMAT_R16G16B16A16_SFLOAT,
       VK_FORMAT_R16G16B16A16_SFLOAT,
       VK_IMAGE_TYPE_2D,
       {289, 17, 1},
       cascade_count,
       8},
  };
}

std::vector<SdfgiCapabilityCheck> evo_engine::EvaluateSdfgiDeviceLimits(const VkPhysicalDeviceFeatures& features,
                                                                        const VkPhysicalDeviceLimits& limits) {
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
  require("maxStorageBufferRange", limits.maxStorageBufferRange, 128u * 128u * 128u / 4u * 16u);
  require("maxComputeWorkGroupInvocations", limits.maxComputeWorkGroupInvocations, 512);
  require("maxComputeWorkGroupSize.x", limits.maxComputeWorkGroupSize[0], 64);
  require("maxComputeWorkGroupSize.y", limits.maxComputeWorkGroupSize[1], 8);
  require("maxComputeWorkGroupSize.z", limits.maxComputeWorkGroupSize[2], 8);
  require("maxComputeWorkGroupCount.x", limits.maxComputeWorkGroupCount[0], 8192);
  require("maxComputeWorkGroupCount.y", limits.maxComputeWorkGroupCount[1], 32);
  require("maxComputeWorkGroupCount.z", limits.maxComputeWorkGroupCount[2], 32);
  require("maxComputeSharedMemorySize", limits.maxComputeSharedMemorySize, 16000);
  require("maxPerStageDescriptorStorageImages", limits.maxPerStageDescriptorStorageImages, 15);
  require("maxDescriptorSetStorageImages", limits.maxDescriptorSetStorageImages, 15);
  require("maxPerStageDescriptorSampledImages", limits.maxPerStageDescriptorSampledImages, 33);
  require("maxDescriptorSetSampledImages", limits.maxDescriptorSetSampledImages, 33);
  require("maxPerStageDescriptorSamplers", limits.maxPerStageDescriptorSamplers, 2);
  require("maxDescriptorSetSamplers", limits.maxDescriptorSetSamplers, 2);
  require("maxPerStageDescriptorStorageBuffers", limits.maxPerStageDescriptorStorageBuffers, 4);
  require("maxDescriptorSetStorageBuffers", limits.maxDescriptorSetStorageBuffers, 4);
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

SdfgiCapabilityReport evo_engine::QuerySdfgiCapabilities(const uint32_t cascade_count, const uint32_t history_size) {
  SdfgiCapabilityReport report;
  const auto requirements = GetSdfgiImageRequirements(cascade_count, history_size);
  report.checks.push_back({"configuration: cascades 1..8, history 5..30 in steps of 5", !requirements.empty()});
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
  const auto device_checks = EvaluateSdfgiDeviceLimits(device->features, device->properties.limits);
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
  return report;
}
