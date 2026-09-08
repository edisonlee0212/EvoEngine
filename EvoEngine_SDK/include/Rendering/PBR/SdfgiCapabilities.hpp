#pragma once

#include <volk.h>
#include <cstdint>
#include <string>
#include <vector>

namespace evo_engine {

inline constexpr const char* kSdfgiReferenceCommit = "34d06658a85845111a50db9e485ec4a0701d4298";

struct EVOENGINE_API SdfgiImageRequirement {
  const char* name;
  VkFormat storage_format;
  VkFormat sampled_format;
  VkImageType type;
  VkExtent3D extent;
  uint32_t layers;
  uint32_t bytes_per_texel;
  bool linear_filter = false;
  bool atomic = false;

  [[nodiscard]] VkImageCreateFlags CreateFlags() const;
};

struct SdfgiCapabilityCheck {
  std::string name;
  bool supported = false;
};

struct EVOENGINE_API SdfgiCapabilityReport {
  std::string device_name;
  uint32_t driver_version = 0;
  bool ray_tracing_enabled = false;
  bool ray_query_enabled = false;
  bool acceleration_structures_enabled = false;
  std::vector<SdfgiCapabilityCheck> checks;

  [[nodiscard]] bool Supported() const;
  [[nodiscard]] std::string ToString() const;
};

EVOENGINE_API std::vector<SdfgiImageRequirement> GetSdfgiImageRequirements(
    uint32_t cascade_count = 4, uint32_t history_size = 30, uint32_t voxel_count_x = 256, uint32_t voxel_count_y = 128,
    uint32_t probe_spacing_cells = 8, uint32_t max_image_dimension = 16384);
EVOENGINE_API std::vector<SdfgiCapabilityCheck> EvaluateSdfgiDeviceLimits(const VkPhysicalDeviceFeatures& features,
                                                                          const VkPhysicalDeviceLimits& limits,
                                                                          uint32_t voxel_count_x = 256,
                                                                          uint32_t voxel_count_y = 128);
EVOENGINE_API bool SupportsSdfgiImage(const SdfgiImageRequirement& requirement, VkFormatFeatureFlags2 storage_features,
                                      VkFormatFeatureFlags2 sampled_features, VkResult query_result,
                                      const VkImageFormatProperties& properties);
EVOENGINE_API SdfgiCapabilityReport QuerySdfgiCapabilities(uint32_t cascade_count = 4, uint32_t history_size = 30,
                                                           uint32_t voxel_count_x = 256, uint32_t voxel_count_y = 128,
                                                           uint32_t probe_spacing_cells = 8,
                                                           uint64_t other_history_bytes = 0);

}  // namespace evo_engine
