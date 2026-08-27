#pragma once

#include "volk.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace evo_engine {

struct PipelineCacheIdentity {
  uint32_t schema = 1;
  uint32_t vendor_id = 0;
  uint32_t device_id = 0;
  uint32_t driver_version = 0;
  uint32_t api_version = 0;
  std::array<uint8_t, VK_UUID_SIZE> uuid{};

  bool operator==(const PipelineCacheIdentity& other) const {
    return schema == other.schema && vendor_id == other.vendor_id && device_id == other.device_id &&
           driver_version == other.driver_version && api_version == other.api_version && uuid == other.uuid;
  }
};

enum class PipelineCacheLoadResult { Missing, Valid, Corrupt, Incompatible, Oversized };

struct PipelineCreationFeedback {
  VkResult result = VK_SUCCESS;
  bool feedback_supported = false;
  bool feedback_valid = false;
  bool application_cache_hit = false;
  bool deferred_requested = false;
  bool deferred_used = false;
  bool synchronous_fallback = false;
  uint64_t duration_nanoseconds = 0;
  double wall_milliseconds = 0.0;
  std::string fallback_reason;
};

class EVOENGINE_API VulkanPipelineCache final {
 public:
  static constexpr size_t kMaxCacheFileBytes = 512ull * 1024ull * 1024ull;

  VulkanPipelineCache() = default;
  VulkanPipelineCache(const VulkanPipelineCache&) = delete;
  VulkanPipelineCache& operator=(const VulkanPipelineCache&) = delete;
  ~VulkanPipelineCache();

  [[nodiscard]] static PipelineCacheIdentity MakeIdentity(const VkPhysicalDeviceProperties& properties);
  [[nodiscard]] static std::filesystem::path ResolveCachePath(const PipelineCacheIdentity& identity);
  [[nodiscard]] static PipelineCacheLoadResult LoadFile(const std::filesystem::path& path,
                                                        const PipelineCacheIdentity& identity,
                                                        std::vector<uint8_t>& payload);
  [[nodiscard]] static bool PublishFile(const std::filesystem::path& path, const PipelineCacheIdentity& identity,
                                        const std::vector<uint8_t>& payload);
  [[nodiscard]] static bool IsRawPayloadCompatible(const std::vector<uint8_t>& payload,
                                                   const PipelineCacheIdentity& identity);
  [[nodiscard]] static bool IsRayTracingCreateSuccess(VkResult result);
  [[nodiscard]] static bool ShouldRetryRayTracingSynchronously(VkResult result);
  [[nodiscard]] static bool IsIncompleteDeferredOperation(VkResult result);

  bool Initialize(VkDevice device, const VkPhysicalDeviceProperties& properties, bool feedback_supported,
                  bool deferred_host_operations_supported);
  void Shutdown() noexcept;

  VkResult CreateComputePipeline(const VkComputePipelineCreateInfo& create_info, VkPipeline& pipeline,
                                 PipelineCreationFeedback& feedback);
  VkResult CreateGraphicsPipeline(const VkGraphicsPipelineCreateInfo& create_info, VkPipeline& pipeline,
                                  PipelineCreationFeedback& feedback);
  VkResult CreateRayTracingPipeline(const VkRayTracingPipelineCreateInfoKHR& create_info, VkPipeline& pipeline,
                                    PipelineCreationFeedback& feedback);

 private:
  bool SaveLocked();

  mutable std::mutex mutex_;
  VkDevice device_ = VK_NULL_HANDLE;
  VkPipelineCache cache_ = VK_NULL_HANDLE;
  PipelineCacheIdentity identity_{};
  std::filesystem::path path_;
  bool feedback_supported_ = false;
  bool deferred_host_operations_supported_ = false;
};

[[nodiscard]] const char* GetPipelineCacheLoadResultName(PipelineCacheLoadResult result);

}  // namespace evo_engine
