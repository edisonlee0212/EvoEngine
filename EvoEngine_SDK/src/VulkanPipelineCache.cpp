#include "VulkanPipelineCache.hpp"

#include "Console.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#ifdef _WIN32
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
constexpr std::array<char, 8> kCacheMagic = {'E', 'V', 'O', 'V', 'K', 'P', 'C', '1'};
constexpr uint64_t kFileHeaderBytes = 60;
std::atomic<uint64_t> temporary_file_counter = 0;

template <typename T>
bool ReadValue(std::istream& stream, T& value) {
  stream.read(reinterpret_cast<char*>(&value), sizeof(T));
  return stream.good();
}

template <typename T>
void WriteValue(std::ostream& stream, const T& value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

uint64_t PayloadChecksum(const std::vector<uint8_t>& payload) {
  uint64_t hash = 14695981039346656037ull;
  for (const auto value : payload) {
    hash ^= value;
    hash *= 1099511628211ull;
  }
  return hash;
}

template <typename T>
T ReadPayloadValue(const std::vector<uint8_t>& payload, const size_t offset) {
  T value{};
  std::memcpy(&value, payload.data() + offset, sizeof(T));
  return value;
}

bool ReplaceFile(const std::filesystem::path& temporary_path, const std::filesystem::path& path) {
#ifdef _WIN32
  return MoveFileExW(temporary_path.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != FALSE;
#else
  std::error_code error;
  std::filesystem::rename(temporary_path, path, error);
  return !error;
#endif
}

void PopulateFeedback(const VkPipelineCreationFeedback& raw_feedback, const bool feedback_supported,
                      PipelineCreationFeedback& feedback) {
  feedback.feedback_supported = feedback_supported;
  feedback.feedback_valid = feedback_supported && (raw_feedback.flags & VK_PIPELINE_CREATION_FEEDBACK_VALID_BIT) != 0;
  if (!feedback.feedback_valid)
    return;
  feedback.application_cache_hit =
      (raw_feedback.flags & VK_PIPELINE_CREATION_FEEDBACK_APPLICATION_PIPELINE_CACHE_HIT_BIT) != 0;
  feedback.duration_nanoseconds = raw_feedback.duration;
}

template <typename CreateInfo, typename CreateAction>
VkResult CreatePipelineLocked(const CreateInfo& create_info, const bool feedback_supported, CreateAction&& create,
                              VkPipeline& pipeline, PipelineCreationFeedback& feedback) {
  VkPipelineCreationFeedback raw_feedback{};
  VkPipelineCreationFeedbackCreateInfo feedback_info{};
  auto local_info = create_info;
  if (feedback_supported) {
    feedback_info.sType = VK_STRUCTURE_TYPE_PIPELINE_CREATION_FEEDBACK_CREATE_INFO;
    feedback_info.pNext = local_info.pNext;
    feedback_info.pPipelineCreationFeedback = &raw_feedback;
    local_info.pNext = &feedback_info;
  }
  const auto start = std::chrono::steady_clock::now();
  feedback.result = create(local_info, pipeline);
  feedback.wall_milliseconds =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  PopulateFeedback(raw_feedback, feedback_supported, feedback);
  return feedback.result;
}

VkResult JoinDeferredOperation(const VkDevice device, const VkDeferredOperationKHR operation) {
  constexpr uint32_t max_host_threads = 8;
  const uint32_t driver_limit = vkGetDeferredOperationMaxConcurrencyKHR(device, operation);
  const uint32_t host_limit = std::min(max_host_threads, std::max(1u, std::thread::hardware_concurrency()));
  const uint32_t thread_count =
      std::max(1u, std::min(driver_limit == UINT32_MAX ? host_limit : driver_limit, host_limit));
  std::atomic<bool> operation_complete = false;
  std::atomic<int32_t> first_join_error = VK_SUCCESS;
  const auto join = [&]() {
    while (!operation_complete.load()) {
      const auto result = vkDeferredOperationJoinKHR(device, operation);
      if (result == VK_SUCCESS) {
        operation_complete.store(true);
        return;
      }
      if (result == VK_THREAD_DONE_KHR)
        return;
      if (result == VK_THREAD_IDLE_KHR) {
        std::this_thread::yield();
        continue;
      }
      auto expected = static_cast<int32_t>(VK_SUCCESS);
      first_join_error.compare_exchange_strong(expected, static_cast<int32_t>(result));
      return;
    }
  };
  std::vector<std::thread> workers;
  for (uint32_t i = 1; i < thread_count; ++i) {
    try {
      workers.emplace_back(join);
    } catch (...) {
      break;
    }
  }
  join();
  for (auto& worker : workers)
    worker.join();
  const auto result = vkGetDeferredOperationResultKHR(device, operation);
  if (VulkanPipelineCache::IsIncompleteDeferredOperation(result)) {
    std::fprintf(stderr, "Deferred Vulkan operation remained pending after all joins (join error %d).\n",
                 first_join_error.load());
    std::terminate();
  }
  return result;
}
}  // namespace

const char* evo_engine::GetPipelineCacheLoadResultName(const PipelineCacheLoadResult result) {
  switch (result) {
    case PipelineCacheLoadResult::Missing:
      return "missing";
    case PipelineCacheLoadResult::Valid:
      return "disk";
    case PipelineCacheLoadResult::Corrupt:
      return "corrupt";
    case PipelineCacheLoadResult::Incompatible:
      return "incompatible";
    case PipelineCacheLoadResult::Oversized:
      return "oversized";
  }
  return "unknown";
}

VulkanPipelineCache::~VulkanPipelineCache() {
  Shutdown();
}

PipelineCacheIdentity VulkanPipelineCache::MakeIdentity(const VkPhysicalDeviceProperties& properties) {
  PipelineCacheIdentity identity;
  identity.vendor_id = properties.vendorID;
  identity.device_id = properties.deviceID;
  identity.driver_version = properties.driverVersion;
  identity.api_version = properties.apiVersion;
  std::copy_n(properties.pipelineCacheUUID, VK_UUID_SIZE, identity.uuid.begin());
  return identity;
}

std::filesystem::path VulkanPipelineCache::ResolveCachePath(const PipelineCacheIdentity& identity) {
  std::filesystem::path directory;
  if (const char* path = std::getenv("EVOENGINE_PIPELINE_CACHE_DIR"); path && path[0] != '\0') {
    directory = path;
  } else if (const char* path = std::getenv("EVOENGINE_SHADER_CACHE_DIR"); path && path[0] != '\0') {
    directory = std::filesystem::path(path).parent_path() / "PipelineCache";
  } else {
    directory = "./PipelineCache";
  }
  std::ostringstream name;
  name << "vkpc-v" << identity.schema << '-' << std::hex << std::setfill('0') << std::setw(8) << identity.vendor_id
       << '-' << std::setw(8) << identity.device_id << '-' << std::setw(8) << identity.driver_version << '-'
       << std::setw(8) << identity.api_version << '-';
  for (const auto value : identity.uuid)
    name << std::setw(2) << static_cast<uint32_t>(value);
  name << ".bin";
  return directory / name.str();
}

bool VulkanPipelineCache::IsRawPayloadCompatible(const std::vector<uint8_t>& payload,
                                                 const PipelineCacheIdentity& identity) {
  constexpr size_t minimum_header_size = 4u * sizeof(uint32_t) + VK_UUID_SIZE;
  if (payload.size() < minimum_header_size)
    return false;
  const auto header_size = ReadPayloadValue<uint32_t>(payload, 0);
  const auto header_version = ReadPayloadValue<uint32_t>(payload, 4);
  const auto vendor_id = ReadPayloadValue<uint32_t>(payload, 8);
  const auto device_id = ReadPayloadValue<uint32_t>(payload, 12);
  return header_size >= minimum_header_size && header_size <= payload.size() &&
         header_version == VK_PIPELINE_CACHE_HEADER_VERSION_ONE && vendor_id == identity.vendor_id &&
         device_id == identity.device_id &&
         std::equal(identity.uuid.begin(), identity.uuid.end(), payload.begin() + 4u * sizeof(uint32_t));
}

PipelineCacheLoadResult VulkanPipelineCache::LoadFile(const std::filesystem::path& path,
                                                      const PipelineCacheIdentity& identity,
                                                      std::vector<uint8_t>& payload) {
  payload.clear();
  std::error_code error;
  if (!std::filesystem::exists(path, error) || error)
    return PipelineCacheLoadResult::Missing;
  const auto file_size = std::filesystem::file_size(path, error);
  if (error)
    return PipelineCacheLoadResult::Corrupt;
  if (file_size > kMaxCacheFileBytes)
    return PipelineCacheLoadResult::Oversized;
  if (file_size < kFileHeaderBytes)
    return PipelineCacheLoadResult::Corrupt;
  try {
    std::ifstream stream(path, std::ios::binary);
    std::array<char, 8> magic{};
    stream.read(magic.data(), magic.size());
    PipelineCacheIdentity cached_identity;
    uint64_t payload_size = 0;
    uint64_t checksum = 0;
    if (!stream || magic != kCacheMagic || !ReadValue(stream, cached_identity.schema) ||
        !ReadValue(stream, cached_identity.vendor_id) || !ReadValue(stream, cached_identity.device_id) ||
        !ReadValue(stream, cached_identity.driver_version) || !ReadValue(stream, cached_identity.api_version)) {
      return PipelineCacheLoadResult::Corrupt;
    }
    stream.read(reinterpret_cast<char*>(cached_identity.uuid.data()), cached_identity.uuid.size());
    if (!stream || !ReadValue(stream, payload_size) || !ReadValue(stream, checksum))
      return PipelineCacheLoadResult::Corrupt;
    if (!(cached_identity == identity))
      return PipelineCacheLoadResult::Incompatible;
    if (payload_size == 0 || payload_size > kMaxCacheFileBytes - kFileHeaderBytes ||
        file_size != kFileHeaderBytes + payload_size)
      return PipelineCacheLoadResult::Corrupt;
    payload.resize(static_cast<size_t>(payload_size));
    stream.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    if (!stream || PayloadChecksum(payload) != checksum || !IsRawPayloadCompatible(payload, identity)) {
      payload.clear();
      return PipelineCacheLoadResult::Corrupt;
    }
    return PipelineCacheLoadResult::Valid;
  } catch (...) {
    payload.clear();
    return PipelineCacheLoadResult::Corrupt;
  }
}

bool VulkanPipelineCache::PublishFile(const std::filesystem::path& path, const PipelineCacheIdentity& identity,
                                      const std::vector<uint8_t>& payload) {
  if (payload.empty() || payload.size() > kMaxCacheFileBytes - kFileHeaderBytes ||
      !IsRawPayloadCompatible(payload, identity))
    return false;
  std::filesystem::path temporary_path;
  try {
    std::filesystem::create_directories(path.parent_path());
    temporary_path = path;
    temporary_path += ".tmp." + std::to_string(temporary_file_counter.fetch_add(1)) + "." +
                      std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) + "." +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    stream.write(kCacheMagic.data(), kCacheMagic.size());
    WriteValue(stream, identity.schema);
    WriteValue(stream, identity.vendor_id);
    WriteValue(stream, identity.device_id);
    WriteValue(stream, identity.driver_version);
    WriteValue(stream, identity.api_version);
    stream.write(reinterpret_cast<const char*>(identity.uuid.data()), identity.uuid.size());
    const auto payload_size = static_cast<uint64_t>(payload.size());
    WriteValue(stream, payload_size);
    const auto checksum = PayloadChecksum(payload);
    WriteValue(stream, checksum);
    stream.write(reinterpret_cast<const char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    stream.flush();
    if (!stream) {
      stream.close();
      std::filesystem::remove(temporary_path);
      return false;
    }
    stream.close();
    if (ReplaceFile(temporary_path, path))
      return true;
    std::error_code error;
    std::filesystem::remove(temporary_path, error);
    return false;
  } catch (...) {
    std::error_code error;
    if (!temporary_path.empty())
      std::filesystem::remove(temporary_path, error);
    return false;
  }
}

bool VulkanPipelineCache::IsRayTracingCreateSuccess(const VkResult result) {
  return result == VK_SUCCESS || result == VK_OPERATION_NOT_DEFERRED_KHR;
}

bool VulkanPipelineCache::ShouldRetryRayTracingSynchronously(const VkResult result) {
  return result != VK_SUCCESS && result != VK_OPERATION_NOT_DEFERRED_KHR && result != VK_ERROR_DEVICE_LOST;
}

bool VulkanPipelineCache::IsIncompleteDeferredOperation(const VkResult result) {
  return result == VK_NOT_READY;
}

bool VulkanPipelineCache::Initialize(const VkDevice device, const VkPhysicalDeviceProperties& properties,
                                     const bool feedback_supported, const bool deferred_host_operations_supported) {
  const std::lock_guard lock(mutex_);
  if (cache_ != VK_NULL_HANDLE)
    return true;
  device_ = device;
  identity_ = MakeIdentity(properties);
  path_ = ResolveCachePath(identity_);
  feedback_supported_ = feedback_supported;
  deferred_host_operations_supported_ = deferred_host_operations_supported;
  std::vector<uint8_t> payload;
  const auto load_result = LoadFile(path_, identity_, payload);
  auto load_source = std::string(GetPipelineCacheLoadResultName(load_result));
  auto initial_bytes = payload.size();
  VkPipelineCacheCreateInfo create_info{VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO};
  create_info.initialDataSize = payload.size();
  create_info.pInitialData = payload.empty() ? nullptr : payload.data();
  auto result = vkCreatePipelineCache(device_, &create_info, nullptr, &cache_);
  if (result != VK_SUCCESS && !payload.empty()) {
    load_source = "driver-rejected";
    initial_bytes = 0;
    create_info.initialDataSize = 0;
    create_info.pInitialData = nullptr;
    result = vkCreatePipelineCache(device_, &create_info, nullptr, &cache_);
  }
  if (result != VK_SUCCESS) {
    cache_ = VK_NULL_HANDLE;
    EVOENGINE_ERROR("Failed to create Vulkan pipeline cache: " + std::to_string(result))
    return false;
  }
  EVOENGINE_LOG("Vulkan pipeline cache initialized: " + load_source + " (" + std::to_string(initial_bytes) +
                " bytes) at " + path_.string())
  return true;
}

void VulkanPipelineCache::Shutdown() noexcept {
  const std::lock_guard lock(mutex_);
  if (device_ == VK_NULL_HANDLE)
    return;
  try {
    SaveLocked();
  } catch (...) {
  }
  if (cache_ != VK_NULL_HANDLE)
    vkDestroyPipelineCache(device_, cache_, nullptr);
  cache_ = VK_NULL_HANDLE;
  device_ = VK_NULL_HANDLE;
}

VkResult VulkanPipelineCache::CreateComputePipeline(const VkComputePipelineCreateInfo& create_info,
                                                    VkPipeline& pipeline, PipelineCreationFeedback& feedback) {
  const std::lock_guard lock(mutex_);
  feedback = {};
  const auto result = CreatePipelineLocked(
      create_info, feedback_supported_,
      [&](const auto& local_info, VkPipeline& output) {
        return vkCreateComputePipelines(device_, cache_, 1, &local_info, nullptr, &output);
      },
      pipeline, feedback);
  return result;
}

VkResult VulkanPipelineCache::CreateGraphicsPipeline(const VkGraphicsPipelineCreateInfo& create_info,
                                                     VkPipeline& pipeline, PipelineCreationFeedback& feedback) {
  const std::lock_guard lock(mutex_);
  feedback = {};
  const auto result = CreatePipelineLocked(
      create_info, feedback_supported_,
      [&](const auto& local_info, VkPipeline& output) {
        return vkCreateGraphicsPipelines(device_, cache_, 1, &local_info, nullptr, &output);
      },
      pipeline, feedback);
  return result;
}

VkResult VulkanPipelineCache::CreateRayTracingPipeline(const VkRayTracingPipelineCreateInfoKHR& create_info,
                                                       VkPipeline& pipeline, PipelineCreationFeedback& feedback) {
  const std::lock_guard lock(mutex_);
  feedback = {};
  feedback.deferred_requested = deferred_host_operations_supported_;
  VkPipelineCreationFeedback raw_feedback{};
  VkPipelineCreationFeedbackCreateInfo feedback_info{};
  auto local_info = create_info;
  if (feedback_supported_) {
    feedback_info.sType = VK_STRUCTURE_TYPE_PIPELINE_CREATION_FEEDBACK_CREATE_INFO;
    feedback_info.pNext = local_info.pNext;
    feedback_info.pPipelineCreationFeedback = &raw_feedback;
    local_info.pNext = &feedback_info;
  }
  const auto start = std::chrono::steady_clock::now();
  VkResult result = VK_ERROR_FEATURE_NOT_PRESENT;
  if (feedback.deferred_requested && vkCreateDeferredOperationKHR && vkCreateRayTracingPipelinesKHR &&
      vkGetDeferredOperationMaxConcurrencyKHR && vkDeferredOperationJoinKHR && vkGetDeferredOperationResultKHR &&
      vkDestroyDeferredOperationKHR) {
    VkDeferredOperationKHR operation = VK_NULL_HANDLE;
    const auto operation_result = vkCreateDeferredOperationKHR(device_, nullptr, &operation);
    if (operation_result == VK_SUCCESS) {
      result = vkCreateRayTracingPipelinesKHR(device_, operation, cache_, 1, &local_info, nullptr, &pipeline);
      if (result == VK_OPERATION_DEFERRED_KHR) {
        feedback.deferred_used = true;
        result = JoinDeferredOperation(device_, operation);
      }
      vkDestroyDeferredOperationKHR(device_, operation, nullptr);
      if (ShouldRetryRayTracingSynchronously(result)) {
        if (pipeline != VK_NULL_HANDLE) {
          vkDestroyPipeline(device_, pipeline, nullptr);
          pipeline = VK_NULL_HANDLE;
        }
        feedback.synchronous_fallback = true;
        feedback.fallback_reason = "deferred-create-failed:" + std::to_string(result);
        raw_feedback = {};
        result = vkCreateRayTracingPipelinesKHR(device_, VK_NULL_HANDLE, cache_, 1, &local_info, nullptr, &pipeline);
      }
    } else {
      feedback.synchronous_fallback = true;
      feedback.fallback_reason = "deferred-operation-create-failed:" + std::to_string(operation_result);
      result = vkCreateRayTracingPipelinesKHR(device_, VK_NULL_HANDLE, cache_, 1, &local_info, nullptr, &pipeline);
    }
  } else {
    feedback.synchronous_fallback = feedback.deferred_requested;
    feedback.fallback_reason = feedback.deferred_requested ? "deferred-functions-unavailable" : "unsupported";
    result = vkCreateRayTracingPipelinesKHR(device_, VK_NULL_HANDLE, cache_, 1, &local_info, nullptr, &pipeline);
  }
  feedback.result = IsRayTracingCreateSuccess(result) ? VK_SUCCESS : result;
  feedback.wall_milliseconds =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  PopulateFeedback(raw_feedback, feedback_supported_, feedback);
  return feedback.result;
}

bool VulkanPipelineCache::SaveLocked() {
  if (cache_ == VK_NULL_HANDLE)
    return false;
  size_t size = 0;
  auto result = vkGetPipelineCacheData(device_, cache_, &size, nullptr);
  if (result != VK_SUCCESS || size == 0 || size > kMaxCacheFileBytes - kFileHeaderBytes) {
    return false;
  }
  std::vector<uint8_t> payload;
  for (uint32_t attempt = 0; attempt < 3; ++attempt) {
    payload.resize(size);
    result = vkGetPipelineCacheData(device_, cache_, &size, payload.data());
    if (result == VK_SUCCESS) {
      payload.resize(size);
      break;
    }
    if (result != VK_INCOMPLETE || size > kMaxCacheFileBytes - kFileHeaderBytes) {
      return false;
    }
    size = 0;
    result = vkGetPipelineCacheData(device_, cache_, &size, nullptr);
    if (result != VK_SUCCESS || size == 0 || size > kMaxCacheFileBytes - kFileHeaderBytes) {
      return false;
    }
  }
  if (result != VK_SUCCESS || !PublishFile(path_, identity_, payload)) {
    return false;
  }
  EVOENGINE_LOG("Vulkan pipeline cache saved (" + std::to_string(payload.size()) + " bytes) at " + path_.string())
  return true;
}
