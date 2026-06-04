#pragma once

#include "Jobs.hpp"
#include "vk_mem_alloc.h"
#include "volk.h"

#include <atomic>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace evo_engine {

class Application;

using GpuWorkHandle = JobHandle;

struct GpuWorkOptions {
  TaskPriority priority = TaskPriority::Normal;
  std::string debug_name;
};

struct GpuStagingBuffer {
  VkBuffer vk_buffer = VK_NULL_HANDLE;
  VmaAllocation vma_allocation = VK_NULL_HANDLE;
  size_t size = 0;
  bool random_access = false;
};

class GpuService final {
 public:
  enum class LifecycleState { Uninitialized, Running, Draining, Stopped };

  static constexpr size_t default_staging_block_size = 16ull * 1024ull * 1024ull;
  static constexpr size_t default_max_in_flight_staging_size = 64ull * 1024ull * 1024ull;

 private:
  struct StagingWorkRecord {
    GpuWorkHandle handle;
    size_t size = 0;
  };

  VkCommandPool command_pool_ = VK_NULL_HANDLE;
  VkCommandBuffer immediate_command_buffer_ = VK_NULL_HANDLE;
  Application* owner_application_ = nullptr;
  std::atomic<LifecycleState> lifecycle_state_{LifecycleState::Uninitialized};
  mutable std::mutex immediate_submit_mutex_;
  std::atomic<bool> immediate_submit_in_progress_{false};

  mutable std::mutex in_flight_mutex_;
  std::vector<GpuWorkHandle> in_flight_work_;

  mutable std::mutex staging_mutex_;
  std::vector<GpuStagingBuffer> available_staging_buffers_;
  std::deque<StagingWorkRecord> staging_work_;
  size_t pending_staging_size_ = 0;
  size_t pooled_staging_size_ = 0;
  size_t staging_block_size_ = default_staging_block_size;
  size_t max_in_flight_staging_size_ = default_max_in_flight_staging_size;

  [[nodiscard]] bool CanRunGpuWork() const;
  void EnsureAcceptingSubmissions() const;
  void EnsureGpuWorkCanRun() const;
  [[nodiscard]] GpuWorkHandle EnqueueInternal(const GpuWorkOptions& options, std::function<void()> action,
                                              bool allow_draining);
  void SubmitImmediateOnGpuThread(const std::function<void(VkCommandBuffer vk_command_buffer)>& action);
  void WaitIdleInternal(bool allow_draining);
  void TrackInFlight(const GpuWorkHandle& handle);
  void ClearCompletedInFlight();
  void ReserveStagingBudget(size_t size);
  void ReleaseStagingBudget(size_t size);
  void PruneCompletedStagingWorkLocked();
  [[nodiscard]] size_t EstimateStagingAllocationSize(size_t size) const;
  [[nodiscard]] GpuStagingBuffer CreateStagingBuffer(size_t size, bool random_access) const;
  void DestroyStagingBuffer(GpuStagingBuffer& staging_buffer) const;

 public:
  GpuService() = default;
  ~GpuService();
  GpuService(const GpuService&) = delete;
  GpuService& operator=(const GpuService&) = delete;

  void Initialize();
  void Shutdown();
  [[nodiscard]] bool Initialized() const;
  [[nodiscard]] LifecycleState GetLifecycleState() const;
  [[nodiscard]] bool IsGpuThread() const;

  [[nodiscard]] GpuWorkHandle Enqueue(const GpuWorkOptions& options, std::function<void()> action);
  [[nodiscard]] GpuWorkHandle Enqueue(std::function<void()> action);
  [[nodiscard]] GpuWorkHandle EnqueueStaging(size_t staging_size, const GpuWorkOptions& options,
                                             std::function<void()> action);
  [[nodiscard]] GpuWorkHandle EnqueueImmediate(const GpuWorkOptions& options,
                                               std::function<void(VkCommandBuffer vk_command_buffer)> action);
  [[nodiscard]] GpuWorkHandle EnqueueImmediate(std::function<void(VkCommandBuffer vk_command_buffer)> action);

  void SubmitImmediate(std::function<void(VkCommandBuffer vk_command_buffer)> action);
  void Wait(const GpuWorkHandle& handle);
  void WaitIdle();

  [[nodiscard]] GpuStagingBuffer AcquireStagingBuffer(size_t size, bool random_access);
  void ReleaseStagingBuffer(GpuStagingBuffer staging_buffer);
};

}  // namespace evo_engine
