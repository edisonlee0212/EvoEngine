#include "GpuService.hpp"

#include "ApplicationContext.hpp"
#include "Platform.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>

using namespace evo_engine;

namespace {
std::unique_ptr<ApplicationContextScope> CreateApplicationContextScope(Application* application) {
  if (ApplicationContext::TryGet() || !application) {
    return {};
  }
  return std::make_unique<ApplicationContextScope>(*application);
}

class ImmediateSubmitProgressScope {
  std::atomic<bool>& flag_;

 public:
  explicit ImmediateSubmitProgressScope(std::atomic<bool>& flag) : flag_(flag) {
    bool expected = false;
    if (!flag_.compare_exchange_strong(expected, true)) {
      throw std::runtime_error("Nested immediate submit is not supported.");
    }
  }

  ~ImmediateSubmitProgressScope() {
    flag_.store(false);
  }

  ImmediateSubmitProgressScope(const ImmediateSubmitProgressScope&) = delete;
  ImmediateSubmitProgressScope& operator=(const ImmediateSubmitProgressScope&) = delete;
};
}  // namespace

GpuService::~GpuService() {
  Shutdown();
}

bool GpuService::CanRunGpuWork() const {
  const auto state = lifecycle_state_.load();
  return state == LifecycleState::Running || state == LifecycleState::Draining;
}

void GpuService::EnsureAcceptingSubmissions() const {
  if (lifecycle_state_.load() != LifecycleState::Running) {
    throw std::runtime_error("GpuService is not accepting new work.");
  }
}

void GpuService::EnsureGpuWorkCanRun() const {
  if (!CanRunGpuWork()) {
    throw std::runtime_error("GpuService is not initialized.");
  }
}

void GpuService::Initialize() {
  const auto state = lifecycle_state_.load();
  if (state == LifecycleState::Running) {
    return;
  }
  if (state == LifecycleState::Draining) {
    throw std::runtime_error("GpuService cannot initialize while draining.");
  }
  if (!Platform::Initialized()) {
    throw std::runtime_error("GpuService requires an initialized Platform.");
  }
  owner_application_ = ApplicationContext::TryGet();

  std::lock_guard lock(immediate_submit_mutex_);
  if (lifecycle_state_.load() == LifecycleState::Running) {
    return;
  }

  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pool_info.queueFamilyIndex = Platform::GetGraphicsAndComputeQueueFamilyIndex();
  Platform::CheckVk(vkCreateCommandPool(Platform::GetVkDevice(), &pool_info, nullptr, &command_pool_));

  VkCommandBufferAllocateInfo allocate_info{};
  allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocate_info.commandPool = command_pool_;
  allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocate_info.commandBufferCount = 1;
  Platform::CheckVk(vkAllocateCommandBuffers(Platform::GetVkDevice(), &allocate_info, &immediate_command_buffer_));
  lifecycle_state_.store(LifecycleState::Running);
}

void GpuService::Shutdown() {
  const auto state = lifecycle_state_.load();
  if (state == LifecycleState::Uninitialized || state == LifecycleState::Stopped) {
    return;
  }

  if (state == LifecycleState::Running) {
    lifecycle_state_.store(LifecycleState::Draining);
  }

  try {
    WaitIdleInternal(true);
  } catch (...) {
  }

  {
    std::lock_guard lock(staging_mutex_);
    for (auto& staging_buffer : available_staging_buffers_) {
      DestroyStagingBuffer(staging_buffer);
    }
    available_staging_buffers_.clear();
    staging_work_.clear();
    pending_staging_size_ = 0;
    pooled_staging_size_ = 0;
  }

  {
    std::lock_guard lock(immediate_submit_mutex_);
    const auto vk_device = Platform::GetVkDevice();
    if (immediate_command_buffer_ != VK_NULL_HANDLE) {
      vkFreeCommandBuffers(vk_device, command_pool_, 1, &immediate_command_buffer_);
      immediate_command_buffer_ = VK_NULL_HANDLE;
    }
    if (command_pool_ != VK_NULL_HANDLE) {
      vkDestroyCommandPool(vk_device, command_pool_, nullptr);
      command_pool_ = VK_NULL_HANDLE;
    }
  }

  {
    std::lock_guard lock(in_flight_mutex_);
    in_flight_work_.clear();
  }
  lifecycle_state_.store(LifecycleState::Stopped);
  owner_application_ = nullptr;
}

bool GpuService::Initialized() const {
  return lifecycle_state_.load() == LifecycleState::Running;
}

GpuService::LifecycleState GpuService::GetLifecycleState() const {
  return lifecycle_state_.load();
}

bool GpuService::IsGpuThread() const {
  const auto context_scope = CreateApplicationContextScope(owner_application_);
  return Jobs::IsExecutorThread(JobExecutorType::Gpu);
}

GpuWorkHandle GpuService::EnqueueInternal(const GpuWorkOptions& options, std::function<void()> action,
                                          const bool allow_draining) {
  if (allow_draining) {
    EnsureGpuWorkCanRun();
  } else {
    EnsureAcceptingSubmissions();
  }

  JobOptions job_options;
  job_options.executor = JobExecutorType::Gpu;
  job_options.affinity = JobThreadAffinity::Gpu;
  job_options.priority = options.priority;
  job_options.debug_name = options.debug_name.empty() ? "GpuService::Enqueue" : options.debug_name;
  const auto context_scope = CreateApplicationContextScope(owner_application_);
  auto handle = Jobs::Run(job_options, std::move(action));
  if (!handle.Valid()) {
    throw std::runtime_error("Failed to schedule GPU work.");
  }
  TrackInFlight(handle);
  Jobs::Execute(handle);
  return handle;
}

GpuWorkHandle GpuService::Enqueue(const GpuWorkOptions& options, std::function<void()> action) {
  return EnqueueInternal(options, std::move(action), false);
}

GpuWorkHandle GpuService::Enqueue(std::function<void()> action) {
  GpuWorkOptions options;
  return Enqueue(options, std::move(action));
}

GpuWorkHandle GpuService::EnqueueStaging(const size_t staging_size, const GpuWorkOptions& options,
                                         std::function<void()> action) {
  const auto reserved_size = EstimateStagingAllocationSize(staging_size);
  ReserveStagingBudget(reserved_size);
  try {
    auto handle = EnqueueInternal(options, std::move(action), false);
    {
      std::lock_guard lock(staging_mutex_);
      staging_work_.push_back({handle, reserved_size});
    }
    return handle;
  } catch (...) {
    ReleaseStagingBudget(reserved_size);
    throw;
  }
}

GpuWorkHandle GpuService::EnqueueImmediate(const GpuWorkOptions& options,
                                           std::function<void(VkCommandBuffer vk_command_buffer)> action) {
  auto shared_action = std::make_shared<std::function<void(VkCommandBuffer vk_command_buffer)>>(std::move(action));
  return Enqueue(options, [this, shared_action]() {
    SubmitImmediateOnGpuThread(*shared_action);
  });
}

GpuWorkHandle GpuService::EnqueueImmediate(std::function<void(VkCommandBuffer vk_command_buffer)> action) {
  GpuWorkOptions options;
  options.debug_name = "GpuService::EnqueueImmediate";
  return EnqueueImmediate(options, std::move(action));
}

void GpuService::SubmitImmediate(std::function<void(VkCommandBuffer vk_command_buffer)> action) {
  EnsureAcceptingSubmissions();
  if (IsGpuThread()) {
    SubmitImmediateOnGpuThread(action);
    return;
  }
  auto handle = EnqueueImmediate(std::move(action));
  Wait(handle);
}

void GpuService::Wait(const GpuWorkHandle& handle) {
  const auto context_scope = CreateApplicationContextScope(owner_application_);
  Jobs::Wait(handle);
  ClearCompletedInFlight();
}

void GpuService::WaitIdle() {
  WaitIdleInternal(false);
}

void GpuService::WaitIdleInternal(const bool allow_draining) {
  if (allow_draining) {
    EnsureGpuWorkCanRun();
  } else {
    EnsureAcceptingSubmissions();
  }

  if (IsGpuThread()) {
    Platform::GetImmediateSubmitQueue()->WaitIdle();
    ClearCompletedInFlight();
    return;
  }

  GpuWorkOptions options;
  options.priority = TaskPriority::Low;
  options.debug_name = "GpuService::WaitIdle";
  auto handle = EnqueueInternal(
      options,
      []() {
        Platform::GetImmediateSubmitQueue()->WaitIdle();
      },
      allow_draining);
  const auto context_scope = CreateApplicationContextScope(owner_application_);
  Jobs::Wait(handle);

  {
    std::lock_guard lock(in_flight_mutex_);
    in_flight_work_.clear();
  }
  {
    std::lock_guard lock(staging_mutex_);
    staging_work_.clear();
    pending_staging_size_ = 0;
  }
}

void GpuService::SubmitImmediateOnGpuThread(const std::function<void(VkCommandBuffer vk_command_buffer)>& action) {
  EnsureGpuWorkCanRun();
  if (!IsGpuThread()) {
    throw std::runtime_error("Immediate GPU submit must run on the GPU executor.");
  }

  const ImmediateSubmitProgressScope immediate_submit_progress_scope(immediate_submit_in_progress_);
  std::lock_guard lock(immediate_submit_mutex_);
  if (immediate_command_buffer_ == VK_NULL_HANDLE) {
    throw std::runtime_error("GpuService immediate command buffer is unavailable.");
  }

  const auto vk_device = Platform::GetVkDevice();
  Platform::CheckVk(vkResetCommandBuffer(immediate_command_buffer_, 0));

  VkCommandBufferBeginInfo begin_info{};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  Platform::CheckVk(vkBeginCommandBuffer(immediate_command_buffer_, &begin_info));
  try {
    action(immediate_command_buffer_);
  } catch (...) {
    vkResetCommandBuffer(immediate_command_buffer_, 0);
    throw;
  }
  Platform::CheckVk(vkEndCommandBuffer(immediate_command_buffer_));

  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &immediate_command_buffer_;

  VkFenceCreateInfo fence_info{};
  fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

  VkFence fence = VK_NULL_HANDLE;
  Platform::CheckVk(vkCreateFence(vk_device, &fence_info, nullptr, &fence));
  VkResult submit_result;
  {
    const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
    submit_result = vkQueueSubmit(Platform::GetImmediateSubmitQueue()->GetVkQueue(), 1, &submit_info, fence);
  }
  if (submit_result != VK_SUCCESS) {
    vkDestroyFence(vk_device, fence, nullptr);
    throw std::runtime_error("Failed to submit immediate GPU work! Error code: " + std::to_string(submit_result));
  }
  Platform::CheckVk(vkWaitForFences(vk_device, 1, &fence, VK_TRUE, UINT64_MAX));
  vkDestroyFence(vk_device, fence, nullptr);
  Platform::CheckVk(vkResetCommandBuffer(immediate_command_buffer_, 0));
}

void GpuService::TrackInFlight(const GpuWorkHandle& handle) {
  if (!handle.Valid()) {
    return;
  }
  std::lock_guard lock(in_flight_mutex_);
  in_flight_work_.emplace_back(handle);
}

void GpuService::ClearCompletedInFlight() {
  const auto context_scope = CreateApplicationContextScope(owner_application_);
  std::lock_guard lock(in_flight_mutex_);
  in_flight_work_.erase(std::remove_if(in_flight_work_.begin(), in_flight_work_.end(),
                                       [](const GpuWorkHandle& handle) {
                                         return !handle.Valid() || Jobs::IsCompleted(handle);
                                       }),
                        in_flight_work_.end());
}

void GpuService::ReserveStagingBudget(const size_t size) {
  if (size == 0) {
    return;
  }

  std::unique_lock lock(staging_mutex_);
  PruneCompletedStagingWorkLocked();
  while (pending_staging_size_ + size > max_in_flight_staging_size_ && !staging_work_.empty()) {
    auto record = staging_work_.front();
    staging_work_.pop_front();
    pending_staging_size_ = pending_staging_size_ > record.size ? pending_staging_size_ - record.size : 0;
    lock.unlock();
    Wait(record.handle);
    lock.lock();
    PruneCompletedStagingWorkLocked();
  }
  pending_staging_size_ += size;
}

void GpuService::ReleaseStagingBudget(const size_t size) {
  std::lock_guard lock(staging_mutex_);
  pending_staging_size_ = pending_staging_size_ > size ? pending_staging_size_ - size : 0;
}

void GpuService::PruneCompletedStagingWorkLocked() {
  const auto context_scope = CreateApplicationContextScope(owner_application_);
  while (!staging_work_.empty()) {
    const auto& record = staging_work_.front();
    if (record.handle.Valid() && !Jobs::IsCompleted(record.handle)) {
      break;
    }
    pending_staging_size_ = pending_staging_size_ > record.size ? pending_staging_size_ - record.size : 0;
    staging_work_.pop_front();
  }
}

size_t GpuService::EstimateStagingAllocationSize(const size_t size) const {
  if (size == 0) {
    return 0;
  }
  if (size > max_in_flight_staging_size_) {
    return size;
  }
  return std::max(size, staging_block_size_);
}

GpuStagingBuffer GpuService::CreateStagingBuffer(const size_t size, const bool random_access) const {
  GpuStagingBuffer staging_buffer;
  staging_buffer.size = EstimateStagingAllocationSize(size);
  staging_buffer.random_access = random_access;

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = staging_buffer.size;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
#if ENABLE_EXTERNAL_MEMORY
  VkExternalMemoryBufferCreateInfo external_memory_info{};
  external_memory_info.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO;
#  ifdef _WIN64
  external_memory_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#  else
  external_memory_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#  endif
  buffer_create_info.pNext = &external_memory_info;
#endif

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  allocation_create_info.flags = random_access ? VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT
                                               : VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;

  Platform::CheckVk(vmaCreateBuffer(Platform::GetVmaAllocator(), &buffer_create_info, &allocation_create_info,
                                    &staging_buffer.vk_buffer, &staging_buffer.vma_allocation, nullptr));
  return staging_buffer;
}

void GpuService::DestroyStagingBuffer(GpuStagingBuffer& staging_buffer) const {
  if (staging_buffer.vk_buffer != VK_NULL_HANDLE && staging_buffer.vma_allocation != VK_NULL_HANDLE &&
      Platform::Initialized() && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vmaDestroyBuffer(Platform::GetVmaAllocator(), staging_buffer.vk_buffer, staging_buffer.vma_allocation);
  }
  staging_buffer.vk_buffer = VK_NULL_HANDLE;
  staging_buffer.vma_allocation = VK_NULL_HANDLE;
  staging_buffer.size = 0;
}

GpuStagingBuffer GpuService::AcquireStagingBuffer(const size_t size, const bool random_access) {
  EnsureGpuWorkCanRun();
  if (!IsGpuThread()) {
    throw std::runtime_error("Staging buffers must be acquired on the GPU executor.");
  }

  {
    std::lock_guard lock(staging_mutex_);
    const auto search = std::find_if(available_staging_buffers_.begin(), available_staging_buffers_.end(),
                                     [size, random_access](const GpuStagingBuffer& candidate) {
                                       return candidate.random_access == random_access && candidate.size >= size;
                                     });
    if (search != available_staging_buffers_.end()) {
      auto staging_buffer = *search;
      pooled_staging_size_ =
          pooled_staging_size_ > staging_buffer.size ? pooled_staging_size_ - staging_buffer.size : 0;
      available_staging_buffers_.erase(search);
      return staging_buffer;
    }
  }

  return CreateStagingBuffer(size, random_access);
}

void GpuService::ReleaseStagingBuffer(GpuStagingBuffer staging_buffer) {
  if (staging_buffer.vk_buffer == VK_NULL_HANDLE || staging_buffer.vma_allocation == VK_NULL_HANDLE) {
    return;
  }

  if (CanRunGpuWork() && IsGpuThread() && staging_buffer.size <= max_in_flight_staging_size_) {
    std::lock_guard lock(staging_mutex_);
    if (pooled_staging_size_ + staging_buffer.size <= max_in_flight_staging_size_) {
      pooled_staging_size_ += staging_buffer.size;
      available_staging_buffers_.emplace_back(staging_buffer);
      return;
    }
  }

  DestroyStagingBuffer(staging_buffer);
}
