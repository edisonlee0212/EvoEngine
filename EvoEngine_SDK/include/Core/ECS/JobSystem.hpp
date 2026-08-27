#pragma once

#include "TaskRuntime.hpp"

namespace evo_engine {

using JobHandle = TaskHandle;
using JobGroup = TaskGroup;
using JobExecutorType = TaskExecutorType;
using JobPriority = TaskPriority;
using JobThreadAffinity = ThreadAffinity;
using JobOptions = TaskOptions;
using JobRuntimeSettings = TaskRuntimeSettings;
using JobRuntimeStats = TaskRuntimeStats;
using JobCancellationToken = CancellationToken;
using JobCancellationSource = CancellationSource;

/**
 * @brief Backward-compatible job system facade over TaskRuntime.
 *
 * Existing code can keep using JobSystem and Jobs while new code targets the generic TaskRuntime executor model.
 */
class EVOENGINE_API JobSystem : public TaskRuntime {
 public:
  void StopAllWorkers();
  [[nodiscard]] size_t IdleWorkerSize() const;
  void OnDestroy();
  void ResizeWorker(size_t worker_size);
  [[nodiscard]] size_t GetWorkerSize() const;

  [[nodiscard]] JobHandle PushJob(const std::vector<JobHandle>& dependencies, std::function<void()>&& func);
  [[nodiscard]] JobHandle PushJob(const std::vector<JobHandle>& dependencies, const JobOptions& options,
                                  std::function<void()>&& func);
  void ExecuteJob(const JobHandle& job_handle);
  void Wait(const JobHandle& job_handle);
};

}  // namespace evo_engine
