#pragma once
#include "ISingleton.hpp"
#include "JobSystem.hpp"
namespace evo_engine {
class Jobs final {
  EVOENGINE_SINGLETON_INSTANCE(Jobs)
  JobSystem job_system_;

 public:
  static size_t GetWorkerSize();
  static void Initialize(size_t worker_size);
  static void RunParallelFor(size_t size, const std::function<void(size_t i)>& func, size_t worker_size = 0);
  static void RunParallelFor(size_t size, const std::function<void(size_t i, size_t worker_index)>& func,
                             size_t worker_size = 0);
  static JobHandle ScheduleParallelFor(size_t size, const std::function<void(size_t i)>& func, size_t worker_size = 0);
  static JobHandle ScheduleParallelFor(size_t size, const std::function<void(size_t i, size_t worker_index)>& func,
                                       size_t worker_size = 0);

  static void RunParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                             const std::function<void(size_t i)>& func, size_t worker_size = 0);
  static void RunParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                             const std::function<void(size_t i, size_t worker_index)>& func, size_t worker_size = 0);
  static JobHandle ScheduleParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                                       const std::function<void(size_t i)>& func, size_t worker_size = 0);
  static JobHandle ScheduleParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                                       const std::function<void(size_t i, size_t worker_index)>& func,
                                       size_t worker_size = 0);

  static JobHandle Run(const std::vector<JobHandle>& dependencies, const std::function<void()>& func);
  static JobHandle Run(const std::function<void()>& func);
  static JobHandle Combine(const std::vector<JobHandle>& dependencies);

  static void Execute(const JobHandle& job_handle);
  static void Wait(const JobHandle& job_handle);
};
}  // namespace evo_engine
