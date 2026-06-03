#include "JobSystem.hpp"

using namespace evo_engine;

void JobSystem::StopAllWorkers() {
  Shutdown();
}

size_t JobSystem::IdleWorkerSize() const {
  const auto stats = GetStats();
  return stats.worker_thread_size > stats.running_task_size ? stats.worker_thread_size - stats.running_task_size : 0;
}

void JobSystem::OnDestroy() {
  Shutdown();
}

void JobSystem::ResizeWorker(const size_t worker_size) {
  TaskRuntime::ResizeWorker(worker_size);
}

size_t JobSystem::GetWorkerSize() const {
  return TaskRuntime::GetWorkerSize();
}

JobHandle JobSystem::PushJob(const std::vector<JobHandle>& dependencies, std::function<void()>&& func) {
  return Schedule(dependencies, std::move(func));
}

JobHandle JobSystem::PushJob(const std::vector<JobHandle>& dependencies, const JobOptions& options,
                             std::function<void()>&& func) {
  return Schedule(dependencies, options, std::move(func));
}

void JobSystem::ExecuteJob(const JobHandle& job_handle) {
  Execute(job_handle);
}

void JobSystem::Wait(const JobHandle& job_handle) {
  TaskRuntime::Wait(job_handle);
}
