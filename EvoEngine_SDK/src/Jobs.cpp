#include "Jobs.hpp"

#include "ApplicationContext.hpp"
#include "Console.hpp"

#include <utility>

using namespace evo_engine;

namespace {
std::function<void()> BindApplicationContext(std::function<void()> func) {
  auto* application = ApplicationContext::TryGet();
  return [application, func = std::move(func)]() mutable {
    if (application) {
      const ApplicationContextScope scope(*application);
      func();
      return;
    }
    func();
  };
}
}  // namespace

size_t Jobs::GetWorkerSize() {
  const auto& jobs = GetInstance();
  return jobs.job_system_.GetWorkerSize();
}

void Jobs::Initialize(const size_t worker_size) {
  auto& jobs = GetInstance();
  JobRuntimeSettings settings;
  settings.worker_thread_size = std::max<size_t>(1, worker_size);
  settings.asset_io_thread_size = 1;
  settings.gpu_thread_size = 1;
  settings.render_thread_size = 1;
  settings.background_thread_size = 1;
  jobs.job_system_.Initialize(settings);
}

JobRuntimeStats Jobs::GetRuntimeStats() {
  const auto& jobs = GetInstance();
  return jobs.job_system_.GetStats();
}

bool Jobs::IsMainThread() {
  const auto& jobs = GetInstance();
  return jobs.job_system_.IsMainThread();
}

bool Jobs::IsExecutorThread(const JobExecutorType executor) {
  const auto& jobs = GetInstance();
  return jobs.job_system_.IsExecutorThread(executor);
}

size_t Jobs::ExecuteMainThreadJobs(const size_t max_task_size) {
  auto& jobs = GetInstance();
  return jobs.job_system_.RunReadyMainThreadTasks(max_task_size);
}

void Jobs::RunParallelFor(const size_t size, const std::function<void(size_t i)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob({}, BindApplicationContext(work)));
  }
  Wait(Combine(job_handles));
}

void Jobs::RunParallelFor(const size_t size, const std::function<void(size_t, size_t)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i, thread_index);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i, thread_index);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob({}, BindApplicationContext(work)));
  }
  Wait(Combine(job_handles));
}

JobHandle Jobs::ScheduleParallelFor(const size_t size, const std::function<void(size_t)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob({}, BindApplicationContext(work)));
  }
  return Combine(job_handles);
}

JobHandle Jobs::ScheduleParallelFor(const size_t size, const std::function<void(size_t, size_t)>& func,
                                    size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i, thread_index);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i, thread_index);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob({}, BindApplicationContext(work)));
  }
  return Combine(job_handles);
}

void Jobs::RunParallelFor(const std::vector<JobHandle>& dependencies, const size_t size,
                          const std::function<void(size_t)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob(dependencies, BindApplicationContext(work)));
  }
  Wait(Combine(job_handles));
}

void Jobs::RunParallelFor(const std::vector<JobHandle>& dependencies, const size_t size,
                          const std::function<void(size_t, size_t)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i, thread_index);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i, thread_index);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob(dependencies, BindApplicationContext(work)));
  }
  Wait(Combine(job_handles));
}

JobHandle Jobs::ScheduleParallelFor(const std::vector<JobHandle>& dependencies, const size_t size,
                                    const std::function<void(size_t)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob(dependencies, BindApplicationContext(work)));
  }
  return Combine(job_handles);
}

JobHandle Jobs::ScheduleParallelFor(const std::vector<JobHandle>& dependencies, const size_t size,
                                    const std::function<void(size_t, size_t)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
  worker_size = std::max<size_t>(1, worker_size);
  const auto thread_load = size / worker_size;
  const auto load_reminder = size % worker_size;
  std::vector<JobHandle> job_handles;
  for (size_t thread_index = 0; thread_index < worker_size; thread_index++) {
    const auto work = [=] {
      for (size_t i = thread_index * thread_load; i < (thread_index + 1) * thread_load; i++) {
        func(i, thread_index);
      }
      if (thread_index < load_reminder) {
        const size_t i = thread_index + worker_size * thread_load;
        func(i, thread_index);
      }
    };
    job_handles.emplace_back(jobs.job_system_.PushJob(dependencies, BindApplicationContext(work)));
  }
  return Combine(job_handles);
}

JobHandle Jobs::Run(const std::vector<JobHandle>& dependencies, const std::function<void()>& func) {
  auto& jobs = GetInstance();
  return jobs.job_system_.PushJob(dependencies, BindApplicationContext(std::function<void()>(func)));
}

JobHandle Jobs::Run(const std::vector<JobHandle>& dependencies, const JobOptions& options,
                    const std::function<void()>& func) {
  auto& jobs = GetInstance();
  return jobs.job_system_.PushJob(dependencies, options, BindApplicationContext(std::function<void()>(func)));
}

JobHandle Jobs::Run(const std::function<void()>& func) {
  auto& jobs = GetInstance();
  return jobs.job_system_.PushJob({}, BindApplicationContext(std::function<void()>(func)));
}

JobHandle Jobs::Run(const JobOptions& options, const std::function<void()>& func) {
  auto& jobs = GetInstance();
  return jobs.job_system_.PushJob({}, options, BindApplicationContext(std::function<void()>(func)));
}

JobHandle Jobs::RunOnMainThread(const std::function<void()>& func) {
  JobOptions options;
  options.executor = JobExecutorType::MainThread;
  options.affinity = JobThreadAffinity::MainThread;
  options.debug_name = "Jobs::RunOnMainThread";
  return Run(options, func);
}

JobHandle Jobs::RunOnAssetIoThread(const std::function<void()>& func) {
  JobOptions options;
  options.executor = JobExecutorType::AssetIo;
  options.affinity = JobThreadAffinity::AssetIo;
  options.debug_name = "Jobs::RunOnAssetIoThread";
  return Run(options, func);
}

JobHandle Jobs::RunOnRenderThread(const std::function<void()>& func) {
  JobOptions options;
  options.executor = JobExecutorType::Render;
  options.affinity = JobThreadAffinity::Render;
  options.debug_name = "Jobs::RunOnRenderThread";
  return Run(options, func);
}

JobHandle Jobs::RunOnGpuThread(const std::function<void()>& func) {
  JobOptions options;
  options.executor = JobExecutorType::Gpu;
  options.affinity = JobThreadAffinity::Gpu;
  options.debug_name = "Jobs::RunOnGpuThread";
  return Run(options, func);
}

JobHandle Jobs::RunOnBackgroundThread(const std::function<void()>& func) {
  JobOptions options;
  options.executor = JobExecutorType::Background;
  options.affinity = JobThreadAffinity::Background;
  options.debug_name = "Jobs::RunOnBackgroundThread";
  return Run(options, func);
}

JobHandle Jobs::Combine(const std::vector<JobHandle>& dependencies) {
  auto& jobs = GetInstance();
  return jobs.job_system_.PushJob(dependencies, BindApplicationContext([]() {
                                  }));
}

JobHandle Jobs::Combine(const JobGroup& job_group) {
  return Combine(job_group.GetHandles());
}

void Jobs::Execute(const JobHandle& job_handle) {
  auto& jobs = GetInstance();
  if (!job_handle.Valid())
    return;
  jobs.job_system_.ExecuteJob(job_handle);
}

void Jobs::Wait(const JobHandle& job_handle) {
  auto& jobs = GetInstance();
  if (!job_handle.Valid())
    return;
  jobs.job_system_.ExecuteJob(job_handle);
  jobs.job_system_.Wait(job_handle);
}

bool Jobs::IsCompleted(const JobHandle& job_handle) {
  const auto& jobs = GetInstance();
  return jobs.job_system_.IsCompleted(job_handle);
}

void Jobs::OnDestroy() {
  auto& jobs = GetInstance();
  jobs.job_system_.OnDestroy();
}
