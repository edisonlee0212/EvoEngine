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
  jobs.job_system_.ResizeWorker(worker_size);
}

void Jobs::RunParallelFor(const size_t size, const std::function<void(size_t i)>& func, size_t worker_size) {
  auto& jobs = GetInstance();
  if (worker_size == 0)
    worker_size = GetWorkerSize();
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

JobHandle Jobs::Run(const std::function<void()>& func) {
  auto& jobs = GetInstance();
  return jobs.job_system_.PushJob({}, BindApplicationContext(std::function<void()>(func)));
}

JobHandle Jobs::Combine(const std::vector<JobHandle>& dependencies) {
  auto& jobs = GetInstance();
  return jobs.job_system_.PushJob(dependencies, BindApplicationContext([]() {
                                  }));
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

void Jobs::OnDestroy() {
  auto& jobs = GetInstance();
  jobs.job_system_.OnDestroy();
}
