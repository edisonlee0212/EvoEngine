#include "TaskRuntime.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <utility>

using namespace evo_engine;

namespace {
thread_local TaskRuntime* g_current_task_runtime = nullptr;
thread_local TaskExecutorType g_current_executor = TaskExecutorType::MainThread;

TaskExecutorType ExecutorFromAffinity(const ThreadAffinity affinity) {
  switch (affinity) {
    case ThreadAffinity::MainThread:
      return TaskExecutorType::MainThread;
    case ThreadAffinity::Worker:
      return TaskExecutorType::Worker;
    case ThreadAffinity::AssetIo:
      return TaskExecutorType::AssetIo;
    case ThreadAffinity::Render:
      return TaskExecutorType::Render;
    case ThreadAffinity::Background:
      return TaskExecutorType::Background;
    case ThreadAffinity::Any:
    default:
      return TaskExecutorType::Worker;
  }
}

const char* ExecutorName(const TaskExecutorType executor) {
  switch (executor) {
    case TaskExecutorType::MainThread:
      return "MainThread";
    case TaskExecutorType::Worker:
      return "Worker";
    case TaskExecutorType::AssetIo:
      return "AssetIo";
    case TaskExecutorType::Render:
      return "Render";
    case TaskExecutorType::Background:
      return "Background";
    default:
      return "Unknown";
  }
}
}  // namespace

CancellationToken::CancellationToken(std::shared_ptr<std::atomic<bool>> state) : state_(std::move(state)) {
}

bool CancellationToken::IsCancellationRequested() const {
  return state_ && state_->load();
}

bool CancellationToken::Valid() const {
  return state_ != nullptr;
}

CancellationToken CancellationSource::GetToken() const {
  return CancellationToken(state_);
}

void CancellationSource::Cancel() const {
  state_->store(true);
}

bool CancellationSource::IsCancellationRequested() const {
  return state_->load();
}

int TaskHandle::GetIndex() const {
  return index_;
}

uint32_t TaskHandle::GetGeneration() const {
  return generation_;
}

bool TaskHandle::Valid() const {
  return index_ >= 0 && generation_ != 0;
}

void TaskGroup::Add(const TaskHandle& handle) {
  if (handle.Valid()) {
    handles_.emplace_back(handle);
  }
}

void TaskGroup::Clear() {
  handles_.clear();
}

bool TaskGroup::Empty() const {
  return handles_.empty();
}

const std::vector<TaskHandle>& TaskGroup::GetHandles() const {
  return handles_;
}

TaskRuntime::TaskRuntime() {
  main_thread_id_ = std::this_thread::get_id();
  main_executor_.type = TaskExecutorType::MainThread;
  main_executor_.name = "MainThread";
  worker_executor_.type = TaskExecutorType::Worker;
  asset_io_executor_.type = TaskExecutorType::AssetIo;
  render_executor_.type = TaskExecutorType::Render;
  background_executor_.type = TaskExecutorType::Background;

  TaskRuntimeSettings settings;
  settings.worker_thread_size = 1;
  settings.asset_io_thread_size = 0;
  settings.render_thread_size = 0;
  settings.background_thread_size = 0;
  Initialize(settings);
}

TaskRuntime::~TaskRuntime() {
  Shutdown();
}

void TaskRuntime::Initialize(const TaskRuntimeSettings& settings) {
  Shutdown();
  shutting_down_ = false;
  running_task_size_ = 0;
  submitted_task_size_ = 0;
  completed_task_size_ = 0;
  failed_task_size_ = 0;
  main_thread_id_ = std::this_thread::get_id();
  StartExecutor(worker_executor_, std::max<size_t>(1, settings.worker_thread_size), "Worker");
  StartExecutor(asset_io_executor_, settings.asset_io_thread_size, "AssetIo");
  StartExecutor(render_executor_, settings.render_thread_size, "Render");
  StartExecutor(background_executor_, settings.background_thread_size, "Background");
}

void TaskRuntime::Shutdown() {
  shutting_down_ = true;
  StopExecutor(worker_executor_);
  StopExecutor(asset_io_executor_);
  StopExecutor(render_executor_);
  StopExecutor(background_executor_);
  {
    std::lock_guard lock(main_executor_.mutex);
    main_executor_.high_priority_tasks.clear();
    main_executor_.normal_priority_tasks.clear();
    main_executor_.low_priority_tasks.clear();
    main_executor_.queued_task_size = 0;
  }
  {
    std::lock_guard lock(task_mutex_);
    tasks_.clear();
    recycled_task_indices_ = {};
  }
  task_finished_cv_.notify_all();
}

void TaskRuntime::ResizeWorker(const size_t worker_size) {
  StopExecutor(worker_executor_);
  StartExecutor(worker_executor_, std::max<size_t>(1, worker_size), "Worker");
}

size_t TaskRuntime::GetWorkerSize() const {
  return GetThreadSize(TaskExecutorType::Worker);
}

size_t TaskRuntime::GetThreadSize(const TaskExecutorType executor) const {
  const auto& executor_state = GetExecutor(executor);
  std::lock_guard lock(executor_state.mutex);
  return executor_state.threads.size();
}

bool TaskRuntime::IsMainThread() const {
  return std::this_thread::get_id() == main_thread_id_;
}

bool TaskRuntime::IsExecutorThread(const TaskExecutorType executor) const {
  return g_current_task_runtime == this && g_current_executor == executor;
}

TaskRuntimeStats TaskRuntime::GetStats() const {
  TaskRuntimeStats stats;
  stats.worker_thread_size = GetThreadSize(TaskExecutorType::Worker);
  stats.asset_io_thread_size = GetThreadSize(TaskExecutorType::AssetIo);
  stats.render_thread_size = GetThreadSize(TaskExecutorType::Render);
  stats.background_thread_size = GetThreadSize(TaskExecutorType::Background);
  stats.running_task_size = running_task_size_.load();
  stats.submitted_task_size = submitted_task_size_.load();
  stats.completed_task_size = completed_task_size_.load();
  stats.failed_task_size = failed_task_size_.load();
  for (const auto executor : {TaskExecutorType::MainThread, TaskExecutorType::Worker, TaskExecutorType::AssetIo,
                              TaskExecutorType::Render, TaskExecutorType::Background}) {
    const auto& executor_state = GetExecutor(executor);
    std::lock_guard lock(executor_state.mutex);
    stats.queued_task_size += executor_state.queued_task_size;
  }
  return stats;
}

TaskHandle TaskRuntime::Schedule(const std::vector<TaskHandle>& dependencies, const TaskOptions& options,
                                 std::function<void()>&& function) {
  if (shutting_down_) {
    return {};
  }

  TaskHandle handle;
  {
    std::lock_guard lock(task_mutex_);
    handle = AllocateTaskLocked();
    auto& task = *tasks_[handle.index_];
    task.state = TaskState::Pending;
    task.dependencies.clear();
    task.dependents.clear();
    task.pending_dependency_size = 0;
    task.scheduled = false;
    task.options = options;
    if (task.options.affinity != ThreadAffinity::Any) {
      task.options.executor = ExecutorFromAffinity(task.options.affinity);
    }
    task.function = std::move(function);
    task.exception = nullptr;

    for (const auto& dependency : dependencies) {
      if (!IsHandleValidLocked(dependency)) {
        continue;
      }
      task.dependencies.emplace_back(dependency);
      auto& dependency_task = *tasks_[dependency.index_];
      if (dependency_task.state != TaskState::Completed) {
        ++task.pending_dependency_size;
        dependency_task.dependents.emplace_back(handle);
      }
    }
  }
  ++submitted_task_size_;
  return handle;
}

TaskHandle TaskRuntime::Schedule(const std::vector<TaskHandle>& dependencies, std::function<void()>&& function) {
  return Schedule(dependencies, TaskOptions{}, std::move(function));
}

TaskHandle TaskRuntime::Schedule(std::function<void()>&& function) {
  return Schedule({}, TaskOptions{}, std::move(function));
}

TaskHandle TaskRuntime::Combine(const std::vector<TaskHandle>& dependencies) {
  return Schedule(dependencies, []() {
  });
}

TaskHandle TaskRuntime::Combine(const TaskGroup& task_group) {
  return Combine(task_group.GetHandles());
}

void TaskRuntime::Execute(const TaskHandle& handle) {
  std::vector<TaskHandle> ready_tasks;
  {
    std::lock_guard lock(task_mutex_);
    if (!IsHandleValidLocked(handle)) {
      return;
    }
    std::vector<int> visited_indices;
    StartTaskGraphLocked(handle, ready_tasks, visited_indices);
  }
  QueueReadyTasks(ready_tasks);
}

void TaskRuntime::Wait(const TaskHandle& handle) {
  if (!handle.Valid()) {
    return;
  }

  Execute(handle);
  std::exception_ptr exception;
  while (true) {
    {
      std::unique_lock lock(task_mutex_);
      if (!IsHandleValidLocked(handle)) {
        return;
      }
      const auto& task = *tasks_[handle.index_];
      if (task.state == TaskState::Completed) {
        exception = task.exception;
        break;
      }
    }

    TaskHandle waiting_task;
    TaskExecutorType waiting_executor = TaskExecutorType::Worker;
    if (TryPopTaskForWaiting(waiting_task, waiting_executor)) {
      RunTask(waiting_task, waiting_executor);
      continue;
    }

    std::unique_lock lock(task_mutex_);
    task_finished_cv_.wait_for(lock, std::chrono::milliseconds(1));
  }

  {
    std::lock_guard lock(task_mutex_);
    std::vector<TaskHandle> task_tree;
    CollectTaskTreeLocked(handle, task_tree);
    for (const auto& task_handle : task_tree) {
      if (CanRecycleTaskLocked(task_handle)) {
        RecycleTaskLocked(task_handle);
      }
    }
  }

  if (exception) {
    std::rethrow_exception(exception);
  }
}

bool TaskRuntime::IsCompleted(const TaskHandle& handle) const {
  std::lock_guard lock(task_mutex_);
  return IsHandleValidLocked(handle) && tasks_[handle.index_]->state == TaskState::Completed;
}

std::exception_ptr TaskRuntime::GetException(const TaskHandle& handle) const {
  std::lock_guard lock(task_mutex_);
  if (!IsHandleValidLocked(handle)) {
    return nullptr;
  }
  return tasks_[handle.index_]->exception;
}

size_t TaskRuntime::RunReadyMainThreadTasks(const size_t max_task_size) {
  if (!IsMainThread()) {
    return 0;
  }

  size_t executed_task_size = 0;
  while (max_task_size == 0 || executed_task_size < max_task_size) {
    TaskHandle handle;
    if (!TryPopTask(main_executor_, handle)) {
      break;
    }
    RunTask(handle, TaskExecutorType::MainThread);
    ++executed_task_size;
  }
  return executed_task_size;
}

bool TaskRuntime::IsHandleValidLocked(const TaskHandle& handle) const {
  return handle.index_ >= 0 && static_cast<size_t>(handle.index_) < tasks_.size() && tasks_[handle.index_] &&
         tasks_[handle.index_]->generation == handle.generation_ && tasks_[handle.index_]->state != TaskState::Recycled;
}

bool TaskRuntime::ExecutorHasTasks(const ExecutorState& executor) {
  return !executor.high_priority_tasks.empty() || !executor.normal_priority_tasks.empty() ||
         !executor.low_priority_tasks.empty();
}

std::deque<TaskHandle>& TaskRuntime::GetQueueForPriority(ExecutorState& executor, const TaskPriority priority) {
  switch (priority) {
    case TaskPriority::High:
      return executor.high_priority_tasks;
    case TaskPriority::Low:
      return executor.low_priority_tasks;
    case TaskPriority::Normal:
    default:
      return executor.normal_priority_tasks;
  }
}

TaskRuntime::ExecutorState& TaskRuntime::GetExecutor(const TaskExecutorType executor) {
  switch (executor) {
    case TaskExecutorType::MainThread:
      return main_executor_;
    case TaskExecutorType::AssetIo:
      return asset_io_executor_;
    case TaskExecutorType::Render:
      return render_executor_;
    case TaskExecutorType::Background:
      return background_executor_;
    case TaskExecutorType::Worker:
    default:
      return worker_executor_;
  }
}

const TaskRuntime::ExecutorState& TaskRuntime::GetExecutor(const TaskExecutorType executor) const {
  return const_cast<TaskRuntime*>(this)->GetExecutor(executor);
}

TaskRuntime::ExecutorState& TaskRuntime::GetRunnableExecutor(const TaskExecutorType executor) {
  auto& executor_state = GetExecutor(executor);
  if (executor != TaskExecutorType::MainThread) {
    std::lock_guard lock(executor_state.mutex);
    if (!executor_state.threads.empty()) {
      return executor_state;
    }
  } else {
    return executor_state;
  }
  return worker_executor_;
}

TaskHandle TaskRuntime::AllocateTaskLocked() {
  TaskHandle handle;
  if (!recycled_task_indices_.empty()) {
    handle.index_ = recycled_task_indices_.front();
    recycled_task_indices_.pop();
    handle.generation_ = tasks_[handle.index_]->generation;
  } else {
    handle.index_ = static_cast<int>(tasks_.size());
    handle.generation_ = 1;
    tasks_.emplace_back(std::make_shared<TaskRecord>());
    tasks_.back()->generation = handle.generation_;
  }
  return handle;
}

void TaskRuntime::StartExecutor(ExecutorState& executor, const size_t thread_size, const std::string& name) {
  executor.name = name;
  executor.stopping = false;
  for (size_t i = 0; i < thread_size; ++i) {
    executor.threads.emplace_back(std::make_unique<std::thread>([this, &executor]() {
      ExecutorLoop(executor);
    }));
  }
}

void TaskRuntime::StopExecutor(ExecutorState& executor) {
  {
    std::lock_guard lock(executor.mutex);
    executor.stopping = true;
  }
  executor.cv.notify_all();
  for (const auto& thread : executor.threads) {
    if (thread && thread->joinable()) {
      thread->join();
    }
  }
  executor.threads.clear();
  {
    std::lock_guard lock(executor.mutex);
    executor.high_priority_tasks.clear();
    executor.normal_priority_tasks.clear();
    executor.low_priority_tasks.clear();
    executor.queued_task_size = 0;
    executor.stopping = false;
  }
}

void TaskRuntime::ExecutorLoop(ExecutorState& executor) {
  while (true) {
    TaskHandle handle;
    {
      std::unique_lock lock(executor.mutex);
      executor.cv.wait(lock, [&executor]() {
        return executor.stopping || ExecutorHasTasks(executor);
      });
      if (executor.stopping && !ExecutorHasTasks(executor)) {
        return;
      }
      if (!executor.high_priority_tasks.empty()) {
        handle = executor.high_priority_tasks.front();
        executor.high_priority_tasks.pop_front();
      } else if (!executor.normal_priority_tasks.empty()) {
        handle = executor.normal_priority_tasks.front();
        executor.normal_priority_tasks.pop_front();
      } else if (!executor.low_priority_tasks.empty()) {
        handle = executor.low_priority_tasks.front();
        executor.low_priority_tasks.pop_front();
      }
      if (executor.queued_task_size > 0) {
        --executor.queued_task_size;
      }
    }
    RunTask(handle, executor.type);
  }
}

void TaskRuntime::QueueReadyTask(const TaskHandle& handle) {
  if (!handle.Valid()) {
    return;
  }

  TaskOptions options;
  {
    std::lock_guard lock(task_mutex_);
    if (!IsHandleValidLocked(handle)) {
      return;
    }
    options = tasks_[handle.index_]->options;
  }

  auto& executor = GetRunnableExecutor(options.executor);
  {
    std::lock_guard lock(executor.mutex);
    GetQueueForPriority(executor, options.priority).emplace_back(handle);
    ++executor.queued_task_size;
  }
  executor.cv.notify_one();
}

void TaskRuntime::QueueReadyTasks(const std::vector<TaskHandle>& handles) {
  for (const auto& handle : handles) {
    QueueReadyTask(handle);
  }
}

bool TaskRuntime::TryPopTask(ExecutorState& executor, TaskHandle& handle) {
  std::lock_guard lock(executor.mutex);
  if (!executor.high_priority_tasks.empty()) {
    handle = executor.high_priority_tasks.front();
    executor.high_priority_tasks.pop_front();
  } else if (!executor.normal_priority_tasks.empty()) {
    handle = executor.normal_priority_tasks.front();
    executor.normal_priority_tasks.pop_front();
  } else if (!executor.low_priority_tasks.empty()) {
    handle = executor.low_priority_tasks.front();
    executor.low_priority_tasks.pop_front();
  } else {
    return false;
  }
  if (executor.queued_task_size > 0) {
    --executor.queued_task_size;
  }
  return true;
}

bool TaskRuntime::TryPopTaskForWaiting(TaskHandle& handle, TaskExecutorType& executor) {
  if (IsMainThread()) {
    executor = TaskExecutorType::MainThread;
    return TryPopTask(main_executor_, handle);
  }
  if (g_current_task_runtime != this) {
    return false;
  }
  executor = g_current_executor;
  return TryPopTask(GetExecutor(executor), handle);
}

void TaskRuntime::RunTask(const TaskHandle& handle, const TaskExecutorType executor) {
  std::function<void()> function;
  CancellationToken cancellation_token;
  TaskOptions options;
  {
    std::lock_guard lock(task_mutex_);
    if (!IsHandleValidLocked(handle)) {
      return;
    }
    auto& task = *tasks_[handle.index_];
    if (task.state != TaskState::Queued) {
      return;
    }
    task.state = TaskState::Running;
    function = task.function;
    cancellation_token = task.options.cancellation_token;
    options = task.options;
  }

  ++running_task_size_;
  std::exception_ptr exception;
  auto* previous_runtime = g_current_task_runtime;
  const auto previous_executor = g_current_executor;
  g_current_task_runtime = this;
  g_current_executor = executor;
  try {
    if (!cancellation_token.IsCancellationRequested() && function) {
      function();
    }
  } catch (...) {
    exception = std::current_exception();
  }
  g_current_task_runtime = previous_runtime;
  g_current_executor = previous_executor;
  --running_task_size_;

  std::vector<TaskHandle> ready_tasks;
  {
    std::lock_guard lock(task_mutex_);
    if (!IsHandleValidLocked(handle)) {
      return;
    }
    auto& task = *tasks_[handle.index_];
    task.state = TaskState::Completed;
    task.exception = exception;
    ++completed_task_size_;
    if (exception) {
      ++failed_task_size_;
      std::cerr << "[EvoEngine]Task failed on executor " << ExecutorName(executor);
      if (!options.debug_name.empty()) {
        std::cerr << " (" << options.debug_name << ")";
      }
      std::cerr << std::endl;
    }

    for (const auto& dependent_handle : task.dependents) {
      if (!IsHandleValidLocked(dependent_handle)) {
        continue;
      }
      auto& dependent_task = *tasks_[dependent_handle.index_];
      if (dependent_task.pending_dependency_size > 0) {
        --dependent_task.pending_dependency_size;
      }
      if (dependent_task.scheduled && dependent_task.pending_dependency_size == 0 &&
          dependent_task.state == TaskState::Pending) {
        dependent_task.state = TaskState::Queued;
        ready_tasks.emplace_back(dependent_handle);
      }
    }
  }
  task_finished_cv_.notify_all();
  QueueReadyTasks(ready_tasks);
}

void TaskRuntime::StartTaskGraphLocked(const TaskHandle& handle, std::vector<TaskHandle>& ready_tasks,
                                       std::vector<int>& visited_indices) {
  if (!IsHandleValidLocked(handle)) {
    return;
  }
  if (std::find(visited_indices.begin(), visited_indices.end(), handle.index_) != visited_indices.end()) {
    return;
  }
  visited_indices.emplace_back(handle.index_);

  auto& task = *tasks_[handle.index_];
  for (const auto& dependency : task.dependencies) {
    StartTaskGraphLocked(dependency, ready_tasks, visited_indices);
  }

  if (!task.scheduled) {
    task.scheduled = true;
  }
  if (task.pending_dependency_size == 0 && task.state == TaskState::Pending) {
    task.state = TaskState::Queued;
    ready_tasks.emplace_back(handle);
  }
}

void TaskRuntime::CollectTaskTreeLocked(const TaskHandle& handle, std::vector<TaskHandle>& task_tree) const {
  if (!IsHandleValidLocked(handle)) {
    return;
  }
  if (std::find_if(task_tree.begin(), task_tree.end(), [handle](const TaskHandle& other) {
        return other.GetIndex() == handle.GetIndex();
      }) != task_tree.end()) {
    return;
  }
  task_tree.emplace_back(handle);
  for (const auto& dependency : tasks_[handle.index_]->dependencies) {
    CollectTaskTreeLocked(dependency, task_tree);
  }
}

bool TaskRuntime::CanRecycleTaskLocked(const TaskHandle& handle) const {
  if (!IsHandleValidLocked(handle)) {
    return false;
  }
  const auto& task = *tasks_[handle.index_];
  if (task.state != TaskState::Completed) {
    return false;
  }
  for (const auto& dependent : task.dependents) {
    if (IsHandleValidLocked(dependent) && tasks_[dependent.index_]->state != TaskState::Completed) {
      return false;
    }
  }
  return true;
}

void TaskRuntime::RecycleTaskLocked(const TaskHandle& handle) {
  if (!IsHandleValidLocked(handle)) {
    return;
  }
  auto& task = *tasks_[handle.index_];
  task.state = TaskState::Recycled;
  task.dependencies.clear();
  task.dependents.clear();
  task.pending_dependency_size = 0;
  task.scheduled = false;
  task.options = {};
  task.function = nullptr;
  task.exception = nullptr;
  ++task.generation;
  if (task.generation == 0) {
    task.generation = 1;
  }
  recycled_task_indices_.emplace(handle.index_);
}
