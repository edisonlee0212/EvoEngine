#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace evo_engine {

/**
 * @brief Logical execution lanes owned by the engine runtime.
 */
enum class TaskExecutorType { MainThread, Worker, AssetIo, Gpu, Render, Background };

/**
 * @brief Queue priority within an executor.
 */
enum class TaskPriority { Low, Normal, High };

/**
 * @brief Optional hard affinity for tasks that must run on a specific lane.
 */
enum class ThreadAffinity { Any, MainThread, Worker, AssetIo, Gpu, Render, Background };

/**
 * @brief Read-only cancellation view checked before a task starts execution.
 */
class CancellationToken {
  std::shared_ptr<std::atomic<bool>> state_;

  explicit CancellationToken(std::shared_ptr<std::atomic<bool>> state);
  friend class CancellationSource;

 public:
  CancellationToken() = default;
  [[nodiscard]] bool IsCancellationRequested() const;
  [[nodiscard]] bool Valid() const;
};

/**
 * @brief Owns cancellation state that can be shared with scheduled tasks.
 */
class CancellationSource {
  std::shared_ptr<std::atomic<bool>> state_ = std::make_shared<std::atomic<bool>>(false);

 public:
  [[nodiscard]] CancellationToken GetToken() const;
  void Cancel() const;
  [[nodiscard]] bool IsCancellationRequested() const;
};

/**
 * @brief Generation-checked handle for a scheduled task.
 */
class TaskHandle {
  int index_ = -1;
  uint32_t generation_ = 0;

  friend class TaskRuntime;

 public:
  [[nodiscard]] int GetIndex() const;
  [[nodiscard]] uint32_t GetGeneration() const;
  [[nodiscard]] bool Valid() const;
};

/**
 * @brief Small dependency collection used when building task graphs incrementally.
 */
class TaskGroup {
  std::vector<TaskHandle> handles_;

 public:
  void Add(const TaskHandle& handle);
  void Clear();
  [[nodiscard]] bool Empty() const;
  [[nodiscard]] const std::vector<TaskHandle>& GetHandles() const;
};

/**
 * @brief Scheduling options for executor selection, priority, cancellation, and diagnostics.
 */
struct TaskOptions {
  TaskExecutorType executor = TaskExecutorType::Worker;
  TaskPriority priority = TaskPriority::Normal;
  ThreadAffinity affinity = ThreadAffinity::Any;
  CancellationToken cancellation_token;
  std::string debug_name;
};

/**
 * @brief Thread counts for the engine-owned task runtime.
 */
struct TaskRuntimeSettings {
  size_t worker_thread_size = 1;
  size_t asset_io_thread_size = 1;
  size_t gpu_thread_size = 1;
  size_t render_thread_size = 1;
  size_t background_thread_size = 1;
};

/**
 * @brief Runtime counters intended for diagnostics and editor tooling.
 */
struct TaskRuntimeStats {
  size_t worker_thread_size = 0;
  size_t asset_io_thread_size = 0;
  size_t gpu_thread_size = 0;
  size_t render_thread_size = 0;
  size_t background_thread_size = 0;
  size_t queued_task_size = 0;
  size_t running_task_size = 0;
  size_t submitted_task_size = 0;
  size_t completed_task_size = 0;
  size_t failed_task_size = 0;
};

/**
 * @brief Engine-owned task scheduler with a worker pool plus named always-on service executors.
 *
 * TaskRuntime is intentionally the stable abstraction boundary for the engine. Higher-level systems should depend on
 * this API instead of a third-party scheduler directly; the backend can be replaced later after focused bake-offs.
 */
class TaskRuntime {
  enum class TaskState { Pending, Queued, Running, Completed, Recycled };

  struct TaskRecord {
    uint32_t generation = 1;
    TaskState state = TaskState::Recycled;
    std::vector<TaskHandle> dependencies;
    std::vector<TaskHandle> dependents;
    size_t pending_dependency_size = 0;
    bool scheduled = false;
    TaskOptions options;
    std::function<void()> function;
    std::exception_ptr exception;
  };

  struct ExecutorState {
    TaskExecutorType type = TaskExecutorType::Worker;
    std::string name;
    std::vector<std::unique_ptr<std::thread>> threads;
    std::deque<TaskHandle> high_priority_tasks;
    std::deque<TaskHandle> normal_priority_tasks;
    std::deque<TaskHandle> low_priority_tasks;
    mutable std::mutex mutex;
    std::condition_variable cv;
    bool stopping = false;
    size_t queued_task_size = 0;
  };

  std::thread::id main_thread_id_;
  ExecutorState main_executor_;
  ExecutorState worker_executor_;
  ExecutorState asset_io_executor_;
  ExecutorState gpu_executor_;
  ExecutorState render_executor_;
  ExecutorState background_executor_;

  mutable std::mutex task_mutex_;
  std::condition_variable task_finished_cv_;
  std::vector<std::shared_ptr<TaskRecord>> tasks_;
  std::queue<int> recycled_task_indices_;

  std::atomic<bool> shutting_down_{false};
  std::atomic<size_t> running_task_size_{0};
  std::atomic<size_t> submitted_task_size_{0};
  std::atomic<size_t> completed_task_size_{0};
  std::atomic<size_t> failed_task_size_{0};

  [[nodiscard]] bool IsHandleValidLocked(const TaskHandle& handle) const;
  [[nodiscard]] static bool ExecutorHasTasks(const ExecutorState& executor);
  [[nodiscard]] static std::deque<TaskHandle>& GetQueueForPriority(ExecutorState& executor, TaskPriority priority);
  [[nodiscard]] ExecutorState& GetExecutor(TaskExecutorType executor);
  [[nodiscard]] const ExecutorState& GetExecutor(TaskExecutorType executor) const;
  [[nodiscard]] ExecutorState& GetRunnableExecutor(TaskExecutorType executor);
  [[nodiscard]] TaskHandle AllocateTaskLocked();

  void StartExecutor(ExecutorState& executor, size_t thread_size, const std::string& name);
  void StopExecutor(ExecutorState& executor);
  void ExecutorLoop(ExecutorState& executor);
  void QueueReadyTask(const TaskHandle& handle);
  void QueueReadyTasks(const std::vector<TaskHandle>& handles);
  [[nodiscard]] bool TryPopTask(ExecutorState& executor, TaskHandle& handle);
  [[nodiscard]] bool TryPopTaskForWaiting(TaskHandle& handle, TaskExecutorType& executor);
  void RunTask(const TaskHandle& handle, TaskExecutorType executor);

  void StartTaskGraphLocked(const TaskHandle& handle, std::vector<TaskHandle>& ready_tasks,
                            std::vector<int>& visited_indices);
  void CollectTaskTreeLocked(const TaskHandle& handle, std::vector<TaskHandle>& task_tree) const;
  [[nodiscard]] bool CanRecycleTaskLocked(const TaskHandle& handle) const;
  void RecycleTaskLocked(const TaskHandle& handle);

 public:
  TaskRuntime();
  ~TaskRuntime();
  TaskRuntime(const TaskRuntime&) = delete;
  TaskRuntime& operator=(const TaskRuntime&) = delete;

  /**
   * @brief Starts the configured executor threads and resets runtime counters.
   */
  void Initialize(const TaskRuntimeSettings& settings);

  /**
   * @brief Stops all executor threads, clears queued tasks, and invalidates outstanding handles.
   */
  void Shutdown();

  /**
   * @brief Resizes only the general worker pool. Service executors keep their configured sizes.
   */
  void ResizeWorker(size_t worker_size);

  [[nodiscard]] size_t GetWorkerSize() const;
  [[nodiscard]] size_t GetThreadSize(TaskExecutorType executor) const;
  [[nodiscard]] bool IsMainThread() const;
  [[nodiscard]] bool IsExecutorThread(TaskExecutorType executor) const;
  [[nodiscard]] TaskRuntimeStats GetStats() const;

  /**
   * @brief Creates a pending task. Call Execute or Wait on the returned handle to start its dependency graph.
   */
  [[nodiscard]] TaskHandle Schedule(const std::vector<TaskHandle>& dependencies, const TaskOptions& options,
                                    std::function<void()>&& function);
  [[nodiscard]] TaskHandle Schedule(const std::vector<TaskHandle>& dependencies, std::function<void()>&& function);
  [[nodiscard]] TaskHandle Schedule(std::function<void()>&& function);

  /**
   * @brief Creates a no-op task that completes after all dependencies complete.
   */
  [[nodiscard]] TaskHandle Combine(const std::vector<TaskHandle>& dependencies);
  [[nodiscard]] TaskHandle Combine(const TaskGroup& task_group);

  /**
   * @brief Starts the selected task and all unscheduled dependencies.
   */
  void Execute(const TaskHandle& handle);

  /**
   * @brief Starts the task graph, helps execute eligible work while waiting, and rethrows task exceptions.
   */
  void Wait(const TaskHandle& handle);
  [[nodiscard]] bool IsCompleted(const TaskHandle& handle) const;
  [[nodiscard]] std::exception_ptr GetException(const TaskHandle& handle) const;

  /**
   * @brief Drains ready main-thread tasks. Must be called from the runtime's main thread.
   */
  size_t RunReadyMainThreadTasks(size_t max_task_size = 0);
};

}  // namespace evo_engine
