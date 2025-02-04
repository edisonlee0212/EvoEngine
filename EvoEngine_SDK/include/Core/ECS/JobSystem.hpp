
#pragma once

namespace evo_engine {

/**
 * @class JobHandle
 * @brief Represents a handle to a job, used to track and manage job states.
 */
class JobHandle {
  int index_ = -1;  ///< Index of the job handle.
  friend class JobSystem;

 public:
  /**
   * @brief Gets the index associated with the job handle.
   * @return The index of the job handle.
   */
  [[nodiscard]] int GetIndex() const;

  /**
   * @brief Checks if the job handle is valid.
   * @return True if the job handle is valid, otherwise false.
   */
  [[nodiscard]] bool Valid() const;
};

/**
 * @class JobSystem
 * @brief Manages a system of jobs for parallel execution.
 */
class JobSystem {
  /**
   * @class JobSystemSemaphore
   * @brief A semaphore for job synchronization.
   */
  class JobSystemSemaphore {
    size_t availability_ = 0;     ///< The current availability count of the semaphore.
    std::mutex update_mutex_;     ///< Mutex for synchronizing semaphore updates.
    std::condition_variable cv_;  ///< Condition variable to signal semaphore availability.

   public:
    /**
     * @brief Resets the semaphore to a specified availability count.
     * @param availability The initial availability count. Default is 0.
     */
    void Reset(size_t availability = 0);

    /**
     * @brief Acquires the semaphore, decrementing its count.
     */
    void Acquire();

    /**
     * @brief Releases the semaphore, incrementing its count.
     */
    void Release();
  };

  /**
   * @struct Job
   * @brief Represents a single executable job in the job system.
   */
  struct Job {
    std::vector<JobHandle> parents;         ///< Parent jobs that this job depends on.
    std::vector<JobHandle> children;        ///< Child jobs dependent on this job.
    JobHandle job_handle;                   ///< Handle associated with this job.
    JobSystemSemaphore finished_semaphore;  ///< Semaphore indicating when the job is complete.
    bool recycled = false;                  ///< Flag indicating if the job has been recycled.
    bool finished = false;                  ///< Flag indicating if the job is finished.
    bool wake = false;                      ///< Flag indicating if the job system should wake up to handle this job.
    std::function<void()> task;             ///< The function to execute as part of the job.
  };

  /**
   * @class JobPool
   * @brief A pool for managing the availability of jobs.
   */
  class JobPool {
   public:
    /**
     * @brief Pushes a new job onto the pool.
     * @param job A pair consisting of a JobHandle and the associated function.
     */
    void Push(std::pair<JobHandle, std::function<void()>>&& job);

    /**
     * @brief Pops a job from the pool if available.
     * @param job Output parameter to store the retrieved job pair.
     * @return True if a job was retrieved, otherwise false.
     */
    [[nodiscard]] bool Pop(std::pair<JobHandle, std::function<void()>>& job);

    /**
     * @brief Checks if the job pool is empty.
     * @return True if the job pool is empty, otherwise false.
     */
    [[nodiscard]] bool Empty();

   private:
    std::queue<std::pair<JobHandle, std::function<void()>>> job_queue_;  ///< Queue holding job pairs.
    std::mutex pool_mutex_;  ///< Mutex for synchronizing access to the job queue.
  };

  std::vector<std::shared_ptr<Job>> jobs_;  ///< List of all jobs managed by the system.
  std::queue<JobHandle> recycled_jobs_;     ///< Queue of recycled job handles.
  JobPool available_job_pool_;              ///< Pool of available jobs.

  /**
   * @brief Checks if the current thread is the main thread.
   * @return True if the current thread is the main thread, otherwise false.
   */
  [[nodiscard]] bool MainThreadCheck() const;

  /**
   * @brief Helper function to collect descendant jobs recursively.
   * @param jobs A vector to collect the descendant jobs.
   * @param walker The current job handle to traverse from.
   */
  void CollectDescendantsHelper(std::vector<JobHandle>& jobs, const JobHandle& walker);

  /**
   * @brief Helper function to check if a job is available.
   * @param job_handle The handle of the job to check.
   */
  void CheckJobAvailableHelper(const JobHandle& job_handle);

  /**
   * @brief Reports that a job has finished execution.
   * @param job_handle The handle of the finished job.
   */
  void ReportFinish(const JobHandle& job_handle);

  /**
   * @brief Initializes a worker thread by its index.
   * @param worker_index The index of the worker thread to initialize.
   */
  void InitializeWorker(size_t worker_index);

  std::atomic<int> idle_thread_amount_;                    ///< Number of idle threads.
  std::vector<std::unique_ptr<std::thread>> workers_;      ///< Worker threads.
  std::vector<std::shared_ptr<std::atomic<bool>>> flags_;  ///< Flags indicating worker states.
  std::atomic<bool> is_done_;                              ///< Flag indicating if the job system is shutting down.

  std::mutex job_management_mutex_;                  ///< Mutex for managing jobs.
  std::mutex job_availability_mutex_;                ///< Mutex for handling job availability.
  std::condition_variable job_available_condition_;  ///< Condition variable for job availability notifications.

  std::thread::id main_thread_id_;  ///< ID of the main thread.

 public:
  /**
   * @brief Stops all worker threads.
   */
  void StopAllWorkers();

  /**
   * @brief Gets the number of currently idle workers.
   * @return The number of idle workers.
   */
  [[nodiscard]] size_t IdleWorkerSize() const;

  /**
   * @brief Constructs the JobSystem.
   */
  JobSystem();

  /**
   * @brief Cleanup function to be called during destruction.
   */
  void OnDestroy();

  /**
   * @brief Destructs the JobSystem.
   */
  ~JobSystem();

  /**
   * @brief Resizes the number of worker threads in the system.
   * @param worker_size The new number of worker threads.
   */
  void ResizeWorker(size_t worker_size);

  /**
   * @brief Gets the current number of worker threads.
   * @return The number of worker threads.
   */
  [[nodiscard]] size_t GetWorkerSize() const;

  /**
   * @brief Pushes a new job into the job system.
   * @param dependencies The list of jobs that must be completed before this job starts.
   * @param func The function to execute as part of the job.
   * @return A handle representing the pushed job.
   */
  [[nodiscard]] JobHandle PushJob(const std::vector<JobHandle>& dependencies, std::function<void()>&& func);

  /**
   * @brief Executes the specified job immediately.
   * @param job_handle The handle of the job to execute.
   */
  void ExecuteJob(const JobHandle& job_handle);

  /**
   * @brief Waits for the completion of a specific job.
   * @param job_handle The handle of the job to wait for.
   */
  void Wait(const JobHandle& job_handle);
};

}  // namespace evo_engine
