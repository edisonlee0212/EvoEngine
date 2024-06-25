#pragma once
namespace evo_engine {
class JobHandle {
  int index_ = -1;
  friend class JobSystem;

 public:
  [[nodiscard]] int GetIndex() const;
  [[nodiscard]] bool Valid() const;
};

class JobSystem {
  class JobSystemSemaphore {
    size_t availability_ = 0;
    std::mutex update_mutex_;
    std::condition_variable cv_;

   public:
    void Reset(size_t availability = 0);
    void Acquire();
    void Release();
  };
  struct Job {
    std::vector<JobHandle> parents;
    std::vector<JobHandle> children;
    JobHandle job_handle;
    JobSystemSemaphore finished_semaphore;
    bool recycled = false;
    bool finished = false;
    bool wake = false;
    std::function<void()> task;
  };
  class JobPool {
   public:
    void Push(std::pair<JobHandle, std::function<void()>>&& job);
    // deletes the retrieved element, do not use for non integral types
    [[nodiscard]] bool Pop(std::pair<JobHandle, std::function<void()>>& job);
    [[nodiscard]] bool Empty();

   private:
    std::queue<std::pair<JobHandle, std::function<void()>>> job_queue_;
    std::mutex pool_mutex_;
  };
  std::vector<std::shared_ptr<Job>> jobs_;
  std::queue<JobHandle> recycled_jobs_;
  JobPool available_job_pool_;

  [[nodiscard]] bool MainThreadCheck() const;
  void CollectDescendantsHelper(std::vector<JobHandle>& jobs, const JobHandle& walker);
  void CheckJobAvailableHelper(const JobHandle& job_handle);
  void ReportFinish(const JobHandle& job_handle);
  void InitializeWorker(size_t worker_index);
  std::atomic<int> idle_thread_amount_;  // how many threads are waiting
  std::vector<std::unique_ptr<std::thread>> workers_;
  std::vector<std::shared_ptr<std::atomic<bool>>> flags_;
  std::atomic<bool> is_done_;

  std::mutex job_management_mutex_;
  std::mutex job_availability_mutex_;
  std::condition_variable job_available_condition_;

  std::thread::id main_thread_id_;

 public:
  void StopAllWorkers();
  [[nodiscard]] size_t IdleWorkerSize() const;
  JobSystem();
  ~JobSystem();
  void ResizeWorker(size_t worker_size);
  [[nodiscard]] size_t GetWorkerSize() const;
  [[nodiscard]] JobHandle PushJob(const std::vector<JobHandle>& dependencies, std::function<void()>&& func);
  void ExecuteJob(const JobHandle& job_handle);
  void Wait(const JobHandle& job_handle);
};
}  // namespace evo_engine
