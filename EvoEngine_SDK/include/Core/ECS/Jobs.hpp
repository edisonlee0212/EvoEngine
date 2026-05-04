
#pragma once
#include "JobSystem.hpp"

namespace evo_engine {

/**
 * @class Jobs
 * @brief A singleton class for managing job system operations such as parallel execution and scheduling.
 */
class Jobs final {
 public:
  static Jobs& GetInstance();

 private:
  JobSystem job_system_;

 public:
  /**
   * @brief Retrieves the number of worker threads available.
   * @return The size of the worker pool.
   */
  static size_t GetWorkerSize();

  /**
   * @brief Initializes the job system with the specified number of worker threads.
   * @param worker_size Number of worker threads to be initialized.
   */
  static void Initialize(size_t worker_size);

  /**
   * @brief Completes a parallel for-loop job on the given function across the specified range.
   * @param size The number of iterations to process.
   * @param func The function to execute for each iteration, taking the index as a parameter.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   */
  static void RunParallelFor(size_t size, const std::function<void(size_t i)>& func, size_t worker_size = 0);

  /**
   * @brief Completes a parallel for-loop job on the given function across the specified range with worker indices.
   * @param size The number of iterations to process.
   * @param func The function to execute for each iteration, taking the index and worker index as parameters.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   */
  static void RunParallelFor(size_t size, const std::function<void(size_t i, size_t worker_index)>& func,
                             size_t worker_size = 0);

  /**
   * @brief Schedules a parallel for-loop job. Job will be in pending start state.
   * @param size The number of iterations to schedule.
   * @param func The function to execute for each iteration, taking the index as a parameter.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   * @return A JobHandle representing the scheduled job.
   */
  static JobHandle ScheduleParallelFor(size_t size, const std::function<void(size_t i)>& func, size_t worker_size = 0);

  /**
   * @brief Schedules a parallel for-loop job with worker indices. Job will be in pending start state.
   * @param size The number of iterations to schedule.
   * @param func The function to execute for each iteration, taking the index and worker index as parameters.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   * @return A JobHandle representing the scheduled job.
   */
  static JobHandle ScheduleParallelFor(size_t size, const std::function<void(size_t i, size_t worker_index)>& func,
                                       size_t worker_size = 0);

  /**
   * @brief Completes a parallel for-loop job with dependencies on a set of prior jobs.
   * @param dependencies A vector of JobHandle representing dependent jobs.
   * @param size The number of iterations to process.
   * @param func The function to execute for each iteration, taking the index as a parameter.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   */
  static void RunParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                             const std::function<void(size_t i)>& func, size_t worker_size = 0);

  /**
   * @brief Completes a parallel for-loop job with dependencies on a set of prior jobs, including worker indices.
   * @param dependencies A vector of JobHandle representing dependent jobs.
   * @param size The number of iterations to process.
   * @param func The function to execute for each iteration, taking the index and worker index as parameters.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   */
  static void RunParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                             const std::function<void(size_t i, size_t worker_index)>& func, size_t worker_size = 0);

  /**
   * @brief Schedules a parallel for-loop job with dependencies. Job will be in pending start state.
   * @param dependencies A vector of JobHandle representing dependent jobs.
   * @param size The number of iterations to schedule.
   * @param func The function to execute for each iteration, taking the index as a parameter.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   * @return A JobHandle representing the scheduled job.
   */
  static JobHandle ScheduleParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                                       const std::function<void(size_t i)>& func, size_t worker_size = 0);

  /**
   * @brief Schedules a parallel for-loop job with dependencies and worker indices. Job will be in pending start state.
   * @param dependencies A vector of JobHandle representing dependent jobs.
   * @param size The number of iterations to schedule.
   * @param func The function to execute for each iteration, taking the index and worker index as parameters.
   * @param worker_size Optional: The number of worker threads to use for execution. Defaults to 0.
   * @return A JobHandle representing the scheduled job.
   */
  static JobHandle ScheduleParallelFor(const std::vector<JobHandle>& dependencies, size_t size,
                                       const std::function<void(size_t i, size_t worker_index)>& func,
                                       size_t worker_size = 0);

  /**
   * @brief Schedules a job with dependencies, executing the given function. Job will be in pending start state.
   * @param dependencies A vector of JobHandle representing dependent jobs.
   * @param func The function to execute once dependencies are resolved.
   * @return A JobHandle representing the scheduled job.
   */
  static JobHandle Run(const std::vector<JobHandle>& dependencies, const std::function<void()>& func);

  /**
   * @brief Schedules a job executing the given function without dependencies. Job will be in pending start state.
   * @param func The function to execute.
   * @return A JobHandle representing the scheduled job.
   */
  static JobHandle Run(const std::function<void()>& func);

  /**
   * @brief Combines multiple jobs into a single job.
   * @param dependencies A vector of JobHandle representing dependent jobs.
   * @return A JobHandle combining the dependencies.
   */
  static JobHandle Combine(const std::vector<JobHandle>& dependencies);

  /**
   * @brief Start execution of the scheduled job.
   * @param job_handle The JobHandle representing the job to execute.
   */
  static void Execute(const JobHandle& job_handle);

  /**
   * @brief Waits for the started job to complete execution.
   * @param job_handle The JobHandle representing the job to wait for.
   */
  static void Wait(const JobHandle& job_handle);

  /**
   * @brief Cleans up and destroys the job system resources.
   */
  static void OnDestroy();
};

}  // namespace evo_engine
