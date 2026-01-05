#pragma once

#include <filesystem>
#include <string>

namespace eco_sys_lab_package {

/// OS process handle for a KineticMeshPrecomputeApp worker.
/// Lifetime is tied to the parent process (Windows job object / Linux PR_SET_PDEATHSIG).
struct PrecomputeWorkerProcess {
#ifdef _WIN32
  void* handle = nullptr;  // HANDLE
  unsigned long pid = 0;
#else
  int pid = -1;
#endif

  [[nodiscard]] bool Valid() const;
  /// Release OS resources without waiting for exit (safe if already reaped by Poll).
  void Reset();
};

enum class PrecomputeWorkerPollStatus {
  StillRunning,
  Exited,
  Lost,
};

struct PrecomputeWorkerPollResult {
  PrecomputeWorkerPollStatus status = PrecomputeWorkerPollStatus::StillRunning;
  int exit_code = 0;  // set when status == Exited
};

/// Spawn KineticMeshPrecomputeApp next to the host executable with `--job <job_file>`.
/// On success, @p out_process is valid and parent-death reaping is armed.
[[nodiscard]] bool SpawnPrecomputeWorkerProcess(const std::filesystem::path& job_file,
                                                const std::filesystem::path& log_file,
                                                const std::filesystem::path& working_directory,
                                                PrecomputeWorkerProcess& out_process, std::string* error_message);

/// Non-blocking poll. On Exited/Lost, @p process is reset.
[[nodiscard]] PrecomputeWorkerPollResult PollPrecomputeWorkerProcess(PrecomputeWorkerProcess& process);

}  // namespace eco_sys_lab_package
