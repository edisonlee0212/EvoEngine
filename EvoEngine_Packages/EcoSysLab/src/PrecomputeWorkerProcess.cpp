#include "PrecomputeWorkerProcess.hpp"

#include <mutex>
#include <sstream>
#include <vector>

#include "Console.hpp"

#ifdef _WIN32
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#elif defined(__linux__)
#  include <fcntl.h>
#  include <limits.h>
#  include <signal.h>
#  include <sys/prctl.h>
#  include <sys/wait.h>
#  include <unistd.h>
#  include <cerrno>
#  include <cstring>
#endif

namespace eco_sys_lab_package {
namespace {

#if defined(_WIN32) || defined(__linux__)
std::filesystem::path HostExecutableDirectory() {
#  ifdef _WIN32
  char module_path[MAX_PATH]{};
  if (GetModuleFileNameA(nullptr, module_path, MAX_PATH) == 0) {
    return {};
  }
  return std::filesystem::path(module_path).parent_path();
#  else
  char module_path[PATH_MAX]{};
  const ssize_t n = readlink("/proc/self/exe", module_path, sizeof(module_path) - 1);
  if (n <= 0) {
    return {};
  }
  module_path[n] = '\0';
  return std::filesystem::path(module_path).parent_path();
#  endif
}

std::filesystem::path WorkerExecutablePath(const std::filesystem::path& exe_dir) {
#  ifdef _WIN32
  return exe_dir / "KineticMeshPrecomputeApp.exe";
#  else
  return exe_dir / "KineticMeshPrecomputeApp";
#  endif
}
#endif

#ifdef _WIN32
HANDLE EnsureKillOnCloseJob() {
  static HANDLE job = nullptr;
  static std::once_flag once;
  std::call_once(once, []() {
    job = CreateJobObjectW(nullptr, nullptr);
    if (!job) {
      return;
    }
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    if (!SetInformationJobObject(job, JobObjectExtendedLimitInformation, &limits, sizeof(limits))) {
      CloseHandle(job);
      job = nullptr;
    }
  });
  return job;
}

bool AssignToKillOnCloseJob(HANDLE process) {
  const HANDLE job = EnsureKillOnCloseJob();
  if (!job) {
    return false;
  }
  return AssignProcessToJobObject(job, process) == TRUE;
}
#elif defined(__linux__)
[[noreturn]] void ChildApplyParentDeathAndExec(std::string worker_exe, std::string job_path, std::string working_dir,
                                               const int log_fd, const pid_t parent_pid) {
  if (prctl(PR_SET_PDEATHSIG, SIGKILL) != 0) {
    _exit(127);
  }
  if (getppid() != parent_pid) {
    _exit(127);
  }
  if (log_fd >= 0) {
    dup2(log_fd, STDOUT_FILENO);
    dup2(log_fd, STDERR_FILENO);
    if (log_fd != STDOUT_FILENO && log_fd != STDERR_FILENO) {
      close(log_fd);
    }
  }
  if (!working_dir.empty() && chdir(working_dir.c_str()) != 0) {
    _exit(127);
  }
  execl(worker_exe.c_str(), worker_exe.c_str(), "--job", job_path.c_str(), static_cast<char*>(nullptr));
  _exit(127);
}
#endif

void SetError(std::string* error_message, std::string message) {
  if (error_message) {
    *error_message = std::move(message);
  }
}

}  // namespace

bool PrecomputeWorkerProcess::Valid() const {
#ifdef _WIN32
  return handle != nullptr;
#else
  return pid > 0;
#endif
}

void PrecomputeWorkerProcess::Reset() {
#ifdef _WIN32
  if (handle) {
    CloseHandle(static_cast<HANDLE>(handle));
    handle = nullptr;
  }
  pid = 0;
#else
  pid = -1;
#endif
}

bool SpawnPrecomputeWorkerProcess(const std::filesystem::path& job_file, const std::filesystem::path& log_file,
                                  const std::filesystem::path& working_directory, PrecomputeWorkerProcess& out_process,
                                  std::string* error_message) {
  out_process.Reset();

#if !defined(_WIN32) && !defined(__linux__)
  SetError(error_message, "Precompute workers require Windows or Linux.");
  return false;
#else
  if (job_file.empty() || !std::filesystem::exists(job_file)) {
    SetError(error_message, "job file missing: " + job_file.string());
    return false;
  }

  const std::filesystem::path exe_dir = HostExecutableDirectory();
  if (exe_dir.empty()) {
    SetError(error_message, "failed to resolve host executable directory");
    return false;
  }
  const std::filesystem::path worker_exe = WorkerExecutablePath(exe_dir);
  if (!std::filesystem::exists(worker_exe)) {
    SetError(error_message, "worker missing beside editor: " + worker_exe.string());
    return false;
  }

  std::filesystem::path cwd = working_directory.empty() ? exe_dir : working_directory;

#  ifdef _WIN32
  std::ostringstream cmd;
  cmd << '"' << worker_exe.string() << '"' << " --job \"" << job_file.string() << '"';
  std::string cmdline = cmd.str();

  SECURITY_ATTRIBUTES sa{};
  sa.nLength = sizeof(sa);
  sa.bInheritHandle = TRUE;
  HANDLE log_handle = INVALID_HANDLE_VALUE;
  if (!log_file.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(log_file.parent_path(), ec);
    log_handle = CreateFileA(log_file.string().c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa,
                             CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  }

  STARTUPINFOA si{};
  si.cb = sizeof(si);
  if (log_handle != INVALID_HANDLE_VALUE) {
    si.dwFlags |= STARTF_USESTDHANDLES;
    si.hStdOutput = log_handle;
    si.hStdError = log_handle;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  }
  PROCESS_INFORMATION pi{};
  std::vector<char> mutable_cmd(cmdline.begin(), cmdline.end());
  mutable_cmd.push_back('\0');

  const BOOL ok = CreateProcessA(nullptr, mutable_cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                                 cwd.string().c_str(), &si, &pi);
  if (log_handle != INVALID_HANDLE_VALUE) {
    CloseHandle(log_handle);
  }
  if (!ok) {
    std::ostringstream err;
    err << "CreateProcess failed GetLastError=" << GetLastError() << " cwd=" << cwd.string();
    SetError(error_message, err.str());
    return false;
  }

  if (!AssignToKillOnCloseJob(pi.hProcess)) {
    EVOENGINE_WARNING("Precompute: AssignProcessToJobObject failed (GetLastError="
                      << GetLastError() << "); worker may keep running if the editor exits.");
  }
  CloseHandle(pi.hThread);
  out_process.handle = pi.hProcess;
  out_process.pid = pi.dwProcessId;
  return true;
#  else
  int log_fd = -1;
  if (!log_file.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(log_file.parent_path(), ec);
    log_fd = open(log_file.string().c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  }

  const pid_t parent_pid = getpid();
  const pid_t child = fork();
  if (child < 0) {
    if (log_fd >= 0) {
      close(log_fd);
    }
    SetError(error_message,
             std::string("fork failed errno=") + std::to_string(errno) + " (" + std::strerror(errno) + ")");
    return false;
  }
  if (child == 0) {
    ChildApplyParentDeathAndExec(worker_exe.string(), job_file.string(), cwd.string(), log_fd, parent_pid);
  }
  if (log_fd >= 0) {
    close(log_fd);
  }
  out_process.pid = child;
  return true;
#  endif
#endif
}

PrecomputeWorkerPollResult PollPrecomputeWorkerProcess(PrecomputeWorkerProcess& process) {
  PrecomputeWorkerPollResult result{};
  if (!process.Valid()) {
    result.status = PrecomputeWorkerPollStatus::Lost;
    return result;
  }

#ifdef _WIN32
  DWORD exit_code = STILL_ACTIVE;
  if (!GetExitCodeProcess(static_cast<HANDLE>(process.handle), &exit_code)) {
    process.Reset();
    result.status = PrecomputeWorkerPollStatus::Lost;
    return result;
  }
  if (exit_code == STILL_ACTIVE) {
    result.status = PrecomputeWorkerPollStatus::StillRunning;
    return result;
  }
  result.status = PrecomputeWorkerPollStatus::Exited;
  result.exit_code = static_cast<int>(exit_code);
  process.Reset();
  return result;
#elif defined(__linux__)
  int status = 0;
  const pid_t waited = waitpid(process.pid, &status, WNOHANG);
  if (waited == 0) {
    result.status = PrecomputeWorkerPollStatus::StillRunning;
    return result;
  }
  if (waited < 0) {
    process.Reset();
    result.status = PrecomputeWorkerPollStatus::Lost;
    return result;
  }
  process.Reset();
  result.status = PrecomputeWorkerPollStatus::Exited;
  if (WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    result.exit_code = 128 + WTERMSIG(status);
  } else {
    result.exit_code = 1;
  }
  return result;
#else
  process.Reset();
  result.status = PrecomputeWorkerPollStatus::Lost;
  return result;
#endif
}

}  // namespace eco_sys_lab_package
