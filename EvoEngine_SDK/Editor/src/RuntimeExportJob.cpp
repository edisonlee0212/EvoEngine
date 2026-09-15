#include "RuntimeExportJob.hpp"
#ifdef _WIN32
#  include <windows.h>
#endif
#include <vector>
namespace evo_engine {
namespace {
#ifdef _WIN32
std::wstring Quote(const std::filesystem::path& path) {
  const auto value = path.wstring();
  std::wstring result = L"\"";
  size_t slashes = 0;
  for (const auto character : value) {
    if (character == L'\\') {
      ++slashes;
      continue;
    }
    if (character == L'\"')
      result.append(slashes * 2 + 1, L'\\');
    else
      result.append(slashes, L'\\');
    slashes = 0;
    result.push_back(character);
  }
  result.append(slashes * 2, L'\\');
  result.push_back(L'\"');
  return result;
}
#endif
void CleanupOwnedRequestDirectory(const std::filesystem::path& directory) {
  if (directory.empty())
    return;
  std::error_code error;
#ifdef _WIN32
  const auto attributes = GetFileAttributesW(directory.c_str());
  if (attributes == INVALID_FILE_ATTRIBUTES || (attributes & FILE_ATTRIBUTE_REPARSE_POINT))
    return;
#else
  if (std::filesystem::is_symlink(std::filesystem::symlink_status(directory, error)) || error)
    return;
#endif
  if (std::filesystem::is_regular_file(directory / ".evoengine-build-request", error) && !error)
    std::filesystem::remove_all(directory, error);
}
}  // namespace
RuntimeExportJob::~RuntimeExportJob() {
  Wait();
}
bool RuntimeExportJob::Start(const std::filesystem::path& executable, const std::filesystem::path& request,
                             const std::filesystem::path& runtime_template, const std::filesystem::path& output,
                             std::string& error) {
#ifdef _WIN32
  if (Active()) {
    error = "Runtime exporter is busy.";
    CleanupOwnedRequestDirectory(request.parent_path());
    return false;
  }
  output_.clear();
  request_directory_ = request.parent_path();
  finished_ = false;
  exit_code_ = 1;
  std::error_code file_error;
  if (!std::filesystem::is_regular_file(executable, file_error) || file_error) {
    error = "Runtime exporter is missing.";
    CleanupRequest();
    return false;
  }
  SECURITY_ATTRIBUTES attributes{sizeof(attributes), nullptr, TRUE};
  HANDLE read_pipe = nullptr, write_pipe = nullptr;
  if (!CreatePipe(&read_pipe, &write_pipe, &attributes, 0)) {
    error = "Could not create exporter output pipe.";
    CleanupRequest();
    return false;
  }
  if (!SetHandleInformation(read_pipe, HANDLE_FLAG_INHERIT, 0)) {
    CloseHandle(read_pipe);
    CloseHandle(write_pipe);
    error = "Could not secure exporter output pipe.";
    CleanupRequest();
    return false;
  }
  auto command = Quote(executable) + L" --request " + Quote(request) + L" --template " + Quote(runtime_template) +
                 L" --output " + Quote(output);
  std::vector<wchar_t> mutable_command(command.begin(), command.end());
  mutable_command.push_back(0);
  STARTUPINFOW startup{sizeof(startup)};
  startup.dwFlags = STARTF_USESTDHANDLES;
  startup.hStdOutput = write_pipe;
  startup.hStdError = write_pipe;
  startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  PROCESS_INFORMATION process{};
  const auto working = executable.parent_path().wstring();
  const bool created = CreateProcessW(nullptr, mutable_command.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW,
                                      nullptr, working.c_str(), &startup, &process) != FALSE;
  CloseHandle(write_pipe);
  if (!created) {
    CloseHandle(read_pipe);
    error = "Could not start installed runtime exporter.";
    CleanupRequest();
    return false;
  }
  CloseHandle(process.hThread);
  process_ = process.hProcess;
  pipe_ = read_pipe;
  return true;
#else
  request_directory_ = request.parent_path();
  error = "Runtime application export is supported on Windows x64.";
  CleanupRequest();
  return false;
#endif
}
void RuntimeExportJob::DrainOutput() {
#ifdef _WIN32
  if (!pipe_)
    return;
  DWORD available = 0;
  while (PeekNamedPipe(static_cast<HANDLE>(pipe_), nullptr, 0, nullptr, &available, nullptr) && available) {
    char buffer[4096];
    DWORD read = 0;
    if (!ReadFile(static_cast<HANDLE>(pipe_), buffer, (std::min<DWORD>)(available, sizeof(buffer)), &read, nullptr))
      break;
    output_.append(buffer, read);
  }
#endif
}
void RuntimeExportJob::CleanupRequest() {
  if (request_directory_.empty())
    return;
  CleanupOwnedRequestDirectory(request_directory_);
  request_directory_.clear();
}
void RuntimeExportJob::Wait() {
#ifdef _WIN32
  while (process_ && WaitForSingleObject(static_cast<HANDLE>(process_), 10) == WAIT_TIMEOUT)
    DrainOutput();
  if (process_) {
    DrainOutput();
    DWORD code = 1;
    GetExitCodeProcess(static_cast<HANDLE>(process_), &code);
    exit_code_ = code;
    finished_ = true;
    if (pipe_)
      CloseHandle(static_cast<HANDLE>(pipe_));
    pipe_ = nullptr;
    CloseHandle(static_cast<HANDLE>(process_));
    process_ = nullptr;
  }
#endif
  CleanupRequest();
}
void RuntimeExportJob::Poll() {
#ifdef _WIN32
  if (!process_ || finished_)
    return;
  DrainOutput();
  DWORD code = STILL_ACTIVE;
  if (GetExitCodeProcess(static_cast<HANDLE>(process_), &code) && code != STILL_ACTIVE) {
    DrainOutput();
    exit_code_ = code;
    finished_ = true;
    CloseHandle(static_cast<HANDLE>(pipe_));
    pipe_ = nullptr;
    CloseHandle(static_cast<HANDLE>(process_));
    process_ = nullptr;
    CleanupRequest();
  }
#endif
}
bool RuntimeExportJob::Active() const {
  return process_ && !finished_;
}
bool RuntimeExportJob::Finished() const {
  return finished_;
}
bool RuntimeExportJob::Succeeded() const {
  return finished_ && exit_code_ == 0;
}
const std::string& RuntimeExportJob::Output() const {
  return output_;
}
}  // namespace evo_engine
