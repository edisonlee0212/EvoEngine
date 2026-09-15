#pragma once

#include <filesystem>
#include <string>
#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"
namespace evo_engine {
class EVOENGINE_EDITOR_API RuntimeExportJob {
 public:
  RuntimeExportJob() = default;
  ~RuntimeExportJob();
  RuntimeExportJob(const RuntimeExportJob&) = delete;
  RuntimeExportJob& operator=(const RuntimeExportJob&) = delete;
  bool Start(const std::filesystem::path& executable, const std::filesystem::path& request,
             const std::filesystem::path& runtime_template, const std::filesystem::path& output, std::string& error);
  void Poll();
  void Wait();
  [[nodiscard]] bool Active() const;
  [[nodiscard]] bool Finished() const;
  [[nodiscard]] bool Succeeded() const;
  [[nodiscard]] const std::string& Output() const;

 private:
  void* process_ = nullptr;
  void* pipe_ = nullptr;
  std::string output_;
  bool finished_ = false;
  unsigned long exit_code_ = 1;
  std::filesystem::path request_directory_;
  void DrainOutput();
  void CleanupRequest();
};
}  // namespace evo_engine
