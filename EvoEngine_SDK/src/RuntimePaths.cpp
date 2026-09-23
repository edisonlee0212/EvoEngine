#include "RuntimePaths.hpp"
#include "Application.hpp"
#include "PathUtils.hpp"

#include <fstream>
#include <stdexcept>

namespace evo_engine::runtime_paths {
bool IsStrict() {
  const auto application = ApplicationContext::TryGet();
  return application && application->GetApplicationInfo().strict_runtime;
}

std::filesystem::path Root() {
  const auto executable = path_utils::CurrentExecutablePath();
  if (executable.empty()) {
    throw std::runtime_error("Cannot locate the runtime executable.");
  }
  return executable.parent_path();
}

std::filesystem::path Resolve(const std::filesystem::path& relative_path) {
  if (relative_path.empty() || relative_path.has_root_path()) {
    throw std::runtime_error("Runtime paths must be relative to the application directory.");
  }
  for (const auto& part : relative_path) {
    if (part == "..") {
      throw std::runtime_error("Runtime paths cannot leave the application directory.");
    }
  }
  const auto root = std::filesystem::weakly_canonical(Root());
  const auto resolved = std::filesystem::weakly_canonical(root / relative_path);
  if (!path_utils::IsSameOrChildPath(resolved, root)) {
    throw std::runtime_error("Runtime path points outside the application directory.");
  }
  return resolved;
}

void PrepareWritableDirectories() {
  for (const auto* name : {"Logs", "Cache", "Cache/Scratch", "UserData"}) {
    const auto directory = Resolve(name);
    std::filesystem::create_directories(directory);
    const auto probe = path_utils::GenerateUniqueChildPath(directory, ".write-test", ".tmp");
    {
      std::ofstream stream(probe, std::ios::binary);
      if (!stream || !(stream << "runtime")) {
        throw std::runtime_error("Application directory is not writable: " + directory.string());
      }
    }
    std::filesystem::remove(probe);
  }
}
}  // namespace evo_engine::runtime_paths
