#include "EcoSysLabPaths.hpp"

#include "Console.hpp"
#include "ProjectManager.hpp"

namespace eco_sys_lab_plugin {

std::filesystem::path EcoSysLabMetadataDirectory() {
  std::filesystem::path dir;
  const auto project_path = evo_engine::ProjectManager::GetProjectPath();
  if (!project_path.empty()) {
    dir = project_path.parent_path() / "Metadata";
  } else {
    dir = std::filesystem::path("Metadata");
  }
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    EVOENGINE_ERROR("Failed to create Metadata directory " << dir.string() << " (" << ec.message() << ").");
  }
  return dir;
}

std::filesystem::path EcoSysLabMetadataPath(const std::filesystem::path& filename) {
  return EcoSysLabMetadataDirectory() / filename;
}

}  // namespace eco_sys_lab_plugin
