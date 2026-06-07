#pragma once

#include "EvoEngine_SDK_PCH.hpp"
#include "PackageManager.hpp"
#include "ProjectManager.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace evo_engine::launcher {
constexpr size_t kMaxRecentProjectCount = 8;

struct ProjectTemplate {
  std::string name;
  std::vector<std::string> startup_runtime_packages;
};

struct DerivedProjectPath {
  std::filesystem::path folder;
  std::filesystem::path project_file;
};

using PackageAvailability = std::unordered_map<std::string, bool>;

[[nodiscard]] const std::vector<ProjectTemplate>& ProjectTemplates();
[[nodiscard]] ProjectLaunchMetadata BuildProjectLaunchMetadata(const std::string& project_name,
                                                               const ProjectTemplate& project_template);
[[nodiscard]] std::string Trim(const std::string& value);
[[nodiscard]] bool IsValidProjectName(const std::string& project_name);
[[nodiscard]] std::string JoinPackages(const std::vector<std::string>& package_names);
[[nodiscard]] PackageAvailability BuildPackageAvailability(const std::vector<AvailablePackageInfo>& packages);
[[nodiscard]] bool IsPackageAvailable(const PackageAvailability& availability, const std::string& package_name);
[[nodiscard]] std::vector<std::string> MissingPackages(const PackageAvailability& availability,
                                                       const std::vector<std::string>& package_names);
[[nodiscard]] bool ArePackagesAvailable(const PackageAvailability& availability,
                                        const std::vector<std::string>& package_names);
[[nodiscard]] bool IsTemplateAvailable(const ProjectTemplate& project_template,
                                       const PackageAvailability& availability);
[[nodiscard]] int SelectAvailableTemplateIndex(const std::vector<ProjectTemplate>& templates,
                                               const PackageAvailability& availability, int selected_index);
[[nodiscard]] DerivedProjectPath BuildDerivedProjectPath(const std::filesystem::path& parent_folder,
                                                         const std::string& project_name);
[[nodiscard]] std::string ValidateCreateProjectRequest(const std::string& project_name,
                                                       const std::filesystem::path& parent_folder,
                                                       const std::filesystem::path& project_folder,
                                                       const std::filesystem::path& project_path,
                                                       const ProjectLaunchMetadata& metadata,
                                                       const PackageAvailability& availability);
[[nodiscard]] std::filesystem::path NormalizeProjectPath(const std::filesystem::path& path);
[[nodiscard]] std::vector<std::filesystem::path> LoadRecentProjects(const std::filesystem::path& settings_path,
                                                                    bool& pruned,
                                                                    size_t max_count = kMaxRecentProjectCount);
void SaveRecentProjects(const std::filesystem::path& settings_path,
                        const std::vector<std::filesystem::path>& recent_project_paths);
void AddRecentProject(std::vector<std::filesystem::path>& recent_project_paths, const std::filesystem::path& path,
                      size_t max_count = kMaxRecentProjectCount);
}  // namespace evo_engine::launcher
