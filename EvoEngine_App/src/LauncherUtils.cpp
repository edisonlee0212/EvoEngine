#include "LauncherUtils.hpp"
#include "PathUtils.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>

using namespace evo_engine;

namespace evo_engine::launcher {
ProjectLaunchMetadata BuildProjectLaunchMetadata(const std::string& project_name,
                                                 const std::vector<std::string>& startup_runtime_packages) {
  ProjectLaunchMetadata metadata;
  metadata.application_name = project_name;
  metadata.preferred_editor = "EvoEngineEditor";
  metadata.startup_runtime_packages = startup_runtime_packages;
  return metadata;
}

std::string Trim(const std::string& value) {
  const auto begin = std::find_if_not(value.begin(), value.end(), [](const unsigned char c) {
    return std::isspace(c);
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](const unsigned char c) {
                     return std::isspace(c);
                   }).base();
  if (begin >= end) {
    return "";
  }
  return {begin, end};
}

bool IsValidProjectName(const std::string& project_name) {
  return path_utils::IsValidFileName(project_name);
}

std::string JoinPackages(const std::vector<std::string>& package_names) {
  std::string text;
  for (const auto& package_name : package_names) {
    if (!text.empty()) {
      text += ", ";
    }
    text += package_name;
  }
  return text;
}

PackageAvailability BuildPackageAvailability(const std::vector<AvailablePackageInfo>& packages) {
  PackageAvailability availability;
  for (const auto& package : packages) {
    availability[package.name] = package.library_exists;
  }
  return availability;
}

bool IsPackageAvailable(const PackageAvailability& availability, const std::string& package_name) {
  const auto search = availability.find(package_name);
  return search != availability.end() && search->second;
}

std::vector<std::string> MissingPackages(const PackageAvailability& availability,
                                         const std::vector<std::string>& package_names) {
  std::vector<std::string> missing_packages;
  for (const auto& package_name : package_names) {
    if (!IsPackageAvailable(availability, package_name)) {
      missing_packages.emplace_back(package_name);
    }
  }
  return missing_packages;
}

bool ArePackagesAvailable(const PackageAvailability& availability, const std::vector<std::string>& package_names) {
  return MissingPackages(availability, package_names).empty();
}

DerivedProjectPath BuildDerivedProjectPath(const std::filesystem::path& parent_folder,
                                           const std::string& project_name) {
  const auto project_folder = parent_folder / project_name;
  return {project_folder, project_folder / (project_name + ".eveproj")};
}

std::string ValidateCreateProjectRequest(const std::string& project_name, const std::filesystem::path& parent_folder,
                                         const std::filesystem::path& project_folder,
                                         const std::filesystem::path& project_path,
                                         const ProjectLaunchMetadata& metadata,
                                         const PackageAvailability& availability) {
  if (!IsValidProjectName(project_name)) {
    return "Project name is empty or contains invalid filename characters.";
  }
  if (!ArePackagesAvailable(availability, metadata.startup_runtime_packages)) {
    return "Selected runtime packages are missing.";
  }
  if (parent_folder.empty() || !std::filesystem::exists(parent_folder) ||
      !std::filesystem::is_directory(parent_folder)) {
    return "Select an existing parent folder.";
  }
  if (std::filesystem::exists(project_folder)) {
    return "Project folder already exists.";
  }
  if (std::filesystem::exists(project_path)) {
    return "Project file already exists.";
  }
  return "";
}

std::filesystem::path NormalizeProjectPath(const std::filesystem::path& path) {
  return path_utils::NormalizeAbsolutePath(path);
}

std::vector<std::filesystem::path> LoadRecentProjects(const std::filesystem::path& settings_path, bool& pruned,
                                                      const size_t max_count) {
  std::vector<std::filesystem::path> recent_project_paths;
  pruned = false;
  if (!std::filesystem::exists(settings_path)) {
    return recent_project_paths;
  }

  try {
    const auto in = YAML::LoadFile(settings_path.string());
    const auto recent_projects = in["recent_projects"];
    if (!recent_projects || !recent_projects.IsSequence()) {
      return recent_project_paths;
    }
    for (const auto& entry : recent_projects) {
      if (!entry.IsScalar()) {
        pruned = true;
        continue;
      }
      const auto path = NormalizeProjectPath(entry.as<std::string>());
      if (path.extension() != ".eveproj" || !std::filesystem::exists(path) || std::filesystem::is_directory(path)) {
        pruned = true;
        continue;
      }
      if (std::find(recent_project_paths.begin(), recent_project_paths.end(), path) == recent_project_paths.end()) {
        recent_project_paths.emplace_back(path);
      } else {
        pruned = true;
      }
      if (recent_project_paths.size() >= max_count) {
        break;
      }
    }
  } catch (const std::exception&) {
    recent_project_paths.clear();
  }
  return recent_project_paths;
}

void SaveRecentProjects(const std::filesystem::path& settings_path,
                        const std::vector<std::filesystem::path>& recent_project_paths) {
  if (!settings_path.parent_path().empty()) {
    std::filesystem::create_directories(settings_path.parent_path());
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "recent_projects" << YAML::Value << YAML::BeginSeq;
  for (const auto& path : recent_project_paths) {
    out << path.string();
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream file_out(settings_path.string());
  file_out << out.c_str();
}

void AddRecentProject(std::vector<std::filesystem::path>& recent_project_paths, const std::filesystem::path& path,
                      const size_t max_count) {
  const auto project_path = NormalizeProjectPath(path);
  if (project_path.extension() != ".eveproj" || std::filesystem::is_directory(project_path)) {
    return;
  }

  recent_project_paths.erase(std::remove(recent_project_paths.begin(), recent_project_paths.end(), project_path),
                             recent_project_paths.end());
  recent_project_paths.insert(recent_project_paths.begin(), project_path);
  if (recent_project_paths.size() > max_count) {
    recent_project_paths.resize(max_count);
  }
}
}  // namespace evo_engine::launcher
