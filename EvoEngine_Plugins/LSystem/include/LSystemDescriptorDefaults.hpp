#pragma once

#include <ProjectManager.hpp>
#include <filesystem>
#include <string>
#include <yaml-cpp/yaml.h>

namespace l_system_plugin::descriptor_defaults {

template <typename TCandidates>
std::filesystem::path ResolveFirstExistingAbsolutePath(const TCandidates& candidates) {
  for (const auto& relative_candidate : candidates) {
    const auto absolute_candidate = std::filesystem::absolute(relative_candidate);
    if (std::filesystem::exists(absolute_candidate)) {
      return absolute_candidate;
    }
  }
  return {};
}

template <typename TCandidates>
std::filesystem::path ResolveFirstExistingProjectAssetPath(const TCandidates& project_asset_candidates) {
  const auto assets_folder = evo_engine::ProjectManager::GetAssetsFolderPath();
  if (assets_folder.empty()) {
    return {};
  }

  for (const auto& relative_candidate : project_asset_candidates) {
    const auto absolute_candidate = assets_folder / relative_candidate;
    if (std::filesystem::exists(absolute_candidate)) {
      return absolute_candidate;
    }
  }
  return {};
}

template <typename TResourceCandidates, typename TProjectAssetCandidates>
std::filesystem::path ResolveExistingDefaultsPath(const TResourceCandidates& resource_candidates,
                                                  const TProjectAssetCandidates& project_asset_candidates) {
  if (const auto existing_resource_path = ResolveFirstExistingAbsolutePath(resource_candidates);
      !existing_resource_path.empty()) {
    return existing_resource_path;
  }
  return ResolveFirstExistingProjectAssetPath(project_asset_candidates);
}

template <typename TResourceCandidates, typename TProjectAssetCandidates, typename TWritableTemplateCandidates>
std::filesystem::path ResolveWritableDefaultsPath(const TResourceCandidates& resource_candidates,
                                                  const TProjectAssetCandidates& project_asset_candidates,
                                                  const TWritableTemplateCandidates& writable_template_candidates,
                                                  const std::filesystem::path& fallback_relative_path) {
  if (const auto existing = ResolveExistingDefaultsPath(resource_candidates, project_asset_candidates);
      !existing.empty()) {
    return existing;
  }

  for (const auto& candidate : writable_template_candidates) {
    const auto absolute_candidate = std::filesystem::absolute(candidate);
    const auto parent = absolute_candidate.parent_path();
    if (parent.empty() || std::filesystem::exists(parent)) {
      return absolute_candidate;
    }
  }

  return fallback_relative_path.empty()
             ? std::filesystem::path{}
             : std::filesystem::absolute(fallback_relative_path);
}

bool LoadDefaultsYamlMap(const std::filesystem::path& file_path,
                         YAML::Node& out_defaults,
                         const std::string& descriptor_name_for_logging);

}  // namespace l_system_plugin::descriptor_defaults
