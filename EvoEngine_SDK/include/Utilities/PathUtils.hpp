#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace evo_engine::path_utils {
[[nodiscard]] EVOENGINE_API std::filesystem::path CurrentExecutablePath(
    const std::filesystem::path& fallback_path = {});
[[nodiscard]] EVOENGINE_API std::filesystem::path NormalizeAbsolutePath(const std::filesystem::path& path);
[[nodiscard]] EVOENGINE_API std::filesystem::path NormalizePathForContainment(const std::filesystem::path& path);
EVOENGINE_API void AddUniqueNormalizedPath(std::vector<std::filesystem::path>& paths,
                                           const std::filesystem::path& path);
[[nodiscard]] EVOENGINE_API std::filesystem::path FindExistingPath(
    const std::vector<std::filesystem::path>& candidates);
[[nodiscard]] EVOENGINE_API std::filesystem::path FindAncestorChildPath(const std::filesystem::path& child_path,
                                                                        const std::filesystem::path& start_path,
                                                                        size_t max_depth);
[[nodiscard]] EVOENGINE_API bool IsSameOrChildPath(const std::filesystem::path& path,
                                                   const std::filesystem::path& parent);
[[nodiscard]] EVOENGINE_API std::optional<std::filesystem::path> RelativePathIfContained(
    const std::filesystem::path& path, const std::filesystem::path& parent);
[[nodiscard]] EVOENGINE_API std::filesystem::path GenerateUniquePath(const std::filesystem::path& stem,
                                                                     const std::string& postfix);
[[nodiscard]] EVOENGINE_API std::filesystem::path GenerateUniqueChildPath(const std::filesystem::path& root,
                                                                          const std::filesystem::path& child_stem,
                                                                          const std::string& postfix);
[[nodiscard]] EVOENGINE_API bool IsValidFileName(const std::string& file_name);
}  // namespace evo_engine::path_utils
