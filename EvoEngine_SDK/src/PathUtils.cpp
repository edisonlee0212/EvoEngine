#include "PathUtils.hpp"

#include <algorithm>
#include <cctype>
#include <string_view>
#include <system_error>

#if defined(_WIN32)
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
bool PathElementEquals(const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  auto lhs_string = lhs.string();
  auto rhs_string = rhs.string();
  std::transform(lhs_string.begin(), lhs_string.end(), lhs_string.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::transform(rhs_string.begin(), rhs_string.end(), rhs_string.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return lhs_string == rhs_string;
#else
  return lhs == rhs;
#endif
}

std::filesystem::path CombineStemPostfix(const std::filesystem::path& stem, const std::string& postfix) {
  return stem.string() + postfix;
}
}  // namespace

std::filesystem::path path_utils::CurrentExecutablePath(const std::filesystem::path& fallback_path) {
#if defined(_WIN32)
  std::vector<wchar_t> buffer(32768);
  const auto size = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
  if (size > 0 && size < buffer.size()) {
    return NormalizeAbsolutePath(std::filesystem::path(buffer.data(), buffer.data() + size));
  }
#elif defined(__linux__)
  std::error_code error;
  const auto executable_path = std::filesystem::read_symlink("/proc/self/exe", error);
  if (!error) {
    return NormalizeAbsolutePath(executable_path);
  }
#endif
  return fallback_path.empty() ? std::filesystem::path() : NormalizeAbsolutePath(fallback_path);
}

std::filesystem::path path_utils::NormalizeAbsolutePath(const std::filesystem::path& path) {
  return std::filesystem::absolute(path).lexically_normal();
}

std::filesystem::path path_utils::NormalizePathForContainment(const std::filesystem::path& path) {
  std::error_code error;
  auto normalized = std::filesystem::weakly_canonical(path, error);
  if (error) {
    normalized = std::filesystem::absolute(path, error);
  }
  if (error) {
    normalized = path;
  }
  return normalized.lexically_normal();
}

void path_utils::AddUniqueNormalizedPath(std::vector<std::filesystem::path>& paths, const std::filesystem::path& path) {
  const auto normalized_path = NormalizeAbsolutePath(path);
  for (const auto& existing : paths) {
    if (IsSameOrChildPath(existing, normalized_path) && IsSameOrChildPath(normalized_path, existing)) {
      return;
    }
  }
  paths.emplace_back(normalized_path);
}

std::filesystem::path path_utils::FindExistingPath(const std::vector<std::filesystem::path>& candidates) {
  for (const auto& candidate : candidates) {
    std::error_code error;
    if (std::filesystem::exists(candidate, error)) {
      return NormalizeAbsolutePath(candidate);
    }
  }
  return {};
}

std::filesystem::path path_utils::FindAncestorChildPath(const std::filesystem::path& child_path,
                                                        const std::filesystem::path& start_path,
                                                        const size_t max_depth) {
  auto current_path = NormalizeAbsolutePath(start_path);
  for (size_t depth = 0; depth <= max_depth && !current_path.empty(); ++depth) {
    const auto candidate = current_path / child_path;
    std::error_code error;
    if (std::filesystem::exists(candidate, error)) {
      return NormalizeAbsolutePath(candidate);
    }
    const auto parent_path = current_path.parent_path();
    if (parent_path.empty() || parent_path == current_path) {
      break;
    }
    current_path = parent_path;
  }
  return {};
}

bool path_utils::IsSameOrChildPath(const std::filesystem::path& path, const std::filesystem::path& parent) {
  const auto normalized_path = NormalizePathForContainment(path);
  const auto normalized_parent = NormalizePathForContainment(parent);
  auto path_iterator = normalized_path.begin();
  for (auto parent_iterator = normalized_parent.begin(); parent_iterator != normalized_parent.end();
       ++parent_iterator, ++path_iterator) {
    if (path_iterator == normalized_path.end() || !PathElementEquals(*path_iterator, *parent_iterator)) {
      return false;
    }
  }
  return true;
}

std::optional<std::filesystem::path> path_utils::RelativePathIfContained(const std::filesystem::path& path,
                                                                         const std::filesystem::path& parent) {
  if (!IsSameOrChildPath(path, parent)) {
    return std::nullopt;
  }
  std::error_code error;
  auto relative_path =
      std::filesystem::relative(NormalizePathForContainment(path), NormalizePathForContainment(parent), error);
  if (error) {
    relative_path = std::filesystem::relative(path, parent, error);
  }
  if (error) {
    return std::nullopt;
  }
  return relative_path;
}

std::filesystem::path path_utils::GenerateUniquePath(const std::filesystem::path& stem, const std::string& postfix) {
  auto candidate = CombineStemPostfix(stem, postfix);
  int index = 0;
  while (std::filesystem::exists(candidate)) {
    ++index;
    candidate = CombineStemPostfix(stem, " (" + std::to_string(index) + ")" + postfix);
  }
  return candidate;
}

std::filesystem::path path_utils::GenerateUniqueChildPath(const std::filesystem::path& root,
                                                          const std::filesystem::path& child_stem,
                                                          const std::string& postfix) {
  auto candidate = CombineStemPostfix(child_stem, postfix);
  int index = 0;
  while (std::filesystem::exists(root / candidate)) {
    ++index;
    candidate = CombineStemPostfix(child_stem, " (" + std::to_string(index) + ")" + postfix);
  }
  return candidate;
}

bool path_utils::IsValidFileName(const std::string& file_name) {
  if (file_name.empty()) {
    return false;
  }
  static constexpr std::string_view invalid_chars = R"(<>:"/\|?*)";
  return file_name.find_first_of(invalid_chars) == std::string::npos;
}
