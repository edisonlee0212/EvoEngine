#pragma once

#include <filesystem>
#include "EvoEngineAPI.hpp"

namespace evo_engine::runtime_paths {
EVOENGINE_API bool IsStrict();
EVOENGINE_API std::filesystem::path Root();
EVOENGINE_API std::filesystem::path Resolve(const std::filesystem::path& relative_path);
EVOENGINE_API void PrepareWritableDirectories();
}  // namespace evo_engine::runtime_paths
