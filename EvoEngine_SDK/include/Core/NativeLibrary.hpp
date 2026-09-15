#pragma once

#include <filesystem>
#include <string>
#include "EvoEngineAPI.hpp"

namespace evo_engine::native_library {
EVOENGINE_API bool OpenLibrary(const std::filesystem::path& path, void*& handle);
EVOENGINE_API void CloseLibrary(void* handle);
EVOENGINE_API void* GetSymbol(void* handle, const char* name);
EVOENGINE_API std::filesystem::path CreateShadowCopy(const std::filesystem::path& source);
EVOENGINE_API bool VerifyLibraryHash(const std::filesystem::path& path, const std::string& expected_sha256);
}  // namespace evo_engine::native_library
