#pragma once

#include "EvoEngineAPI.hpp"

#include <string>

namespace evo_engine {
struct NativeBuildIdentity {
  const char* sdk_source_id = nullptr;
  const char* compiler_id = nullptr;
  const char* compiler_version = nullptr;
  const char* configuration = nullptr;
  const char* platform = nullptr;
  const char* architecture = nullptr;
  bool with_editor = false;  // Build composition metadata; runtime ABI compatibility is independent of the editor.
};

EVOENGINE_API const NativeBuildIdentity& GetNativeBuildIdentity();
EVOENGINE_API bool IsNativeBuildCompatible(const NativeBuildIdentity& expected, const NativeBuildIdentity& candidate,
                                           std::string* reason = nullptr);
}  // namespace evo_engine
