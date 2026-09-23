#include "NativeBuildIdentity.hpp"

#include "EvoEngineBuildIdentity.hpp"

#include <string_view>

using namespace evo_engine;

namespace {
const NativeBuildIdentity identity{EVOENGINE_SDK_SOURCE_ID,           EVOENGINE_NATIVE_COMPILER_ID,
                                   EVOENGINE_NATIVE_COMPILER_VERSION, EVOENGINE_NATIVE_BUILD_CONFIGURATION,
                                   EVOENGINE_NATIVE_PLATFORM,         EVOENGINE_NATIVE_ARCHITECTURE};

bool Match(const char* field, const char* expected, const char* candidate, std::string* reason) {
  const std::string_view expected_value = expected ? expected : "";
  const std::string_view candidate_value = candidate ? candidate : "";
  if (expected_value.empty() || candidate_value.empty()) {
    if (reason) {
      *reason = std::string(field) + " is missing";
    }
    return false;
  }
  if (expected_value == candidate_value) {
    return true;
  }
  if (reason) {
    *reason = std::string(field) + " mismatch: expected [" + std::string(expected_value) + "], found [" +
              std::string(candidate_value) + "]";
  }
  return false;
}
}  // namespace

const NativeBuildIdentity& evo_engine::GetNativeBuildIdentity() {
  return identity;
}

bool evo_engine::IsNativeBuildCompatible(const NativeBuildIdentity& expected, const NativeBuildIdentity& candidate,
                                         std::string* reason) {
  if (!Match("SDK source ID", expected.sdk_source_id, candidate.sdk_source_id, reason) ||
      !Match("compiler", expected.compiler_id, candidate.compiler_id, reason) ||
      !Match("compiler version", expected.compiler_version, candidate.compiler_version, reason) ||
      !Match("configuration", expected.configuration, candidate.configuration, reason) ||
      !Match("platform", expected.platform, candidate.platform, reason) ||
      !Match("architecture", expected.architecture, candidate.architecture, reason)) {
    return false;
  }
  return true;
}
