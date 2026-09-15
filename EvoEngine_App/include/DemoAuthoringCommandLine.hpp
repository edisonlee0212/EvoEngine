#pragma once
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace evo_engine {
struct DemoAuthoringArguments {
  bool enabled = false;
  std::optional<std::filesystem::path> resource_root;
  size_t warmup_frames = 0;
};
DemoAuthoringArguments ParseDemoAuthoringArguments(const std::vector<std::string>& arguments);
void ValidateDemoAuthoringArguments(const DemoAuthoringArguments& arguments, bool has_demo, bool incompatible_mode);
}  // namespace evo_engine
