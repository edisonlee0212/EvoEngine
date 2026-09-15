#include "DemoAuthoringCommandLine.hpp"

#include <charconv>
#include <stdexcept>

namespace evo_engine {
DemoAuthoringArguments ParseDemoAuthoringArguments(const std::vector<std::string>& arguments) {
  DemoAuthoringArguments result;
  for (size_t index = 0; index < arguments.size(); ++index) {
    if (arguments[index] == "--author-runtime-scene") {
      result.enabled = true;
    } else if (arguments[index] == "--resource-root") {
      if (++index == arguments.size())
        throw std::invalid_argument("--resource-root requires a path.");
      result.resource_root = std::filesystem::absolute(std::filesystem::u8path(arguments[index]));
    } else if (arguments[index] == "--author-warmup-frames") {
      if (++index == arguments.size())
        throw std::invalid_argument("--author-warmup-frames requires a non-negative integer.");
      size_t count = 0;
      const auto& value = arguments[index];
      const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), count);
      if (error != std::errc() || end != value.data() + value.size())
        throw std::invalid_argument("--author-warmup-frames requires a non-negative integer.");
      result.warmup_frames = count;
    }
  }
  return result;
}

void ValidateDemoAuthoringArguments(const DemoAuthoringArguments& arguments, const bool has_demo,
                                    const bool incompatible_mode) {
  if (arguments.enabled && (!has_demo || !arguments.resource_root))
    throw std::invalid_argument(
        "--author-runtime-scene requires --demo <id> and --resource-root <isolated Resources path>.");
  if (arguments.resource_root &&
      (!std::filesystem::is_directory(*arguments.resource_root) || arguments.resource_root->filename() != "Resources"))
    throw std::invalid_argument("--resource-root must name an existing Resources directory.");
  if (arguments.warmup_frames && !arguments.enabled)
    throw std::invalid_argument("--author-warmup-frames requires --author-runtime-scene.");
  if (arguments.enabled && incompatible_mode)
    throw std::invalid_argument("--author-runtime-scene cannot be combined with capture or smoke modes.");
}
}  // namespace evo_engine
