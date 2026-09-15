#pragma once
#include <yaml-cpp/yaml.h>
#include "ProjectBuildSettings.hpp"
namespace evo_engine {
EVOENGINE_API void SerializeProjectBuildSettings(const ProjectBuildSettings& settings, YAML::Emitter& out);
[[nodiscard]] EVOENGINE_API ProjectBuildSettings DeserializeProjectBuildSettings(const YAML::Node& node);
}  // namespace evo_engine
