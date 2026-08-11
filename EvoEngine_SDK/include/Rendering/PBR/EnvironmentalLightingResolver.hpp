#pragma once

#include "DdgiRuntime.hpp"
#include "EnvironmentalLighting.hpp"
#include "ResolvedEnvironmentalLighting.hpp"

#include <cstdint>
#include <memory>

namespace evo_engine {

class Scene;

[[nodiscard]] ResolvedEnvironmentalLighting ResolveEnvironmentalLighting(const std::shared_ptr<Scene>& scene);
[[nodiscard]] std::vector<DdgiVolumeRuntimeInfo> CollectDdgiVolumeRuntimeInfos(
    const ResolvedEnvironmentalLighting& lighting);

}  // namespace evo_engine
