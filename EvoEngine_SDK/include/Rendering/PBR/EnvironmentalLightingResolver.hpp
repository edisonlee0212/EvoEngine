#pragma once

#include "DdgiRuntime.hpp"
#include "EnvironmentalLighting.hpp"
#include "ResolvedEnvironmentalLighting.hpp"

#include <cstdint>
#include <memory>

namespace evo_engine {

class EVOENGINE_API Scene;

[[nodiscard]] EVOENGINE_API ResolvedEnvironmentalLighting
ResolveEnvironmentalLighting(const std::shared_ptr<Scene>& scene);
[[nodiscard]] EVOENGINE_API std::vector<DdgiCascadeRuntimeInfo> CollectDdgiCascadeRuntimeInfos(
    const ResolvedEnvironmentalLighting& lighting);

}  // namespace evo_engine
