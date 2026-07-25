#pragma once

#include "EnvironmentalLighting.hpp"
#include "ResolvedEnvironmentalLighting.hpp"

#include <cstdint>
#include <memory>

namespace evo_engine {

class Scene;

[[nodiscard]] ResolvedEnvironmentalLighting ResolveEnvironmentalLighting(const std::shared_ptr<Scene>& scene);

}  // namespace evo_engine
