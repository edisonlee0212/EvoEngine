#pragma once

#include "InspectorRegistry.hpp"

namespace universe_package {
class PlanetTerrain;
class UniverseLayer;

bool InspectPlanetTerrain(evo_engine::InspectorContext& context, PlanetTerrain& planet_terrain);
bool InspectUniverseLayer(evo_engine::InspectorContext& context, UniverseLayer& layer);
}  // namespace universe_package
