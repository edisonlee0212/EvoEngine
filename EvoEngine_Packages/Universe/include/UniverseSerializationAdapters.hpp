#pragma once

#include "PlanetTerrain.hpp"

namespace universe_package {
void SerializePlanetTerrain(YAML::Emitter& out, const PlanetTerrain& target);
void DeserializePlanetTerrain(const YAML::Node& in, PlanetTerrain& target);
}  // namespace universe_package
