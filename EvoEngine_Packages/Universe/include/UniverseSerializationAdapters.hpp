#pragma once

#include "PlanetTerrain.hpp"
#include "StarCluster.hpp"

namespace universe_package {
void SerializePlanetTerrain(YAML::Emitter& out, const PlanetTerrain& target);
void DeserializePlanetTerrain(const YAML::Node& in, PlanetTerrain& target);
void SerializeStarCluster(YAML::Emitter& out, const StarCluster& target);
void DeserializeStarCluster(const YAML::Node& in, StarCluster& target);
}  // namespace universe_package
