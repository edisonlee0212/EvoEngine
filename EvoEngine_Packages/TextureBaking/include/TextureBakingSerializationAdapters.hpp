#pragma once

#include "TextureBaking.hpp"

namespace texture_baking_package {
void SerializeTextureBaking(YAML::Emitter& out, const TextureBaking& target);
void DeserializeTextureBaking(const YAML::Node& in, TextureBaking& target);
}  // namespace texture_baking_package
