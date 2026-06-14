#pragma once

#include "LSystemDescriptor.hpp"
#include "LSystemLayer.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"

namespace l_system_package {
void SerializeLSystemDescriptor(YAML::Emitter& out, const LSystemDescriptor& target);
void DeserializeLSystemDescriptor(const YAML::Node& in, LSystemDescriptor& target);
void SerializeScotsPineDescriptor(YAML::Emitter& out, const ScotsPineDescriptor& target);
void DeserializeScotsPineDescriptor(const YAML::Node& in, ScotsPineDescriptor& target);
void SerializeScotsPine(YAML::Emitter& out, const ScotsPine& target);
void DeserializeScotsPine(const YAML::Node& in, ScotsPine& target);
void SerializeLSystemLayer(YAML::Emitter& out, const LSystemLayer& target);
void DeserializeLSystemLayer(const YAML::Node& in, LSystemLayer& target);
}  // namespace l_system_package
