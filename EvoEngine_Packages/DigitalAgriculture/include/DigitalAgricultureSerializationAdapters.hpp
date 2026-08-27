#pragma once

#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFGroup.hpp"
#include "PARSensorGroup.hpp"
#include "SkyIlluminance.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumState.hpp"

namespace digital_agriculture_package {
void SerializeSorghumDescriptor(YAML::Emitter& out, const SorghumDescriptor& target);
void DeserializeSorghumDescriptor(const YAML::Node& in, SorghumDescriptor& target);
void SerializeSorghum(YAML::Emitter& out, const Sorghum& target);
void DeserializeSorghum(const YAML::Node& in, Sorghum& target);
void SerializeSorghumGrowthStages(YAML::Emitter& out, const SorghumGrowthStages& target);
void DeserializeSorghumGrowthStages(const YAML::Node& in, SorghumGrowthStages& target);
void SerializeSorghumState(YAML::Emitter& out, const SorghumState& target);
void DeserializeSorghumState(const YAML::Node& in, SorghumState& target);
void SerializeSorghumGenerator(YAML::Emitter& out, const SorghumGenerator& target);
void DeserializeSorghumGenerator(const YAML::Node& in, SorghumGenerator& target);
void SerializeSorghumField(YAML::Emitter& out, const SorghumField& target);
void DeserializeSorghumField(const YAML::Node& in, SorghumField& target);
void SerializePARSensorGroup(YAML::Emitter& out, const PARSensorGroup& target);
void DeserializePARSensorGroup(const YAML::Node& in, PARSensorGroup& target);
void SerializeCBTFGroup(YAML::Emitter& out, const CBTFGroup& target);
void DeserializeCBTFGroup(const YAML::Node& in, CBTFGroup& target);
void SerializeSkyIlluminance(YAML::Emitter& out, const SkyIlluminance& target);
void DeserializeSkyIlluminance(const YAML::Node& in, SkyIlluminance& target);
void SerializeSorghumCoordinates(YAML::Emitter& out, const SorghumCoordinates& target);
void DeserializeSorghumCoordinates(const YAML::Node& in, SorghumCoordinates& target);
}  // namespace digital_agriculture_package

namespace evo_engine {
void SerializeBtfMeshRenderer(YAML::Emitter& out, const BtfMeshRenderer& target);
void DeserializeBtfMeshRenderer(const YAML::Node& in, BtfMeshRenderer& target);
void SerializeBtfMaterial(YAML::Emitter& out, const BtfMaterial& target);
void DeserializeBtfMaterial(const YAML::Node& in, BtfMaterial& target);
}  // namespace evo_engine
