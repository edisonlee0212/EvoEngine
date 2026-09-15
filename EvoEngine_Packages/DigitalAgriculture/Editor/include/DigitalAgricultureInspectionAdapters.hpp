#pragma once
#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"

#include "InspectorRegistry.hpp"

namespace digital_agriculture_package {
class CBTFGroup;
class CBTFImporter;
class PARSensorGroup;
class SkyIlluminance;
class Sorghum;
class SorghumCoordinates;
class SorghumDescriptor;
class SorghumField;
class SorghumGenerator;
class SorghumGrowthStages;
class SorghumLayer;
class SorghumState;
struct SorghumMeshGeneratorSettings;
class SorghumPanicleDescriptor;
class SorghumStemDescriptor;
class SorghumLeafDescriptor;
struct SorghumPanicleState;
struct SorghumStemState;
struct SorghumLeafState;

bool DrawSorghumMeshGeneratorSettingsGui(SorghumMeshGeneratorSettings& settings);
bool DrawSorghumPanicleDescriptorGui(SorghumPanicleDescriptor& descriptor);
bool DrawSorghumStemDescriptorGui(SorghumStemDescriptor& descriptor);
bool DrawSorghumLeafDescriptorGui(SorghumLeafDescriptor& descriptor);
bool DrawSorghumPanicleStateGui(SorghumPanicleState& state);
bool DrawSorghumStemStateGui(SorghumStemState& state, int mode);
bool DrawSorghumLeafStateGui(SorghumLeafState& state, int mode);
bool DrawSorghumStateGui(SorghumState& state, int mode);

bool InspectSorghumCoordinates(evo_engine::InspectorContext& context, SorghumCoordinates& coordinates);
bool InspectSorghumDescriptor(evo_engine::InspectorContext& context, SorghumDescriptor& descriptor);

bool InspectSorghumLayer(evo_engine::InspectorContext& context, SorghumLayer& layer);

bool InspectCBTFImporter(evo_engine::InspectorContext& context, CBTFImporter& importer);

bool InspectBtfMaterial(evo_engine::InspectorContext& context, evo_engine::BtfMaterial& target);
bool InspectBtfMeshRenderer(evo_engine::InspectorContext& context, evo_engine::BtfMeshRenderer& target);
}  // namespace digital_agriculture_package
