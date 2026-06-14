#pragma once

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

bool InspectSkyIlluminance(evo_engine::InspectorContext& context, SkyIlluminance& illuminance);
bool InspectSorghum(evo_engine::InspectorContext& context, Sorghum& sorghum);
bool InspectSorghumCoordinates(evo_engine::InspectorContext& context, SorghumCoordinates& coordinates);
bool InspectSorghumDescriptor(evo_engine::InspectorContext& context, SorghumDescriptor& descriptor);
bool InspectSorghumField(evo_engine::InspectorContext& context, SorghumField& field);
bool InspectSorghumGenerator(evo_engine::InspectorContext& context, SorghumGenerator& generator);
bool InspectSorghumGrowthStages(evo_engine::InspectorContext& context, SorghumGrowthStages& growth_stages);
bool InspectSorghumLayer(evo_engine::InspectorContext& context, SorghumLayer& layer);
bool InspectSorghumState(evo_engine::InspectorContext& context, SorghumState& state);
bool InspectCBTFGroup(evo_engine::InspectorContext& context, CBTFGroup& group);
bool InspectCBTFImporter(evo_engine::InspectorContext& context, CBTFImporter& importer);

#ifdef CUDA_MODULE_SERVICE
bool InspectPARSensorGroup(evo_engine::InspectorContext& context, PARSensorGroup& group);
#endif
}  // namespace digital_agriculture_package
