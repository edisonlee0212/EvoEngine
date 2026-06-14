#pragma once

#include "InspectorRegistry.hpp"

namespace l_system_package {
class LSystemDescriptor;
class LSystemLayer;
class ScotsPine;
class ScotsPineDescriptor;

bool InspectLSystemDescriptor(evo_engine::InspectorContext& context, LSystemDescriptor& descriptor);
bool InspectLSystemLayer(evo_engine::InspectorContext& context, LSystemLayer& layer);
bool InspectScotsPine(evo_engine::InspectorContext& context, ScotsPine& pine);
bool InspectScotsPineDescriptor(evo_engine::InspectorContext& context, ScotsPineDescriptor& descriptor);
}  // namespace l_system_package
