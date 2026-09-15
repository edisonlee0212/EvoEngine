#pragma once
#include "InspectorRegistry.hpp"
#include "LSystemString.hpp"
#include "TreeGraph.hpp"
namespace eco_sys_lab_package {
bool InspectLSystemString(evo_engine::InspectorContext& context, LSystemString& target);
bool InspectTreeGraph(evo_engine::InspectorContext& context, TreeGraph& target);
bool InspectTreeGraphV2(evo_engine::InspectorContext& context, TreeGraphV2& target);
}  // namespace eco_sys_lab_package
