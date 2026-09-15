#pragma once
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DsPhysics.hpp"
#include "InspectorRegistry.hpp"
namespace eco_sys_lab_package {
bool InspectDsLeafDrop(evo_engine::InspectorContext& context, DsLeafDrop& target);
bool InspectDsAttraction(evo_engine::InspectorContext& context, DsAttraction& target);
bool InspectDsSnow(evo_engine::InspectorContext& context, DsSnow& target);
bool InspectDsWind(evo_engine::InspectorContext& context, DsWind& target);
bool InspectDsStiffRod(evo_engine::InspectorContext& context, DsStiffRod& target);
bool InspectDsBundle(evo_engine::InspectorContext& context, DsBundle& target);
bool InspectDsFungus(evo_engine::InspectorContext& context, DsFungus& target);
bool InspectDsPrediction(evo_engine::InspectorContext& context, DsPrediction& target);
bool InspectDsDynamicHashedGrid(evo_engine::InspectorContext& context, DsDynamicHashedGrid& target);
}  // namespace eco_sys_lab_package
