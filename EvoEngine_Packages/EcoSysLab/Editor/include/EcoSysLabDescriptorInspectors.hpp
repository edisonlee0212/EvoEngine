#pragma once
#include "InspectorRegistry.hpp"
namespace eco_sys_lab_package {
struct GrowthDescriptorInspectorState {
  bool show_branching_angle_graph = false;
  bool show_roll_angle_graph = false;
  bool show_apical_angle_graph = false;
};
class AdvancedShootDescriptor;
bool InspectAdvancedShootDescriptor(evo_engine::InspectorContext& context, AdvancedShootDescriptor& target);
class BasicBarkDescriptor;
bool InspectBasicBarkDescriptor(evo_engine::InspectorContext& context, BasicBarkDescriptor& target);
class BasicFineRootDescriptor;
bool InspectBasicFineRootDescriptor(evo_engine::InspectorContext& context, BasicFineRootDescriptor& target);
class BasicFoliageDescriptor;
bool InspectBasicFoliageDescriptor(evo_engine::InspectorContext& context, BasicFoliageDescriptor& target);
class BasicPruningDescriptor;
bool InspectBasicPruningDescriptor(evo_engine::InspectorContext& context, BasicPruningDescriptor& target);
class BasicReproductionModuleDescriptor;
bool InspectBasicReproductionModuleDescriptor(evo_engine::InspectorContext& context,
                                              BasicReproductionModuleDescriptor& target);
class BasicRootDescriptor;
bool InspectBasicRootDescriptor(evo_engine::InspectorContext& context, BasicRootDescriptor& target,
                                GrowthDescriptorInspectorState& state);
class BasicShootDescriptor;
bool InspectBasicShootDescriptor(evo_engine::InspectorContext& context, BasicShootDescriptor& target,
                                 GrowthDescriptorInspectorState& state);
class TreeDescriptor;
bool InspectTreeDescriptor(evo_engine::InspectorContext& context, TreeDescriptor& target);
}  // namespace eco_sys_lab_package
