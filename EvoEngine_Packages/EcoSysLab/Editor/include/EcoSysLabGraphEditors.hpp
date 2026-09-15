#pragma once
#include "DynamicTreeStrandGenerators.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "InspectorRegistry.hpp"
namespace eco_sys_lab_package {
struct ModulusGraphInspectorState {
  ModulusGraph::Input temp_input{};
  ModulusGraph::Output::DensityType temp_output_density = glm::vec2(0.0f);
  ModulusGraph::Output::ShearStretchModulusType temp_output_shear_stretch = glm::vec2(0.0f);
  ModulusGraph::Output::BendingModulusType temp_output_bending = glm::vec2(0.0f);
  ModulusGraph::Output::TwistingModulusType temp_output_twisting = glm::vec2(0.0f);
  bool show_node_graph = true;
};
struct StrengthGraphInspectorState {
  StrengthGraph::Input temp_input{};
  StrengthGraph::Output::ShearStretchStrengthType temp_output_shear_stretch = glm::vec2(0.0f);
  StrengthGraph::Output::BendingStrengthType temp_output_bending = glm::vec2(0.0f);
  StrengthGraph::Output::TwistingStrengthType temp_output_twisting = glm::vec2(0.0f);
  StrengthGraph::Output::BundleStrengthType temp_output_bundle = glm::vec2(0.0f);
  StrengthGraph::Output::ConnectivityStrengthType temp_output_connectivity = glm::vec2(0.0f);
  bool show_node_graph = true;
};
struct BiologicalPropertiesGraphInspectorState {
  BiologicalPropertiesGraph::Input temp_input{};
  BiologicalPropertiesGraph::Output temp_output;
  bool show_node_graph = true;
};
bool InspectModulusGraph(InspectorContext& context, ModulusGraph& target, ModulusGraphInspectorState& state);
bool InspectStrengthGraph(InspectorContext& context, StrengthGraph& target, StrengthGraphInspectorState& state);
bool InspectBiologicalPropertiesGraph(InspectorContext& context, BiologicalPropertiesGraph& target,
                                      BiologicalPropertiesGraphInspectorState& state);
bool DrawStrandGraph(IDynamicTreeStrands& target, const std::string& window_title,
                     const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
bool InspectConstantNode(evo_engine::InspectorContext& context, ConstantNode& target);
}  // namespace eco_sys_lab_package
