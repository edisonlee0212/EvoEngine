#pragma once
#include "DynamicSkeleton.hpp"
#include "DynamicStrands.hpp"
#include "DynamicStrandsInitializationParameters.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "DynamicTreeStrands.hpp"
#include "InspectorRegistry.hpp"
namespace eco_sys_lab_package {
bool InspectSettings(DynamicSkeleton::InitializeParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(DynamicSkeleton::PhysicsParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(DynamicSkeleton::VisualizationParameters& target,
                     const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(DynamicStrands::PhysicsParameters& target, const std::shared_ptr<EditorLayer>& editor_layer,
                     bool show_fungus_toggle = true);
bool InspectSettings(DynamicStrandsVisualizationParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(DynamicTreeStrands::BoardExperimentSetupSettings& target,
                     const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(DynamicTreeStrands::LogExperimentSetupSettings& target,
                     const std::shared_ptr<EditorLayer>& editor_layer);
struct DynamicStrandsInitializationInspector {
  bool show_damage_graph = false;
  bool show_modulus_graph = false;
  bool show_strength_graph = false;
  bool show_biological_properties_graph = false;
  bool Inspect(evo_engine::InspectorContext& context, DynamicStrandsInitializeParameters& target);
};
}  // namespace eco_sys_lab_package
