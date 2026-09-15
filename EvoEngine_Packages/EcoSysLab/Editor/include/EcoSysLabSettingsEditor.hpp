#pragma once
#include "DynamicStrandsInitializationParameters.hpp"
#include "InspectorRegistry.hpp"
#include "RenderParameters.hpp"
#include "SimulationSettings.hpp"
#include "SkeletalGraphSettings.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "StrandModelParameters.hpp"
#include "TreeGrowthSettings.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreeStatistics.hpp"
#include "TreeStructor.hpp"

namespace eco_sys_lab_package {
bool InspectSettings(BranchesRenderParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(SmallSegmentsRenderParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(SmallSegmentsVisualizationRenderParameters& target,
                     const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(SegmentPairsRenderParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(FoliageRenderParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(BundleSolverSettings& target);
bool InspectSettings(SimulationSettings& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(SimulationStats& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(SkeletalGraphSettings& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(StrandModelParameters& target, const std::shared_ptr<EditorLayer>& editor_layer);
void InspectSettings(StrandModelMeshGeneratorSettings& target, const std::shared_ptr<EditorLayer>& editor_layer);
bool InspectSettings(TreeGrowthSettings& target, const std::shared_ptr<EditorLayer>& editor_layer);
void InspectSettings(TreeMeshGeneratorSettings& target, const std::shared_ptr<EditorLayer>& editor_layer);
void InspectSettings(TreeStatistics& target, const std::shared_ptr<EditorLayer>& editor_layer);
void InspectSettings(ConnectivityGraphSettings& target);
void InspectSettings(ReconstructionSettings& target);
}  // namespace eco_sys_lab_package
