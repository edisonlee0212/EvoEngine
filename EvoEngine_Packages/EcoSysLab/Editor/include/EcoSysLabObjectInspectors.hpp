#pragma once
#include <unordered_map>
#include "Climate.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "ForestDescriptor.hpp"
#include "FungusTest.hpp"
#include "HeightField.hpp"
#include "InspectorRegistry.hpp"
#include "ObjectRotator.hpp"
#include "SoilDescriptor.hpp"
namespace eco_sys_lab_package {
struct ObjectRotatorInspector {
  bool Inspect(evo_engine::InspectorContext& context, ObjectRotator& target);
};
struct FungusTestInspector {
  std::shared_ptr<ParticleInfoList> particle_info_list;
  bool Inspect(evo_engine::InspectorContext& context, FungusTest& target);
};
struct DynamicTreeSkeletonInspector {
  struct Preview {
    std::weak_ptr<evo_engine::IPrivateComponent> owner;
    std::shared_ptr<evo_engine::ParticleInfoList> matrices;
  };
  std::weak_ptr<evo_engine::Scene> scene;
  std::unordered_map<const DynamicTreeSkeleton*, Preview> previews;
  PrivateComponentRef dynamic_tree_skeleton_tree_ref{};
  bool Inspect(evo_engine::InspectorContext& context, DynamicTreeSkeleton& target);
};
struct ClimateDescriptorInspector {
  bool Inspect(evo_engine::InspectorContext& context, ClimateDescriptor& target);
};
struct ClimateInspector {
  bool Inspect(evo_engine::InspectorContext& context, Climate& target);
};
struct HeightFieldInspector {
  bool show_noise_graph = false;
  int resolution = 64;
  float position_scale = 1.f;
  Handle current_handle;
  bool show_test_texture = true;
  std::shared_ptr<Texture2D> test_texture_2d;
  float debug_scale = 1.f;
  bool Inspect(evo_engine::InspectorContext& context, HeightField& target);
};
struct ForestPatchInspector {
  glm::ivec2 gridSize = {8, 8};
  bool setParent = true;
  bool setSimulationSettings = true;
  bool Inspect(evo_engine::InspectorContext& context, ForestPatch& target);
};
struct ForestDescriptorInspector {
  glm::ivec2 gridSize = {4, 4};
  float gridDistance = 1.5f;
  float randomShift = 0.5f;
  bool setParent = true;
  bool enableHistory = false;
  int historyIteration = 30;
  AssetRef treeDescriptorRef;
  bool Inspect(evo_engine::InspectorContext& context, ForestDescriptor& target);
};
struct SoilLayerDescriptorInspector {
  float sand_ratio = 0.1f;
  float silt_ratio = 0.1f;
  float clay_ratio = 0.8f;
  float compactness = 1.0f;
  unsigned soil_type_preset = 0;
  bool show_capacity = false;
  bool show_permeability = false;
  bool show_density = false;
  bool show_initial_nutrients = false;
  bool show_initial_water = false;
  bool show_thickness = false;
  bool Inspect(evo_engine::InspectorContext& context, SoilLayerDescriptor& target);
};
struct SoilDescriptorInspector {
  bool Inspect(evo_engine::InspectorContext& context, SoilDescriptor& target);
};
}  // namespace eco_sys_lab_package
