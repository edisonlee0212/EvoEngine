#pragma once
#include "InspectorRegistry.hpp"
#include "Tree.hpp"
#include "TreeVisualizer.hpp"
namespace eco_sys_lab_package {
struct TreeEditorState {
  ShootVisualizer shoot_visualizer;
  RootVisualizer root_visualizer;
  Tree::ModelRevision shoot_revision;
  Tree::ModelRevision root_revision;
#ifdef BILLBOARD_CLOUDS_PACKAGE
  BillboardCloud::GenerateSettings foliage_billboard_cloud_generate_settings{};
#endif
  bool show_space_colonization_grid = true;
  std::shared_ptr<ParticleInfoList> space_colonization_grid_particle_info_list;
  float radius = 1.5f;
  int markers_per_voxel = 5;
  PrivateComponentRef private_component_ref{};
  int mesh_generate_iterations = 0;

  explicit TreeEditorState(const Tree& tree);
  void Sync(const Tree& tree);
  bool Inspect(evo_engine::InspectorContext& context, Tree& target);
};
TreeEditorState& GetTreeEditorState(Tree& tree);
}  // namespace eco_sys_lab_package
