#include "EcoSysLabEditorLayer.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_package;
TreeEditorState::TreeEditorState(const Tree& tree) {
  shoot_visualizer.Initialize();
  root_visualizer.Initialize();
  shoot_visualizer.Reset(tree.shoot_model);
  root_visualizer.Reset(tree.root_model);
  shoot_revision = tree.GetShootModelRevision();
  root_revision = tree.GetRootModelRevision();
}
void TreeEditorState::Sync(const Tree& tree) {
  const auto sync = [](auto& visualizer, auto& previous, const auto& revision, const auto& model) {
    if (revision.topology != previous.topology)
      visualizer.ClearSelections();
    if (revision.content != previous.content) {
      visualizer.checkpoint_iteration = model.CurrentIteration();
      visualizer.need_update = true;
      if (model.CurrentIteration() == 0)
        visualizer.Reset(model);
    }
    previous = revision;
  };
  sync(shoot_visualizer, shoot_revision, tree.GetShootModelRevision(), tree.shoot_model);
  sync(root_visualizer, root_revision, tree.GetRootModelRevision(), tree.root_model);
}
TreeEditorState& EcoSysLabEditorLayer::GetTreeState(Tree& tree) {
  const auto scene = tree.GetScene();
  if (scene_.lock() != scene) {
    ClearSceneVisualization();
    selected_tree = {};
    scene_ = scene;
  }
  for (auto it = trees_.begin(); it != trees_.end();) {
    if (it->second.owner.expired())
      it = trees_.erase(it);
    else
      ++it;
  }
  auto& entry = trees_[&tree];
  if (!entry.state) {
    entry.owner = scene->GetPrivateComponent(tree.GetOwner(), "Tree");
    entry.state = std::make_unique<TreeEditorState>(tree);
  }
  entry.state->Sync(tree);
  return *entry.state;
}
TreeEditorState& eco_sys_lab_package::GetTreeEditorState(Tree& tree) {
  return ApplicationContext::Get().GetLayer<EcoSysLabEditorLayer>()->GetTreeState(tree);
}

void EcoSysLabEditorLayer::Update() {
  const auto runtime = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!runtime)
    return;
  if (const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>())
    runtime->demo_growth_enabled_ = editor->IsPlantVisualSplitLayoutReady();
  if (scene_.lock() != GetScene()) {
    ClearSceneVisualization();
    selected_tree = {};
    scene_ = GetScene();
  }
  for (auto it = trees_.begin(); it != trees_.end();) {
    if (it->second.owner.expired())
      it = trees_.erase(it);
    else
      ++it;
  }
  if (reset_revision_ != runtime->reset_revision_) {
    reset_revision_ = runtime->reset_revision_;
    ClearSceneVisualization();
  }
  if (simulation_revision_ != runtime->simulation_revision_) {
    simulation_revision_ = runtime->simulation_revision_;
    need_full_flow_update = true;
    UpdateGroundFruitAndLeaves();
    if (const auto scene = GetScene(); scene && scene->IsEntityValid(selected_tree)) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
      GetTreeState(*tree);
      if (auto_generate_skeletal_graph_every_frame_)
        tree->GenerateSkeletalGraph(runtime->skeletal_graph_settings, -1,
                                    Resources::GetInstance().GetPrimitives().sphere,
                                    Resources::GetInstance().GetPrimitives().cube);
    }
  }
}

void EcoSysLabEditorLayer::LateUpdate() {
  if (GetScene() && ApplicationContext::Get().GetLayer<EcoSysLabLayer>())
    DynamicStrandVisualization();
}

void EcoSysLabEditorLayer::OnDestroy() {
  if (const auto runtime = ApplicationContext::Get().GetLayer<EcoSysLabLayer>())
    runtime->demo_growth_enabled_ = true;
  if (visualization_camera_) {
    if (const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>())
      editor->UnregisterEditorCamera(visualization_camera_->GetHandle());
    visualization_camera_->OnDestroy();
    visualization_camera_.reset();
  }
  trees_.clear();
  scene_.reset();
}

void EcoSysLabEditorLayer::ClearSceneVisualization() {
  trees_.clear();
  last_selected_tree_index_ = -1;
  shoot_versions_.clear();
  soil_version_ = -1;
  need_full_flow_update = true;
  need_flow_update_for_selection_ = false;
  shoot_stem_segments_.clear();
  shoot_stem_points_.clear();
  shoot_stem_strands_ = AssetManager::CreateTemporaryAsset<Strands>();
  for (const auto& matrices :
       {soil_matrices_, bounding_box_matrices_, foliage_matrices_, flower_matrices_, fruit_matrices_,
        ground_flower_matrices_, ground_fruit_matrices_, ground_leaf_matrices_, vector_matrices_, scalar_matrices_,
        shadow_grid_particle_info_list_, lighting_grid_particle_info_list_}) {
    if (matrices)
      matrices->SetParticleInfos({});
  }
  auto_time_grow = false;
  mouse_positions.clear();
  strand_operator_mouse_points.clear();
  may_need_geometry_generation = false;
  last_gizmos_used = last_frame_invigorate = last_frame_reduce = false;
  is_box_selection_previously = is_operating_previously = false;
}
