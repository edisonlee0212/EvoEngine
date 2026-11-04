#include "EcoSysLabEditorLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "Serialization.hpp"
#include "TreeDebugRenderer.hpp"
#include "TreeDescriptor.hpp"

using namespace eco_sys_lab_package;

const std::vector<glm::vec3>& EcoSysLabEditorLayer::RandomColors() {
  return random_colors_;
}

void EcoSysLabEditorLayer::VisualizationCameraDragAndDrop() const {
  if (AssetRef asset_ref; EditorLayer::UnsafeDroppableAsset(asset_ref, {"Scene", "Prefab", "Mesh", "TreeDescriptor"})) {
    const auto scene = GetScene();
    if (const auto asset = asset_ref.Get<IAsset>(); asset->GetTypeName() == "TreeDescriptor") {
      std::dynamic_pointer_cast<TreeDescriptor>(asset)->Instantiate();
    }
  }
}

glm::vec2 EcoSysLabEditorLayer::GetMouseSceneCameraPosition() const {
  return visualization_camera_mouse_position;
}

void EcoSysLabEditorLayer::OnCreate() {
  tree_debug_renderer_ = std::make_shared<TreeDebugRenderer>();
  tree_debug_renderer_->Prepare();
  if (const auto runtime = ApplicationContext::Get().GetLayer<EcoSysLabLayer>())
    runtime->demo_growth_enabled_ = false;
  if (random_colors_.empty()) {
    for (int i = 0; i < 20000; i++) {
      random_colors_.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
    }
  }

  shoot_stem_strands_ = AssetManager::CreateTemporaryAsset<Strands>();
  soil_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  bounding_box_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  foliage_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  flower_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  fruit_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  ground_fruit_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  ground_flower_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  ground_leaf_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  vector_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  scalar_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  shadow_grid_particle_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  lighting_grid_particle_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
#pragma region Internode camera
  visualization_camera_ = Serialization::ProduceSerializable<Camera>();

  visualization_camera_->OnCreate();
  visualization_camera_->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
  visualization_camera_->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
#pragma endregion

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->RegisterEditorCamera(visualization_camera_);
  }
}

bool EcoSysLabEditorLayer::CanEditTree(const Tree& tree) const {
  const auto status = GetApplication().GetApplicationStatus();
  const auto scene = tree.GetScene();
  if ((status != Application::ExecutionStatus::Playing && status != Application::ExecutionStatus::Step) ||
      !scene->HasPrivateComponent<DynamicTreeSkeleton>(tree.GetOwner()))
    return true;
  const auto physics = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(tree.GetOwner()).lock();
  return !physics->IsEnabled() || !physics->physics_running || !physics->GetGpuPhysics();
}

void EcoSysLabEditorLayer::OnPostRender() {
  if (!tree_debug_requested_)
    return;
  tree_debug_requested_ = false;
  const auto scene = GetScene();
  if (!scene)
    return;
  const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto view =
      glm::inverse(glm::translate(editor->GetSceneCameraPosition()) * glm::mat4_cast(editor->GetSceneCameraRotation()));
  const auto projection_view = visualization_camera_->GetProjection() * view;
  const auto enabled_tree = [&](Entity entity) -> std::shared_ptr<Tree> {
    if (!scene->IsEntityValid(entity) || !scene->IsEntityEnabled(entity) || !scene->HasPrivateComponent<Tree>(entity))
      return {};
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
    return tree->IsEnabled() ? tree : nullptr;
  };
  if (const auto tree = enabled_tree(selected_tree)) {
    auto& state = GetTreeState(*tree);
    tree_debug_renderer_->Render(tree, state.shoot_visualizer, state.root_visualizer, visualization_camera_,
                                 projection_view);
  }
  if (const auto entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      entities && tree_visualization_settings_.display_shoot_stem)
    for (const auto entity : *entities) {
      if (entity == selected_tree)
        continue;
      if (const auto tree = enabled_tree(entity)) {
        auto& state = GetTreeState(*tree);
        auto& renderer = trees_.at(tree.get()).debug_renderer;
        if (!renderer)
          renderer = std::make_shared<TreeDebugRenderer>(tree_debug_renderer_);
        renderer->Render(tree, state.shoot_visualizer, state.root_visualizer, visualization_camera_, projection_view,
                         true);
      }
    }
}
