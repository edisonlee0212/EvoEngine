#include "EcoSysLabEditorLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Serialization.hpp"
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
