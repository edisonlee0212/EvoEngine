#include "Climate.hpp"

#include "AssetManager.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab_plugin;

bool ClimateDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Button("Instantiate")) {
    const auto scene = Application::GetActiveScene();
    const auto climate_entity = scene->CreateEntity(GetTitle());
    const auto climate = scene->GetOrSetPrivateComponent<Climate>(climate_entity).lock();
    climate->climate_descriptor_ref = GetSelf();
  }
  return changed;
}

std::shared_ptr<Texture2D> ClimateDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/ClimateDescriptor.png"));
  }
  return thumbnail;
}

void ClimateDescriptor::Serialize(YAML::Emitter& out) const {
}

void ClimateDescriptor::Deserialize(const YAML::Node& in) {
}

bool Climate::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<ClimateDescriptor>(climate_descriptor_ref, "ClimateDescriptor", true)) {
    InitializeClimateModel();
    changed = true;
  }

  if (climate_descriptor_ref.Get<ClimateDescriptor>()) {
  }
  return changed;
}

void Climate::Serialize(YAML::Emitter& out) const {
  climate_descriptor_ref.Save("climate_descriptor_ref", out);
}

void Climate::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(climate_descriptor_ref);
}

void Climate::InitializeClimateModel() {
  if (const auto climate_descriptor = this->climate_descriptor_ref.Get<ClimateDescriptor>()) {
    const auto params = climate_descriptor->climate_parameters;
    climate_model.Initialize(params);
  }
}

void Climate::PrepareForGrowth() {
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  const auto scene = GetScene();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (!tree_entities || tree_entities->empty())
    return;

  auto& estimator = climate_model.environment_grid;
  auto min_bound = estimator.voxel_grid.GetMinBound();
  auto max_bound = estimator.voxel_grid.GetMaxBound();
  bool bound_changed = false;
  for (const auto& tree_entity : *tree_entities) {
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    const auto global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity).value;

    tree->tree_model.RefShootSkeleton().CalculateMinMax();
    const glm::vec3 current_min_bound = global_transform * glm::vec4(tree->tree_model.RefShootSkeleton().min, 1.0f);

    if (const glm::vec3 current_max_bound = global_transform * glm::vec4(tree->tree_model.RefShootSkeleton().max, 1.0f);
        current_min_bound.x <= min_bound.x || current_min_bound.y <= min_bound.y ||
        current_min_bound.z <= min_bound.z || current_max_bound.x >= max_bound.x ||
        current_max_bound.y >= max_bound.y || current_max_bound.z >= max_bound.z) {
      min_bound = glm::min(current_min_bound - glm::vec3(1.0f, 0.1f, 1.0f), min_bound);
      max_bound = glm::max(current_max_bound + glm::vec3(1.0f), max_bound);
      bound_changed = true;
    }
  }
  if (bound_changed)
    estimator.voxel_grid.Initialize(estimator.voxel_size, min_bound, max_bound);
  estimator.voxel_grid.Reset();
  for (const auto& tree_entity : *tree_entities) {
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->RegisterVoxel();
  }

  estimator.LightPropagation(eco_sys_lab_layer->simulation_settings);
}

void Climate::Deserialize(const YAML::Node& in) {
  climate_descriptor_ref.Load("climate_descriptor_ref", in);
}
