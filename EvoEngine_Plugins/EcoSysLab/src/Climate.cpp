#include "Climate.hpp"

#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab;

bool ClimateDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  if (ImGui::Button("Instantiate")) {
    const auto scene = Application::GetActiveScene();
    const auto climateEntity = scene->CreateEntity(GetTitle());
    const auto climate = scene->GetOrSetPrivateComponent<Climate>(climateEntity).lock();
    climate->climate_descriptor = ProjectManager::GetAsset(GetHandle());
  }
  return changed;
}

void ClimateDescriptor::Serialize(YAML::Emitter& out) const {
}

void ClimateDescriptor::Deserialize(const YAML::Node& in) {
}

bool Climate::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  if (editorLayer->DragAndDropButton<ClimateDescriptor>(climate_descriptor, "ClimateDescriptor", true)) {
    InitializeClimateModel();
  }

  if (climate_descriptor.Get<ClimateDescriptor>()) {
  }
  return changed;
}

void Climate::Serialize(YAML::Emitter& out) const {
  climate_descriptor.Save("climate_descriptor", out);
}

void Climate::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(climate_descriptor);
}

void Climate::InitializeClimateModel() {
  auto climateDescriptor = climate_descriptor.Get<ClimateDescriptor>();
  if (climateDescriptor) {
    auto params = climateDescriptor->climate_parameters;
    climate_model.Initialize(params);
  }
}

void Climate::PrepareForGrowth() {
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (!treeEntities || treeEntities->empty())
    return;

  auto& estimator = climate_model.environment_grid;
  auto minBound = estimator.voxel_grid.GetMinBound();
  auto maxBound = estimator.voxel_grid.GetMaxBound();
  bool boundChanged = false;
  for (const auto& treeEntity : *treeEntities) {
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
    const auto globalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity).value;
    const glm::vec3 currentMinBound = globalTransform * glm::vec4(tree->tree_model.RefShootSkeleton().min, 1.0f);
    const glm::vec3 currentMaxBound = globalTransform * glm::vec4(tree->tree_model.RefShootSkeleton().max, 1.0f);

    if (currentMinBound.x <= minBound.x || currentMinBound.y <= minBound.y || currentMinBound.z <= minBound.z ||
        currentMaxBound.x >= maxBound.x || currentMaxBound.y >= maxBound.y || currentMaxBound.z >= maxBound.z) {
      minBound = glm::min(currentMinBound - glm::vec3(1.0f, 0.1f, 1.0f), minBound);
      maxBound = glm::max(currentMaxBound + glm::vec3(1.0f), maxBound);
      boundChanged = true;
      // EVOENGINE_LOG("Shadow grid resized!");
    }
    tree->crown_shyness_distance = ecoSysLabLayer->m_simulationSettings.crown_shyness_distance;
  }
  if (boundChanged)
    estimator.voxel_grid.Initialize(estimator.voxel_size, minBound, maxBound);
  estimator.voxel_grid.Reset();
  for (const auto& tree_entity : *treeEntities) {
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->RegisterVoxel();
  }

  estimator.LightPropagation(ecoSysLabLayer->m_simulationSettings);
}

void Climate::Deserialize(const YAML::Node& in) {
  climate_descriptor.Load("climate_descriptor", in);
}
