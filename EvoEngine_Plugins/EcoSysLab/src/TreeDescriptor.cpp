//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"

#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
#include "Octree.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "TreeDescriptor.hpp"

#include "AssetManager.hpp"
#include "TreeMeshGenerator.hpp"

#include "BasicBarkDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "DynamicTreeSkeleton.hpp"
using namespace eco_sys_lab_plugin;

std::shared_ptr<Texture2D> IShootDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/ShootDescriptor.png"));
  }
  return thumbnail;
}
std::shared_ptr<Texture2D> IPruningDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/PruningDescriptor.png"));
  }
  return thumbnail;
}
std::shared_ptr<Texture2D> IFoliageDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/FoliageDescriptor.png"));
  }
  return thumbnail;
}
std::shared_ptr<Texture2D> IReproductionModuleDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/FruitDescriptor.png"));
  }
  return thumbnail;
}
std::shared_ptr<Texture2D> IBarkDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/BarkDescriptor.png"));
  }
  return thumbnail;
}
std::shared_ptr<Texture2D> IFlowerDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/FlowerDescriptor.png"));
  }
  return thumbnail;
}
void TreeDescriptor::OnCreate() {
}

bool TreeDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Climate> climate;
  std::shared_ptr<Soil> soil;
  if (const auto climate_candidate = EcoSysLabLayer::FindClimate(); !climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  if (soil && climate) {
    if (ImGui::Button("Instantiate")) {
      editor_layer->SetSelectedEntity(Instantiate());
    }
  } else {
    ImGui::Text("Create soil and climate entity to instantiate!");
  }
  if (editor_layer->DragAndDropButton<IShootDescriptor>(shoot_descriptor, "Shoot Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<IPruningDescriptor>(pruning_descriptor, "Pruning Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<IFoliageDescriptor>(foliage_descriptor, "Foliage Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<IReproductionModuleDescriptor>(reproduction_module_descriptor,
                                                                     "Reproduction Descriptor"))
    changed = true;

  if (editor_layer->DragAndDropButton<IBarkDescriptor>(bark_descriptor, "Bark Descriptor"))
    changed = true;
  return changed;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (shoot_descriptor.Get<BasicShootDescriptor>())
    list.push_back(shoot_descriptor);
  if (pruning_descriptor.Get<BasicPruningDescriptor>())
    list.push_back(pruning_descriptor);
  if (foliage_descriptor.Get<BasicFoliageDescriptor>())
    list.push_back(foliage_descriptor);
  if (reproduction_module_descriptor.Get<BasicReproductionModuleDescriptor>())
    list.push_back(reproduction_module_descriptor);

  if (bark_descriptor.Get<BasicBarkDescriptor>())
    list.push_back(bark_descriptor);
}

Entity TreeDescriptor::Instantiate() const {
  std::shared_ptr<Climate> climate;
  std::shared_ptr<Soil> soil;
  if (const auto climate_candidate = EcoSysLabLayer::FindClimate(); !climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  if (soil && climate) {
    const auto scene = Application::GetActiveScene();
    const auto tree_entity = scene->CreateEntity(GetTitle());
    const auto dynamic_tree_skeleton_entity = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(tree_entity).lock();
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    float height = 0;
    if (const auto soil_descriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>()) {
      if (const auto height_field = soil_descriptor->height_field.Get<HeightField>())
        height = height_field->GetValue({0.0f, 0.0f}) - 0.05f;
    }
    GlobalTransform global_transform;
    global_transform.SetPosition(glm::vec3(0, height, 0));
    scene->SetDataComponent(tree_entity, global_transform);
    tree->tree_descriptor_ref = GetSelf();
    return tree_entity;
  }

  return {};
}

void TreeDescriptor::Serialize(YAML::Emitter& out) const {
  shoot_descriptor.Save("shoot_descriptor", out);
  pruning_descriptor.Save("pruning_descriptor", out);
  foliage_descriptor.Save("foliage_descriptor", out);
  bark_descriptor.Save("bark_descriptor", out);

  reproduction_module_descriptor.Save("reproduction_module_descriptor", out);
}

std::shared_ptr<Texture2D> TreeDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/TreeDescriptor.png"));
  }
  return thumbnail;
}

void TreeDescriptor::Deserialize(const YAML::Node& in) {
  shoot_descriptor.Load("shoot_descriptor", in);
  pruning_descriptor.Load("pruning_descriptor", in);
  foliage_descriptor.Load("foliage_descriptor", in);
  bark_descriptor.Load("bark_descriptor", in);

  reproduction_module_descriptor.Load("reproduction_module_descriptor", in);
}
