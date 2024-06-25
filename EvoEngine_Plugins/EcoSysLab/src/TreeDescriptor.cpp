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
#include "TreeMeshGenerator.hpp"

#include "BarkDescriptor.hpp"
#include "FlowerDescriptor.hpp"
#include "FoliageDescriptor.hpp"
#include "FruitDescriptor.hpp"
#include "ShootDescriptor.hpp"
using namespace eco_sys_lab;

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
      const auto scene = Application::GetActiveScene();
      const auto tree_entity = scene->CreateEntity(GetTitle());
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      float height = 0;
      if (const auto soil_descriptor = soil->soil_descriptor.Get<SoilDescriptor>()) {
        if (const auto height_field = soil_descriptor->height_field.Get<HeightField>())
          height = height_field->GetValue({0.0f, 0.0f}) - 0.05f;
      }
      GlobalTransform global_transform;
      global_transform.SetPosition(glm::vec3(0, height, 0));
      scene->SetDataComponent(tree_entity, global_transform);
      tree->tree_descriptor = ProjectManager::GetAsset(GetHandle());
      editor_layer->SetSelectedEntity(tree_entity);
    }
  } else {
    ImGui::Text("Create soil and climate entity to instantiate!");
  }
  if (editor_layer->DragAndDropButton<ShootDescriptor>(shoot_descriptor, "Shoot Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<FoliageDescriptor>(foliage_descriptor, "Foliage Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<FruitDescriptor>(fruit_descriptor, "Fruit Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<FlowerDescriptor>(flower_descriptor, "Flower Descriptor"))
    changed = true;

  editor_layer->DragAndDropButton<BarkDescriptor>(bark_descriptor, "BarkDescriptor");
  return changed;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (shoot_descriptor.Get<ShootDescriptor>())
    list.push_back(shoot_descriptor);
  if (foliage_descriptor.Get<FoliageDescriptor>())
    list.push_back(foliage_descriptor);
  if (fruit_descriptor.Get<FruitDescriptor>())
    list.push_back(fruit_descriptor);
  if (flower_descriptor.Get<FlowerDescriptor>())
    list.push_back(flower_descriptor);

  if (bark_descriptor.Get<BarkDescriptor>())
    list.push_back(bark_descriptor);
}

void TreeDescriptor::Serialize(YAML::Emitter& out) const {
  shoot_descriptor.Save("shoot_descriptor", out);
  foliage_descriptor.Save("foliage_descriptor", out);
  bark_descriptor.Save("bark_descriptor", out);

  fruit_descriptor.Save("fruit_descriptor", out);
  flower_descriptor.Save("flower_descriptor", out);
}

void TreeDescriptor::Deserialize(const YAML::Node& in) {
  shoot_descriptor.Load("shoot_descriptor", in);
  foliage_descriptor.Load("foliage_descriptor", in);
  bark_descriptor.Load("bark_descriptor", in);

  fruit_descriptor.Load("fruit_descriptor", in);
  flower_descriptor.Load("flower_descriptor", in);
}