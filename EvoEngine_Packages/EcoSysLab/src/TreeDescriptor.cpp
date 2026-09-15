//
// Created by lllll on 10/24/2022.
//

#include "EcoSysLabSerializationAdapters.hpp"
#include "Tree.hpp"

#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
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
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "DynamicTreeSkeleton.hpp"
using namespace eco_sys_lab_package;

void TreeDescriptor::OnCreate() {
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (shoot_descriptor.Get<BasicShootDescriptor>())
    list.push_back(shoot_descriptor);
  if (root_descriptor.Get<BasicRootDescriptor>())
    list.push_back(root_descriptor);
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
    const auto scene = ApplicationContext::Get().GetActiveScene();
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

void eco_sys_lab_package::SerializeTreeDescriptor(YAML::Emitter& out, const TreeDescriptor& target) {
  target.shoot_descriptor.Save("shoot_descriptor", out);
  target.root_descriptor.Save("root_descriptor", out);
  target.pruning_descriptor.Save("pruning_descriptor", out);
  target.foliage_descriptor.Save("foliage_descriptor", out);
  target.bark_descriptor.Save("bark_descriptor", out);

  target.reproduction_module_descriptor.Save("reproduction_module_descriptor", out);
}

void eco_sys_lab_package::DeserializeTreeDescriptor(const YAML::Node& in, TreeDescriptor& target) {
  target.shoot_descriptor.Load("shoot_descriptor", in);
  target.root_descriptor.Load("root_descriptor", in);
  target.pruning_descriptor.Load("pruning_descriptor", in);
  target.foliage_descriptor.Load("foliage_descriptor", in);
  target.bark_descriptor.Load("bark_descriptor", in);

  target.reproduction_module_descriptor.Load("reproduction_module_descriptor", in);
}
