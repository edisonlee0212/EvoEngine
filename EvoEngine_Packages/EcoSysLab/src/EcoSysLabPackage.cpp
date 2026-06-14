#include "PackageManager.hpp"

#include "AdvancedShootDescriptor.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DsColliders.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "ForestDescriptor.hpp"
#include "FungusTest.hpp"
#include "HeightField.hpp"
#include "InspectorRegistry.hpp"
#include "ObjectRotator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"
#include "RadialBoundingVolume.hpp"
#include "Serialization.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeStructor.hpp"

#include <type_traits>

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace eco_sys_lab_package {
void SerializeSpatialPlantDistributionSimulator(YAML::Emitter&, const SpatialPlantDistributionSimulator&) {
}

void DeserializeSpatialPlantDistributionSimulator(const YAML::Node&, SpatialPlantDistributionSimulator&) {
}

void SerializeDynamicTreeSkeleton(YAML::Emitter&, const DynamicTreeSkeleton&) {
}

void DeserializeDynamicTreeSkeleton(const YAML::Node&, DynamicTreeSkeleton&) {
}

void SerializeDynamicStrandsDemo(YAML::Emitter&, const DynamicStrandsDemo&) {
}

void DeserializeDynamicStrandsDemo(const YAML::Node&, DynamicStrandsDemo&) {
}

void SerializePhysics2DDemo(YAML::Emitter&, const Physics2DDemo&) {
}

void DeserializePhysics2DDemo(const YAML::Node&, Physics2DDemo&) {
}

void SerializeParticlePhysics2DDemo(YAML::Emitter&, const ParticlePhysics2DDemo&) {
}

void DeserializeParticlePhysics2DDemo(const YAML::Node&, ParticlePhysics2DDemo&) {
}

void SerializeFungusTest(YAML::Emitter&, const FungusTest&) {
}

void DeserializeFungusTest(const YAML::Node&, FungusTest&) {
}
}  // namespace eco_sys_lab_package

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "EcoSysLab", "0.1.0", "EcoSysLab runtime package."};

template <typename T>
void RegisterAssetPreviewHandler(const std::string& owner_name, const std::string& type_name) {
  Serialization::RegisterAssetPreviewHandler<T>(
      [](const std::shared_ptr<T>& asset, const OffscreenPreviewSettings&) {
        return asset ? asset->GenerateThumbnailTexture() : nullptr;
      },
      owner_name, type_name);
}

void RegisterEcoSysLabAssetPreviewHandlers(const std::string& owner_name) {
  RegisterAssetPreviewHandler<ClimateDescriptor>(owner_name, "ClimateDescriptor");
  RegisterAssetPreviewHandler<ForestPatch>(owner_name, "ForestPatch");
  RegisterAssetPreviewHandler<BasicBarkDescriptor>(owner_name, "BasicBarkDescriptor");
  RegisterAssetPreviewHandler<ForestDescriptor>(owner_name, "ForestDescriptor");
  RegisterAssetPreviewHandler<TreeDescriptor>(owner_name, "TreeDescriptor");
  RegisterAssetPreviewHandler<BasicPruningDescriptor>(owner_name, "BasicPruningDescriptor");
  RegisterAssetPreviewHandler<BasicShootDescriptor>(owner_name, "BasicShootDescriptor");
  RegisterAssetPreviewHandler<BasicRootDescriptor>(owner_name, "BasicRootDescriptor");
  RegisterAssetPreviewHandler<BasicFineRootDescriptor>(owner_name, "BasicFineRootDescriptor");
  RegisterAssetPreviewHandler<BasicReproductionModuleDescriptor>(owner_name, "BasicReproductionModuleDescriptor");
  RegisterAssetPreviewHandler<BasicFoliageDescriptor>(owner_name, "BasicFoliageDescriptor");
  RegisterAssetPreviewHandler<AdvancedShootDescriptor>(owner_name, "AdvancedShootDescriptor");
  RegisterAssetPreviewHandler<HeightField>(owner_name, "HeightField");
  RegisterAssetPreviewHandler<SoilDescriptor>(owner_name, "SoilDescriptor");
}

void RegisterEcoSysLabSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<TreeStructor>(SerializeTreeStructor, DeserializeTreeStructor, owner_name,
                                                            "TreeStructor");
  Serialization::RegisterSerializationHandler<Climate>(SerializeClimate, DeserializeClimate, owner_name, "Climate");
  Serialization::RegisterSerializationHandler<SpatialPlantDistributionSimulator>(
      SerializeSpatialPlantDistributionSimulator, DeserializeSpatialPlantDistributionSimulator, owner_name,
      "SpatialPlantDistributionSimulator");
  Serialization::RegisterSerializationHandler<DynamicTreeSkeleton>(
      SerializeDynamicTreeSkeleton, DeserializeDynamicTreeSkeleton, owner_name, "DynamicTreeSkeleton");
  Serialization::RegisterSerializationHandler<DynamicStrandsDemo>(
      SerializeDynamicStrandsDemo, DeserializeDynamicStrandsDemo, owner_name, "DynamicStrandsDemo");
  Serialization::RegisterSerializationHandler<Tree>(SerializeTree, DeserializeTree, owner_name, "Tree");
  Serialization::RegisterSerializationHandler<Soil>(SerializeSoil, DeserializeSoil, owner_name, "Soil");
  Serialization::RegisterSerializationHandler<DsBoxCollider>(SerializeDsBoxCollider, DeserializeDsBoxCollider,
                                                             owner_name, "DsBoxCollider");
  Serialization::RegisterSerializationHandler<DsSphereCollider>(SerializeDsSphereCollider, DeserializeDsSphereCollider,
                                                                owner_name, "DsSphereCollider");
  Serialization::RegisterSerializationHandler<DsCylinderCollider>(
      SerializeDsCylinderCollider, DeserializeDsCylinderCollider, owner_name, "DsCylinderCollider");
  Serialization::RegisterSerializationHandler<DynamicTreeStrands>(
      SerializeDynamicTreeStrands, DeserializeDynamicTreeStrands, owner_name, "DynamicTreeStrands");
  Serialization::RegisterSerializationHandler<ObjectRotator>(SerializeObjectRotator, DeserializeObjectRotator,
                                                             owner_name, "ObjectRotator");
  Serialization::RegisterSerializationHandler<Physics2DDemo>(SerializePhysics2DDemo, DeserializePhysics2DDemo,
                                                             owner_name, "Physics2DDemo");
  Serialization::RegisterSerializationHandler<ParticlePhysics2DDemo>(
      SerializeParticlePhysics2DDemo, DeserializeParticlePhysics2DDemo, owner_name, "ParticlePhysics2DDemo");
  Serialization::RegisterSerializationHandler<FungusTest>(SerializeFungusTest, DeserializeFungusTest, owner_name,
                                                          "FungusTest");
  Serialization::RegisterSerializationHandler<ClimateDescriptor>(
      SerializeClimateDescriptor, DeserializeClimateDescriptor, owner_name, "ClimateDescriptor");
  Serialization::RegisterSerializationHandler<RadialBoundingVolume>(
      SerializeRadialBoundingVolume, DeserializeRadialBoundingVolume, owner_name, "RadialBoundingVolume");
  Serialization::RegisterSerializationHandler<CubeVolume>(SerializeCubeVolume, DeserializeCubeVolume, owner_name,
                                                          "CubeVolume");
  Serialization::RegisterSerializationHandler<ForestPatch>(SerializeForestPatch, DeserializeForestPatch, owner_name,
                                                           "ForestPatch");
  Serialization::RegisterSerializationHandler<BasicBarkDescriptor>(
      SerializeBasicBarkDescriptor, DeserializeBasicBarkDescriptor, owner_name, "BasicBarkDescriptor");
  Serialization::RegisterSerializationHandler<ForestDescriptor>(SerializeForestDescriptor, DeserializeForestDescriptor,
                                                                owner_name, "ForestDescriptor");
  Serialization::RegisterSerializationHandler<TreeDescriptor>(SerializeTreeDescriptor, DeserializeTreeDescriptor,
                                                              owner_name, "TreeDescriptor");
  Serialization::RegisterSerializationHandler<BasicPruningDescriptor>(
      SerializeBasicPruningDescriptor, DeserializeBasicPruningDescriptor, owner_name, "BasicPruningDescriptor");
  Serialization::RegisterSerializationHandler<BasicShootDescriptor>(
      SerializeBasicShootDescriptor, DeserializeBasicShootDescriptor, owner_name, "BasicShootDescriptor");
  Serialization::RegisterSerializationHandler<BasicRootDescriptor>(
      SerializeBasicRootDescriptor, DeserializeBasicRootDescriptor, owner_name, "BasicRootDescriptor");
  Serialization::RegisterSerializationHandler<BasicFineRootDescriptor>(
      SerializeBasicFineRootDescriptor, DeserializeBasicFineRootDescriptor, owner_name, "BasicFineRootDescriptor");
  Serialization::RegisterSerializationHandler<BasicReproductionModuleDescriptor>(
      SerializeBasicReproductionModuleDescriptor, DeserializeBasicReproductionModuleDescriptor, owner_name,
      "BasicReproductionModuleDescriptor");
  Serialization::RegisterSerializationHandler<BasicFoliageDescriptor>(
      SerializeBasicFoliageDescriptor, DeserializeBasicFoliageDescriptor, owner_name, "BasicFoliageDescriptor");
  Serialization::RegisterSerializationHandler<AdvancedShootDescriptor>(
      SerializeAdvancedShootDescriptor, DeserializeAdvancedShootDescriptor, owner_name, "AdvancedShootDescriptor");
  Serialization::RegisterSerializationHandler<ModulusGraph>(SerializeModulusGraph, DeserializeModulusGraph, owner_name,
                                                            "ModulusGraph");
  Serialization::RegisterSerializationHandler<StrengthGraph>(SerializeStrengthGraph, DeserializeStrengthGraph,
                                                             owner_name, "StrengthGraph");
  Serialization::RegisterSerializationHandler<BiologicalPropertiesGraph>(
      SerializeBiologicalPropertiesGraph, DeserializeBiologicalPropertiesGraph, owner_name, "TrunkGraph");
  Serialization::RegisterSerializationHandler<HeightField>(SerializeHeightField, DeserializeHeightField, owner_name,
                                                           "HeightField");
  Serialization::RegisterSerializationHandler<SoilLayerDescriptor>(
      SerializeSoilLayerDescriptor, DeserializeSoilLayerDescriptor, owner_name, "SoilLayerDescriptor");
  Serialization::RegisterSerializationHandler<SoilDescriptor>(SerializeSoilDescriptor, DeserializeSoilDescriptor,
                                                              owner_name, "SoilDescriptor");
}

template <typename T>
void RegisterPackageInspector(const std::string& owner_name, const std::string& type_name) {
  InspectorRegistry::GetInstance().RegisterInspector<T>(
      [](InspectorContext& context, T& target) {
        if constexpr (std::is_void_v<decltype(target.DrawGui(context.editor_layer))>) {
          target.DrawGui(context.editor_layer);
          return false;
        } else {
          return target.DrawGui(context.editor_layer);
        }
      },
      owner_name, type_name);
}

void RegisterEcoSysLabInspectors(const std::string& owner_name) {
  RegisterPackageInspector<TreeStructor>(owner_name, "TreeStructor");
  RegisterPackageInspector<Climate>(owner_name, "Climate");
  RegisterPackageInspector<SpatialPlantDistributionSimulator>(owner_name, "SpatialPlantDistributionSimulator");
  RegisterPackageInspector<DynamicTreeSkeleton>(owner_name, "DynamicTreeSkeleton");
  RegisterPackageInspector<DynamicStrandsDemo>(owner_name, "DynamicStrandsDemo");
  RegisterPackageInspector<Tree>(owner_name, "Tree");
  RegisterPackageInspector<Soil>(owner_name, "Soil");
  RegisterPackageInspector<DsBoxCollider>(owner_name, "DsBoxCollider");
  RegisterPackageInspector<DsSphereCollider>(owner_name, "DsSphereCollider");
  RegisterPackageInspector<DsCylinderCollider>(owner_name, "DsCylinderCollider");
  RegisterPackageInspector<DynamicTreeStrands>(owner_name, "DynamicTreeStrands");
  RegisterPackageInspector<ObjectRotator>(owner_name, "ObjectRotator");
  RegisterPackageInspector<Physics2DDemo>(owner_name, "Physics2DDemo");
  RegisterPackageInspector<ParticlePhysics2DDemo>(owner_name, "ParticlePhysics2DDemo");
  RegisterPackageInspector<FungusTest>(owner_name, "FungusTest");
  RegisterPackageInspector<ClimateDescriptor>(owner_name, "ClimateDescriptor");
  RegisterPackageInspector<RadialBoundingVolume>(owner_name, "RadialBoundingVolume");
  RegisterPackageInspector<CubeVolume>(owner_name, "CubeVolume");
  RegisterPackageInspector<ForestPatch>(owner_name, "ForestPatch");
  RegisterPackageInspector<BasicBarkDescriptor>(owner_name, "BasicBarkDescriptor");
  RegisterPackageInspector<ForestDescriptor>(owner_name, "ForestDescriptor");
  RegisterPackageInspector<TreeDescriptor>(owner_name, "TreeDescriptor");
  RegisterPackageInspector<BasicPruningDescriptor>(owner_name, "BasicPruningDescriptor");
  RegisterPackageInspector<BasicShootDescriptor>(owner_name, "BasicShootDescriptor");
  RegisterPackageInspector<BasicRootDescriptor>(owner_name, "BasicRootDescriptor");
  RegisterPackageInspector<BasicFineRootDescriptor>(owner_name, "BasicFineRootDescriptor");
  RegisterPackageInspector<BasicReproductionModuleDescriptor>(owner_name, "BasicReproductionModuleDescriptor");
  RegisterPackageInspector<BasicFoliageDescriptor>(owner_name, "BasicFoliageDescriptor");
  RegisterPackageInspector<AdvancedShootDescriptor>(owner_name, "AdvancedShootDescriptor");
  RegisterPackageInspector<ModulusGraph>(owner_name, "ModulusGraph");
  RegisterPackageInspector<StrengthGraph>(owner_name, "StrengthGraph");
  RegisterPackageInspector<BiologicalPropertiesGraph>(owner_name, "TrunkGraph");
  RegisterPackageInspector<HeightField>(owner_name, "HeightField");
  RegisterPackageInspector<SoilLayerDescriptor>(owner_name, "SoilLayerDescriptor");
  RegisterPackageInspector<SoilDescriptor>(owner_name, "SoilDescriptor");
  RegisterPackageInspector<EcoSysLabLayer>(owner_name, "EcoSysLab Layer");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  const bool registered =
      registrar->RegisterPrivateComponent<TreeStructor>("TreeStructor") &&
      registrar->RegisterPrivateComponent<Climate>("Climate") &&
      registrar->RegisterPrivateComponent<SpatialPlantDistributionSimulator>("SpatialPlantDistributionSimulator") &&
      registrar->RegisterPrivateComponent<DynamicTreeSkeleton>("DynamicTreeSkeleton") &&
      registrar->RegisterPrivateComponent<DynamicStrandsDemo>("DynamicStrandsDemo") &&
      registrar->RegisterPrivateComponent<Tree>("Tree") && registrar->RegisterPrivateComponent<Soil>("Soil") &&
      registrar->RegisterPrivateComponent<DsBoxCollider>("DsBoxCollider") &&
      registrar->RegisterPrivateComponent<DsSphereCollider>("DsSphereCollider") &&
      registrar->RegisterPrivateComponent<DsCylinderCollider>("DsCylinderCollider") &&
      registrar->RegisterPrivateComponent<DynamicTreeStrands>("DynamicTreeStrands") &&
      registrar->RegisterPrivateComponent<ObjectRotator>("ObjectRotator") &&
      registrar->RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo") &&
      registrar->RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo") &&
      registrar->RegisterPrivateComponent<FungusTest>("FungusTest") &&
      registrar->RegisterAsset<ClimateDescriptor>("ClimateDescriptor", {".climate"}) &&
      registrar->RegisterAsset<RadialBoundingVolume>("RadialBoundingVolume", {".rbv"}) &&
      registrar->RegisterAsset<CubeVolume>("CubeVolume", {".cubevolume"}) &&
      registrar->RegisterAsset<ForestPatch>("ForestPatch", {".forestpatch"}) &&
      registrar->RegisterAsset<BasicBarkDescriptor>("BasicBarkDescriptor", {".bark"}) &&
      registrar->RegisterAsset<ForestDescriptor>("ForestDescriptor", {".forest"}) &&
      registrar->RegisterAsset<TreeDescriptor>("TreeDescriptor", {".tree"}) &&
      registrar->RegisterAsset<BasicPruningDescriptor>("BasicPruningDescriptor", {".pruning"}) &&
      registrar->RegisterAsset<BasicShootDescriptor>("BasicShootDescriptor", {".shoot"}) &&
      registrar->RegisterAsset<BasicRootDescriptor>("BasicRootDescriptor", {".root"}) &&
      registrar->RegisterAsset<BasicFineRootDescriptor>("BasicFineRootDescriptor", {".froot"}) &&
      registrar->RegisterAsset<BasicReproductionModuleDescriptor>("BasicReproductionModuleDescriptor", {".repro"}) &&
      registrar->RegisterAsset<BasicFoliageDescriptor>("BasicFoliageDescriptor", {".foliage"}) &&
      registrar->RegisterAsset<AdvancedShootDescriptor>("AdvancedShootDescriptor", {".ashoot"}) &&
      registrar->RegisterAsset<ModulusGraph>("ModulusGraph", {".evemodulus"}) &&
      registrar->RegisterAsset<StrengthGraph>("StrengthGraph", {".evestrength"}) &&
      registrar->RegisterAsset<BiologicalPropertiesGraph>("TrunkGraph", {".evetrunk"}) &&
      registrar->RegisterAsset<HeightField>("HeightField", {".heightfield"}) &&
      registrar->RegisterAsset<SoilLayerDescriptor>("SoilLayerDescriptor", {".soillayer"}) &&
      registrar->RegisterAsset<SoilDescriptor>("SoilDescriptor", {".soil"}) &&
      registrar->RegisterLayer<EcoSysLabLayer>("EcoSysLab Layer");
  if (registered) {
    RegisterEcoSysLabSerializationHandlers(descriptor.name);
    RegisterEcoSysLabAssetPreviewHandlers(descriptor.name);
    RegisterEcoSysLabInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
