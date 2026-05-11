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
#include "ForestDescriptor.hpp"
#include "FungusTest.hpp"
#include "HeightField.hpp"
#include "ObjectRotator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"
#include "RadialBoundingVolume.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeStructor.hpp"

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "EcoSysLab", "0.1.0", "EcoSysLab runtime package."};
}

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  return registrar->RegisterPrivateComponent<TreeStructor>("TreeStructor") &&
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
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
