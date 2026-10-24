#include "PackageManager.hpp"

#include "AdvancedShootDescriptor.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "Blower.hpp"
#include "BranchRenderer.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DsColliders.hpp"
#include "DsOperators.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicStrandsProfiler.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "FoliageRenderer.hpp"
#include "ForestDescriptor.hpp"
#include "FungusTest.hpp"
#include "HeightField.hpp"
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

#include <iterator>
#include <string>
#include <type_traits>

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace eco_sys_lab_package {
void SerializeBlower(YAML::Emitter& out, const Blower& target) {
  out << YAML::Key << "speed" << YAML::Value << target.speed;
  out << YAML::Key << "particle_rate" << YAML::Value << target.particle_rate;
  out << YAML::Key << "spread_degrees" << YAML::Value << target.spread_degrees;
  out << YAML::Key << "seed" << YAML::Value << target.seed;
}

void DeserializeBlower(const YAML::Node& in, Blower& target) {
  if (in["speed"])
    target.speed = in["speed"].as<float>();
  if (in["particle_rate"])
    target.particle_rate = in["particle_rate"].as<float>();
  if (in["spread_degrees"])
    target.spread_degrees = in["spread_degrees"].as<float>();
  if (in["seed"])
    target.seed = in["seed"].as<uint32_t>();
}

void SerializeBranchRenderer(YAML::Emitter& out, const BranchRenderer& target) {
  target.material_ref.Save("material_ref", out);
}

void DeserializeBranchRenderer(const YAML::Node& in, BranchRenderer& target) {
  target.material_ref.Load("material_ref", in);
}

void SerializeFoliageRenderer(YAML::Emitter& out, const FoliageRenderer& target) {
  target.material_ref.Save("material_ref", out);
}

void DeserializeFoliageRenderer(const YAML::Node& in, FoliageRenderer& target) {
  target.material_ref.Load("material_ref", in);
}

void SerializeSpatialPlantDistributionSimulator(YAML::Emitter&, const SpatialPlantDistributionSimulator&) {
}

void DeserializeSpatialPlantDistributionSimulator(const YAML::Node&, SpatialPlantDistributionSimulator&) {
}

void SerializeDynamicTreeSkeleton(YAML::Emitter& out, const DynamicTreeSkeleton& target) {
  const auto& p = target.physics_settings;
  out << YAML::Key << "gpu_physics_enabled" << YAML::Value << target.gpu_physics_enabled;
  out << YAML::Key << "time_step" << YAML::Value << p.time_step;
  out << YAML::Key << "substeps" << YAML::Value << p.substeps;
  out << YAML::Key << "iterations" << YAML::Value << p.iterations;
  out << YAML::Key << "stretch_stiffness" << YAML::Value << p.stretch_stiffness;
  out << YAML::Key << "bend_stiffness" << YAML::Value << p.bend_stiffness;
  out << YAML::Key << "density" << YAML::Value << p.density;
  out << YAML::Key << "damping" << YAML::Value << p.damping;
  out << YAML::Key << "contact_friction" << YAML::Value << p.contact_friction;
  out << YAML::Key << "gravity" << YAML::Value << p.gravity;
  out << YAML::Key << "anchor_offset" << YAML::Value << p.anchor_offset;
  out << YAML::Key << "fracture_enabled" << YAML::Value << p.fracture_enabled;
  out << YAML::Key << "fracture_angle_degrees" << YAML::Value << p.fracture_angle_degrees;
  out << YAML::Key << "fracture_duration" << YAML::Value << p.fracture_duration;
}

void DeserializeDynamicTreeSkeleton(const YAML::Node& in, DynamicTreeSkeleton& target) {
  auto& p = target.physics_settings;
  if (in["gpu_physics_enabled"])
    target.gpu_physics_enabled = in["gpu_physics_enabled"].as<bool>();
  if (in["time_step"])
    p.time_step = in["time_step"].as<float>();
  if (in["substeps"])
    p.substeps = in["substeps"].as<int>();
  if (in["iterations"])
    p.iterations = in["iterations"].as<int>();
  if (in["stretch_stiffness"])
    p.stretch_stiffness = in["stretch_stiffness"].as<float>();
  if (in["bend_stiffness"])
    p.bend_stiffness = in["bend_stiffness"].as<float>();
  if (in["density"])
    p.density = in["density"].as<float>();
  if (in["damping"])
    p.damping = in["damping"].as<float>();
  if (in["contact_friction"])
    p.contact_friction = in["contact_friction"].as<float>();
  if (in["gravity"])
    p.gravity = in["gravity"].as<glm::vec3>();
  if (in["anchor_offset"])
    p.anchor_offset = in["anchor_offset"].as<glm::vec3>();
  if (in["fracture_enabled"])
    p.fracture_enabled = in["fracture_enabled"].as<bool>();
  if (in["fracture_angle_degrees"])
    p.fracture_angle_degrees = in["fracture_angle_degrees"].as<float>();
  if (in["fracture_duration"])
    p.fracture_duration = in["fracture_duration"].as<float>();
  target.RequestGpuPoseInitialization();
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
PackageDescriptor descriptor{
    EVOENGINE_PACKAGE_API_VERSION, "EcoSysLab", "0.1.0", "EcoSysLab runtime package.", EVOENGINE_PACKAGE_BUILD_IDENTITY,
    EVOENGINE_PACKAGE_SOURCE_ID};

void RegisterEcoSysLabSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<Blower>(SerializeBlower, DeserializeBlower, owner_name, "Blower");
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
  Serialization::RegisterSerializationHandler<BranchRenderer>(SerializeBranchRenderer, DeserializeBranchRenderer,
                                                              owner_name, "BranchRenderer");
  Serialization::RegisterSerializationHandler<FoliageRenderer>(SerializeFoliageRenderer, DeserializeFoliageRenderer,
                                                               owner_name, "FoliageRenderer");
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
      registrar->RegisterPrivateComponent<Blower>("Blower") &&
      registrar->RegisterPrivateComponent<DynamicStrandsDemo>("DynamicStrandsDemo") &&
      registrar->RegisterPrivateComponent<Tree>("Tree") &&
      registrar->RegisterPrivateComponent<BranchRenderer>("BranchRenderer") &&
      registrar->RegisterPrivateComponent<FoliageRenderer>("FoliageRenderer") &&
      registrar->RegisterPrivateComponent<Soil>("Soil") &&
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
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar* registrar) {
  return registrar && dynamic_strands_profiler::RegisterItems(*registrar);
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
  DsFungusInjection::ReleaseStaticGpuResources();
}
