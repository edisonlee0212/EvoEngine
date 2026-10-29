//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "AdvancedShootDescriptor.hpp"
#include "Application.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "BranchRenderer.hpp"
#include "Times.hpp"
#ifdef BILLBOARD_CLOUDS_PACKAGE
#  include "BillboardCloudsConverter.hpp"
#endif
#include "Blower.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DsColliders.hpp"
#include "DsOperators.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EcoSysLabSystem.hpp"
#include "FoliageRenderer.hpp"
#include "ForestDescriptor.hpp"
#include "GpuTreeMeshRenderer.hpp"
#include "GpuTreePhysics.hpp"
#include "HeightField.hpp"
#include "Platform/Platform.hpp"
#include "RadialBoundingVolume.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeStructor.hpp"
#include "WindParticleField.hpp"

#include <algorithm>

using namespace eco_sys_lab_package;

void EcoSysLabLayer::OnDestroy() {
  if (const auto scene = GetScene())
    if (const auto system = scene->GetSystem<EcoSysLabSystem>())
      system->dynamics.Reset();
  gpu_tree_mesh_renderer_.reset();
  demo_bootstrap_.reset();
  DsFungusInjection::ReleaseStaticGpuResources();
  DynamicStrands::ReleaseStaticGpuResources();
}

namespace {

void RegisterEcoSysLabSerializationHandlers() {
  evo_engine::Serialization::RegisterSerializationHandler<TreeStructor>(SerializeTreeStructor, DeserializeTreeStructor,
                                                                        {}, "TreeStructor");
  evo_engine::Serialization::RegisterSerializationHandler<Climate>(SerializeClimate, DeserializeClimate, {}, "Climate");
  evo_engine::Serialization::RegisterSerializationHandler<SpatialPlantDistributionSimulator>(
      SerializeSpatialPlantDistributionSimulator, DeserializeSpatialPlantDistributionSimulator, {},
      "SpatialPlantDistributionSimulator");
  evo_engine::Serialization::RegisterSerializationHandler<DynamicTreeSkeleton>(
      SerializeDynamicTreeSkeleton, DeserializeDynamicTreeSkeleton, {}, "DynamicTreeSkeleton");
  evo_engine::Serialization::RegisterSerializationHandler<DynamicStrandsDemo>(
      SerializeDynamicStrandsDemo, DeserializeDynamicStrandsDemo, {}, "DynamicStrandsDemo");
  evo_engine::Serialization::RegisterSerializationHandler<Tree>(SerializeTree, DeserializeTree, {}, "Tree");
  evo_engine::Serialization::RegisterSerializationHandler<BranchRenderer>(
      SerializeBranchRenderer, DeserializeBranchRenderer, {}, "BranchRenderer");
  evo_engine::Serialization::RegisterSerializationHandler<FoliageRenderer>(
      SerializeFoliageRenderer, DeserializeFoliageRenderer, {}, "FoliageRenderer");
  evo_engine::Serialization::RegisterSerializationHandler<Soil>(SerializeSoil, DeserializeSoil, {}, "Soil");
  evo_engine::Serialization::RegisterSerializationHandler<DsBoxCollider>(SerializeDsBoxCollider,
                                                                         DeserializeDsBoxCollider, {}, "DsBoxCollider");
  evo_engine::Serialization::RegisterSerializationHandler<DsSphereCollider>(
      SerializeDsSphereCollider, DeserializeDsSphereCollider, {}, "DsSphereCollider");
  evo_engine::Serialization::RegisterSerializationHandler<DsCylinderCollider>(
      SerializeDsCylinderCollider, DeserializeDsCylinderCollider, {}, "DsCylinderCollider");
  evo_engine::Serialization::RegisterSerializationHandler<DynamicTreeStrands>(
      SerializeDynamicTreeStrands, DeserializeDynamicTreeStrands, {}, "DynamicTreeStrands");
  evo_engine::Serialization::RegisterSerializationHandler<ClimateDescriptor>(
      SerializeClimateDescriptor, DeserializeClimateDescriptor, {}, "ClimateDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<RadialBoundingVolume>(
      SerializeRadialBoundingVolume, DeserializeRadialBoundingVolume, {}, "RadialBoundingVolume");
  evo_engine::Serialization::RegisterSerializationHandler<CubeVolume>(SerializeCubeVolume, DeserializeCubeVolume, {},
                                                                      "CubeVolume");
  evo_engine::Serialization::RegisterSerializationHandler<ForestPatch>(SerializeForestPatch, DeserializeForestPatch, {},
                                                                       "ForestPatch");
  evo_engine::Serialization::RegisterSerializationHandler<BasicBarkDescriptor>(
      SerializeBasicBarkDescriptor, DeserializeBasicBarkDescriptor, {}, "BasicBarkDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<ForestDescriptor>(
      SerializeForestDescriptor, DeserializeForestDescriptor, {}, "ForestDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<TreeDescriptor>(
      SerializeTreeDescriptor, DeserializeTreeDescriptor, {}, "TreeDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicPruningDescriptor>(
      SerializeBasicPruningDescriptor, DeserializeBasicPruningDescriptor, {}, "BasicPruningDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicShootDescriptor>(
      SerializeBasicShootDescriptor, DeserializeBasicShootDescriptor, {}, "BasicShootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicRootDescriptor>(
      SerializeBasicRootDescriptor, DeserializeBasicRootDescriptor, {}, "BasicRootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicFineRootDescriptor>(
      SerializeBasicFineRootDescriptor, DeserializeBasicFineRootDescriptor, {}, "BasicFineRootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicReproductionModuleDescriptor>(
      SerializeBasicReproductionModuleDescriptor, DeserializeBasicReproductionModuleDescriptor, {},
      "BasicReproductionModuleDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicFoliageDescriptor>(
      SerializeBasicFoliageDescriptor, DeserializeBasicFoliageDescriptor, {}, "BasicFoliageDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<AdvancedShootDescriptor>(
      SerializeAdvancedShootDescriptor, DeserializeAdvancedShootDescriptor, {}, "AdvancedShootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<ModulusGraph>(SerializeModulusGraph, DeserializeModulusGraph,
                                                                        {}, "ModulusGraph");
  evo_engine::Serialization::RegisterSerializationHandler<StrengthGraph>(SerializeStrengthGraph,
                                                                         DeserializeStrengthGraph, {}, "StrengthGraph");
  evo_engine::Serialization::RegisterSerializationHandler<BiologicalPropertiesGraph>(
      SerializeBiologicalPropertiesGraph, DeserializeBiologicalPropertiesGraph, {}, "TrunkGraph");
  evo_engine::Serialization::RegisterSerializationHandler<HeightField>(SerializeHeightField, DeserializeHeightField, {},
                                                                       "HeightField");
  evo_engine::Serialization::RegisterSerializationHandler<SoilLayerDescriptor>(
      SerializeSoilLayerDescriptor, DeserializeSoilLayerDescriptor, {}, "SoilLayerDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<SoilDescriptor>(
      SerializeSoilDescriptor, DeserializeSoilDescriptor, {}, "SoilDescriptor");
}
}  // namespace

void EcoSysLabLayer::RegisterTypes(Application& application) {
  application.RegisterSystem<EcoSysLabSystem>("EcoSysLabSystem");
  application.RegisterPrivateComponent<TreeStructor>("TreeStructor");
  application.RegisterPrivateComponent<Climate>("Climate");
  application.RegisterPrivateComponent<SpatialPlantDistributionSimulator>("SpatialPlantDistributionSimulator");
  application.RegisterPrivateComponent<DynamicTreeSkeleton>("DynamicTreeSkeleton");
  application.RegisterPrivateComponent<DynamicStrandsDemo>("DynamicStrandsDemo");
  application.RegisterPrivateComponent<Tree>("Tree");
  application.RegisterPrivateComponent<BranchRenderer>("BranchRenderer");
  application.RegisterPrivateComponent<FoliageRenderer>("FoliageRenderer");
  application.RegisterPrivateComponent<Soil>("Soil");
  application.RegisterPrivateComponent<DsBoxCollider>("DsBoxCollider");
  application.RegisterPrivateComponent<DsSphereCollider>("DsSphereCollider");
  application.RegisterPrivateComponent<DsCylinderCollider>("DsCylinderCollider");
  application.RegisterPrivateComponent<DynamicTreeStrands>("DynamicTreeStrands");
#ifdef BILLBOARD_CLOUDS_PACKAGE
  application.RegisterPrivateComponent<BillboardCloudsConverter>("BillboardCloudsConverter");
#endif

  application.RegisterAsset<ClimateDescriptor>("ClimateDescriptor", {".climate"});
  application.RegisterAsset<RadialBoundingVolume>("RadialBoundingVolume", {".rbv"});
  application.RegisterAsset<CubeVolume>("CubeVolume", {".cubevolume"});
  application.RegisterAsset<ForestPatch>("ForestPatch", {".forestpatch"});
  application.RegisterAsset<BasicBarkDescriptor>("BasicBarkDescriptor", {".bark"});
  application.RegisterAsset<ForestDescriptor>("ForestDescriptor", {".forest"});
  application.RegisterAsset<TreeDescriptor>("TreeDescriptor", {".tree"});
  application.RegisterAsset<BasicPruningDescriptor>("BasicPruningDescriptor", {".pruning"});
  application.RegisterAsset<BasicShootDescriptor>("BasicShootDescriptor", {".shoot"});
  application.RegisterAsset<BasicRootDescriptor>("BasicRootDescriptor", {".root"});
  application.RegisterAsset<BasicFineRootDescriptor>("BasicFineRootDescriptor", {".froot"});
  application.RegisterAsset<BasicReproductionModuleDescriptor>("BasicReproductionModuleDescriptor", {".repro"});
  application.RegisterAsset<BasicFoliageDescriptor>("BasicFoliageDescriptor", {".foliage"});
  application.RegisterAsset<AdvancedShootDescriptor>("AdvancedShootDescriptor", {".ashoot"});
  application.RegisterAsset<ModulusGraph>("ModulusGraph", {".evemodulus"});
  application.RegisterAsset<StrengthGraph>("StrengthGraph", {".evestrength"});
  application.RegisterAsset<BiologicalPropertiesGraph>("TrunkGraph", {".evetrunk"});
  application.RegisterAsset<HeightField>("HeightField", {".heightfield"});
  application.RegisterAsset<SoilLayerDescriptor>("SoilLayerDescriptor", {".soillayer"});
  application.RegisterAsset<SoilDescriptor>("SoilDescriptor", {".soil"});
  RegisterEcoSysLabSerializationHandlers();
  Serialization::RegisterSerializationHandler<EcoSysLabSystem>(SerializeEcoSysLabSystem, DeserializeEcoSysLabSystem, {},
                                                               "EcoSysLabSystem");
}

void EcoSysLabLayer::ClearGroundFruitAndLeaf() {
  if (const auto scene = GetScene())
    if (const auto system = scene->GetSystem<EcoSysLabSystem>())
      system->interactions.Clear();
}

float EcoSysLabLayer::GetSimulatedTime() const {
  const auto scene = GetScene();
  const auto system = scene ? scene->GetSystem<EcoSysLabSystem>() : nullptr;
  return system ? system->growth.simulated_time : 0.0f;
}

void EcoSysLabLayer::Update() {
  const auto scene = GetScene();
  if (!scene)
    return;
  auto system = scene->GetSystem<EcoSysLabSystem>();
  if (!system) {
    system = scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
    system->Enable();
  }
  system->environment.EnsureConfiguration(scene);
  UpdateDemoTreeGrowth();
  system->dynamics.UpdateAir(scene, system->environment,
                             system->growth.remaining_time > 0.0f || system->growth.grew_this_frame);
  RegisterStrandRenderingProcedure();
  if (system->dynamics.wind)
    system->dynamics.wind->RegisterRender(visualize_wind_particles, visualize_air_grid, visualize_air_coverage);
  if (Platform::MeshShaderEnabled()) {
    if (!gpu_tree_mesh_renderer_)
      gpu_tree_mesh_renderer_ = std::make_shared<GpuTreeMeshRenderer>();
    gpu_tree_mesh_renderer_->Register(GetScene());
  }
}

void EcoSysLabSystem::Dynamics::Step(const std::shared_ptr<Scene>& scene, const float delta_time, const bool growing) {
  auto& wind_particle_field_ = wind;
  auto& gpu_tree_physics_ = tree_physics;
  auto& gpu_physics_entries_ = entries;
  SyncTrees(scene, growing);
  if (scene->UnsafeGetPrivateComponentOwnersList<Blower>() || wind_particle_field_) {
    if (!wind_particle_field_)
      wind_particle_field_ = std::make_shared<WindParticleField>();
    wind_particle_field_->Step(scene, delta_time, gpu_tree_physics_);
  }
  if (gpu_tree_physics_) {
    std::vector<GpuTreePhysics::TreeStep> steps;
    steps.reserve(gpu_physics_entries_.size());
    for (const auto& item : gpu_physics_entries_) {
      const auto physics = item.physics.lock();
      steps.push_back(
          {physics->physics_settings, physics->physics_running, scene->GetDataComponent<GlobalTransform>(item.entity)});
    }
    GpuTreePhysics::WindInput wind;
    if (wind_particle_field_)
      wind = {wind_particle_field_->GetAirDescriptorSet(), wind_particle_field_->GetAirCellCount(),
              GpuSpatialGrid::bucket_count, wind_particle_field_->GetAirCellSize()};
    gpu_tree_physics_->Step(steps, wind);
  }
}

void EcoSysLabLayer::OnCreate() {
  Shader::RegisterShaderIncludePath(std::filesystem::path("./EcoSysLabResources/Shaders/Modules"));
}

EcoSysLabLayer::EcoSysLabLayer() {
  if (soil_layer_colors.empty()) {
    for (int i = 0; i < 10; i++) {
      glm::vec4 color = {glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)), 1.0f};
      soil_layer_colors.emplace_back(color);
    }
  }
}
