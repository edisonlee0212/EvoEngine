//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "AdvancedShootDescriptor.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
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
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "FoliageRenderer.hpp"
#include "ForestDescriptor.hpp"
#include "GeometryStorage.hpp"
#include "GpuTreeMeshRenderer.hpp"
#include "GpuTreePhysics.hpp"
#include "GraphicsResources.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
#include "Platform/Platform.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RadialBoundingVolume.hpp"
#include "RenderLayer.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "TextureStorage.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeStructor.hpp"
#include "WindParticleField.hpp"

#include <algorithm>

using namespace eco_sys_lab_package;

void EcoSysLabLayer::OnDestroy() {
  for (const auto& entry : gpu_physics_entries_)
    if (const auto physics = entry.physics.lock())
      physics->SetGpuPhysics(nullptr, 0);
  gpu_physics_entries_.clear();
  gpu_tree_physics_.reset();
  gpu_physics_scene_.reset();
  gpu_tree_mesh_renderer_.reset();
  wind_particle_field_.reset();
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
}

std::weak_ptr<Climate> EcoSysLabLayer::FindClimate() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const std::vector<Entity>* climate_entities = scene->UnsafeGetPrivateComponentOwnersList<Climate>();
  if (climate_entities && !climate_entities->empty()) {
    return scene->GetOrSetPrivateComponent<Climate>(climate_entities->at(0));
  }
  return {};
}

void EcoSysLabLayer::ClearGroundFruitAndLeaf() {
  fruits_.clear();
  leaves_.clear();
  flowers_.clear();
  ++simulation_revision_;
}

float EcoSysLabLayer::GetSimulatedTime() const {
  return simulated_time_;
}

void EcoSysLabLayer::UpdateDemoTreeGrowth() {
  if (demo_tree_growth_finished_)
    return;
  if (!ProjectManager::IsProjectIdle()) {
    demo_background_idle_frames_ = 0;
    return;
  }
  const auto project_path = ProjectManager::GetProjectPath();
  if (project_path.filename() != "test.eveproj" || project_path.parent_path().filename() != "EcoSysLabProject") {
    demo_tree_growth_finished_ = true;
    return;
  }
  const auto scene = GetScene();
  if (!scene) {
    return;
  }
  if (!demo_tree_initialized_) {
    if (!demo_growth_enabled_)
      return;
    if (Platform::MeshShaderEnabled()) {
      if (!gpu_tree_mesh_renderer_)
        gpu_tree_mesh_renderer_ = std::make_shared<GpuTreeMeshRenderer>();
      if (!gpu_tree_mesh_renderer_->Prepare())
        return;
    }
    if (FindClimate().expired()) {
      const auto descriptor =
          std::dynamic_pointer_cast<ClimateDescriptor>(ProjectManager::GetOrCreateAsset("Climate.climate"));
      if (descriptor) {
        const auto entity = scene->CreateEntity("Climate");
        const auto climate = scene->GetOrSetPrivateComponent<Climate>(entity).lock();
        climate->climate_descriptor_ref = descriptor;
        climate->InitializeClimateModel();
      }
    }
    if (FindSoil().expired()) {
      const auto descriptor =
          std::dynamic_pointer_cast<SoilDescriptor>(ProjectManager::GetOrCreateAsset("Soils/PlayGround.soil"));
      if (descriptor) {
        const auto entity = scene->CreateEntity("Soil");
        const auto soil = scene->GetOrSetPrivateComponent<Soil>(entity).lock();
        soil->soil_descriptor_ref = descriptor;
        soil->InitializeSoilModel();
      }
    }
    const auto patch =
        std::dynamic_pointer_cast<ForestPatch>(ProjectManager::GetOrCreateAsset("AppleForest.forestpatch"));
    if (!patch || !patch->tree_descriptor.Get<TreeDescriptor>()) {
      EVOENGINE_ERROR("Failed to load the EcoSysLab demo AppleForest patch.")
      demo_tree_growth_finished_ = true;
      return;
    }
    const size_t previous_tree_count = scene->UnsafeGetPrivateComponentOwnersList<Tree>()
                                           ? scene->UnsafeGetPrivateComponentOwnersList<Tree>()->size()
                                           : 0;
    const auto forest = patch->InstantiatePatch(patch->grid_size, false);
    const auto* trees = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (!scene->IsEntityValid(forest) || !trees || trees->size() != previous_tree_count + 4) {
      EVOENGINE_ERROR("Failed to instantiate four EcoSysLab demo AppleForest trees.")
      demo_tree_growth_finished_ = true;
      return;
    }
    demo_tree_entities_.assign(trees->begin() + previous_tree_count, trees->end());
    if (Platform::MeshShaderEnabled())
      for (const auto entity : demo_tree_entities_)
        scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity).lock()->gpu_physics_enabled = true;
    demo_tree_initialized_ = true;
    EVOENGINE_LOG("Instantiated four EcoSysLab demo AppleForest trees.")
  }
  if (!demo_growth_started_) {
    if (!demo_growth_enabled_)
      return;
    const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
    const auto shader_progress =
        render_layer ? render_layer->GetShaderPreparationProgress() : ShaderPreparationProgress{};
    if (AssetManager::GetAssetLoadSnapshot().Active() || TextureStorage::HasPendingUploads() ||
        GeometryStorage::HasPendingUploads() || BottomLevelAccelerationStructure::HasPendingStaticBuilds() ||
        (render_layer && (!render_layer->IsSceneShaderPrewarmComplete(scene) ||
                          shader_progress.completed != shader_progress.total || !shader_progress.error.empty()))) {
      demo_background_idle_frames_ = 0;
      return;
    }
    if (++demo_background_idle_frames_ < 4)
      return;
    demo_growth_started_ = true;
    EVOENGINE_LOG("Started the EcoSysLab demo AppleForest four-year growth animation.")
  }

  constexpr float target_growth_time = 4.0f * 365.0f;
  if (simulated_time_ < target_growth_time) {
    auto growth_settings = simulation_settings;
    growth_settings.delta_time = std::min(simulation_settings.delta_time, target_growth_time - simulated_time_);
    Simulate(growth_settings, simulation_stats);
  }
  if (simulated_time_ < target_growth_time) {
    return;
  }
  if (!Platform::MeshShaderEnabled()) {
    auto demo_mesh_generator_settings = mesh_generator_settings;
    demo_mesh_generator_settings.foliage_instancing = false;
    for (const auto entity : demo_tree_entities_)
      scene->GetOrSetPrivateComponent<Tree>(entity).lock()->GenerateGeometryEntities(demo_mesh_generator_settings);
  }
  demo_tree_growth_finished_ = true;
  EVOENGINE_LOG("Finished the EcoSysLab demo AppleForest four-year growth animation.")
}

void EcoSysLabLayer::Update() {
  if (const auto scene = GetScene(); !scene)
    return;
  UpdateDemoTreeGrowth();
  SyncGpuTreePhysics();
  RegisterStrandRenderingProcedure();
  if (wind_particle_field_)
    wind_particle_field_->RegisterRender(visualize_wind_particles);
  if (Platform::MeshShaderEnabled()) {
    if (!gpu_tree_mesh_renderer_)
      gpu_tree_mesh_renderer_ = std::make_shared<GpuTreeMeshRenderer>();
    gpu_tree_mesh_renderer_->Register(GetScene());
  }
  DynamicStrandSimulation();
}

void EcoSysLabLayer::FixedUpdate() {
  const auto scene = GetScene();
  if (!scene)
    return;
  if (scene->UnsafeGetPrivateComponentOwnersList<Blower>() || wind_particle_field_) {
    if (!wind_particle_field_)
      wind_particle_field_ = std::make_shared<WindParticleField>();
    wind_particle_field_->Step(scene, static_cast<float>(ApplicationContext::Get().GetTimes().FixedDeltaTime()));
  }
  SyncGpuTreePhysics();
  if (const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeSkeleton>()) {
    for (const auto entity : *entities) {
      if (!scene->IsEntityEnabled(entity) || !scene->HasPrivateComponent<Tree>(entity))
        continue;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
      const auto physics = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity).lock();
      if (tree && tree->IsEnabled() && physics && physics->IsEnabled() && !physics->gpu_physics_enabled) {
        physics->SetGpuPhysics(nullptr, 0);
        physics->StepPhysics(*tree);
      }
    }
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
      wind = {wind_particle_field_->GetDescriptorSet(), wind_particle_field_->GetParticleCount(),
              WindParticleField::grid_bucket_count, WindParticleField::grid_cell_size,
              wind_particle_field_->GetMaxParticleRadius()};
    gpu_tree_physics_->Step(steps, wind);
  }
}

void EcoSysLabLayer::SyncGpuTreePhysics() {
  const auto scene = GetScene();
  if (!scene)
    return;
  const bool growing = demo_growth_started_ && !demo_tree_growth_finished_;
  struct ActiveGpuTree {
    Entity entity;
    std::shared_ptr<Tree> tree;
    std::shared_ptr<DynamicTreeSkeleton> physics;
  };
  std::vector<ActiveGpuTree> active;
  if (const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeSkeleton>()) {
    for (const auto entity : *entities) {
      if (!scene->IsEntityEnabled(entity) || !scene->HasPrivateComponent<Tree>(entity))
        continue;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
      const auto physics = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity).lock();
      if (tree && tree->IsEnabled() && physics && physics->IsEnabled() && physics->gpu_physics_enabled) {
        physics->RestoreGpuPoseAfterLoad(*tree);
        if (physics->SyncGpuPhysics(*tree, scene->GetDataComponent<GlobalTransform>(entity), growing))
          active.push_back({entity, tree, physics});
      }
    }
  }
  bool rebuild = (!gpu_tree_physics_ && !active.empty()) || gpu_physics_scene_.lock() != scene ||
                 active.size() != gpu_physics_entries_.size();
  if (!rebuild) {
    for (size_t i = 0; i < active.size(); ++i) {
      const auto& previous = gpu_physics_entries_[i];
      if (previous.entity != active[i].entity || previous.tree.lock() != active[i].tree ||
          previous.physics.lock() != active[i].physics ||
          previous.revision != active[i].tree->GetShootModelRevision().content ||
          active[i].physics->GetGpuPhysics() != gpu_tree_physics_ || active[i].physics->GetGpuTreeIndex() != i) {
        rebuild = true;
        break;
      }
    }
  }
  if (rebuild) {
    for (const auto& entry : gpu_physics_entries_)
      if (const auto physics = entry.physics.lock())
        physics->SetGpuPhysics(nullptr, 0);
    gpu_physics_entries_.clear();
    gpu_tree_physics_.reset();
    gpu_physics_scene_ = scene;
    if (!active.empty()) {
      std::vector<DynamicTreeSkeleton::GpuRestState> states;
      states.reserve(active.size());
      for (const auto& item : active)
        states.push_back(item.physics->ExportGpuRestState());
      auto shared = std::make_shared<GpuTreePhysics>();
      if (shared->Initialize(states)) {
        gpu_tree_physics_ = shared;
        EVOENGINE_LOG("Initialized shared GPU tree physics for " + std::to_string(active.size()) + " trees.")
        for (uint32_t i = 0; i < active.size(); ++i) {
          const auto& item = active[i];
          item.physics->SetGpuPhysics(shared, i);
          gpu_physics_entries_.push_back(
              {item.entity, item.tree, item.physics, item.tree->GetShootModelRevision().content});
        }
      }
    }
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
