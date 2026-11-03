#pragma once

#include <optional>
#include "ClimateModel.hpp"
#include "DynamicStrands.hpp"
#include "Entity.hpp"
#include "ISystem.hpp"
#include "SimulationSettings.hpp"
#include "Transform.hpp"
#include "TreePhysicsSettings.hpp"
#include "VoxelSoilModel.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;
class GpuTreePhysics;
class GpuTreeClusterPhysics;
class GpuTreeClusterForest;
struct GpuTreePhysicsState;
class DynamicTreeSkeleton;
class WindParticleField;
class Tree;

class EcoSysLabSystem : public ISystem {
 public:
  struct Rendering {
    bool screen_space_tessellation = true;
    bool camera_impostors = true;
    bool canopy_shadows = true;
    float canopy_shadow_texels = 16.0f;
    float impostor_diameter_pixels = 3.0f;
    float pixel_error = 0.5f;
    float segment_length_pixels = 12.0f;
  } rendering;

  struct Environment {
    AssetRef climate_descriptor;
    AssetRef soil_descriptor;
    Handle soil_visualization_owner = Handle(0);
    float air_temperature_c = 20.0f;
    float air_relative_humidity = 0.5f;
    float air_pressure_pa = 101325.0f;
    float air_coupling_rate = 4.0f;
    float air_friction = 4.0f;
    float air_cell_size = 0.1f;
    ClimateModel climate;
    VoxelSoilModel soil;
    bool initialized = false;

    void EnsureConfiguration(const std::shared_ptr<Scene>& scene);
    Entity GetOrCreateVisualizationOwner(const std::shared_ptr<Scene>& scene);
    void InitializeModels();
    void PrepareForGrowth(const std::shared_ptr<Scene>& scene, const SimulationSettings& settings);
  } environment;

  struct Growth {
    float remaining_time = 0.0f;
    float simulated_time = 0.0f;
    bool grew_this_frame = false;
    SimulationSettings settings{};
    SimulationStats stats{};
    std::optional<std::pair<Entity, int>> invigoration;

    void Queue(float days);
    float TakeStep();
    bool Step(EcoSysLabSystem& system, const SimulationSettings& settings, SimulationStats& stats);
    void Stop();
    void RequestInvigoration(Entity tree, int node);
  } growth;

  struct Interactions {
    struct Flower {
      GlobalTransform global_transform;
      float flower_maturity = 0.0f;
      float flower_health = 1.0f;
    };
    struct Fruit {
      GlobalTransform global_transform;
      float fruit_maturity = 0.0f;
      float fruit_health = 1.0f;
    };
    struct Leaf {
      GlobalTransform global_transform;
      float leaf_maturity = 0.0f;
      float leaf_health = 1.0f;
    };
    std::vector<Flower> flowers;
    std::vector<Fruit> fruits;
    std::vector<Leaf> leaves;
    uint64_t revision = 0;
    uint64_t reset_revision = 0;
    void Clear();
  } interactions;

  struct Dynamics {
    float physics_approximation_tolerance = 0.0f;
    bool use_subtree_clusters = false;
    TreeClusterLodSettings cluster_lod{2};
    struct StrandsSettings {
      FoliageRenderParameters foliage_render_parameters{};
      SegmentPairsRenderParameters segment_pairs_render_parameters{};
      DynamicStrands::PhysicsParameters physics_parameters{};
      DynamicStrands::FungusParameters fungus_parameters{};
      bool enable_physics = true;
      bool enable_fungus = true;
      bool enable_geometry_updates = true;
      bool enable_rendering = true;
      int remaining_step = 0;
      int remaining_fungus_step = 0;
      int remaining_geometry_step = 0;
      int fungus_sub_step = 25;
    } strands;
    struct Entry {
      Entity entity;
      std::weak_ptr<Tree> tree;
      std::weak_ptr<DynamicTreeSkeleton> physics;
      uint64_t revision = 0;
      uint32_t gpu_id = UINT32_MAX;
      std::shared_ptr<GpuTreeClusterPhysics> cluster;
      std::shared_ptr<GpuTreePhysicsState> cluster_source;
      bool cluster_running = false;
      uint32_t pose_revision = 0;
    };
    std::shared_ptr<GpuTreePhysics> tree_physics;
    std::shared_ptr<GpuTreeClusterForest> cluster_forest;
    std::vector<std::shared_ptr<GpuTreeClusterPhysics>> cluster_members;
    std::weak_ptr<Scene> physics_scene;
    std::vector<Entry> entries;
    std::shared_ptr<WindParticleField> wind;

    void Reset();
    void SyncClusters(bool growing);
    bool StepClusters(const std::shared_ptr<Scene>& scene);
    void SyncTrees(const std::shared_ptr<Scene>& scene, bool growing);
    void UpdateAir(const std::shared_ptr<Scene>& scene, Environment& environment, bool growing);
    void Step(const std::shared_ptr<Scene>& scene, float delta_time, bool growing);
    void ResetAirGrid(const std::shared_ptr<Scene>& scene, float cell_size);
    void StepStrands(const std::shared_ptr<Scene>& scene);
  } dynamics;

  void Update() override;
  void FixedUpdate() override;
  void OnDestroy() override;
  void PostCloneAction(const std::shared_ptr<ISystem>& target) override;
  void CollectAssetRef(std::vector<AssetRef>& list);
};

void SerializeEcoSysLabSystem(YAML::Emitter& out, const EcoSysLabSystem& target);
void DeserializeEcoSysLabSystem(const YAML::Node& in, EcoSysLabSystem& target);
}  // namespace eco_sys_lab_package
