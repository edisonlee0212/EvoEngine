#pragma once
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicStrands.hpp"
#include "StrandModelData.hpp"
#include "Tree.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {
class DynamicTreeStrands : public IPrivateComponent {
  Handle foliage_rendering_instance_handle;
  Handle small_segments_rendering_instance_handle;
  Handle mesh_wireframe_rendering_instance_handle;

 public:
  int seed = 0;
  StrandModelSkeleton strand_model_skeleton{};
  DtsStrandGroup subdivided_strand_group{};

  DynamicStrands::InitializeParameters initialize_parameters{};
  bool enable_physics = true;
  std::shared_ptr<DynamicStrands> dynamic_strands{};

  struct PivotTransform {
    Entity target_entity;
    std::shared_ptr<DsPivotTransform> ds_pivot_transform;
  };
  struct PivotAxis {
    Entity target_entity;
    std::shared_ptr<DsPivotAxis> ds_pivot_axis;
  };
  struct PivotPoint {
    Entity target_entity;
    std::shared_ptr<DsPivotPoint> ds_pivot_point;
  };
  bool limit_strand_length = false;
  float max_strand_length = 1.f;

  std::vector<PivotPoint> point_pivots;
  std::vector<PivotAxis> axis_pivots;
  std::vector<PivotTransform> transform_pivots;

  AssetRef bark_material_ref;
  AssetRef inner_wood_material_ref;
  AssetRef splinter_material_ref;
  AssetRef leaf_material_ref;
  AssetRef snow_material_ref;
  AssetRef segment_pair_material_ref;
  AssetRef wireframe_material_ref;
  std::shared_ptr<DsBoxSelection> box_selection_operator;
  std::shared_ptr<DsLineCut> line_cut_operator;
  std::shared_ptr<DsPointCut> point_cut_operator;
  std::shared_ptr<DsSaw> saw_operator;
  std::shared_ptr<DsDrag> drag_operator;
  std::shared_ptr<DsLeafDrop> leaf_drop;
  std::shared_ptr<DsSnow> snow;
  std::shared_ptr<DsWind> wind;
  std::shared_ptr<DsStopAll> stop_all;

  void UpdateDynamicStrands();
  void CreateStaticRoot();
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  enum class PivotType { Empty, Point, Axis, Transform };

  struct BoardExperimentSetupSettings {
    float segment_length = 0.05f;
    float radius = 0.002f;
    glm::ivec3 rod_dimension = {20, 40, 20};
    unsigned left_pivot_type = static_cast<unsigned>(PivotType::Transform);
    unsigned right_pivot_type = static_cast<unsigned>(PivotType::Empty);
    float center_damage = 0.5f;
    float center_distance_offset = 0.1f;
    float center_damage_transition = 0.1f;
    glm::vec3 initial_velocity = glm::vec3(0.f);
    glm::vec3 initial_angular_velocity = glm::vec3(0.f);
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  struct LogExperimentSetupSettings {
    float segment_length = 0.05f;
    float radius = 0.002f;
    int rod_size = 800;
    int rod_segment_count = 20;
    float center_attraction_strength = 40000;
    float center_damage = 0.5f;
    float center_distance_offset = 0.1f;
    float center_damage_transition = 0.1f;
    unsigned left_pivot_type = static_cast<unsigned>(PivotType::Transform);
    unsigned right_pivot_type = static_cast<unsigned>(PivotType::Empty);
    glm::vec3 initial_velocity = glm::vec3(0.f);
    glm::vec3 initial_angular_velocity = glm::vec3(0.f);
    bool lock_upper = false;
    bool t_cut = false;
    float t_cut_width = 0.7f;
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  void BoardExperimentSetup(const BoardExperimentSetupSettings& settings);
  void LogExperimentSetup(const LogExperimentSetupSettings& settings);
  void InitializeStrandParticles(const DtsStrandGroup& target_strand_group) const;
  void ClearStrandParticles() const;

  void InteractionStep() const;
  void InitializeFromTree(const std::shared_ptr<Tree>& tree);
  void PhysicsStep(const DynamicStrands::PhysicsParameters& physics_parameters) const;

  void Visualization(const std::shared_ptr<Camera>& target_camera,
                     const DynamicStrands::VisualizationParameters& visualization_parameters) const;
  void RegisterBranchesRenderInstance(const DynamicStrands::BranchesRenderParameters& render_parameters);
  void RegisterBranchesWireframeRenderInstance(const DynamicStrands::BranchesRenderParameters& render_parameters);
  void RegisterSmallSegmentsRenderInstance(const DynamicStrands::SmallSegmentsRenderParameters& render_parameters);
  void RegisterSmallSegmentsVisualizationRenderInstance(
      const DynamicStrands::SmallSegmentsRenderParameters& render_parameters,
      const DynamicStrands::SmallSegmentsVisualizationRenderParameters& visualization_render_parameters);
  void RegisterFoliageRenderInstance(const DynamicStrands::FoliageRenderParameters& render_parameters);

  void RegisterSegmentPairRenderInstance(const DynamicStrands::SegmentPairsRenderParameters& render_parameters);
};
}  // namespace eco_sys_lab_plugin