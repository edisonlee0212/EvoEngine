#pragma once
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicStrands.hpp"
#include "StrandModelData.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {
class DynamicTreeStrands : public IPrivateComponent {
  Handle foliage_rendering_instance_handle;

 public:
  bool enable_simulation = true;

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

  AssetRef material_ref;
  AssetRef leaf_material_ref;
  std::shared_ptr<DsBoxSelection> box_selection_operator;
  std::shared_ptr<DsLineCut> line_cut_operator;
  std::shared_ptr<DsSaw> saw_operator;
  std::shared_ptr<DsDrag> drag_operator;
  std::shared_ptr<DsGravity> gravity;
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
    glm::ivec3 rod_dimension = {10, 40, 20};
    unsigned left_pivot_type = static_cast<unsigned>(PivotType::Transform);
    unsigned right_pivot_type = static_cast<unsigned>(PivotType::Empty);
  };
  struct LogExperimentSetupSettings {
    float segment_length = 0.05f;
    float radius = 0.002f;
    int rod_size = 400;
    int rod_segment_count = 10;
    float center_attraction_strength = 40000;
    unsigned left_pivot_type = static_cast<unsigned>(PivotType::Transform);
    unsigned right_pivot_type = static_cast<unsigned>(PivotType::Empty);
  };
  void BoardExperimentSetup(const BoardExperimentSetupSettings& settings);
  void LogExperimentSetup(const LogExperimentSetupSettings& settings);
  void InitializeStrandParticles(const DtsStrandGroup& target_strand_group) const;
  void ClearStrandParticles() const;

  void InteractionStep() const;

  void PhysicsStep(const DynamicStrands::PhysicsParameters& physics_parameters) const;

  void Visualization(const std::shared_ptr<Camera>& target_camera,
                     const DynamicStrands::VisualizationParameters& visualization_parameters) const;
  void RegisterRenderInstance(const DynamicStrands::RenderParameters& render_parameters);
  void RegisterFoliageRenderInstance(const DynamicStrands::FoliageRenderParameters& render_parameters);
};
}  // namespace eco_sys_lab_plugin