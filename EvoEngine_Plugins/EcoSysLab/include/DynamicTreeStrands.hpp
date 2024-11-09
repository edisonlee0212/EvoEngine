#pragma once
#include "DynamicStrands.hpp"
#include "StrandModelData.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DsParticleNeighbor;
class DsGravity;
class DsTransform;
class DsDragForce;
class DynamicTreeStrands : public IPrivateComponent {
 public:
  bool enable_simulation = true;

  PrivateComponentRef tree_ref{};
  StrandModelSkeleton strand_model_skeleton{};
  StrandModelStrandGroup subdivided_strand_group{};

  DynamicStrands::InitializeParameters initialize_parameters{};
  bool enable_physics = false;
  bool enable_visualization = false;
  DynamicStrands::PhysicsParameters physics_parameters{};
  DynamicStrands::VisualizationParameters visualization_parameters{};
  std::shared_ptr<DynamicStrands> dynamic_strands{};

  struct TransformOperator {
    Entity target_entity;
    std::shared_ptr<DsTransform> ds_transform;
  };
  struct DragForceOperator {
    Entity target_entity;
    std::shared_ptr<DsDragForce> ds_drag_force;
  };
  std::vector<TransformOperator> transform_operators;
  std::vector<DragForceOperator> drag_force_operators;
  std::shared_ptr<DsGravity> gravity;
  std::shared_ptr<DsParticleNeighbor> connectivity;
  void UpdateDynamicStrands();
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void LateUpdate() override;
  void FixedUpdate() override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void SingleRodExperimentSetup(float total_length, float segment_length);
  void MultipleRodExperimentSetup(float total_length, float segment_length, float radius, const glm::vec2& intersection,
                                  bool add_operator);
  void Subdivide(float segment_length, const StrandModelStrandGroup& src);
  void InitializeStrandParticles(const StrandModelStrandGroup& strand_group) const;
  void ClearStrandParticles() const;
  void Step();
};
}  // namespace eco_sys_lab_plugin