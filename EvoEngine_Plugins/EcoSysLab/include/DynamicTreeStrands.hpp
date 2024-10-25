#pragma once
#include "DynamicStrands.hpp"
#include "StrandModelData.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DsParticleNeighbor;
class DsGravityForce;
class DsRotationUpdate;
class DsPositionUpdate;
class DsExternalForce;
class DynamicTreeStrands : public IPrivateComponent {
 public:
  bool enable_simulation = true;

  PrivateComponentRef tree_ref{};
  StrandModelSkeleton strand_model_skeleton{};
  StrandModelStrandGroup subdivided_strand_group{};

  DynamicStrands::InitializeParameters initialize_parameters{};
  DynamicStrands::StepParameters step_parameters{};
  std::shared_ptr<DynamicStrands> dynamic_strands{};

  GlobalTransform previous_global_transform;
  std::shared_ptr<DsPositionUpdate> base_position_update;
  std::shared_ptr<DsRotationUpdate> base_rotation_update;

  EntityRef operator_entity_ref{};
  GlobalTransform operator_root_transform{};
  std::shared_ptr<DsPositionUpdate> operator_position_update;
  std::shared_ptr<DsRotationUpdate> operator_rotation_update;

  std::shared_ptr<DsExternalForce> external_force;
  std::shared_ptr<DsGravityForce> gravity_force;
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
  void MultipleRodExperimentSetup(float total_length, float segment_length, float radius, const glm::vec2 &intersection);
  void Subdivide(float segment_length, const StrandModelStrandGroup& src);
  void InitializeStrandParticles(const StrandModelStrandGroup& strand_group) const;
  void ClearStrandParticles() const;
  void Step();
};
}  // namespace eco_sys_lab_plugin