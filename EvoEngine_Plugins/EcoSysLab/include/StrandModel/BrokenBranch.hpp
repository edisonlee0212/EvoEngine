#pragma once
#include "DynamicStrands.hpp"
#include "StrandModelData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DsParticleNeighbor;
class DsRotationUpdate;
class DsPositionUpdate;
class DsExternalForce;
class BrokenBranch : public IPrivateComponent {
 public:
  
  bool enable_simulation = true;

  PrivateComponentRef tree_ref{};
  StrandModelStrandGroup strand_group{};
  StrandModelStrandGroup subdivided_strand_group{};

  DynamicStrands::InitializeParameters initialize_parameters{};
  DynamicStrands::StepParameters step_parameters{};
  DynamicStrands dynamic_strands{};

  GlobalTransform previous_global_transform;
  std::shared_ptr<DsPositionUpdate> position_update;
  std::shared_ptr<DsRotationUpdate> rotation_update;
  std::shared_ptr<DsExternalForce> external_force;
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
  void ExperimentSetup(float total_length, float segment_length);
  void Subdivide(float segment_length, const StrandModelStrandGroup& src);
  void InitializeStrandParticles(const StrandModelStrandGroup& strand_group) const;
  void ClearStrandParticles() const;
  void Step();
  void InitializeStrandConcaveMesh(const StrandModelStrandGroup& strand_group, float max_edge_length) const;
  void ClearStrandConcaveMesh() const;
};
}  // namespace eco_sys_lab_plugin