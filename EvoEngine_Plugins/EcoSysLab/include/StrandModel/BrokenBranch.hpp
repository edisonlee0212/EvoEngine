#pragma once
#include "DynamicStrands.hpp"
#include "StrandModelData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
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

  std::shared_ptr<DsPositionUpdate> position_update;
  std::shared_ptr<DsExternalForce> external_force;
  void UpdateDynamicStrands();
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void LateUpdate() override;
  void FixedUpdate() override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void ExperimentSetup();
  void Subdivide(float segment_length, const StrandModelStrandGroup& src);
  void InitializeStrandParticles(const StrandModelStrandGroup& strand_group) const;
  void ClearStrandParticles() const;
  void Step();
  void InitializeStrandConcaveMesh(const StrandModelStrandGroup& strand_group, float max_edge_length) const;
  void ClearStrandConcaveMesh() const;
};
}  // namespace eco_sys_lab_plugin