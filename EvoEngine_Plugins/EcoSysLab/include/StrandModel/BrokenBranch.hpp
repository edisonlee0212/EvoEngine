#pragma once
#include "DynamicStrands.hpp"
#include "StrandModelData.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class BrokenBranch : public IPrivateComponent {
 public:
  
  bool enable_simulation = true;

  PrivateComponentRef tree_ref;
  StrandModelStrandGroup strand_group;
  StrandModelStrandGroup subdivided_strand_group;

  DynamicStrands dynamic_strands;
  void UploadStrands();
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
  void LateUpdate() override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void ExperimentSetup();
  static void Subdivide(float segment_length, const StrandModelStrandGroup& src, StrandModelStrandGroup& dst);
  void InitializeStrandParticles(const StrandModelStrandGroup& strand_group) const;
  void ClearStrandParticles() const;

  void InitializeStrandConcaveMesh(const StrandModelStrandGroup& strand_group, float max_edge_length) const;
  void ClearStrandConcaveMesh() const;
};
}  // namespace eco_sys_lab_plugin