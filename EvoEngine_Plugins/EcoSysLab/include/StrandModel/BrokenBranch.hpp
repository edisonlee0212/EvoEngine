#pragma once
#include "StrandModelData.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class BrokenBranch : public IPrivateComponent {
 public:
  PrivateComponentRef tree_ref;
  StrandModelStrandGroup strand_group;
  StrandModelStrandGroup subdivided_strand_group;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void InitializeStrandParticles(const StrandModelStrandGroup& strand_group) const;
  void ClearStrandParticles() const;
};
}  // namespace eco_sys_lab_plugin