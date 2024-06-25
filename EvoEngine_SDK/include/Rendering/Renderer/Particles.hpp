#pragma once
#include "Material.hpp"
#include "Mesh.hpp"
#include "Scene.hpp"
namespace evo_engine {
class Particles : public IPrivateComponent {
 public:
  void OnCreate() override;
  Bound bounding_box;
  bool cast_shadow = true;
  AssetRef particle_info_list;
  AssetRef mesh;
  AssetRef material;
  void RecalculateBoundingBox();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void OnDestroy() override;
};
}  // namespace evo_engine
