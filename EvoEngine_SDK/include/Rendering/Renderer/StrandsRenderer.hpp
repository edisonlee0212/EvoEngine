#pragma once
#include "IPrivateComponent.hpp"
#include "Material.hpp"
#include "Strands.hpp"
namespace evo_engine {
class StrandsRenderer : public IPrivateComponent {
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, glm::vec4& color);

 public:
  bool cast_shadow = true;
  AssetRef strands;
  AssetRef material;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};
}  // namespace evo_engine
