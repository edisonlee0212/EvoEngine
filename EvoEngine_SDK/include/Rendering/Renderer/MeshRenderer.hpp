#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {
class MeshRenderer final : public IPrivateComponent {
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const glm::vec4& color);

 public:
  bool cast_shadow = true;
  AssetRef mesh;
  AssetRef material;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};
}  // namespace evo_engine
