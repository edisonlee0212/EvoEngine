#pragma once
using namespace evo_engine;
namespace eco_sys_lab {
class TreeDescriptor : public IAsset {
 public:
  AssetRef shoot_descriptor;
  AssetRef foliage_descriptor;

  AssetRef fruit_descriptor;
  AssetRef flower_descriptor;

  AssetRef bark_descriptor;
  void OnCreate() override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;
};

}  // namespace eco_sys_lab