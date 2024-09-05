#pragma once

using namespace evo_engine;
namespace digital_agriculture {

class DoubleCBTF : public IAsset {
 public:
  AssetRef top;
  AssetRef bottom;
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;
  void CollectAssetRef(std::vector<AssetRef> &list) override;
  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
};
}  // namespace digital_agriculture