#pragma once

using namespace evo_engine;
namespace digital_agriculture {

class CBTFGroup : public IAsset {
 public:
  std::vector<AssetRef> m_doubleCBTFs;
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;
  void CollectAssetRef(std::vector<AssetRef> &list) override;
  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
  AssetRef GetRandom() const;
};
}  // namespace digital_agriculture