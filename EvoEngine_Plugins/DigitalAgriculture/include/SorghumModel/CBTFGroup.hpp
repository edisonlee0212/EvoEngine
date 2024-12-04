#pragma once
#ifdef OPTIX_RAY_TRACER_PLUGIN
#include "CompressedBTF.hpp"
#endif

using namespace evo_engine;
namespace digital_agriculture_plugin {

class CBTFGroup : public IAsset {
 public:
  std::vector<AssetRef> btfs;
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;
  void CollectAssetRef(std::vector<AssetRef> &list) override;
  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
#ifdef OPTIX_RAY_TRACER_PLUGIN
  std::shared_ptr<CompressedBTF> GetRandom();
#endif
};
}  // namespace digital_agriculture_plugin