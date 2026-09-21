#pragma once
#include "AssetRef.hpp"
#include "IRuntimeGui.hpp"
namespace evo_engine {
class EVOENGINE_API RuntimeDebugGui final : public IRuntimeGui {
 public:
  AssetRef texture;
  bool show_camera = true;
  void OnGui(RuntimeGuiContext& context) override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  void CollectAssetRef(std::vector<AssetRef>& refs);
};
}  // namespace evo_engine
