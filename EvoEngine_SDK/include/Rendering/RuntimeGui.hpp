#pragma once
#include "IPrivateComponent.hpp"
#include "PrivateComponentRef.hpp"

namespace evo_engine {
class EVOENGINE_API RuntimeGui final : public IPrivateComponent {
  std::vector<AssetRef> gui_assets_;
  std::string layout_;

 public:
  PrivateComponentRef camera;
  int draw_order = 0;

  const std::vector<AssetRef>& GetGuiAssets() const;
  bool AddGuiAsset(AssetRef asset);
  bool RemoveGuiAsset(size_t index);
  bool MoveGuiAsset(size_t from, size_t to);
  const std::string& GetLayout() const;
  void SetLayout(std::string layout);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene);
  void CollectAssetRef(std::vector<AssetRef>& list);
};
}  // namespace evo_engine
