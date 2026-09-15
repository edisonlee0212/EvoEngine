#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "IPrivateComponent.hpp"

namespace evo_engine {
class BtfMeshRenderer : public IPrivateComponent {
 public:
  AssetRef mesh;
  AssetRef btf;

  void CollectAssetRef(std::vector<AssetRef> &list);
};
}  // namespace evo_engine
