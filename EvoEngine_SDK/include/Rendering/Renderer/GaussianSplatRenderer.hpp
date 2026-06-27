#pragma once
#include "GaussianSplat.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {

enum class GaussianSplatSortMode { None = 0, CpuDepth = 1 };
enum class GaussianSplatDepthMode { Always = 0, SceneDepth = 1 };

class GaussianSplatRenderer final : public IPrivateComponent {
 public:
  AssetRef gaussian_splat;
  float opacity_scale = 1.0f;
  int sh_degree = 0;
  GaussianSplatSortMode sort_mode = GaussianSplatSortMode::CpuDepth;
  GaussianSplatDepthMode depth_mode = GaussianSplatDepthMode::SceneDepth;

  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list);
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

}  // namespace evo_engine
