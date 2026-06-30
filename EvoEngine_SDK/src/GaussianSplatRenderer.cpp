#include "GaussianSplatRenderer.hpp"

using namespace evo_engine;

void GaussianSplatRenderer::OnCreate() {
  SetEnabled(true);
}

void GaussianSplatRenderer::OnDestroy() {
  gaussian_splat.Clear();
  opacity_scale = 1.0f;
  sh_degree = 0;
  sort_mode = GaussianSplatSortMode::GpuRadix;
  depth_mode = GaussianSplatDepthMode::SceneDepth;
}

void GaussianSplatRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(gaussian_splat);
}

void GaussianSplatRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
