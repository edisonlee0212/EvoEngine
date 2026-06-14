#include "StrandsRenderer.hpp"
using namespace evo_engine;

void StrandsRenderer::OnCreate() {
  SetEnabled(true);
}

void StrandsRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
void StrandsRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(strands);
  list.push_back(material);
}
void StrandsRenderer::OnDestroy() {
  strands.Clear();
  material.Clear();

  material.Clear();
  cast_shadow = true;
}
