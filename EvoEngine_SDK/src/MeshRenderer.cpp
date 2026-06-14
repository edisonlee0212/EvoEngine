#include "MeshRenderer.hpp"

using namespace evo_engine;

void MeshRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
void MeshRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(mesh);
  list.push_back(material);
}
void MeshRenderer::OnDestroy() {
  mesh.Clear();
  material.Clear();

  cast_shadow = true;
}
