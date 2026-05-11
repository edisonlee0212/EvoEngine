//
// Created by lllll on 9/3/2021.
//

#include "BtfMeshRenderer.hpp"

using namespace evo_engine;

#include "BtfMaterial.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"

bool BtfMeshRenderer::OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) {
  bool changed = false;

  if (editor_layer->DragAndDropButton<Mesh>(mesh, "Mesh"))
    changed = true;
  if (editor_layer->DragAndDropButton<BtfMaterial>(btf, "BtfMaterial"))
    changed = true;

  return changed;
}

void BtfMeshRenderer::Serialize(YAML::Emitter &out) const {
  mesh.Save("mesh", out);
  btf.Save("btf", out);
}

void BtfMeshRenderer::Deserialize(const YAML::Node &in) {
  mesh.Load("mesh", in);
  btf.Load("btf", in);
}

void BtfMeshRenderer::CollectAssetRef(std::vector<AssetRef> &list) {
  list.push_back(mesh);
  list.push_back(btf);
}