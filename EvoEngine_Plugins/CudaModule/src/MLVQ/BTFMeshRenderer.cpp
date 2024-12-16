//
// Created by lllll on 9/3/2021.
//

#include "BTFMeshRenderer.hpp"

using namespace evo_engine;

#include "CompressedBTF.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"

bool BTFMeshRenderer::OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) {
  bool changed = false;

  if (editor_layer->DragAndDropButton<Mesh>(mesh, "Mesh"))
    changed = true;
  if (editor_layer->DragAndDropButton<CompressedBTF>(btf, "CompressedBTF"))
    changed = true;

  return changed;
}

void BTFMeshRenderer::Serialize(YAML::Emitter &out) const {
  mesh.Save("mesh", out);
  btf.Save("btf", out);
}

void BTFMeshRenderer::Deserialize(const YAML::Node &in) {
  mesh.Load("mesh", in);
  btf.Load("btf", in);
}

void BTFMeshRenderer::CollectAssetRef(std::vector<AssetRef> &list) {
  list.push_back(mesh);
  list.push_back(btf);
}