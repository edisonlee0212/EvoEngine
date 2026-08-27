//
// Created by lllll on 9/3/2021.
//

#include "DigitalAgricultureSerializationAdapters.hpp"

using namespace evo_engine;

#include "BtfMaterial.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"

bool BtfMeshRenderer::DrawGui(const std::shared_ptr<EditorLayer> &editor_layer) {
  bool changed = false;

  if (editor_layer->DragAndDropButton<Mesh>(mesh, "Mesh"))
    changed = true;
  if (editor_layer->DragAndDropButton<BtfMaterial>(btf, "BtfMaterial"))
    changed = true;

  return changed;
}

void evo_engine::SerializeBtfMeshRenderer(YAML::Emitter &out, const BtfMeshRenderer &target) {
  target.mesh.Save("mesh", out);
  target.btf.Save("btf", out);
}

void evo_engine::DeserializeBtfMeshRenderer(const YAML::Node &in, BtfMeshRenderer &target) {
  target.mesh.Load("mesh", in);
  target.btf.Load("btf", in);
}

void BtfMeshRenderer::CollectAssetRef(std::vector<AssetRef> &list) {
  list.push_back(mesh);
  list.push_back(btf);
}
