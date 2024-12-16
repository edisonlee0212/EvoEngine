//
// Created by lllll on 10/17/2022.
//

#include "CBTFGroup.hpp"

#ifdef CUDA_MODULE_PLUGIN
#  include "CompressedBTF.hpp"
#endif

using namespace digital_agriculture_plugin;
bool CBTFGroup::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
#ifdef CUDA_MODULE_PLUGIN
  static AssetRef temp;
  if (editor_layer->DragAndDropButton<CompressedBTF>(temp, ("Drop to add..."))) {
    btfs.emplace_back(temp);
    temp.Clear();
  }

  if (ImGui::TreeNodeEx("List", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (int i = 0; i < btfs.size(); i++) {
      if (editor_layer->DragAndDropButton<CompressedBTF>(btfs[i], ("No." + std::to_string(i + 1))) &&
          !btfs[i].Get<CompressedBTF>()) {
        btfs.erase(btfs.begin() + i);
        i--;
      }
    }
    ImGui::TreePop();
  }
#endif
  return changed;
}

void CBTFGroup::CollectAssetRef(std::vector<AssetRef>& list) {
  for (const auto& i : btfs)
    list.push_back(i);
}
void CBTFGroup::Serialize(YAML::Emitter& out) const {
  if (!btfs.empty()) {
    out << YAML::Key << "btfs" << YAML::Value << YAML::BeginSeq;
    for (auto& c_btf : btfs) {
      out << YAML::BeginMap;
      c_btf.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}
void CBTFGroup::Deserialize(const YAML::Node& in) {
  if (auto in_cbt_fs = in["btfs"]) {
    for (const auto& i : in_cbt_fs) {
      AssetRef ref;
      ref.Deserialize(i);
      btfs.emplace_back(ref);
    }
  }
}
#ifdef CUDA_MODULE_PLUGIN
std::shared_ptr<CompressedBTF> CBTFGroup::GetRandom() {
  if (!btfs.empty()) {
    return btfs[glm::linearRand(0, static_cast<int>(btfs.size()) - 1)].Get<CompressedBTF>();
  }
  return {};
}
#endif