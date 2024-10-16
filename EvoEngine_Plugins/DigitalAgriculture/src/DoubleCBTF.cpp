//
// Created by lllll on 11/16/2022.
//

#include "DoubleCBTF.hpp"
#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include "CompressedBTF.hpp"
#endif
using namespace digital_agriculture_plugin;
bool DoubleCBTF::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
#ifdef OPTIX_RAY_TRACER_PLUGIN
  if (editor_layer->DragAndDropButton<CompressedBTF>(top, "Top"))
    changed = true;
  if (editor_layer->DragAndDropButton<CompressedBTF>(bottom, "Bottom"))
    changed = true;
#endif
  return changed;
}
void DoubleCBTF::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(top);
  list.push_back(bottom);
}
void DoubleCBTF::Serialize(YAML::Emitter& out) const {
  top.Save("top", out);
  bottom.Save("bottom", out);
}
void DoubleCBTF::Deserialize(const YAML::Node& in) {
  top.Load("top", in);
  bottom.Load("bottom", in);
  top.Load("m_top", in);
  bottom.Load("m_bottom", in);
}
