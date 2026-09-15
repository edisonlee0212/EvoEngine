#include "BtfMaterial.hpp"
#include "CBTFGroup.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
using namespace digital_agriculture_package;
bool CBTFGroupInspector::Inspect(InspectorContext& context, CBTFGroup& group) {
  const auto& editor_layer = context.editor_layer;
  auto& btfs = group.btfs;
  bool changed = false;
  auto& temp = ui_temp;
  if (editor_layer->DragAndDropButton<BtfMaterial>(temp, ("Drop to add..."))) {
    btfs.emplace_back(temp);
    temp.Clear();
  }

  if (ImGui::TreeNodeEx("List", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (int i = 0; i < btfs.size(); i++) {
      if (editor_layer->DragAndDropButton<BtfMaterial>(btfs[i], ("No." + std::to_string(i + 1))) &&
          !btfs[i].Get<BtfMaterial>()) {
        btfs.erase(btfs.begin() + i);
        i--;
      }
    }
    ImGui::TreePop();
  }
  return changed;
}
