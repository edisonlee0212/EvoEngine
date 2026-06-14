#include "DynamicTreeStrandGenerators.hpp"

using namespace eco_sys_lab_package;

bool ConstantNode::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Value", &value, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}

void ConstantNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                           const NodeGraphNodeHandle node_handle,
                           std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
  const auto& node = graph.PeekNode(node_handle);
  for (const auto& output_pin_handle : node.GetOutputPinHandles()) {
    results[output_pin_handle] = value;
  }
}
