#pragma once
#include "DynamicTreeStrandGraph.hpp"

namespace eco_sys_lab_package {
class ConstantNode : public INode {
 public:
  float value = 0.0f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace eco_sys_lab_package