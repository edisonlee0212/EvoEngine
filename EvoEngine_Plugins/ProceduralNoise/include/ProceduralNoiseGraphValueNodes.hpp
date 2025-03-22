#pragma once
#include "ProceduralNoiseGraph.hpp"

namespace evo_engine::procedural_noise {
class ConstantNode : public INodeImpl {
 public:
  float value = 0.0f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class PerlinNode : public INodeImpl {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class SimplexNode : public INodeImpl {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class SineNode : public INodeImpl {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace evo_engine::procedural_noise