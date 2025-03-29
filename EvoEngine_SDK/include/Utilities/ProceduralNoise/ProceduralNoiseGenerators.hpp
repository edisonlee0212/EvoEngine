#pragma once
#include "ProceduralNoise.hpp"

namespace evo_engine::procedural_noise {
class ConstantNode : public INode {
 public:
  float value = 0.0f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class Perlin2DNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class Simplex2DNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class Perlin3DNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class Simplex3DNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class Perlin4DNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class Simplex4DNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class SineNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
class CosineNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
class TangentNode : public INode {
 public:
  float frequency = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace evo_engine::procedural_noise