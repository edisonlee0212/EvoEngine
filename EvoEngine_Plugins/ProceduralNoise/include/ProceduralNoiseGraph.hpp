#pragma once
#include "NodeGraph.hpp"

namespace evo_engine::procedural_noise {
enum class NodeType {
  Unknown,
  Input,
  Output,

  Constant,  ///< Constant value
  Linear,    ///< Linearly varying value

  Sine,     ///< Sine wave
  Simplex,  ///< Simplex noise
  Perlin,   ///< Perlin noise

  Add,       ///< Add operation
  Subtract,  ///< Subtract operation
  Multiply,  ///< Multiply operation
  Divide,    ///< Divide operation
  Pow,       ///< Power operation
  Min,       ///< Minimum operation
  Max,       ///< Maximum operation
  FlipUp,    ///< Flip upwards operation
  FlipDown   ///< Flip downwards operation
};

class INodeImpl;

struct NodeData {
  NodeType type{};
  std::shared_ptr<INodeImpl> node_impl;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};
struct InputPinData {
  std::string name = "Input";
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};
struct OutputPinData {
  std::string name = "Output";
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

class INodeImpl {
 public:
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  static void PrepareInputs(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                            NodeGraphNodeHandle node_handle,
                            std::unordered_map<NodeGraphOutputPinHandle, float>& results);
  virtual void Serialize(YAML::Emitter& out) const;
  virtual void Deserialize(const YAML::Node& in);
  virtual void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
                       std::unordered_map<NodeGraphOutputPinHandle, float>& results) = 0;
};

class InputNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

class OutputNode : public INodeImpl {
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

class ProceduralNoiseGraph : public IAsset {
  void Reset();
  bool ShowGraph(const std::shared_ptr<EditorLayer>& editor_layer);

 public:
  NodeGraph<InputPinData, OutputPinData, NodeData, int> node_graph{};
  float GetValue(const glm::vec4& offset);
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine::procedural_noise
