#pragma once
#include "ProceduralNoiseGraph.hpp"

namespace evo_engine::procedural_noise {
class AddNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

class SubtractNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};
class MultiplyNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};
class DivideNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};
class PowNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

class MinNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

class MaxNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

class FlipUpNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

class FlipDownNode : public INodeImpl {
 public:
  void Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) override;
};

}  // namespace evo_engine::procedural_noise