#pragma once
#include "ProceduralNoise.hpp"

namespace evo_engine::procedural_noise {
class ConstantNode : public INode {
 public:
  float value = 0.0f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class Perlin2DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class Simplex2DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class Perlin3DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class Simplex3DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class Perlin4DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class Simplex4DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class SineNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};
class CosineNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};
class TangentNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};
}  // namespace evo_engine::procedural_noise
