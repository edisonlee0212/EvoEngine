#pragma once
#include "ProceduralNoise.hpp"

namespace evo_engine::procedural_noise {
class EVOENGINE_API ConstantNode : public INode {
 public:
  float value = 0.0f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API Perlin2DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API Simplex2DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API Perlin3DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API Simplex3DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API Perlin4DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API Simplex4DNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API SineNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};
class EVOENGINE_API CosineNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};
class EVOENGINE_API TangentNode : public INode {
 public:
  float frequency = 1.f;
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};
}  // namespace evo_engine::procedural_noise
