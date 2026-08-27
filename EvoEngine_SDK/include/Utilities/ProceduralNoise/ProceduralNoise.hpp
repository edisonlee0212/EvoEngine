#pragma once
#include "NodeGraph.hpp"
#include "Texture2D.hpp"

namespace evo_engine::procedural_noise {
enum class NodeType {
  Unknown,
  Input,
  Output,

  // Generators
  Constant,  ///< Constant value
  Sine,      ///< Sine wave
  Cosine,    ///< Cosine wave
  Tangent,   ///< Tangent wave

  Simplex2D,  ///< Simplex2D noise
  Simplex3D,  ///< Simplex3D noise
  Simplex4D,  ///< Simplex4D noise
  Perlin2D,   ///< Perlin2D noise
  Perlin3D,   ///< Perlin2D noise
  Perlin4D,   ///< Perlin2D noise

  // Combiners
  Add,       ///< Add operation
  Subtract,  ///< Subtract operation
  Multiply,  ///< Multiply operation
  Divide,    ///< Divide operation
  Power,     ///< Power operation
  Min,       ///< Minimum operation
  Max,       ///< Maximum operation

  // Modifiers
  Abs,
  Clamp,
  Exponent,
  Negate,
  FlipUp,    ///< Flip upwards operation
  FlipDown,  ///< Flip downwards operation

  // Activation
  Sigmoid,
  SoftSign,
  Tanh,

};

class EVOENGINE_API INode;

struct NodeData {
  NodeType type{};
  std::shared_ptr<INode> node_impl;
};
struct InputPinData {
  std::string name = "Input";
};
struct OutputPinData {
  std::string name = "Output";
};

class EVOENGINE_API INode {
 public:
  static void PrepareInputs(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                            NodeGraphNodeHandle node_handle,
                            std::unordered_map<NodeGraphOutputPinHandle, float>& results);
  virtual void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                       NodeGraphNodeHandle node_handle,
                       std::unordered_map<NodeGraphOutputPinHandle, float>& results) const = 0;
};

class EVOENGINE_API InputNode : public INode {
 public:
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API OutputNode : public INode {
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class EVOENGINE_API IProceduralNoise {
 public:
  virtual void Reset() = 0;
  NodeGraph<InputPinData, OutputPinData, NodeData, int> node_graph{};
};

void SaveProceduralNoiseGraph(YAML::Emitter& out, const IProceduralNoise& noise);
void LoadProceduralNoiseGraph(const YAML::Node& in, IProceduralNoise& noise);

class EVOENGINE_API ProceduralNoise2D : public IAsset, public IProceduralNoise {
 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  void Reset() override;
  ProceduralNoise2D();
  float GetValue(const glm::vec2& offset) const;
  void OnCreate() override;
};

class EVOENGINE_API ProceduralNoise3D : public IAsset, public IProceduralNoise {
 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  void Reset() override;
  ProceduralNoise3D();
  float GetValue(const glm::vec3& offset) const;
  void OnCreate() override;
};

class EVOENGINE_API ProceduralNoise4D : public IAsset, public IProceduralNoise {
 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  void Reset() override;
  ProceduralNoise4D();
  float GetValue(const glm::vec4& offset) const;
  void OnCreate() override;
};
}  // namespace evo_engine::procedural_noise
