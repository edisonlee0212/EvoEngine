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

class INode;

struct NodeData {
  NodeType type{};
  std::shared_ptr<INode> node_impl;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) const;
};
struct InputPinData {
  std::string name = "Input";
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) const;
};
struct OutputPinData {
  std::string name = "Output";
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) const;
};

class INode {
 public:
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  static void PrepareInputs(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                            NodeGraphNodeHandle node_handle,
                            std::unordered_map<NodeGraphOutputPinHandle, float>& results);
  virtual void Serialize(YAML::Emitter& out) const;
  virtual void Deserialize(const YAML::Node& in);
  virtual void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                       NodeGraphNodeHandle node_handle,
                       std::unordered_map<NodeGraphOutputPinHandle, float>& results) const = 0;
};

class InputNode : public INode {
 public:
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class OutputNode : public INode {
  void Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
               std::unordered_map<NodeGraphOutputPinHandle, float>& results) const override;
};

class IProceduralNoise {
 protected:
  void SerializeImpl(YAML::Emitter& out) const;
  void DeserializeImpl(const YAML::Node& in);

 public:
  virtual void Reset() = 0;
  bool ShowGraph(const std::string& window_title, const std::shared_ptr<EditorLayer>& editor_layer);
  NodeGraph<InputPinData, OutputPinData, NodeData, int> node_graph{};
};

class ProceduralNoise2D : public IAsset, public IProceduralNoise {
  std::shared_ptr<Texture2D> test_texture_2d_;

 public:
  [[nodiscard]] bool SupportsStagedLoading() const override {
    return true;
  }

  void Reset() override;
  ProceduralNoise2D();
  float GetValue(const glm::vec2& offset) const;
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

class ProceduralNoise3D : public IAsset, public IProceduralNoise {
  std::shared_ptr<Texture2D> test_texture_2d_;

 public:
  [[nodiscard]] bool SupportsStagedLoading() const override {
    return true;
  }

  void Reset() override;
  ProceduralNoise3D();
  float GetValue(const glm::vec3& offset) const;
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

class ProceduralNoise4D : public IAsset, public IProceduralNoise {
  std::shared_ptr<Texture2D> test_texture_2d_;

 public:
  [[nodiscard]] bool SupportsStagedLoading() const override {
    return true;
  }

  void Reset() override;
  ProceduralNoise4D();
  float GetValue(const glm::vec4& offset) const;
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine::procedural_noise
