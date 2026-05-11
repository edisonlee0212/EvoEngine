#pragma once
#include "NodeGraph.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

enum class NodeType {
  Unknown,
  // input
  InputSegment,
  InputParticle,
  // modulus
  OutputDensity,
  OutputStretchShearModulus,
  OutputBendingModulus,
  OutputTwistingModulus,
  // strength
  OutputSheerStretchStrength,
  OutputBendingStrength,
  OutputTwistingStrength,
  OutputBundleStrength,
  OutputConnectivityStrength,
  // trunk
  OutputTrunk,

  // Generators
  Constant,  ///< Constant value

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

class IDynamicTreeStrands {
 protected:
  void SerializeImpl(YAML::Emitter& out) const;
  void DeserializeImpl(const YAML::Node& in);

 public:
  virtual void Reset() = 0;
  bool ShowGraph(const std::string& window_title, const std::shared_ptr<EditorLayer>& editor_layer);
  NodeGraph<InputPinData, OutputPinData, NodeData, int> node_graph{};
};

class ModulusGraph : public IAsset, public IDynamicTreeStrands {
 public:
  struct Input {
    float root_distance;
    float polar_distance;
    float polar_angle;
    float profile_boundary_distance;
  };

  struct Output {
    typedef glm::vec2 DensityType;
    typedef glm::vec2 ShearStretchModulusType;
    typedef glm::vec2 BendingModulusType;
    typedef glm::vec2 TwistingModulusType;
    DensityType density = {600.f, 700.f};                           ///< Density range (min, max) for materials.
    ShearStretchModulusType shear_stretch_modulus = {9.5f, 13.5f};  ///< Maximum shear modulus range.
    BendingModulusType bending_modulus = {0.15f, 2.f};              ///< Maximum bending modulus range.
    TwistingModulusType twisting_modulus = {0.15f, 2.f};            ///< Maximum twisting modulus range.
  };

  void Reset() override;
  ModulusGraph();
  ModulusGraph(const Output& output);  ///< Constructor to create constant nodes to be fed into output
  Output::DensityType GetDensity(const Input& input) const;
  void SetDensity(const Output::DensityType& value);
  Output::ShearStretchModulusType GetShearStretchModulus(const Input& input) const;
  void SetShearStretchModulus(const Output::ShearStretchModulusType& value);

  // TODO: needs input from two segments
  Output::BendingModulusType GetBendingModulus(const Input& input) const;
  void SetBendingModulus(const Output::BendingModulusType& value);

  // TODO: needs input from two segments
  Output::TwistingModulusType GetTwistingModulus(const Input& input) const;
  void SetTwistingModulus(const Output::TwistingModulusType& value);
  Output GetValues(const Input& input) const;
  void SetValues(const Output& values);
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

 private:
  template <typename ReturnType>
  ReturnType GetOutput(NodeGraphNodeHandle output_node_handle, const Input& input) const {
    const auto& input_node = node_graph.PeekNode(input_node_handle);
    std::unordered_map<NodeGraphOutputPinHandle, float> results{};
    const auto& output_pin_handles = input_node.GetOutputPinHandles();
    results[output_pin_handles[0]] = input.root_distance;
    results[output_pin_handles[1]] = input.polar_distance;
    results[output_pin_handles[2]] = input.polar_angle;
    results[output_pin_handles[3]] = input.profile_boundary_distance;

    ReturnType output_result(0.0f);

    // Use the stored handle for the output node
    const auto& node = node_graph.PeekNode(output_node_handle);
    node.data.node_impl->Process(node_graph, output_node_handle, results);

    // Process the input pins of the output node
    const auto& input_pin_handles = node.GetInputPinHandles();
    if (input_pin_handles.size() >= 2) {
      for (size_t i = 0; i < 2; ++i) {
        const auto& input_pin = node_graph.PeekInputPin(input_pin_handles[i]);
        const auto link_handle = input_pin.GetLinkHandle();
        if (link_handle != -1) {
          const auto& link = node_graph.PeekLink(link_handle);
          if (const auto search = results.find(link.GetOutputPinHandle()); search != results.end()) {
            output_result[i] = search->second;
          }
        }
      }
    }

    return output_result;
  }

  void SetOutput(NodeGraphNodeHandle output_node_handle, const glm::vec2& value);
  // Store handles to the output nodes for easier access
  NodeGraphNodeHandle output_density_node_handle;
  NodeGraphNodeHandle output_shear_stretch_node_handle;
  NodeGraphNodeHandle output_bending_node_handle;
  NodeGraphNodeHandle output_twisting_node_handle;
  NodeGraphNodeHandle input_node_handle;
};

class StrengthGraph : public IAsset, public IDynamicTreeStrands {
 public:
  struct Input {
    float root_distance;
    float polar_distance;
    float polar_angle;
    float profile_boundary_distance;
  };

  struct Output {
    typedef glm::vec2 ShearStretchStrengthType;
    typedef glm::vec2 BendingStrengthType;
    typedef glm::vec2 TwistingStrengthType;
    typedef glm::vec2 BundleStrengthType;
    typedef glm::vec2 ConnectivityStrengthType;

    ShearStretchStrengthType shear_stretch_strength = {500.f,
                                                       250.f};  ///< Strength settings for shear stretch constraints.
    BendingStrengthType bending_strength = {500.f, 250.f};      ///< Strength settings for bending constraints.
    TwistingStrengthType twisting_strength = {500.f, 250.f};    ///< Strength settings for twisting constraints.
    BundleStrengthType bundle_strength = {500.f, 250.f};        ///< Strength settings for bundle constraints.
    ConnectivityStrengthType connectivity_strength = {250.f,
                                                      125.f};  ///< Strength settings for connectivity constraints.
  };

  void Reset() override;
  StrengthGraph();
  StrengthGraph(const Output& output);  ///< Constructor to create constant nodes to be fed into output
  Output::ShearStretchStrengthType GetShearStretchStrength(const Input& input) const;
  void SetShearStretchStrength(const Output::ShearStretchStrengthType& value);

  // TODO: needs input from two segments
  Output::BendingStrengthType GetBendingStrength(const Input& input) const;
  void SetBendingStrength(const Output::BendingStrengthType& value);
  Output::TwistingStrengthType GetTwistingStrength(const Input& input) const;
  void SetTwistingStrength(const Output::TwistingStrengthType& value);
  Output::BundleStrengthType GetBundleStrength(const Input& input) const;
  void SetBundleStrength(const Output::BundleStrengthType& value);
  Output::ConnectivityStrengthType GetConnectivityStrength(const Input& input) const;
  void SetConnectivityStrength(const Output::ConnectivityStrengthType& value);
  Output GetValues(const Input& input) const;
  void SetValues(const Output& values);
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

 private:
  template <typename ReturnType>
  ReturnType GetOutput(NodeGraphNodeHandle output_node_handle, const Input& input) const {
    const auto& input_node = node_graph.PeekNode(input_node_handle);
    std::unordered_map<NodeGraphOutputPinHandle, float> results{};
    const auto& output_pin_handles = input_node.GetOutputPinHandles();
    results[output_pin_handles[0]] = input.root_distance;
    results[output_pin_handles[1]] = input.polar_distance;
    results[output_pin_handles[2]] = input.polar_angle;
    results[output_pin_handles[3]] = input.profile_boundary_distance;
    ReturnType output_result(0.0f);
    // Use the stored handle for the output node
    const auto& node = node_graph.PeekNode(output_node_handle);
    node.data.node_impl->Process(node_graph, output_node_handle, results);
    // Process the input pins of the output node
    const auto& input_pin_handles = node.GetInputPinHandles();
    if (input_pin_handles.size() >= 2) {
      for (size_t i = 0; i < 2; ++i) {
        const auto& input_pin = node_graph.PeekInputPin(input_pin_handles[i]);
        const auto link_handle = input_pin.GetLinkHandle();
        if (link_handle != -1) {
          const auto& link = node_graph.PeekLink(link_handle);
          if (const auto search = results.find(link.GetOutputPinHandle()); search != results.end()) {
            output_result[i] = search->second;
          }
        }
      }
    }
    return output_result;
  }

  void SetOutput(NodeGraphNodeHandle output_node_handle, const glm::vec2& value);
  // Store handles to the output nodes for easier access
  NodeGraphNodeHandle output_shear_stretch_node_handle;
  NodeGraphNodeHandle output_bending_node_handle;
  NodeGraphNodeHandle output_twisting_node_handle;
  NodeGraphNodeHandle output_bundle_node_handle;
  NodeGraphNodeHandle output_connectivity_node_handle;
  NodeGraphNodeHandle input_node_handle;
};

class BiologicalPropertiesGraph : public IAsset, public IDynamicTreeStrands {
 public:
  struct Input {
    float root_distance;
    float polar_distance;
    float polar_angle;
    float profile_boundary_distance;
  };

  struct Output {
    float trunk_offset = 0.3f;                        ///< Offset distance for trunk-based calculations.
    float trunk_transition = 0.1f;                    ///< Transition factor for trunk segmentation.
    float trunk_additional_strength_factor = 1250.f;  ///< Additional strength factor applied to trunks.
  };

  void Reset() override;
  BiologicalPropertiesGraph();
  BiologicalPropertiesGraph(const Output& output);  ///< Constructor to create constant nodes to be fed into output
  Output GetValues(const Input& input) const;
  void SetValues(const Output& values);
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

 private:
  // Store handles to the output nodes for easier access
  NodeGraphNodeHandle output_node_handle;
  NodeGraphNodeHandle input_node_handle;
};
}  // namespace eco_sys_lab_package