#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrandGenerators.hpp"
#include "DynamicTreeStrandOperators.hpp"
#include "EcoSysLabSerializationAdapters.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

static void SerializeDynamicTreeStrandsGraph(YAML::Emitter& out, const IDynamicTreeStrands& target);
static void DeserializeDynamicTreeStrandsGraph(const YAML::Node& in, IDynamicTreeStrands& target);

#pragma region ModulusGraph
void ModulusGraph::Reset() {
  node_graph = {};

  // density
  output_density_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_density_node = node_graph.RefNode(output_density_node_handle);
  output_density_node.data.type = NodeType::OutputDensity;
  output_density_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_density_node.GetInputPinHandles()[0]).data.name = "min";
  node_graph.RefInputPin(output_density_node.GetInputPinHandles()[1]).data.name = "max";

  // shear/stretch modulus
  output_shear_stretch_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_shear_stretch_node = node_graph.RefNode(output_shear_stretch_node_handle);
  output_shear_stretch_node.data.type = NodeType::OutputStretchShearModulus;
  output_shear_stretch_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_shear_stretch_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_shear_stretch_node.GetInputPinHandles()[1]).data.name = "upper bound";

  // bending modulus
  output_bending_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_bending_node = node_graph.RefNode(output_bending_node_handle);
  output_bending_node.data.type = NodeType::OutputBendingModulus;
  output_bending_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_bending_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_bending_node.GetInputPinHandles()[1]).data.name = "upper bound";

  // twisting modulus
  output_twisting_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_twisting_node = node_graph.RefNode(output_twisting_node_handle);
  output_twisting_node.data.type = NodeType::OutputTwistingModulus;
  output_twisting_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_twisting_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_twisting_node.GetInputPinHandles()[1]).data.name = "upper bound";

  // input segment node
  input_node_handle = node_graph.AllocateNode(0, 4);
  auto& input_node = node_graph.RefNode(input_node_handle);
  input_node.data.type = NodeType::InputSegment;
  input_node.data.node_impl = std::make_shared<InputNode>();
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[0]).data.name = "root distance";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[1]).data.name = "polar distance";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[2]).data.name = "polar angle";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[3]).data.name = "profile boundary distance";

  node_graph.SetNodePosition(output_density_node_handle, glm::vec2(500, 200));
  node_graph.SetNodePosition(output_shear_stretch_node_handle, glm::vec2(500, 400));
  node_graph.SetNodePosition(output_bending_node_handle, glm::vec2(500, 600));
  node_graph.SetNodePosition(output_twisting_node_handle, glm::vec2(500, 800));
  node_graph.SetNodePosition(input_node_handle, glm::vec2(100, 300));
}

ModulusGraph::ModulusGraph() {
  Reset();
}

ModulusGraph::ModulusGraph(const Output& output) {
  Reset();
  SetValues(output);
}

ModulusGraph::Output::DensityType ModulusGraph::GetDensity(const ModulusGraph::Input& input) const {
  return GetOutput<Output::DensityType>(output_density_node_handle, input);
}

void ModulusGraph::SetDensity(const ModulusGraph::Output::DensityType& value) {
  SetOutput(output_density_node_handle, value);
}

ModulusGraph::Output::ShearStretchModulusType ModulusGraph::GetShearStretchModulus(
    const ModulusGraph::Input& input) const {
  return GetOutput<Output::ShearStretchModulusType>(output_shear_stretch_node_handle, input);
}

void ModulusGraph::SetShearStretchModulus(const ModulusGraph::Output::ShearStretchModulusType& value) {
  SetOutput(output_shear_stretch_node_handle, value);
}

ModulusGraph::Output::BendingModulusType ModulusGraph::GetBendingModulus(const ModulusGraph::Input& input) const {
  return GetOutput<Output::BendingModulusType>(output_bending_node_handle, input);
}

void ModulusGraph::SetBendingModulus(const ModulusGraph::Output::BendingModulusType& value) {
  SetOutput(output_bending_node_handle, value);
}

ModulusGraph::Output::TwistingModulusType ModulusGraph::GetTwistingModulus(const ModulusGraph::Input& input) const {
  return GetOutput<Output::TwistingModulusType>(output_twisting_node_handle, input);
}

void ModulusGraph::SetTwistingModulus(const ModulusGraph::Output::TwistingModulusType& value) {
  SetOutput(output_twisting_node_handle, value);
}

ModulusGraph::Output ModulusGraph::GetValues(const Input& input) const {
  Output output{};
  output.density = GetDensity(input);
  output.shear_stretch_modulus = GetShearStretchModulus(input);
  output.bending_modulus = GetBendingModulus(input);
  output.twisting_modulus = GetTwistingModulus(input);
  return output;
}

void ModulusGraph::SetValues(const ModulusGraph::Output& values) {
  SetDensity(values.density);
  SetShearStretchModulus(values.shear_stretch_modulus);
  SetBendingModulus(values.bending_modulus);
  SetTwistingModulus(values.twisting_modulus);
}

void ModulusGraph::OnCreate() {
  Reset();
}

void eco_sys_lab_package::SerializeModulusGraph(YAML::Emitter& out, const ModulusGraph& target) {
  SerializeDynamicTreeStrandsGraph(out, target);
}

void eco_sys_lab_package::DeserializeModulusGraph(const YAML::Node& in, ModulusGraph& target) {
  DeserializeDynamicTreeStrandsGraph(in, target);
}

void eco_sys_lab_package::ModulusGraph::SetOutput(NodeGraphNodeHandle output_node_handle, const glm::vec2& value) {
  // Create two new constant nodes to input the values into the output node
  const auto min_node_handle = node_graph.AllocateNode(0, 1);
  {
    auto& min_node = node_graph.RefNode(min_node_handle);
    min_node.data.type = NodeType::Constant;
    const auto min_node_impl = std::make_shared<ConstantNode>();
    min_node_impl->value = value.x;
    min_node.data.node_impl = min_node_impl;
  }

  const auto max_node_handle = node_graph.AllocateNode(0, 1);
  {
    auto& max_node = node_graph.RefNode(max_node_handle);
    max_node.data.type = NodeType::Constant;
    const auto max_node_impl = std::make_shared<ConstantNode>();
    max_node_impl->value = value.y;
    max_node.data.node_impl = max_node_impl;
  }

  // Connect the output pins of the constant nodes to the input pins of the output density node
  auto& output_node = node_graph.RefNode(output_node_handle);

  // Recycle all existing links
  for (const auto& input_pin_handle : output_node.GetInputPinHandles()) {
    const auto& input_pin = node_graph.PeekInputPin(input_pin_handle);
    const auto link_handle = input_pin.GetLinkHandle();
    if (link_handle != -1) {
      node_graph.RecycleLink(link_handle);
    }
  }

  auto& min_node = node_graph.RefNode(min_node_handle);
  node_graph.AllocateLink(min_node.GetOutputPinHandles()[0], output_node.GetInputPinHandles()[0]);

  auto& max_node = node_graph.RefNode(max_node_handle);
  node_graph.AllocateLink(max_node.GetOutputPinHandles()[0], output_node.GetInputPinHandles()[1]);

  // Set node positions
  glm::vec2 pos = node_graph.GetNodePosition(output_node.GetHandle());
  node_graph.SetNodePosition(min_node_handle, glm::vec2(pos.x - 200, pos.y - 50));
  node_graph.SetNodePosition(max_node_handle, glm::vec2(pos.x - 200, pos.y + 50));
}
#pragma endregion ModulusGraph

#pragma region StrengthGraph
void StrengthGraph::Reset() {
  node_graph = {};
  // shear/stretch strength
  output_shear_stretch_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_shear_stretch_node = node_graph.RefNode(output_shear_stretch_node_handle);
  output_shear_stretch_node.data.type = NodeType::OutputSheerStretchStrength;
  output_shear_stretch_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_shear_stretch_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_shear_stretch_node.GetInputPinHandles()[1]).data.name = "upper bound";

  // bending strength
  output_bending_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_bending_node = node_graph.RefNode(output_bending_node_handle);
  output_bending_node.data.type = NodeType::OutputBendingStrength;
  output_bending_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_bending_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_bending_node.GetInputPinHandles()[1]).data.name = "upper bound";

  // twisting strength
  output_twisting_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_twisting_node = node_graph.RefNode(output_twisting_node_handle);
  output_twisting_node.data.type = NodeType::OutputTwistingStrength;
  output_twisting_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_twisting_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_twisting_node.GetInputPinHandles()[1]).data.name = "upper bound";

  // bundle strength
  output_bundle_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_bundle_node = node_graph.RefNode(output_bundle_node_handle);
  output_bundle_node.data.type = NodeType::OutputBundleStrength;
  output_bundle_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_bundle_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_bundle_node.GetInputPinHandles()[1]).data.name = "upper bound";

  // connectivity strength
  output_connectivity_node_handle = node_graph.AllocateNode(2, 0);
  auto& output_connectivity_node = node_graph.RefNode(output_connectivity_node_handle);
  output_connectivity_node.data.type = NodeType::OutputConnectivityStrength;
  output_connectivity_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_connectivity_node.GetInputPinHandles()[0]).data.name = "lower bound";
  node_graph.RefInputPin(output_connectivity_node.GetInputPinHandles()[1]).data.name = "upper bound";

  input_node_handle = node_graph.AllocateNode(0, 4);
  auto& input_node = node_graph.RefNode(input_node_handle);
  input_node.data.type = NodeType::InputSegment;
  input_node.data.node_impl = std::make_shared<InputNode>();
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[0]).data.name = "root distance";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[1]).data.name = "polar distance";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[2]).data.name = "polar angle";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[3]).data.name = "profile boundary distance";

  node_graph.SetNodePosition(output_shear_stretch_node_handle, glm::vec2(500, 200));
  node_graph.SetNodePosition(output_bending_node_handle, glm::vec2(500, 400));
  node_graph.SetNodePosition(output_twisting_node_handle, glm::vec2(500, 600));
  node_graph.SetNodePosition(output_bundle_node_handle, glm::vec2(500, 800));
  node_graph.SetNodePosition(output_connectivity_node_handle, glm::vec2(500, 1000));
  node_graph.SetNodePosition(input_node_handle, glm::vec2(100, 600));
}

StrengthGraph::StrengthGraph() {
  Reset();
}

StrengthGraph::StrengthGraph(const Output& output) {
  Reset();
  SetValues(output);
}

glm::vec2 StrengthGraph::GetShearStretchStrength(const StrengthGraph::Input& input) const {
  return GetOutput<Output::ShearStretchStrengthType>(output_shear_stretch_node_handle, input);
}

void StrengthGraph::SetShearStretchStrength(const StrengthGraph::Output::ShearStretchStrengthType& value) {
  SetOutput(output_shear_stretch_node_handle, value);
}

StrengthGraph::Output::BendingStrengthType StrengthGraph::GetBendingStrength(const StrengthGraph::Input& input) const {
  return GetOutput<Output::BendingStrengthType>(output_bending_node_handle, input);
}

void StrengthGraph::SetBendingStrength(const StrengthGraph::Output::BendingStrengthType& value) {
  SetOutput(output_bending_node_handle, value);
}

StrengthGraph::Output::TwistingStrengthType StrengthGraph::GetTwistingStrength(
    const StrengthGraph::Input& input) const {
  return GetOutput<Output::TwistingStrengthType>(output_twisting_node_handle, input);
}

void StrengthGraph::SetTwistingStrength(const StrengthGraph::Output::TwistingStrengthType& value) {
  SetOutput(output_twisting_node_handle, value);
}

StrengthGraph::Output::BendingStrengthType StrengthGraph::GetBundleStrength(const StrengthGraph::Input& input) const {
  return GetOutput<Output::BundleStrengthType>(output_bundle_node_handle, input);
}

void StrengthGraph::SetBundleStrength(const StrengthGraph::Output::BundleStrengthType& value) {
  SetOutput(output_bundle_node_handle, value);
}

StrengthGraph::Output::ConnectivityStrengthType StrengthGraph::GetConnectivityStrength(
    const StrengthGraph::Input& input) const {
  return GetOutput<Output::ConnectivityStrengthType>(output_connectivity_node_handle, input);
}

void StrengthGraph::SetConnectivityStrength(const StrengthGraph::Output::ConnectivityStrengthType& value) {
  SetOutput(output_connectivity_node_handle, value);
}

StrengthGraph::Output StrengthGraph::GetValues(const StrengthGraph::Input& input) const {
  Output output{};
  output.shear_stretch_strength = GetShearStretchStrength(input);
  output.bending_strength = GetBendingStrength(input);
  output.twisting_strength = GetTwistingStrength(input);
  output.bundle_strength = GetBundleStrength(input);
  output.connectivity_strength = GetConnectivityStrength(input);
  return output;
}

void StrengthGraph::SetValues(const StrengthGraph::Output& values) {
  SetShearStretchStrength(values.shear_stretch_strength);
  SetBendingStrength(values.bending_strength);
  SetTwistingStrength(values.twisting_strength);
  SetBundleStrength(values.bundle_strength);
  SetConnectivityStrength(values.connectivity_strength);
}

void StrengthGraph::OnCreate() {
  Reset();
}

void eco_sys_lab_package::SerializeStrengthGraph(YAML::Emitter& out, const StrengthGraph& target) {
  SerializeDynamicTreeStrandsGraph(out, target);
}

void eco_sys_lab_package::DeserializeStrengthGraph(const YAML::Node& in, StrengthGraph& target) {
  DeserializeDynamicTreeStrandsGraph(in, target);
}

void eco_sys_lab_package::StrengthGraph::SetOutput(NodeGraphNodeHandle output_node_handle, const glm::vec2& value) {
  // Create two new constant nodes to input the values into the output node
  const auto min_node_handle = node_graph.AllocateNode(0, 1);
  {
    auto& min_node = node_graph.RefNode(min_node_handle);
    min_node.data.type = NodeType::Constant;
    const auto min_node_impl = std::make_shared<ConstantNode>();
    min_node_impl->value = value.x;
    min_node.data.node_impl = min_node_impl;
  }

  const auto max_node_handle = node_graph.AllocateNode(0, 1);
  {
    auto& max_node = node_graph.RefNode(max_node_handle);
    max_node.data.type = NodeType::Constant;
    const auto max_node_impl = std::make_shared<ConstantNode>();
    max_node_impl->value = value.y;
    max_node.data.node_impl = max_node_impl;
  }

  // Connect the output pins of the constant nodes to the input pins of the output density node
  auto& output_node = node_graph.RefNode(output_node_handle);

  // Recycle all existing links
  for (const auto& input_pin_handle : output_node.GetInputPinHandles()) {
    const auto& input_pin = node_graph.PeekInputPin(input_pin_handle);
    const auto link_handle = input_pin.GetLinkHandle();
    if (link_handle != -1) {
      node_graph.RecycleLink(link_handle);
    }
  }

  auto& min_node = node_graph.RefNode(min_node_handle);
  node_graph.AllocateLink(min_node.GetOutputPinHandles()[0], output_node.GetInputPinHandles()[0]);

  auto& max_node = node_graph.RefNode(max_node_handle);
  node_graph.AllocateLink(max_node.GetOutputPinHandles()[0], output_node.GetInputPinHandles()[1]);

  // Set node positions
  glm::vec2 pos = node_graph.GetNodePosition(output_node.GetHandle());
  node_graph.SetNodePosition(min_node_handle, glm::vec2(pos.x - 200, pos.y - 50));
  node_graph.SetNodePosition(max_node_handle, glm::vec2(pos.x - 200, pos.y + 50));
}
#pragma endregion StrengthGraph

#pragma region BiologicalPropertiesGraph
void BiologicalPropertiesGraph::Reset() {
  node_graph = {};
  output_node_handle = node_graph.AllocateNode(3, 0);
  auto& output_node = node_graph.RefNode(output_node_handle);
  output_node.data.type = NodeType::OutputTrunk;
  output_node.data.node_impl = std::make_shared<OutputNode>();

  node_graph.RefInputPin(output_node.GetInputPinHandles()[0]).data.name = "trunk input";
  node_graph.RefInputPin(output_node.GetInputPinHandles()[1]).data.name = "trunk transition";
  node_graph.RefInputPin(output_node.GetInputPinHandles()[2]).data.name = "trunk additional strength factor";

  input_node_handle = node_graph.AllocateNode(0, 4);
  auto& input_segment_node = node_graph.RefNode(input_node_handle);
  input_segment_node.data.type = NodeType::InputSegment;
  input_segment_node.data.node_impl = std::make_shared<InputNode>();
  node_graph.RefOutputPin(input_segment_node.GetOutputPinHandles()[0]).data.name = "root distance";
  node_graph.RefOutputPin(input_segment_node.GetOutputPinHandles()[1]).data.name = "polar distance";
  node_graph.RefOutputPin(input_segment_node.GetOutputPinHandles()[2]).data.name = "polar angle";
  node_graph.RefOutputPin(input_segment_node.GetOutputPinHandles()[3]).data.name = "profile boundary distance";

  node_graph.SetNodePosition(output_node_handle, glm::vec2(400, 250));
  node_graph.SetNodePosition(input_node_handle, glm::vec2(100, 250));
}

BiologicalPropertiesGraph::BiologicalPropertiesGraph() {
  Reset();
}

BiologicalPropertiesGraph::BiologicalPropertiesGraph(const Output& output) {
  Reset();
  SetValues(output);
}

BiologicalPropertiesGraph::Output BiologicalPropertiesGraph::GetValues(
    const BiologicalPropertiesGraph::Input& input) const {
  const auto& input_node = node_graph.PeekNode(input_node_handle);
  std::unordered_map<NodeGraphOutputPinHandle, float> results{};
  const auto& output_pin_handles = input_node.GetOutputPinHandles();
  results[output_pin_handles[0]] = input.root_distance;
  results[output_pin_handles[1]] = input.polar_distance;
  results[output_pin_handles[2]] = input.polar_angle;
  results[output_pin_handles[3]] = input.profile_boundary_distance;

  BiologicalPropertiesGraph::Output value_result;

  // Find the output node
  const auto& output_node = node_graph.PeekNode(output_node_handle);
  output_node.data.node_impl->Process(node_graph, 0, results);

  // Process the input pins of the output node
  const auto& input_pin_handles = output_node.GetInputPinHandles();
  if (input_pin_handles.size() >= 3) {
    // trunk offset
    {
      const auto& input_pin = node_graph.PeekInputPin(input_pin_handles[0]);
      const auto link_handle = input_pin.GetLinkHandle();
      if (link_handle != -1) {
        const auto& link = node_graph.PeekLink(link_handle);
        if (const auto search = results.find(link.GetOutputPinHandle()); search != results.end()) {
          value_result.trunk_offset = search->second;
        }
      }
    }

    // trunk transition
    {
      const auto& input_pin = node_graph.PeekInputPin(input_pin_handles[1]);
      const auto link_handle = input_pin.GetLinkHandle();
      if (link_handle != -1) {
        const auto& link = node_graph.PeekLink(link_handle);
        if (const auto search = results.find(link.GetOutputPinHandle()); search != results.end()) {
          value_result.trunk_transition = search->second;
        }
      }
    }

    // trunk additional strength factor
    {
      const auto& input_pin = node_graph.PeekInputPin(input_pin_handles[2]);
      const auto link_handle = input_pin.GetLinkHandle();
      if (link_handle != -1) {
        const auto& link = node_graph.PeekLink(link_handle);
        if (const auto search = results.find(link.GetOutputPinHandle()); search != results.end()) {
          value_result.trunk_additional_strength_factor = search->second;
        }
      }
    }
  }

  return value_result;
}

void BiologicalPropertiesGraph::SetValues(const BiologicalPropertiesGraph::Output& value) {
  // create three new constant nodes to input the values into the output node

  // offset
  const auto offset_node_handle = node_graph.AllocateNode(0, 1);
  {
    auto& offset_node = node_graph.RefNode(offset_node_handle);
    offset_node.data.type = NodeType::Constant;
    const auto offset_node_impl = std::make_shared<ConstantNode>();
    offset_node_impl->value = value.trunk_offset;
    offset_node.data.node_impl = offset_node_impl;
  }
  // transition
  const auto transition_node_handle = node_graph.AllocateNode(0, 1);
  {
    auto& transition_node = node_graph.RefNode(transition_node_handle);
    transition_node.data.type = NodeType::Constant;
    const auto transition_node_impl = std::make_shared<ConstantNode>();
    transition_node_impl->value = value.trunk_transition;
    transition_node.data.node_impl = transition_node_impl;
  }

  // additional strength factor
  const auto additional_strength_node_handle = node_graph.AllocateNode(0, 1);
  {
    auto& additional_strength_node = node_graph.RefNode(additional_strength_node_handle);
    additional_strength_node.data.type = NodeType::Constant;
    const auto additional_strength_node_impl = std::make_shared<ConstantNode>();
    additional_strength_node_impl->value = value.trunk_additional_strength_factor;
    additional_strength_node.data.node_impl = additional_strength_node_impl;
  }

  // connect the output pins of the constant nodes to the input pin of the output node
  auto& output_node = node_graph.RefNode(output_node_handle);

  // recycle all existing links
  for (const auto& input_pin_handle : output_node.GetInputPinHandles()) {
    const auto& input_pin = node_graph.PeekInputPin(input_pin_handle);
    const auto link_handle = input_pin.GetLinkHandle();
    if (link_handle != -1) {
      node_graph.RecycleLink(link_handle);
    }
  }

  auto& offset_node = node_graph.RefNode(offset_node_handle);
  node_graph.AllocateLink(offset_node.GetOutputPinHandles()[0], output_node.GetInputPinHandles()[0]);

  auto& transition_node = node_graph.RefNode(transition_node_handle);
  node_graph.AllocateLink(transition_node.GetOutputPinHandles()[0], output_node.GetInputPinHandles()[1]);

  auto& additional_strength_node = node_graph.RefNode(additional_strength_node_handle);
  node_graph.AllocateLink(additional_strength_node.GetOutputPinHandles()[0], output_node.GetInputPinHandles()[2]);

  glm::vec2 pos = node_graph.GetNodePosition(output_node.GetHandle());
  node_graph.SetNodePosition(offset_node_handle, glm::vec2(pos.x - 200, pos.y - 100));
  node_graph.SetNodePosition(transition_node_handle, glm::vec2(pos.x - 200, pos.y));
  node_graph.SetNodePosition(additional_strength_node_handle, glm::vec2(pos.x - 200, pos.y + 100));
}

void BiologicalPropertiesGraph::OnCreate() {
  Reset();
}

void eco_sys_lab_package::SerializeBiologicalPropertiesGraph(YAML::Emitter& out,
                                                             const BiologicalPropertiesGraph& target) {
  SerializeDynamicTreeStrandsGraph(out, target);
}

void eco_sys_lab_package::DeserializeBiologicalPropertiesGraph(const YAML::Node& in,
                                                               BiologicalPropertiesGraph& target) {
  DeserializeDynamicTreeStrandsGraph(in, target);
}

#pragma endregion BiologicalPropertiesGraph

static void SerializeDynamicTreeStrandsGraph(YAML::Emitter& out, const IDynamicTreeStrands& target) {
  target.node_graph.Save(
      "node_graph", out,
      [&](YAML::Emitter& input_pin_out, const InputPinData& data) {
        input_pin_out << YAML::Key << "N" << YAML::Value << data.name;
      },
      [&](YAML::Emitter& output_pin_out, const OutputPinData& data) {
        output_pin_out << YAML::Key << "N" << YAML::Value << data.name;
      },
      [&](YAML::Emitter& node_out, const NodeData& data) {
        node_out << YAML::Key << "T" << static_cast<unsigned>(data.type);
        node_out << YAML::Key << "C" << YAML::BeginMap;
        if (data.type == NodeType::Constant) {
          if (const auto constant_node = std::dynamic_pointer_cast<ConstantNode>(data.node_impl)) {
            node_out << YAML::Key << "value" << YAML::Value << constant_node->value;
          }
        }
        node_out << YAML::EndMap;
      },
      [&](YAML::Emitter& link_out, const int& data) {
      });
}

static void DeserializeDynamicTreeStrandsGraph(const YAML::Node& in, IDynamicTreeStrands& target) {
  target.node_graph.Load(
      "node_graph", in,
      [&](const YAML::Node& input_pin_in, InputPinData& data) {
        if (input_pin_in["N"]) {
          data.name = input_pin_in["N"].as<std::string>();
        }
      },
      [&](const YAML::Node& output_pin_in, OutputPinData& data) {
        if (output_pin_in["N"]) {
          data.name = output_pin_in["N"].as<std::string>();
        }
      },
      [&](const YAML::Node& node_in, NodeData& data) {
        if (node_in["T"]) {
          data.type = static_cast<NodeType>(node_in["T"].as<unsigned>());
        }
        data.node_impl = CreateStrandGraphNode(data.type);

        if (node_in["C"]) {
          if (data.type == NodeType::Constant) {
            if (const auto constant_node = std::dynamic_pointer_cast<ConstantNode>(data.node_impl);
                constant_node && node_in["C"]["value"]) {
              constant_node->value = node_in["C"]["value"].as<float>();
            }
          }
        }
      },
      [&](const YAML::Node& link_in, int& data) {
      });
}

void INode::PrepareInputs(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                          const NodeGraphNodeHandle node_handle,
                          std::unordered_map<NodeGraphOutputPinHandle, float>& results) {
  const auto& node = graph.PeekNode(node_handle);
  for (const auto& input_pin_handle : node.GetInputPinHandles()) {
    const auto& input_pin = graph.PeekInputPin(input_pin_handle);
    const auto link_handle = input_pin.GetLinkHandle();
    if (link_handle != -1) {
      const auto& link = graph.PeekLink(link_handle);
      if (results.find(link.GetOutputPinHandle()) == results.end()) {
        const auto& output_pin = graph.PeekOutputPin(link.GetOutputPinHandle());
        graph.PeekNode(output_pin.GetNodeHandle()).data.node_impl->Process(graph, output_pin.GetNodeHandle(), results);
      }
    }
  }
}

void InputNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                        const NodeGraphNodeHandle node_handle,
                        std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
}

void OutputNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                         const NodeGraphNodeHandle node_handle,
                         std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
  PrepareInputs(graph, node_handle, results);
}

std::shared_ptr<INode> eco_sys_lab_package::CreateStrandGraphNode(const NodeType type) {
  switch (type) {
    case NodeType::Unknown:
      break;
    case NodeType::InputSegment:
      return std::make_shared<InputNode>();
    case NodeType::InputParticle:
      return std::make_shared<InputNode>();
    case NodeType::OutputDensity:
      return std::make_shared<OutputNode>();
    case NodeType::OutputStretchShearModulus:
      return std::make_shared<OutputNode>();
    case NodeType::OutputBendingModulus:
      return std::make_shared<OutputNode>();
    case NodeType::OutputTwistingModulus:
      return std::make_shared<OutputNode>();
    case NodeType::OutputSheerStretchStrength:
      return std::make_shared<OutputNode>();
    case NodeType::OutputBendingStrength:
      return std::make_shared<OutputNode>();
    case NodeType::OutputTwistingStrength:
      return std::make_shared<OutputNode>();
    case NodeType::OutputBundleStrength:
      return std::make_shared<OutputNode>();
    case NodeType::OutputConnectivityStrength:
      return std::make_shared<OutputNode>();
    case NodeType::OutputTrunk:
      return std::make_shared<OutputNode>();
    case NodeType::Constant:
      return std::make_shared<ConstantNode>();
    case NodeType::Add:
      return std::make_shared<AddNode>();
    case NodeType::Subtract:
      return std::make_shared<SubtractNode>();
    case NodeType::Multiply:
      return std::make_shared<MultiplyNode>();
    case NodeType::Divide:
      return std::make_shared<DivideNode>();
    case NodeType::Power:
      return std::make_shared<PowerNode>();
    case NodeType::Min:
      return std::make_shared<MinNode>();
    case NodeType::Max:
      return std::make_shared<MaxNode>();
    case NodeType::Abs:
      return std::make_shared<AbsNode>();
    case NodeType::Clamp:
      return std::make_shared<ClampNode>();
    case NodeType::Negate:
      return std::make_shared<NegateNode>();
    case NodeType::Exponent:
      return std::make_shared<ExponentNode>();
    case NodeType::FlipUp:
      return std::make_shared<FlipUpNode>();
    case NodeType::FlipDown:
      return std::make_shared<FlipDownNode>();

    case NodeType::Sigmoid:
      return std::make_shared<SigmoidNode>();
    case NodeType::SoftSign:
      return std::make_shared<SoftSignNode>();
    case NodeType::Tanh:
      return std::make_shared<TanhNode>();
  }
  return {};
}
