#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrandGenerators.hpp"
#include "DynamicTreeStrandOperators.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_plugin;

bool NodeData::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) const {
  return node_impl->OnInspect(editor_layer);
}
bool InputPinData::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) const {
  bool changed = false;
  ImGui::Text(name.c_str());
  return changed;
}
bool OutputPinData::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) const {
  bool changed = false;
  ImGui::Text(name.c_str());
  return changed;
}

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

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(&node_graph.RefImNodesEditorContext());
    ImNodes::SetNodeScreenSpacePos(output_density_node_handle, ImVec2(500, 200));
    ImNodes::SetNodeScreenSpacePos(output_shear_stretch_node_handle, ImVec2(500, 400));
    ImNodes::SetNodeScreenSpacePos(output_bending_node_handle, ImVec2(500, 600));
    ImNodes::SetNodeScreenSpacePos(output_twisting_node_handle, ImVec2(500, 800));

    ImNodes::SetNodeScreenSpacePos(input_node_handle, ImVec2(100, 300));
    ImNodes::EditorContextSet(prev_editor_context);
  }
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

void ModulusGraph::Serialize(YAML::Emitter& out) const {
  SerializeImpl(out);
}

void ModulusGraph::Deserialize(const YAML::Node& in) {
  DeserializeImpl(in);
}

bool ModulusGraph::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  // Test output nodes for all their pins
  static ModulusGraph::Input temp_input{};
  static ModulusGraph::Output::DensityType temp_output_density = glm::vec2(0.0f);
  static ModulusGraph::Output::ShearStretchModulusType temp_output_shear_stretch = glm::vec2(0.0f);
  static ModulusGraph::Output::BendingModulusType temp_output_bending = glm::vec2(0.0f);
  static ModulusGraph::Output::TwistingModulusType temp_output_twisting = glm::vec2(0.0f);

  ImGui::Text("Test Inputs:");
  if (ImGui::DragFloat("Root Distance", &temp_input.root_distance)) {
    temp_output_density = GetDensity(temp_input);
    temp_output_shear_stretch = GetShearStretchModulus(temp_input);
    temp_output_bending = GetBendingModulus(temp_input);
    temp_output_twisting = GetTwistingModulus(temp_input);
  }
  if (ImGui::DragFloat("Polar Distance", &temp_input.polar_distance)) {
    temp_output_density = GetDensity(temp_input);
    temp_output_shear_stretch = GetShearStretchModulus(temp_input);
    temp_output_bending = GetBendingModulus(temp_input);
    temp_output_twisting = GetTwistingModulus(temp_input);
  }
  if (ImGui::DragFloat("Polar Angle", &temp_input.polar_angle)) {
    temp_output_density = GetDensity(temp_input);
    temp_output_shear_stretch = GetShearStretchModulus(temp_input);
    temp_output_bending = GetBendingModulus(temp_input);
    temp_output_twisting = GetTwistingModulus(temp_input);
  }
  if (ImGui::DragFloat("Profile Boundary Distance", &temp_input.profile_boundary_distance)) {
    temp_output_density = GetDensity(temp_input);
    temp_output_shear_stretch = GetShearStretchModulus(temp_input);
    temp_output_bending = GetBendingModulus(temp_input);
    temp_output_twisting = GetTwistingModulus(temp_input);
  }
  if (ImGui::Button("Calculate")) {
    temp_output_density = GetDensity(temp_input);
    temp_output_shear_stretch = GetShearStretchModulus(temp_input);
    temp_output_bending = GetBendingModulus(temp_input);
    temp_output_twisting = GetTwistingModulus(temp_input);
  }

  ImGui::Text("Density min: %.3f", temp_output_density[0]);
  ImGui::Text("Density max: %.3f", temp_output_density[1]);
  ImGui::Text("Shear Stretch lower bound: %.3f", temp_output_shear_stretch[0]);
  ImGui::Text("Shear Stretch upper bound: %.3f", temp_output_shear_stretch[1]);
  ImGui::Text("Bending lower bound: %.3f", temp_output_bending[0]);
  ImGui::Text("Bending upper bound: %.3f", temp_output_bending[1]);
  ImGui::Text("Twisting lower bound: %.3f", temp_output_twisting[0]);
  ImGui::Text("Twisting upper bound: %.3f", temp_output_twisting[1]);

  // Test setting values
  if (ImGui::Button("Set Density")) {
    glm::vec2 values;
    SetDensity(values);
  }
  if (ImGui::Button("Set Shear Stretch Modulus")) {
    glm::vec2 values;
    SetShearStretchModulus(values);
  }
  if (ImGui::Button("Set Bending Modulus")) {
    glm::vec2 values;
    SetBendingModulus(values);
  }
  if (ImGui::Button("Set Twisting Modulus")) {
    glm::vec2 values;
    SetTwistingModulus(values);
  }

  static bool show_node_graph = true;
  ImGui::Checkbox("Show modulus node graph", &show_node_graph);
  if (show_node_graph) {
    changed = ShowGraph("Dynamic Tree Strands", editor_layer) || changed;
  }
  return changed;
}
void eco_sys_lab_plugin::ModulusGraph::SetOutput(NodeGraphNodeHandle output_node_handle, const glm::vec2& value) {
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
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  ImNodesEditorContext* prev_editor_context = nullptr;
  if (editor_layer) {
    prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(const_cast<ImNodesEditorContext*>(&node_graph.RefImNodesEditorContext()));
  }

  ImVec2 pos = ImNodes::GetNodeScreenSpacePos(output_node.GetHandle());
  ImNodes::SetNodeScreenSpacePos(min_node_handle, ImVec2(pos.x - 200, pos.y - 50));
  ImNodes::SetNodeScreenSpacePos(max_node_handle, ImVec2(pos.x - 200, pos.y + 50));

  if (editor_layer) {
    ImNodes::EditorContextSet(prev_editor_context);
  }
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

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(&node_graph.RefImNodesEditorContext());
    ImNodes::SetNodeScreenSpacePos(output_shear_stretch_node_handle, ImVec2(500, 200));
    ImNodes::SetNodeScreenSpacePos(output_bending_node_handle, ImVec2(500, 400));
    ImNodes::SetNodeScreenSpacePos(output_twisting_node_handle, ImVec2(500, 600));
    ImNodes::SetNodeScreenSpacePos(output_bundle_node_handle, ImVec2(500, 800));
    ImNodes::SetNodeScreenSpacePos(output_connectivity_node_handle, ImVec2(500, 1000));
    ImNodes::SetNodeScreenSpacePos(input_node_handle, ImVec2(100, 600));
    ImNodes::EditorContextSet(prev_editor_context);
  }
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

void StrengthGraph::Serialize(YAML::Emitter& out) const {
  SerializeImpl(out);
}

void StrengthGraph::Deserialize(const YAML::Node& in) {
  DeserializeImpl(in);
}

bool StrengthGraph::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  // Test output nodes for all their pins
  static StrengthGraph::Input temp_input{};
  static StrengthGraph::Output::ShearStretchStrengthType temp_output_shear_stretch = glm::vec2(0.0f);
  static StrengthGraph::Output::BendingStrengthType temp_output_bending = glm::vec2(0.0f);
  static StrengthGraph::Output::TwistingStrengthType temp_output_twisting = glm::vec2(0.0f);
  static StrengthGraph::Output::BundleStrengthType temp_output_bundle = glm::vec2(0.0f);
  static StrengthGraph::Output::ConnectivityStrengthType temp_output_connectivity = glm::vec2(0.0f);

  ImGui::Text("Test Inputs:");
  if (ImGui::DragFloat("Root Distance", &temp_input.root_distance)) {
    temp_output_shear_stretch = GetShearStretchStrength(temp_input);
    temp_output_bending = GetBendingStrength(temp_input);
    temp_output_twisting = GetTwistingStrength(temp_input);
    temp_output_bundle = GetBundleStrength(temp_input);
    temp_output_connectivity = GetConnectivityStrength(temp_input);
  }
  if (ImGui::DragFloat("Polar Distance", &temp_input.polar_distance)) {
    temp_output_shear_stretch = GetShearStretchStrength(temp_input);
    temp_output_bending = GetBendingStrength(temp_input);
    temp_output_twisting = GetTwistingStrength(temp_input);
    temp_output_bundle = GetBundleStrength(temp_input);
    temp_output_connectivity = GetConnectivityStrength(temp_input);
  }
  if (ImGui::DragFloat("Polar Angle", &temp_input.polar_angle)) {
    temp_output_shear_stretch = GetShearStretchStrength(temp_input);
    temp_output_bending = GetBendingStrength(temp_input);
    temp_output_twisting = GetTwistingStrength(temp_input);
    temp_output_bundle = GetBundleStrength(temp_input);
    temp_output_connectivity = GetConnectivityStrength(temp_input);
  }
  if (ImGui::DragFloat("Profile Boundary Distance", &temp_input.profile_boundary_distance)) {
    temp_output_shear_stretch = GetShearStretchStrength(temp_input);
    temp_output_bending = GetBendingStrength(temp_input);
    temp_output_twisting = GetTwistingStrength(temp_input);
    temp_output_bundle = GetBundleStrength(temp_input);
    temp_output_connectivity = GetConnectivityStrength(temp_input);
  }
  if (ImGui::Button("Calculate")) {
    temp_output_shear_stretch = GetShearStretchStrength(temp_input);
    temp_output_bending = GetBendingStrength(temp_input);
    temp_output_twisting = GetTwistingStrength(temp_input);
    temp_output_bundle = GetBundleStrength(temp_input);
    temp_output_connectivity = GetConnectivityStrength(temp_input);
  }

  ImGui::Text("Shear Stretch lower bound: %.3f", temp_output_shear_stretch[0]);
  ImGui::Text("Shear Stretch upper bound: %.3f", temp_output_shear_stretch[1]);
  ImGui::Text("Bending lower bound: %.3f", temp_output_bending[0]);
  ImGui::Text("Bending upper bound: %.3f", temp_output_bending[1]);
  ImGui::Text("Twisting lower bound: %.3f", temp_output_twisting[0]);
  ImGui::Text("Twisting upper bound: %.3f", temp_output_twisting[1]);
  ImGui::Text("Bundle lower bound: %.3f", temp_output_bundle[0]);
  ImGui::Text("Bundle upper bound: %.3f", temp_output_bundle[1]);
  ImGui::Text("Connectivity lower bound: %.3f", temp_output_connectivity[0]);
  ImGui::Text("Connectivity upper bound: %.3f", temp_output_connectivity[1]);

  // Test setting values
  if (ImGui::Button("Set Shear Stretch Strength")) {
    glm::vec2 values;
    SetShearStretchStrength(values);
  }
  if (ImGui::Button("Set Bending Strength")) {
    glm::vec2 values;
    SetBendingStrength(values);
  }
  if (ImGui::Button("Set Twisting Strength")) {
    glm::vec2 values;
    SetTwistingStrength(values);
  }
  if (ImGui::Button("Set Bundle Strength")) {
    glm::vec2 values;
    SetBundleStrength(values);
  }
  if (ImGui::Button("Set Connectivity Strength")) {
    glm::vec2 values;
    SetConnectivityStrength(values);
  }

  static bool show_node_graph = true;
  ImGui::Checkbox("Show strength node graph", &show_node_graph);
  if (show_node_graph) {
    changed = ShowGraph("Dynamic Tree Strands", editor_layer) || changed;
  }

  return changed;
}

void eco_sys_lab_plugin::StrengthGraph::SetOutput(NodeGraphNodeHandle output_node_handle, const glm::vec2& value) {
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
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  ImNodesEditorContext* prev_editor_context = nullptr;
  if (editor_layer) {
    prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(const_cast<ImNodesEditorContext*>(&node_graph.RefImNodesEditorContext()));
  }

  ImVec2 pos = ImNodes::GetNodeScreenSpacePos(output_node.GetHandle());
  ImNodes::SetNodeScreenSpacePos(min_node_handle, ImVec2(pos.x - 200, pos.y - 50));
  ImNodes::SetNodeScreenSpacePos(max_node_handle, ImVec2(pos.x - 200, pos.y + 50));

  if (editor_layer) {
    ImNodes::EditorContextSet(prev_editor_context);
  }
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

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(&node_graph.RefImNodesEditorContext());
    ImNodes::SetNodeScreenSpacePos(output_node_handle, ImVec2(400, 250));
    ImNodes::SetNodeScreenSpacePos(input_node_handle, ImVec2(100, 250));
    ImNodes::EditorContextSet(prev_editor_context);
  }
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

  const auto editor_layer = Application::GetLayer<EditorLayer>();
  ImNodesEditorContext* prev_editor_context = nullptr;
  if (editor_layer) {
    prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(const_cast<ImNodesEditorContext*>(&node_graph.RefImNodesEditorContext()));
  }

  ImVec2 pos = ImNodes::GetNodeScreenSpacePos(output_node.GetHandle());
  ImNodes::SetNodeScreenSpacePos(offset_node_handle, ImVec2(pos.x - 200, pos.y - 100));
  ImNodes::SetNodeScreenSpacePos(transition_node_handle, ImVec2(pos.x - 200, pos.y));
  ImNodes::SetNodeScreenSpacePos(additional_strength_node_handle, ImVec2(pos.x - 200, pos.y + 100));

  if (editor_layer) {
    ImNodes::EditorContextSet(prev_editor_context);
  }
}

void BiologicalPropertiesGraph::OnCreate() {
  Reset();
}

void BiologicalPropertiesGraph::Serialize(YAML::Emitter& out) const {
  SerializeImpl(out);
}

void BiologicalPropertiesGraph::Deserialize(const YAML::Node& in) {
  DeserializeImpl(in);
}

bool BiologicalPropertiesGraph::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  // Test output node for all three pins
  static BiologicalPropertiesGraph::Input temp_input{};
  static BiologicalPropertiesGraph::Output temp_output;
  ImGui::Text("Test Inputs:");
  if (ImGui::DragFloat("Root Distance", &temp_input.root_distance)) {
    temp_output = GetValues(temp_input);
  }
  if (ImGui::DragFloat("Polar Distance", &temp_input.polar_distance)) {
    temp_output = GetValues(temp_input);
  }
  if (ImGui::DragFloat("Polar Angle", &temp_input.polar_angle)) {
    temp_output = GetValues(temp_input);
  }
  if (ImGui::DragFloat("Profile Boundary Distance", &temp_input.profile_boundary_distance)) {
    temp_output = GetValues(temp_input);
  }
  if (ImGui::Button("Calculate")) {
    temp_output = GetValues(temp_input);
  }
  ImGui::Text("input: %.3f", temp_output.trunk_offset);
  ImGui::Text("transition: %.3f", temp_output.trunk_transition);
  ImGui::Text("additional strength factor: %.3f", temp_output.trunk_additional_strength_factor);

  // Test setting values
  if (ImGui::Button("Set Values")) {
    BiologicalPropertiesGraph::Output values;
    SetValues(values);
  }

  static bool show_node_graph = true;
  ImGui::Checkbox("Show biological properties node graph", &show_node_graph);
  if (show_node_graph) {
    changed = ShowGraph("Dynamic Tree Strands", editor_layer) || changed;
  }

  return changed;
}
#pragma endregion BiologicalPropertiesGraph

bool IDynamicTreeStrands::ShowGraph(const std::string& window_title, const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Begin(window_title.c_str())) {
    const auto id = ImGui::GetID(window_title.c_str());

    static NodeGraphNodeHandle hovered_node_handle = -1;
    static NodeGraphLinkHandle hovered_link_handle = -1;

    node_graph.OnInspect(
        id, editor_layer,
        [&](const NodeGraphNodeHandle node_handle) {
          const auto& node = node_graph.PeekNode(node_handle);
          switch (node.data.type) {
            case NodeType::Unknown:
              ImGui::Text("Unknown");
              break;
            case NodeType::InputSegment:
              ImGui::Text("Segment");
              break;
            case NodeType::InputParticle:
              ImGui::Text("Particle");
              break;
            case NodeType::OutputDensity:
              ImGui::Text("Density");
              break;
            case NodeType::OutputStretchShearModulus:
              ImGui::Text("Stretch Shear Modulus");
              break;
            case NodeType::OutputBendingModulus:
              ImGui::Text("Bending Modulus");
              break;
            case NodeType::OutputTwistingModulus:
              ImGui::Text("Twisting Modulus");
              break;
            case NodeType::OutputSheerStretchStrength:
              ImGui::Text("Sheer Stretch Strength");
              break;
            case NodeType::OutputBendingStrength:
              ImGui::Text("Bending Strength");
              break;
            case NodeType::OutputTwistingStrength:
              ImGui::Text("Twisting Strength");
              break;
            case NodeType::OutputBundleStrength:
              ImGui::Text("Bundle Strength");
              break;
            case NodeType::OutputConnectivityStrength:
              ImGui::Text("Connectivity Strength");
              break;
            case NodeType::OutputTrunk:
              ImGui::Text("Trunk");
              break;
            case NodeType::Constant:
              ImGui::Text("Constant");
              break;
            case NodeType::Add:
              ImGui::Text("Add");
              break;
            case NodeType::Subtract:
              ImGui::Text("Subtract");
              break;
            case NodeType::Multiply:
              ImGui::Text("Multiply");
              break;
            case NodeType::Divide:
              ImGui::Text("Divide");
              break;
            case NodeType::Power:
              ImGui::Text("Pow");
              break;
            case NodeType::Min:
              ImGui::Text("Min");
              break;
            case NodeType::Max:
              ImGui::Text("Max");
              break;

            case NodeType::Abs:
              ImGui::Text("Abs");
              break;
            case NodeType::Clamp:
              ImGui::Text("Clamp");
              break;
            case NodeType::Exponent:
              ImGui::Text("Exponent");
              break;
            case NodeType::Negate:
              ImGui::Text("Negate");
              break;

            case NodeType::FlipUp:
              ImGui::Text("FlipUp");
              break;
            case NodeType::FlipDown:
              ImGui::Text("FlipDown");
              break;

            case NodeType::Sigmoid:
              ImGui::Text("Sigmoid");
              break;
            case NodeType::SoftSign:
              ImGui::Text("SoftSign");
              break;
            case NodeType::Tanh:
              ImGui::Text("Tanh");
              break;
          }
        },
        [&](const NodeGraphNodeHandle node_handle) {
          if (node_graph.RefNode(node_handle).data.OnInspect(editor_layer)) {
            changed = true;
          }
        },
        [&](const NodeGraphInputPinHandle input_pin_handle) {
          if (node_graph.RefInputPin(input_pin_handle).data.OnInspect(editor_layer)) {
            changed = true;
          }
        },
        [&](const NodeGraphOutputPinHandle output_pin_handle) {
          if (node_graph.RefOutputPin(output_pin_handle).data.OnInspect(editor_layer)) {
            changed = true;
          }
        },
        [&](const NodeGraphNodeHandle node_handle, const NodeGraphLinkHandle link_handle,
            const NodeGraphInputPinHandle input_pin_handle, const NodeGraphOutputPinHandle output_pin_handle) {
          hovered_node_handle = node_handle;
          hovered_link_handle = link_handle;
        },
        [&](const std::vector<NodeGraphNodeHandle>& selected_node_handles,
            const std::vector<NodeGraphLinkHandle>& selected_link_handles) {

        },
        [&](const ImVec2 click_pos) {
          if (hovered_node_handle > 1) {
            if (ImGui::MenuItem("Delete node")) {
              node_graph.RecycleNode(hovered_node_handle);
              changed = true;
            }
          } else if (hovered_link_handle != -1) {
            if (ImGui::MenuItem("Delete link")) {
              node_graph.RecycleLink(hovered_link_handle);
              changed = true;
            }
          } else {
            if (ImGui::BeginMenu("New node...")) {
              if (ImGui::BeginMenu("Generators")) {
                if (ImGui::MenuItem("Constant")) {
                  const auto new_node_handle = node_graph.AllocateNode(0, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Constant;
                  const auto node_impl = std::make_shared<ConstantNode>();
                  node_impl->value = 0.f;
                  node.data.node_impl = node_impl;
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                ImGui::EndMenu();
              }
              if (ImGui::BeginMenu("Combiners")) {
                if (ImGui::MenuItem("Add")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Add;
                  const auto node_impl = std::make_shared<AddNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Subtract")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Subtract;
                  const auto node_impl = std::make_shared<SubtractNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Multiply")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Multiply;
                  const auto node_impl = std::make_shared<MultiplyNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Divide")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Divide;
                  const auto node_impl = std::make_shared<DivideNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Power")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Power;
                  const auto node_impl = std::make_shared<PowerNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "power";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Min")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Min;
                  const auto node_impl = std::make_shared<MinNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Max")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Max;
                  const auto node_impl = std::make_shared<MaxNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                ImGui::EndMenu();
              }
              if (ImGui::BeginMenu("Modifiers")) {
                if (ImGui::MenuItem("Abs")) {
                  const auto new_node_handle = node_graph.AllocateNode(1, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Abs;
                  const auto node_impl = std::make_shared<AbsNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Negate")) {
                  const auto new_node_handle = node_graph.AllocateNode(1, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Negate;
                  const auto node_impl = std::make_shared<NegateNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Exponent")) {
                  const auto new_node_handle = node_graph.AllocateNode(1, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Exponent;
                  const auto node_impl = std::make_shared<ExponentNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Clamp")) {
                  const auto new_node_handle = node_graph.AllocateNode(3, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Clamp;
                  const auto node_impl = std::make_shared<ClampNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "lower";
                  node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "upper";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("FlipUp")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::FlipUp;
                  const auto node_impl = std::make_shared<FlipUpNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "base";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("FlipDown")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::FlipDown;
                  const auto node_impl = std::make_shared<FlipDownNode>();
                  node.data.node_impl = node_impl;
                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "base";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                ImGui::EndMenu();
              }

              if (ImGui::BeginMenu("Activation")) {
                if (ImGui::MenuItem("Sigmoid")) {
                  const auto new_node_handle = node_graph.AllocateNode(4, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Sigmoid;
                  const auto node_impl = std::make_shared<SigmoidNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "speed";
                  node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "x";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("SoftSign")) {
                  const auto new_node_handle = node_graph.AllocateNode(4, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::SoftSign;
                  const auto node_impl = std::make_shared<SoftSignNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "speed";
                  node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "x";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Tanh")) {
                  const auto new_node_handle = node_graph.AllocateNode(4, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Tanh;
                  const auto node_impl = std::make_shared<TanhNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "speed";
                  node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "x";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                ImGui::EndMenu();
              }
              ImGui::EndMenu();
            }
          }
        },
        [&](const NodeGraphOutputPinHandle start_handle, const NodeGraphInputPinHandle end_handle) {
          const auto& input_pin = node_graph.PeekInputPin(end_handle);
          if (input_pin.GetLinkHandle() == -1) {
            node_graph.AllocateLink(start_handle, end_handle);
            changed = true;
          }
        },
        [&](const NodeGraphLinkHandle link_handle) {
          node_graph.RecycleLink(link_handle);
          changed = true;
        }

    );
  }
  ImGui::End();
  return changed;
}
void IDynamicTreeStrands::SerializeImpl(YAML::Emitter& out) const {
  node_graph.Save(
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
        data.node_impl->Serialize(node_out);
        node_out << YAML::EndMap;
      },
      [&](YAML::Emitter& link_out, const int& data) {

      });
}
void IDynamicTreeStrands::DeserializeImpl(const YAML::Node& in) {
  node_graph.Load(
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
        switch (data.type) {
          case NodeType::Unknown:
            ImGui::Text("Unknown");
            break;
          case NodeType::InputSegment:
            ImGui::Text("Segment");
            data.node_impl = std::make_shared<InputNode>();
            break;
          case NodeType::InputParticle:
            ImGui::Text("Particle");
            data.node_impl = std::make_shared<InputNode>();
            break;
          case NodeType::OutputDensity:
            ImGui::Text("Density");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputStretchShearModulus:
            ImGui::Text("Stretch Shear Modulus");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputBendingModulus:
            ImGui::Text("Bending Modulus");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputTwistingModulus:
            ImGui::Text("Twisting Modulus");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputSheerStretchStrength:
            ImGui::Text("Sheer Stretch Strength");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputBendingStrength:
            ImGui::Text("Bending Strength");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputTwistingStrength:
            ImGui::Text("Twisting Strength");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputBundleStrength:
            ImGui::Text("Bundle Strength");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputConnectivityStrength:
            ImGui::Text("Connectivity Strength");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::OutputTrunk:
            ImGui::Text("Trunk");
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::Constant:
            ImGui::Text("Constant");
            data.node_impl = std::make_shared<ConstantNode>();
            break;
          case NodeType::Add:
            data.node_impl = std::make_shared<AddNode>();
            break;
          case NodeType::Subtract:
            data.node_impl = std::make_shared<SubtractNode>();
            break;
          case NodeType::Multiply:
            data.node_impl = std::make_shared<MultiplyNode>();
            break;
          case NodeType::Divide:
            data.node_impl = std::make_shared<DivideNode>();
            break;
          case NodeType::Power:
            data.node_impl = std::make_shared<PowerNode>();
            break;
          case NodeType::Min:
            data.node_impl = std::make_shared<MinNode>();
            break;
          case NodeType::Max:
            data.node_impl = std::make_shared<MaxNode>();
            break;
          case NodeType::Abs:
            data.node_impl = std::make_shared<AbsNode>();
            break;
          case NodeType::Clamp:
            data.node_impl = std::make_shared<ClampNode>();
            break;
          case NodeType::Negate:
            data.node_impl = std::make_shared<NegateNode>();
            break;
          case NodeType::Exponent:
            data.node_impl = std::make_shared<ExponentNode>();
            break;
          case NodeType::FlipUp:
            data.node_impl = std::make_shared<FlipUpNode>();
            break;
          case NodeType::FlipDown:
            data.node_impl = std::make_shared<FlipDownNode>();
            break;

          case NodeType::Sigmoid:
            data.node_impl = std::make_shared<SigmoidNode>();
            break;
          case NodeType::SoftSign:
            data.node_impl = std::make_shared<SoftSignNode>();
            break;
          case NodeType::Tanh:
            data.node_impl = std::make_shared<TanhNode>();
            break;
        }
        if (node_in["C"]) {
          data.node_impl->Deserialize(node_in["C"]);
        }
      },
      [&](const YAML::Node& link_in, int& data) {

      });
}
bool INode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
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
void INode::Serialize(YAML::Emitter& out) const {
}
void INode::Deserialize(const YAML::Node& in) {
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
