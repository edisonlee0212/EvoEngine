#include "EcoSysLabGraphEditors.hpp"
#include "DynamicTreeStrandOperators.hpp"
#include "NodeGraphEditor.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectModulusGraph(InspectorContext& context, ModulusGraph& target,
                                              ModulusGraphInspectorState& state) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  // Test output nodes for all their pins

  ImGui::Text("Test Inputs:");
  if (ImGui::DragFloat("Root Distance", &state.temp_input.root_distance)) {
    state.temp_output_density = target.GetDensity(state.temp_input);
    state.temp_output_shear_stretch = target.GetShearStretchModulus(state.temp_input);
    state.temp_output_bending = target.GetBendingModulus(state.temp_input);
    state.temp_output_twisting = target.GetTwistingModulus(state.temp_input);
  }
  if (ImGui::DragFloat("Polar Distance", &state.temp_input.polar_distance)) {
    state.temp_output_density = target.GetDensity(state.temp_input);
    state.temp_output_shear_stretch = target.GetShearStretchModulus(state.temp_input);
    state.temp_output_bending = target.GetBendingModulus(state.temp_input);
    state.temp_output_twisting = target.GetTwistingModulus(state.temp_input);
  }
  if (ImGui::DragFloat("Polar Angle", &state.temp_input.polar_angle)) {
    state.temp_output_density = target.GetDensity(state.temp_input);
    state.temp_output_shear_stretch = target.GetShearStretchModulus(state.temp_input);
    state.temp_output_bending = target.GetBendingModulus(state.temp_input);
    state.temp_output_twisting = target.GetTwistingModulus(state.temp_input);
  }
  if (ImGui::DragFloat("Profile Boundary Distance", &state.temp_input.profile_boundary_distance)) {
    state.temp_output_density = target.GetDensity(state.temp_input);
    state.temp_output_shear_stretch = target.GetShearStretchModulus(state.temp_input);
    state.temp_output_bending = target.GetBendingModulus(state.temp_input);
    state.temp_output_twisting = target.GetTwistingModulus(state.temp_input);
  }
  if (ImGui::Button("Calculate")) {
    state.temp_output_density = target.GetDensity(state.temp_input);
    state.temp_output_shear_stretch = target.GetShearStretchModulus(state.temp_input);
    state.temp_output_bending = target.GetBendingModulus(state.temp_input);
    state.temp_output_twisting = target.GetTwistingModulus(state.temp_input);
  }

  ImGui::Text("Density min: %.3f", state.temp_output_density[0]);
  ImGui::Text("Density max: %.3f", state.temp_output_density[1]);
  ImGui::Text("Shear Stretch lower bound: %.3f", state.temp_output_shear_stretch[0]);
  ImGui::Text("Shear Stretch upper bound: %.3f", state.temp_output_shear_stretch[1]);
  ImGui::Text("Bending lower bound: %.3f", state.temp_output_bending[0]);
  ImGui::Text("Bending upper bound: %.3f", state.temp_output_bending[1]);
  ImGui::Text("Twisting lower bound: %.3f", state.temp_output_twisting[0]);
  ImGui::Text("Twisting upper bound: %.3f", state.temp_output_twisting[1]);

  // Test setting values
  if (ImGui::Button("Set Density")) {
    glm::vec2 values{};
    target.SetDensity(values);
  }
  if (ImGui::Button("Set Shear Stretch Modulus")) {
    glm::vec2 values{};
    target.SetShearStretchModulus(values);
  }
  if (ImGui::Button("Set Bending Modulus")) {
    glm::vec2 values{};
    target.SetBendingModulus(values);
  }
  if (ImGui::Button("Set Twisting Modulus")) {
    glm::vec2 values{};
    target.SetTwistingModulus(values);
  }

  ImGui::Checkbox("Show modulus node graph", &state.show_node_graph);
  if (state.show_node_graph) {
    changed = DrawStrandGraph(target, "Dynamic Tree Strands", editor_layer) || changed;
  }
  return changed;
}

bool eco_sys_lab_package::InspectStrengthGraph(InspectorContext& context, StrengthGraph& target,
                                               StrengthGraphInspectorState& state) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  // Test output nodes for all their pins

  ImGui::Text("Test Inputs:");
  if (ImGui::DragFloat("Root Distance", &state.temp_input.root_distance)) {
    state.temp_output_shear_stretch = target.GetShearStretchStrength(state.temp_input);
    state.temp_output_bending = target.GetBendingStrength(state.temp_input);
    state.temp_output_twisting = target.GetTwistingStrength(state.temp_input);
    state.temp_output_bundle = target.GetBundleStrength(state.temp_input);
    state.temp_output_connectivity = target.GetConnectivityStrength(state.temp_input);
  }
  if (ImGui::DragFloat("Polar Distance", &state.temp_input.polar_distance)) {
    state.temp_output_shear_stretch = target.GetShearStretchStrength(state.temp_input);
    state.temp_output_bending = target.GetBendingStrength(state.temp_input);
    state.temp_output_twisting = target.GetTwistingStrength(state.temp_input);
    state.temp_output_bundle = target.GetBundleStrength(state.temp_input);
    state.temp_output_connectivity = target.GetConnectivityStrength(state.temp_input);
  }
  if (ImGui::DragFloat("Polar Angle", &state.temp_input.polar_angle)) {
    state.temp_output_shear_stretch = target.GetShearStretchStrength(state.temp_input);
    state.temp_output_bending = target.GetBendingStrength(state.temp_input);
    state.temp_output_twisting = target.GetTwistingStrength(state.temp_input);
    state.temp_output_bundle = target.GetBundleStrength(state.temp_input);
    state.temp_output_connectivity = target.GetConnectivityStrength(state.temp_input);
  }
  if (ImGui::DragFloat("Profile Boundary Distance", &state.temp_input.profile_boundary_distance)) {
    state.temp_output_shear_stretch = target.GetShearStretchStrength(state.temp_input);
    state.temp_output_bending = target.GetBendingStrength(state.temp_input);
    state.temp_output_twisting = target.GetTwistingStrength(state.temp_input);
    state.temp_output_bundle = target.GetBundleStrength(state.temp_input);
    state.temp_output_connectivity = target.GetConnectivityStrength(state.temp_input);
  }
  if (ImGui::Button("Calculate")) {
    state.temp_output_shear_stretch = target.GetShearStretchStrength(state.temp_input);
    state.temp_output_bending = target.GetBendingStrength(state.temp_input);
    state.temp_output_twisting = target.GetTwistingStrength(state.temp_input);
    state.temp_output_bundle = target.GetBundleStrength(state.temp_input);
    state.temp_output_connectivity = target.GetConnectivityStrength(state.temp_input);
  }

  ImGui::Text("Shear Stretch lower bound: %.3f", state.temp_output_shear_stretch[0]);
  ImGui::Text("Shear Stretch upper bound: %.3f", state.temp_output_shear_stretch[1]);
  ImGui::Text("Bending lower bound: %.3f", state.temp_output_bending[0]);
  ImGui::Text("Bending upper bound: %.3f", state.temp_output_bending[1]);
  ImGui::Text("Twisting lower bound: %.3f", state.temp_output_twisting[0]);
  ImGui::Text("Twisting upper bound: %.3f", state.temp_output_twisting[1]);
  ImGui::Text("Bundle lower bound: %.3f", state.temp_output_bundle[0]);
  ImGui::Text("Bundle upper bound: %.3f", state.temp_output_bundle[1]);
  ImGui::Text("Connectivity lower bound: %.3f", state.temp_output_connectivity[0]);
  ImGui::Text("Connectivity upper bound: %.3f", state.temp_output_connectivity[1]);

  // Test setting values
  if (ImGui::Button("Set Shear Stretch Strength")) {
    glm::vec2 values{};
    target.SetShearStretchStrength(values);
  }
  if (ImGui::Button("Set Bending Strength")) {
    glm::vec2 values{};
    target.SetBendingStrength(values);
  }
  if (ImGui::Button("Set Twisting Strength")) {
    glm::vec2 values{};
    target.SetTwistingStrength(values);
  }
  if (ImGui::Button("Set Bundle Strength")) {
    glm::vec2 values{};
    target.SetBundleStrength(values);
  }
  if (ImGui::Button("Set Connectivity Strength")) {
    glm::vec2 values{};
    target.SetConnectivityStrength(values);
  }

  ImGui::Checkbox("Show strength node graph", &state.show_node_graph);
  if (state.show_node_graph) {
    changed = DrawStrandGraph(target, "Dynamic Tree Strands", editor_layer) || changed;
  }

  return changed;
}

bool eco_sys_lab_package::InspectBiologicalPropertiesGraph(InspectorContext& context, BiologicalPropertiesGraph& target,
                                                           BiologicalPropertiesGraphInspectorState& state) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  // Test output node for all three pins
  ImGui::Text("Test Inputs:");
  if (ImGui::DragFloat("Root Distance", &state.temp_input.root_distance)) {
    state.temp_output = target.GetValues(state.temp_input);
  }
  if (ImGui::DragFloat("Polar Distance", &state.temp_input.polar_distance)) {
    state.temp_output = target.GetValues(state.temp_input);
  }
  if (ImGui::DragFloat("Polar Angle", &state.temp_input.polar_angle)) {
    state.temp_output = target.GetValues(state.temp_input);
  }
  if (ImGui::DragFloat("Profile Boundary Distance", &state.temp_input.profile_boundary_distance)) {
    state.temp_output = target.GetValues(state.temp_input);
  }
  if (ImGui::Button("Calculate")) {
    state.temp_output = target.GetValues(state.temp_input);
  }
  ImGui::Text("input: %.3f", state.temp_output.trunk_offset);
  ImGui::Text("transition: %.3f", state.temp_output.trunk_transition);
  ImGui::Text("additional strength factor: %.3f", state.temp_output.trunk_additional_strength_factor);

  // Test setting values
  if (ImGui::Button("Set Values")) {
    BiologicalPropertiesGraph::Output values;
    target.SetValues(values);
  }

  ImGui::Checkbox("Show biological properties node graph", &state.show_node_graph);
  if (state.show_node_graph) {
    changed = DrawStrandGraph(target, "Dynamic Tree Strands", editor_layer) || changed;
  }

  return changed;
}

bool eco_sys_lab_package::DrawStrandGraph(IDynamicTreeStrands& target, const std::string& window_title,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  InspectorContext context{editor_layer, ApplicationContext::Get().GetActiveScene()};
  if (ImGui::Begin(window_title.c_str())) {
    const auto id = ImGui::GetID(window_title.c_str());

    ImGui::PushID(&target);
    auto* storage = ImGui::GetStateStorage();
    const auto node_key = ImGui::GetID("hovered-node");
    const auto link_key = ImGui::GetID("hovered-link");
    auto hovered_node_handle = storage->GetInt(node_key, -1);
    auto hovered_link_handle = storage->GetInt(link_key, -1);

    NodeGraphEditor::Draw(
        target.node_graph, dynamic_cast<IAsset&>(target).GetSelf(), id, editor_layer,
        [&](const NodeGraphNodeHandle node_handle) {
          const auto& node = target.node_graph.PeekNode(node_handle);
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
          if (InspectorRegistry::GetInstance().InspectValue(context,
                                                            *target.node_graph.RefNode(node_handle).data.node_impl)) {
            changed = true;
          }
        },
        [&](const NodeGraphInputPinHandle input_pin_handle) {
          ImGui::TextUnformatted(target.node_graph.RefInputPin(input_pin_handle).data.name.c_str());
        },
        [&](const NodeGraphOutputPinHandle output_pin_handle) {
          ImGui::TextUnformatted(target.node_graph.RefOutputPin(output_pin_handle).data.name.c_str());
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
              target.node_graph.RecycleNode(hovered_node_handle);
              changed = true;
            }
          } else if (hovered_link_handle != -1) {
            if (ImGui::MenuItem("Delete link")) {
              target.node_graph.RecycleLink(hovered_link_handle);
              changed = true;
            }
          } else {
            if (ImGui::BeginMenu("New node...")) {
              if (ImGui::BeginMenu("Generators")) {
                if (ImGui::MenuItem("Constant")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(0, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Constant;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Constant);
                  node.data.node_impl = node_impl;
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                ImGui::EndMenu();
              }
              if (ImGui::BeginMenu("Combiners")) {
                if (ImGui::MenuItem("Add")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Add;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Add);
                  node.data.node_impl = node_impl;

                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Subtract")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Subtract;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Subtract);
                  node.data.node_impl = node_impl;

                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Multiply")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Multiply;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Multiply);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Divide")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Divide;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Divide);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Power")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Power;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Power);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "power";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Min")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Min;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Min);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Max")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Max;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Max);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                ImGui::EndMenu();
              }
              if (ImGui::BeginMenu("Modifiers")) {
                if (ImGui::MenuItem("Abs")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(1, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Abs;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Abs);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Negate")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(1, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Negate;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Negate);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Exponent")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(1, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Exponent;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Exponent);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Clamp")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(3, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Clamp;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Clamp);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "lower";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "upper";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("FlipUp")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::FlipUp;
                  const auto node_impl = CreateStrandGraphNode(NodeType::FlipUp);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "base";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("FlipDown")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(2, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::FlipDown;
                  const auto node_impl = CreateStrandGraphNode(NodeType::FlipDown);
                  node.data.node_impl = node_impl;
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "base";
                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                ImGui::EndMenu();
              }

              if (ImGui::BeginMenu("Activation")) {
                if (ImGui::MenuItem("Sigmoid")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(4, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Sigmoid;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Sigmoid);
                  node.data.node_impl = node_impl;

                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "speed";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "x";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("SoftSign")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(4, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::SoftSign;
                  const auto node_impl = CreateStrandGraphNode(NodeType::SoftSign);
                  node.data.node_impl = node_impl;

                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "speed";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "x";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Tanh")) {
                  const auto new_node_handle = target.node_graph.AllocateNode(4, 1);
                  auto& node = target.node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Tanh;
                  const auto node_impl = CreateStrandGraphNode(NodeType::Tanh);
                  node.data.node_impl = node_impl;

                  target.node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "a";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "b";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "speed";
                  target.node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "x";

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
          const auto& input_pin = target.node_graph.PeekInputPin(end_handle);
          if (input_pin.GetLinkHandle() == -1) {
            target.node_graph.AllocateLink(start_handle, end_handle);
            changed = true;
          }
        },
        [&](const NodeGraphLinkHandle link_handle) {
          target.node_graph.RecycleLink(link_handle);
          changed = true;
        }

    );
    storage->SetInt(node_key, hovered_node_handle);
    storage->SetInt(link_key, hovered_link_handle);
    ImGui::PopID();
  }
  ImGui::End();
  return changed;
}

bool eco_sys_lab_package::InspectConstantNode(InspectorContext& context, ConstantNode& target) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Value", &target.value, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
