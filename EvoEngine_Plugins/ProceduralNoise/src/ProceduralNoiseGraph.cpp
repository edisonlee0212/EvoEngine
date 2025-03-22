#include "ProceduralNoiseGraph.hpp"

#include "ProceduralNoiseGraphOperatorNodes.hpp"
#include "ProceduralNoiseGraphValueNodes.hpp"

using namespace evo_engine::procedural_noise;
bool NodeData::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  changed = node_impl->OnInspect(editor_layer) || changed;
  return changed;
}
bool InputPinData::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text(name.c_str());
  return changed;
}
bool OutputPinData::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text(name.c_str());
  return changed;
}
void ProceduralNoiseGraph::Reset() {
  node_graph = {};
  auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
  ImNodes::EditorContextSet(&node_graph.RefImNodesEditorContext());
  const auto output_node_handle = node_graph.AllocateNode(1, 0);
  ImNodes::SetNodeScreenSpacePos(output_node_handle, ImVec2(400, 250));
  const auto input_node_handle = node_graph.AllocateNode(0, 4);
  ImNodes::SetNodeScreenSpacePos(input_node_handle, ImVec2(100, 250));

  auto& output_node = node_graph.RefNode(output_node_handle);
  output_node.data.type = NodeType::Output;
  output_node.data.node_impl = std::make_shared<OutputNode>();
  auto& input_node = node_graph.RefNode(input_node_handle);
  input_node.data.type = NodeType::Input;
  input_node.data.node_impl = std::make_shared<InputNode>();
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[0]).data.name = "x";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[1]).data.name = "y";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[2]).data.name = "z";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[3]).data.name = "w";
  ImNodes::EditorContextSet(prev_editor_context);
}
bool ProceduralNoiseGraph::ShowGraph(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Begin("Procedural Noise Graph")) {
    const auto id = ImGui::GetID("Procedural Noise Graph");

    static NodeGraphNodeHandle hovered_node_handle = -1;
    static NodeGraphLinkHandle hovered_link_handle = -1;
    static NodeGraphInputPinHandle hovered_input_handle = -1;
    static NodeGraphOutputPinHandle hovered_output_handle = -1;
    node_graph.OnInspect(
        id, editor_layer,
        [&](const NodeGraphNodeHandle node_handle) {
          const auto& node = node_graph.PeekNode(node_handle);
          switch (node.data.type) {
            case NodeType::Unknown:
              ImGui::Text("Unknown");
              break;
            case NodeType::Input:
              ImGui::Text("Input");
              break;
            case NodeType::Output:
              ImGui::Text("Output");
              break;
            case NodeType::Constant:
              ImGui::Text("Constant");
              break;
            case NodeType::Linear:
              ImGui::Text("Linear");
              break;
            case NodeType::Sine:
              ImGui::Text("Sine");
              break;
            case NodeType::Simplex:
              ImGui::Text("Simplex");
              break;
            case NodeType::Perlin:
              ImGui::Text("Perlin");
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
            case NodeType::Pow:
              ImGui::Text("Pow");
              break;
            case NodeType::Min:
              ImGui::Text("Min");
              break;
            case NodeType::Max:
              ImGui::Text("Max");
              break;
            case NodeType::FlipUp:
              ImGui::Text("FlipUp");
              break;
            case NodeType::FlipDown:
              ImGui::Text("FlipDown");
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
          hovered_input_handle = input_pin_handle;
          hovered_output_handle = output_pin_handle;
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
              if (ImGui::BeginMenu("Values")) {
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

                if (ImGui::MenuItem("Perlin")) {
                  const auto new_node_handle = node_graph.AllocateNode(4, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Perlin;
                  const auto node_impl = std::make_shared<PerlinNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "y";
                  node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "z";
                  node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "w";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                if (ImGui::MenuItem("Simplex")) {
                  const auto new_node_handle = node_graph.AllocateNode(4, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Simplex;
                  const auto node_impl = std::make_shared<SimplexNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";
                  node_graph.RefInputPin(node.GetInputPinHandles()[1]).data.name = "y";
                  node_graph.RefInputPin(node.GetInputPinHandles()[2]).data.name = "z";
                  node_graph.RefInputPin(node.GetInputPinHandles()[3]).data.name = "w";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }
                if (ImGui::MenuItem("Sine")) {
                  const auto new_node_handle = node_graph.AllocateNode(1, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Sine;
                  const auto node_impl = std::make_shared<SineNode>();
                  node.data.node_impl = node_impl;

                  node_graph.RefInputPin(node.GetInputPinHandles()[0]).data.name = "x";

                  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
                  changed = true;
                }

                ImGui::EndMenu();
              }
              if (ImGui::BeginMenu("Operators")) {
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

                if (ImGui::MenuItem("Pow")) {
                  const auto new_node_handle = node_graph.AllocateNode(2, 1);
                  auto& node = node_graph.RefNode(new_node_handle);
                  node.data.type = NodeType::Pow;
                  const auto node_impl = std::make_shared<PowNode>();
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
bool INodeImpl::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

void INodeImpl::PrepareInputs(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
void INodeImpl::Serialize(YAML::Emitter& out) const {
}
void INodeImpl::Deserialize(const YAML::Node& in) {
}

void InputNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                        const NodeGraphNodeHandle node_handle,
                        std::unordered_map<NodeGraphOutputPinHandle, float>& results) {
}
void OutputNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                         const NodeGraphNodeHandle node_handle,
                         std::unordered_map<NodeGraphOutputPinHandle, float>& results) {
  PrepareInputs(graph, node_handle, results);
}

float ProceduralNoiseGraph::GetValue(const glm::vec4& offset) {
  const auto& input_node = node_graph.PeekNode(1);
  std::unordered_map<NodeGraphOutputPinHandle, float> results{};
  const auto& output_pin_handles = input_node.GetOutputPinHandles();
  results[output_pin_handles[0]] = offset.x;
  results[output_pin_handles[1]] = offset.y;
  results[output_pin_handles[2]] = offset.z;
  results[output_pin_handles[3]] = offset.w;

  const auto& output_node = node_graph.PeekNode(0);
  output_node.data.node_impl->Process(node_graph, 0, results);
  const auto& input_pin = node_graph.PeekInputPin(output_node.GetInputPinHandles()[0]);
  const auto link_handle = input_pin.GetLinkHandle();
  if (link_handle != -1) {
    const auto& link = node_graph.PeekLink(link_handle);
    if (const auto search = results.find(link.GetOutputPinHandle()); search != results.end()) {
      return search->second;
    }
  }
  return 0.f;
}

void ProceduralNoiseGraph::OnCreate() {
  Reset();
}
void ProceduralNoiseGraph::Serialize(YAML::Emitter& out) const {
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

void ProceduralNoiseGraph::Deserialize(const YAML::Node& in) {
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
            break;
          case NodeType::Input:
            data.node_impl = std::make_shared<InputNode>();
            break;
          case NodeType::Output:
            data.node_impl = std::make_shared<OutputNode>();
            break;
          case NodeType::Constant:
            data.node_impl = std::make_shared<ConstantNode>();
            break;
          case NodeType::Linear:
            break;
          case NodeType::Sine:
            data.node_impl = std::make_shared<SineNode>();
            break;
          case NodeType::Simplex:
            data.node_impl = std::make_shared<SimplexNode>();
            break;
          case NodeType::Perlin:
            data.node_impl = std::make_shared<PerlinNode>();
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
          case NodeType::Pow:
            data.node_impl = std::make_shared<PowNode>();
            break;
          case NodeType::Min:
            data.node_impl = std::make_shared<MinNode>();
            break;
          case NodeType::Max:
            data.node_impl = std::make_shared<MaxNode>();
            break;
          case NodeType::FlipUp:
            data.node_impl = std::make_shared<FlipUpNode>();
            break;
          case NodeType::FlipDown:
            data.node_impl = std::make_shared<FlipDownNode>();
            break;
        }
        if (node_in["C"]) {
          data.node_impl->Deserialize(node_in["C"]);
        }
      },
      [&](const YAML::Node& link_in, int& data) {

      });
}

bool ProceduralNoiseGraph::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  static glm::vec4 temp_input{};
  static float temp_output = 0.0f;
  if (ImGui::DragFloat4("Test Input", &temp_input.x)) {
    temp_output = GetValue(temp_input);
  }
  if (ImGui::Button("Calculate")) {
    temp_output = GetValue(temp_input);
  }
  ImGui::Text("Test Output: %.3f", temp_output);

  static bool show_node_graph = true;
  ImGui::Checkbox("Show node graph", &show_node_graph);
  if (show_node_graph) {
    changed = ShowGraph(editor_layer) || changed;
  }

  static int resolution = 16;
  bool resolution_changed = false;
  if (ImGui::DragInt("Resolution", &resolution, 1, 16, 1024)) {
    resolution = glm::clamp(resolution, 16, 1024);
    resolution_changed = true;
  }

  static Handle current_handle;

  static bool show_test_texture = true;
  static std::shared_ptr<Texture2D> test_texture_2d;
  if (!test_texture_2d) {
    test_texture_2d = AssetManager::CreateTemporaryAsset<Texture2D>();
  }
  if (show_test_texture) {
    if (changed || resolution_changed || GetHandle() != current_handle) {
      current_handle = GetHandle();
      std::vector<glm::vec4> color(resolution * resolution);
      Jobs::RunParallelFor(resolution * resolution, [&](size_t i) {
        float x = i / resolution;
        float y = i % resolution;
        x /= resolution;
        y /= resolution;
        color[i] = glm::vec4(glm::vec3(GetValue(glm::vec4(x, y, 0, 1))), 1.0f);
      });

      test_texture_2d->SetRgbaChannelData(color, glm::uvec2(resolution));
    }

    const auto texture_storage = test_texture_2d->PeekTexture2DStorage();
    if (texture_storage.im_texture_id) {
      static float debug_scale = 1.f;
      ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
      debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
      ImGui::Image(texture_storage.im_texture_id,
                   ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                          texture_storage.image->GetExtent().height * debug_scale),
                   ImVec2(0, 1), ImVec2(1, 0));
    }
  }
  return changed;
}