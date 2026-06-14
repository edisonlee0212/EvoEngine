#include "ProceduralNoise.hpp"

#include "ProceduralNoiseGenerators.hpp"
#include "ProceduralNoiseOperators.hpp"

using namespace evo_engine::procedural_noise;

template <typename T>
static void SaveFrequency(YAML::Emitter& out, const NodeData& data) {
  if (const auto node = std::dynamic_pointer_cast<T>(data.node_impl)) {
    out << YAML::Key << "frequency" << YAML::Value << node->frequency;
  }
}

template <typename T>
static void LoadFrequency(const YAML::Node& in, const NodeData& data) {
  if (const auto node = std::dynamic_pointer_cast<T>(data.node_impl); node && in["frequency"]) {
    node->frequency = in["frequency"].as<float>();
  }
}

static void SaveNodePayload(YAML::Emitter& out, const NodeData& data) {
  switch (data.type) {
    case NodeType::Constant:
      if (const auto node = std::dynamic_pointer_cast<ConstantNode>(data.node_impl)) {
        out << YAML::Key << "value" << YAML::Value << node->value;
      }
      break;
    case NodeType::Sine:
      SaveFrequency<SineNode>(out, data);
      break;
    case NodeType::Cosine:
      SaveFrequency<CosineNode>(out, data);
      break;
    case NodeType::Tangent:
      SaveFrequency<TangentNode>(out, data);
      break;
    case NodeType::Simplex2D:
      SaveFrequency<Simplex2DNode>(out, data);
      break;
    case NodeType::Simplex3D:
      SaveFrequency<Simplex3DNode>(out, data);
      break;
    case NodeType::Simplex4D:
      SaveFrequency<Simplex4DNode>(out, data);
      break;
    case NodeType::Perlin2D:
      SaveFrequency<Perlin2DNode>(out, data);
      break;
    case NodeType::Perlin3D:
      SaveFrequency<Perlin3DNode>(out, data);
      break;
    case NodeType::Perlin4D:
      SaveFrequency<Perlin4DNode>(out, data);
      break;
    default:
      break;
  }
}

static void LoadNodePayload(const YAML::Node& in, const NodeData& data) {
  switch (data.type) {
    case NodeType::Constant:
      if (const auto node = std::dynamic_pointer_cast<ConstantNode>(data.node_impl); node && in["value"]) {
        node->value = in["value"].as<float>();
      }
      break;
    case NodeType::Sine:
      LoadFrequency<SineNode>(in, data);
      break;
    case NodeType::Cosine:
      LoadFrequency<CosineNode>(in, data);
      break;
    case NodeType::Tangent:
      LoadFrequency<TangentNode>(in, data);
      break;
    case NodeType::Simplex2D:
      LoadFrequency<Simplex2DNode>(in, data);
      break;
    case NodeType::Simplex3D:
      LoadFrequency<Simplex3DNode>(in, data);
      break;
    case NodeType::Simplex4D:
      LoadFrequency<Simplex4DNode>(in, data);
      break;
    case NodeType::Perlin2D:
      LoadFrequency<Perlin2DNode>(in, data);
      break;
    case NodeType::Perlin3D:
      LoadFrequency<Perlin3DNode>(in, data);
      break;
    case NodeType::Perlin4D:
      LoadFrequency<Perlin4DNode>(in, data);
      break;
    default:
      break;
  }
}

void ProceduralNoise2D::Reset() {
  node_graph = {};
  const auto output_node_handle = node_graph.AllocateNode(1, 0);
  const auto input_node_handle = node_graph.AllocateNode(0, 2);
  auto& output_node = node_graph.RefNode(output_node_handle);
  output_node.data.type = NodeType::Output;
  output_node.data.node_impl = std::make_shared<OutputNode>();
  auto& input_node = node_graph.RefNode(input_node_handle);
  input_node.data.type = NodeType::Input;
  input_node.data.node_impl = std::make_shared<InputNode>();
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[0]).data.name = "x";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[1]).data.name = "y";

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(&node_graph.RefImNodesEditorContext());
    ImNodes::SetNodeScreenSpacePos(output_node_handle, ImVec2(400, 250));
    ImNodes::SetNodeScreenSpacePos(input_node_handle, ImVec2(100, 250));
    ImNodes::EditorContextSet(prev_editor_context);
  }
}
ProceduralNoise2D::ProceduralNoise2D() {
  Reset();
}

void ProceduralNoise3D::Reset() {
  node_graph = {};
  const auto output_node_handle = node_graph.AllocateNode(1, 0);
  const auto input_node_handle = node_graph.AllocateNode(0, 3);
  auto& output_node = node_graph.RefNode(output_node_handle);
  output_node.data.type = NodeType::Output;
  output_node.data.node_impl = std::make_shared<OutputNode>();
  auto& input_node = node_graph.RefNode(input_node_handle);
  input_node.data.type = NodeType::Input;
  input_node.data.node_impl = std::make_shared<InputNode>();
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[0]).data.name = "x";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[1]).data.name = "y";
  node_graph.RefOutputPin(input_node.GetOutputPinHandles()[2]).data.name = "z";

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(&node_graph.RefImNodesEditorContext());
    ImNodes::SetNodeScreenSpacePos(output_node_handle, ImVec2(400, 250));
    ImNodes::SetNodeScreenSpacePos(input_node_handle, ImVec2(100, 250));
    ImNodes::EditorContextSet(prev_editor_context);
  }
}
ProceduralNoise3D::ProceduralNoise3D() {
  Reset();
}

void ProceduralNoise4D::Reset() {
  node_graph = {};
  const auto output_node_handle = node_graph.AllocateNode(1, 0);
  const auto input_node_handle = node_graph.AllocateNode(0, 4);
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

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(&node_graph.RefImNodesEditorContext());
    ImNodes::SetNodeScreenSpacePos(output_node_handle, ImVec2(400, 250));
    ImNodes::SetNodeScreenSpacePos(input_node_handle, ImVec2(100, 250));
    ImNodes::EditorContextSet(prev_editor_context);
  }
}
ProceduralNoise4D::ProceduralNoise4D() {
  Reset();
}

void evo_engine::procedural_noise::SaveProceduralNoiseGraph(YAML::Emitter& out, const IProceduralNoise& noise) {
  noise.node_graph.Save(
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
        SaveNodePayload(node_out, data);
        node_out << YAML::EndMap;
      },
      [&](YAML::Emitter& link_out, const int& data) {

      });
}
void evo_engine::procedural_noise::LoadProceduralNoiseGraph(const YAML::Node& in, IProceduralNoise& noise) {
  noise.node_graph.Load(
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
          case NodeType::Sine:
            data.node_impl = std::make_shared<SineNode>();
            break;
          case NodeType::Cosine:
            data.node_impl = std::make_shared<CosineNode>();
            break;
          case NodeType::Tangent:
            data.node_impl = std::make_shared<TangentNode>();
            break;
          case NodeType::Simplex2D:
            data.node_impl = std::make_shared<Simplex2DNode>();
            break;
          case NodeType::Simplex3D:
            data.node_impl = std::make_shared<Simplex3DNode>();
            break;
          case NodeType::Simplex4D:
            data.node_impl = std::make_shared<Simplex4DNode>();
            break;
          case NodeType::Perlin2D:
            data.node_impl = std::make_shared<Perlin2DNode>();
            break;
          case NodeType::Perlin3D:
            data.node_impl = std::make_shared<Perlin3DNode>();
            break;
          case NodeType::Perlin4D:
            data.node_impl = std::make_shared<Perlin4DNode>();
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
          LoadNodePayload(node_in["C"], data);
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

float ProceduralNoise2D::GetValue(const glm::vec2& offset) const {
  const auto& input_node = node_graph.PeekNode(1);
  std::unordered_map<NodeGraphOutputPinHandle, float> results{};
  const auto& output_pin_handles = input_node.GetOutputPinHandles();
  results[output_pin_handles[0]] = offset.x;
  results[output_pin_handles[1]] = offset.y;

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

float ProceduralNoise3D::GetValue(const glm::vec3& offset) const {
  const auto& input_node = node_graph.PeekNode(1);
  std::unordered_map<NodeGraphOutputPinHandle, float> results{};
  const auto& output_pin_handles = input_node.GetOutputPinHandles();
  results[output_pin_handles[0]] = offset.x;
  results[output_pin_handles[1]] = offset.y;
  results[output_pin_handles[2]] = offset.z;

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

float ProceduralNoise4D::GetValue(const glm::vec4& offset) const {
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

void ProceduralNoise2D::OnCreate() {
  Reset();
}

void ProceduralNoise3D::OnCreate() {
  Reset();
}

void ProceduralNoise4D::OnCreate() {
  Reset();
}
