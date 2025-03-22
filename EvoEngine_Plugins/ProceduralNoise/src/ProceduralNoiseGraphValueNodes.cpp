#include "ProceduralNoiseGraphValueNodes.hpp"

using namespace evo_engine;
using namespace evo_engine::procedural_noise;
bool ConstantNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Value", &value, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void ConstantNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                           const NodeGraphNodeHandle node_handle,
                           std::unordered_map<NodeGraphOutputPinHandle, float>& results) {
  const auto& node = graph.PeekNode(node_handle);
  for (const auto& output_pin_handle : node.GetOutputPinHandles()) {
    results[output_pin_handle] = value;
  }
}
void ConstantNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "value" << YAML::Value << value;
}
void ConstantNode::Deserialize(const YAML::Node& in) {
  if (in["value"])
    value = in["value"].as<float>();
}
bool PerlinNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void PerlinNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                         const NodeGraphNodeHandle node_handle,
                         std::unordered_map<NodeGraphOutputPinHandle, float>& results) {
  const auto& node = graph.PeekNode(node_handle);
  if (const auto search = results.find(node.GetOutputPinHandles()[0]); search != results.end())
    return;
  PrepareInputs(graph, node_handle, results);
  float value0 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[0]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value0 = search->second;
    }
  }
  float value1 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[1]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value1 = search->second;
    }
  }
  float value2 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[2]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value2 = search->second;
    }
  }
  float value3 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[3]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value3 = search->second;
    }
  }

  const float result = glm::perlin(glm::vec4(value0, value1, value2, value3) * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}
void PerlinNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void PerlinNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

void SimplexNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void SimplexNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

void SineNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void SineNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

bool SimplexNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void SimplexNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                          const NodeGraphNodeHandle node_handle,
                          std::unordered_map<NodeGraphOutputPinHandle, float>& results) {
  const auto& node = graph.PeekNode(node_handle);
  if (const auto search = results.find(node.GetOutputPinHandles()[0]); search != results.end())
    return;
  PrepareInputs(graph, node_handle, results);
  float value0 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[0]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value0 = search->second;
    }
  }
  float value1 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[1]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value1 = search->second;
    }
  }
  float value2 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[2]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value2 = search->second;
    }
  }
  float value3 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[3]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value3 = search->second;
    }
  }

  const float result = glm::simplex(glm::vec4(value0, value1, value2, value3) * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}
bool SineNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void SineNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                       const NodeGraphNodeHandle node_handle,
                       std::unordered_map<NodeGraphOutputPinHandle, float>& results) {
  const auto& node = graph.PeekNode(node_handle);
  if (const auto search = results.find(node.GetOutputPinHandles()[0]); search != results.end())
    return;
  PrepareInputs(graph, node_handle, results);
  float value0 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[0]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value0 = search->second;
    }
  }
  const float result = glm::sin(value0 * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}