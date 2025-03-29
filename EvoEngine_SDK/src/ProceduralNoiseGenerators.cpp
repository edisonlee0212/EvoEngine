#include "ProceduralNoiseGenerators.hpp"

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
void ConstantNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                           const NodeGraphNodeHandle node_handle,
                           std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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

bool Perlin2DNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void Perlin2DNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                           const NodeGraphNodeHandle node_handle,
                           std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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

  const float result = glm::perlin(glm::vec2(value0, value1) * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}
void Perlin2DNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void Perlin2DNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

void Simplex2DNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void Simplex2DNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

bool Simplex2DNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void Simplex2DNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                            const NodeGraphNodeHandle node_handle,
                            std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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

  const float result = glm::simplex(glm::vec2(value0, value1) * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}
bool Perlin3DNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void Perlin3DNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                           const NodeGraphNodeHandle node_handle,
                           std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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

  const float result = glm::perlin(glm::vec3(value0, value1, value2) * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}
void Perlin3DNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void Perlin3DNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

void Simplex3DNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void Simplex3DNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

bool Simplex3DNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void Simplex3DNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                            const NodeGraphNodeHandle node_handle,
                            std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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

  const float result = glm::simplex(glm::vec3(value0, value1, value2) * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}

bool Perlin4DNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void Perlin4DNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                           const NodeGraphNodeHandle node_handle,
                           std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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
void Perlin4DNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void Perlin4DNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

void Simplex4DNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void Simplex4DNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

bool Simplex4DNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void Simplex4DNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                            const NodeGraphNodeHandle node_handle,
                            std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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

void SineNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void SineNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
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
void SineNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                       const NodeGraphNodeHandle node_handle,
                       std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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

bool CosineNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void CosineNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                         NodeGraphNodeHandle node_handle,
                         std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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
  const float result = glm::cos(value0 * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}
void CosineNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void CosineNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}

bool TangentNode::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f)) {
    changed = true;
  }
  ImGui::PopItemWidth();
  return changed;
}
void TangentNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                          NodeGraphNodeHandle node_handle,
                          std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
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
  const float result = glm::tan(value0 * frequency);
  results[node.GetOutputPinHandles()[0]] = result;
}
void TangentNode::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "frequency" << YAML::Value << frequency;
}
void TangentNode::Deserialize(const YAML::Node& in) {
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
}