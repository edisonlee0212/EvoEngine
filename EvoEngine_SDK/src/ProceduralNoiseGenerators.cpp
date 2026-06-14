#include "ProceduralNoiseGenerators.hpp"

using namespace evo_engine;
using namespace evo_engine::procedural_noise;
void ConstantNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
                           const NodeGraphNodeHandle node_handle,
                           std::unordered_map<NodeGraphOutputPinHandle, float>& results) const {
  const auto& node = graph.PeekNode(node_handle);
  for (const auto& output_pin_handle : node.GetOutputPinHandles()) {
    results[output_pin_handle] = value;
  }
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
