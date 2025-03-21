#include "ProceduralNoiseGraphOperatorNodes.hpp"

using namespace evo_engine;
using namespace evo_engine::procedural_noise;

void AddNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 + value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void SubtractNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 - value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void MultiplyNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 * value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void DivideNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 / value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void PowNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = glm::pow(value0, value1);
  results[node.GetOutputPinHandles()[0]] = result;
}
void MinNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = glm::min(value0, value1);
  results[node.GetOutputPinHandles()[0]] = result;
}
void MaxNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph, NodeGraphNodeHandle node_handle,
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

  const float result = glm::max(value0, value1);
  results[node.GetOutputPinHandles()[0]] = result;
}
void FlipUpNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = glm::abs(value0 - value1) + value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void FlipDownNode::Process(NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = -glm::abs(value0 - value1) + value1;
  results[node.GetOutputPinHandles()[0]] = result;
}