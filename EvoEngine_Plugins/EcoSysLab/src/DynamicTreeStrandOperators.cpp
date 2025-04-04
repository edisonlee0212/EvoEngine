#include "DynamicTreeStrandOperators.hpp"

using namespace eco_sys_lab_plugin;

void AddNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 + value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void SubtractNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 - value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void MultiplyNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 * value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void DivideNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = value0 / value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void PowerNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = glm::pow(value0, value1);
  results[node.GetOutputPinHandles()[0]] = result;
}
void MinNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = glm::min(value0, value1);
  results[node.GetOutputPinHandles()[0]] = result;
}
void MaxNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  float value1 = 0.f;
  if (const auto link_handle = graph.PeekInputPin(node.GetInputPinHandles()[1]).GetLinkHandle(); link_handle != -1) {
    if (const auto search = results.find(graph.PeekLink(link_handle).GetOutputPinHandle()); search != results.end()) {
      value1 = search->second;
    }
  }

  const float result = glm::max(value0, value1);
  results[node.GetOutputPinHandles()[0]] = result;
}
void AbsNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  results[node.GetOutputPinHandles()[0]] = glm::abs(value0);
}
void ClampNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  const float result = glm::clamp(value0, value1, value2);
  results[node.GetOutputPinHandles()[0]] = result;
}
void ExponentNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  results[node.GetOutputPinHandles()[0]] = glm::exp(value0);
}
void NegateNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  results[node.GetOutputPinHandles()[0]] = -value0;
}
void FlipUpNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = glm::abs(value0 - value1) + value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void FlipDownNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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

  const float result = -glm::abs(value0 - value1) + value1;
  results[node.GetOutputPinHandles()[0]] = result;
}
void SigmoidNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  const float result = ActivationFunction::Sigmoid(value0, value1, 0, value2, value3);
  results[node.GetOutputPinHandles()[0]] = result;
}
void SoftSignNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  const float result = ActivationFunction::SoftSign(value0, value1, 0, value2, value3);
  results[node.GetOutputPinHandles()[0]] = result;
}
void TanhNode::Process(const NodeGraph<InputPinData, OutputPinData, NodeData, int>& graph,
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
  const float result = ActivationFunction::Tanh(value0, value1, 0, value2, value3);
  results[node.GetOutputPinHandles()[0]] = result;
}
