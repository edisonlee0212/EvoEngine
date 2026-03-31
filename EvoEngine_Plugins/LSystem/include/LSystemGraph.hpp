#pragma once

#include <cassert>
#include <queue>
#include <unordered_map>
#include <map>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace l_system_plugin {

typedef int LNodeHandle;
typedef int LFlowHandle;

#pragma region Structural Info

struct LNodeInfo {
  bool locked = false;

  glm::vec3 global_position = glm::vec3(0.0f);
  glm::quat global_rotation = glm::vec3(0.0f);

  float length = 0.0f;
  float thickness = 0.1f;
  float root_distance = 0.0f;
  float end_distance = 0.0f;
  int chain_index = 0;
  glm::quat regulated_global_rotation = glm::vec3(0.0f);

  glm::vec4 color = glm::vec4(1.0f);

  float volume = 0;
  float descendant_total_volume = 0;
  int order = 0;
  int level = 0;
  bool max_child = false;

  [[nodiscard]] glm::vec3 GetGlobalEndPosition() const;
  [[nodiscard]] glm::vec3 GetGlobalCenterPosition() const;
  [[nodiscard]] glm::vec3 GetGlobalDirection() const;
};

inline glm::vec3 LNodeInfo::GetGlobalEndPosition() const {
  return global_position + glm::normalize(global_rotation * glm::vec3(0, 0, -1)) * length;
}

inline glm::vec3 LNodeInfo::GetGlobalCenterPosition() const {
  return global_position + glm::normalize(global_rotation * glm::vec3(0, 0, -1)) * length * 0.5f;
}

inline glm::vec3 LNodeInfo::GetGlobalDirection() const {
  return glm::normalize(global_rotation * glm::vec3(0, 0, -1));
}

struct LFlowInfo {
  glm::vec3 global_start_position = glm::vec3(0.0f);
  glm::quat global_start_rotation = glm::vec3(0.0f);
  float start_thickness = 0.0f;

  glm::vec3 global_end_position = glm::vec3(0.0f);
  glm::quat global_end_rotation = glm::vec3(0.0f);
  float end_thickness = 0.0f;

  float flow_length = 0.0f;
  int order = 0;
};
#pragma endregion

/**
 * @brief A node in the L-system graph, representing a single module instance.
 * @tparam ModuleData The typed data stored per-module (a std::variant of module types).
 */
template <typename ModuleData>
class LGraphNode {
  template <typename Fd>
  friend class LGraphFlow;

  template <typename Gd, typename Fd, typename Md>
  friend class LSystemGraph;

  bool end_node_ = true;
  LNodeHandle handle_ = -1;
  LFlowHandle flow_handle_ = -1;
  LNodeHandle parent_handle_ = -1;
  std::vector<LNodeHandle> child_handles_;
  bool apical_ = true;
  int index_ = -1;

 public:
  int symbol_id = -1;       ///< Which module type this node represents.
  ModuleData data;           ///< User-defined per-module data (variant or struct).
  LNodeInfo info;            ///< Structural geometry info.

  [[nodiscard]] bool IsEndNode() const;
  [[nodiscard]] bool IsApical() const;
  [[nodiscard]] LNodeHandle GetHandle() const;
  [[nodiscard]] LNodeHandle GetParentHandle() const;
  [[nodiscard]] LFlowHandle GetFlowHandle() const;
  [[nodiscard]] const std::vector<LNodeHandle>& PeekChildHandles() const;
  [[nodiscard]] std::vector<LNodeHandle>& UnsafeRefChildHandles();

  LGraphNode() = default;
  explicit LGraphNode(LNodeHandle handle);
  [[nodiscard]] int GetIndex() const;
};

/**
 * @brief A flow (linear chain of nodes) in the L-system graph.
 * @tparam FlowData The type of data stored per-flow.
 */
template <typename FlowData>
class LGraphFlow {
  template <typename Gd, typename Fd, typename Md>
  friend class LSystemGraph;

  LFlowHandle handle_ = -1;
  std::vector<LNodeHandle> nodes_;
  LFlowHandle parent_handle_ = -1;
  std::vector<LFlowHandle> child_handles_;
  bool apical_ = false;
  int index_ = -1;

 public:
  FlowData data;
  LFlowInfo info;

  [[nodiscard]] bool IsApical() const;
  [[nodiscard]] LFlowHandle GetHandle() const;
  [[nodiscard]] LFlowHandle GetParentHandle() const;
  [[nodiscard]] const std::vector<LFlowHandle>& PeekChildHandles() const;
  [[nodiscard]] const std::vector<LNodeHandle>& PeekNodeHandles() const;

  LGraphFlow() = default;
  explicit LGraphFlow(LFlowHandle handle);
  [[nodiscard]] int GetIndex() const;
};

struct BaseLGraphData {};
struct BaseLFlowData {};
struct BaseLModuleData {};

/**
 * @brief The primary data structure for the L-system plugin.
 *
 * Forked from eco_sys_lab_plugin::Skeleton<> with L-system-specific additions.
 * Operates as a graph of typed modules with flows representing linear chains.
 *
 * @tparam GraphData  Whole-graph data (e.g., cumulative GDD, global state).
 * @tparam FlowData   Per-flow data.
 * @tparam ModuleData Per-node module data (typically a std::variant of module types).
 */
template <typename GraphData, typename FlowData, typename ModuleData>
class LSystemGraph {
  template <typename Gd, typename Fd, typename Md>
  friend class LSystemGraph;

  std::vector<LGraphFlow<FlowData>> flows_;
  std::vector<LGraphNode<ModuleData>> nodes_;

  int new_version_ = 0;
  int version_ = -1;

  std::vector<LNodeHandle> sorted_node_list_;
  std::vector<LFlowHandle> sorted_flow_list_;

  LNodeHandle AllocateNode();
  LFlowHandle AllocateFlow();

  void SetParentFlow(LFlowHandle target_handle, LFlowHandle parent_handle);
  void DetachChildFlow(LFlowHandle target_handle, LFlowHandle child_handle);
  void SetParentNode(LNodeHandle target_handle, LNodeHandle parent_handle);
  void DetachChildNode(LNodeHandle target_handle, LNodeHandle child_handle);

  int max_node_index_ = -1;
  int max_flow_index_ = -1;

  std::vector<LNodeHandle> base_node_list_;
  void RefreshBaseNodeList();

  int max_level_ = 0;
  int max_order_ = 0;

 public:
  [[nodiscard]] int GetMaxLevel() const;
  [[nodiscard]] int GetMaxOrder() const;
  [[nodiscard]] int GetMaxNodeIndex() const;
  [[nodiscard]] int GetMaxFlowIndex() const;

  GraphData data;  ///< Whole-graph data.

  void CalculateDistanceVolumeLevel();
  void CalculateRegulatedGlobalRotation();
  void CalculateMinMax();

  void RemoveNodes(const std::vector<LNodeHandle>& node_handles);

  [[nodiscard]] LNodeHandle Extend(LNodeHandle target_handle, bool branching);

  [[nodiscard]] const std::vector<LNodeHandle>& PeekBaseNodeList();
  [[nodiscard]] const std::vector<LNodeHandle>& PeekSortedNodeList() const;
  [[nodiscard]] std::vector<LNodeHandle> GetSubTree(LNodeHandle base_node_handle) const;
  [[nodiscard]] std::vector<LNodeHandle> GetChainToRoot(LNodeHandle end_node_handle) const;
  [[nodiscard]] const std::vector<LFlowHandle>& PeekSortedFlowList() const;

  [[nodiscard]] std::vector<LGraphFlow<FlowData>>& RefRawFlows();
  [[nodiscard]] std::vector<LGraphNode<ModuleData>>& RefRawNodes();
  [[nodiscard]] const std::vector<LGraphFlow<FlowData>>& PeekRawFlows() const;
  [[nodiscard]] const std::vector<LGraphNode<ModuleData>>& PeekRawNodes() const;

  void SortLists();

  explicit LSystemGraph(unsigned initial_node_count = 1);

  [[nodiscard]] int GetVersion() const;

  void CalculateFlows();

  LGraphNode<ModuleData>& RefNode(LNodeHandle handle);
  LGraphFlow<FlowData>& RefFlow(LFlowHandle handle);
  [[nodiscard]] const LGraphNode<ModuleData>& PeekNode(LNodeHandle handle) const;
  [[nodiscard]] const LGraphFlow<FlowData>& PeekFlow(LFlowHandle handle) const;

  glm::vec3 min = glm::vec3(0.0f);
  glm::vec3 max = glm::vec3(0.0f);
};

typedef LSystemGraph<BaseLGraphData, BaseLFlowData, BaseLModuleData> BaseLSystemGraph;

// =============================================================================
// Template implementations
// =============================================================================

#pragma region LGraphNode

template <typename ModuleData>
LGraphNode<ModuleData>::LGraphNode(const LNodeHandle handle) {
  handle_ = handle;
  end_node_ = true;
  symbol_id = -1;
  data = {};
  info = {};
  index_ = -1;
}

template <typename ModuleData>
bool LGraphNode<ModuleData>::IsEndNode() const {
  return end_node_;
}

template <typename ModuleData>
bool LGraphNode<ModuleData>::IsApical() const {
  return apical_;
}

template <typename ModuleData>
LNodeHandle LGraphNode<ModuleData>::GetHandle() const {
  return handle_;
}

template <typename ModuleData>
LNodeHandle LGraphNode<ModuleData>::GetParentHandle() const {
  return parent_handle_;
}

template <typename ModuleData>
LFlowHandle LGraphNode<ModuleData>::GetFlowHandle() const {
  return flow_handle_;
}

template <typename ModuleData>
const std::vector<LNodeHandle>& LGraphNode<ModuleData>::PeekChildHandles() const {
  return child_handles_;
}

template <typename ModuleData>
std::vector<LNodeHandle>& LGraphNode<ModuleData>::UnsafeRefChildHandles() {
  return child_handles_;
}

template <typename ModuleData>
int LGraphNode<ModuleData>::GetIndex() const {
  return index_;
}

#pragma endregion

#pragma region LGraphFlow

template <typename FlowData>
LGraphFlow<FlowData>::LGraphFlow(const LFlowHandle handle) {
  handle_ = handle;
  data = {};
  info = {};
  apical_ = false;
  index_ = -1;
}

template <typename FlowData>
int LGraphFlow<FlowData>::GetIndex() const {
  return index_;
}

template <typename FlowData>
const std::vector<LNodeHandle>& LGraphFlow<FlowData>::PeekNodeHandles() const {
  return nodes_;
}

template <typename FlowData>
LFlowHandle LGraphFlow<FlowData>::GetParentHandle() const {
  return parent_handle_;
}

template <typename FlowData>
const std::vector<LFlowHandle>& LGraphFlow<FlowData>::PeekChildHandles() const {
  return child_handles_;
}

template <typename FlowData>
LFlowHandle LGraphFlow<FlowData>::GetHandle() const {
  return handle_;
}

template <typename FlowData>
bool LGraphFlow<FlowData>::IsApical() const {
  return apical_;
}

#pragma endregion

#pragma region LSystemGraph

template <typename GraphData, typename FlowData, typename ModuleData>
LSystemGraph<GraphData, FlowData, ModuleData>::LSystemGraph(const unsigned initial_node_count) {
  max_node_index_ = -1;
  max_flow_index_ = -1;
  for (unsigned i = 0; i < initial_node_count; i++) {
    auto flow_handle = AllocateFlow();
    auto node_handle = AllocateNode();
    auto& root_flow = flows_[flow_handle];
    auto& root_node = nodes_[node_handle];
    root_node.flow_handle_ = flow_handle;
    root_flow.nodes_.emplace_back(node_handle);
    base_node_list_.emplace_back(node_handle);
  }
}

// --- Accessors ---

template <typename GraphData, typename FlowData, typename ModuleData>
LGraphFlow<FlowData>& LSystemGraph<GraphData, FlowData, ModuleData>::RefFlow(LFlowHandle handle) {
  assert(handle >= 0 && handle < static_cast<int>(flows_.size()));
  return flows_[handle];
}

template <typename GraphData, typename FlowData, typename ModuleData>
const LGraphFlow<FlowData>& LSystemGraph<GraphData, FlowData, ModuleData>::PeekFlow(LFlowHandle handle) const {
  assert(handle >= 0 && handle < static_cast<int>(flows_.size()));
  return flows_[handle];
}

template <typename GraphData, typename FlowData, typename ModuleData>
LGraphNode<ModuleData>& LSystemGraph<GraphData, FlowData, ModuleData>::RefNode(LNodeHandle handle) {
  assert(handle >= 0 && handle < static_cast<int>(nodes_.size()));
  return nodes_[handle];
}

template <typename GraphData, typename FlowData, typename ModuleData>
const LGraphNode<ModuleData>& LSystemGraph<GraphData, FlowData, ModuleData>::PeekNode(LNodeHandle handle) const {
  assert(handle >= 0 && handle < static_cast<int>(nodes_.size()));
  return nodes_[handle];
}

template <typename GraphData, typename FlowData, typename ModuleData>
int LSystemGraph<GraphData, FlowData, ModuleData>::GetVersion() const {
  return version_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
int LSystemGraph<GraphData, FlowData, ModuleData>::GetMaxNodeIndex() const {
  return max_node_index_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
int LSystemGraph<GraphData, FlowData, ModuleData>::GetMaxFlowIndex() const {
  return max_flow_index_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
int LSystemGraph<GraphData, FlowData, ModuleData>::GetMaxLevel() const {
  return max_level_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
int LSystemGraph<GraphData, FlowData, ModuleData>::GetMaxOrder() const {
  return max_order_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
std::vector<LGraphFlow<FlowData>>& LSystemGraph<GraphData, FlowData, ModuleData>::RefRawFlows() {
  return flows_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
std::vector<LGraphNode<ModuleData>>& LSystemGraph<GraphData, FlowData, ModuleData>::RefRawNodes() {
  return nodes_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
const std::vector<LGraphFlow<FlowData>>& LSystemGraph<GraphData, FlowData, ModuleData>::PeekRawFlows() const {
  return flows_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
const std::vector<LGraphNode<ModuleData>>& LSystemGraph<GraphData, FlowData, ModuleData>::PeekRawNodes() const {
  return nodes_;
}

// --- Topology ---

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::SortLists() {
  if (version_ == new_version_)
    return;
  if (nodes_.empty())
    return;
  version_ = new_version_;
  sorted_flow_list_.clear();
  sorted_node_list_.clear();
  RefreshBaseNodeList();
  std::queue<LFlowHandle> flow_wait_list;
  std::queue<LNodeHandle> node_wait_list;

  for (const auto& base_node_handle : base_node_list_) {
    node_wait_list.push(base_node_handle);
    flow_wait_list.push(nodes_[base_node_handle].flow_handle_);
  }

  while (!flow_wait_list.empty()) {
    sorted_flow_list_.emplace_back(flow_wait_list.front());
    flow_wait_list.pop();
    for (const auto& i : flows_[sorted_flow_list_.back()].child_handles_) {
      flow_wait_list.push(i);
    }
  }

  while (!node_wait_list.empty()) {
    sorted_node_list_.emplace_back(node_wait_list.front());
    node_wait_list.pop();
    for (const auto& i : nodes_[sorted_node_list_.back()].child_handles_) {
      node_wait_list.push(i);
    }
  }
}

template <typename GraphData, typename FlowData, typename ModuleData>
const std::vector<LFlowHandle>& LSystemGraph<GraphData, FlowData, ModuleData>::PeekSortedFlowList() const {
  return sorted_flow_list_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
const std::vector<LNodeHandle>& LSystemGraph<GraphData, FlowData, ModuleData>::PeekSortedNodeList() const {
  return sorted_node_list_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
std::vector<LNodeHandle> LSystemGraph<GraphData, FlowData, ModuleData>::GetSubTree(
    const LNodeHandle base_node_handle) const {
  std::vector<LNodeHandle> ret_val{};
  std::queue<LNodeHandle> node_handles;
  node_handles.push(base_node_handle);
  while (!node_handles.empty()) {
    auto next_node_handle = node_handles.front();
    ret_val.emplace_back(node_handles.front());
    node_handles.pop();
    for (const auto& child_handle : nodes_[next_node_handle].child_handles_) {
      node_handles.push(child_handle);
    }
  }
  return ret_val;
}

template <typename GraphData, typename FlowData, typename ModuleData>
std::vector<LNodeHandle> LSystemGraph<GraphData, FlowData, ModuleData>::GetChainToRoot(
    const LNodeHandle end_node_handle) const {
  std::vector<LNodeHandle> ret_val{};
  LNodeHandle walker = end_node_handle;
  while (walker != -1) {
    ret_val.emplace_back(walker);
    walker = nodes_[walker].parent_handle_;
  }
  return ret_val;
}

template <typename GraphData, typename FlowData, typename ModuleData>
LNodeHandle LSystemGraph<GraphData, FlowData, ModuleData>::Extend(LNodeHandle target_handle, const bool branching) {
  assert(target_handle < static_cast<int>(nodes_.size()));
  auto& target_node = nodes_[target_handle];
  assert(target_node.flow_handle_ < static_cast<int>(flows_.size()));
  auto new_node_handle = AllocateNode();
  SetParentNode(new_node_handle, target_handle);
  auto& original_node = nodes_[target_handle];
  auto& new_node = nodes_[new_node_handle];
  original_node.end_node_ = false;
  if (branching) {
    auto new_flow_handle = AllocateFlow();
    auto& new_flow = flows_[new_flow_handle];

    new_node.flow_handle_ = new_flow_handle;
    new_node.apical_ = false;
    new_flow.nodes_.emplace_back(new_node_handle);
    new_flow.apical_ = false;
    if (target_handle != flows_[original_node.flow_handle_].nodes_.back()) {
      auto extended_flow_handle = AllocateFlow();
      auto& extended_flow = flows_[extended_flow_handle];
      extended_flow.apical_ = true;
      auto& original_flow = flows_[original_node.flow_handle_];
      for (auto r = original_flow.nodes_.begin(); r != original_flow.nodes_.end(); ++r) {
        if (*r == target_handle) {
          extended_flow.nodes_.insert(extended_flow.nodes_.end(), r + 1, original_flow.nodes_.end());
          original_flow.nodes_.erase(r + 1, original_flow.nodes_.end());
          break;
        }
      }
      for (const auto& extracted_node_handle : extended_flow.nodes_) {
        auto& extracted_node = nodes_[extracted_node_handle];
        extracted_node.flow_handle_ = extended_flow_handle;
      }
      extended_flow.child_handles_ = original_flow.child_handles_;
      original_flow.child_handles_.clear();
      for (const auto& child_flow_handle : extended_flow.child_handles_) {
        flows_[child_flow_handle].parent_handle_ = extended_flow_handle;
      }
      SetParentFlow(extended_flow_handle, original_node.flow_handle_);
    }
    SetParentFlow(new_flow_handle, original_node.flow_handle_);
  } else {
    auto& flow = flows_[original_node.flow_handle_];
    flow.nodes_.emplace_back(new_node_handle);
    new_node.flow_handle_ = original_node.flow_handle_;
    new_node.apical_ = true;
  }
  new_version_++;
  return new_node_handle;
}

template <typename GraphData, typename FlowData, typename ModuleData>
const std::vector<LNodeHandle>& LSystemGraph<GraphData, FlowData, ModuleData>::PeekBaseNodeList() {
  RefreshBaseNodeList();
  return base_node_list_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::RemoveNodes(const std::vector<LNodeHandle>& node_handles) {
  SortLists();
  std::unordered_map<LNodeHandle, uint32_t> sorted_node_indices;
  std::unordered_map<LFlowHandle, uint32_t> sorted_flow_indices;

  for (uint32_t i = 0; i < sorted_node_list_.size(); i++) {
    sorted_node_indices[sorted_node_list_[i]] = i;
  }
  for (uint32_t i = 0; i < sorted_flow_list_.size(); i++) {
    sorted_flow_indices[sorted_flow_list_[i]] = i;
  }
  std::map<uint32_t, LNodeHandle> collected_node_handle_set{};
  std::map<uint32_t, LFlowHandle> collected_flow_handle_set{};

  std::queue<LNodeHandle> processing_node_handles;
  for (const auto& i : node_handles) {
    processing_node_handles.emplace(i);
  }
  while (!processing_node_handles.empty()) {
    auto node_handle = processing_node_handles.front();
    processing_node_handles.pop();
    collected_node_handle_set[sorted_node_indices.at(node_handle)] = node_handle;
    const auto& node = nodes_[node_handle];
    if (const auto& flow = flows_[node.flow_handle_]; !flow.nodes_.empty() && flow.nodes_.front() == node_handle) {
      collected_flow_handle_set[sorted_flow_indices.at(node.flow_handle_)] = node.flow_handle_;
    }
    for (const auto& child_node_handle : node.child_handles_) {
      processing_node_handles.push(child_node_handle);
    }
  }

  std::vector<LNodeHandle> sorted_node_handle_removal_list;
  std::vector<LFlowHandle> sorted_flow_handle_removal_list;
  for (auto i = collected_node_handle_set.rbegin(); i != collected_node_handle_set.rend(); ++i) {
    sorted_node_handle_removal_list.emplace_back(i->second);
  }
  for (auto i = collected_flow_handle_set.rbegin(); i != collected_flow_handle_set.rend(); ++i) {
    sorted_flow_handle_removal_list.emplace_back(i->second);
  }

  // Remove nodes via swap-and-pop.
  for (uint32_t i = 0; i < sorted_node_handle_removal_list.size(); i++) {
    const auto removal_node_handle = sorted_node_handle_removal_list[i];
    auto& node = nodes_[removal_node_handle];
    if (node.parent_handle_ != -1) {
      auto& parent_node = nodes_[node.parent_handle_];
      for (int32_t ci = parent_node.child_handles_.size() - 1; ci >= 0; --ci) {
        if (parent_node.child_handles_[ci] == removal_node_handle) {
          parent_node.child_handles_.erase(parent_node.child_handles_.begin() + ci);
          break;
        }
      }
    }
    auto& flow = flows_[node.flow_handle_];
    for (int32_t fi = flow.nodes_.size() - 1; fi >= 0; --fi) {
      if (flow.nodes_[fi] == removal_node_handle) {
        flow.nodes_.erase(flow.nodes_.begin() + fi);
        break;
      }
    }
    if (removal_node_handle != static_cast<int>(nodes_.size()) - 1) {
      auto& repair_node = nodes_[removal_node_handle];
      repair_node = nodes_.back();
      const auto repair_node_handle = static_cast<int>(nodes_.size()) - 1;
      repair_node.handle_ = removal_node_handle;
      for (auto& handle : sorted_node_handle_removal_list) {
        if (handle == repair_node_handle) {
          handle = removal_node_handle;
        }
      }
      if (repair_node.parent_handle_ != -1) {
        auto& parent_node = nodes_[repair_node.parent_handle_];
        for (auto ci = parent_node.child_handles_.rbegin(); ci != parent_node.child_handles_.rend(); ++ci) {
          if (*ci == repair_node_handle) {
            *ci = removal_node_handle;
            break;
          }
        }
      }
      for (const auto& child_handle : repair_node.child_handles_) {
        nodes_[child_handle].parent_handle_ = removal_node_handle;
      }
      auto& repair_flow = flows_[repair_node.flow_handle_];
      for (int32_t fi = repair_flow.nodes_.size() - 1; fi >= 0; --fi) {
        if (repair_flow.nodes_[fi] == repair_node_handle) {
          repair_flow.nodes_[fi] = removal_node_handle;
          break;
        }
      }
    }
    nodes_.pop_back();
  }

  // Remove flows via swap-and-pop.
  for (uint32_t i = 0; i < sorted_flow_handle_removal_list.size(); i++) {
    const auto removal_flow_handle = sorted_flow_handle_removal_list[i];
    auto& flow = flows_[removal_flow_handle];
    if (flow.parent_handle_ != -1 && flow.parent_handle_ < static_cast<int>(flows_.size())) {
      auto& parent_flow = flows_[flow.parent_handle_];
      for (int32_t ci = parent_flow.child_handles_.size() - 1; ci >= 0; --ci) {
        if (parent_flow.child_handles_[ci] == removal_flow_handle) {
          parent_flow.child_handles_.erase(parent_flow.child_handles_.begin() + ci);
          break;
        }
      }
    }
    assert(flow.nodes_.empty());
    if (removal_flow_handle != static_cast<int>(flows_.size()) - 1) {
      auto& repair_flow = flows_[removal_flow_handle];
      repair_flow = flows_.back();
      const auto repair_flow_handle = static_cast<int>(flows_.size()) - 1;
      repair_flow.handle_ = removal_flow_handle;
      for (auto& handle : sorted_flow_handle_removal_list) {
        if (handle == repair_flow_handle) {
          handle = removal_flow_handle;
        }
      }
      if (repair_flow.parent_handle_ != -1) {
        auto& parent_flow = flows_[repair_flow.parent_handle_];
        for (auto ci = parent_flow.child_handles_.rbegin(); ci != parent_flow.child_handles_.rend(); ++ci) {
          if (*ci == repair_flow_handle) {
            *ci = removal_flow_handle;
            break;
          }
        }
      }
      for (const auto& child_handle : repair_flow.child_handles_) {
        flows_[child_handle].parent_handle_ = removal_flow_handle;
      }
      for (const auto& node_handle : repair_flow.nodes_) {
        nodes_[node_handle].flow_handle_ = removal_flow_handle;
      }
    }
    flows_.pop_back();
  }
  new_version_++;
  SortLists();
}

// --- Calculation passes ---

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::CalculateDistanceVolumeLevel() {
  for (const auto& node_handle : sorted_node_list_) {
    auto& node = nodes_[node_handle];
    auto& node_info = node.info;
    node_info.volume = node_info.thickness * node_info.thickness * node_info.length;
    if (node.GetParentHandle() == -1) {
      node_info.root_distance = node_info.length;
      node_info.chain_index = 0;
    } else {
      const auto& parent_node = nodes_[node.GetParentHandle()];
      node_info.root_distance = parent_node.info.root_distance + node_info.length;
      if (node.IsApical()) {
        node.info.chain_index = parent_node.info.chain_index + 1;
      } else {
        node.info.chain_index = 0;
      }
    }
  }
  for (auto it = sorted_node_list_.rbegin(); it != sorted_node_list_.rend(); ++it) {
    auto& node = nodes_[*it];
    float max_distance_to_any_branch_end = 0;
    node.info.end_distance = 0;
    node.info.descendant_total_volume = 0;
    for (const auto& i : node.PeekChildHandles()) {
      const auto& child_node = nodes_[i];
      const float child_max_dist = child_node.info.end_distance + child_node.info.length;
      max_distance_to_any_branch_end = glm::max(max_distance_to_any_branch_end, child_max_dist);
      node.info.descendant_total_volume += child_node.info.volume + child_node.info.descendant_total_volume;
    }
    node.info.end_distance = max_distance_to_any_branch_end;
  }
  max_level_ = 0;
  max_order_ = 0;

  for (const auto& flow_handle : sorted_flow_list_) {
    auto& flow = flows_[flow_handle];
    if (flow.GetParentHandle() == -1) {
      flow.info.order = 0;
    } else {
      const auto& parent_flow = flows_[flow.GetParentHandle()];
      flow.info.order = flow.IsApical() ? parent_flow.info.order : parent_flow.info.order + 1;
    }
    max_order_ = glm::max(max_order_, flow.info.order);
  }

  for (const auto& node_handle : sorted_node_list_) {
    auto& node = nodes_[node_handle];
    auto& node_info = node.info;
    node_info.order = flows_[node.flow_handle_].info.order;
    if (node.GetParentHandle() == -1) {
      node_info.level = 0;
    } else {
      float max_score = 0.0f;
      LNodeHandle max_child = -1;
      for (const auto& child_handle : node.PeekChildHandles()) {
        auto& child_info = nodes_[child_handle].info;
        if (const auto score = child_info.descendant_total_volume + child_info.volume; score > max_score) {
          max_score = score;
          max_child = child_handle;
        }
      }
      for (const auto& child_handle : node.PeekChildHandles()) {
        auto& child_info = nodes_[child_handle].info;
        if (child_handle == max_child) {
          child_info.level = node_info.level;
          child_info.max_child = true;
        } else {
          child_info.level = node_info.level + 1;
          child_info.max_child = false;
        }
      }
    }
    max_level_ = glm::max(max_level_, node_info.level);
  }
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::CalculateRegulatedGlobalRotation() {
  min = glm::vec3(FLT_MAX);
  max = glm::vec3(-FLT_MAX);
  for (const auto& node_handle : sorted_node_list_) {
    auto& node = nodes_[node_handle];
    auto& node_info = node.info;
    min = glm::min(min, node_info.global_position);
    min = glm::min(min, node_info.GetGlobalEndPosition());
    max = glm::max(max, node_info.global_position);
    max = glm::max(max, node_info.GetGlobalEndPosition());
    if (node.parent_handle_ != -1) {
      auto& parent_info = nodes_[node.parent_handle_].info;
      auto front = node_info.global_rotation * glm::vec3(0, 0, -1);
      auto parent_regulated_up = parent_info.regulated_global_rotation * glm::vec3(0, 1, 0);
      auto regulated_up = glm::normalize(glm::cross(glm::cross(front, parent_regulated_up), front));
      node_info.regulated_global_rotation = glm::quatLookAt(front, regulated_up);
    } else {
      node_info.regulated_global_rotation = node_info.global_rotation;
    }
  }
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::CalculateFlows() {
  for (const auto& flow_handle : sorted_flow_list_) {
    auto& flow = flows_[flow_handle];
    auto& first_node = nodes_[flow.nodes_.front()];
    auto& last_node = nodes_[flow.nodes_.back()];
    flow.info.start_thickness = first_node.info.thickness;
    flow.info.global_start_position = first_node.info.global_position;
    flow.info.global_start_rotation = first_node.info.global_rotation;
    flow.info.end_thickness = last_node.info.thickness;
    flow.info.global_end_position =
        last_node.info.global_position + last_node.info.length * (last_node.info.global_rotation * glm::vec3(0, 0, -1));
    flow.info.global_end_rotation = last_node.info.global_rotation;
    flow.info.flow_length = 0.0f;
    for (const auto& node_handle : flow.nodes_) {
      flow.info.flow_length += nodes_[node_handle].info.length;
    }
  }
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::CalculateMinMax() {
  if (nodes_.empty()) {
    min = glm::vec3(0.f);
    max = glm::vec3(0.f);
    return;
  }
  min = glm::vec3(FLT_MAX);
  max = glm::vec3(-FLT_MAX);
  for (const auto& node : nodes_) {
    min = glm::min(min, node.info.global_position);
    min = glm::min(min, node.info.GetGlobalEndPosition());
    max = glm::max(max, node.info.global_position);
    max = glm::max(max, node.info.GetGlobalEndPosition());
  }
}

// --- Internal helpers ---

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::RefreshBaseNodeList() {
  std::vector<LNodeHandle> temp;
  for (const auto& i : base_node_list_)
    if (nodes_[i].parent_handle_ == -1)
      temp.emplace_back(i);
  base_node_list_ = temp;
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::SetParentNode(LNodeHandle target_handle,
                                                                   LNodeHandle parent_handle) {
  assert(target_handle >= 0 && parent_handle >= 0 &&
         target_handle < static_cast<int>(nodes_.size()) && parent_handle < static_cast<int>(nodes_.size()));
  auto& target_node = nodes_[target_handle];
  auto& parent_node = nodes_[parent_handle];
  target_node.parent_handle_ = parent_handle;
  parent_node.child_handles_.emplace_back(target_handle);
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::DetachChildNode(LNodeHandle target_handle,
                                                                     LNodeHandle child_handle) {
  assert(target_handle >= 0 && child_handle >= 0 &&
         target_handle < static_cast<int>(nodes_.size()) && child_handle < static_cast<int>(nodes_.size()));
  auto& target_node = nodes_[target_handle];
  auto& child_node = nodes_[child_handle];
  auto& children = target_node.child_handles_;
  for (size_t i = 0; i < children.size(); i++) {
    if (children[i] == child_handle) {
      children[i] = children.back();
      children.pop_back();
      child_node.parent_handle_ = -1;
      if (children.empty())
        target_node.end_node_ = true;
      return;
    }
  }
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::SetParentFlow(LFlowHandle target_handle,
                                                                   LFlowHandle parent_handle) {
  assert(target_handle >= 0 && parent_handle >= 0 &&
         target_handle < static_cast<int>(flows_.size()) && parent_handle < static_cast<int>(flows_.size()));
  auto& target_flow = flows_[target_handle];
  auto& parent_flow = flows_[parent_handle];
  target_flow.parent_handle_ = parent_handle;
  parent_flow.child_handles_.emplace_back(target_handle);
}

template <typename GraphData, typename FlowData, typename ModuleData>
void LSystemGraph<GraphData, FlowData, ModuleData>::DetachChildFlow(LFlowHandle target_handle,
                                                                     LFlowHandle child_handle) {
  assert(target_handle >= 0 && child_handle >= 0 &&
         target_handle < static_cast<int>(flows_.size()) && child_handle < static_cast<int>(flows_.size()));
  auto& target_flow = flows_[target_handle];
  auto& child_flow = flows_[child_handle];
  if (!child_flow.nodes_.empty()) {
    auto first_node_handle = child_flow.nodes_[0];
    if (auto& first_node = nodes_[first_node_handle]; first_node.parent_handle_ != -1)
      DetachChildNode(first_node.parent_handle_, first_node_handle);
  }
  auto& children = target_flow.child_handles_;
  for (size_t i = 0; i < children.size(); i++) {
    if (children[i] == child_handle) {
      children[i] = children.back();
      children.pop_back();
      child_flow.parent_handle_ = -1;
      return;
    }
  }
}

template <typename GraphData, typename FlowData, typename ModuleData>
LFlowHandle LSystemGraph<GraphData, FlowData, ModuleData>::AllocateFlow() {
  max_flow_index_++;
  flows_.emplace_back(static_cast<LFlowHandle>(flows_.size()));
  flows_.back().index_ = max_flow_index_;
  return flows_.back().handle_;
}

template <typename GraphData, typename FlowData, typename ModuleData>
LNodeHandle LSystemGraph<GraphData, FlowData, ModuleData>::AllocateNode() {
  max_node_index_++;
  nodes_.emplace_back(static_cast<LNodeHandle>(nodes_.size()));
  nodes_.back().index_ = max_node_index_;
  return nodes_.back().handle_;
}

#pragma endregion

}  // namespace l_system_plugin
