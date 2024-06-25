#pragma once

namespace eco_sys_lab {
typedef int SkeletonNodeHandle;
typedef int SkeletonFlowHandle;

#pragma region Structural Info
struct SkeletonNodeWound {
  bool apical = false;
  glm::quat local_rotation = glm::vec3(0.f);
  float thickness = 0.f;
  float healing = 0.f;
};

struct SkeletonNodeInfo {
  bool locked = false;
  /**
   * \brief The global position at the start of the node.
   */
  glm::vec3 global_position = glm::vec3(0.0f);
  /**
   * \brief The global rotation at the start of the node.
   */
  glm::quat global_rotation = glm::vec3(0.0f);

  float length = 0.0f;
  float thickness = 0.1f;
  float root_distance = 0.0f;
  float end_distance = 0.0f;
  int chain_index = 0;
  glm::quat regulated_global_rotation = glm::vec3(0.0f);
  std::vector<SkeletonNodeWound> wounds;

  float leaves = 1.f;
  float fruits = 1.f;
  glm::vec4 color = glm::vec4(1.0f);

  int cluster_index = 0;

  [[nodiscard]] glm::vec3 GetGlobalEndPosition() const;
  [[nodiscard]] glm::vec3 GetGlobalDirection() const;
};

inline glm::vec3 SkeletonNodeInfo::GetGlobalEndPosition() const {
  return global_position + glm::normalize(global_rotation * glm::vec3(0, 0, -1)) * length;
}

inline glm::vec3 SkeletonNodeInfo::GetGlobalDirection() const {
  return glm::normalize(global_rotation * glm::vec3(0, 0, -1));
}
struct SkeletonFlowInfo {
  glm::vec3 global_start_position = glm::vec3(0.0f);
  glm::quat global_start_rotation = glm::vec3(0.0f);
  float start_thickness = 0.0f;

  glm::vec3 global_end_position = glm::vec3(0.0f);
  glm::quat global_end_rotation = glm::vec3(0.0f);
  float end_thickness = 0.0f;

  /**
   * The length from the start of the first node to the end of the last node.
   */
  float flow_length = 0.0f;
};
#pragma endregion

template <typename SkeletonNodeData>
class SkeletonNode {
  template <typename Fd>
  friend class SkeletonFlow;

  template <typename Sd, typename Fd, typename Id>
  friend class Skeleton;

  template <typename Sd, typename Fd, typename Id>
  friend class SkeletonSerializer;

  bool end_node_ = true;
  bool recycled_ = false;
  SkeletonNodeHandle handle_ = -1;
  SkeletonFlowHandle flow_handle_ = -1;
  SkeletonNodeHandle parent_handle_ = -1;
  std::vector<SkeletonNodeHandle> child_handles_;
  bool apical_ = true;
  int index_ = -1;

 public:
  SkeletonNodeData data;
  /**
   * The structural information of current node.
   */
  SkeletonNodeInfo info;

  /**
   * Whether this node is the end node.
   * @return True if this is end node, false else wise.
   */
  [[nodiscard]] bool IsEndNode() const;

  /**
   * Whether this node is recycled (removed).
   * @return True if this node is recycled (removed), false else wise.
   */
  [[nodiscard]] bool IsRecycled() const;
  /**
   * Whether this node is apical_.
   * @return True if this node is apical, false else wise.
   */
  [[nodiscard]] bool IsApical() const;
  /**
   * Get the handle of self.
   * @return NodeHandle of current node.
   */
  [[nodiscard]] SkeletonNodeHandle GetHandle() const;

  /**
   * Get the handle of parent.
   * @return NodeHandle of parent node.
   */
  [[nodiscard]] SkeletonNodeHandle GetParentHandle() const;

  /**
   * Get the handle to belonged flow.
   * @return FlowHandle of belonged flow.
   */
  [[nodiscard]] SkeletonFlowHandle GetFlowHandle() const;

  /**
   * Access the children by their handles.
   * @return The list of handles.
   */
  [[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekChildHandles() const;

  /**
   * Access the children by their handles. Allow modification. Potentially break the skeleton structure!
   * @return The list of handles.
   */
  [[nodiscard]] std::vector<SkeletonNodeHandle>& UnsafeRefChildHandles();
  SkeletonNode() = default;
  SkeletonNode(SkeletonNodeHandle handle);

  [[nodiscard]] int GetIndex() const;
};

template <typename SkeletonFlowData>
class SkeletonFlow {
  template <typename Sd, typename Fd, typename Id>
  friend class Skeleton;

  template <typename Sd, typename Fd, typename Id>
  friend class SkeletonSerializer;

  bool recycled_ = false;
  SkeletonFlowHandle handle_ = -1;
  std::vector<SkeletonNodeHandle> nodes_;
  SkeletonFlowHandle parent_handle_ = -1;
  std::vector<SkeletonFlowHandle> child_handles_;
  bool apical_ = false;
  int index_ = -1;

 public:
  SkeletonFlowData data;
  SkeletonFlowInfo info;

  /**
   * Whether this flow is recycled (removed).
   * @return True if this flow is recycled (removed), false else wise.
   */
  [[nodiscard]] bool IsRecycled() const;

  /**
   * Whether this flow is extended from an apical bud. The apical flow will have the same order as parent flow.
   * @return True if this flow is from apical bud.
   */
  [[nodiscard]] bool IsApical() const;

  /**
   * Get the handle of self.
   * @return FlowHandle of current flow.
   */
  [[nodiscard]] SkeletonFlowHandle GetHandle() const;

  /**
   * Get the handle of parent.
   * @return FlowHandle of parent flow.
   */
  [[nodiscard]] SkeletonFlowHandle GetParentHandle() const;

  /**
   * Access the children by their handles.
   * @return The list of handles.
   */
  [[nodiscard]] const std::vector<SkeletonFlowHandle>& PeekChildHandles() const;

  /**
   * Access the nodes that belongs to this flow.
   * @return The list of handles.
   */
  [[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekNodeHandles() const;
  SkeletonFlow() = default;
  explicit SkeletonFlow(SkeletonFlowHandle handle);

  [[nodiscard]] int GetIndex() const;
};

struct SkeletonClusterSettings {};

template <typename SkeletonData, typename FlowData, typename NodeData>
class Skeleton {
  template <typename Sd, typename Fd, typename Id>
  friend class Skeleton;

  template <typename Sd, typename Fd, typename Id>
  friend class SkeletonSerializer;

  std::vector<SkeletonFlow<FlowData>> flows_;
  std::vector<SkeletonNode<NodeData>> nodes_;
  std::queue<SkeletonNodeHandle> node_pool_;
  std::queue<SkeletonFlowHandle> flow_pool_;

  int new_version_ = 0;
  int version_ = -1;
  std::vector<SkeletonNodeHandle> sorted_node_list_;
  std::vector<SkeletonFlowHandle> sorted_flow_list_;

  SkeletonNodeHandle AllocateNode();

  void RecycleNodeSingle(SkeletonNodeHandle handle, const std::function<void(SkeletonNodeHandle)>& node_handler);

  void RecycleFlowSingle(SkeletonFlowHandle handle, const std::function<void(SkeletonFlowHandle)>& flow_handler);

  SkeletonFlowHandle AllocateFlow();

  void SetParentFlow(SkeletonFlowHandle target_handle, SkeletonFlowHandle parent_handle);

  void DetachChildFlow(SkeletonFlowHandle target_handle, SkeletonFlowHandle child_handle);

  void SetParentNode(SkeletonNodeHandle target_handle, SkeletonNodeHandle parent_handle);

  void DetachChildNode(SkeletonNodeHandle target_handle, SkeletonNodeHandle child_handle);

  int max_node_index_ = -1;
  int max_flow_index_ = -1;

  std::vector<SkeletonNodeHandle> base_node_list_;

  void RefreshBaseNodeList();

 public:
  void CalculateClusters(const SkeletonClusterSettings& cluster_settings);

  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void Clone(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);

  [[nodiscard]] int GetMaxNodeIndex() const;
  [[nodiscard]] int GetMaxFlowIndex() const;
  SkeletonData data;

  void CalculateDistance();
  void CalculateRegulatedGlobalRotation();
  /**
   * Recycle (Remove) a node, the descendants of this node will also be recycled. The relevant flow will also be
   * removed/restructured.
   * @param handle The handle of the node to be removed. Must be valid (non-zero and the node should not be recycled
   * prior to this operation).
   * @param flow_handler Function to be called right before a flow in recycled.
   * @param node_handler Function to be called right before a node in recycled.
   */
  void RecycleNode(SkeletonNodeHandle handle, const std::function<void(SkeletonFlowHandle)>& flow_handler,
                   const std::function<void(SkeletonNodeHandle)>& node_handler);

  /**
   * Recycle (Remove) a flow, the descendants of this flow will also be recycled. The relevant node will also be
   * removed/restructured.
   * @param handle The handle of the flow to be removed. Must be valid (non-zero and the flow should not be recycled
   * prior to this operation).
   * @param flow_handler Function to be called right before a flow in recycled.
   * @param node_handler Function to be called right before a node in recycled.
   */
  void RecycleFlow(SkeletonFlowHandle handle, const std::function<void(SkeletonFlowHandle)>& flow_handler,
                   const std::function<void(SkeletonNodeHandle)>& node_handler);

  /**
   * Branch/prolong node during growth process. The flow structure will also be updated.
   * @param target_handle The handle of the node to branch/prolong
   * @param branching True if branching, false if prolong. During branching, 2 new flows will be generated.
   * @return The handle of new node.
   */
  [[nodiscard]] SkeletonNodeHandle Extend(SkeletonNodeHandle target_handle, bool branching);

  /**
   * To retrieve a list of handles of all nodes contained within the tree.
   * @return The list of handles of nodes sorted from root to ends.
   */
  [[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekBaseNodeList();

  /**
   * To retrieve a list of handles of all nodes contained within the tree.
   * @return The list of handles of nodes sorted from root to ends.
   */
  [[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekSortedNodeList() const;

  [[nodiscard]] std::vector<SkeletonNodeHandle> GetSubTree(SkeletonNodeHandle base_node_handle) const;
  [[nodiscard]] std::vector<SkeletonNodeHandle> GetChainToRoot(SkeletonNodeHandle end_node_handle) const;

  [[nodiscard]] std::vector<SkeletonNodeHandle> GetNodeListBaseIndex(unsigned base_index) const;
  /**
   * To retrieve a list of handles of all flows contained within the tree.
   * @return The list of handles of flows sorted from root to ends.
   */
  [[nodiscard]] const std::vector<SkeletonFlowHandle>& PeekSortedFlowList() const;

  [[nodiscard]] std::vector<SkeletonFlow<FlowData>>& RefRawFlows();

  [[nodiscard]] std::vector<SkeletonNode<NodeData>>& RefRawNodes();

  [[nodiscard]] const std::vector<SkeletonFlow<FlowData>>& PeekRawFlows() const;

  [[nodiscard]] const std::vector<SkeletonNode<NodeData>>& PeekRawNodes() const;

  /**
   *  Force the structure to sort the node and flow list.
   *  \n!!You MUST call this after you prune the tree or altered the tree structure manually!!
   */
  void SortLists();

  Skeleton(unsigned initial_node_count = 1);

  /**
   * Get the structural version of the tree. The version will change when the tree structure changes.
   * @return The version
   */
  [[nodiscard]] int GetVersion() const;

  /**
   * Calculate the structural information of the flows.
   */
  void CalculateFlows();

  /**
   * Retrieve a modifiable reference to the node with the handle.
   * @param handle The handle to the target node.
   * @return The modifiable reference to the node.
   */
  SkeletonNode<NodeData>& RefNode(SkeletonNodeHandle handle);

  /**
   * Retrieve a modifiable reference to the flow with the handle.
   * @param handle The handle to the target flow.
   * @return The modifiable reference to the flow.
   */
  SkeletonFlow<FlowData>& RefFlow(SkeletonFlowHandle handle);

  /**
   * Retrieve a non-modifiable reference to the node with the handle.
   * @param handle The handle to the target node.
   * @return The non-modifiable reference to the node.
   */
  [[nodiscard]] const SkeletonNode<NodeData>& PeekNode(SkeletonNodeHandle handle) const;

  /**
   * Retrieve a non-modifiable reference to the flow with the handle.
   * @param handle The handle to the target flow.
   * @return The non-modifiable reference to the flow.
   */
  [[nodiscard]] const SkeletonFlow<FlowData>& PeekFlow(SkeletonFlowHandle handle) const;

  /**
   * The min value of the bounding box of current tree structure.
   */
  glm::vec3 min = glm::vec3(0.0f);

  /**
   * The max value of the bounding box of current tree structure.
   */
  glm::vec3 max = glm::vec3(0.0f);
};

struct BaseSkeletonData {};
struct BaseFlowData {};
struct BaseNodeData {};

typedef Skeleton<BaseSkeletonData, BaseFlowData, BaseNodeData> BaseSkeleton;

#pragma region TreeSkeleton
#pragma region Helper

template <typename SkeletonData, typename FlowData, typename NodeData>
SkeletonFlow<FlowData>& Skeleton<SkeletonData, FlowData, NodeData>::RefFlow(SkeletonFlowHandle handle) {
  assert(handle >= 0 && handle < flows_.size());
  return flows_[handle];
}

template <typename SkeletonData, typename FlowData, typename NodeData>
const SkeletonFlow<FlowData>& Skeleton<SkeletonData, FlowData, NodeData>::PeekFlow(SkeletonFlowHandle handle) const {
  assert(handle >= 0 && handle < flows_.size());
  return flows_[handle];
}

template <typename SkeletonData, typename FlowData, typename NodeData>
SkeletonNode<NodeData>& Skeleton<SkeletonData, FlowData, NodeData>::RefNode(SkeletonNodeHandle handle) {
  assert(handle >= 0 && handle < nodes_.size());
  return nodes_[handle];
}

template <typename SkeletonData, typename FlowData, typename NodeData>
const SkeletonNode<NodeData>& Skeleton<SkeletonData, FlowData, NodeData>::PeekNode(SkeletonNodeHandle handle) const {
  assert(handle >= 0 && handle < nodes_.size());
  return nodes_[handle];
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::SortLists() {
  if (version_ == new_version_)
    return;
  if (nodes_.empty())
    return;
  version_ = new_version_;
  sorted_flow_list_.clear();
  sorted_node_list_.clear();
  RefreshBaseNodeList();
  std::queue<SkeletonFlowHandle> flow_wait_list;
  std::queue<SkeletonNodeHandle> node_wait_list;

  for (const auto& base_node_handle : base_node_list_) {
    node_wait_list.push(base_node_handle);
    flow_wait_list.push(nodes_[base_node_handle].flow_handle_);
  }

  while (!flow_wait_list.empty()) {
    sorted_flow_list_.emplace_back(flow_wait_list.front());
    flow_wait_list.pop();
    for (const auto& i : flows_[sorted_flow_list_.back()].child_handles_) {
      assert(!flows_[i].recycled_);
      flow_wait_list.push(i);
    }
  }

  while (!node_wait_list.empty()) {
    sorted_node_list_.emplace_back(node_wait_list.front());
    node_wait_list.pop();
    for (const auto& i : nodes_[sorted_node_list_.back()].child_handles_) {
      assert(!nodes_[i].recycled_);
      node_wait_list.push(i);
    }
  }
}

template <typename SkeletonData, typename FlowData, typename NodeData>
const std::vector<SkeletonFlowHandle>& Skeleton<SkeletonData, FlowData, NodeData>::PeekSortedFlowList() const {
  return sorted_flow_list_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
const std::vector<SkeletonNodeHandle>& Skeleton<SkeletonData, FlowData, NodeData>::PeekSortedNodeList() const {
  return sorted_node_list_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
std::vector<SkeletonNodeHandle> Skeleton<SkeletonData, FlowData, NodeData>::GetSubTree(
    const SkeletonNodeHandle base_node_handle) const {
  std::vector<SkeletonNodeHandle> ret_val{};
  std::queue<SkeletonNodeHandle> node_handles;
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

template <typename SkeletonData, typename FlowData, typename NodeData>
std::vector<SkeletonNodeHandle> Skeleton<SkeletonData, FlowData, NodeData>::GetChainToRoot(
    const SkeletonNodeHandle end_node_handle) const {
  std::vector<SkeletonNodeHandle> ret_val{};
  SkeletonNodeHandle walker = end_node_handle;
  while (walker != -1) {
    ret_val.emplace_back(walker);
    walker = nodes_[walker].parent_handle_;
  }
  return ret_val;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
std::vector<SkeletonNodeHandle> Skeleton<SkeletonData, FlowData, NodeData>::GetNodeListBaseIndex(
    unsigned base_index) const {
  std::vector<SkeletonNodeHandle> ret_val{};
  for (const auto& i : sorted_node_list_) {
    if (nodes_[i].index_ >= base_index)
      ret_val.push_back(i);
  }
  return ret_val;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
SkeletonNodeHandle Skeleton<SkeletonData, FlowData, NodeData>::Extend(SkeletonNodeHandle target_handle,
                                                                      const bool branching) {
  assert(target_handle < nodes_.size());
  auto& target_node = nodes_[target_handle];
  assert(!target_node.recycled_);
  assert(target_node.flow_handle_ < flows_.size());
  auto& flow = flows_[target_node.flow_handle_];
  assert(!flow.recycled_);
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
      // Find target node.
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
    flow.nodes_.emplace_back(new_node_handle);
    new_node.flow_handle_ = original_node.flow_handle_;
    new_node.apical_ = true;
  }
  new_version_++;
  return new_node_handle;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
const std::vector<SkeletonNodeHandle>& Skeleton<SkeletonData, FlowData, NodeData>::PeekBaseNodeList() {
  RefreshBaseNodeList();
  return base_node_list_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::RecycleFlow(
    SkeletonFlowHandle handle, const std::function<void(SkeletonFlowHandle)>& flow_handler,
    const std::function<void(SkeletonNodeHandle)>& node_handler) {
  assert(handle != 0);
  assert(!flows_[handle].recycled_);
  auto& flow = flows_[handle];
  // Remove children
  auto children = flow.child_handles_;
  for (const auto& child : children) {
    if (flows_[child].recycled_)
      continue;
    RecycleFlow(child, flow_handler, node_handler);
  }
  // Detach from parent
  auto parent_handle = flow.parent_handle_;
  if (parent_handle != -1)
    DetachChildFlow(parent_handle, handle);
  // Remove node
  if (!flow.nodes_.empty()) {
    // Detach first node from parent.
    auto nodes = flow.nodes_;
    for (const auto& i : nodes) {
      RecycleNodeSingle(i, node_handler);
    }
  }
  RecycleFlowSingle(handle, flow_handler);
  new_version_++;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::RecycleNode(
    SkeletonNodeHandle handle, const std::function<void(SkeletonFlowHandle)>& flow_handler,
    const std::function<void(SkeletonNodeHandle)>& node_handler) {
  assert(handle != 0);
  assert(!nodes_[handle].recycled_);
  auto& node = nodes_[handle];
  auto flow_handle = node.flow_handle_;
  auto& flow = flows_[flow_handle];
  if (handle == flow.nodes_[0]) {
    auto parent_flow_handle = flow.parent_handle_;

    RecycleFlow(node.flow_handle_, flow_handler, node_handler);
    if (parent_flow_handle != -1) {
      // Connect parent branch with the only apical child flow.
      if (auto& parent_flow = flows_[parent_flow_handle]; parent_flow.child_handles_.size() == 1) {
        auto child_handle = parent_flow.child_handles_[0];
        if (auto& child_flow = flows_[child_handle]; child_flow.apical_) {
          for (const auto& node_handle : child_flow.nodes_) {
            nodes_[node_handle].flow_handle_ = parent_flow_handle;
          }
          for (const auto& grand_child_flow_handle : child_flow.child_handles_) {
            flows_[grand_child_flow_handle].parent_handle_ = parent_flow_handle;
          }
          parent_flow.nodes_.insert(parent_flow.nodes_.end(), child_flow.nodes_.begin(), child_flow.nodes_.end());
          parent_flow.child_handles_.clear();
          parent_flow.child_handles_.insert(parent_flow.child_handles_.end(), child_flow.child_handles_.begin(),
                                            child_flow.child_handles_.end());
          RecycleFlowSingle(child_handle, flow_handler);
        }
      }
    }
    return;
  }
  // Collect list of subsequent nodes
  std::vector<SkeletonNodeHandle> subsequent_nodes;
  while (flow.nodes_.back() != handle) {
    subsequent_nodes.emplace_back(flow.nodes_.back());
    flow.nodes_.pop_back();
  }
  subsequent_nodes.emplace_back(flow.nodes_.back());
  flow.nodes_.pop_back();
  assert(!flow.nodes_.empty());
  // Detach from parent
  if (node.parent_handle_ != -1)
    DetachChildNode(node.parent_handle_, handle);
  // From end node remove one by one.
  SkeletonNodeHandle prev = -1;
  for (const auto& i : subsequent_nodes) {
    auto children = nodes_[i].child_handles_;
    for (const auto& child_node_handle : children) {
      if (child_node_handle == prev)
        continue;
      auto& child = nodes_[child_node_handle];
      assert(!child.recycled_);
      auto child_branch_handle = child.flow_handle_;
      if (child_branch_handle != flow_handle) {
        RecycleFlow(child_branch_handle, flow_handler, node_handler);
      }
    }
    prev = i;
    RecycleNodeSingle(i, node_handler);
  }
  new_version_++;
}

#pragma endregion
#pragma region Internal

template <typename NodeData>
SkeletonNode<NodeData>::SkeletonNode(const SkeletonNodeHandle handle) {
  handle_ = handle;
  recycled_ = false;
  end_node_ = true;
  data = {};
  info = {};
  index_ = -1;
}

template <typename NodeData>
bool SkeletonNode<NodeData>::IsEndNode() const {
  return end_node_;
}

template <typename NodeData>
bool SkeletonNode<NodeData>::IsRecycled() const {
  return recycled_;
}

template <typename NodeData>
bool SkeletonNode<NodeData>::IsApical() const {
  return apical_;
}

template <typename NodeData>
SkeletonNodeHandle SkeletonNode<NodeData>::GetHandle() const {
  return handle_;
}

template <typename NodeData>
SkeletonNodeHandle SkeletonNode<NodeData>::GetParentHandle() const {
  return parent_handle_;
}

template <typename NodeData>
SkeletonFlowHandle SkeletonNode<NodeData>::GetFlowHandle() const {
  return flow_handle_;
}

template <typename NodeData>
const std::vector<SkeletonNodeHandle>& SkeletonNode<NodeData>::PeekChildHandles() const {
  return child_handles_;
}

template <typename NodeData>
std::vector<SkeletonNodeHandle>& SkeletonNode<NodeData>::UnsafeRefChildHandles() {
  return child_handles_;
}

template <typename NodeData>
int SkeletonNode<NodeData>::GetIndex() const {
  return index_;
}

template <typename FlowData>
SkeletonFlow<FlowData>::SkeletonFlow(const SkeletonFlowHandle handle) {
  handle_ = handle;
  recycled_ = false;
  data = {};
  info = {};
  apical_ = false;
  index_ = -1;
}

template <typename FlowData>
int SkeletonFlow<FlowData>::GetIndex() const {
  return index_;
}

template <typename FlowData>
const std::vector<SkeletonNodeHandle>& SkeletonFlow<FlowData>::PeekNodeHandles() const {
  return nodes_;
}

template <typename FlowData>
SkeletonFlowHandle SkeletonFlow<FlowData>::GetParentHandle() const {
  return parent_handle_;
}

template <typename FlowData>
const std::vector<SkeletonFlowHandle>& SkeletonFlow<FlowData>::PeekChildHandles() const {
  return child_handles_;
}

template <typename FlowData>
bool SkeletonFlow<FlowData>::IsRecycled() const {
  return recycled_;
}

template <typename FlowData>
SkeletonFlowHandle SkeletonFlow<FlowData>::GetHandle() const {
  return handle_;
}

template <typename FlowData>
bool SkeletonFlow<FlowData>::IsApical() const {
  return apical_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
Skeleton<SkeletonData, FlowData, NodeData>::Skeleton(const unsigned initial_node_count) {
  max_node_index_ = -1;
  max_flow_index_ = -1;
  for (int i = 0; i < initial_node_count; i++) {
    auto flow_handle = AllocateFlow();
    auto node_handle = AllocateNode();
    auto& root_flow = flows_[flow_handle];
    auto& root_node = nodes_[node_handle];
    root_node.flow_handle_ = flow_handle;
    root_flow.nodes_.emplace_back(node_handle);
    base_node_list_.emplace_back(node_handle);
  }
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::DetachChildNode(SkeletonNodeHandle target_handle,
                                                                 SkeletonNodeHandle child_handle) {
  assert(target_handle >= 0 && child_handle >= 0 && target_handle < nodes_.size() && child_handle < nodes_.size());
  auto& target_node = nodes_[target_handle];
  auto& child_node = nodes_[child_handle];
  assert(!target_node.recycled_);
  assert(!child_node.recycled_);
  auto& children = target_node.child_handles_;
  for (int i = 0; i < children.size(); i++) {
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

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::RefreshBaseNodeList() {
  std::vector<SkeletonNodeHandle> temp;
  for (const auto& i : base_node_list_)
    if (!nodes_[i].recycled_ && nodes_[i].parent_handle_ == -1)
      temp.emplace_back(i);
  base_node_list_ = temp;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::CalculateClusters(const SkeletonClusterSettings& cluster_settings) {
}

template <typename SkeletonData, typename FlowData, typename NodeData>
template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::Clone(
    const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  data = {};
  flow_pool_ = src_skeleton.flow_pool_;
  node_pool_ = src_skeleton.node_pool_;
  sorted_node_list_ = src_skeleton.sorted_node_list_;
  sorted_flow_list_ = src_skeleton.sorted_flow_list_;

  nodes_.resize(src_skeleton.nodes_.size());
  for (int i = 0; i < src_skeleton.nodes_.size(); i++) {
    nodes_[i].info = src_skeleton.nodes_[i].info;

    nodes_[i].end_node_ = src_skeleton.nodes_[i].end_node_;
    nodes_[i].recycled_ = src_skeleton.nodes_[i].recycled_;
    nodes_[i].handle_ = src_skeleton.nodes_[i].handle_;
    nodes_[i].flow_handle_ = src_skeleton.nodes_[i].flow_handle_;
    nodes_[i].parent_handle_ = src_skeleton.nodes_[i].parent_handle_;
    nodes_[i].child_handles_ = src_skeleton.nodes_[i].child_handles_;
    nodes_[i].apical_ = src_skeleton.nodes_[i].apical_;
    nodes_[i].index_ = src_skeleton.nodes_[i].index_;
  }

  flows_.resize(src_skeleton.flows_.size());
  for (int i = 0; i < src_skeleton.flows_.size(); i++) {
    flows_[i].info = src_skeleton.flows_[i].info;

    flows_[i].recycled_ = src_skeleton.flows_[i].recycled_;
    flows_[i].handle_ = src_skeleton.flows_[i].handle_;
    flows_[i].nodes_ = src_skeleton.flows_[i].nodes_;
    flows_[i].parent_handle_ = src_skeleton.flows_[i].parent_handle_;
    flows_[i].apical_ = src_skeleton.flows_[i].apical_;
  }
  max_node_index_ = src_skeleton.max_node_index_;
  max_flow_index_ = src_skeleton.max_flow_index_;
  new_version_ = src_skeleton.new_version_;
  version_ = src_skeleton.version_;
  min = src_skeleton.min;
  max = src_skeleton.max;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
int Skeleton<SkeletonData, FlowData, NodeData>::GetMaxNodeIndex() const {
  return max_node_index_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
int Skeleton<SkeletonData, FlowData, NodeData>::GetMaxFlowIndex() const {
  return max_flow_index_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::CalculateDistance() {
  for (const auto& node_handle : sorted_node_list_) {
    auto& node = nodes_[node_handle];
    auto& node_info = node.info;
    if (node.GetParentHandle() == -1) {
      node_info.root_distance = node_info.length;
      node_info.chain_index = 0;
    } else {
      const auto& parent_internode = nodes_[node.GetParentHandle()];
      node_info.root_distance = parent_internode.info.root_distance + node_info.length;

      if (node.IsApical()) {
        node.info.chain_index = parent_internode.info.chain_index + 1;
      } else {
        node.info.chain_index = 0;
      }
    }
  }
  for (auto it = sorted_node_list_.rbegin(); it != sorted_node_list_.rend(); ++it) {
    auto& node = nodes_[*it];
    float max_distance_to_any_branch_end = 0;
    node.info.end_distance = 0;
    for (const auto& i : node.PeekChildHandles()) {
      const auto& child_node = nodes_[i];
      const float child_max_distance_to_any_branch_end = child_node.info.end_distance + child_node.info.length;
      max_distance_to_any_branch_end = glm::max(max_distance_to_any_branch_end, child_max_distance_to_any_branch_end);
    }
    node.info.end_distance = max_distance_to_any_branch_end;
  }
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::CalculateRegulatedGlobalRotation() {
  min = glm::vec3(FLT_MAX);
  max = glm::vec3(-FLT_MAX);
  for (const auto& node_handle : sorted_node_list_) {
    auto& node = nodes_[node_handle];
    auto& node_info = node.info;
    min = glm::min(min, node.info.global_position);
    min = glm::min(min, node.info.GetGlobalEndPosition());
    max = glm::max(max, node.info.global_position);
    max = glm::max(max, node.info.GetGlobalEndPosition());
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

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::SetParentNode(SkeletonNodeHandle target_handle,
                                                               SkeletonNodeHandle parent_handle) {
  assert(target_handle >= 0 && parent_handle >= 0 && target_handle < nodes_.size() && parent_handle < nodes_.size());
  auto& target_node = nodes_[target_handle];
  auto& parent_node = nodes_[parent_handle];
  assert(!target_node.recycled_);
  assert(!parent_node.recycled_);
  target_node.parent_handle_ = parent_handle;
  parent_node.child_handles_.emplace_back(target_handle);
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::DetachChildFlow(SkeletonFlowHandle target_handle,
                                                                 SkeletonFlowHandle child_handle) {
  assert(target_handle >= 0 && child_handle >= 0 && target_handle < flows_.size() && child_handle < flows_.size());
  auto& target_branch = flows_[target_handle];
  auto& child_branch = flows_[child_handle];
  assert(!target_branch.recycled_);
  assert(!child_branch.recycled_);

  if (!child_branch.nodes_.empty()) {
    auto first_node_handle = child_branch.nodes_[0];
    if (auto& first_node = nodes_[first_node_handle]; first_node.parent_handle_ != -1)
      DetachChildNode(first_node.parent_handle_, first_node_handle);
  }

  auto& children = target_branch.child_handles_;
  for (int i = 0; i < children.size(); i++) {
    if (children[i] == child_handle) {
      children[i] = children.back();
      children.pop_back();
      child_branch.parent_handle_ = -1;
      return;
    }
  }
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::SetParentFlow(SkeletonFlowHandle target_handle,
                                                               SkeletonFlowHandle parent_handle) {
  assert(target_handle >= 0 && parent_handle >= 0 && target_handle < flows_.size() && parent_handle < flows_.size());
  auto& target_branch = flows_[target_handle];
  auto& parent_branch = flows_[parent_handle];
  assert(!target_branch.recycled_);
  assert(!parent_branch.recycled_);
  target_branch.parent_handle_ = parent_handle;
  parent_branch.child_handles_.emplace_back(target_handle);
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::RecycleFlowSingle(
    SkeletonFlowHandle handle, const std::function<void(SkeletonFlowHandle)>& flow_handler) {
  assert(!flows_[handle].recycled_);
  auto& flow = flows_[handle];
  flow_handler(handle);
  flow.parent_handle_ = -1;
  flow.child_handles_.clear();
  flow.nodes_.clear();

  flow.data = {};
  flow.info = {};

  flow.recycled_ = true;
  flow.apical_ = false;
  flow_pool_.emplace(handle);
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::RecycleNodeSingle(
    SkeletonNodeHandle handle, const std::function<void(SkeletonNodeHandle)>& node_handler) {
  assert(!nodes_[handle].recycled_);
  auto& node = nodes_[handle];
  node_handler(handle);
  node.parent_handle_ = -1;
  node.flow_handle_ = -1;
  node.end_node_ = true;
  node.child_handles_.clear();

  node.data = {};
  node.info = {};
  node.info.locked = false;
  node.info.wounds.clear();

  node.recycled_ = true;
  node_pool_.emplace(handle);
}

template <typename SkeletonData, typename FlowData, typename NodeData>
SkeletonFlowHandle Skeleton<SkeletonData, FlowData, NodeData>::AllocateFlow() {
  max_flow_index_++;
  if (flow_pool_.empty()) {
    flows_.emplace_back(flows_.size());
    flows_.back().index_ = max_flow_index_;
    return flows_.back().handle_;
  }
  auto handle = flow_pool_.front();
  flow_pool_.pop();
  auto& flow = flows_[handle];
  flow.recycled_ = false;
  flow.index_ = max_flow_index_;
  return handle;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
SkeletonNodeHandle Skeleton<SkeletonData, FlowData, NodeData>::AllocateNode() {
  max_node_index_++;
  if (node_pool_.empty()) {
    nodes_.emplace_back(nodes_.size());
    nodes_.back().index_ = max_node_index_;
    return nodes_.back().handle_;
  }
  auto handle = node_pool_.front();
  node_pool_.pop();
  auto& node = nodes_[handle];
  node.recycled_ = false;
  node.index_ = max_node_index_;
  return handle;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
int Skeleton<SkeletonData, FlowData, NodeData>::GetVersion() const {
  return version_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void Skeleton<SkeletonData, FlowData, NodeData>::CalculateFlows() {
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

template <typename SkeletonData, typename FlowData, typename NodeData>
std::vector<SkeletonFlow<FlowData>>& Skeleton<SkeletonData, FlowData, NodeData>::RefRawFlows() {
  return flows_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
std::vector<SkeletonNode<NodeData>>& Skeleton<SkeletonData, FlowData, NodeData>::RefRawNodes() {
  return nodes_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
const std::vector<SkeletonFlow<FlowData>>& Skeleton<SkeletonData, FlowData, NodeData>::PeekRawFlows() const {
  return flows_;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
const std::vector<SkeletonNode<NodeData>>& Skeleton<SkeletonData, FlowData, NodeData>::PeekRawNodes() const {
  return nodes_;
}

#pragma endregion
#pragma endregion
}  // namespace eco_sys_lab