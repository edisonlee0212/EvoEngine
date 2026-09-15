#pragma once
#include "Serialization.hpp"
namespace evo_engine {
/** @brief Handle type for identifying a node in the node graph. */
typedef int NodeGraphNodeHandle;

/** @brief Handle type for identifying an input pin in the node graph. */
typedef int NodeGraphInputPinHandle;

/** @brief Handle type for identifying an output pin in the node graph. */
typedef int NodeGraphOutputPinHandle;

/** @brief Handle type for identifying a link between nodes in the node graph. */
typedef int NodeGraphLinkHandle;

/**
 * @brief Represents an input pin of a node in the node graph.
 * @tparam NodeGraphInputPinData Data type associated with the input pin.
 */
template <typename NodeGraphInputPinData>
class NodeGraphInputPin {
  template <typename Id, typename Od, typename Nd, typename Ld>
  friend class NodeGraph;

  NodeGraphNodeHandle node_handle_ = -1; /**< Handle to the node this input pin belongs to. */
  bool recycled_ = false;                /**< Flag indicating if this pin has been recycled. */
  NodeGraphInputPinHandle handle_ = -1;  /**< Handle identifying this input pin uniquely. */
  NodeGraphLinkHandle link_handle_ = -1; /**< Handle to the link connected to this input pin, if any. */

 public:
  NodeGraphInputPinData data{}; /**< Data associated with this input pin. */

  /** @brief Default constructor. */
  NodeGraphInputPin() = default;

  /**
   * @brief Constructs an input pin with a given handle and associated node.
   * @param handle The handle associated with this input pin.
   * @param node_handle The handle of the node this input pin belongs to.
   */
  NodeGraphInputPin(NodeGraphInputPinHandle handle, NodeGraphNodeHandle node_handle);

  /**
   * \brief Return handle to the node this input pin belongs to.
   * \return Handle to the node this input pin belongs to.
   */
  NodeGraphNodeHandle GetNodeHandle() const;

  /**
   * \brief Get handle of this input pin.
   * \return Handle of this input pin.
   */
  NodeGraphInputPinHandle GetHandle() const;
  /**
   * \brief Get handle of connected link.
   * \return Handle of the link, -1 if no connected link.
   */
  NodeGraphLinkHandle GetLinkHandle() const;
  /**
   * \brief Return flag indicating if this pin has been recycled.
   * \return Flag indicating if this pin has been recycled.
   */
  bool Recycled() const;
};

/**
 * @brief Represents an output pin of a node in the node graph.
 * @tparam NodeGraphOutputPinData Data type associated with the output pin.
 */
template <typename NodeGraphOutputPinData>
class NodeGraphOutputPin {
  template <typename Id, typename Od, typename Nd, typename Ld>
  friend class NodeGraph;

  NodeGraphNodeHandle node_handle_ = -1;            /**< Handle to the node this output pin belongs to. */
  bool recycled_ = false;                           /**< Flag indicating if this pin has been recycled. */
  NodeGraphOutputPinHandle handle_ = -1;            /**< Handle identifying this output pin uniquely. */
  std::vector<NodeGraphLinkHandle> link_handles_{}; /**< Handles to links connected to this output pin. */

 public:
  NodeGraphOutputPinData data{}; /**< Data associated with this output pin. */

  /** @brief Default constructor. */
  NodeGraphOutputPin() = default;

  /**
   * @brief Constructs an output pin with a given handle and associated node.
   * @param handle The handle associated with this output pin.
   * @param node_handle The handle of the node this output pin belongs to.
   */
  NodeGraphOutputPin(NodeGraphOutputPinHandle handle, NodeGraphNodeHandle node_handle);

  /**
   * \brief Return handle to the node this output pin belongs to.
   * \return Handle to the node this output pin belongs to.
   */
  NodeGraphNodeHandle GetNodeHandle() const;

  /**
   * \brief Get handle of this output pin.
   * \return Handle of this output pin.
   */
  NodeGraphOutputPinHandle GetHandle() const;
  /**
   * \brief Get handle of connected link.
   * \return Handle of the link, -1 if no connected link.
   */
  const std::vector<NodeGraphLinkHandle>& GetLinkHandles() const;
  /**
   * \brief Return flag indicating if this pin has been recycled.
   * \return Flag indicating if this pin has been recycled.
   */
  bool Recycled() const;
};

/**
 * @brief Represents a node in the node graph.
 * @tparam NodeGraphNodeData Data type associated with the node.
 */
template <typename NodeGraphNodeData>
class NodeGraphNode {
  template <typename Id, typename Od, typename Nd, typename Ld>
  friend class NodeGraph;
  NodeGraphNodeHandle handle_ = -1; /**< Unique handle identifying this node. */
  bool recycled_ = false;           /**< Flag indicating whether this node has been recycled. */

  std::vector<NodeGraphInputPinHandle> input_pin_handles_; /**< Handles of input pins attached to this node. */
  std::vector<NodeGraphOutputPinHandle>
      output_pin_handles_; /**< Handles of the output pin attached to this node (if any). */

 public:
  NodeGraphNodeData data{}; /**< Data associated with this node. */

  /** @brief Default constructor. */
  NodeGraphNode() = default;

  /**
   * @brief Constructs a node with a given handle.
   * @param handle The handle associated with this node.
   */
  NodeGraphNode(NodeGraphNodeHandle handle);

  /**
   * \brief Get handle of this node.
   * \return Handle of this node.
   */
  NodeGraphNodeHandle GetHandle() const;

  /**
   * \brief Return flag indicating if this node has been recycled.
   * \return Flag indicating if this node has been recycled.
   */
  bool Recycled() const;
  /**
   * \brief Get handles of input pins attached to this node.
   * \return Handles of input pins attached to this node.
   */
  const std::vector<NodeGraphInputPinHandle>& GetInputPinHandles() const;
  /**
   * \brief Get handles of output pins attached to this node.
   * \return Handles of output pins attached to this node.
   */
  const std::vector<NodeGraphOutputPinHandle>& GetOutputPinHandles() const;
};

/**
 * @brief Represents a link between a node graph input pin and output pin.
 * @tparam NodeGraphLinkData Data type associated with the link.
 */
template <typename NodeGraphLinkData>
class NodeGraphLink {
  NodeGraphOutputPinHandle start_ = -1; /**< Handle to the output pin where this link starts. */
  NodeGraphInputPinHandle end_ = -1;    /**< Handle to the input pin where this link ends. */
  NodeGraphLinkHandle handle_ = -1;     /**< Unique handle identifying this link. */
  bool recycled_ = false;               /**< Flag indicating whether this link has been recycled. */

  template <typename Id, typename Od, typename Nd, typename Ld>
  friend class NodeGraph;

 public:
  NodeGraphLinkData data{}; /**< Data associated with this link. */

  /** @brief Default constructor. */
  NodeGraphLink() = default;

  /**
   * @brief Constructs a link between an output pin and an input pin.
   * @param handle Unique handle for this link.
   * @param output_pin_handle Handle to the output pin where this link starts.
   * @param input_pin_handle Handle to the input pin where this link ends.
   */
  NodeGraphLink(NodeGraphLinkHandle handle, NodeGraphOutputPinHandle output_pin_handle,
                NodeGraphInputPinHandle input_pin_handle);
  /**
   * \brief Get output pin handle.
   * \return Start of this link (handle to an output pin).
   */
  NodeGraphOutputPinHandle GetOutputPinHandle() const;
  /**
   * \brief Get input pin handle.
   * \return End of this link (handle to an input pin).
   */
  NodeGraphInputPinHandle GetInputPinHandle() const;

  /**
   * \brief Get handle of this link.
   * \return Handle of this link.
   */
  NodeGraphLinkHandle GetHandle() const;

  /**
   * \brief Return flag indicating if this link has been recycled.
   * \return Flag indicating if this link has been recycled.
   */
  bool Recycled() const;
};

/**
 * @brief Represents a node graph structure containing nodes, input pins, output pins, and links.
 * @tparam Id Input pin data type.
 * @tparam Od Output pin data type.
 * @tparam Nd Node data type.
 * @tparam Ld Link data type.
 */
template <typename Id, typename Od, typename Nd, typename Ld>
class NodeGraph {
  std::vector<NodeGraphInputPin<Id>> input_pins_;   /**< List of all input pins in the graph. */
  std::vector<NodeGraphOutputPin<Od>> output_pins_; /**< List of all output pins in the graph. */
  std::vector<NodeGraphNode<Nd>> nodes_;            /**< List of all nodes in the graph. */
  std::vector<NodeGraphLink<Ld>> links_;            /**< List of all links in the graph. */

  std::queue<NodeGraphInputPinHandle> input_pin_pool_;   /**< Pool of recycled input pins. */
  std::queue<NodeGraphOutputPinHandle> output_pin_pool_; /**< Pool of recycled output pins. */
  std::queue<NodeGraphNodeHandle> node_pool_;            /**< Pool of recycled nodes. */
  std::queue<NodeGraphLinkHandle> link_pool_;            /**< Pool of recycled links. */

  /**
   * @brief Allocates an input pin for the given node.
   * @param node_handle Handle for the node requesting an input pin.
   * @return Handle to the allocated input pin.
   */
  NodeGraphInputPinHandle AllocateInputPin(NodeGraphNodeHandle node_handle);

  /**
   * @brief Allocates an output pin for the given node.
   * @param node_handle Handle for the node requesting an output pin.
   * @return Handle to the allocated output pin.
   */
  NodeGraphOutputPinHandle AllocateOutputPin(NodeGraphNodeHandle node_handle);

  /**
   * @brief Recycles an output pin back into the pool.
   * @param handle Handle of the output pin to recycle.
   */
  void RecycleOutputPin(NodeGraphOutputPinHandle handle);

  /**
   * @brief Recycles an input pin back into the pool.
   * @param handle Handle of the input pin to recycle.
   */
  void RecycleInputPin(NodeGraphInputPinHandle handle);
  glm::vec2 editor_panning_{};
  glm::vec2 editor_auto_panning_delta_{};
  std::vector<glm::vec2> editor_node_positions_{};

 public:
  [[nodiscard]] glm::vec2 GetPanning() const {
    return editor_panning_;
  }
  void SetPanning(const glm::vec2& value) {
    editor_panning_ = value;
  }
  [[nodiscard]] glm::vec2 GetAutoPanningDelta() const {
    return editor_auto_panning_delta_;
  }
  void SetAutoPanningDelta(const glm::vec2& value) {
    editor_auto_panning_delta_ = value;
  }
  [[nodiscard]] glm::vec2 GetNodePosition(NodeGraphNodeHandle handle) const {
    return editor_node_positions_.at(static_cast<size_t>(handle));
  }
  void SetNodePosition(NodeGraphNodeHandle handle, const glm::vec2& position) {
    editor_node_positions_.at(static_cast<size_t>(handle)) = position;
  }

  /**
   * \brief Access all input pins.
   * \return Const reference to all input pins.
   */
  const std::vector<NodeGraphInputPin<Id>>& PeekInputPins() const;
  /**
   * \brief Access all output pins.
   * \return Const reference to all output pins.
   */
  const std::vector<NodeGraphOutputPin<Od>>& PeekOutputPins() const;
  /**
   * \brief Access all nodes.
   * \return Const reference to all nodes.
   */
  const std::vector<NodeGraphNode<Nd>>& PeekNodes() const;
  /**
   * \brief Access all links.
   * \return Const reference to all links.
   */
  const std::vector<NodeGraphLink<Ld>>& PeekLinks() const;

  /**
   * \brief Access all input pins.
   * \return Reference to all input pins.
   */
  std::vector<NodeGraphInputPin<Id>>& RefInputPins();
  /**
   * \brief Access all output pins.
   * \return Reference to all output pins.
   */
  std::vector<NodeGraphOutputPin<Od>>& RefOutputPins();
  /**
   * \brief Access all nodes.
   * \return Reference to all nodes.
   */
  std::vector<NodeGraphNode<Nd>>& RefNodes();
  /**
   * \brief Access all links.
   * \return Reference to all links.
   */
  std::vector<NodeGraphLink<Ld>>& RefLinks();

  /**
   * @brief Allocates a link between an output pin and an input pin.
   * @param start_handle Handle for the starting output pin.
   * @param end_handle Handle for the ending input pin.
   * @return Handle to the allocated link.
   */
  NodeGraphLinkHandle AllocateLink(NodeGraphOutputPinHandle start_handle, NodeGraphInputPinHandle end_handle);

  /**
   * @brief Recycles a link and returns it back to the pool.
   * @param handle Handle of the link to recycle.
   */
  void RecycleLink(NodeGraphLinkHandle handle);

  /**
   * @brief Allocates a node with the specified number of input pins and determines if it has an output pin.
   * @param input_pin_count Number of input pins the node should have.
   * @param output_pin_count Number of output pins the node should have.
   * @return Handle to the allocated node.
   */
  NodeGraphNodeHandle AllocateNode(size_t input_pin_count, size_t output_pin_count);

  /**
   * @brief Get reference to a node.
   * @param node_handle The handle of target node.
   * @return Reference to target node.
   */
  NodeGraphNode<Nd>& RefNode(NodeGraphNodeHandle node_handle);

  /**
   * @brief Get const reference to a node.
   * @param node_handle The handle of target node.
   * @return Const reference to target node.
   */
  const NodeGraphNode<Nd>& PeekNode(NodeGraphNodeHandle node_handle) const;
  /**
   * @brief Recycles a node and returns it back to the pool.
   * @param handle Handle of the node to recycle.
   */
  void RecycleNode(NodeGraphNodeHandle handle);

  /**
   * @brief Get const reference to a input pin.
   * @param input_pin_handle The handle of input pin.
   * @return Const reference to target input pin.
   */
  const NodeGraphInputPin<Id>& PeekInputPin(NodeGraphInputPinHandle input_pin_handle) const;
  /**
   * @brief Get reference to a input pin.
   * @param input_pin_handle The handle of input pin.
   * @return Reference to target input pin.
   */
  NodeGraphInputPin<Id>& RefInputPin(NodeGraphInputPinHandle input_pin_handle);

  /**
   * @brief Get const reference to a output pin.
   * @param output_pin_handle The handle of output pin.
   * @return Const reference to target output pin.
   */
  const NodeGraphOutputPin<Od>& PeekOutputPin(NodeGraphOutputPinHandle output_pin_handle) const;
  /**
   * @brief Get reference to a output pin.
   * @param output_pin_handle The handle of output pin.
   * @return Reference to target output pin.
   */
  NodeGraphOutputPin<Od>& RefOutputPin(NodeGraphOutputPinHandle output_pin_handle);
  /**
   * @brief Get const reference to a link.
   * @param link_handle The handle of link.
   * @return Const reference to target link.
   */
  const NodeGraphLink<Ld>& PeekLink(NodeGraphLinkHandle link_handle) const;
  /**
   * @brief Get reference to a link.
   * @param link_handle The handle of link.
   * @return Reference to target link.
   */
  NodeGraphLink<Ld>& RefLink(NodeGraphLinkHandle link_handle);

  void Serialize(YAML::Emitter& out, const std::function<void(YAML::Emitter&, const Id&)>& input_pin_func,
                 const std::function<void(YAML::Emitter&, const Od&)>& output_pin_func,
                 const std::function<void(YAML::Emitter&, const Nd&)>& node_func,
                 const std::function<void(YAML::Emitter&, const Ld&)>& link_func) const;
  void Deserialize(const YAML::Node& in, const std::function<void(const YAML::Node&, Id&)>& input_pin_func,
                   const std::function<void(const YAML::Node&, Od&)>& output_pin_func,
                   const std::function<void(const YAML::Node&, Nd&)>& node_func,
                   const std::function<void(const YAML::Node&, Ld&)>& link_func);

  void Save(const std::string& name, YAML::Emitter& out,
            const std::function<void(YAML::Emitter&, const Id&)>& input_pin_func,
            const std::function<void(YAML::Emitter&, const Od&)>& output_pin_func,
            const std::function<void(YAML::Emitter&, const Nd&)>& node_func,
            const std::function<void(YAML::Emitter&, const Ld&)>& link_func) const;
  void Load(const std::string& name, const YAML::Node& in,
            const std::function<void(const YAML::Node&, Id&)>& input_pin_func,
            const std::function<void(const YAML::Node&, Od&)>& output_pin_func,
            const std::function<void(const YAML::Node&, Nd&)>& node_func,
            const std::function<void(const YAML::Node&, Ld&)>& link_func);
};

template <typename NodeGraphInputPinData>
NodeGraphInputPin<NodeGraphInputPinData>::NodeGraphInputPin(const NodeGraphInputPinHandle handle,
                                                            const NodeGraphNodeHandle node_handle) {
  node_handle_ = node_handle;
  link_handle_ = -1;
  handle_ = handle;
  recycled_ = false;
  data = {};
}
template <typename NodeGraphInputPinData>
NodeGraphNodeHandle NodeGraphInputPin<NodeGraphInputPinData>::GetNodeHandle() const {
  return node_handle_;
}
template <typename NodeGraphInputPinData>
NodeGraphInputPinHandle NodeGraphInputPin<NodeGraphInputPinData>::GetHandle() const {
  return handle_;
}
template <typename NodeGraphInputPinData>
NodeGraphLinkHandle NodeGraphInputPin<NodeGraphInputPinData>::GetLinkHandle() const {
  return link_handle_;
}
template <typename NodeGraphInputPinData>
bool NodeGraphInputPin<NodeGraphInputPinData>::Recycled() const {
  return recycled_;
}

template <typename NodeGraphOutputPinData>
NodeGraphOutputPin<NodeGraphOutputPinData>::NodeGraphOutputPin(const NodeGraphOutputPinHandle handle,
                                                               const NodeGraphNodeHandle node_handle) {
  node_handle_ = node_handle;
  link_handles_.clear();
  handle_ = handle;
  recycled_ = false;
  data = {};
}
template <typename NodeGraphOutputPinData>
NodeGraphNodeHandle NodeGraphOutputPin<NodeGraphOutputPinData>::GetNodeHandle() const {
  return node_handle_;
}
template <typename NodeGraphOutputPinData>
NodeGraphOutputPinHandle NodeGraphOutputPin<NodeGraphOutputPinData>::GetHandle() const {
  return handle_;
}
template <typename NodeGraphOutputPinData>
const std::vector<NodeGraphLinkHandle>& NodeGraphOutputPin<NodeGraphOutputPinData>::GetLinkHandles() const {
  return link_handles_;
}
template <typename NodeGraphOutputPinData>
bool NodeGraphOutputPin<NodeGraphOutputPinData>::Recycled() const {
  return recycled_;
}

template <typename NodeGraphNodeData>
NodeGraphNode<NodeGraphNodeData>::NodeGraphNode(const NodeGraphNodeHandle handle) {
  input_pin_handles_.clear();
  output_pin_handles_.clear();
  handle_ = handle;
  recycled_ = false;
  data = {};
}
template <typename NodeGraphNodeData>
NodeGraphNodeHandle NodeGraphNode<NodeGraphNodeData>::GetHandle() const {
  return handle_;
}
template <typename NodeGraphNodeData>
bool NodeGraphNode<NodeGraphNodeData>::Recycled() const {
  return recycled_;
}
template <typename NodeGraphNodeData>
const std::vector<NodeGraphInputPinHandle>& NodeGraphNode<NodeGraphNodeData>::GetInputPinHandles() const {
  return input_pin_handles_;
}
template <typename NodeGraphNodeData>
const std::vector<NodeGraphOutputPinHandle>& NodeGraphNode<NodeGraphNodeData>::GetOutputPinHandles() const {
  return output_pin_handles_;
}

template <typename NodeGraphLinkData>
NodeGraphLink<NodeGraphLinkData>::NodeGraphLink(const NodeGraphLinkHandle handle,
                                                const NodeGraphOutputPinHandle output_pin_handle,
                                                const NodeGraphInputPinHandle input_pin_handle) {
  handle_ = handle;
  recycled_ = false;
  data = {};

  start_ = output_pin_handle;
  end_ = input_pin_handle;
}
template <typename NodeGraphLinkData>
NodeGraphOutputPinHandle NodeGraphLink<NodeGraphLinkData>::GetOutputPinHandle() const {
  return start_;
}
template <typename NodeGraphLinkData>
NodeGraphInputPinHandle NodeGraphLink<NodeGraphLinkData>::GetInputPinHandle() const {
  return end_;
}
template <typename NodeGraphLinkData>
NodeGraphLinkHandle NodeGraphLink<NodeGraphLinkData>::GetHandle() const {
  return handle_;
}
template <typename NodeGraphLinkData>
bool NodeGraphLink<NodeGraphLinkData>::Recycled() const {
  return recycled_;
}

template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphInputPinHandle NodeGraph<Id, Od, Nd, Ld>::AllocateInputPin(const NodeGraphNodeHandle node_handle) {
  NodeGraphInputPinHandle ret_val;
  if (input_pin_pool_.empty()) {
    input_pins_.emplace_back(static_cast<NodeGraphInputPinHandle>(input_pins_.size()), node_handle);
    ret_val = input_pins_.back().handle_;
  } else {
    ret_val = input_pin_pool_.front();
    input_pin_pool_.pop();
  }
  auto& input_pin = input_pins_[ret_val];
  input_pin.link_handle_ = -1;
  input_pin.node_handle_ = node_handle;
  input_pin.recycled_ = false;
  return ret_val;
}

template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphOutputPinHandle NodeGraph<Id, Od, Nd, Ld>::AllocateOutputPin(const NodeGraphNodeHandle node_handle) {
  NodeGraphOutputPinHandle ret_val;
  if (output_pin_pool_.empty()) {
    output_pins_.emplace_back(static_cast<NodeGraphOutputPinHandle>(output_pins_.size()), node_handle);
    ret_val = output_pins_.back().handle_;
  } else {
    ret_val = output_pin_pool_.front();
    output_pin_pool_.pop();
  }
  auto& output_pin = output_pins_[ret_val];
  output_pin.link_handles_.clear();
  output_pin.node_handle_ = node_handle;
  output_pin.recycled_ = false;
  return ret_val;
}

template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphLinkHandle NodeGraph<Id, Od, Nd, Ld>::AllocateLink(const NodeGraphOutputPinHandle start_handle,
                                                            const NodeGraphInputPinHandle end_handle) {
  assert(input_pins_[end_handle].link_handle_ == -1);
  NodeGraphLinkHandle ret_val;
  if (link_pool_.empty()) {
    links_.emplace_back(static_cast<NodeGraphLinkHandle>(links_.size()), start_handle, end_handle);
    ret_val = links_.back().handle_;
  } else {
    ret_val = link_pool_.front();
    link_pool_.pop();
  }
  auto& link = links_[ret_val];
  link.start_ = start_handle;
  link.end_ = end_handle;

  auto& output_link_start_handles = output_pins_[start_handle].link_handles_;
  output_link_start_handles.emplace_back(ret_val);

  input_pins_[end_handle].link_handle_ = ret_val;

  link.recycled_ = false;
  return ret_val;
}

template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::RecycleLink(const NodeGraphLinkHandle handle) {
  assert(!links_[handle].recycled_);
  auto& link = links_[handle];
  link.data = {};
  auto& output_pin_link_handles = output_pins_[link.start_].link_handles_;
  for (size_t i = 0; i < output_pin_link_handles.size(); i++) {
    if (output_pin_link_handles[i] == handle) {
      output_pin_link_handles[i] = output_pin_link_handles.back();
      output_pin_link_handles.pop_back();
      break;
    }
  }
  input_pins_[link.end_].link_handle_ = -1;
  link.recycled_ = true;
  link_pool_.emplace(handle);
}

template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::RecycleOutputPin(NodeGraphOutputPinHandle handle) {
  assert(!output_pins_[handle].recycled_);
  auto& output_pin = output_pins_[handle];
  output_pin.data = {};

  auto link_handles = output_pin.link_handles_;
  for (const auto& i : link_handles)
    RecycleLink(i);
  assert(output_pin.link_handles_.empty());
  output_pin.recycled_ = true;
  output_pin_pool_.emplace(handle);
}

template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::RecycleInputPin(NodeGraphInputPinHandle handle) {
  assert(!input_pins_[handle].recycled_);
  auto& input_pin = input_pins_[handle];
  input_pin.data = {};
  if (input_pin.link_handle_ != -1) {
    RecycleLink(input_pin.link_handle_);
    input_pin.link_handle_ = -1;
  }
  input_pin.recycled_ = true;
  input_pin_pool_.emplace(handle);
}
template <typename Id, typename Od, typename Nd, typename Ld>
const std::vector<NodeGraphOutputPin<Od>>& NodeGraph<Id, Od, Nd, Ld>::PeekOutputPins() const {
  return output_pins_;
}
template <typename Id, typename Od, typename Nd, typename Ld>
const std::vector<NodeGraphInputPin<Id>>& NodeGraph<Id, Od, Nd, Ld>::PeekInputPins() const {
  return input_pins_;
}
template <typename Id, typename Od, typename Nd, typename Ld>
const std::vector<NodeGraphNode<Nd>>& NodeGraph<Id, Od, Nd, Ld>::PeekNodes() const {
  return nodes_;
}
template <typename Id, typename Od, typename Nd, typename Ld>
const std::vector<NodeGraphLink<Ld>>& NodeGraph<Id, Od, Nd, Ld>::PeekLinks() const {
  return links_;
}
template <typename Id, typename Od, typename Nd, typename Ld>
std::vector<NodeGraphInputPin<Id>>& NodeGraph<Id, Od, Nd, Ld>::RefInputPins() {
  return input_pins_;
}
template <typename Id, typename Od, typename Nd, typename Ld>
std::vector<NodeGraphOutputPin<Od>>& NodeGraph<Id, Od, Nd, Ld>::RefOutputPins() {
  return output_pins_;
}
template <typename Id, typename Od, typename Nd, typename Ld>
std::vector<NodeGraphNode<Nd>>& NodeGraph<Id, Od, Nd, Ld>::RefNodes() {
  return nodes_;
}
template <typename Id, typename Od, typename Nd, typename Ld>
std::vector<NodeGraphLink<Ld>>& NodeGraph<Id, Od, Nd, Ld>::RefLinks() {
  return links_;
}

template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphNodeHandle NodeGraph<Id, Od, Nd, Ld>::AllocateNode(const size_t input_pin_count,
                                                            const size_t output_pin_count) {
  NodeGraphNodeHandle new_node_handle;
  if (node_pool_.empty()) {
    nodes_.emplace_back(static_cast<NodeGraphNodeHandle>(nodes_.size()));
    editor_node_positions_.emplace_back();
    new_node_handle = nodes_.back().handle_;
  } else {
    new_node_handle = node_pool_.front();
    node_pool_.pop();
    editor_node_positions_[new_node_handle] = {};
  }
  auto& node = nodes_[new_node_handle];
  node.data = {};
  node.input_pin_handles_.clear();
  node.output_pin_handles_.clear();
  for (size_t i = 0; i < input_pin_count; i++) {
    node.input_pin_handles_.emplace_back(AllocateInputPin(new_node_handle));
  }
  for (size_t i = 0; i < output_pin_count; i++) {
    node.output_pin_handles_.emplace_back(AllocateOutputPin(new_node_handle));
  }
  node.recycled_ = false;
  return new_node_handle;
}
template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphNode<Nd>& NodeGraph<Id, Od, Nd, Ld>::RefNode(NodeGraphNodeHandle node_handle) {
  assert(!nodes_[node_handle].recycled_);
  return nodes_[node_handle];
}
template <typename Id, typename Od, typename Nd, typename Ld>
const NodeGraphNode<Nd>& NodeGraph<Id, Od, Nd, Ld>::PeekNode(NodeGraphNodeHandle node_handle) const {
  assert(!nodes_[node_handle].recycled_);
  return nodes_[node_handle];
}

template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::RecycleNode(const NodeGraphNodeHandle handle) {
  assert(!nodes_[handle].recycled_);
  auto& node = nodes_[handle];
  node.data = {};
  editor_node_positions_[handle] = {};
  for (const auto& i : node.input_pin_handles_) {
    RecycleInputPin(i);
  }
  for (const auto& i : node.output_pin_handles_) {
    RecycleOutputPin(i);
  }
  node.recycled_ = true;
  node_pool_.emplace(handle);
}
template <typename Id, typename Od, typename Nd, typename Ld>
const NodeGraphInputPin<Id>& NodeGraph<Id, Od, Nd, Ld>::PeekInputPin(NodeGraphInputPinHandle input_pin_handle) const {
  return input_pins_[input_pin_handle];
}
template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphInputPin<Id>& NodeGraph<Id, Od, Nd, Ld>::RefInputPin(NodeGraphInputPinHandle input_pin_handle) {
  return input_pins_[input_pin_handle];
}
template <typename Id, typename Od, typename Nd, typename Ld>
const NodeGraphOutputPin<Od>& NodeGraph<Id, Od, Nd, Ld>::PeekOutputPin(
    NodeGraphOutputPinHandle output_pin_handle) const {
  return output_pins_[output_pin_handle];
}
template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphOutputPin<Od>& NodeGraph<Id, Od, Nd, Ld>::RefOutputPin(NodeGraphOutputPinHandle output_pin_handle) {
  return output_pins_[output_pin_handle];
}
template <typename Id, typename Od, typename Nd, typename Ld>
const NodeGraphLink<Ld>& NodeGraph<Id, Od, Nd, Ld>::PeekLink(NodeGraphLinkHandle link_handle) const {
  return links_[link_handle];
}
template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphLink<Ld>& NodeGraph<Id, Od, Nd, Ld>::RefLink(NodeGraphLinkHandle link_handle) {
  return links_[link_handle];
}

template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::Serialize(YAML::Emitter& out,
                                          const std::function<void(YAML::Emitter&, const Id&)>& input_pin_func,
                                          const std::function<void(YAML::Emitter&, const Od&)>& output_pin_func,
                                          const std::function<void(YAML::Emitter&, const Nd&)>& node_func,
                                          const std::function<void(YAML::Emitter&, const Ld&)>& link_func) const {
  const glm::vec2 panning = editor_panning_;
  const glm::vec2 auto_panning_delta = editor_auto_panning_delta_;
  out << YAML::Key << "Panning" << YAML::Value << panning;
  out << YAML::Key << "AutoPanningDelta" << YAML::Value << auto_panning_delta;

  std::unordered_map<NodeGraphOutputPinHandle, int> output_pin_map;
  std::unordered_map<NodeGraphInputPinHandle, int> input_pin_map;
  std::unordered_map<NodeGraphLinkHandle, int> link_map;
  std::unordered_map<NodeGraphNodeHandle, int> node_map;
  int index = 0;
  for (NodeGraphNodeHandle handle = 0; handle < static_cast<NodeGraphNodeHandle>(nodes_.size()); handle++) {
    if (!nodes_[handle].recycled_) {
      node_map[handle] = index;
      index++;
    }
  }
  index = 0;
  for (NodeGraphOutputPinHandle handle = 0; handle < static_cast<NodeGraphOutputPinHandle>(output_pins_.size());
       handle++) {
    if (!output_pins_[handle].recycled_) {
      output_pin_map[handle] = index;
      index++;
    }
  }
  index = 0;
  for (NodeGraphInputPinHandle handle = 0; handle < static_cast<NodeGraphInputPinHandle>(input_pins_.size());
       handle++) {
    if (!input_pins_[handle].recycled_) {
      input_pin_map[handle] = index;
      index++;
    }
  }
  index = 0;
  for (NodeGraphLinkHandle handle = 0; handle < static_cast<NodeGraphLinkHandle>(links_.size()); handle++) {
    if (!links_[handle].recycled_) {
      link_map[handle] = index;
      index++;
    }
  }

  out << YAML::Key << "Nodes" << YAML::BeginSeq;
  for (NodeGraphNodeHandle handle = 0; handle < static_cast<NodeGraphNodeHandle>(nodes_.size()); handle++) {
    const auto& node = nodes_[handle];
    if (!node.recycled_) {
      out << YAML::BeginMap;
      glm::vec2 node_position = (handle < editor_node_positions_.size() ? editor_node_positions_[handle] : glm::vec2{});
      out << YAML::Key << "P" << YAML::Value << node_position;
      if (!node.input_pin_handles_.empty()) {
        out << YAML::Key << "I" << YAML::BeginSeq;
        for (const auto& input_pin_handle : node.input_pin_handles_) {
          out << input_pin_map[input_pin_handle];
        }
        out << YAML::EndSeq;
      }
      if (!node.output_pin_handles_.empty()) {
        out << YAML::Key << "O" << YAML::BeginSeq;
        for (const auto& output_pin_handle : node.output_pin_handles_) {
          out << output_pin_map[output_pin_handle];
        }
        out << YAML::EndSeq;
      }
      out << YAML::Key << "D" << YAML::BeginMap;
      node_func(out, node.data);
      out << YAML::EndMap;
      out << YAML::EndMap;
    }
  }
  out << YAML::EndSeq;
  out << YAML::Key << "OutputPins" << YAML::BeginSeq;
  for (NodeGraphOutputPinHandle handle = 0; handle < static_cast<NodeGraphOutputPinHandle>(output_pins_.size());
       handle++) {
    const auto& output_pin = output_pins_[handle];
    if (!output_pin.recycled_) {
      out << YAML::BeginMap;
      if (!output_pin.link_handles_.empty()) {
        out << YAML::Key << "L" << YAML::BeginSeq;
        for (const auto& link_handle : output_pin.link_handles_) {
          out << link_map[link_handle];
        }
        out << YAML::EndSeq;
      }

      out << YAML::Key << "D" << YAML::BeginMap;
      output_pin_func(out, output_pin.data);
      out << YAML::EndMap;
      out << YAML::EndMap;
    }
  }
  out << YAML::EndSeq;

  out << YAML::Key << "InputPins" << YAML::BeginSeq;
  for (NodeGraphInputPinHandle handle = 0; handle < static_cast<NodeGraphInputPinHandle>(input_pins_.size());
       handle++) {
    const auto& input_pin = input_pins_[handle];
    if (!input_pin.recycled_) {
      out << YAML::BeginMap;
      out << YAML::Key << "L" << YAML::Value
          << (input_pin.link_handle_ == -1 ? -1 : static_cast<int>(link_map[input_pin.link_handle_]));
      out << YAML::Key << "D" << YAML::BeginMap;
      input_pin_func(out, input_pin.data);
      out << YAML::EndMap;
      out << YAML::EndMap;
    }
  }
  out << YAML::EndSeq;
  out << YAML::Key << "Links" << YAML::BeginSeq;
  for (NodeGraphLinkHandle handle = 0; handle < static_cast<NodeGraphLinkHandle>(links_.size()); handle++) {
    const auto& link = links_[handle];
    if (!link.recycled_) {
      out << YAML::BeginMap;
      out << YAML::Key << "O" << YAML::Value << output_pin_map[link.start_];
      out << YAML::Key << "I" << YAML::Value << input_pin_map[link.end_];
      out << YAML::Key << "D" << YAML::BeginMap;
      link_func(out, link.data);
      out << YAML::EndMap;
      out << YAML::EndMap;
    }
  }
  out << YAML::EndSeq;
}
template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::Deserialize(const YAML::Node& in,
                                            const std::function<void(const YAML::Node&, Id&)>& input_pin_func,
                                            const std::function<void(const YAML::Node&, Od&)>& output_pin_func,
                                            const std::function<void(const YAML::Node&, Nd&)>& node_func,
                                            const std::function<void(const YAML::Node&, Ld&)>& link_func) {
  editor_panning_ = in["Panning"] ? in["Panning"].as<glm::vec2>() : glm::vec2{};
  editor_auto_panning_delta_ = in["AutoPanningDelta"] ? in["AutoPanningDelta"].as<glm::vec2>() : glm::vec2{};

  nodes_.clear();
  input_pins_.clear();
  output_pins_.clear();
  links_.clear();
  editor_node_positions_.clear();
  node_pool_ = {};
  input_pin_pool_ = {};
  output_pin_pool_ = {};
  link_pool_ = {};
  if (in["InputPins"]) {
    const auto& in_input_pins = in["InputPins"];
    NodeGraphInputPinHandle current_handle = 0;
    for (const auto& in_input_pin : in_input_pins) {
      input_pins_.emplace_back();
      auto& new_input_pin = input_pins_.back();
      new_input_pin.handle_ = current_handle;
      current_handle++;
      new_input_pin.link_handle_ = in_input_pin["L"].as<NodeGraphLinkHandle>();
      if (in_input_pin["D"]) {
        input_pin_func(in_input_pin["D"], new_input_pin.data);
      }
    }
  }
  if (in["OutputPins"]) {
    const auto& in_output_pins = in["OutputPins"];
    NodeGraphOutputPinHandle current_handle = 0;
    for (const auto& in_output_pin : in_output_pins) {
      output_pins_.emplace_back();
      auto& new_output_pin = output_pins_.back();
      new_output_pin.handle_ = current_handle;

      if (in_output_pin["L"]) {
        for (const auto& i : in_output_pin["L"]) {
          new_output_pin.link_handles_.emplace_back(i.as<NodeGraphLinkHandle>());
        }
      }
      if (in_output_pin["D"]) {
        output_pin_func(in_output_pin["D"], new_output_pin.data);
      }

      current_handle++;
    }
  }
  if (in["Nodes"]) {
    const auto& in_nodes = in["Nodes"];
    NodeGraphNodeHandle current_handle = 0;
    for (const auto& in_node : in_nodes) {
      nodes_.emplace_back();
      auto& new_node = nodes_.back();
      new_node.handle_ = current_handle;

      const auto position = in_node["P"] ? in_node["P"].as<glm::vec2>() : glm::vec2{};
      editor_node_positions_.emplace_back(position);
      if (in_node["I"]) {
        for (const auto& i : in_node["I"]) {
          const auto input_pin_handle = i.as<NodeGraphInputPinHandle>();
          new_node.input_pin_handles_.emplace_back(input_pin_handle);
          input_pins_[input_pin_handle].node_handle_ = current_handle;
        }
      }

      if (in_node["O"]) {
        for (const auto& i : in_node["O"]) {
          const auto output_pin_handle = i.as<NodeGraphOutputPinHandle>();
          new_node.output_pin_handles_.emplace_back(output_pin_handle);
          output_pins_[output_pin_handle].node_handle_ = current_handle;
        }
      }

      if (in_node["D"]) {
        node_func(in_node["D"], new_node.data);
      }

      current_handle++;
    }
  }
  if (in["Links"]) {
    const auto& in_links = in["Links"];
    NodeGraphLinkHandle current_handle = 0;
    for (const auto& in_link : in_links) {
      links_.emplace_back();
      auto& new_link = links_.back();
      new_link.handle_ = current_handle;

      new_link.start_ = in_link["O"].as<NodeGraphOutputPinHandle>();
      new_link.end_ = in_link["I"].as<NodeGraphInputPinHandle>();

      if (in_link["D"]) {
        link_func(in_link["D"], new_link.data);
      }

      current_handle++;
    }
  }
}
template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::Save(const std::string& name, YAML::Emitter& out,
                                     const std::function<void(YAML::Emitter&, const Id&)>& input_pin_func,
                                     const std::function<void(YAML::Emitter&, const Od&)>& output_pin_func,
                                     const std::function<void(YAML::Emitter&, const Nd&)>& node_func,
                                     const std::function<void(YAML::Emitter&, const Ld&)>& link_func) const {
  out << YAML::Key << name << YAML::BeginMap;
  Serialize(out, input_pin_func, output_pin_func, node_func, link_func);
  out << YAML::EndMap;
}
template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::Load(const std::string& name, const YAML::Node& in,
                                     const std::function<void(const YAML::Node&, Id&)>& input_pin_func,
                                     const std::function<void(const YAML::Node&, Od&)>& output_pin_func,
                                     const std::function<void(const YAML::Node&, Nd&)>& node_func,
                                     const std::function<void(const YAML::Node&, Ld&)>& link_func) {
  if (in[name]) {
    Deserialize(in[name], input_pin_func, output_pin_func, node_func, link_func);
  }
}
}  // namespace evo_engine
