#pragma once

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
  bool recycled_ = true;                 /**< Flag indicating if this pin has been recycled. */
  NodeGraphInputPinHandle handle_ = -1;  /**< Handle identifying this input pin uniquely. */
  NodeGraphLinkHandle link_handle_ = -1; /**< Handle to the link connected to this input pin, if any. */

 public:
  NodeGraphInputPinData m_data{}; /**< Data associated with this input pin. */

  /** @brief Default constructor. */
  NodeGraphInputPin() = default;

  /**
   * @brief Constructs an input pin with a given handle and associated node.
   * @param handle The handle associated with this input pin.
   * @param node_handle The handle of the node this input pin belongs to.
   */
  NodeGraphInputPin(NodeGraphInputPinHandle handle, NodeGraphNodeHandle node_handle);
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
  bool recycled_ = true;                            /**< Flag indicating if this pin has been recycled. */
  NodeGraphOutputPinHandle handle_ = -1;            /**< Handle identifying this output pin uniquely. */
  std::vector<NodeGraphLinkHandle> link_handles_{}; /**< Handles to links connected to this output pin. */

 public:
  NodeGraphOutputPinData m_data{}; /**< Data associated with this output pin. */

  /** @brief Default constructor. */
  NodeGraphOutputPin() = default;

  /**
   * @brief Constructs an output pin with a given handle and associated node.
   * @param handle The handle associated with this output pin.
   * @param node_handle The handle of the node this output pin belongs to.
   */
  NodeGraphOutputPin(NodeGraphOutputPinHandle handle, NodeGraphNodeHandle node_handle);
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
  bool recycled_ = true;            /**< Flag indicating whether this node has been recycled. */

  std::vector<NodeGraphInputPinHandle> input_pin_handles_; /**< Handles of input pins attached to this node. */
  NodeGraphOutputPinHandle output_pin_handle_; /**< Handle of the output pin attached to this node (if any). */

 public:
  NodeGraphNodeData data{}; /**< Data associated with this node. */

  /** @brief Default constructor. */
  NodeGraphNode() = default;

  /**
   * @brief Constructs a node with a given handle.
   * @param handle The handle associated with this node.
   */
  NodeGraphNode(NodeGraphNodeHandle handle);
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
  bool recycled_ = true;                /**< Flag indicating whether this link has been recycled. */

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

 public:
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
   * @param has_output Whether the node should have an output pin.
   * @return Handle to the allocated node.
   */
  NodeGraphNodeHandle AllocateNode(size_t input_pin_count, bool has_output);

  /**
   * @brief Recycles a node and returns it back to the pool.
   * @param handle Handle of the node to recycle.
   */
  void RecycleNode(NodeGraphNodeHandle handle);

  /**
   * @brief Handles the inspection and GUI rendering for the node graph in the editor.
   * @param title The title of the node graph window.
   * @param editor_layer Shared pointer to the editor layer handling the inspection.
   * @param node_editor_popup_gui Callback for handling right-click popups in the graph editor.
   * @param node_title_bar_gui Callback for rendering the title bar of nodes.
   * @param node_input_pin_gui Callback for rendering input pins.
   * @param node_output_pin_gui Callback for rendering output pins.
   * @param link_create_handler Callback for handling new link creation.
   * @param link_destroy_handler Callback for handling link deletion.
   * @param hover_handler Callback for handling hover events over nodes, links, or pins.
   * @param selection_handler Callback for handling node/link selection.
   * @return True if the asset's content is not modified during inspection; otherwise, false.
   */
  bool OnInspect(
      const std::string& title, const std::shared_ptr<EditorLayer>& editor_layer,
      const std::function<void(ImVec2 click_pos)>& node_editor_popup_gui,
      const std::function<void(NodeGraphNodeHandle node_handle)>& node_title_bar_gui,
      const std::function<void(NodeGraphInputPinHandle input_pin_handle)>& node_input_pin_gui,
      const std::function<void(NodeGraphOutputPinHandle output_pin_handle)>& node_output_pin_gui,
      const std::function<void(NodeGraphOutputPinHandle start_handle, NodeGraphInputPinHandle end_handle)>&
          link_create_handler,
      const std::function<void(NodeGraphLinkHandle link_handle)>& link_destroy_handler,
      const std::function<void(NodeGraphNodeHandle node_handle, NodeGraphLinkHandle link_handle,
                               NodeGraphInputPinHandle input_pin_handle, NodeGraphOutputPinHandle output_pin_handle)>&
          hover_handler,
      const std::function<void(const std::vector<NodeGraphNodeHandle>& selected_node_handles,
                               const std::vector<NodeGraphLinkHandle>& selected_link_handles)>& selection_handler);
};

template <typename NodeGraphInputPinData>
NodeGraphInputPin<NodeGraphInputPinData>::NodeGraphInputPin(const NodeGraphInputPinHandle handle,
                                                            const NodeGraphNodeHandle node_handle) {
  node_handle_ = node_handle;
  link_handle_ = -1;
  handle_ = handle;
  recycled_ = false;
  m_data = {};
}

template <typename NodeGraphOutputPinData>
NodeGraphOutputPin<NodeGraphOutputPinData>::NodeGraphOutputPin(const NodeGraphOutputPinHandle handle,
                                                               const NodeGraphNodeHandle node_handle) {
  node_handle_ = node_handle;
  link_handles_.clear();
  handle_ = handle;
  recycled_ = false;
  m_data = {};
}

template <typename NodeGraphNodeData>
NodeGraphNode<NodeGraphNodeData>::NodeGraphNode(const NodeGraphNodeHandle handle) {
  input_pin_handles_.clear();
  output_pin_handle_ = -1;
  handle_ = handle;
  recycled_ = false;
  data = {};
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

template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphInputPinHandle NodeGraph<Id, Od, Nd, Ld>::AllocateInputPin(const NodeGraphNodeHandle node_handle) {
  if (input_pin_pool_.empty()) {
    input_pins_.emplace_back(input_pins_.size(), node_handle);
    return input_pins_.back().handle_;
  }
  auto handle = input_pin_pool_.front();
  input_pin_pool_.pop();
  auto& input_pin = input_pins_[handle];
  input_pin.link_handle_ = -1;
  input_pin.node_handle_ = node_handle;
  input_pin.recycled_ = false;
  return handle;
}

template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphOutputPinHandle NodeGraph<Id, Od, Nd, Ld>::AllocateOutputPin(const NodeGraphNodeHandle node_handle) {
  if (output_pin_pool_.empty()) {
    output_pins_.emplace_back(output_pins_.size(), node_handle);
    return output_pins_.back().handle_;
  }
  auto handle = output_pin_pool_.front();
  output_pin_pool_.pop();
  auto& output_pin = output_pins_[handle];
  output_pin.link_handles_.clear();
  output_pin.node_handle_ = node_handle;
  output_pin.recycled_ = false;
  return handle;
}

template <typename Id, typename Od, typename Nd, typename Ld>
NodeGraphLinkHandle NodeGraph<Id, Od, Nd, Ld>::AllocateLink(const NodeGraphOutputPinHandle start_handle,
                                                            const NodeGraphInputPinHandle end_handle) {
  if (link_pool_.empty()) {
    links_.emplace_back(links_.size(), start_handle, end_handle);
    return links_.back().handle_;
  }
  auto handle = link_pool_.front();
  link_pool_.pop();
  auto& link = links_[handle];
  link.start_ = start_handle;
  link.end_ = end_handle;
  link.recycled_ = false;
  return handle;
}

template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::RecycleLink(const NodeGraphLinkHandle handle) {
  assert(!links_[handle].recycled_);
  auto& link = links_[handle];
  link.data = {};
  auto& output_pin_link_handles = output_pins_[link.start_].link_handles_;
  for (int i = 0; i < output_pin_link_handles.size(); i++) {
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
NodeGraphNodeHandle NodeGraph<Id, Od, Nd, Ld>::AllocateNode(const size_t input_pin_count, bool has_output) {
  NodeGraphNodeHandle new_node_handle;
  if (node_pool_.empty()) {
    nodes_.emplace_back(nodes_.size());
    new_node_handle = nodes_.back().handle_;
  } else {
    new_node_handle = node_pool_.front();
    node_pool_.pop();
  }
  auto& node = nodes_[new_node_handle];
  node.data = {};
  node.input_pin_handles_.clear();
  node.output_pin_handle_ = -1;
  for (int i = 0; i < input_pin_count; i++) {
    node.input_pin_handles_.emplace_back(AllocateInputPin(new_node_handle));
  }
  if (has_output)
    node.output_pin_handle_ = AllocateOutputPin(new_node_handle);
  node.recycled_ = false;
  return new_node_handle;
}

template <typename Id, typename Od, typename Nd, typename Ld>
void NodeGraph<Id, Od, Nd, Ld>::RecycleNode(const NodeGraphNodeHandle handle) {
  assert(!nodes_[handle].recycled_);
  auto& node = nodes_[handle];
  node.data = {};
  for (const auto& i : node.input_pin_handles_) {
    RecycleInputPin(i);
  }
  if (node.output_pin_handle_ != -1)
    RecycleOutputPin(node.output_pin_handle_);
  node.recycled_ = true;
  node_pool_.emplace(handle);
}

template <typename Id, typename Od, typename Nd, typename Ld>
bool NodeGraph<Id, Od, Nd, Ld>::OnInspect(
    const std::string& title, const std::shared_ptr<EditorLayer>& editor_layer,
    const std::function<void(ImVec2 click_pos)>& node_editor_popup_gui,
    const std::function<void(NodeGraphNodeHandle node_handle)>& node_title_bar_gui,
    const std::function<void(NodeGraphInputPinHandle input_pin_handle)>& node_input_pin_gui,
    const std::function<void(NodeGraphOutputPinHandle output_pin_handle)>& node_output_pin_gui,
    const std::function<void(NodeGraphOutputPinHandle start_handle, NodeGraphInputPinHandle end_handle)>&
        link_create_handler,
    const std::function<void(NodeGraphLinkHandle link_handle)>& link_destroy_handler,
    const std::function<void(NodeGraphNodeHandle node_handle, NodeGraphLinkHandle link_handle,
                             NodeGraphInputPinHandle input_pin_handle, NodeGraphOutputPinHandle output_pin_handle)>&
        hover_handler,
    const std::function<void(const std::vector<NodeGraphNodeHandle>& selected_node_handles,
                             const std::vector<NodeGraphLinkHandle>& selected_link_handles)>& selection_handler) {
  ImNodesIO& io = ImNodes::GetIO();
  io.LinkDetachWithModifierClick.Modifier = &ImGui::GetIO().KeyAlt;
  io.MultipleSelectModifier.Modifier = &ImGui::GetIO().KeyCtrl;

  ImGui::Begin(title.c_str());

  ImNodes::BeginNodeEditor();

  // Handle new nodes
  // These are driven by the user, so we place this code before rendering the nodes

  if (const bool open_popup = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
                              ImNodes::IsEditorHovered() &&
                              editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Press;
      !ImGui::IsAnyItemHovered() && open_popup) {
    ImGui::OpenPopup((title + "_editor_menu").c_str());
  }
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.f, 8.f));
  if (ImGui::BeginPopup((title + "_editor_menu").c_str())) {
    const ImVec2 click_pos = ImGui::GetMousePosOnOpeningCurrentPopup();
    node_editor_popup_gui(click_pos);
    ImGui::EndPopup();
  }
  ImGui::PopStyleVar();

  for (const auto& node : nodes_) {
    if (node.recycled_)
      continue;
    ImNodes::BeginNode(node.handle_);
    ImNodes::BeginNodeTitleBar();
    node_title_bar_gui(node.handle_);
    ImNodes::EndNodeTitleBar();

    for (const auto input_pin_handle : node.input_pin_handles_) {
      ImNodes::BeginInputAttribute(input_pin_handle + (1 << 16), ImNodesPinShape_QuadFilled);
      // in between Begin|EndAttribute calls, you can call ImGui
      // UI functions
      node_input_pin_gui(input_pin_handle);
      ImNodes::EndInputAttribute();
    }

    if (node.output_pin_handle_ != -1) {
      ImNodes::BeginOutputAttribute(node.output_pin_handle_ + (1 << 17));
      // in between Begin|EndAttribute calls, you can call ImGui
      // UI functions
      node_output_pin_gui(node.output_pin_handle_);
      ImNodes::EndOutputAttribute();
    }

    ImNodes::EndNode();
  }

  for (const auto& link : links_) {
    if (link.recycled_)
      continue;
    ImNodes::Link(link.handle_, link.start_ + (1 << 17), link.end_ + (1 << 16));
  }

  ImNodes::EndNodeEditor();
  {
    NodeGraphOutputPinHandle output_handle;
    NodeGraphInputPinHandle input_handle;
    if (ImNodes::IsLinkCreated(&output_handle, &input_handle)) {
      link_create_handler(output_handle - (1 << 17), input_handle - (1 << 16));
    }
  }
  {
    NodeGraphLinkHandle link_handle;
    if (ImNodes::IsLinkDestroyed(&link_handle)) {
      link_destroy_handler(link_handle);
    }
  }
  NodeGraphNodeHandle hovered_node_handle = -1;
  NodeGraphLinkHandle hovered_link_handle = -1;
  NodeGraphInputPinHandle hovered_input_pin_handle = -1;
  NodeGraphOutputPinHandle hovered_output_pin_handle = -1;
  std::vector<NodeGraphNodeHandle> selected_nodes;
  std::vector<NodeGraphLinkHandle> selected_links;
  int id = -1;
  if (ImNodes::IsNodeHovered(&id)) {
    hovered_node_handle = id;
  }
  if (ImNodes::IsLinkHovered(&id)) {
    hovered_link_handle = id;
  }
  if (ImNodes::IsPinHovered(&id)) {
    if (id < (1 << 17))
      hovered_input_pin_handle = id - (1 << 16);
    else
      hovered_output_pin_handle = id - (1 << 17);
  }
  hover_handler(hovered_node_handle, hovered_link_handle, hovered_input_pin_handle, hovered_output_pin_handle);
  if (const int num_selected_nodes = ImNodes::NumSelectedNodes(); num_selected_nodes > 0) {
    selected_nodes.resize(num_selected_nodes);
    ImNodes::GetSelectedNodes(selected_nodes.data());
  }
  if (const int num_selected_links = ImNodes::NumSelectedLinks(); num_selected_links > 0) {
    selected_links.resize(num_selected_links);
    ImNodes::GetSelectedLinks(selected_links.data());
  }
  selection_handler(selected_nodes, selected_links);
  ImGui::End();
  return false;
}
}  // namespace evo_engine
