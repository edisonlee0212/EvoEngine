#pragma once

#include "EditorLayer.hpp"
#include "EvoEngineEditorAPI.hpp"
#include "NodeGraph.hpp"
#include "imnodes_internal.hpp"

namespace evo_engine {
class EVOENGINE_EDITOR_API NodeGraphEditor {
 public:
  static ImNodesEditorContext& GetContext(const std::shared_ptr<IAsset>& owner);
  static void Clear();
  template <typename Graph>
  static bool Draw(
      Graph& graph, const std::shared_ptr<IAsset>& owner, const ImGuiID id,
      const std::shared_ptr<EditorLayer>& editor_layer,
      const std::function<void(NodeGraphNodeHandle node_handle)>& node_title_bar_gui,
      const std::function<void(NodeGraphNodeHandle node_handle)>& node_content_gui,
      const std::function<void(NodeGraphInputPinHandle input_pin_handle)>& node_input_pin_gui,
      const std::function<void(NodeGraphOutputPinHandle output_pin_handle)>& node_output_pin_gui,
      const std::function<void(NodeGraphNodeHandle node_handle, NodeGraphLinkHandle link_handle,
                               NodeGraphInputPinHandle input_pin_handle, NodeGraphOutputPinHandle output_pin_handle)>&
          hover_handler,
      const std::function<void(const std::vector<NodeGraphNodeHandle>& selected_node_handles,
                               const std::vector<NodeGraphLinkHandle>& selected_link_handles)>& selection_handler,
      const std::function<void(ImVec2 click_pos)>& canvas_popup_gui,
      const std::function<void(NodeGraphOutputPinHandle start_handle, NodeGraphInputPinHandle end_handle)>&
          link_create_handler,
      const std::function<void(NodeGraphLinkHandle link_handle)>& link_destroy_handler) {
    auto& editor_context = GetContext(owner);
    auto* prev_editor_context = ImNodes::GetCurrentContext()->EditorCtx;
    ImNodes::EditorContextSet(&editor_context);
    const auto panning = graph.GetPanning();
    const auto auto_panning = graph.GetAutoPanningDelta();
    editor_context.Panning = ImVec2(panning.x, panning.y);
    editor_context.AutoPanningDelta = ImVec2(auto_panning.x, auto_panning.y);

    ImNodesIO& io = ImNodes::GetIO();
    io.LinkDetachWithModifierClick.Modifier = &ImGui::GetIO().KeyAlt;
    io.MultipleSelectModifier.Modifier = &ImGui::GetIO().KeyCtrl;

    ImNodes::BeginNodeEditor();
    for (const auto& node : graph.PeekNodes()) {
      if (!node.Recycled()) {
        const auto position = graph.GetNodePosition(node.GetHandle());
        ImNodes::SetNodeScreenSpacePos(node.GetHandle(), ImVec2(position.x, position.y));
      }
    }

    if (const bool open_popup = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
                                ImNodes::IsEditorHovered() &&
                                editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Press;
        !ImGui::IsAnyItemHovered() && open_popup) {
      ImGui::OpenPopup(id);
    }
    if (ImGui::BeginPopupEx(id, ImGuiWindowFlags_NoDecoration)) {
      const ImVec2 click_pos = ImGui::GetMousePosOnOpeningCurrentPopup();
      canvas_popup_gui(click_pos);
      ImGui::EndPopup();
    }

    for (const auto& node : graph.PeekNodes()) {
      if (node.Recycled())
        continue;
      ImNodes::BeginNode(node.GetHandle());
      ImNodes::BeginNodeTitleBar();
      node_title_bar_gui(node.GetHandle());
      ImNodes::EndNodeTitleBar();
      node_content_gui(node.GetHandle());
      for (const auto input_pin_handle : node.GetInputPinHandles()) {
        ImNodes::BeginInputAttribute(input_pin_handle + (1 << 16), ImNodesPinShape_QuadFilled);
        // in between Begin|EndAttribute calls, you can call ImGui
        // UI functions
        node_input_pin_gui(input_pin_handle);
        ImNodes::EndInputAttribute();
      }

      for (const auto output_pin_handle : node.GetOutputPinHandles()) {
        ImNodes::BeginOutputAttribute(output_pin_handle + (1 << 17));
        // in between Begin|EndAttribute calls, you can call ImGui
        // UI functions
        node_output_pin_gui(output_pin_handle);
        ImNodes::EndOutputAttribute();
      }
      ImNodes::EndNode();
    }

    for (const auto& link : graph.PeekLinks()) {
      if (link.Recycled())
        continue;
      ImNodes::Link(link.GetHandle(), link.GetOutputPinHandle() + (1 << 17), link.GetInputPinHandle() + (1 << 16));
    }
    ImNodes::MiniMap();

    ImNodes::EndNodeEditor();

    if (ImNodes::IsEditorHovered() && ImGui::GetIO().MouseWheel != 0) {
      // const float zoom = ImNodes::EditorContextGet().+ ImGui::GetIO().MouseWheel * 0.1f;
      // ImNodes::EditorContextSetZoom(zoom, ImGui::GetMousePos());
    }

    NodeGraphNodeHandle hovered_node_handle = -1;
    NodeGraphLinkHandle hovered_link_handle = -1;
    NodeGraphInputPinHandle hovered_input_pin_handle = -1;
    NodeGraphOutputPinHandle hovered_output_pin_handle = -1;
    std::vector<NodeGraphNodeHandle> selected_nodes;
    std::vector<NodeGraphLinkHandle> selected_links;
    int handle = -1;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.f, 8.f));
    {
      if (ImNodes::IsNodeHovered(&handle)) {
        hovered_node_handle = handle;
      }
      if (ImNodes::IsLinkHovered(&handle)) {
        hovered_link_handle = handle;
      }
      if (ImNodes::IsPinHovered(&handle)) {
        if (handle < 1 << 17)
          hovered_input_pin_handle = handle - (1 << 16);
        else
          hovered_output_pin_handle = handle - (1 << 17);
      }
      hover_handler(hovered_node_handle, hovered_link_handle, hovered_input_pin_handle, hovered_output_pin_handle);
    }
    ImGui::PopStyleVar();
    if (const int num_selected_nodes = ImNodes::NumSelectedNodes(); num_selected_nodes > 0) {
      selected_nodes.resize(num_selected_nodes);
      ImNodes::GetSelectedNodes(selected_nodes.data());
    }
    if (const int num_selected_links = ImNodes::NumSelectedLinks(); num_selected_links > 0) {
      selected_links.resize(num_selected_links);
      ImNodes::GetSelectedLinks(selected_links.data());
    }
    selection_handler(selected_nodes, selected_links);

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

    graph.SetPanning({editor_context.Panning.x, editor_context.Panning.y});
    graph.SetAutoPanningDelta({editor_context.AutoPanningDelta.x, editor_context.AutoPanningDelta.y});
    for (const auto& node : graph.PeekNodes()) {
      if (!node.Recycled()) {
        const auto position = ImNodes::GetNodeScreenSpacePos(node.GetHandle());
        graph.SetNodePosition(node.GetHandle(), {position.x, position.y});
      }
    }
    ImNodes::EditorContextSet(prev_editor_context);
    return false;
  }
};
}  // namespace evo_engine
