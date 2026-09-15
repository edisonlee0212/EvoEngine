#include "EditorTheme.hpp"

using namespace evo_engine;

namespace {
editor_theme::Theme current_theme = editor_theme::Theme::Dark;

ImU32 ColorU32(const int r, const int g, const int b, const int a = 255) {
  return IM_COL32(r, g, b, a);
}
}  // namespace

void editor_theme::Apply(const Theme theme) {
  switch (theme) {
    case Theme::Dark:
      ApplyEvoEngineDark();
      break;
    case Theme::Light:
      ApplyEvoEngineLight();
      break;
  }
}

void editor_theme::ApplyDefault() {
  Apply(Theme::Dark);
}

editor_theme::Theme editor_theme::GetCurrentTheme() {
  return current_theme;
}

void editor_theme::ApplyEvoEngineDark() {
  current_theme = Theme::Dark;
  ImGui::StyleColorsDark();
  ImNodes::StyleColorsDark();

  ImGuiStyle& style = ImGui::GetStyle();
  ImVec4* colors = style.Colors;

  colors[ImGuiCol_Text] = ImVec4(0.78f, 0.78f, 0.78f, 1.00f);
  colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.082f, 0.082f, 0.082f, 1.00f);
  colors[ImGuiCol_ChildBg] = ImVec4(0.118f, 0.118f, 0.118f, 1.00f);
  colors[ImGuiCol_PopupBg] = ImVec4(0.196f, 0.196f, 0.196f, 0.98f);
  colors[ImGuiCol_Border] = ImVec4(0.102f, 0.102f, 0.102f, 1.00f);
  colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.059f, 0.059f, 0.059f, 1.00f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.145f, 0.145f, 0.145f, 1.00f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.185f, 0.185f, 0.185f, 1.00f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.047f, 0.047f, 0.047f, 1.00f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(0.082f, 0.082f, 0.082f, 1.00f);
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.047f, 0.047f, 0.047f, 0.86f);
  colors[ImGuiCol_MenuBarBg] = ImVec4(0.047f, 0.047f, 0.047f, 1.00f);
  colors[ImGuiCol_ScrollbarBg] = ImVec4(0.047f, 0.047f, 0.047f, 0.90f);
  colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.46f, 0.46f, 0.46f, 1.00f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.93f, 0.62f, 0.14f, 1.00f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.15f, 0.73f, 0.95f, 1.00f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.93f, 0.62f, 0.14f, 1.00f);
  colors[ImGuiCol_Button] = ImVec4(0.145f, 0.145f, 0.145f, 1.00f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.205f, 0.205f, 0.205f, 1.00f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.30f, 0.22f, 0.12f, 1.00f);
  colors[ImGuiCol_Header] = ImVec4(0.184f, 0.184f, 0.184f, 1.00f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.245f, 0.245f, 0.245f, 1.00f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.30f, 0.24f, 0.16f, 1.00f);
  colors[ImGuiCol_Separator] = ImVec4(0.102f, 0.102f, 0.102f, 1.00f);
  colors[ImGuiCol_SeparatorHovered] = ImVec4(0.15f, 0.73f, 0.95f, 0.80f);
  colors[ImGuiCol_SeparatorActive] = ImVec4(0.15f, 0.73f, 0.95f, 1.00f);
  colors[ImGuiCol_ResizeGrip] = ImVec4(0.93f, 0.62f, 0.14f, 0.20f);
  colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.93f, 0.62f, 0.14f, 0.55f);
  colors[ImGuiCol_ResizeGripActive] = ImVec4(0.93f, 0.62f, 0.14f, 0.85f);
  colors[ImGuiCol_Tab] = ImVec4(0.047f, 0.047f, 0.047f, 1.00f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.205f, 0.205f, 0.205f, 1.00f);
  colors[ImGuiCol_TabSelected] = ImVec4(0.141f, 0.141f, 0.141f, 1.00f);
  colors[ImGuiCol_TabSelectedOverline] = ImVec4(0.93f, 0.62f, 0.14f, 1.00f);
  colors[ImGuiCol_TabDimmed] = ImVec4(0.047f, 0.047f, 0.047f, 1.00f);
  colors[ImGuiCol_TabDimmedSelected] = ImVec4(0.102f, 0.102f, 0.102f, 1.00f);
  colors[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.46f, 0.34f, 0.18f, 1.00f);
  colors[ImGuiCol_DockingPreview] = ImVec4(0.15f, 0.73f, 0.95f, 0.35f);
  colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.047f, 0.047f, 0.047f, 1.00f);
  colors[ImGuiCol_PlotLines] = ImVec4(0.62f, 0.62f, 0.62f, 1.00f);
  colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.93f, 0.62f, 0.14f, 1.00f);
  colors[ImGuiCol_PlotHistogram] = ImVec4(0.15f, 0.73f, 0.95f, 1.00f);
  colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.93f, 0.62f, 0.14f, 1.00f);
  colors[ImGuiCol_TableHeaderBg] = ImVec4(0.184f, 0.184f, 0.184f, 1.00f);
  colors[ImGuiCol_TableBorderStrong] = ImVec4(0.102f, 0.102f, 0.102f, 1.00f);
  colors[ImGuiCol_TableBorderLight] = ImVec4(0.145f, 0.145f, 0.145f, 1.00f);
  colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.00f, 1.00f, 1.00f, 0.025f);
  colors[ImGuiCol_TextLink] = ImVec4(0.93f, 0.62f, 0.14f, 1.00f);
  colors[ImGuiCol_TextSelectedBg] = ImVec4(0.93f, 0.75f, 0.47f, 0.35f);
  colors[ImGuiCol_DragDropTarget] = ImVec4(0.15f, 0.73f, 0.95f, 0.90f);
  colors[ImGuiCol_NavCursor] = ImVec4(0.15f, 0.73f, 0.95f, 1.00f);
  colors[ImGuiCol_NavWindowingHighlight] = ImVec4(0.78f, 0.78f, 0.78f, 0.70f);
  colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.35f);
  colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.55f);

  style.WindowPadding = ImVec2(8.0f, 8.0f);
  style.FramePadding = ImVec2(6.0f, 4.0f);
  style.ItemSpacing = ImVec2(6.0f, 5.0f);
  style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
  style.CellPadding = ImVec2(6.0f, 4.0f);
  style.WindowRounding = 4.0f;
  style.ChildRounding = 4.0f;
  style.FrameRounding = 4.0f;
  style.GrabRounding = 3.0f;
  style.PopupRounding = 4.0f;
  style.ScrollbarRounding = 3.0f;
  style.TabRounding = 4.0f;
  style.WindowMenuButtonPosition = ImGuiDir_Right;
  style.ScrollbarSize = 10.0f;
  style.GrabMinSize = 10.0f;
  style.DockingSeparatorSize = 1.0f;
  style.SeparatorTextBorderSize = 2.0f;
  style.FrameBorderSize = 0.0f;
  style.WindowBorderSize = 1.0f;
  style.ChildBorderSize = 1.0f;
  style.PopupBorderSize = 1.0f;
  style.DisabledAlpha = 0.58f;

  ImNodesStyle& node_style = ImNodes::GetStyle();
  node_style.NodeCornerRounding = 4.0f;
  node_style.NodeBorderThickness = 1.0f;
  node_style.LinkThickness = 2.5f;
  node_style.Colors[ImNodesCol_NodeBackground] = ColorU32(30, 30, 30);
  node_style.Colors[ImNodesCol_NodeBackgroundHovered] = ColorU32(42, 42, 42);
  node_style.Colors[ImNodesCol_NodeBackgroundSelected] = ColorU32(50, 42, 30);
  node_style.Colors[ImNodesCol_NodeOutline] = ColorU32(70, 70, 70);
  node_style.Colors[ImNodesCol_TitleBar] = ColorU32(21, 21, 21);
  node_style.Colors[ImNodesCol_TitleBarHovered] = ColorU32(47, 47, 47);
  node_style.Colors[ImNodesCol_TitleBarSelected] = ColorU32(70, 48, 22);
  node_style.Colors[ImNodesCol_Link] = ColorU32(39, 185, 242, 210);
  node_style.Colors[ImNodesCol_LinkHovered] = ColorU32(236, 158, 36);
  node_style.Colors[ImNodesCol_LinkSelected] = ColorU32(237, 192, 119);
  node_style.Colors[ImNodesCol_Pin] = ColorU32(39, 185, 242, 190);
  node_style.Colors[ImNodesCol_PinHovered] = ColorU32(236, 158, 36);
  node_style.Colors[ImNodesCol_BoxSelector] = ColorU32(237, 192, 119, 38);
  node_style.Colors[ImNodesCol_BoxSelectorOutline] = ColorU32(237, 192, 119, 160);
  node_style.Colors[ImNodesCol_GridBackground] = ColorU32(18, 18, 18, 230);
  node_style.Colors[ImNodesCol_GridLine] = ColorU32(120, 120, 120, 24);
  node_style.Colors[ImNodesCol_GridLinePrimary] = ColorU32(150, 150, 150, 42);
  node_style.Colors[ImNodesCol_MiniMapBackground] = ColorU32(18, 18, 18, 180);
  node_style.Colors[ImNodesCol_MiniMapBackgroundHovered] = ColorU32(26, 26, 26, 220);
  node_style.Colors[ImNodesCol_MiniMapOutline] = ColorU32(80, 80, 80, 140);
  node_style.Colors[ImNodesCol_MiniMapOutlineHovered] = ColorU32(120, 120, 120, 220);
  node_style.Colors[ImNodesCol_MiniMapNodeBackground] = ColorU32(150, 150, 150, 110);
  node_style.Colors[ImNodesCol_MiniMapNodeBackgroundHovered] = ColorU32(190, 190, 190, 220);
  node_style.Colors[ImNodesCol_MiniMapNodeBackgroundSelected] = ColorU32(237, 192, 119, 255);
  node_style.Colors[ImNodesCol_MiniMapNodeOutline] = ColorU32(180, 180, 180, 120);
  node_style.Colors[ImNodesCol_MiniMapLink] = node_style.Colors[ImNodesCol_Link];
  node_style.Colors[ImNodesCol_MiniMapLinkSelected] = node_style.Colors[ImNodesCol_LinkSelected];
  node_style.Colors[ImNodesCol_MiniMapCanvas] = ColorU32(237, 192, 119, 28);
  node_style.Colors[ImNodesCol_MiniMapCanvasOutline] = ColorU32(237, 192, 119, 200);
}

void editor_theme::ApplyEvoEngineLight() {
  current_theme = Theme::Light;
  ImGui::StyleColorsLight();
  ImNodes::StyleColorsLight();

  ImGuiStyle& style = ImGui::GetStyle();
  ImVec4* colors = style.Colors;

  colors[ImGuiCol_Text] = ImVec4(0.10f, 0.11f, 0.12f, 1.00f);
  colors[ImGuiCol_TextDisabled] = ImVec4(0.49f, 0.51f, 0.54f, 1.00f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.985f, 0.988f, 0.990f, 1.00f);
  colors[ImGuiCol_ChildBg] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
  colors[ImGuiCol_PopupBg] = ImVec4(1.00f, 1.00f, 1.00f, 0.98f);
  colors[ImGuiCol_Border] = ImVec4(0.86f, 0.88f, 0.90f, 1.00f);
  colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.970f, 0.975f, 0.980f, 1.00f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.925f, 0.955f, 0.980f, 1.00f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.875f, 0.925f, 0.960f, 1.00f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.970f, 0.975f, 0.980f, 1.00f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.970f, 0.975f, 0.980f, 0.86f);
  colors[ImGuiCol_MenuBarBg] = ImVec4(0.970f, 0.975f, 0.980f, 1.00f);
  colors[ImGuiCol_ScrollbarBg] = ImVec4(0.965f, 0.970f, 0.975f, 0.90f);
  colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.74f, 0.76f, 0.78f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.62f, 0.65f, 0.68f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.49f, 0.52f, 0.55f, 1.00f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.70f, 0.40f, 0.08f, 1.00f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.09f, 0.55f, 0.74f, 1.00f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.70f, 0.40f, 0.08f, 1.00f);
  colors[ImGuiCol_Button] = ImVec4(0.955f, 0.965f, 0.975f, 1.00f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.900f, 0.940f, 0.975f, 1.00f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.94f, 0.78f, 0.52f, 1.00f);
  colors[ImGuiCol_Header] = ImVec4(0.940f, 0.965f, 0.985f, 1.00f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.880f, 0.930f, 0.970f, 1.00f);
  colors[ImGuiCol_HeaderActive] = ImVec4(1.00f, 0.900f, 0.760f, 1.00f);
  colors[ImGuiCol_Separator] = ImVec4(0.84f, 0.86f, 0.88f, 1.00f);
  colors[ImGuiCol_SeparatorHovered] = ImVec4(0.09f, 0.55f, 0.74f, 0.80f);
  colors[ImGuiCol_SeparatorActive] = ImVec4(0.09f, 0.55f, 0.74f, 1.00f);
  colors[ImGuiCol_ResizeGrip] = ImVec4(0.70f, 0.40f, 0.08f, 0.20f);
  colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.70f, 0.40f, 0.08f, 0.55f);
  colors[ImGuiCol_ResizeGripActive] = ImVec4(0.70f, 0.40f, 0.08f, 0.85f);
  colors[ImGuiCol_Tab] = ImVec4(0.955f, 0.965f, 0.975f, 1.00f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.900f, 0.940f, 0.975f, 1.00f);
  colors[ImGuiCol_TabSelected] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
  colors[ImGuiCol_TabSelectedOverline] = ImVec4(0.70f, 0.40f, 0.08f, 1.00f);
  colors[ImGuiCol_TabDimmed] = ImVec4(0.940f, 0.950f, 0.960f, 1.00f);
  colors[ImGuiCol_TabDimmedSelected] = ImVec4(0.985f, 0.988f, 0.990f, 1.00f);
  colors[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.78f, 0.61f, 0.35f, 1.00f);
  colors[ImGuiCol_DockingPreview] = ImVec4(0.09f, 0.55f, 0.74f, 0.35f);
  colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.985f, 0.988f, 0.990f, 1.00f);
  colors[ImGuiCol_PlotLines] = ImVec4(0.38f, 0.40f, 0.42f, 1.00f);
  colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.70f, 0.40f, 0.08f, 1.00f);
  colors[ImGuiCol_PlotHistogram] = ImVec4(0.09f, 0.55f, 0.74f, 1.00f);
  colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.70f, 0.40f, 0.08f, 1.00f);
  colors[ImGuiCol_TableHeaderBg] = ImVec4(0.945f, 0.955f, 0.965f, 1.00f);
  colors[ImGuiCol_TableBorderStrong] = ImVec4(0.78f, 0.80f, 0.82f, 1.00f);
  colors[ImGuiCol_TableBorderLight] = ImVec4(0.88f, 0.90f, 0.92f, 1.00f);
  colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.00f, 0.00f, 0.00f, 0.035f);
  colors[ImGuiCol_TextLink] = ImVec4(0.70f, 0.40f, 0.08f, 1.00f);
  colors[ImGuiCol_TextSelectedBg] = ImVec4(0.70f, 0.40f, 0.08f, 0.24f);
  colors[ImGuiCol_DragDropTarget] = ImVec4(0.09f, 0.55f, 0.74f, 0.90f);
  colors[ImGuiCol_NavCursor] = ImVec4(0.09f, 0.55f, 0.74f, 1.00f);
  colors[ImGuiCol_NavWindowingHighlight] = ImVec4(0.12f, 0.13f, 0.14f, 0.60f);
  colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.12f);
  colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.25f);

  style.WindowPadding = ImVec2(8.0f, 8.0f);
  style.FramePadding = ImVec2(6.0f, 4.0f);
  style.ItemSpacing = ImVec2(6.0f, 5.0f);
  style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
  style.CellPadding = ImVec2(6.0f, 4.0f);
  style.WindowRounding = 4.0f;
  style.ChildRounding = 4.0f;
  style.FrameRounding = 4.0f;
  style.GrabRounding = 3.0f;
  style.PopupRounding = 4.0f;
  style.ScrollbarRounding = 3.0f;
  style.TabRounding = 4.0f;
  style.WindowMenuButtonPosition = ImGuiDir_Right;
  style.ScrollbarSize = 10.0f;
  style.GrabMinSize = 10.0f;
  style.DockingSeparatorSize = 1.0f;
  style.SeparatorTextBorderSize = 2.0f;
  style.FrameBorderSize = 0.0f;
  style.WindowBorderSize = 1.0f;
  style.ChildBorderSize = 1.0f;
  style.PopupBorderSize = 1.0f;
  style.DisabledAlpha = 0.58f;

  ImNodesStyle& node_style = ImNodes::GetStyle();
  node_style.NodeCornerRounding = 4.0f;
  node_style.NodeBorderThickness = 1.0f;
  node_style.LinkThickness = 2.5f;
  node_style.Colors[ImNodesCol_NodeBackground] = ColorU32(248, 248, 245);
  node_style.Colors[ImNodesCol_NodeBackgroundHovered] = ColorU32(239, 241, 242);
  node_style.Colors[ImNodesCol_NodeBackgroundSelected] = ColorU32(255, 246, 232);
  node_style.Colors[ImNodesCol_NodeOutline] = ColorU32(190, 195, 200);
  node_style.Colors[ImNodesCol_TitleBar] = ColorU32(225, 228, 230);
  node_style.Colors[ImNodesCol_TitleBarHovered] = ColorU32(211, 222, 230);
  node_style.Colors[ImNodesCol_TitleBarSelected] = ColorU32(239, 216, 184);
  node_style.Colors[ImNodesCol_Link] = ColorU32(23, 140, 188, 210);
  node_style.Colors[ImNodesCol_LinkHovered] = ColorU32(179, 102, 20);
  node_style.Colors[ImNodesCol_LinkSelected] = ColorU32(179, 102, 20);
  node_style.Colors[ImNodesCol_Pin] = ColorU32(23, 140, 188, 190);
  node_style.Colors[ImNodesCol_PinHovered] = ColorU32(179, 102, 20);
  node_style.Colors[ImNodesCol_BoxSelector] = ColorU32(179, 102, 20, 35);
  node_style.Colors[ImNodesCol_BoxSelectorOutline] = ColorU32(179, 102, 20, 150);
  node_style.Colors[ImNodesCol_GridBackground] = ColorU32(246, 247, 245, 230);
  node_style.Colors[ImNodesCol_GridLine] = ColorU32(80, 84, 88, 28);
  node_style.Colors[ImNodesCol_GridLinePrimary] = ColorU32(80, 84, 88, 52);
  node_style.Colors[ImNodesCol_MiniMapBackground] = ColorU32(246, 247, 245, 185);
  node_style.Colors[ImNodesCol_MiniMapBackgroundHovered] = ColorU32(235, 238, 240, 225);
  node_style.Colors[ImNodesCol_MiniMapOutline] = ColorU32(150, 154, 158, 150);
  node_style.Colors[ImNodesCol_MiniMapOutlineHovered] = ColorU32(110, 114, 118, 220);
  node_style.Colors[ImNodesCol_MiniMapNodeBackground] = ColorU32(110, 114, 118, 110);
  node_style.Colors[ImNodesCol_MiniMapNodeBackgroundHovered] = ColorU32(90, 94, 98, 220);
  node_style.Colors[ImNodesCol_MiniMapNodeBackgroundSelected] = ColorU32(179, 102, 20, 255);
  node_style.Colors[ImNodesCol_MiniMapNodeOutline] = ColorU32(120, 124, 128, 130);
  node_style.Colors[ImNodesCol_MiniMapLink] = node_style.Colors[ImNodesCol_Link];
  node_style.Colors[ImNodesCol_MiniMapLinkSelected] = node_style.Colors[ImNodesCol_LinkSelected];
  node_style.Colors[ImNodesCol_MiniMapCanvas] = ColorU32(179, 102, 20, 28);
  node_style.Colors[ImNodesCol_MiniMapCanvasOutline] = ColorU32(179, 102, 20, 200);
}
