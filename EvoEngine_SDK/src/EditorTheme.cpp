#include "EditorTheme.hpp"

using namespace evo_engine;

namespace {
ImU32 ColorU32(const int r, const int g, const int b, const int a = 255) {
  return IM_COL32(r, g, b, a);
}
}  // namespace

void editor_theme::ApplyDefault() {
  ApplyEvoEngineLegacy();
}

void editor_theme::ApplyEvoEngineLegacy() {
  ImGui::StyleColorsDark();
  ImNodes::StyleColorsDark();

  ImGuiStyle& style = ImGui::GetStyle();
  ImVec4* colors = style.Colors;

  style.WindowRounding = 5.3f;
  style.ChildRounding = 0.0f;
  style.FrameRounding = 2.3f;
  style.GrabRounding = 0.0f;
  style.PopupRounding = 0.0f;
  style.ScrollbarRounding = 0.0f;
  style.TabRounding = 5.0f;
  style.WindowMenuButtonPosition = ImGuiDir_Left;
  style.ScrollbarSize = 14.0f;
  style.GrabMinSize = 12.0f;
  style.DockingSeparatorSize = 2.0f;
  style.SeparatorTextBorderSize = 3.0f;
  style.FrameBorderSize = 0.0f;
  style.WindowBorderSize = 1.0f;
  style.ChildBorderSize = 1.0f;
  style.PopupBorderSize = 1.0f;

  colors[ImGuiCol_Text] = ImVec4(0.90f, 0.90f, 0.90f, 0.90f);
  colors[ImGuiCol_TextDisabled] = ImVec4(0.30f, 0.30f, 0.30f, 0.90f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.09f, 0.09f, 0.15f, 1.00f);
  colors[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_PopupBg] = ImVec4(0.05f, 0.05f, 0.10f, 0.85f);
  colors[ImGuiCol_Border] = ImVec4(0.70f, 0.70f, 0.70f, 0.65f);
  colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.00f, 0.00f, 0.01f, 1.00f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.90f, 0.80f, 0.80f, 0.40f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.90f, 0.65f, 0.65f, 0.45f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.83f);
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.40f, 0.40f, 0.80f, 0.20f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(0.00f, 0.00f, 0.00f, 0.87f);
  colors[ImGuiCol_MenuBarBg] = ImVec4(0.01f, 0.01f, 0.02f, 0.80f);
  colors[ImGuiCol_ScrollbarBg] = ImVec4(0.20f, 0.25f, 0.30f, 0.60f);
  colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.55f, 0.53f, 0.55f, 0.51f);
  colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.56f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.56f, 0.56f, 0.56f, 0.91f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.90f, 0.90f, 0.90f, 0.83f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.70f, 0.70f, 0.70f, 0.62f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.30f, 0.30f, 0.30f, 0.84f);
  colors[ImGuiCol_Button] = ImVec4(0.48f, 0.72f, 0.89f, 0.49f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.50f, 0.69f, 0.99f, 0.68f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.80f, 0.50f, 0.50f, 1.00f);
  colors[ImGuiCol_Header] = ImVec4(0.30f, 0.69f, 1.00f, 0.53f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.44f, 0.61f, 0.86f, 1.00f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.38f, 0.62f, 0.83f, 1.00f);
  colors[ImGuiCol_TabSelectedOverline] = ImVec4(0.70f, 0.30f, 0.30f, 1.00f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.70f, 0.00f, 0.30f, 1.00f);
  colors[ImGuiCol_TabSelected] = ImVec4(0.50f, 0.00f, 0.50f, 1.00f);
  colors[ImGuiCol_ResizeGrip] = ImVec4(1.00f, 1.00f, 1.00f, 0.85f);
  colors[ImGuiCol_ResizeGripHovered] = ImVec4(1.00f, 1.00f, 1.00f, 0.60f);
  colors[ImGuiCol_ResizeGripActive] = ImVec4(1.00f, 1.00f, 1.00f, 0.90f);
  colors[ImGuiCol_PlotLines] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
  colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
  colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
  colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
  colors[ImGuiCol_TextSelectedBg] = ImVec4(0.00f, 0.00f, 1.00f, 0.35f);
}

void editor_theme::ApplyEvoEngineDark() {
  ImGui::StyleColorsDark();
  ImNodes::StyleColorsDark();

  ImGuiStyle& style = ImGui::GetStyle();
  ImVec4* colors = style.Colors;

  colors[ImGuiCol_Text] = ImVec4(0.92f, 0.94f, 0.95f, 1.00f);
  colors[ImGuiCol_TextDisabled] = ImVec4(0.42f, 0.45f, 0.47f, 1.00f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.045f, 0.050f, 0.058f, 1.00f);
  colors[ImGuiCol_ChildBg] = ImVec4(0.055f, 0.062f, 0.072f, 1.00f);
  colors[ImGuiCol_PopupBg] = ImVec4(0.065f, 0.072f, 0.084f, 0.98f);
  colors[ImGuiCol_Border] = ImVec4(0.18f, 0.20f, 0.22f, 0.90f);
  colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.085f, 0.095f, 0.110f, 1.00f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.27f, 0.26f, 0.90f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.16f, 0.34f, 0.24f, 1.00f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.032f, 0.036f, 0.042f, 1.00f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(0.055f, 0.062f, 0.072f, 1.00f);
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.032f, 0.036f, 0.042f, 0.82f);
  colors[ImGuiCol_MenuBarBg] = ImVec4(0.032f, 0.036f, 0.042f, 1.00f);
  colors[ImGuiCol_ScrollbarBg] = ImVec4(0.040f, 0.045f, 0.052f, 0.90f);
  colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.18f, 0.20f, 0.22f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.27f, 0.30f, 0.32f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.34f, 0.38f, 0.40f, 1.00f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.46f, 0.78f, 0.32f, 1.00f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.40f, 0.68f, 0.28f, 1.00f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.56f, 0.88f, 0.36f, 1.00f);
  colors[ImGuiCol_Button] = ImVec4(0.13f, 0.145f, 0.16f, 1.00f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.20f, 0.29f, 0.28f, 1.00f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.25f, 0.39f, 0.30f, 1.00f);
  colors[ImGuiCol_Header] = ImVec4(0.105f, 0.118f, 0.136f, 1.00f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.18f, 0.27f, 0.26f, 1.00f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.20f, 0.34f, 0.26f, 1.00f);
  colors[ImGuiCol_Separator] = ImVec4(0.18f, 0.20f, 0.22f, 0.75f);
  colors[ImGuiCol_SeparatorHovered] = ImVec4(0.24f, 0.46f, 0.36f, 0.95f);
  colors[ImGuiCol_SeparatorActive] = ImVec4(0.35f, 0.68f, 0.42f, 1.00f);
  colors[ImGuiCol_ResizeGrip] = ImVec4(0.30f, 0.54f, 0.34f, 0.25f);
  colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.38f, 0.72f, 0.42f, 0.70f);
  colors[ImGuiCol_ResizeGripActive] = ImVec4(0.46f, 0.82f, 0.48f, 0.95f);
  colors[ImGuiCol_Tab] = ImVec4(0.050f, 0.056f, 0.065f, 1.00f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.16f, 0.22f, 0.22f, 1.00f);
  colors[ImGuiCol_TabSelected] = ImVec4(0.075f, 0.085f, 0.098f, 1.00f);
  colors[ImGuiCol_TabSelectedOverline] = ImVec4(0.42f, 0.78f, 0.26f, 1.00f);
  colors[ImGuiCol_TabDimmed] = ImVec4(0.040f, 0.045f, 0.052f, 1.00f);
  colors[ImGuiCol_TabDimmedSelected] = ImVec4(0.060f, 0.068f, 0.078f, 1.00f);
  colors[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.30f, 0.58f, 0.24f, 1.00f);
  colors[ImGuiCol_DockingPreview] = ImVec4(0.38f, 0.78f, 0.34f, 0.35f);
  colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.030f, 0.034f, 0.040f, 1.00f);
  colors[ImGuiCol_PlotLines] = ImVec4(0.62f, 0.66f, 0.68f, 1.00f);
  colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.52f, 0.88f, 0.32f, 1.00f);
  colors[ImGuiCol_PlotHistogram] = ImVec4(0.42f, 0.70f, 0.30f, 1.00f);
  colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.54f, 0.88f, 0.34f, 1.00f);
  colors[ImGuiCol_TableHeaderBg] = ImVec4(0.12f, 0.14f, 0.16f, 1.00f);
  colors[ImGuiCol_TableBorderStrong] = ImVec4(0.22f, 0.25f, 0.28f, 1.00f);
  colors[ImGuiCol_TableBorderLight] = ImVec4(0.16f, 0.18f, 0.20f, 1.00f);
  colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.00f, 1.00f, 1.00f, 0.035f);
  colors[ImGuiCol_TextLink] = ImVec4(0.50f, 0.78f, 0.48f, 1.00f);
  colors[ImGuiCol_TextSelectedBg] = ImVec4(0.34f, 0.62f, 0.36f, 0.35f);
  colors[ImGuiCol_DragDropTarget] = ImVec4(0.50f, 0.86f, 0.32f, 0.90f);
  colors[ImGuiCol_NavCursor] = ImVec4(0.48f, 0.82f, 0.36f, 1.00f);
  colors[ImGuiCol_NavWindowingHighlight] = ImVec4(0.92f, 0.94f, 0.95f, 0.70f);
  colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.35f);
  colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.55f);

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

  ImNodesStyle& node_style = ImNodes::GetStyle();
  node_style.NodeCornerRounding = 4.0f;
  node_style.NodeBorderThickness = 1.0f;
  node_style.LinkThickness = 2.5f;
  node_style.Colors[ImNodesCol_NodeBackground] = ColorU32(22, 25, 29);
  node_style.Colors[ImNodesCol_NodeBackgroundHovered] = ColorU32(32, 42, 40);
  node_style.Colors[ImNodesCol_NodeBackgroundSelected] = ColorU32(34, 52, 42);
  node_style.Colors[ImNodesCol_NodeOutline] = ColorU32(58, 64, 68);
  node_style.Colors[ImNodesCol_TitleBar] = ColorU32(14, 16, 19);
  node_style.Colors[ImNodesCol_TitleBarHovered] = ColorU32(28, 48, 38);
  node_style.Colors[ImNodesCol_TitleBarSelected] = ColorU32(34, 64, 42);
  node_style.Colors[ImNodesCol_Link] = ColorU32(86, 142, 88, 210);
  node_style.Colors[ImNodesCol_LinkHovered] = ColorU32(122, 205, 92);
  node_style.Colors[ImNodesCol_LinkSelected] = ColorU32(142, 230, 104);
  node_style.Colors[ImNodesCol_Pin] = ColorU32(98, 170, 86, 190);
  node_style.Colors[ImNodesCol_PinHovered] = ColorU32(144, 226, 104);
  node_style.Colors[ImNodesCol_BoxSelector] = ColorU32(102, 190, 96, 38);
  node_style.Colors[ImNodesCol_BoxSelectorOutline] = ColorU32(128, 220, 104, 160);
  node_style.Colors[ImNodesCol_GridBackground] = ColorU32(12, 14, 17, 230);
  node_style.Colors[ImNodesCol_GridLine] = ColorU32(150, 160, 150, 28);
  node_style.Colors[ImNodesCol_GridLinePrimary] = ColorU32(170, 190, 170, 46);
  node_style.Colors[ImNodesCol_MiniMapBackground] = ColorU32(10, 12, 14, 180);
  node_style.Colors[ImNodesCol_MiniMapBackgroundHovered] = ColorU32(14, 17, 19, 220);
  node_style.Colors[ImNodesCol_MiniMapOutline] = ColorU32(80, 92, 84, 140);
  node_style.Colors[ImNodesCol_MiniMapOutlineHovered] = ColorU32(120, 150, 126, 220);
  node_style.Colors[ImNodesCol_MiniMapNodeBackground] = ColorU32(140, 160, 144, 110);
  node_style.Colors[ImNodesCol_MiniMapNodeBackgroundHovered] = ColorU32(180, 220, 174, 220);
  node_style.Colors[ImNodesCol_MiniMapNodeBackgroundSelected] = ColorU32(150, 230, 114, 255);
  node_style.Colors[ImNodesCol_MiniMapNodeOutline] = ColorU32(180, 210, 180, 120);
  node_style.Colors[ImNodesCol_MiniMapLink] = node_style.Colors[ImNodesCol_Link];
  node_style.Colors[ImNodesCol_MiniMapLinkSelected] = node_style.Colors[ImNodesCol_LinkSelected];
  node_style.Colors[ImNodesCol_MiniMapCanvas] = ColorU32(180, 220, 180, 28);
  node_style.Colors[ImNodesCol_MiniMapCanvasOutline] = ColorU32(180, 220, 180, 200);
}
