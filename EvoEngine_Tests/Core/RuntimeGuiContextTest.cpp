#include <gtest/gtest.h>
#include <imgui_internal.h>
#include "RuntimeGuiContext.hpp"

using namespace evo_engine;

namespace {
class RuntimeGuiContextTest : public testing::Test {
 protected:
  RuntimeGuiContext gui;
  void SetUp() override {
    ImGui::CreateContext();
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {800, 600};
    io.DeltaTime = 1.0f / 60;
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  }
  void TearDown() override {
    gui.ClearDrawLists();
    ImGui::DestroyContext();
  }
  void Begin() {
    gui.PrepareFrame();
    ImGui::NewFrame();
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({800, 600});
    ImGui::Begin("Editor camera host");
    gui.BeginView({100, 100}, {400, 300}, 42);
  }
  void End() {
    gui.FinishView();
    ImGui::End();
    ImGui::Render();
    gui.PartitionDrawData();
  }
};
}  // namespace

TEST_F(RuntimeGuiContextTest, PartitionsWindowChildPopupAndTooltipWithoutCopyingEditorLists) {
  for (int frame = 0; frame < 3; ++frame) {
    Begin();
    gui.BeginWindow("Debug");
    ImGui::TextUnformatted("Runtime");
    ImGui::GetWindowDrawList()->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
    ImGui::BeginChild("Child", {100, 50});
    ImGui::TextUnformatted("Child content");
    auto* child = ImGui::GetWindowDrawList();
    ImGui::EndChild();
    gui.OpenPopup("Popup");
    if (gui.BeginPopup("Popup")) {
      ImGui::TextUnformatted("Popup content");
      gui.EndPopup();
    }
    if (gui.BeginTooltip()) {
      ImGui::TextUnformatted("Runtime tooltip");
      gui.EndTooltip();
    }
    gui.EndWindow();
    End();
    const auto* runtime = gui.GetDrawData();
    ASSERT_NE(runtime, nullptr);
    if (frame < 2)
      continue;
    EXPECT_GE(runtime->CmdLists.Size, 4);
    bool child_found = false;
    bool callback_found = false;
    int vertex_count = 0;
    int index_count = 0;
    for (auto* list : runtime->CmdLists) {
      vertex_count += list->VtxBuffer.Size;
      index_count += list->IdxBuffer.Size;
      child_found |= list == child;
      for (auto* editor : ImGui::GetDrawData()->CmdLists)
        EXPECT_NE(editor, list);
      for (const auto& command : list->CmdBuffer) {
        callback_found |= command.UserCallback == ImDrawCallback_ResetRenderState;
        EXPECT_GE(command.ClipRect.x, 100);
        EXPECT_GE(command.ClipRect.y, 100);
        EXPECT_LE(command.ClipRect.z, 500);
        EXPECT_LE(command.ClipRect.w, 400);
      }
    }
    EXPECT_TRUE(child_found);
    EXPECT_TRUE(callback_found);
    EXPECT_EQ(runtime->TotalVtxCount, vertex_count);
    EXPECT_EQ(runtime->TotalIdxCount, index_count);
    EXPECT_EQ(runtime->Textures, &ImGui::GetPlatformIO().Textures);
    EXPECT_GT(ImGui::GetDrawData()->TotalVtxCount, 0);
  }
}

TEST_F(RuntimeGuiContextTest, ReusedTooltipReturnsToEditorOwnership) {
  for (int frame = 0; frame < 4; ++frame) {
    Begin();
    gui.BeginWindow("Debug");
    ImGui::TextUnformatted("Runtime");
    if (frame < 2) {
      if (gui.BeginTooltip()) {
        ImGui::TextUnformatted("Runtime tooltip");
        gui.EndTooltip();
      }
    }
    gui.EndWindow();
    if (frame >= 2)
      ImGui::SetTooltip("Editor tooltip");
    End();
    if (frame < 3)
      continue;
    for (auto* list : gui.GetDrawData()->CmdLists)
      EXPECT_NE(std::string(list->_OwnerName).find("##Tooltip"), 0u);
  }
}

TEST_F(RuntimeGuiContextTest, CollapsedWindowStillBalancesEnd) {
  Begin();
  ImGui::SetNextWindowCollapsed(true);
  EXPECT_FALSE(gui.BeginWindow("Collapsed"));
  gui.EndWindow();
  EXPECT_EQ(GImGui->CurrentWindow->Name, std::string("Editor camera host"));
  End();
}

TEST_F(RuntimeGuiContextTest, WindowMovesWithCameraWithoutChangingLocalOffset) {
  Begin();
  gui.BeginWindow("Debug");
  const auto original = ImGui::GetWindowPos();
  gui.EndWindow();
  End();
  ImGui::NewFrame();
  ImGui::Begin("Editor camera host");
  gui.BeginView({150, 130}, {400, 300}, 42);
  gui.BeginWindow("Debug");
  EXPECT_FLOAT_EQ(ImGui::GetWindowPos().x, original.x + 50);
  EXPECT_FLOAT_EQ(ImGui::GetWindowPos().y, original.y + 30);
  gui.EndWindow();
  End();
}

TEST_F(RuntimeGuiContextTest, ResizeHitPaddingDoesNotCaptureOutsideCamera) {
  for (int frame = 0; frame < 3; ++frame) {
    ImGui::GetIO().AddMousePosEvent(99, 150);
    Begin();
    gui.BeginWindow("Debug");
    ImGui::SetWindowPos({100, 100});
    gui.EndWindow();
    if (frame == 2)
      EXPECT_FALSE(gui.CapturesMouse());
    if (frame == 2)
      EXPECT_FALSE(gui.Owns(GImGui->HoveredWindow));
    End();
  }
}

TEST_F(RuntimeGuiContextTest, ModalDrawCommandsRemainInsideOverlay) {
  for (int frame = 0; frame < 4; ++frame) {
    ImGui::GetIO().AddMousePosEvent(50, 50);
    if (frame == 2 || frame == 3)
      ImGui::GetIO().AddMouseButtonEvent(0, frame == 2);
    Begin();
    ImGui::SetCursorScreenPos({30, 40});
    EXPECT_FALSE(ImGui::Button("Blocked editor button", {120, 40}));
    gui.BeginWindow("Debug");
    gui.OpenPopup("Modal");
    if (gui.BeginPopupModal("Modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::TextUnformatted("Modal content");
      gui.EndPopup();
    }
    gui.EndWindow();
    EXPECT_TRUE(gui.CapturesMouse());
    End();
    for (auto* list : gui.GetDrawData()->CmdLists)
      for (const auto& command : list->CmdBuffer) {
        EXPECT_GE(command.ClipRect.x, 100);
        EXPECT_GE(command.ClipRect.y, 100);
        EXPECT_LE(command.ClipRect.z, 500);
        EXPECT_LE(command.ClipRect.w, 400);
      }
  }
}

TEST_F(RuntimeGuiContextTest, OccludingEditorWindowReceivesMouseEvenWhenRuntimeWindowHasFocus) {
  for (int frame = 0; frame < 4; ++frame) {
    ImGui::GetIO().AddMousePosEvent(160, 160);
    Begin();
    gui.BeginWindow("Debug");
    auto* runtime = ImGui::GetCurrentWindow();
    ImGui::TextUnformatted("Runtime");
    gui.EndWindow();
    ImGui::SetNextWindowPos({150, 140});
    ImGui::SetNextWindowSize({200, 200});
    ImGui::Begin("Occluding editor panel");
    ImGui::TextUnformatted("Editor");
    auto* cover = ImGui::GetCurrentWindow();
    ImGui::End();
    ImGui::FocusWindow(runtime);
    if (frame == 3) {
      EXPECT_EQ(GImGui->HoveredWindow, cover);
      EXPECT_FALSE(gui.CapturesMouse());
    }
    End();
  }
}

TEST_F(RuntimeGuiContextTest, MenuAtCameraEdgeRespondsAtItsVisibleButton) {
  for (const float width : {700.0f, 400.0f}) {
    ImVec2 click{0, 0};
    bool clicked = false;
    for (int frame = 0; frame < 7; ++frame) {
      if (frame >= 3)
        ImGui::GetIO().AddMousePosEvent(click.x, click.y);
      if (frame == 4 || frame == 5)
        ImGui::GetIO().AddMouseButtonEvent(0, frame == 4);
      Begin();
      gui.BeginView({100, 100}, {width, 300}, 42);
      gui.BeginWindow("Debug");
      ImGui::SetCursorScreenPos({360, 200});
      gui.OpenPopup("Menu");
      ImGuiWindow* menu = nullptr;
      ImVec2 before{};
      ImVec2 button{};
      if (gui.BeginMenu("Menu")) {
        menu = ImGui::GetCurrentWindow();
        before = menu->Pos;
        clicked |= ImGui::Button("Click here", {180, 30});
        const auto rect = GImGui->LastItemData.Rect;
        button = {(rect.Min.x + rect.Max.x) / 2, (rect.Min.y + rect.Max.y) / 2};
        gui.EndMenu();
      }
      gui.EndWindow();
      End();
      if (menu)
        click = {button.x + menu->Pos.x - before.x, button.y + menu->Pos.y - before.y};
    }
    EXPECT_TRUE(clicked) << "Camera width: " << width;
  }
}

TEST_F(RuntimeGuiContextTest, ConfinedPopupChildUsesVisibleHitTarget) {
  bool clicked = false;
  for (int frame = 0; frame < 7; ++frame) {
    ImGui::GetIO().AddMousePosEvent(410, 330);
    if (frame == 4 || frame == 5)
      ImGui::GetIO().AddMouseButtonEvent(0, frame == 4);
    Begin();
    gui.BeginWindow("Debug");
    gui.OpenPopup("Edge popup");
    ImGui::SetNextWindowPos({480, 380});
    ImGui::SetNextWindowSize({160, 100});
    if (gui.BeginPopup("Edge popup")) {
      EXPECT_LE(ImGui::GetWindowPos().x + ImGui::GetWindowSize().x, 500);
      EXPECT_LE(ImGui::GetWindowPos().y + ImGui::GetWindowSize().y, 400);
      ImGui::BeginChild("Child button", {140, 65});
      clicked |= ImGui::Button("Hit", {120, 45});
      ImGui::EndChild();
      gui.EndPopup();
    }
    gui.EndWindow();
    End();
  }
  EXPECT_TRUE(clicked);
}

TEST_F(RuntimeGuiContextTest, DragCaptureSurvivesLeavingCameraAndEndsOnRelease) {
  for (int frame = 0; frame < 7; ++frame) {
    ImGui::GetIO().AddMousePosEvent(frame < 4 ? 150.0f : 600.0f, 160);
    if (frame == 3 || frame == 5)
      ImGui::GetIO().AddMouseButtonEvent(0, frame == 3);
    Begin();
    gui.BeginWindow("Debug");
    ImGui::Button("Drag", {150, 80});
    gui.EndWindow();
    if (frame == 3 || frame == 4)
      EXPECT_TRUE(gui.CapturesMouse());
    if (frame >= 5)
      EXPECT_FALSE(gui.CapturesMouse());
    End();
  }
}

TEST_F(RuntimeGuiContextTest, ClosedTransientHelpersLeaveWindowStackBalanced) {
  Begin();
  gui.BeginWindow("Debug");
  auto* parent = ImGui::GetCurrentWindow();
  EXPECT_FALSE(gui.BeginMenu("Disabled", false));
  EXPECT_FALSE(gui.BeginPopup("Closed popup"));
  EXPECT_FALSE(gui.BeginPopupModal("Closed modal"));
  EXPECT_EQ(ImGui::GetCurrentWindow(), parent);
  gui.EndWindow();
  End();
}

TEST_F(RuntimeGuiContextTest, SameModalTitleHasIndependentComponentAndEditorWindows) {
  Begin();
  ImGui::Begin("Options");
  const auto editor_id = ImGui::GetCurrentWindow()->ID;
  ImGui::End();
  ImGuiID previous_id = editor_id;
  for (const uint64_t owner : {1, 2}) {
    gui.BeginView({100, 100}, {400, 300}, owner);
    gui.BeginWindow("Debug");
    gui.OpenPopup("Options");
    ASSERT_TRUE(gui.BeginPopupModal("Options"));
    const auto id = ImGui::GetCurrentWindow()->ID;
    EXPECT_NE(id, previous_id);
    EXPECT_NE(id, editor_id);
    previous_id = id;
    ImGui::CloseCurrentPopup();
    gui.EndPopup();
    gui.EndWindow();
  }
  End();
}

TEST_F(RuntimeGuiContextTest, ChildSizesCopyAcrossOwnersWithoutLeakingIntoEditorIni) {
  Begin();
  gui.BeginWindow("Debug");
  gui.BeginChild("Resizable", {120, 70}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  auto* original = ImGui::GetCurrentWindow();
  ImGui::SetWindowSize({160, 90});
  gui.EndChild();
  gui.EndWindow();
  End();
  const auto saved = gui.SaveLayout("42");
  EXPECT_NE(saved.find("IsChild=1\nSize=160,90"), std::string::npos);
  EXPECT_EQ(std::string(ImGui::SaveIniSettingsToMemory()).find("RuntimeGui"), std::string::npos);
  Begin();
  gui.LoadLayout("43", saved);
  gui.SetOwner("43", "");
  // Empty asset scope in the proof uses window IDs without an asset prefix.
  gui.BeginView({100, 100}, {400, 300}, 43);
  gui.BeginWindow("Debug");
  gui.BeginChild("Resizable", {120, 70}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  EXPECT_NE(ImGui::GetCurrentWindow(), original);
  EXPECT_EQ(ImGui::GetWindowSize().x, 160);
  EXPECT_EQ(ImGui::GetWindowSize().y, 90);
  gui.EndChild();
  gui.EndWindow();
  End();
}

TEST_F(RuntimeGuiContextTest, NestedChildSettingsAndEncodedNamesRoundTripAtNewOrigin) {
  Begin();
  gui.BeginWindow("Display###stable]\nname");
  ImGui::SetWindowPos({140, 150});
  gui.BeginChild("parent/one", {180, 120}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  gui.BeginChild("nested]child", {90, 50}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  ImGui::SetWindowSize({105, 60});
  gui.EndChild();
  gui.EndChild();
  gui.EndWindow();
  End();
  const auto saved = gui.SaveLayout("42");
  EXPECT_NE(saved.find("stable%5D%0Aname"), std::string::npos);
  EXPECT_NE(saved.find("Pos=40,50"), std::string::npos);
  EXPECT_NE(saved.find("Size=105,60"), std::string::npos);
  Begin();
  gui.LoadLayout("99", saved);
  gui.BeginView({200, 180}, {400, 300}, 99);
  gui.BeginWindow("Renamed###stable]\nname");
  EXPECT_EQ(ImGui::GetWindowPos().x, 240);
  EXPECT_EQ(ImGui::GetWindowPos().y, 230);
  gui.BeginChild("parent/one", {180, 120}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  gui.BeginChild("nested]child", {90, 50}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  EXPECT_EQ(ImGui::GetWindowSize().x, 105);
  EXPECT_EQ(ImGui::GetWindowSize().y, 60);
  gui.EndChild();
  gui.EndChild();
  gui.EndWindow();
  End();
  EXPECT_EQ(gui.SaveLayout("99"), saved);
}

TEST_F(RuntimeGuiContextTest, ReloadingLayoutUpdatesAnExistingResizableChild) {
  Begin();
  gui.BeginWindow("Debug");
  gui.BeginChild("Panel", {120, 70}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  gui.EndChild();
  gui.EndWindow();
  End();
  gui.LoadLayout("42",
                 "[Window][Debug]\nPos=20,25\nSize=300,220\nCollapsed=0\n\n"
                 "[Window][Debug/Child/Panel]\nIsChild=1\nSize=155,85\n\n");
  Begin();
  gui.BeginWindow("Debug");
  gui.BeginChild("Panel", {120, 70}, ImGuiChildFlags_ResizeX | ImGuiChildFlags_ResizeY);
  EXPECT_EQ(ImGui::GetWindowSize().x, 155);
  EXPECT_EQ(ImGui::GetWindowSize().y, 85);
  EXPECT_TRUE(ImGui::GetCurrentWindow()->ChildFlags & ImGuiChildFlags_ResizeY);
  gui.EndChild();
  gui.EndWindow();
  End();
  const auto saved = gui.SaveLayout("42");
  ImGui::LoadIniSettingsFromMemory(saved.c_str());
  const auto* native = ImGui::FindWindowSettingsByID(ImHashStr("Debug/Child/Panel"));
  ASSERT_NE(native, nullptr);
  EXPECT_TRUE(native->IsChild);
  EXPECT_EQ(native->Size.x, 155);
  EXPECT_EQ(native->Size.y, 85);
}

TEST_F(RuntimeGuiContextTest, CameraDrawListsSurroundWindowsAndLeaveEditorListsUntouched) {
  for (int frame = 0; frame < 3; ++frame) {
    Begin();
    auto* editor_foreground = ImGui::GetForegroundDrawList();
    editor_foreground->AddRectFilled({0, 0}, {10, 10}, IM_COL32_WHITE);
    auto* background = gui.GetBackgroundDrawList();
    background->PushClipRectFullScreen();
    background->AddRectFilled({0, 0}, {800, 600}, IM_COL32_WHITE);
    background->PopClipRect();
    auto* foreground = gui.GetForegroundDrawList();
    foreground->AddText({120, 120}, IM_COL32_WHITE, "Camera annotation");
    gui.SetOwner("another component", "shared asset");
    EXPECT_EQ(gui.GetForegroundDrawList(), foreground);
    foreground->AddLine({110, 110}, {160, 160}, IM_COL32_WHITE);
    gui.BeginWindow("Debug");
    ImGui::TextUnformatted("Between camera drawing layers");
    auto* window = ImGui::GetWindowDrawList();
    gui.EndWindow();
    End();
    const auto* data = gui.GetDrawData();
    ASSERT_GE(data->CmdLists.Size, 2);
    EXPECT_EQ(data->CmdLists.front(), background);
    EXPECT_EQ(data->CmdLists.back(), foreground);
    if (frame > 0) {
      ASSERT_GE(data->CmdLists.Size, 3);
      EXPECT_EQ(data->CmdLists[1], window);
    }
    int vertices = 0, indices = 0;
    for (const auto* list : data->CmdLists) {
      EXPECT_NE(list, editor_foreground);
      vertices += list->VtxBuffer.Size;
      indices += list->IdxBuffer.Size;
      for (const auto& command : list->CmdBuffer) {
        EXPECT_GE(command.ClipRect.x, 100);
        EXPECT_GE(command.ClipRect.y, 100);
        EXPECT_LE(command.ClipRect.z, 500);
        EXPECT_LE(command.ClipRect.w, 400);
      }
      for (const auto* editor_list : ImGui::GetDrawData()->CmdLists)
        EXPECT_NE(editor_list, list);
    }
    EXPECT_EQ(data->TotalVtxCount, vertices);
    EXPECT_EQ(data->TotalIdxCount, indices);
    EXPECT_TRUE(ImGui::GetDrawData()->CmdLists.contains(editor_foreground));
  }
}

TEST_F(RuntimeGuiContextTest, CameraDrawingWithoutWindowsDoesNotCaptureOrSurviveNextFrame) {
  Begin();
  auto* foreground = gui.GetForegroundDrawList();
  foreground->AddCircleFilled({200, 200}, 10, IM_COL32_WHITE);
  End();
  EXPECT_EQ(gui.GetDrawData()->CmdLists.Size, 1);
  EXPECT_FALSE(gui.CapturesMouse());
  EXPECT_FALSE(gui.CapturesKeyboard());
  EXPECT_TRUE(gui.SaveLayout("42").empty());

  Begin();
  End();
  EXPECT_EQ(gui.GetDrawData()->TotalVtxCount, 0);

  Begin();
  gui.BeginView({200, 150}, {300, 200}, 42);
  foreground = gui.GetForegroundDrawList();
  EXPECT_EQ(foreground->VtxBuffer.Size, 0);
  foreground->AddRectFilled({200, 150}, {220, 170}, IM_COL32_WHITE);
  End();
  EXPECT_EQ(foreground->CmdBuffer.front().ClipRect.x, 200);
  EXPECT_EQ(foreground->CmdBuffer.front().ClipRect.y, 150);
  EXPECT_EQ(foreground->CmdBuffer.front().ClipRect.z, 500);
  EXPECT_EQ(foreground->CmdBuffer.front().ClipRect.w, 350);
}

TEST_F(RuntimeGuiContextTest, FractionalCameraOriginsDoNotDriftAcrossLayoutReloads) {
  const std::string saved = "[Window][Debug]\nPos=40,50\nSize=200,180\nCollapsed=0\n\n";
  for (const auto origin : {ImVec2{100.25f, 100.75f}, ImVec2{200.75f, 180.25f}, ImVec2{-100.75f, -90.25f}}) {
    Begin();
    gui.LoadLayout("42", saved);
    gui.BeginView(origin, {400, 300}, 42);
    gui.BeginWindow("Debug");
    gui.EndWindow();
    End();
    EXPECT_EQ(gui.SaveLayout("42"), saved);
  }
}
