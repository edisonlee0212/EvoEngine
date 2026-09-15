#include <gtest/gtest.h>
#include "Application.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "NodeGraphEditor.hpp"

using namespace evo_engine;

namespace {
class GraphOwner final : public IAsset {};
class NodeGraphEditorTest : public testing::Test {
 protected:
  Application application_;
  ApplicationContextScope scope_{application_};
  ImGuiContext* previous_gui_ = nullptr;
  ImGuiContext* gui_ = nullptr;
  ImNodesContext* previous_nodes_ = nullptr;
  ImNodesContext* nodes_ = nullptr;
  void SetUp() override {
    previous_gui_ = ImGui::GetCurrentContext();
    previous_nodes_ = ImNodes::GetCurrentContext();
    gui_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(gui_);
    nodes_ = ImNodes::CreateContext();
    ImNodes::SetCurrentContext(nodes_);
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {800, 600};
    io.DeltaTime = 1.0f / 60.0f;
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  }
  void TearDown() override {
    NodeGraphEditor::Clear();
    ImNodes::DestroyContext(nodes_);
    ImNodes::SetCurrentContext(previous_nodes_);
    ImGui::DestroyContext(gui_);
    ImGui::SetCurrentContext(previous_gui_);
  }
};
}  // namespace

TEST_F(NodeGraphEditorTest, ContextsAreOwnedByAssetsAndClearResetsSelectionState) {
  auto first = std::make_shared<GraphOwner>();
  auto second = std::make_shared<GraphOwner>();
  auto& first_context = NodeGraphEditor::GetContext(first);
  first_context.Panning = {9, 7};
  first_context.SelectedNodeIndices.push_back(4);
  auto& second_context = NodeGraphEditor::GetContext(second);
  EXPECT_NE(&first_context, &second_context);
  EXPECT_EQ(second_context.Panning.x, 0);
  EXPECT_TRUE(second_context.SelectedNodeIndices.empty());
  NodeGraphEditor::Clear();
  EXPECT_TRUE(NodeGraphEditor::GetContext(first).SelectedNodeIndices.empty());
  EXPECT_EQ(NodeGraphEditor::GetContext(first).Panning.x, 0);
}

TEST_F(NodeGraphEditorTest, DrawingPreservesSavedScreenCoordinatesAndCapturesEdits) {
  auto owner = std::make_shared<GraphOwner>();
  auto editor = std::make_shared<EditorLayer>();
  NodeGraph<int, int, int, int> graph;
  const auto node = graph.AllocateNode(0, 0);
  graph.SetNodePosition(node, {300, 200});
  graph.SetPanning({10, 20});
  for (int frame = 0; frame < 2; ++frame) {
    ImGui::NewFrame();
    ImGui::SetNextWindowPos({40.0f + frame * 20.0f, 60}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({600, 400}, ImGuiCond_Always);
    ImGui::Begin("Graph layout");
    NodeGraphEditor::Draw(
        graph, owner, ImGui::GetID("graph"), editor,
        [](int) {
          ImGui::TextUnformatted("Node");
        },
        [&](int handle) {
          if (frame == 1)
            ImNodes::SetNodeScreenSpacePos(handle, {340, 220});
        },
        [](int) {
        },
        [](int) {
        },
        [](int, int, int, int) {
        },
        [](const std::vector<int>&, const std::vector<int>&) {
        },
        [](ImVec2) {
        },
        [](int, int) {
        },
        [](int) {
        });
    ImGui::End();
    ImGui::Render();
    EXPECT_EQ(graph.GetNodePosition(node), frame == 0 ? glm::vec2(300, 200) : glm::vec2(340, 220));
    EXPECT_EQ(graph.GetPanning(), glm::vec2(10, 20));
  }
}
