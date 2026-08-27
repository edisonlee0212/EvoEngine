#include "EvoEngine_SDK_PCH.hpp"

#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {
std::string ReadSource(const std::filesystem::path& relative_path) {
  std::ifstream file(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path);
  EXPECT_TRUE(file.good()) << relative_path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

TEST(EditorConsole, CapturesCompatibleElapsedAndWallClockTimesWithBoundedHistory) {
  const auto header = ReadSource("EvoEngine_SDK/include/Layers/EditorLayer.hpp");
  const auto console = ReadSource("EvoEngine_SDK/src/Console.cpp");

  EXPECT_NE(header.find("double m_time = 0"), std::string::npos);
  EXPECT_NE(header.find("std::time_t m_timestamp = 0"), std::string::npos);
  EXPECT_NE(console.find("std::time(nullptr)"), std::string::npos);
  EXPECT_NE(console.find("kMaxConsoleMessages = 10000"), std::string::npos);
  EXPECT_NE(console.find("console_messages_.erase(editor_layer->console_messages_.begin())"), std::string::npos);
  EXPECT_NE(console.find("++editor_layer->console_message_revision_"), std::string::npos);
}

TEST(EditorConsole, ProvidesHazelStyleToolbarTableFiltersAndDetails) {
  const auto editor = ReadSource("EvoEngine_SDK/src/EditorLayer.cpp");

  EXPECT_NE(editor.find("ConsoleToolbar"), std::string::npos);
  EXPECT_NE(editor.find("Clear on Play"), std::string::npos);
  EXPECT_NE(editor.find("InfoButton"), std::string::npos);
  EXPECT_NE(editor.find("WarningButton"), std::string::npos);
  EXPECT_NE(editor.find("ErrorButton"), std::string::npos);
  EXPECT_NE(editor.find("ImGui::TableSetupColumn(\"Type\""), std::string::npos);
  EXPECT_NE(editor.find("ImGui::TableSetupColumn(\"Timestamp\""), std::string::npos);
  EXPECT_NE(editor.find("ImGui::TableSetupColumn(\"Message\""), std::string::npos);
  EXPECT_NE(editor.find("ConsoleMessagePreview"), std::string::npos);
  EXPECT_NE(editor.find("std::isspace(character)"), std::string::npos);
  EXPECT_NE(editor.find("ImGui::TextUnformatted(preview.c_str())"), std::string::npos);
  EXPECT_NE(editor.find("Console Message Details"), std::string::npos);
  EXPECT_NE(editor.find("Copy to Clipboard"), std::string::npos);
}

TEST(EditorConsole, PersistsClearOnPlayAndTracksBottomFollowing) {
  const auto editor = ReadSource("EvoEngine_SDK/src/EditorLayer.cpp");

  EXPECT_NE(editor.find("console_clear_on_play"), std::string::npos);
  EXPECT_NE(editor.find("scroll_y < console_previous_scroll_y_"), std::string::npos);
  EXPECT_NE(editor.find("scroll_y >= max_scroll_y - 1.0f"), std::string::npos);
  EXPECT_NE(editor.find("revision != console_rendered_revision_ || filter_changed"), std::string::npos);
  EXPECT_NE(editor.find("ImGui::SetScrollY(ImGui::GetScrollMaxY())"), std::string::npos);
}

TEST(EditorConsole, DefaultsToInfoOnlyAndUsesTwoProjectReadyMessages) {
  const auto header = ReadSource("EvoEngine_SDK/include/Layers/EditorLayer.hpp");
  const auto editor = ReadSource("EvoEngine_SDK/src/EditorLayer.cpp");
  const auto project = ReadSource("EvoEngine_SDK/src/ProjectManager.cpp");

  EXPECT_NE(header.find("bool enable_console_logs_ = true"), std::string::npos);
  EXPECT_NE(header.find("bool enable_console_errors_ = false"), std::string::npos);
  EXPECT_NE(header.find("bool enable_console_warnings_ = false"), std::string::npos);
  EXPECT_NE(project.find("EVOENGINE_LOG(\"Scanned all assets.\")"), std::string::npos);
  EXPECT_NE(editor.find("EVOENGINE_LOG(\"Scene is ready.\")"), std::string::npos);
  EXPECT_EQ(project.find("Found and loaded project"), std::string::npos);
  EXPECT_EQ(project.find("Created new start scene!"), std::string::npos);
}

TEST(EditorConsole, ClearsAtNewRuntimeAndProjectBoundaries) {
  const auto application = ReadSource("EvoEngine_SDK/src/Application.cpp");
  const auto project = ReadSource("EvoEngine_SDK/src/ProjectManager.cpp");

  const auto first_runtime_clear = application.find("ClearConsoleOnRuntimeStart()");
  ASSERT_NE(first_runtime_clear, std::string::npos);
  EXPECT_NE(application.find("ClearConsoleOnRuntimeStart()", first_runtime_clear + 1), std::string::npos);

  const auto project_clear = project.find("ClearConsoleMessages()");
  const auto project_metadata = project.find("LoadProjectLaunchMetadata(project_absolute_path)");
  ASSERT_NE(project_clear, std::string::npos);
  ASSERT_NE(project_metadata, std::string::npos);
  EXPECT_LT(project_clear, project_metadata);
}

TEST(EditorConsole, UsesGearIconForProjectBrowserSettings) {
  const auto browser = ReadSource("EvoEngine_SDK/src/ProjectContentBrowserPanel.cpp");

  EXPECT_NE(browser.find("draw_icon_button(\"ProjectBrowserSettings\", EditorLayer::FindIcon(\"SceneSettings\")"),
            std::string::npos);
  EXPECT_EQ(browser.find("ImGui::Button(\"...##ProjectBrowserSettings\""), std::string::npos);
}

TEST(EditorConsole, GivesEachPostProcessingEffectItsOwnDefaultButton) {
  const auto inspector = ReadSource("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp");

  EXPECT_NE(inspector.find("Apply default settings##AmbientOcclusion"), std::string::npos);
  EXPECT_NE(inspector.find("Apply default settings##ScreenSpaceReflection"), std::string::npos);
  EXPECT_NE(inspector.find("Apply default settings##AntiAliasing"), std::string::npos);
  EXPECT_NE(inspector.find("Apply default settings##Bloom"), std::string::npos);
  EXPECT_NE(inspector.find("Apply default settings##ToneMapping"), std::string::npos);
  EXPECT_EQ(inspector.find("ImGui::Button(\"Apply default settings\")"), std::string::npos);
}

TEST(EditorConsole, SharesAssetContextMenuBetweenThumbnailAndHierarchyViews) {
  const auto browser = ReadSource("EvoEngine_SDK/src/ProjectContentBrowserPanel.cpp");

  EXPECT_NE(browser.find("DrawAssetContextMenu(i.second, icon_tag)"), std::string::npos);
  EXPECT_NE(browser.find("DrawAssetContextMenu(i.second, tag)"), std::string::npos);
  EXPECT_NE(browser.find("ProjectContentBrowserPanel::DrawAssetContextMenu"), std::string::npos);
  EXPECT_NE(browser.find("ImGui::Button(\"Duplicate\")"), std::string::npos);
  EXPECT_NE(browser.find("ProjectManager::DeleteAsset(file->GetAssetHandle())"), std::string::npos);
}
