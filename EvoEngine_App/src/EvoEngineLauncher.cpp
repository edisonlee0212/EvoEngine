#include "Application.hpp"
#include "AssetManager.hpp"
#include "EditorTheme.hpp"
#include "ILayer.hpp"
#include "ImGuiLayer.hpp"
#include "LauncherUtils.hpp"
#include "PackageManager.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Serialization.hpp"
#include "Texture2D.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <unordered_map>
#include <vector>

#ifdef EVOENGINE_WINDOWS
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#  include <shellapi.h>
#endif

using namespace evo_engine;

namespace {
std::filesystem::path LauncherSettingsPath() {
#ifdef EVOENGINE_WINDOWS
  if (const char* local_app_data = std::getenv("LOCALAPPDATA")) {
    return std::filesystem::path(local_app_data) / "EvoEngine" / "EditorSettings.yaml";
  }
  if (const char* user_profile = std::getenv("USERPROFILE")) {
    return std::filesystem::path(user_profile) / "AppData" / "Local" / "EvoEngine" / "EditorSettings.yaml";
  }
#else
  if (const char* config_home = std::getenv("XDG_CONFIG_HOME")) {
    return std::filesystem::path(config_home) / "EvoEngine" / "EditorSettings.yaml";
  }
  if (const char* home = std::getenv("HOME")) {
    return std::filesystem::path(home) / ".config" / "EvoEngine" / "EditorSettings.yaml";
  }
#endif
  return std::filesystem::absolute("EvoEngineEditorSettings.yaml");
}

std::filesystem::path CurrentExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  std::wstring path(MAX_PATH, L'\0');
  const DWORD size = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
  if (size == 0 || size == path.size()) {
    return std::filesystem::absolute("EvoEngineLauncher.exe");
  }
  path.resize(size);
  return path;
#else
  return std::filesystem::absolute("EvoEngineLauncher");
#endif
}

std::filesystem::path DefaultResourcesPath() {
  const std::array candidates = {CurrentExecutablePath().parent_path() / "DefaultResources",
                                 std::filesystem::current_path() / "DefaultResources",
                                 std::filesystem::current_path() / "EvoEngine_SDK/Internals/DefaultResources"};
  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return candidates.front();
}

void AppendTestLog(const std::string& line) {
  const char* log_path = std::getenv("EVOENGINE_LAUNCHER_TEST_LOG");
  if (!log_path || std::string(log_path).empty()) {
    return;
  }
  std::ofstream log_file(log_path, std::ios::app);
  log_file << line << "\n";
}

ImVec4 ColorTextMuted() {
  return ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
}

float BackgroundLuminance() {
  const ImVec4 background = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
  return background.x * 0.299f + background.y * 0.587f + background.z * 0.114f;
}

bool UsesLightBackground() {
  return BackgroundLuminance() > 0.5f;
}

ImVec4 ColorSuccess() {
  return UsesLightBackground() ? ImVec4(0.10f, 0.48f, 0.22f, 1.0f) : ImVec4(0.35f, 0.78f, 0.48f, 1.0f);
}

ImVec4 ColorWarning() {
  return UsesLightBackground() ? ImVec4(0.70f, 0.38f, 0.04f, 1.0f) : ImVec4(0.95f, 0.67f, 0.24f, 1.0f);
}

ImVec4 ColorError() {
  return UsesLightBackground() ? ImVec4(0.74f, 0.13f, 0.13f, 1.0f) : ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
}

ImVec4 ColorPanel() {
  return ImGui::GetStyleColorVec4(ImGuiCol_ChildBg);
}

ImVec4 ColorPanelAlt() {
  return ImGui::GetStyleColorVec4(ImGuiCol_FrameBg);
}

ImVec4 ColorBorder() {
  return ImGui::GetStyleColorVec4(ImGuiCol_Border);
}

ImVec4 ColorSelectedPanel() {
  return ImGui::GetStyleColorVec4(ImGuiCol_Header);
}

ImVec4 ColorSelectedBorder() {
  return ImGui::GetStyleColorVec4(ImGuiCol_TextLink);
}

constexpr const char* kRecentProjectsWindow = "Recent Projects";
constexpr const char* kNewProjectWindow = "New Project";
constexpr float kCustomTitleBarHeight = 57.0f;
constexpr float kTitleBarLogoSize = 38.0f;
constexpr float kTitleBarLogoX = 10.0f;
constexpr float kTitleBarButtonsAreaWidth = 94.0f;
constexpr float kTitleBarButtonSize = 14.0f;
constexpr ImU32 kTitleBarColor = IM_COL32(21, 21, 21, 255);
constexpr ImU32 kTitleBarText = IM_COL32(192, 192, 192, 255);
constexpr ImU32 kTitleBarTextDarker = IM_COL32(128, 128, 128, 255);

ImU32 MultiplyColor(const ImU32 color, const float multiplier) {
  ImVec4 value = ImGui::ColorConvertU32ToFloat4(color);
  value.x = std::clamp(value.x * multiplier, 0.0f, 1.0f);
  value.y = std::clamp(value.y * multiplier, 0.0f, 1.0f);
  value.z = std::clamp(value.z * multiplier, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(value);
}

std::shared_ptr<Texture2D> FindIcon(const std::unordered_map<std::string, std::shared_ptr<Texture2D>>& icons,
                                    const std::string& name) {
  if (const auto search = icons.find(name); search != icons.end()) {
    return search->second;
  }
  return {};
}

bool DrawFittedImage(const std::shared_ptr<Texture2D>& icon, const ImVec2 min, const ImVec2 max, const ImU32 tint) {
  if (!icon || icon->GetImTextureId() == 0) {
    return false;
  }
  const glm::uvec2 resolution = icon->GetResolution();
  if (resolution.x == 0 || resolution.y == 0) {
    return false;
  }

  const ImVec2 bounds(max.x - min.x, max.y - min.y);
  const float scale =
      std::min(bounds.x / static_cast<float>(resolution.x), bounds.y / static_cast<float>(resolution.y));
  const ImVec2 size(static_cast<float>(resolution.x) * scale, static_cast<float>(resolution.y) * scale);
  const ImVec2 image_min(min.x + (bounds.x - size.x) * 0.5f, min.y + (bounds.y - size.y) * 0.5f);
  ImGui::GetWindowDrawList()->AddImage(icon->GetImTextureId(), image_min,
                                       ImVec2(image_min.x + size.x, image_min.y + size.y), ImVec2(0, 1), ImVec2(1, 0),
                                       tint);
  return true;
}

void DrawFallbackLogo(const ImVec2 min, const float size, const ImU32 tint) {
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 center(min.x + size * 0.5f, min.y + size * 0.5f);
  const float radius = size * 0.36f;
  std::array<ImVec2, 6> points{};
  for (size_t i = 0; i < points.size(); ++i) {
    const float angle = -IM_PI * 0.5f + IM_PI / 3.0f * static_cast<float>(i);
    points[i] = ImVec2(center.x + std::cos(angle) * radius, center.y + std::sin(angle) * radius);
  }
  draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), tint, ImDrawFlags_Closed, 3.0f);
  draw_list->AddLine(center, points[0], tint, 3.0f);
  draw_list->AddLine(center, points[2], tint, 3.0f);
  draw_list->AddLine(center, points[4], tint, 3.0f);
}

enum class TitleBarGlyph { Minimize, Maximize, Restore, Close };

void DrawFallbackTitleBarGlyph(const ImVec2 min, const ImVec2 max, const TitleBarGlyph glyph, const ImU32 tint) {
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const float thickness = 1.6f;
  switch (glyph) {
    case TitleBarGlyph::Minimize: {
      const float y = (min.y + max.y) * 0.5f;
      draw_list->AddLine(ImVec2(min.x, y), ImVec2(max.x, y), tint, thickness);
      break;
    }
    case TitleBarGlyph::Maximize:
      draw_list->AddRect(min, max, tint, 0.0f, 0, thickness);
      break;
    case TitleBarGlyph::Restore:
      draw_list->AddRect(ImVec2(min.x + 3.0f, min.y), max, tint, 0.0f, 0, thickness);
      draw_list->AddRect(ImVec2(min.x, min.y + 3.0f), ImVec2(max.x - 3.0f, max.y), tint, 0.0f, 0, thickness);
      break;
    case TitleBarGlyph::Close:
      draw_list->AddLine(ImVec2(min.x + 1.0f, min.y + 1.0f), ImVec2(max.x - 1.0f, max.y - 1.0f), tint, thickness);
      draw_list->AddLine(ImVec2(max.x - 1.0f, min.y + 1.0f), ImVec2(min.x + 1.0f, max.y - 1.0f), tint, thickness);
      break;
  }
}

bool DrawTitleBarImageButton(const char* id, const std::shared_ptr<Texture2D>& icon, const ImVec2 screen_position,
                             const bool close_button, const TitleBarGlyph fallback_glyph) {
  ImGui::SetCursorScreenPos(screen_position);
  const ImVec2 button_max(screen_position.x + kTitleBarButtonSize, screen_position.y + kTitleBarButtonSize);
  const bool clicked = ImGui::InvisibleButton(id, ImVec2(kTitleBarButtonSize, kTitleBarButtonSize));
  ImU32 tint = close_button ? kTitleBarText : MultiplyColor(kTitleBarText, 0.9f);
  if (ImGui::IsItemActive()) {
    tint = kTitleBarTextDarker;
  } else if (ImGui::IsItemHovered()) {
    tint = close_button ? MultiplyColor(kTitleBarText, 1.4f) : MultiplyColor(kTitleBarText, 1.2f);
  }
  if (!DrawFittedImage(icon, screen_position, button_max, tint)) {
    DrawFallbackTitleBarGlyph(screen_position, button_max, fallback_glyph, tint);
  }
  return clicked;
}

std::filesystem::path EditorExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return CurrentExecutablePath().parent_path() / "EvoEngineEditor.exe";
#else
  return CurrentExecutablePath().parent_path() / "EvoEngineEditor";
#endif
}

bool LaunchEditorProcess(const std::filesystem::path& project_path, std::string& error) {
  const auto editor_path = EditorExecutablePath();
  if (!std::filesystem::exists(editor_path)) {
    error = "Could not find EvoEngineEditor next to EvoEngineLauncher.";
    return false;
  }

#ifdef EVOENGINE_WINDOWS
  std::wstring command_line =
      L"\"" + editor_path.wstring() + L"\" --project \"" + std::filesystem::absolute(project_path).wstring() + L"\"";
  STARTUPINFOW startup_info{};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info{};
  const auto working_directory = editor_path.parent_path().wstring();
  if (!CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, FALSE, 0, nullptr, working_directory.c_str(),
                      &startup_info, &process_info)) {
    error = "Failed to launch EvoEngineEditor.";
    return false;
  }
  CloseHandle(process_info.hProcess);
  CloseHandle(process_info.hThread);
  return true;
#else
  const auto command =
      "\"" + editor_path.string() + "\" --project \"" + std::filesystem::absolute(project_path).string() + "\" &";
  if (std::system(command.c_str()) != 0) {
    error = "Failed to launch EvoEngineEditor.";
    return false;
  }
  return true;
#endif
}

bool RevealProjectInExplorer(const std::filesystem::path& project_path, std::string& error) {
  const auto absolute_path = std::filesystem::absolute(project_path);
  if (!std::filesystem::exists(absolute_path) || std::filesystem::is_directory(absolute_path)) {
    error = "Project file is missing.";
    return false;
  }
#ifdef EVOENGINE_WINDOWS
  const auto parameters = L"/select,\"" + absolute_path.wstring() + L"\"";
  const auto result = ShellExecuteW(nullptr, L"open", L"explorer.exe", parameters.c_str(), nullptr, SW_SHOWNORMAL);
  if (reinterpret_cast<intptr_t>(result) <= 32) {
    error = "Failed to reveal project in Explorer.";
    return false;
  }
  return true;
#else
  error = "Reveal is only supported on Windows.";
  return false;
#endif
}

class LauncherLayer final : public ILayer {
 protected:
  void OnCreate() override {
    editor_theme::ApplyDefault();
    if (const char* parent_folder = std::getenv("EVOENGINE_LAUNCHER_TEST_PARENT_FOLDER")) {
      parent_folder_ = parent_folder;
    }
    RefreshPackageAvailability();
    LoadRecentProjects();
    AppendRecentProjectCountTestLog();
    AppendPackageAvailabilityTestLog();
    AppendTestLog("mode:hub");
    if (const char* open_project = std::getenv("EVOENGINE_LAUNCHER_TEST_OPEN_PROJECT")) {
      pending_test_open_project_ = open_project;
    }
  }

  void PreUpdate() override {
    if (!pending_test_open_project_.empty()) {
      const auto project_path = pending_test_open_project_;
      pending_test_open_project_.clear();
      OpenProject(project_path);
      return;
    }
    const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
    if (window_layer && window_layer->UsesCustomTitleBar()) {
      DrawWorkspace(kCustomTitleBarHeight);
      DrawCustomTitleBar();
    } else {
      DrawWorkspace(0.0f);
    }
  }

 private:
  char project_name_[256] = {};
  std::filesystem::path parent_folder_;
  std::string create_error_;
  std::string launch_error_;
  std::vector<std::filesystem::path> recent_project_paths_;
  std::vector<AvailablePackageInfo> available_packages_;
  std::vector<std::string> selected_startup_runtime_packages_;
  launcher::PackageAvailability package_availability_;
  std::unordered_map<std::string, std::shared_ptr<Texture2D>> title_bar_icons_;
  bool dock_layout_dirty_ = true;
  ImGuiID dock_space_id_ = 0;
  std::filesystem::path pending_test_open_project_;

  void LoadTitleBarIcons() {
    const auto default_resources = DefaultResourcesPath();
    auto load_icon = [&](const std::string& name, const std::filesystem::path& path) {
      auto icon = AssetManager::CreateTemporaryAsset<Texture2D>();
      const bool loaded = Serialization::LoadAsset(*icon, path);
      icon->UnsafeUploadDataImmediately();
      const glm::uvec2 resolution = icon->GetResolution();
      AppendTestLog("titlebar-icon:" + name + ":" + (loaded ? "loaded" : "load-failed") + ":" +
                    std::to_string(resolution.x) + "x" + std::to_string(resolution.y) + ":" +
                    (icon->GetImTextureId() != 0 ? "texture-id" : "zero-texture-id") + ":" + path.string());
      title_bar_icons_[name] = std::move(icon);
    };

    load_icon("TitleBarLogo", default_resources / "Editor/TitleBar/EvoEngine64White.png");
    load_icon("WindowMinimize", default_resources / "Editor/Window/Minimize.png");
    load_icon("WindowMaximize", default_resources / "Editor/Window/Maximize.png");
    load_icon("WindowRestore", default_resources / "Editor/Window/Restore.png");
    load_icon("WindowClose", default_resources / "Editor/Window/Close.png");
  }

  void DrawCustomTitleBar() {
    const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
    if (!window_layer) {
      return;
    }
    if (title_bar_icons_.empty()) {
      LoadTitleBarIcons();
    }

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kCustomTitleBarHeight));
    ImGui::SetNextWindowViewport(viewport->ID);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                                       ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 5.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::ColorConvertU32ToFloat4(kTitleBarColor));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kTitleBarText));
    if (ImGui::Begin("Launcher Custom Title Bar", nullptr, flags)) {
      const auto draw_list = ImGui::GetWindowDrawList();
      const ImVec2 titlebar_min = ImGui::GetWindowPos();
      const ImVec2 titlebar_max(titlebar_min.x + ImGui::GetWindowWidth(), titlebar_min.y + kCustomTitleBarHeight);
      draw_list->AddRectFilled(titlebar_min, titlebar_max, kTitleBarColor);

      const ImVec2 logo_min(titlebar_min.x + kTitleBarLogoX,
                            titlebar_min.y + (kCustomTitleBarHeight - kTitleBarLogoSize) * 0.5f);

      const float controls_x = ImGui::GetWindowWidth() - kTitleBarButtonsAreaWidth;

      if (!DrawFittedImage(FindIcon(title_bar_icons_, "TitleBarLogo"), logo_min,
                           ImVec2(logo_min.x + kTitleBarLogoSize, logo_min.y + kTitleBarLogoSize), IM_COL32_WHITE)) {
        DrawFallbackLogo(logo_min, kTitleBarLogoSize, IM_COL32_WHITE);
      }

      const float drag_start_x = kTitleBarLogoX + kTitleBarLogoSize + 18.0f;
      const float drag_width = controls_x - drag_start_x;
      if (drag_width > 0.0f) {
        window_layer->SetCustomTitleBarDragRegion(glm::vec4(drag_start_x, 0.0f, drag_width, kCustomTitleBarHeight));
      } else {
        window_layer->ClearCustomTitleBarDragRegion();
      }

      ImGui::PushClipRect(titlebar_min, titlebar_max, false);
      const float button_y = titlebar_min.y + (kCustomTitleBarHeight - kTitleBarButtonSize) * 0.5f;
      float button_x = titlebar_min.x + ImGui::GetWindowWidth() - 18.0f - kTitleBarButtonSize;
      if (DrawTitleBarImageButton("Close##Launcher", FindIcon(title_bar_icons_, "WindowClose"),
                                  ImVec2(button_x, button_y), true, TitleBarGlyph::Close)) {
        ApplicationContext::Get().End();
      }
      button_x -= 15.0f + kTitleBarButtonSize;
      if (DrawTitleBarImageButton(
              window_layer->IsWindowMaximized() ? "Restore##Launcher" : "Maximize##Launcher",
              FindIcon(title_bar_icons_, window_layer->IsWindowMaximized() ? "WindowRestore" : "WindowMaximize"),
              ImVec2(button_x, button_y), false,
              window_layer->IsWindowMaximized() ? TitleBarGlyph::Restore : TitleBarGlyph::Maximize)) {
        ApplicationContext::Get().QueueEndOfLoopAction([window_layer]() {
          window_layer->ToggleMaximized();
        });
      }
      button_x -= 17.0f + kTitleBarButtonSize;
      if (DrawTitleBarImageButton("Minimize##Launcher", FindIcon(title_bar_icons_, "WindowMinimize"),
                                  ImVec2(button_x, button_y), false, TitleBarGlyph::Minimize)) {
        ApplicationContext::Get().QueueEndOfLoopAction([window_layer]() {
          window_layer->MinimizeWindow();
        });
      }
      ImGui::PopClipRect();
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(4);
  }

  void DrawWorkspace(const float top_offset) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImVec2 pos = top_offset > 0.0f ? ImVec2(viewport->Pos.x, viewport->Pos.y + top_offset) : viewport->WorkPos;
    const ImVec2 size = top_offset > 0.0f ? ImVec2(viewport->Size.x, std::max(1.0f, viewport->Size.y - top_offset))
                                          : viewport->WorkSize;
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowViewport(viewport->ID);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
                                       ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    if (ImGui::Begin("Launcher Workspace", nullptr, flags)) {
      ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
      ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
      ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 8.0f));
      ImGui::PushStyleColor(ImGuiCol_Border, ColorBorder());
      ImGui::SetCursorPos(ImVec2(0.0f, 0.0f));
      if (ImGui::BeginChild("LauncherDockHost", ImVec2(0.0f, 0.0f), false,
                            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
        DrawLauncherDockspace();
      }
      ImGui::EndChild();
      ImGui::PopStyleColor();
      ImGui::PopStyleVar(3);
    }
    ImGui::End();
    ImGui::PopStyleVar(3);

    DrawProjectHub();
  }

  void DrawLauncherDockspace() {
    dock_space_id_ = ImGui::GetID("LauncherDockSpace");
    const ImVec2 dock_size = ImGui::GetContentRegionAvail();
    if ((dock_layout_dirty_ || ImGui::DockBuilderGetNode(dock_space_id_) == nullptr) && dock_size.x > 1.0f &&
        dock_size.y > 1.0f) {
      RebuildLauncherDockLayout(dock_size);
    }
    ImGui::DockSpace(dock_space_id_, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);
  }

  void RebuildLauncherDockLayout(const ImVec2& dock_size) {
    ImGui::DockBuilderRemoveNode(dock_space_id_);
    ImGui::DockBuilderAddNode(dock_space_id_, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dock_space_id_, dock_size);
    ImGuiID main_node = dock_space_id_;
    const ImGuiID recent_node = ImGui::DockBuilderSplitNode(main_node, ImGuiDir_Left, 0.34f, nullptr, &main_node);
    ImGui::DockBuilderDockWindow(kRecentProjectsWindow, recent_node);
    ImGui::DockBuilderDockWindow(kNewProjectWindow, main_node);
    ImGui::DockBuilderFinish(dock_space_id_);
    dock_layout_dirty_ = false;
  }

  void DrawProjectHub() {
    DrawDockedPanel(kRecentProjectsWindow, [this](const float width) {
      DrawRecentProjectsPanel(width);
    });
    DrawDockedPanel(kNewProjectWindow, [this](const float width) {
      DrawCreateProjectForm(ImVec2(width, 0.0f));
    });
  }

  template <typename DrawBody>
  void DrawDockedPanel(const char* title, DrawBody&& draw_body) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 12.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ColorPanel());
    ImGui::PushStyleColor(ImGuiCol_Border, ColorBorder());
    if (ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoCollapse)) {
      draw_body(std::max(ImGui::GetContentRegionAvail().x, 260.0f));
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
  }

  void DrawRecentProjectsPanel(const float content_width) {
    if (!launch_error_.empty()) {
      ImGui::TextColored(ColorError(), "%s", launch_error_.c_str());
    }
    ImGui::PushID("RecentOpenProject");
    FileUtils::OpenFile(
        "Open Project", "Project", {".eveproj"},
        [this](const std::filesystem::path& path) {
          OpenProject(path);
        },
        false);
    ImGui::PopID();
    ImGui::SameLine();
    if (ImGui::Button("Refresh", ImVec2(92.0f, 28.0f))) {
      RefreshRecentProjects();
    }
    ImGui::Spacing();
    if (recent_project_paths_.empty()) {
      ImGui::Dummy(ImVec2(0.0f, 24.0f));
      ImGui::TextUnformatted("No recent projects");
      ImGui::TextColored(ColorTextMuted(), "Open a .eveproj file to add it here.");
      ImGui::Spacing();
      ImGui::PushID("EmptyRecentOpenProject");
      FileUtils::OpenFile(
          "Open Project", "Project", {".eveproj"},
          [this](const std::filesystem::path& path) {
            OpenProject(path);
          },
          false);
      ImGui::PopID();
      return;
    }

    if (ImGui::BeginChild("RecentProjectRows", ImVec2(content_width - 22.0f, 0.0f), false)) {
      for (size_t i = 0; i < recent_project_paths_.size(); ++i) {
        if (DrawRecentProjectRow(i, recent_project_paths_[i], content_width - 36.0f)) {
          break;
        }
      }
    }
    ImGui::EndChild();
  }

  bool DrawRecentProjectRow(const size_t index, const std::filesystem::path& path, const float row_width) {
    const bool available = std::filesystem::exists(path) && !std::filesystem::is_directory(path);
    const auto metadata = ProjectManager::LoadProjectLaunchMetadata(path);
    auto label = metadata.application_name;
    if (label.empty() || label == "EvoEngine Editor") {
      label = path.stem().string();
    }
    if (!available) {
      label += " (missing)";
    }

    ImGui::PushID(static_cast<int>(index));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ColorPanelAlt());
    ImGui::BeginChild("RecentRow", ImVec2(row_width, 124.0f), true);
    ImGui::TextUnformatted(label.c_str());
    ImGui::TextColored(ColorTextMuted(), "%s", path.parent_path().string().c_str());
    if (!metadata.startup_runtime_packages.empty()) {
      const auto missing_packages = launcher::MissingPackages(package_availability_, metadata.startup_runtime_packages);
      const auto package_text = launcher::JoinPackages(metadata.startup_runtime_packages);
      if (missing_packages.empty()) {
        ImGui::TextColored(ColorSuccess(), "Packages: %s", package_text.c_str());
      } else {
        ImGui::TextColored(ColorWarning(), "Missing: %s", launcher::JoinPackages(missing_packages).c_str());
      }
    }
    if (!available) {
      ImGui::TextColored(ColorWarning(), "Project file is missing.");
    }
    ImGui::Spacing();
    ImGui::BeginDisabled(!available);
    const bool open_project = ImGui::Button("Open", ImVec2(58.0f, 26.0f));
    ImGui::SameLine();
    const bool reveal_project = ImGui::Button("Reveal", ImVec2(66.0f, 26.0f));
    ImGui::EndDisabled();
    ImGui::SameLine();
    const bool remove_project = ImGui::Button("Remove", ImVec2(74.0f, 26.0f));
    if (ImGui::IsWindowHovered()) {
      ImGui::SetTooltip("%s", path.string().c_str());
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopID();
    if (open_project) {
      OpenProject(path);
      return true;
    }
    if (reveal_project) {
      RevealProject(path);
    }
    if (remove_project) {
      RemoveRecentProject(index);
      return true;
    }
    return false;
  }

  void DrawCreateProjectForm(const ImVec2& content_size) {
    ImGui::SetNextItemWidth(content_size.x);
    ImGui::InputText("Project Name", project_name_, sizeof(project_name_));
    FileUtils::OpenFolder(
        "Parent Folder",
        [this](const std::filesystem::path& path) {
          parent_folder_ = path;
          create_error_.clear();
        },
        false);

    DrawPackageSelector(content_size.x);

    const auto project_name = launcher::Trim(project_name_);
    const auto derived_project_path = launcher::BuildDerivedProjectPath(parent_folder_, project_name);
    ImGui::Spacing();
    ImGui::TextUnformatted("Project Path");
    ImGui::TextWrapped("%s", derived_project_path.project_file.string().c_str());
    if (!create_error_.empty()) {
      ImGui::TextColored(ColorError(), "%s", create_error_.c_str());
    }
    if (!launch_error_.empty()) {
      ImGui::TextColored(ColorError(), "%s", launch_error_.c_str());
    }

    ImGui::Spacing();
    if (ImGui::Button("Create", ImVec2(120.0f, 32.0f))) {
      CreateProject(project_name, derived_project_path.folder, derived_project_path.project_file,
                    launcher::BuildProjectLaunchMetadata(project_name, selected_startup_runtime_packages_));
    }
  }

  void DrawPackageStatusList(const std::vector<std::string>& package_names) const {
    for (const auto& package_name : package_names) {
      const bool available = launcher::IsPackageAvailable(package_availability_, package_name);
      ImGui::TextColored(available ? ColorSuccess() : ColorWarning(), "%s: %s", package_name.c_str(),
                         available ? "available" : "missing");
    }
  }

  void DrawPackageSelector(const float content_width) {
    ImGui::Spacing();
    ImGui::TextUnformatted("Runtime Packages");
    ImGui::SameLine();
    if (ImGui::SmallButton("Refresh")) {
      RefreshPackageAvailability();
    }
    ImGui::TextColored(ColorTextMuted(), "%zu manifest%s", available_packages_.size(),
                       available_packages_.size() == 1 ? "" : "s");
    ImGui::Spacing();

    if (selected_startup_runtime_packages_.empty()) {
      ImGui::TextColored(ColorTextMuted(), "No startup runtime packages selected.");
    } else {
      DrawPackageStatusList(selected_startup_runtime_packages_);
    }
    ImGui::Separator();

    if (available_packages_.empty()) {
      ImGui::TextUnformatted("No package manifests found.");
      return;
    }
    const float package_height = std::clamp(ImGui::GetContentRegionAvail().y * 0.45f, 180.0f, 360.0f);
    if (ImGui::BeginChild("PackageSelectionRows", ImVec2(content_width, package_height), false)) {
      for (const auto& package : available_packages_) {
        DrawPackageSelectionRow(package, content_width - 18.0f);
      }
    }
    ImGui::EndChild();
  }

  void DrawPackageSelectionRow(const AvailablePackageInfo& package, const float width) {
    bool selected = std::find(selected_startup_runtime_packages_.begin(), selected_startup_runtime_packages_.end(),
                              package.name) != selected_startup_runtime_packages_.end();
    ImGui::PushID(package.name.c_str());
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ColorPanelAlt());
    ImGui::BeginChild("PackageSelectionRow", ImVec2(width, 76.0f), true);
    ImGui::BeginDisabled(!package.library_exists);
    if (ImGui::Checkbox(package.name.c_str(), &selected)) {
      SetPackageSelected(package.name, selected);
      create_error_.clear();
    }
    ImGui::EndDisabled();
    ImGui::TextColored(package.library_exists ? ColorTextMuted() : ColorWarning(), "Version: %s",
                       package.version.empty() ? "unknown" : package.version.c_str());
    if (!package.dependencies.empty()) {
      ImGui::TextColored(ColorTextMuted(), "Dependencies: %s", launcher::JoinPackages(package.dependencies).c_str());
    }
    if (!package.library_exists) {
      ImGui::TextColored(ColorWarning(), "Library missing");
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopID();
  }

  void SetPackageSelected(const std::string& package_name, const bool selected) {
    const auto search =
        std::find(selected_startup_runtime_packages_.begin(), selected_startup_runtime_packages_.end(), package_name);
    if (selected && search == selected_startup_runtime_packages_.end()) {
      selected_startup_runtime_packages_.emplace_back(package_name);
    } else if (!selected && search != selected_startup_runtime_packages_.end()) {
      selected_startup_runtime_packages_.erase(search);
    }
  }

  void RefreshPackageAvailability() {
    PackageManager::ScanAvailablePackages();
    available_packages_ = PackageManager::GetAvailablePackages();
    package_availability_ = launcher::BuildPackageAvailability(available_packages_);
    selected_startup_runtime_packages_.erase(
        std::remove_if(selected_startup_runtime_packages_.begin(), selected_startup_runtime_packages_.end(),
                       [this](const std::string& package_name) {
                         return !launcher::IsPackageAvailable(package_availability_, package_name);
                       }),
        selected_startup_runtime_packages_.end());
  }

  void CreateProject(const std::string& project_name, const std::filesystem::path& project_folder,
                     const std::filesystem::path& project_path, const ProjectLaunchMetadata& metadata) {
    create_error_.clear();
    launch_error_.clear();
    create_error_ = launcher::ValidateCreateProjectRequest(project_name, parent_folder_, project_folder, project_path,
                                                           metadata, package_availability_);
    if (create_error_.empty()) {
      try {
        std::filesystem::create_directories(project_folder);
        ProjectManager::SaveProjectLaunchMetadata(project_path, metadata);
        OpenProject(project_path);
      } catch (const std::exception& error) {
        create_error_ = error.what();
      }
    }
  }

  void OpenProject(const std::filesystem::path& path) {
    const auto project_path = launcher::NormalizeProjectPath(path);
    launch_error_.clear();
    if (project_path.extension() != ".eveproj" || std::filesystem::is_directory(project_path)) {
      launch_error_ = "Select a valid .eveproj file.";
      return;
    }

    std::string error;
    if (!LaunchEditorProcess(project_path, error)) {
      launch_error_ = error;
      return;
    }

    AddRecentProject(project_path);
    ApplicationContext::Get().End();
  }

  void LoadRecentProjects() {
    const auto settings_path = LauncherSettingsPath();
    bool pruned = false;
    recent_project_paths_ = launcher::LoadRecentProjects(settings_path, pruned);
    if (pruned) {
      SaveRecentProjects();
    }
  }

  void RefreshRecentProjects() {
    LoadRecentProjects();
    AppendRecentProjectCountTestLog();
    launch_error_.clear();
  }

  void SaveRecentProjects() const {
    try {
      launcher::SaveRecentProjects(LauncherSettingsPath(), recent_project_paths_);
    } catch (const std::exception&) {
    }
  }

  void AppendPackageAvailabilityTestLog() const {
    AppendTestLog("package-count:" + std::to_string(available_packages_.size()));
    for (const auto& package : available_packages_) {
      AppendTestLog("package:" + package.name + ":" + (package.library_exists ? "available" : "missing"));
    }
  }

  void AppendRecentProjectCountTestLog() const {
    AppendTestLog("recent-count:" + std::to_string(recent_project_paths_.size()));
  }

  void AddRecentProject(const std::filesystem::path& path) {
    launcher::AddRecentProject(recent_project_paths_, path);
    SaveRecentProjects();
    AppendRecentProjectCountTestLog();
  }

  void RemoveRecentProject(const size_t index) {
    if (index >= recent_project_paths_.size()) {
      return;
    }
    recent_project_paths_.erase(recent_project_paths_.begin() + static_cast<std::ptrdiff_t>(index));
    SaveRecentProjects();
    AppendRecentProjectCountTestLog();
  }

  void RevealProject(const std::filesystem::path& path) {
    launch_error_.clear();
    std::string error;
    if (!RevealProjectInExplorer(path, error)) {
      launch_error_ = error;
    }
  }
};
}  // namespace

int main() {
  Application application;
  bool initialized = false;
  try {
    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
    ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
    ApplicationContext::Get().PushLayer<LauncherLayer>("Launcher Layer");

    ApplicationInitializationSettings application_info{};
    application_info.application_name = "EvoEngine Launcher";
    application_info.allow_empty_project = true;
    application_info.use_custom_title_bar = true;
    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    ApplicationContext::Get().Start();
    ApplicationContext::Get().Run();
    ApplicationContext::Get().Terminate();
    return 0;
  } catch (const std::exception& error) {
    EVOENGINE_ERROR(error.what())
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    return 1;
  }
}
