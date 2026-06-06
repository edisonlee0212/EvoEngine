#include "Application.hpp"
#include "ILayer.hpp"
#include "ImGuiLayer.hpp"
#include "LauncherUtils.hpp"
#include "PackageManager.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>

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

void AppendTestLog(const std::string& line) {
  const char* log_path = std::getenv("EVOENGINE_LAUNCHER_TEST_LOG");
  if (!log_path || std::string(log_path).empty()) {
    return;
  }
  std::ofstream log_file(log_path, std::ios::app);
  log_file << line << "\n";
}

ImVec4 ColorTextMuted() {
  return {0.62f, 0.66f, 0.72f, 1.0f};
}

ImVec4 ColorSuccess() {
  return {0.35f, 0.78f, 0.48f, 1.0f};
}

ImVec4 ColorWarning() {
  return {0.95f, 0.67f, 0.24f, 1.0f};
}

ImVec4 ColorError() {
  return {1.0f, 0.35f, 0.35f, 1.0f};
}

ImVec4 ColorPanel() {
  return {0.105f, 0.115f, 0.135f, 1.0f};
}

ImVec4 ColorPanelAlt() {
  return {0.13f, 0.145f, 0.17f, 1.0f};
}

ImVec4 ColorBorder() {
  return {0.25f, 0.28f, 0.33f, 1.0f};
}

constexpr const char* kRecentProjectsWindow = "Recent Projects";
constexpr const char* kTemplateWindow = "Choose Template";
constexpr const char* kAvailablePackagesWindow = "Available Packages";
constexpr const char* kProjectDetailsWindow = "Project Details";

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
    if (const char* parent_folder = std::getenv("EVOENGINE_LAUNCHER_TEST_PARENT_FOLDER")) {
      parent_folder_ = parent_folder;
    }
    RefreshPackageAvailability();
    LoadRecentProjects();
    AppendRecentProjectCountTestLog();
    AppendTemplateAvailabilityTestLog();
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
    DrawMainMenuBar();
    DrawWorkspace();
  }

 private:
  char project_name_[256] = {};
  int selected_project_template_index_ = 0;
  std::filesystem::path parent_folder_;
  std::string create_error_;
  std::string launch_error_;
  std::vector<std::filesystem::path> recent_project_paths_;
  std::vector<AvailablePackageInfo> available_packages_;
  launcher::PackageAvailability package_availability_;
  bool dock_layout_dirty_ = true;
  ImGuiID dock_space_id_ = 0;
  std::filesystem::path pending_test_open_project_;

  void DrawMainMenuBar() {
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 5));
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Project")) {
        if (ImGui::MenuItem("Exit")) {
          ApplicationContext::Get().End();
        }
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    ImGui::PopStyleVar();
  }

  void DrawWorkspace() {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
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
      const ImVec2 workspace_size = ImGui::GetContentRegionAvail();
      const float padding = 28.0f;
      ImGui::SetCursorPos(ImVec2(padding, 24.0f));
      DrawHeader(workspace_size.x - padding * 2.0f);
      ImGui::SetCursorPos(ImVec2(0.0f, 92.0f));
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

  void DrawHeader(const float width) {
    const float header_top = ImGui::GetCursorPosY();
    ImGui::BeginGroup();
    ImGui::TextUnformatted("EvoEngine Launcher");
    ImGui::TextColored(ColorTextMuted(), "Open an existing project or create a new workspace.");
    ImGui::EndGroup();

    ImGui::SetCursorPosY(header_top);
    ImGui::SetCursorPosX(std::max(width - 136.0f, 0.0f));
    ImGui::PushID("HeaderOpenProject");
    FileUtils::OpenFile(
        "Open Project", "Project", {".eveproj"},
        [this](const std::filesystem::path& path) {
          OpenProject(path);
        },
        false);
    ImGui::PopID();
    ImGui::SameLine();
    if (ImGui::Button("Exit")) {
      ApplicationContext::Get().End();
    }
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
    const ImGuiID recent_node = ImGui::DockBuilderSplitNode(main_node, ImGuiDir_Left, 0.30f, nullptr, &main_node);
    const ImGuiID details_node = ImGui::DockBuilderSplitNode(main_node, ImGuiDir_Right, 0.32f, nullptr, &main_node);
    const ImGuiID packages_node = ImGui::DockBuilderSplitNode(main_node, ImGuiDir_Down, 0.36f, nullptr, &main_node);
    ImGui::DockBuilderDockWindow(kRecentProjectsWindow, recent_node);
    ImGui::DockBuilderDockWindow(kTemplateWindow, main_node);
    ImGui::DockBuilderDockWindow(kAvailablePackagesWindow, packages_node);
    ImGui::DockBuilderDockWindow(kProjectDetailsWindow, details_node);
    ImGui::DockBuilderFinish(dock_space_id_);
    dock_layout_dirty_ = false;
  }

  void DrawProjectHub() {
    DrawDockedPanel(kRecentProjectsWindow, [this](const float width) {
      DrawRecentProjectsPanel(width);
    });
    DrawDockedPanel(kTemplateWindow, [this](const float width) {
      DrawTemplateCards(width);
    });
    DrawDockedPanel(kAvailablePackagesWindow, [this](const float width) {
      DrawAvailablePackagesPanel(width);
    });
    DrawDockedPanel(kProjectDetailsWindow, [this](const float width) {
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
    EnsureSelectedTemplateAvailable();
    ImGui::SetNextItemWidth(content_size.x);
    ImGui::InputText("Project Name", project_name_, sizeof(project_name_));
    DrawTemplateSummary(SelectedProjectTemplate(), content_size.x);
    FileUtils::OpenFolder(
        "Parent Folder",
        [this](const std::filesystem::path& path) {
          parent_folder_ = path;
          create_error_.clear();
        },
        false);

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
                    launcher::BuildProjectLaunchMetadata(project_name, SelectedProjectTemplate()));
    }
  }

  const launcher::ProjectTemplate& SelectedProjectTemplate() const {
    const auto& templates = launcher::ProjectTemplates();
    if (selected_project_template_index_ < 0 ||
        selected_project_template_index_ >= static_cast<int>(templates.size())) {
      return templates.front();
    }
    return templates[static_cast<size_t>(selected_project_template_index_)];
  }

  void DrawTemplateCards(const float content_width) {
    const auto& templates = launcher::ProjectTemplates();
    const bool two_columns = content_width >= 500.0f;
    const float card_width = two_columns ? (content_width - 42.0f) * 0.5f : content_width - 22.0f;
    for (size_t i = 0; i < templates.size(); ++i) {
      if (two_columns && i % 2 == 1) {
        ImGui::SameLine();
      }
      DrawTemplateCard(i, card_width);
    }
  }

  void DrawTemplateCard(const size_t template_index, const float width) {
    const auto& project_template = launcher::ProjectTemplates()[template_index];
    const bool selected = selected_project_template_index_ == static_cast<int>(template_index);
    const bool available = IsTemplateAvailable(project_template);
    ImGui::PushID(static_cast<int>(template_index));
    ImGui::PushStyleColor(ImGuiCol_Border, selected ? ImVec4(0.42f, 0.63f, 0.92f, 1.0f) : ColorBorder());
    ImGui::PushStyleColor(ImGuiCol_ChildBg, selected ? ImVec4(0.15f, 0.18f, 0.23f, 1.0f) : ColorPanelAlt());
    ImGui::BeginChild("TemplateCard", ImVec2(width, 124.0f), true);
    ImGui::TextUnformatted(project_template.name.c_str());
    if (project_template.startup_runtime_packages.empty()) {
      ImGui::TextColored(ColorTextMuted(), "No runtime packages");
    } else {
      ImGui::TextColored(ColorTextMuted(), "Required packages:");
      ImGui::TextWrapped("%s", launcher::JoinPackages(project_template.startup_runtime_packages).c_str());
      if (available) {
        ImGui::TextColored(ColorSuccess(), "Packages available");
      } else {
        ImGui::TextColored(ColorWarning(), "Missing packages: %s",
                           launcher::JoinPackages(launcher::MissingPackages(package_availability_,
                                                                            project_template.startup_runtime_packages))
                               .c_str());
      }
    }
    ImGui::EndChild();
    const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
    if (hovered && available && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      selected_project_template_index_ = static_cast<int>(template_index);
      create_error_.clear();
    }
    if (hovered && !available) {
      ImGui::SetTooltip("Missing packages: %s",
                        launcher::JoinPackages(
                            launcher::MissingPackages(package_availability_, project_template.startup_runtime_packages))
                            .c_str());
    }
    ImGui::PopStyleColor(2);
    ImGui::PopID();
  }

  bool IsTemplateAvailable(const launcher::ProjectTemplate& project_template) const {
    return launcher::IsTemplateAvailable(project_template, package_availability_);
  }

  void EnsureSelectedTemplateAvailable() {
    const int selected_index = launcher::SelectAvailableTemplateIndex(
        launcher::ProjectTemplates(), package_availability_, selected_project_template_index_);
    if (selected_index != selected_project_template_index_) {
      selected_project_template_index_ = selected_index;
      create_error_.clear();
    }
  }

  void DrawPackageStatusList(const std::vector<std::string>& package_names) const {
    for (const auto& package_name : package_names) {
      const bool available = launcher::IsPackageAvailable(package_availability_, package_name);
      ImGui::TextColored(available ? ColorSuccess() : ColorWarning(), "%s: %s", package_name.c_str(),
                         available ? "available" : "missing");
    }
  }

  void DrawTemplateSummary(const launcher::ProjectTemplate& project_template, const float width) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ColorPanelAlt());
    ImGui::BeginChild("TemplateSummary",
                      ImVec2(width, project_template.startup_runtime_packages.empty() ? 72.0f : 112.0f), true);
    ImGui::TextUnformatted(project_template.name.c_str());
    ImGui::TextColored(
        ColorTextMuted(), "%s",
        project_template.startup_runtime_packages.empty() ? "Generic project" : "Package-backed template");
    if (project_template.startup_runtime_packages.empty()) {
      ImGui::TextColored(ColorTextMuted(), "Packages: none");
    } else {
      DrawPackageStatusList(project_template.startup_runtime_packages);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
  }

  void DrawAvailablePackagesPanel(const float content_width) {
    if (ImGui::Button("Refresh Packages", ImVec2(138.0f, 28.0f))) {
      RefreshPackageAvailability();
    }
    ImGui::SameLine();
    ImGui::TextColored(ColorTextMuted(), "%zu manifest%s", available_packages_.size(),
                       available_packages_.size() == 1 ? "" : "s");
    ImGui::Spacing();

    const auto& selected_template = SelectedProjectTemplate();
    if (selected_template.startup_runtime_packages.empty()) {
      ImGui::TextColored(ColorTextMuted(), "Selected template does not require runtime packages.");
    } else {
      ImGui::TextUnformatted("Selected template requires:");
      DrawPackageStatusList(selected_template.startup_runtime_packages);
    }
    ImGui::Separator();

    if (available_packages_.empty()) {
      ImGui::TextUnformatted("No package manifests found.");
      return;
    }
    if (ImGui::BeginChild("AvailablePackageRows", ImVec2(0.0f, 0.0f), false)) {
      for (const auto& package : available_packages_) {
        DrawAvailablePackageRow(package, content_width - 16.0f);
      }
    }
    ImGui::EndChild();
  }

  void DrawAvailablePackageRow(const AvailablePackageInfo& package, const float width) const {
    ImGui::PushID(package.name.c_str());
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ColorPanelAlt());
    ImGui::BeginChild("PackageRow", ImVec2(width, 42.0f), true);
    ImGui::TextUnformatted(package.name.c_str());
    ImGui::SameLine();
    ImGui::TextColored(ColorTextMuted(), "Version: %s", package.version.empty() ? "unknown" : package.version.c_str());
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopID();
  }

  void RefreshPackageAvailability() {
    PackageManager::ScanAvailablePackages();
    available_packages_ = PackageManager::GetAvailablePackages();
    package_availability_ = launcher::BuildPackageAvailability(available_packages_);
    EnsureSelectedTemplateAvailable();
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

  void AppendTemplateAvailabilityTestLog() const {
    for (const auto& project_template : launcher::ProjectTemplates()) {
      AppendTestLog("template:" + project_template.name + ":" +
                    (IsTemplateAvailable(project_template) ? "available" : "missing"));
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
