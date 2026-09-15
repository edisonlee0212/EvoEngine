#include "Application.hpp"
#include "EditorLayer.hpp"
#include "PathUtils.hpp"
#include "ProjectManager.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;
namespace {
enum class ProjectEditorStateApplyMode { Full, LayoutOnly, SceneStateOnly };
void ApplyProjectEditorState(const std::filesystem::path& path,
                             const ProjectEditorStateApplyMode mode = ProjectEditorStateApplyMode::Full) {
  const auto application = ApplicationContext::TryGet();
  if (!application) {
    return;
  }
  const auto editor_layer = application->GetLayer<EditorLayer>();
  if (!editor_layer) {
    return;
  }
  const auto apply = [&](const YAML::Node& editor_state) {
    switch (mode) {
      case ProjectEditorStateApplyMode::LayoutOnly:
        editor_layer->DeserializeLayout(editor_state);
        return;
      case ProjectEditorStateApplyMode::SceneStateOnly:
        editor_layer->DeserializeSceneState(editor_state);
        return;
      case ProjectEditorStateApplyMode::Full:
        editor_layer->Deserialize(editor_state);
        return;
    }
  };
  if (path.empty() || !std::filesystem::exists(path) || std::filesystem::is_directory(path)) {
    apply(YAML::Node());
    return;
  }
  try {
    const auto in = YAML::LoadFile(path.string());
    if (const auto editor_state = in["EditorLayer"]) {
      apply(editor_state);
    } else {
      apply(YAML::Node());
    }
  } catch (const std::exception& error) {
    EVOENGINE_ERROR("Failed to read project editor state: " + std::string(error.what()))
    apply(YAML::Node());
  }
}

bool ProjectStateHasUsableEditorDockLayout(const YAML::Node& in) {
  const auto editor_state = in["EditorLayer"];
  if (!editor_state || !editor_state.IsMap()) {
    return false;
  }
  try {
    const auto imgui_ini = editor_state["ImGuiIni"];
    if (!imgui_ini || !imgui_ini.IsScalar()) {
      return false;
    }
    return EditorLayer::HasUsableImGuiDockLayout(imgui_ini.as<std::string>());
  } catch (const std::exception&) {
    return false;
  }
}

void RequestDefaultEditorLayoutIfProjectHasNoSavedLayout(const YAML::Node& project_state) {
  if (ProjectStateHasUsableEditorDockLayout(project_state)) {
    return;
  }
  const auto application = ApplicationContext::TryGet();
  if (!application) {
    return;
  }
  const auto editor_layer = application->GetLayer<EditorLayer>();
  if (editor_layer) {
    editor_layer->RequestDefaultEditorLayout();
  }
}

std::filesystem::path CurrentExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return path_utils::CurrentExecutablePath("EvoEngineEditor.exe");
#else
  return path_utils::CurrentExecutablePath("EvoEngineEditor");
#endif
}

std::filesystem::path LauncherExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher.exe";
#else
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher";
#endif
}

bool LaunchLauncherProcess(std::string& error) {
  const auto launcher_path = LauncherExecutablePath();
  if (!std::filesystem::exists(launcher_path)) {
    error = "Could not find EvoEngineLauncher next to the editor executable.";
    return false;
  }

#ifdef EVOENGINE_WINDOWS
  std::wstring command_line = L"\"" + launcher_path.wstring() + L"\"";
  STARTUPINFOW startup_info{};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info{};
  const auto working_directory = launcher_path.parent_path().wstring();
  if (!CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, FALSE, 0, nullptr, working_directory.c_str(),
                      &startup_info, &process_info)) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  CloseHandle(process_info.hProcess);
  CloseHandle(process_info.hThread);
  return true;
#else
  const auto command = "\"" + launcher_path.string() + "\" &";
  if (std::system(command.c_str()) != 0) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  return true;
#endif
}
}  // namespace
void EditorLayer::DrawProjectMenuItems() {
  if (ImGui::MenuItem("Save Project")) {
    ProjectManager::SaveProject();
  }
  if (ImGui::MenuItem("Close Project")) {
    close_project_error_.clear();
    ProjectManager::SaveProject();
    std::string error;
    if (LaunchLauncherProcess(error)) {
      ApplicationContext::Get().End();
    } else {
      close_project_error_ = error;
    }
  }
  if (!close_project_error_.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", close_project_error_.c_str());
  }
}

void EditorLayer::RegisterProjectCallbacks() {
  ProjectHostCallbacks callbacks;
  callbacks.before_project_change = [this] {
    ClearConsoleMessages();
    ClearAssetInspectors();
    ClearEntitySelectionState();
    scene_loading_popup_visible_ = false;
  };
  callbacks.project_opened = [](const std::filesystem::path& path) {
    ApplyProjectEditorState(path, ProjectEditorStateApplyMode::LayoutOnly);
  };
  callbacks.defer_scene_setup = [this] {
    if (scene_loading_popup_visible_ || !ApplicationContext::Get().GetLayer<WindowLayer>())
      return false;
    scene_loading_popup_visible_ = true;
    return true;
  };
  callbacks.scene_metadata_loaded = RequestDefaultEditorLayoutIfProjectHasNoSavedLayout;
  callbacks.scene_attached = [](const std::filesystem::path& path) {
    ApplyProjectEditorState(path, ProjectEditorStateApplyMode::SceneStateOnly);
  };
  callbacks.save_extensions = [this](YAML::Node& extensions) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    Serialize(out);
    out << YAML::EndMap;
    extensions["EditorLayer"] = YAML::Load(out.c_str());
  };
  ProjectManager::SetHostCallbacks(std::move(callbacks));
}
