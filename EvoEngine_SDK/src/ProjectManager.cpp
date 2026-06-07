#include "ProjectManager.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
#include "WindowLayer.hpp"
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  include "shellapi.h"
#endif

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

using namespace evo_engine;

namespace {
constexpr const char* kDefaultEditorName = "EvoEngineEditor";
constexpr const char* kDefaultApplicationName = "EvoEngine Editor";

void AddUnique(std::vector<std::string>& values, const std::string& value) {
  if (!value.empty() && std::find(values.begin(), values.end(), value) == values.end()) {
    values.emplace_back(value);
  }
}

void ReadStringKey(const YAML::Node& in, const char* key, std::string& value) {
  const auto node = in[key];
  if (node && node.IsScalar()) {
    value = node.as<std::string>();
  }
}

void ReadStringSequenceKey(const YAML::Node& in, const char* key, std::vector<std::string>& values) {
  const auto node = in[key];
  if (!node || !node.IsSequence()) {
    return;
  }
  values.clear();
  for (const auto& entry : node) {
    if (entry.IsScalar()) {
      AddUnique(values, entry.as<std::string>());
    }
  }
}

uint64_t ReadStartSceneHandle(const YAML::Node& in) {
  if (const auto start_scene_handle = in["start_scene_handle"]) {
    return start_scene_handle.as<uint64_t>();
  }
  if (const auto start_scene_handle = in["m_startSceneHandle"]) {
    return start_scene_handle.as<uint64_t>();
  }
  return 0;
}

std::optional<uint64_t> ReadExistingStartSceneHandle(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path) || std::filesystem::is_directory(path)) {
    return std::nullopt;
  }
  try {
    const auto scene_handle = ReadStartSceneHandle(YAML::LoadFile(path.string()));
    if (scene_handle != 0) {
      return scene_handle;
    }
  } catch (const std::exception& error) {
    EVOENGINE_ERROR("Failed to read project start scene handle: " + std::string(error.what()))
  }
  return std::nullopt;
}

void WriteProjectFile(const std::filesystem::path& path, const ProjectLaunchMetadata& metadata,
                      const std::optional<uint64_t> start_scene_handle) {
  if (const auto directory = path.parent_path(); !directory.empty() && !std::filesystem::exists(directory)) {
    std::filesystem::create_directories(directory);
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "application_name" << YAML::Value << metadata.application_name;
  out << YAML::Key << "preferred_editor" << YAML::Value << metadata.preferred_editor;
  out << YAML::Key << "startup_runtime_packages" << YAML::Value << YAML::BeginSeq;
  for (const auto& package_name : metadata.startup_runtime_packages) {
    out << package_name;
  }
  out << YAML::EndSeq;
  if (start_scene_handle) {
    out << YAML::Key << "start_scene_handle" << YAML::Value << *start_scene_handle;
  }
  out << YAML::EndMap;

  std::ofstream file_out(path.string());
  file_out << out.c_str();
  file_out.flush();
}

void MergeApplicationLaunchMetadata(ProjectLaunchMetadata& metadata) {
  const auto& application_info = ApplicationContext::Get().GetApplicationInfo();
  if (metadata.application_name == kDefaultApplicationName && !application_info.application_name.empty()) {
    metadata.application_name = application_info.application_name;
  }
  for (const auto& package_name : application_info.startup_runtime_packages) {
    AddUnique(metadata.startup_runtime_packages, package_name);
  }
  if (metadata.preferred_editor.empty()) {
    metadata.preferred_editor = kDefaultEditorName;
  }
}

std::filesystem::path CurrentExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  std::wstring path(MAX_PATH, L'\0');
  const DWORD size = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
  if (size == 0 || size == path.size()) {
    return std::filesystem::absolute("EvoEngineEditor.exe");
  }
  path.resize(size);
  return path;
#else
  return std::filesystem::absolute(kDefaultEditorName);
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

std::weak_ptr<Folder> ProjectManager::GetOrCreateFolder(const std::filesystem::path& assets_relative_path) {
  const auto& project_manager = GetInstance();
  if (!assets_relative_path.is_relative()) {
    EVOENGINE_ERROR("Path not relative!")
    return {};
  }
  auto dir_path = project_manager.assets_folder_->GetAbsolutePath() / assets_relative_path;
  std::shared_ptr<Folder> ret_val = project_manager.assets_folder_;
  for (auto it = assets_relative_path.begin(); it != assets_relative_path.end(); ++it) {
    if (it == assets_relative_path.begin() && it->filename().string() == ".")
      continue;
    ret_val = ret_val->GetOrCreateChild(it->filename().string()).lock();
  }
  return ret_val;
}
std::shared_ptr<IAsset> ProjectManager::GetOrCreateAsset(const std::filesystem::path& assets_relative_path) {
  if (std::filesystem::is_directory(assets_relative_path)) {
    EVOENGINE_ERROR("Path is directory!")
    return {};
  }
  const auto folder = GetOrCreateFolder(assets_relative_path.parent_path()).lock();
  auto stem = assets_relative_path.stem().string();
  const auto file_name = assets_relative_path.filename().string();
  auto extension = assets_relative_path.extension().string();
  if (file_name == stem) {
    stem = "";
    extension = file_name;
  }
  return folder->GetOrCreateAsset(stem, extension);
}

void ProjectManager::SetupDefaultScene() {
  auto& project_manager = GetInstance();
  auto project_absolute_path = std::filesystem::absolute(project_manager.new_project_path_);
  bool found_scene = false;
  std::shared_ptr<Scene> scene;
  if (std::filesystem::exists(project_absolute_path)) {
    std::ifstream stream(project_absolute_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    YAML::Node in = YAML::Load(string_stream.str());
    const uint64_t scene_handle = ReadStartSceneHandle(in);
    if (scene_handle != 0) {
      if (auto temp = AssetManager::GetAssetImpl(scene_handle)) {
        scene = std::dynamic_pointer_cast<Scene>(temp);
        SetStartScene(scene);
        SaveProject();
        ApplicationContext::Get().Attach(scene);
        found_scene = true;
      }
    }
    EVOENGINE_LOG("Found and loaded project")
    if (found_scene && project_manager.scene_post_load_function_.has_value()) {
      project_manager.scene_post_load_function_.value()(scene);
      TransformGraph::CalculateTransformGraphs(scene);
    }
  }
  if (!found_scene) {
    scene = AssetManager::CreateTemporaryAsset<Scene>();
    if (std::filesystem::path new_scene_relative_path = GenerateNewAssetsRelativePath("New Scene", ".evescene");
        scene->SetPathAndSave(new_scene_relative_path)) {
      EVOENGINE_LOG("Created new start scene!")
    }
    SetStartScene(scene);
    SaveProject();
    ApplicationContext::Get().Attach(scene);

    if (project_manager.new_scene_customizer_.has_value()) {
      project_manager.new_scene_customizer_.value()(scene);
      TransformGraph::CalculateTransformGraphs(scene);
    }
  }

  project_manager.new_project_path_ = "";
}

void ProjectManager::PreUpdate() {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  if (window_layer && Platform::GetFrameCount() < 4)
    return;
  auto& project_manager = GetInstance();

  if (project_manager.scan_assets_pending) {
    ScanAssets();
    return;
  }

  if (!project_manager.pending_assets.empty()) {
    LoadAllPendingAssets();
  }

  if (project_manager.project_asset_load_dispatched) {
    const auto snapshot = AssetManager::GetAssetLoadSnapshot();
    if (snapshot.Active()) {
      return;
    }
    project_manager.project_asset_load_dispatched = false;
    project_manager.pending_asset_size = 0;
  }

  if (!project_manager.new_project_path_.empty()) {
    if (ApplicationContext::Get().GetApplicationInfo().load_project_start_scene) {
      SetupDefaultScene();
    } else {
      project_manager.new_project_path_ = "";
    }
  }
}

bool ProjectManager::IsProjectIdle() {
  return IsProjectLoaded();
}

ProjectState ProjectManager::GetProjectState() {
  const auto& project_manager = GetInstance();
  if (!HasProject()) {
    return ProjectState::NoProject;
  }

  const auto asset_load_snapshot = AssetManager::GetAssetLoadSnapshot();
  if (project_manager.new_project_path_.empty() && !project_manager.scan_assets_pending &&
      project_manager.pending_assets.empty() && !project_manager.project_asset_load_dispatched &&
      project_manager.pending_asset_size == 0 && !asset_load_snapshot.Active() && project_manager.start_scene_) {
    return ProjectState::Loaded;
  }
  return ProjectState::Loading;
}

bool ProjectManager::HasProject() {
  const auto& project_manager = GetInstance();
  return !project_manager.project_path_.empty() || !project_manager.new_project_path_.empty();
}

bool ProjectManager::IsProjectLoaded() {
  return GetProjectState() == ProjectState::Loaded;
}

ProjectLaunchMetadata ProjectManager::LoadProjectLaunchMetadata(const std::filesystem::path& path) {
  ProjectLaunchMetadata metadata;
  if (path.empty() || !std::filesystem::exists(path) || std::filesystem::is_directory(path)) {
    return metadata;
  }

  try {
    const auto in = YAML::LoadFile(path.string());
    ReadStringKey(in, "application_name", metadata.application_name);
    ReadStringKey(in, "preferred_editor", metadata.preferred_editor);
    ReadStringSequenceKey(in, "startup_runtime_packages", metadata.startup_runtime_packages);
  } catch (const std::exception& error) {
    EVOENGINE_ERROR("Failed to read project launch metadata: " + std::string(error.what()))
  }
  if (metadata.application_name.empty()) {
    metadata.application_name = kDefaultApplicationName;
  }
  if (metadata.preferred_editor.empty()) {
    metadata.preferred_editor = kDefaultEditorName;
  }
  return metadata;
}

void ProjectManager::SaveProjectLaunchMetadata(const std::filesystem::path& path,
                                               const ProjectLaunchMetadata& metadata) {
  WriteProjectFile(path, metadata, ReadExistingStartSceneHandle(path));
}

ProjectLaunchMetadata ProjectManager::GetProjectLaunchMetadata() {
  const auto& project_manager = GetInstance();
  return project_manager.project_launch_metadata_;
}

void ProjectManager::LoadAllPendingAssets() {
  auto& project_manager = GetInstance();
  if (project_manager.pending_assets.empty()) {
    return;
  }

  std::vector<Handle> handles;
  handles.reserve(project_manager.pending_assets.size());
  for (const auto& handle : project_manager.pending_assets) {
    handles.emplace_back(handle);
  }

  project_manager.pending_asset_size = handles.size();
  [[maybe_unused]] const auto load_futures = AssetManager::RequestAssetLoads(project_manager.pending_assets);
  project_manager.project_asset_load_dispatched = true;
  project_manager.pending_assets.clear();

  if (ApplicationContext::Get().GetLayer<WindowLayer>()) {
    return;
  }

  for (const auto& handle : handles) {
    AssetManager::GetAssetImpl(handle);
  }
  project_manager.project_asset_load_dispatched = false;
  project_manager.pending_asset_size = 0;
}

void ProjectManager::SaveProject() {
  const auto& project_manager = GetInstance();
  WriteProjectFile(project_manager.project_path_, project_manager.project_launch_metadata_,
                   static_cast<uint64_t>(project_manager.start_scene_->GetHandle()));
}

std::filesystem::path ProjectManager::GetProjectPath() {
  auto& project_manager = GetInstance();
  return project_manager.project_path_;
}

std::filesystem::path ProjectManager::GetAssetsFolderPath() {
  auto& project_manager = GetInstance();
  return project_manager.assets_folder_path;
}

std::string ProjectManager::GetProjectName() {
  const auto& project_manager = GetInstance();
  return project_manager.project_path_.stem().string();
}
std::weak_ptr<Folder> ProjectManager::GetCurrentFocusedFolder() {
  auto& project_manager = GetInstance();
  return project_manager.current_focused_folder_;
}

std::shared_ptr<Folder> ProjectManager::GetAssetsFolder() {
  auto& project_manager = GetInstance();
  return project_manager.assets_folder_;
}

bool ProjectManager::IsInAssetsFolder(const std::filesystem::path& absolute_path) {
  if (!absolute_path.is_absolute()) {
    EVOENGINE_ERROR("Not absolute path!")
    return false;
  }
  const auto& project_manager = GetInstance();
  const auto project_folder_path = project_manager.assets_folder_path;
  const auto absolute_path_string = absolute_path.string();
  const auto project_folder_path_string = project_folder_path.string();
  return std::search(absolute_path_string.begin(), absolute_path_string.end(), project_folder_path_string.begin(),
                     project_folder_path_string.end()) != absolute_path_string.end();
}
bool ProjectManager::IsValidAssetFileName(const std::filesystem::path& path) {
  auto stem = path.stem().string();
  const auto file_name = path.filename().string();
  auto extension = path.extension().string();
  if (file_name == stem) {
    stem = "";
    extension = file_name;
  }
  return Serialization::GetAssetTypeName(extension) == "Binary";
}

void ProjectManager::GetOrCreateProject(const std::filesystem::path& path) {
  auto& project_manager = GetInstance();
  auto project_absolute_path = std::filesystem::absolute(path);
  if (std::filesystem::is_directory(project_absolute_path)) {
    EVOENGINE_ERROR("Path is directory!")
    return;
  }
  if (!project_absolute_path.is_absolute()) {
    EVOENGINE_ERROR("Path not absolute!")
    return;
  }
  if (project_absolute_path.extension() != ".eveproj") {
    EVOENGINE_ERROR("Wrong extension!")
    return;
  }
  project_manager.new_project_path_ = project_absolute_path;
  project_manager.project_path_ = project_absolute_path;
  project_manager.assets_folder_path = project_absolute_path.parent_path() / "Assets";
  project_manager.project_launch_metadata_ = LoadProjectLaunchMetadata(project_absolute_path);
  MergeApplicationLaunchMetadata(project_manager.project_launch_metadata_);
  AssetManager::Clear();
  FileManager::Clear();
  ApplicationContext::Get().Reset();

  auto& file_manager = FileManager::GetInstance();
  project_manager.current_focused_folder_ = project_manager.assets_folder_ = std::make_shared<Folder>();
  project_manager.assets_folder_->name_ = "Assets";
  file_manager.file_registry_mutex.lock();
  file_manager.folder_registry_[0] = project_manager.assets_folder_;
  file_manager.file_registry_mutex.unlock();
  project_manager.assets_folder_->self_ = project_manager.assets_folder_;

  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    DispatchScanAssetsTask();
  } else {
    ScanAssets();
    if (ApplicationContext::Get().GetApplicationInfo().load_project_assets) {
      LoadAllPendingAssets();
    }
    if (ApplicationContext::Get().GetApplicationInfo().load_project_start_scene) {
      SetupDefaultScene();
    } else {
      project_manager.new_project_path_ = "";
    }
  }
}

void ProjectManager::DispatchScanAssetsTask() {
  auto& project_manager = GetInstance();
  project_manager.scan_assets_pending = true;
}

std::filesystem::path ProjectManager::GenerateNewAssetsRelativePath(const std::string& relative_stem,
                                                                    const std::string& postfix) {
  assert(std::filesystem::path(relative_stem + postfix).is_relative());
  const auto& project_manager = GetInstance();
  const auto assets_path = project_manager.assets_folder_path;
  std::filesystem::path test_path = assets_path / (relative_stem + postfix);
  int i = 0;
  while (std::filesystem::exists(test_path)) {
    i++;
    test_path = assets_path / (relative_stem + " (" + std::to_string(i) + ")" + postfix);
  }
  if (i == 0)
    return relative_stem + postfix;
  return relative_stem + " (" + std::to_string(i) + ")" + postfix;
}

std::filesystem::path ProjectManager::GenerateNewAbsolutePath(const std::string& absolute_stem,
                                                              const std::string& postfix) {
  std::filesystem::path test_path = absolute_stem + postfix;
  int i = 0;
  while (std::filesystem::exists(test_path)) {
    i++;
    test_path = absolute_stem + " (" + std::to_string(i) + ")" + postfix;
  }
  if (i == 0)
    return absolute_stem + postfix;
  return absolute_stem + " (" + std::to_string(i) + ")" + postfix;
}

void ProjectManager::SetActionAfterSceneLoad(const std::function<void(const std::shared_ptr<Scene>&)>& actions) {
  auto& project_manager = GetInstance();
  project_manager.scene_post_load_function_ = actions;
}

void ProjectManager::SetActionAfterNewScene(const std::function<void(const std::shared_ptr<Scene>&)>& actions) {
  auto& project_manager = GetInstance();
  project_manager.new_scene_customizer_ = actions;
}

void ProjectManager::ScanAssets() {
  auto& project_manager = GetInstance();
  project_manager.scan_assets_pending = false;
  if (!project_manager.assets_folder_)
    return;
  if (!std::filesystem::exists(project_manager.assets_folder_->GetAbsolutePath())) {
    std::filesystem::create_directories(project_manager.assets_folder_->GetAbsolutePath());
  }
  auto& file_manager = FileManager::GetInstance();
  file_manager.file_registry_mutex.lock();
  project_manager.assets_folder_->handle_ = 0;
  std::vector<Handle> missing_asset_handles;
  project_manager.assets_folder_->Refresh(missing_asset_handles);
  file_manager.file_registry_mutex.unlock();
  project_manager.pending_assets.clear();
  project_manager.pending_asset_size = missing_asset_handles.size();
  for (const auto& i : missing_asset_handles) {
    project_manager.pending_assets.emplace(i);
  }
}

void ProjectManager::Initialize() {
  auto& project_manager = GetInstance();
  project_manager.initialized = true;
}

void ProjectManager::OnDestroy() {
  auto& project_manager = GetInstance();

  project_manager.scan_assets_pending = false;
  project_manager.project_asset_load_dispatched = false;
  project_manager.pending_asset_size = 0;
  project_manager.pending_assets.clear();
  project_manager.assets_folder_.reset();
  project_manager.new_scene_customizer_.reset();
  project_manager.current_focused_folder_.reset();
  project_manager.start_scene_.reset();
  project_manager.project_launch_metadata_ = {};
  project_manager.new_project_path_ = "";
  project_manager.project_path_ = "";
  project_manager.assets_folder_path = "";

  project_manager.initialized = false;
}

void ProjectManager::DrawViewMenuItems() {
  ImGui::Checkbox("Project", &GetInstance().show_project_window);
}

void ProjectManager::DrawProjectMenu() {
  auto& project_manager = GetInstance();
  static std::string close_project_error;
  if (ImGui::BeginMenu("Project")) {
    ImGui::Text(("Current Project path: " + project_manager.project_path_.string()).c_str());

    if (ImGui::Button("Save")) {
      SaveProject();
    }
    if (ImGui::Button("Close Project")) {
      close_project_error.clear();
      SaveProject();
      std::string error;
      if (LaunchLauncherProcess(error)) {
        ApplicationContext::Get().End();
      } else {
        close_project_error = error;
      }
    }
    if (!close_project_error.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", close_project_error.c_str());
    }
    ImGui::EndMenu();
  }
}

void ProjectManager::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& project_manager = GetInstance();
  if (project_manager.show_project_window) {
    if (ImGui::Begin("Project")) {
      if (project_manager.assets_folder_) {
        auto current_focused_folder = project_manager.current_focused_folder_.lock();
        auto current_folder_path = current_focused_folder->GetAssetsRelativePath();
        if (ImGui::BeginDragDropTarget()) {
          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            Handle handle = *static_cast<Handle*>(payload->Data);
            if (const auto asset = AssetManager::GetAssetImpl(handle)) {
              if (asset->IsTemporary()) {
                auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
                auto file_name = "New " + asset->GetTypeName();
                auto file_path = GenerateNewAssetsRelativePath(
                    (current_focused_folder->GetAssetsRelativePath() / file_name).string(), file_extension);
                asset->SetPathAndSave(file_path);
              } else {
                if (auto file = asset->file_record_.lock();
                    file->GetFolder().lock().get() != current_focused_folder.get()) {
                  auto file_extension = file->GetAssetExtension();
                  auto file_name = file->GetAssetFileName();
                  auto file_path = GenerateNewAssetsRelativePath(
                      (current_focused_folder->GetAssetsRelativePath() / file_name).string(), file_extension);
                  asset->SetPathAndSave(file_path);
                }
              }
            } else {
              if (const auto file = FileManager::GetFile(handle)) {
                auto folder = file->GetFolder().lock();
                if (folder.get() != current_focused_folder.get()) {
                  folder->MoveAsset(file->GetAssetHandle(), current_focused_folder);
                }
              }
            }
          }

          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
            auto entity_handle = *static_cast<Handle*>(payload->Data);
            auto scene = ApplicationContext::Get().GetActiveScene();
            if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
              prefab->FromEntity(entity);
              // If current folder doesn't contain file with same name
              auto file_name = scene->GetEntityName(entity);
              auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
              auto file_path =
                  GenerateNewAssetsRelativePath((current_folder_path / file_name).string(), file_extension);
              prefab->SetPathAndSave(file_path);
            }
          }

          ImGui::EndDragDropTarget();
        }
        static glm::vec2 thumbnail_size_padding = {75.0f, 8.0f};
        float cell_size = thumbnail_size_padding.x + thumbnail_size_padding.y;
        static float size1 = 200;
        static float size2 = 200;
        static float h = 100;
        auto avail = ImGui::GetContentRegionAvail();
        size2 = glm::max(avail.x - size1, cell_size + 8.0f);
        size1 = glm::max(avail.x - size2, 32.0f);
        h = avail.y;
        ImGui::Splitter(true, 8.0, size1, size2, 32.0f, cell_size + 8.0f, h);
        ImGui::BeginChild("1", ImVec2(size1, h), true);
        FolderHierarchyHelper(editor_layer, project_manager.assets_folder_);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("2", ImVec2(size2 - 5.0f, h), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
        ImGui::PushID(static_cast<ImTextureID>(
            static_cast<intptr_t>(editor_layer->editor_icons_["RefreshButton"]->GetImTextureId())));
        ImGui::PushID(static_cast<ImTextureID>(
            static_cast<intptr_t>(editor_layer->editor_icons_["BackButton"]->GetImTextureId())));

        if (ImGui::ImageButton("RefreshButton", editor_layer->editor_icons_["RefreshButton"]->GetImTextureId(),
                               {16, 16}, {0, 1}, {1, 0})) {
          DispatchScanAssetsTask();
        }
        ImGui::SameLine();
        if (current_focused_folder != project_manager.assets_folder_) {
          if (ImGui::ImageButton("BackButton", editor_layer->editor_icons_["BackButton"]->GetImTextureId(), {16, 16},
                                 {0, 1}, {1, 0})) {
            project_manager.current_focused_folder_ = current_focused_folder->parent_;
          }
        } else {
          ImGui::BeginDisabled();
          if (ImGui::ImageButton("BackButton", editor_layer->editor_icons_["BackButton"]->GetImTextureId(), {16, 16},
                                 {0, 1}, {1, 0})) {
            project_manager.current_focused_folder_ = current_focused_folder->parent_;
          }
          ImGui::EndDisabled();
        }
        ImGui::PopID();
        ImGui::PopID();
        static bool show_extension = false;
        ImGui::SameLine();
        ImGui::Checkbox("Ext", &show_extension);
        ImGui::SameLine();
        ImGui::Text(current_focused_folder->GetAssetsRelativePath().string().c_str());

        ImGui::SameLine();

        ImGui::PushItemWidth(60);
        ImGui::SliderFloat("##Thumbnail size", &thumbnail_size_padding.x, 10, 150, "%.0f", ImGuiSliderFlags_None);
        ImGui::PopItemWidth();

        ImGui::Separator();
        bool updated = false;
        if (ImGui::BeginPopupContextWindow("NewAssetPopup")) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
          if (ImGui::Button("Show in Explorer...")) {
            const auto folder_path = current_focused_folder->GetAbsolutePath().string();
            ShellExecuteA(nullptr, "open", folder_path.c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
          }
#else
#endif

          FileUtils::OpenFile(
              "Import model...", "Model",
              {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d"},
              [&](const std::filesystem::path& path) {
                const auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
                if (prefab->Import(path)) {
                  prefab->SetPathAndSave(current_focused_folder->GetAssetsRelativePath() /
                                         path.filename().replace_extension(".eveprefab"));
                }
              },
              false);

          if (ImGui::Button("New folder...")) {
            auto new_path = GenerateNewAssetsRelativePath(
                (current_focused_folder->GetAssetsRelativePath() / "New Folder").string(), "");
            GetOrCreateFolder(new_path);
          }
          if (ImGui::BeginMenu("New asset...")) {
            for (auto& i : Serialization::GetInstance().asset_extensions_) {
              if (i.first == "IAsset")
                continue;
              if (ImGui::Button(i.first.c_str())) {
                std::string new_file_name = "New " + i.first;
                std::filesystem::path new_path = GenerateNewAssetsRelativePath(
                    (current_focused_folder->GetAssetsRelativePath() / new_file_name).string(), i.second.front());
                current_focused_folder->GetOrCreateAsset(new_path.stem().string(), new_path.extension().string());
              }
            }
            ImGui::EndMenu();
          }
          ImGui::EndPopup();
        }

        float panel_width = ImGui::GetContentRegionAvail().x;
        int column_count = glm::max(1, static_cast<int>(panel_width / (cell_size + thumbnail_size_padding.y)));
        ImGui::Columns(column_count, nullptr, false);
        if (!updated) {
          for (auto& i : current_focused_folder->children_) {
            const std::string icon_tag = "##Icon" + std::to_string(i.second->handle_);
            const auto& thumbnail_tex = editor_layer->editor_icons_["Folder"];
            glm::vec2 resolution = thumbnail_tex->GetResolution();
            resolution *= thumbnail_size_padding.x / glm::max(resolution.x, resolution.y);
            ImGui::ImageButton(icon_tag.c_str(), thumbnail_tex->GetImTextureId(), {resolution.x, resolution.y}, {0, 1},
                               {1, 0});

            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
              ImGui::SetDragDropPayload("Folder", &i.second->handle_, sizeof(Handle));
              ImGui::TextColored(ImVec4(0, 0, 1, 1), ("Folder" + icon_tag).c_str());
              ImGui::EndDragDropSource();
            }
            if (i.second->GetHandle() != 0) {
              if (ImGui::BeginPopupContextItem(icon_tag.c_str())) {
                if (ImGui::BeginMenu(("Rename" + icon_tag).c_str())) {
                  static char new_name[256] = {0};
                  ImGui::InputText(("New name" + icon_tag).c_str(), new_name, 256);
                  if (ImGui::Button(("Confirm" + icon_tag).c_str())) {
                    i.second->Rename(std::string(new_name));
                    memset(new_name, 0, 256);
                    ImGui::CloseCurrentPopup();
                  }
                  ImGui::EndMenu();
                }
                if (ImGui::Button(("Remove" + icon_tag).c_str())) {
                  i.second->parent_.lock()->DeleteChild(i.second->handle_);
                  updated = true;
                  ImGui::CloseCurrentPopup();
                  ImGui::EndPopup();
                  break;
                }
                ImGui::EndPopup();
              }
            }
            if (ImGui::BeginDragDropTarget()) {
              if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
                IM_ASSERT(payload->DataSize == sizeof(Handle));
                if (Handle payload_n = *static_cast<Handle*>(payload->Data); payload_n.GetValue() != 0) {
                  if (auto received_folder = FileManager::GetFolder(payload_n)) {
                    if (!i.second->IsSelfOrAncestor(received_folder->handle_) &&
                        received_folder->parent_.lock().get() != i.second.get()) {
                      received_folder->parent_.lock()->MoveChild(received_folder->GetHandle(), i.second);
                    }
                  }
                }
              }
              if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
                IM_ASSERT(payload->DataSize == sizeof(Handle));
                Handle payload_n = *static_cast<Handle*>(payload->Data);
                if (const auto asset = AssetManager::GetAssetImpl(payload_n)) {
                  if (asset->IsTemporary()) {
                    auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
                    auto file_name = "New " + asset->GetTypeName();
                    auto file_path = GenerateNewAssetsRelativePath(
                        (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                    asset->SetPathAndSave(file_path);
                  } else {
                    if (auto asset_record = asset->file_record_.lock();
                        asset_record->GetFolder().lock().get() != i.second.get()) {
                      auto file_extension = asset_record->GetAssetExtension();
                      auto file_name = asset_record->GetAssetFileName();
                      auto file_path = GenerateNewAssetsRelativePath(
                          (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                      asset->SetPathAndSave(file_path);
                    }
                  }
                } else {
                  if (const auto file = FileManager::GetFile(payload_n)) {
                    auto folder = file->GetFolder().lock();
                    if (folder.get() != i.second.get()) {
                      folder->MoveAsset(file->GetAssetHandle(), i.second);
                    }
                  }
                }
              }

              if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Binary")) {
                IM_ASSERT(payload->DataSize == sizeof(Handle));
                Handle payload_n = *static_cast<Handle*>(payload->Data);
                if (const auto file = FileManager::GetFile(payload_n))
                  file->GetFolder().lock()->MoveAsset(payload_n, i.second);
              }

              if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
                IM_ASSERT(payload->DataSize == sizeof(Handle));
                auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
                auto entity_handle = *static_cast<Handle*>(payload->Data);
                auto scene = ApplicationContext::Get().GetActiveScene();
                if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
                  // If current folder doesn't contain file with same name
                  auto file_name = scene->GetEntityName(entity);
                  auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
                  auto file_path = GenerateNewAssetsRelativePath(
                      (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                  prefab->SetPathAndSave(file_path);
                }
              }

              ImGui::EndDragDropTarget();
            }
            bool item_hovered = false;
            if (ImGui::IsItemHovered()) {
              item_hovered = true;
              if (ImGui::IsMouseDoubleClicked(0)) {
                project_manager.current_focused_folder_ = i.second;
                updated = true;
                break;
              }
            }
            if (item_hovered)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 1, 0, 1});
            ImGui::BeginDisabled();
            const std::string button_tag = "Folder##Button" + std::to_string(i.second->handle_);
            ImGui::Button(button_tag.c_str(), {thumbnail_size_padding.x + 8, 20});
            ImGui::EndDisabled();
            ImGui::TextWrapped(i.second->name_.c_str());
            if (item_hovered)
              ImGui::PopStyleColor(1);
            ImGui::NextColumn();
          }
        }
        if (!updated) {
          for (auto& i : current_focused_folder->files) {
            auto file_name = i.second->GetAssetsFolderRelativePath().filename();
            if (file_name.string() == ".eveproj" || file_name.extension().string() == ".eveproj")
              continue;
            static Handle focused_asset_handle;
            bool item_focused = false;
            if (focused_asset_handle == i.first.GetValue()) {
              item_focused = true;
            }
            auto type_text = i.second->GetAssetTypeName();
            const std::string icon_tag = "##Icon" + std::to_string(i.first.GetValue());

            const auto thumbnail_tex = i.second->GetThumbnail();
            glm::vec2 resolution = thumbnail_tex->GetResolution();
            resolution *= thumbnail_size_padding.x / glm::max(resolution.x, resolution.y);
            ImGui::ImageButton(icon_tag.c_str(), thumbnail_tex->GetImTextureId(), {resolution.x, resolution.y}, {0, 1},
                               {1, 0});
            bool item_hovered = false;
            if (ImGui::IsItemHovered()) {
              item_hovered = true;
              if (ImGui::IsMouseDoubleClicked(0) && i.second->GetAssetTypeName() != "Binary") {
                // If it's an asset then inspect.
                if (auto asset = AssetManager::GetAssetImpl(i.second->asset_handle_))
                  editor_layer->inspecting_asset = asset;
              }
            }
            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
              ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
              ImGui::TextColored(ImVec4(0, 0, 1, 1), i.second->GetAssetFileName().c_str());
              ImGui::EndDragDropSource();
            }

            if (ImGui::BeginPopupContextItem(icon_tag.c_str())) {
              if (ImGui::Button("Duplicate")) {
                i.second->GetFolder().lock()->Duplicate(i.second->GetAssetHandle());
              }
              if (i.second->GetAssetTypeName() != "Binary" && ImGui::BeginMenu(("Rename" + icon_tag).c_str())) {
                static char new_name[256] = {};
                ImGui::InputText(("New name" + icon_tag).c_str(), new_name, 256);
                if (ImGui::Button(("Confirm" + icon_tag).c_str())) {
                  auto ptr = AssetManager::GetAssetImpl(i.second->asset_handle_);
                  ptr->SetPathAndSave(ptr->GetAssetsFolderRelativePath().replace_filename(
                      std::string(new_name) + ptr->GetFileRecord().lock()->GetAssetExtension()));
                  memset(new_name, 0, 256);
                }
                ImGui::EndMenu();
              }
              if (ImGui::Button(("Delete" + icon_tag).c_str())) {
                current_focused_folder->RemoveFile(i.first);
                ImGui::EndPopup();
                break;
              }

              ImGui::EndPopup();
            }

            if (item_focused)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 0, 0, 1});
            else if (item_hovered)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 1, 0, 1});
            ImGui::BeginDisabled();

            if (type_text == "Binary") {
              type_text = "??? (";
              type_text.append(i.second->asset_extension_);
              type_text.append(")");
            }
            const std::string button_tag = type_text + "##Button" + std::to_string(i.first.GetValue());
            ImGui::Button(button_tag.c_str(), {thumbnail_size_padding.x + 8, 20});
            ImGui::EndDisabled();
            if (show_extension) {
              ImGui::TextWrapped(file_name.string().c_str());
            } else {
              ImGui::TextWrapped(file_name.stem().string().c_str());
            }
            if (item_focused || item_hovered)
              ImGui::PopStyleColor(1);
            ImGui::NextColumn();
          }
        }

        ImGui::Columns(1);
        // ImGui::SliderFloat("Thumbnail Size", &thumbnailSizePadding.x, 16, 512);
        ImGui::EndChild();
      } else {
        ImGui::Text("No project loaded!");
      }
    }
    ImGui::End();
  }

  const auto asset_load_snapshot = AssetManager::GetAssetLoadSnapshot();
  if (project_manager.scan_assets_pending) {
    ImGui::OpenPopup("Scanning assets...");
  } else if (asset_load_snapshot.Active()) {
    ImGui::OpenPopup("Loading assets...");
  } else if (!project_manager.new_project_path_.empty()) {
    ImGui::OpenPopup("Loading Project...");
  }
  if (ImGui::BeginPopupModal("Loading Project...", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Busy...");
    if (project_manager.new_project_path_.empty()) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("Scanning assets...", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Busy...");
    if (!project_manager.scan_assets_pending) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("Loading assets...", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Progress: ");
    const auto completed_asset_count =
        asset_load_snapshot.completed + asset_load_snapshot.failed + asset_load_snapshot.cancelled;
    const auto active_asset_count = asset_load_snapshot.queued + asset_load_snapshot.loading_cpu +
                                    asset_load_snapshot.waiting_for_finalize + asset_load_snapshot.gpu_pending;
    auto total_asset_count = asset_load_snapshot.total;
    total_asset_count = std::max(total_asset_count, completed_asset_count + active_asset_count);
    total_asset_count = std::max(total_asset_count, project_manager.pending_asset_size);
    const float fraction = total_asset_count == 0
                               ? 1.0f
                               : static_cast<float>(completed_asset_count) / static_cast<float>(total_asset_count);
    const std::string text = std::to_string(static_cast<int>(fraction * 100.0f)) + "% - " +
                             std::to_string(completed_asset_count) + "/" + std::to_string(total_asset_count);
    ImGui::ProgressBar(fraction, ImVec2(240, 0), text.c_str());
    if (!asset_load_snapshot.active_asset_name.empty()) {
      ImGui::Text("Asset: %s", asset_load_snapshot.active_asset_name.c_str());
    }
    if (!asset_load_snapshot.message.empty()) {
      ImGui::Text("%s", asset_load_snapshot.message.c_str());
    }
    if (asset_load_snapshot.failed != 0 || asset_load_snapshot.cancelled != 0) {
      ImGui::Text("Failed: %zu  Cancelled: %zu", asset_load_snapshot.failed, asset_load_snapshot.cancelled);
    }
    ImGui::SetItemDefaultFocus();
    if (!asset_load_snapshot.Active()) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void ProjectManager::FolderHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer,
                                           const std::shared_ptr<Folder>& folder) {
  auto& project_manager = GetInstance();
  auto focus_folder = project_manager.current_focused_folder_.lock();
  const bool opened = ImGui::TreeNodeEx(
      folder->name_.c_str(), ImGuiTreeNodeFlags_OpenOnArrow |
                                 (folder == focus_folder ? ImGuiTreeNodeFlags_Selected : ImGuiTreeNodeFlags_None));
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (payload_n.GetValue() != 0) {
        if (const auto received_folder = FileManager::GetFolder(payload_n)) {
          if (!folder->IsSelfOrAncestor(received_folder->handle_) &&
              received_folder->parent_.lock().get() != folder.get()) {
            received_folder->parent_.lock()->MoveChild(received_folder->GetHandle(), folder);
          }
        }
      }
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (const auto asset = AssetManager::GetAssetImpl(payload_n)) {
        if (asset->IsTemporary()) {
          auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
          auto file_name = "New " + asset->GetTypeName();
          auto file_path =
              GenerateNewAssetsRelativePath((folder->GetAssetsRelativePath() / file_name).string(), file_extension);
          asset->SetPathAndSave(file_path);
        } else {
          if (auto asset_record = asset->file_record_.lock(); asset_record->GetFolder().lock().get() != folder.get()) {
            auto file_extension = asset_record->GetAssetExtension();
            auto file_name = asset_record->GetAssetFileName();
            auto file_path =
                GenerateNewAssetsRelativePath((folder->GetAssetsRelativePath() / file_name).string(), file_extension);
            asset->SetPathAndSave(file_path);
          }
        }
      } else {
        if (const auto file = FileManager::GetFile(payload_n)) {
          auto previous_folder = file->GetFolder().lock();
          if (folder && previous_folder.get() != folder.get()) {
            previous_folder->MoveAsset(file->GetAssetHandle(), folder);
          }
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    project_manager.current_focused_folder_ = folder;
  }
  const std::string tag = "##Folder" + std::to_string(folder->handle_);
  if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    ImGui::SetDragDropPayload("Folder", &folder->handle_, sizeof(Handle));
    ImGui::TextColored(ImVec4(0, 0, 1, 1), ("Folder" + tag).c_str());
    ImGui::EndDragDropSource();
  }
  if (folder->GetHandle() != 0) {
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256] = {};
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          folder->Rename(std::string(new_name));
          memset(new_name, 0, 256);
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndMenu();
      }
      if (ImGui::Button(("Remove" + tag).c_str())) {
        folder->parent_.lock()->DeleteChild(folder->handle_);
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
      }
      ImGui::EndPopup();
    }
  }
  if (opened) {
    for (const auto& i : folder->children_) {
      FolderHierarchyHelper(editor_layer, i.second);
    }
    for (const auto& i : folder->files) {
      if (ImGui::TreeNodeEx((i.second->GetAssetFileName() + i.second->GetAssetExtension()).c_str(),
                            ImGuiTreeNodeFlags_Bullet)) {
        ImGui::TreePop();
      }
      if (ImGui::IsItemHovered()) {
        if (ImGui::IsMouseDoubleClicked(0) && i.second->GetAssetTypeName() != "Binary") {
          // If it's an asset then inspect.
          if (auto asset = AssetManager::GetAssetImpl(i.second->asset_handle_))
            editor_layer->inspecting_asset = asset;
        }
      }
      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
        ImGui::TextColored(ImVec4(0, 0, 1, 1), i.second->GetAssetFileName().c_str());
        ImGui::EndDragDropSource();
      }
    }
    ImGui::TreePop();
  }
}

std::weak_ptr<Scene> ProjectManager::GetStartScene() {
  auto& project_manager = GetInstance();
  return project_manager.start_scene_;
}
void ProjectManager::SetStartScene(const std::shared_ptr<Scene>& scene) {
  auto& project_manager = GetInstance();
  project_manager.start_scene_ = scene;
}

std::filesystem::path ProjectManager::GetAssetsRelativePath(const std::filesystem::path& absolute_path) {
  const auto& project_manager = GetInstance();
  if (!project_manager.assets_folder_)
    return {};
  if (!absolute_path.is_absolute())
    return {};
  if (!IsInAssetsFolder(absolute_path))
    return {};
  return std::filesystem::relative(absolute_path, project_manager.assets_folder_path);
}
