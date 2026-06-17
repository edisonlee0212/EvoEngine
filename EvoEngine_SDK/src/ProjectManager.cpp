#include "ProjectManager.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "PathUtils.hpp"
#include "Profiler.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string>
#include <vector>

using namespace evo_engine;

namespace {
constexpr const char* kDefaultEditorName = "EvoEngineEditor";
constexpr const char* kDefaultApplicationName = "EvoEngine Editor";
using LoadingClock = std::chrono::steady_clock;

int64_t ElapsedMilliseconds(const LoadingClock::time_point start) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(LoadingClock::now() - start).count();
}

void LogLoadingDuration(const std::string& stage, const LoadingClock::time_point start,
                        const std::string& detail = {}) {
  auto message = "Project loading: " + stage + " took " + std::to_string(ElapsedMilliseconds(start)) + " ms";
  if (!detail.empty()) {
    message += " (" + detail + ")";
  }
  EVOENGINE_LOG(message)
}

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
  return path_utils::CurrentExecutablePath("EvoEngineEditor.exe");
#else
  return path_utils::CurrentExecutablePath(kDefaultEditorName);
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

std::shared_ptr<Folder> ProjectManager::CreateFolder(const std::shared_ptr<Folder>& folder,
                                                     const std::string& folder_name) {
  if (!folder || folder_name.empty()) {
    return {};
  }
  const auto new_path = GenerateNewAssetsRelativePath((folder->GetAssetsRelativePath() / folder_name).string(), "");
  return GetOrCreateFolder(new_path).lock();
}

std::shared_ptr<IAsset> ProjectManager::CreateAsset(const std::shared_ptr<Folder>& folder,
                                                    const std::string& type_name) {
  if (!folder || type_name.empty()) {
    return {};
  }
  const auto& extensions = Serialization::PeekAssetExtensions(type_name);
  if (extensions.empty()) {
    EVOENGINE_ERROR("Asset type has no registered extensions: " + type_name)
    return {};
  }
  const auto new_path = GenerateNewAssetsRelativePath((folder->GetAssetsRelativePath() / ("New " + type_name)).string(),
                                                      extensions.front());
  return folder->GetOrCreateAsset(new_path.stem().string(), new_path.extension().string());
}

bool ProjectManager::SaveAsset(const std::shared_ptr<IAsset>& asset, const std::shared_ptr<Folder>& folder,
                               const std::string& file_stem, const std::string& extension,
                               const bool generate_unique_path) {
  if (!asset || !folder || file_stem.empty() || extension.empty()) {
    return false;
  }
  auto preferred_path = folder->GetAssetsRelativePath() / file_stem;
  const auto asset_path = generate_unique_path ? GenerateNewAssetsRelativePath(preferred_path.string(), extension)
                                               : preferred_path.replace_extension(extension);
  return asset->SetPathAndSave(asset_path);
}

bool ProjectManager::MoveAsset(const Handle& asset_handle, const std::shared_ptr<Folder>& folder) {
  if (!folder) {
    return false;
  }
  const auto file = FileManager::GetFile(asset_handle);
  if (!file || file->GetAssetTypeName() != "Binary") {
    try {
      if (const auto asset = AssetManager::GetAssetImpl(asset_handle)) {
        if (asset->IsTemporary()) {
          const auto& extensions = Serialization::PeekAssetExtensions(asset->GetTypeName());
          if (extensions.empty()) {
            EVOENGINE_ERROR("Asset type has no registered extensions: " + asset->GetTypeName())
            return false;
          }
          return SaveAsset(asset, folder, "New " + asset->GetTypeName(), extensions.front());
        }

        const auto asset_record = asset->file_record_.lock();
        if (!asset_record) {
          return false;
        }
        const auto source_folder = asset_record->GetFolder().lock();
        if (source_folder.get() == folder.get()) {
          return false;
        }
        return SaveAsset(asset, folder, asset_record->GetAssetFileName(), asset_record->GetAssetExtension());
      }
    } catch (const std::exception& error) {
      if (!file) {
        EVOENGINE_ERROR(error.what())
      }
    }
  }
  if (!file) {
    return false;
  }
  const auto source_folder = file->GetFolder().lock();
  if (!source_folder || source_folder.get() == folder.get()) {
    return false;
  }
  try {
    source_folder->MoveAsset(file->GetAssetHandle(), folder);
  } catch (const std::exception& error) {
    EVOENGINE_ERROR(error.what())
    return false;
  }
  return true;
}

bool ProjectManager::MoveFolder(const Handle& folder_handle, const std::shared_ptr<Folder>& destination_folder) {
  if (!destination_folder || folder_handle.GetValue() == 0) {
    return false;
  }
  const auto folder = FileManager::GetFolder(folder_handle);
  if (!folder || destination_folder->IsSelfOrAncestor(folder_handle)) {
    return false;
  }
  const auto source_parent = folder->parent_.lock();
  if (!source_parent || source_parent.get() == destination_folder.get()) {
    return false;
  }
  source_parent->MoveChild(folder->GetHandle(), destination_folder);
  return true;
}

bool ProjectManager::DeleteAsset(const Handle& asset_handle) {
  const auto file = FileManager::GetFile(asset_handle);
  if (!file) {
    return false;
  }
  const auto folder = file->GetFolder().lock();
  if (!folder) {
    return false;
  }
  folder->RemoveFile(asset_handle);
  return true;
}

bool ProjectManager::DeleteFolder(const Handle& folder_handle) {
  if (folder_handle.GetValue() == 0) {
    return false;
  }
  const auto folder = FileManager::GetFolder(folder_handle);
  if (!folder) {
    return false;
  }
  const auto parent = folder->parent_.lock();
  if (!parent) {
    return false;
  }
  parent->DeleteChild(folder_handle);
  return true;
}

void ProjectManager::SetupDefaultScene() {
  if (ArmSceneLoadingPopupBeforeSetup()) {
    return;
  }
  const ProfilerScope profiler_scope("ProjectManager::SetupDefaultScene", "Scene Load");
  auto& project_manager = GetInstance();
  project_manager.loading_status_ = "Loading start scene...";
  const auto setup_start = LoadingClock::now();
  auto project_absolute_path = std::filesystem::absolute(project_manager.new_project_path_);
  bool found_scene = false;
  std::shared_ptr<Scene> scene;
  if (std::filesystem::exists(project_absolute_path)) {
    const auto project_file_read_start = LoadingClock::now();
    std::ifstream stream(project_absolute_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    YAML::Node in = YAML::Load(string_stream.str());
    LogLoadingDuration("Project manifest read", project_file_read_start, project_absolute_path.filename().string());
    const uint64_t scene_handle = ReadStartSceneHandle(in);
    if (scene_handle != 0) {
      project_manager.loading_status_ = "Loading start scene asset...";
      const auto scene_load_start = LoadingClock::now();
      if (auto temp = AssetManager::GetAssetImpl(scene_handle)) {
        scene = std::dynamic_pointer_cast<Scene>(temp);
        SetStartScene(scene);
        SaveProject();
        LogLoadingDuration("Start scene asset load", scene_load_start);
        project_manager.loading_status_ = "Attaching start scene...";
        const auto attach_start = LoadingClock::now();
        ApplicationContext::Get().Attach(scene);
        LogLoadingDuration("Start scene attach", attach_start);
        found_scene = true;
      }
    }
    EVOENGINE_LOG("Found and loaded project")
    if (found_scene && project_manager.scene_post_load_function_.has_value()) {
      project_manager.loading_status_ = "Running scene post-load actions...";
      const auto post_load_start = LoadingClock::now();
      project_manager.scene_post_load_function_.value()(scene);
      LogLoadingDuration("Scene post-load actions", post_load_start);
      project_manager.loading_status_ = "Synchronizing scene transforms...";
      const auto transform_start = LoadingClock::now();
      const ProfilerScope transform_scope("ProjectManager::SceneTransformSync", "Scene Sync");
      TransformGraph::CalculateTransformGraphs(scene);
      LogLoadingDuration("Scene transform graph sync", transform_start);
    }
  }
  if (!found_scene) {
    project_manager.loading_status_ = "Creating start scene...";
    const auto create_scene_start = LoadingClock::now();
    scene = AssetManager::CreateTemporaryAsset<Scene>();
    if (std::filesystem::path new_scene_relative_path = GenerateNewAssetsRelativePath("New Scene", ".evescene");
        scene->SetPathAndSave(new_scene_relative_path)) {
      EVOENGINE_LOG("Created new start scene!")
    }
    SetStartScene(scene);
    SaveProject();
    LogLoadingDuration("Default scene creation", create_scene_start);
    project_manager.loading_status_ = "Attaching start scene...";
    const auto attach_start = LoadingClock::now();
    ApplicationContext::Get().Attach(scene);
    LogLoadingDuration("Start scene attach", attach_start);

    if (project_manager.new_scene_customizer_.has_value()) {
      project_manager.loading_status_ = "Running new-scene actions...";
      const auto customizer_start = LoadingClock::now();
      project_manager.new_scene_customizer_.value()(scene);
      LogLoadingDuration("New-scene actions", customizer_start);
      project_manager.loading_status_ = "Synchronizing scene transforms...";
      const auto transform_start = LoadingClock::now();
      const ProfilerScope transform_scope("ProjectManager::SceneTransformSync", "Scene Sync");
      TransformGraph::CalculateTransformGraphs(scene);
      LogLoadingDuration("Scene transform graph sync", transform_start);
    }
  }

  project_manager.loading_status_ = "Scene ready.";
  LogLoadingDuration("Start scene setup", setup_start);
  project_manager.new_project_path_ = "";
  project_manager.scene_loading_popup_visible_ = false;
}

bool ProjectManager::ArmSceneLoadingPopupBeforeSetup() {
  auto& project_manager = GetInstance();
  if (project_manager.scene_loading_popup_visible_) {
    return false;
  }
  if (!ApplicationContext::Get().GetLayer<WindowLayer>() || !ApplicationContext::Get().GetLayer<EditorLayer>()) {
    return false;
  }
  project_manager.scene_loading_popup_visible_ = true;
  project_manager.loading_status_ = "Loading start scene...";
  return true;
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

  const auto& application_info = ApplicationContext::Get().GetApplicationInfo();
  if (!project_manager.new_project_path_.empty()) {
    if (application_info.load_project_start_scene) {
      SetupDefaultScene();
      return;
    }
    project_manager.new_project_path_ = "";
  }

  if (project_manager.project_asset_load_dispatched) {
    const auto snapshot = AssetManager::GetAssetLoadSnapshot();
    if (snapshot.Active()) {
      return;
    }
    project_manager.project_asset_load_dispatched = false;
    project_manager.pending_asset_size = 0;
    project_manager.loading_status_ = project_manager.start_scene_ ? "Scene ready." : "Project assets loaded.";
  }

  if (!application_info.load_project_assets) {
    project_manager.pending_assets.clear();
    project_manager.pending_asset_size = 0;
    return;
  }

  if (!project_manager.pending_assets.empty()) {
    LoadAllPendingAssets();
  }
}

bool ProjectManager::IsProjectIdle() {
  const auto& project_manager = GetInstance();
  const auto asset_load_snapshot = AssetManager::GetAssetLoadSnapshot();
  return IsProjectLoaded() && project_manager.pending_assets.empty() &&
         !project_manager.project_asset_load_dispatched && project_manager.pending_asset_size == 0 &&
         !asset_load_snapshot.Active();
}

ProjectState ProjectManager::GetProjectState() {
  const auto& project_manager = GetInstance();
  if (!HasProject()) {
    return ProjectState::NoProject;
  }

  if (project_manager.new_project_path_.empty() && !project_manager.scan_assets_pending &&
      project_manager.start_scene_) {
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
  const ProfilerScope profiler_scope("ProjectManager::LoadAllPendingAssets", "Asset Load");
  auto& project_manager = GetInstance();
  if (project_manager.pending_assets.empty()) {
    return;
  }

  const auto dispatch_start = LoadingClock::now();
  project_manager.loading_status_ =
      project_manager.start_scene_ ? "Loading remaining project assets..." : "Loading project assets...";
  std::set<Handle> handles_to_load;
  std::vector<Handle> blocking_handles;
  blocking_handles.reserve(project_manager.pending_assets.size());
  for (const auto& handle : project_manager.pending_assets) {
    if (AssetManager::PeekAssetImpl(handle)) {
      continue;
    }
    handles_to_load.emplace(handle);
    blocking_handles.emplace_back(handle);
  }

  if (handles_to_load.empty()) {
    project_manager.pending_assets.clear();
    project_manager.pending_asset_size = 0;
    project_manager.loading_status_ = project_manager.start_scene_ ? "Scene ready." : "Project assets loaded.";
    return;
  }

  project_manager.pending_asset_size = handles_to_load.size();
  [[maybe_unused]] const auto load_futures = AssetManager::RequestAssetLoads(handles_to_load);
  project_manager.project_asset_load_dispatched = true;
  project_manager.pending_assets.clear();
  LogLoadingDuration("Project asset load dispatch", dispatch_start,
                     std::to_string(project_manager.pending_asset_size) + " assets");

  if (ApplicationContext::Get().GetLayer<WindowLayer>()) {
    return;
  }

  const auto blocking_load_start = LoadingClock::now();
  const ProfilerScope blocking_load_scope("ProjectManager::BlockingProjectAssetLoad", "Asset Load");
  for (const auto& handle : blocking_handles) {
    AssetManager::GetAssetImpl(handle);
  }
  project_manager.project_asset_load_dispatched = false;
  project_manager.pending_asset_size = 0;
  project_manager.loading_status_ = "Project assets loaded.";
  LogLoadingDuration("Blocking project asset load", blocking_load_start,
                     std::to_string(blocking_handles.size()) + " assets");
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
  if (project_manager.assets_folder_path.empty()) {
    return false;
  }
  return path_utils::IsSameOrChildPath(absolute_path, project_manager.assets_folder_path);
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
  const ProfilerScope profiler_scope("ProjectManager::GetOrCreateProject", "Project");
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
  project_manager.loading_status_ = "Opening project...";
  project_manager.scene_loading_popup_visible_ = false;
  project_manager.start_scene_.reset();
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
    } else {
      project_manager.pending_assets.clear();
      project_manager.pending_asset_size = 0;
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
  return path_utils::GenerateUniqueChildPath(project_manager.assets_folder_path, relative_stem, postfix);
}

std::filesystem::path ProjectManager::GenerateNewAbsolutePath(const std::string& absolute_stem,
                                                              const std::string& postfix) {
  return path_utils::GenerateUniquePath(absolute_stem, postfix);
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
  const ProfilerScope profiler_scope("ProjectManager::ScanAssets", "Asset Load");
  auto& project_manager = GetInstance();
  project_manager.loading_status_ = "Scanning assets...";
  const auto scan_start = LoadingClock::now();
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
  LogLoadingDuration("Asset metadata scan", scan_start, std::to_string(missing_asset_handles.size()) + " assets");
  project_manager.loading_status_ =
      missing_asset_handles.empty() ? "Asset scan complete." : "Asset scan complete. Loading start scene...";
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
  project_manager.scene_loading_popup_visible_ = false;
  project_manager.assets_folder_.reset();
  project_manager.new_scene_customizer_.reset();
  project_manager.current_focused_folder_.reset();
  project_manager.start_scene_.reset();
  project_manager.project_launch_metadata_ = {};
  project_manager.new_project_path_ = "";
  project_manager.project_path_ = "";
  project_manager.assets_folder_path = "";
  project_manager.loading_status_.clear();

  project_manager.initialized = false;
}

void ProjectManager::DrawProjectMenuItems() {
  static std::string close_project_error;

  if (ImGui::MenuItem("Save Project")) {
    SaveProject();
  }
  if (ImGui::MenuItem("Close Project")) {
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
  const auto relative_path = path_utils::RelativePathIfContained(absolute_path, project_manager.assets_folder_path);
  return relative_path.value_or(std::filesystem::path());
}
