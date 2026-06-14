#include "ProjectManager.hpp"
#include "Application.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <optional>
#include <string>
#include <system_error>
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

std::filesystem::path NormalizePathForContainment(const std::filesystem::path& path) {
  std::error_code error;
  auto normalized = std::filesystem::weakly_canonical(path, error);
  if (error) {
    normalized = std::filesystem::absolute(path, error);
  }
  if (error) {
    normalized = path;
  }
  return normalized.lexically_normal();
}

bool PathElementEquals(const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  auto lhs_string = lhs.string();
  auto rhs_string = rhs.string();
  std::transform(lhs_string.begin(), lhs_string.end(), lhs_string.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::transform(rhs_string.begin(), rhs_string.end(), rhs_string.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return lhs_string == rhs_string;
#else
  return lhs == rhs;
#endif
}

bool IsSamePathOrChildPath(const std::filesystem::path& path, const std::filesystem::path& parent) {
  auto path_iterator = path.begin();
  for (auto parent_iterator = parent.begin(); parent_iterator != parent.end(); ++parent_iterator, ++path_iterator) {
    if (path_iterator == path.end() || !PathElementEquals(*path_iterator, *parent_iterator)) {
      return false;
    }
  }
  return true;
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
  if (project_manager.assets_folder_path.empty()) {
    return false;
  }
  const auto path = NormalizePathForContainment(absolute_path);
  const auto assets_folder_path = NormalizePathForContainment(project_manager.assets_folder_path);
  return IsSamePathOrChildPath(path, assets_folder_path);
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
  std::error_code error;
  auto relative_path =
      std::filesystem::relative(NormalizePathForContainment(absolute_path),
                                NormalizePathForContainment(project_manager.assets_folder_path), error);
  if (error) {
    relative_path = std::filesystem::relative(absolute_path, project_manager.assets_folder_path, error);
  }
  return error ? std::filesystem::path() : relative_path;
}
