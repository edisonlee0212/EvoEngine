#include "BuildManagerPanel.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <vector>
#include "Application.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "NativeBuildIdentity.hpp"
#include "PackageManager.hpp"
#include "PathUtils.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Utilities.hpp"
#ifdef _WIN32
#  include <bcrypt.h>
#  include <shellapi.h>
#endif
using json = nlohmann::json;
namespace evo_engine {
namespace {
std::filesystem::path CurrentTemplate(const NativeBuildIdentity& identity) {
  try {
    const auto root = path_utils::CurrentExecutablePath().parent_path() / "RuntimeTemplates" / identity.platform /
                      identity.architecture / identity.configuration;
    std::ifstream stream(root / "current.json");
    json pointer;
    stream >> pointer;
    if (pointer.value("schema_version", 0) != 1 || pointer.value("kind", "") != "EvoEngineRuntimeTemplatePointer")
      return {};
    const auto template_id = pointer.at("template_id").get<std::string>();
    if (template_id.size() != 64 || !std::all_of(template_id.begin(), template_id.end(), [](const unsigned char value) {
          return std::isdigit(value) || (value >= 'a' && value <= 'f');
        }))
      return {};
    return root / template_id;
  } catch (...) {
    return {};
  }
}
const char* ModeName(WindowDisplayMode mode) {
  switch (mode) {
    case WindowDisplayMode::Windowed:
      return "windowed";
    case WindowDisplayMode::BorderlessWindowed:
      return "borderless_windowed";
    case WindowDisplayMode::BorderlessFullscreen:
      return "borderless_fullscreen";
    case WindowDisplayMode::ExclusiveFullscreen:
      return "exclusive_fullscreen";
  }
  return "windowed";
}
bool TemplateMatches(const std::filesystem::path& runtime_template, const NativeBuildIdentity& editor) {
  try {
    std::ifstream stream(runtime_template / "template.json");
    json metadata;
    stream >> metadata;
    const auto& identity = metadata.at("identity");
    return identity.at("with_editor") == false && identity.at("sdk_source_id") == editor.sdk_source_id &&
           identity.at("compiler_id") == editor.compiler_id &&
           identity.at("compiler_version") == editor.compiler_version &&
           identity.at("configuration") == editor.configuration && identity.at("platform") == editor.platform &&
           identity.at("architecture") == editor.architecture;
  } catch (...) {
    return false;
  }
}
bool IsWithin(const std::filesystem::path& path, const std::filesystem::path& root) {
  std::error_code error;
  const auto relative = std::filesystem::relative(path, root, error);
  return !error && !relative.empty() && *relative.begin() != "..";
}
std::string Sha256(const std::filesystem::path& path) {
#ifdef _WIN32
  BCRYPT_ALG_HANDLE algorithm = nullptr;
  BCRYPT_HASH_HANDLE hash = nullptr;
  try {
    DWORD object_size = 0, hash_size = 0, got = 0;
    if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) < 0 ||
        BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&object_size), sizeof(object_size),
                          &got, 0) < 0 ||
        BCryptGetProperty(algorithm, BCRYPT_HASH_LENGTH, reinterpret_cast<PUCHAR>(&hash_size), sizeof(hash_size), &got,
                          0) < 0)
      throw std::runtime_error("Could not initialize source hashing.");
    std::vector<UCHAR> object(object_size), digest(hash_size);
    if (BCryptCreateHash(algorithm, &hash, object.data(), object_size, nullptr, 0, 0) < 0)
      throw std::runtime_error("Could not initialize the source file hash.");
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
      throw std::runtime_error("Could not open source file for hashing: " + path.string());
    std::vector<char> buffer(65536);
    while (stream) {
      stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
      if (const auto size = stream.gcount();
          size > 0 && BCryptHashData(hash, reinterpret_cast<PUCHAR>(buffer.data()), static_cast<ULONG>(size), 0) < 0)
        throw std::runtime_error("Could not hash source file: " + path.string());
    }
    if (!stream.eof() || BCryptFinishHash(hash, digest.data(), hash_size, 0) < 0)
      throw std::runtime_error("Could not finish source file hash: " + path.string());
    BCryptDestroyHash(hash);
    hash = nullptr;
    BCryptCloseAlgorithmProvider(algorithm, 0);
    algorithm = nullptr;
    constexpr char hex[] = "0123456789abcdef";
    std::string out;
    out.reserve(digest.size() * 2);
    for (const auto value : digest) {
      out += hex[value >> 4];
      out += hex[value & 15];
    }
    return out;
  } catch (...) {
    if (hash)
      BCryptDestroyHash(hash);
    if (algorithm)
      BCryptCloseAlgorithmProvider(algorithm, 0);
    throw;
  }
#else
  (void)path;
  throw std::runtime_error("Source hashing is supported on Windows x64.");
#endif
}
bool IsReparsePoint(const std::filesystem::path& path) {
#ifdef _WIN32
  const auto attributes = GetFileAttributesW(path.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_REPARSE_POINT);
#else
  std::error_code error;
  return std::filesystem::is_symlink(std::filesystem::symlink_status(path, error));
#endif
}
json SourceInventory(const std::filesystem::path& project) {
  json files = json::array();
  const auto root = project.parent_path();
  const auto assets = root / "Assets";
  if (IsReparsePoint(project) || IsReparsePoint(assets))
    throw std::runtime_error("Project source paths cannot be symlinks or reparse points.");
  std::vector<std::filesystem::path> paths{project};
  for (std::filesystem::recursive_directory_iterator iterator(assets), end; iterator != end; ++iterator) {
    if (IsReparsePoint(iterator->path())) {
      if (iterator->is_directory())
        iterator.disable_recursion_pending();
      throw std::runtime_error("Project assets cannot contain symlinks or reparse points: " +
                               iterator->path().string());
    }
    if (iterator->is_regular_file())
      paths.push_back(iterator->path());
  }
  std::sort(paths.begin(), paths.end());
  for (const auto& path : paths)
    files.push_back({{"path", path.lexically_relative(root).generic_u8string()},
                     {"size", std::filesystem::file_size(path)},
                     {"sha256", Sha256(path)}});
  return files;
}
json IdentityJson(const NativeBuildIdentity& i) {
  return {{"schema_version", 1},
          {"sdk_source_id", i.sdk_source_id},
          {"compiler_id", i.compiler_id},
          {"compiler_version", i.compiler_version},
          {"configuration", i.configuration},
          {"platform", i.platform},
          {"architecture", i.architecture},
          {"with_editor", true}};
}
void CleanupRequestDirectory(const std::filesystem::path& directory) {
  if (directory.empty())
    return;
  std::error_code error;
#ifdef _WIN32
  const auto attributes = GetFileAttributesW(directory.c_str());
  if (attributes == INVALID_FILE_ATTRIBUTES || (attributes & FILE_ATTRIBUTE_REPARSE_POINT))
    return;
#else
  if (std::filesystem::is_symlink(std::filesystem::symlink_status(directory, error)) || error)
    return;
#endif
  if (std::filesystem::is_regular_file(directory / ".evoengine-build-request", error) && !error)
    std::filesystem::remove_all(directory, error);
}
}  // namespace
BuildManagerPanel::BuildManagerPanel() = default;
BuildManagerPanel::~BuildManagerPanel() {
  job_.Wait();
  export_active_ = false;
}
bool BuildManagerPanel::ExportActive() const {
  return export_active_.load();
}
void BuildManagerPanel::RefreshFromProject() {
  draft_ = ProjectManager::GetBuildSettings();
  std::snprintf(application_name_.data(), application_name_.size(), "%s", draft_.application_name.c_str());
  const auto output = draft_.output_directory.u8string();
  std::snprintf(output_directory_.data(), output_directory_.size(), "%s", output.c_str());
  startup_scene_.Clear();
  if (draft_.startup_scene_handle)
    startup_scene_.Set(AssetManager::GetAsset<Scene>(Handle(draft_.startup_scene_handle)));
  else if (const auto scene = ApplicationContext::Get().GetActiveScene();
           scene && ApplicationContext::Get().GetApplicationStatus() == Application::ExecutionStatus::NotPlaying &&
           ProjectManager::GetProjectState() == ProjectState::Loaded && ProjectManager::IsProjectIdle()) {
    startup_scene_.Set(scene);
    draft_.startup_scene_handle = scene->GetHandle().GetValue();
    ProjectManager::SetBuildSettings(draft_);
  }
  loaded_project_path_ = ProjectManager::GetProjectPath();
  initialized_ = true;
}
BuildPreflightResult BuildManagerPanel::Preflight(std::shared_ptr<Scene>& scene,
                                                  std::filesystem::path& runtime_template) const {
  BuildPreflightSnapshot snapshot;
  snapshot.project_loaded = ProjectManager::GetProjectState() == ProjectState::Loaded;
  snapshot.project_idle = ProjectManager::IsProjectIdle();
  snapshot.editor_stopped =
      ApplicationContext::Get().GetApplicationStatus() == Application::ExecutionStatus::NotPlaying;
  snapshot.build_settings_saved = ProjectManager::ProjectMetadataSaved();
  scene = startup_scene_.Peek<Scene>();
  snapshot.startup_scene_loadable = scene != nullptr;
  snapshot.startup_scene_saved =
      scene && !scene->IsTemporary() && scene->Saved() && std::filesystem::is_regular_file(scene->GetAbsolutePath());
  snapshot.main_camera_present = snapshot.main_camera_enabled = snapshot.main_camera_owner_enabled = true;
  if (scene)
    snapshot.content_errors = scene->ValidateRuntimeStartupContent();
  for (const auto& package : PackageManager::GetLoadedPackages())
    if (!package.ready)
      snapshot.content_errors.emplace_back("Package activation failed: " + package.name);
  for (const auto& asset : AssetManager::GetLoadedAssetStates())
    if (!asset.temporary && !asset.saved)
      snapshot.unsaved_project_assets.emplace_back(asset.path.generic_string());
  runtime_template = CurrentTemplate(GetNativeBuildIdentity());
  snapshot.template_compatible = TemplateMatches(runtime_template, GetNativeBuildIdentity());
  snapshot.destination_available =
      !IsWithin(draft_.output_directory, ProjectManager::GetProjectFolderPath()) &&
      (!std::filesystem::exists(draft_.output_directory) ||
       (std::filesystem::is_directory(draft_.output_directory) && std::filesystem::is_empty(draft_.output_directory)));
  return ValidateBuildPreflight(draft_, snapshot);
}
void BuildManagerPanel::StartExport() {
  if (job_.Active())
    return;
  status_.clear();
  std::shared_ptr<Scene> scene;
  std::filesystem::path runtime_template;
  json source_inventory;
  try {
    if (ProjectManager::GetProjectState() == ProjectState::Loaded && ProjectManager::IsProjectIdle() &&
        ApplicationContext::Get().GetApplicationStatus() == Application::ExecutionStatus::NotPlaying) {
      ProjectManager::SetBuildSettings(draft_);
      ProjectManager::SaveProject();
    }
    errors_ = Preflight(scene, runtime_template).errors;
    if (!errors_.empty())
      return;
    source_inventory = SourceInventory(ProjectManager::GetProjectPath());
  } catch (const std::exception& error) {
    errors_ = {error.what()};
    return;
  }
  const auto identity = GetNativeBuildIdentity();
  json packages = json::array();
  for (const auto& package : PackageManager::GetLoadedPackages())
    packages.push_back({{"name", package.name}, {"source_id", package.package_source_id}});
  const auto& graphics = ApplicationContext::Get().GetApplicationInfo().graphics_settings;
  json request{{"schema_version", 1},
               {"application_name", draft_.application_name},
               {"startup_scene_handle", draft_.startup_scene_handle},
               {"project", ProjectManager::GetProjectPath().u8string()},
               {"editor_identity", IdentityJson(identity)},
               {"loaded_packages", packages},
               {"source_inventory", std::move(source_inventory)},
               {"runtime_config",
                {{"show_console", draft_.show_console},
                 {"window",
                  {{"width", draft_.window_width},
                   {"height", draft_.window_height},
                   {"mode", ModeName(draft_.window_mode)},
                   {"allow_resize", draft_.allow_window_resize},
                   {"allow_resolution_change", draft_.allow_resolution_change}}},
                 {"graphics",
                  {{"use_mesh_shader", graphics.use_mesh_shader},
                   {"use_ray_tracing", graphics.use_ray_tracing},
                   {"directional_light_shadow_map_resolution", graphics.directional_light_shadow_map_resolution},
                   {"point_light_shadow_map_resolution", graphics.point_light_shadow_map_resolution},
                   {"spot_light_shadow_map_resolution", graphics.spot_light_shadow_map_resolution},
                   {"max_texture_2d_resource_size", graphics.max_texture_2d_resource_size},
                   {"max_cubemap_resource_size", graphics.max_cubemap_resource_size},
                   {"max_directional_light_size", graphics.max_directional_light_size},
                   {"max_point_light_size", graphics.max_point_light_size},
                   {"max_spot_light_size", graphics.max_spot_light_size}}}}}};
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto request_root = ProjectManager::GetProjectFolderPath() / "Cache" / "BuildManager";
  const auto request_dir = request_root / std::to_string(nonce);
  const auto request_path = request_dir / "request.json";
  try {
    std::filesystem::create_directories(request_root);
    if (!std::filesystem::create_directory(request_dir))
      throw std::runtime_error("Could not create a unique runtime build request directory.");
    std::ofstream marker(request_dir / ".evoengine-build-request");
    marker << "schema_version=1\n";
    marker.close();
    if (!marker)
      throw std::runtime_error("Could not create the runtime build request marker.");
    std::ofstream stream(request_path);
    stream << request.dump(2);
    stream.close();
    if (!stream)
      throw std::runtime_error("Could not write the runtime build request.");
    const auto exporter = path_utils::CurrentExecutablePath().parent_path() / "EvoEngineRuntimeExporter.exe";
    std::string error;
    if (!job_.Start(exporter, request_path, runtime_template, draft_.output_directory, error)) {
      errors_ = {error};
      return;
    }
  } catch (const std::exception& error) {
    CleanupRequestDirectory(request_dir);
    std::error_code cleanup_error;
    std::filesystem::remove(request_dir, cleanup_error);
    errors_ = {error.what()};
    return;
  }
  export_output_directory_ = draft_.output_directory;
  export_active_ = true;
  status_ = "Exporting runtime application...";
}
void BuildManagerPanel::Tick() {
  if (job_.Active())
    job_.Poll();
  if (job_.Finished() && export_active_.exchange(false)) {
    status_ = job_.Succeeded() ? "Build completed." : job_.Output();
#ifdef _WIN32
    if (job_.Succeeded()) {
      const auto shell_result = reinterpret_cast<INT_PTR>(
          ShellExecuteW(nullptr, L"open", export_output_directory_.c_str(), nullptr, nullptr, SW_SHOWNORMAL));
      if (shell_result <= 32)
        status_ = "Build completed, but Explorer could not open the distribution (ShellExecute result " +
                  std::to_string(shell_result) + "). Output: " + export_output_directory_.string();
    }
#endif
  }
}
void BuildManagerPanel::Draw(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (!initialized_ || loaded_project_path_ != ProjectManager::GetProjectPath() ||
      (draft_.startup_scene_handle == 0 && ApplicationContext::Get().GetActiveScene()))
    RefreshFromProject();

  if (!ImGui::Begin("Build Manager")) {
    ImGui::End();
    return;
  }
  ImGui::BeginDisabled(job_.Active());
  bool changed = ImGui::InputText("Application name", application_name_.data(), application_name_.size());
  changed |= ImGui::InputText("Output directory", output_directory_.data(), output_directory_.size());
  EditorFileDialogs::OpenFolder(
      "Browse...",
      [&](const std::filesystem::path& path) {
        const auto output = path.u8string();
        std::snprintf(output_directory_.data(), output_directory_.size(), "%s", output.c_str());
        changed = true;
      },
      false);
  if (editor_layer->DragAndDropButton<Scene>(startup_scene_, "Startup Scene"))
    changed = true;
  int window_size[2] = {draft_.window_width, draft_.window_height};
  if (ImGui::InputInt2("Window size", window_size)) {
    draft_.window_width = window_size[0];
    draft_.window_height = window_size[1];
    changed = true;
  }
  const char* modes[] = {"Windowed", "Borderless windowed", "Borderless fullscreen", "Exclusive fullscreen"};
  int mode = static_cast<int>(draft_.window_mode);
  if (ImGui::Combo("Window mode", &mode, modes, 4)) {
    draft_.window_mode = static_cast<WindowDisplayMode>(mode);
    changed = true;
  }
  changed |= ImGui::Checkbox("Allow window resize", &draft_.allow_window_resize);
  changed |= ImGui::Checkbox("Allow resolution changes", &draft_.allow_resolution_change);
  changed |= ImGui::Checkbox("Show Windows console", &draft_.show_console);
  if (changed) {
    draft_.application_name = application_name_.data();
    draft_.output_directory = std::filesystem::u8path(output_directory_.data());
    draft_.startup_scene_handle = startup_scene_.GetAssetHandle().GetValue();
    ProjectManager::SetBuildSettings(draft_);
  }
  ImGui::Text("Target: Windows x64   Configuration: %s", GetNativeBuildIdentity().configuration);
  if (ImGui::Button("Build"))
    StartExport();
  ImGui::EndDisabled();
  for (const auto& error : errors_)
    ImGui::TextWrapped("%s", error.c_str());
  if (!status_.empty())
    ImGui::TextWrapped("%s", status_.c_str());
  if (job_.Active())
    ImGui::OpenPopup("Building runtime application");
  if (ImGui::BeginPopupModal("Building runtime application", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted(status_.c_str());
    if (!job_.Output().empty())
      ImGui::TextUnformatted(job_.Output().c_str());
    if (!job_.Active())
      ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
  ImGui::End();
}
}  // namespace evo_engine
