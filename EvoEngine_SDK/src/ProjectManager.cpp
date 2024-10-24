#include "ProjectManager.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Prefab.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  include "shellapi.h"
#endif

using namespace evo_engine;

std::shared_ptr<IAsset> AssetRecord::GetAsset() {
  if (!asset_.expired())
    return asset_.lock();
  if (!asset_type_name_.empty() && asset_type_name_ != "Binary" && asset_handle_ != 0) {
    size_t hash_code;
    auto ret_val = std::dynamic_pointer_cast<IAsset>(
        Serialization::ProduceSerializable(asset_type_name_, hash_code, asset_handle_));
    ret_val->asset_record_ = self_;
    ret_val->self_ = ret_val;
    ret_val->OnCreate();
    if (const auto absolute_path = GetAbsolutePath(); std::filesystem::exists(absolute_path)) {
      ret_val->Load();
    } else {
      ret_val->Save();
    }
    asset_ = ret_val;
    auto& project_manager = ProjectManager::GetInstance();
    project_manager.asset_registry_[asset_handle_] = ret_val;
    project_manager.loaded_assets_[asset_handle_] = ret_val;
    project_manager.asset_record_registry_[asset_handle_] = self_;
    return ret_val;
  }
  return nullptr;
}
std::string AssetRecord::GetAssetTypeName() const {
  return asset_type_name_;
}
std::string AssetRecord::GetAssetFileName() const {
  return asset_file_name_;
}
std::string AssetRecord::GetAssetExtension() const {
  return asset_extension_;
}
std::filesystem::path AssetRecord::GetProjectRelativePath() const {
  if (folder_.expired()) {
    EVOENGINE_ERROR("Folder expired!")
    return {};
  }
  return folder_.lock()->GetProjectRelativePath() / (asset_file_name_ + asset_extension_);
}
std::filesystem::path AssetRecord::GetAbsolutePath() const {
  if (folder_.expired()) {
    EVOENGINE_ERROR("Folder expired!")
    return {};
  }
  return folder_.lock()->GetAbsolutePath() / (asset_file_name_ + asset_extension_);
}
void AssetRecord::SetAssetFileName(const std::string& new_name) {
  if (asset_file_name_ == new_name)
    return;
  // TODO: Check invalid filename.
  const std::filesystem::path old_path = GetAbsolutePath();
  auto new_path = old_path;
  new_path.replace_filename(new_name + old_path.extension().string());
  if (std::filesystem::exists(new_path)) {
    EVOENGINE_ERROR("File with new name already exists!")
    return;
  }
  DeleteMetadata();
  asset_file_name_ = new_name;
  if (std::filesystem::exists(old_path)) {
    std::filesystem::rename(old_path, new_path);
  }
  Save();
}
void AssetRecord::SetAssetExtension(const std::string& new_extension) {
  if (asset_type_name_ == "Binary") {
    EVOENGINE_ERROR("File is binary!")
    return;
  }
  const auto& valid_extensions = Serialization::PeekAssetExtensions(asset_type_name_);
  bool found = false;
  for (const auto& i : valid_extensions) {
    if (i == new_extension) {
      found = true;
      break;
    }
  }
  if (!found) {
    EVOENGINE_ERROR("Extension not valid!")
    return;
  }
  const auto old_path = GetAbsolutePath();
  auto new_path = old_path;
  new_path.replace_extension(new_extension);
  if (std::filesystem::exists(new_path)) {
    EVOENGINE_ERROR("File with new name already exists!")
    return;
  }
  DeleteMetadata();
  asset_extension_ = new_extension;
  if (std::filesystem::exists(old_path)) {
    std::filesystem::rename(old_path, new_path);
  }
  Save();
}
void AssetRecord::Save() const {
  auto path = GetAbsolutePath().string() + ".evefilemeta";
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "asset_extension_" << YAML::Value << asset_extension_;
  out << YAML::Key << "asset_file_name_" << YAML::Value << asset_file_name_;
  out << YAML::Key << "asset_type_name_" << YAML::Value << asset_type_name_;
  out << YAML::Key << "asset_handle_" << YAML::Value << asset_handle_;
  out << YAML::EndMap;
  std::ofstream file_out(path);
  file_out << out.c_str();
  file_out.close();
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  const DWORD attributes = GetFileAttributes(path.c_str());
  SetFileAttributes(path.c_str(), attributes | FILE_ATTRIBUTE_HIDDEN);
#endif
}

Handle AssetRecord::GetAssetHandle() const {
  return asset_handle_;
}
void AssetRecord::DeleteMetadata() const {
  const auto path = GetAbsolutePath().string() + ".evefilemeta";
  std::filesystem::remove(path);
}
void AssetRecord::Load(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Metadata not exist!")
    return;
  }
  const std::ifstream stream(path.string());
  std::stringstream string_stream;
  string_stream << stream.rdbuf();
  YAML::Node in = YAML::Load(string_stream.str());
  if (in["asset_file_name_"])
    asset_file_name_ = in["asset_file_name_"].as<std::string>();
  if (in["asset_extension_"])
    asset_extension_ = in["asset_extension_"].as<std::string>();
  if (in["asset_type_name_"])
    asset_type_name_ = in["asset_type_name_"].as<std::string>();
  if (in["asset_handle_"])
    asset_handle_ = in["asset_handle_"].as<uint64_t>();

  if (!Serialization::HasSerializableType(asset_type_name_)) {
    asset_type_name_ = "Binary";
  }
}
std::weak_ptr<Folder> AssetRecord::GetFolder() const {
  return folder_;
}
std::filesystem::path Folder::GetProjectRelativePath() const {
  if (parent_.expired()) {
    return "";
  }
  return parent_.lock()->GetProjectRelativePath() / name_;
}
std::filesystem::path Folder::GetAbsolutePath() const {
  const auto& project_manager = ProjectManager::GetInstance();
  const auto project_path = project_manager.project_path_.parent_path();
  return project_path / GetProjectRelativePath();
}

Handle Folder::GetHandle() const {
  return handle_;
}
std::string Folder::GetName() const {
  return name_;
}
void Folder::Rename(const std::string& new_name) {
  const auto old_path = GetAbsolutePath();
  auto new_path = old_path;
  new_path.replace_filename(new_name);
  if (std::filesystem::exists(new_path)) {
    EVOENGINE_ERROR("Folder with new name already exists!")
    return;
  }
  DeleteMetadata();
  name_ = new_name;
  if (std::filesystem::exists(old_path)) {
    std::filesystem::rename(old_path, new_path);
  }
  Save();
}
void Folder::Save() const {
  const auto path = GetAbsolutePath().string() + ".evefoldermeta";
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "handle_" << YAML::Value << handle_;
  out << YAML::Key << "type_name" << YAML::Value << name_;
  out << YAML::EndMap;
  std::ofstream file_out(path);
  file_out << out.c_str();
  file_out.close();
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  const DWORD attributes = GetFileAttributes(path.c_str());
  SetFileAttributes(path.c_str(), attributes | FILE_ATTRIBUTE_HIDDEN);
#endif
}
void Folder::Load(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Folder metadata not exist!")
    return;
  }
  const std::ifstream stream(path.string());
  std::stringstream string_stream;
  string_stream << stream.rdbuf();
  YAML::Node in = YAML::Load(string_stream.str());
  if (in["handle_"])
    handle_ = in["handle_"].as<uint64_t>();
  if (in["type_name"])
    name_ = in["type_name"].as<std::string>();
}
void Folder::DeleteMetadata() const {
  const auto path = GetAbsolutePath().replace_extension(".evefoldermeta");
  std::filesystem::remove(path);
}
void Folder::MoveChild(const Handle& child_handle, const std::shared_ptr<Folder>& dest) {
  const auto search = children_.find(child_handle);
  if (search == children_.end()) {
    EVOENGINE_ERROR("Child not exist!")
    return;
  }
  auto child = search->second;
  const auto new_path = dest->GetAbsolutePath() / child->GetName();
  if (std::filesystem::exists(new_path)) {
    EVOENGINE_ERROR("Destination folder already exists!")
    return;
  }
  const auto old_path = child->GetAbsolutePath();
  child->DeleteMetadata();
  children_.erase(child_handle);
  if (std::filesystem::exists(old_path)) {
    std::filesystem::rename(old_path, new_path);
  }
  dest->children_.insert({child_handle, child});
  child->parent_ = dest;
  child->Save();
}
std::weak_ptr<Folder> Folder::GetChild(const Handle& child_handle) {
  const auto search = children_.find(child_handle);
  if (search == children_.end()) {
    return {};
  }
  return search->second;
}
std::weak_ptr<Folder> Folder::GetOrCreateChild(const std::string& folder_name) {
  for (const auto& i : children_) {
    if (i.second->name_ == folder_name)
      return i.second;
  }
  auto new_folder = std::make_shared<Folder>();
  new_folder->name_ = folder_name;
  new_folder->handle_ = Handle();
  new_folder->self_ = new_folder;
  children_[new_folder->handle_] = new_folder;
  ProjectManager::GetInstance().folder_registry_[new_folder->handle_] = new_folder;
  new_folder->parent_ = self_;
  if (const auto new_folder_path = new_folder->GetAbsolutePath(); !std::filesystem::exists(new_folder_path)) {
    std::filesystem::create_directories(new_folder_path);
  }
  new_folder->Save();
  return new_folder;
}
void Folder::DeleteChild(const Handle& child_handle) {
  const auto child = GetChild(child_handle).lock();
  const auto child_folder_path = child->GetAbsolutePath();
  std::filesystem::remove_all(child_folder_path);
  child->DeleteMetadata();
  children_.erase(child_handle);
}
std::shared_ptr<IAsset> Folder::GetOrCreateAsset(const std::string& file_name, const std::string& extension) {
  const auto type_name = Serialization::GetAssetTypeName(extension);
  if (type_name == "Binary") {
    EVOENGINE_ERROR("Asset type not registered!")
    return {};
  }
  for (const auto& i : asset_records_) {
    if (i.second->asset_file_name_ == file_name && i.second->asset_extension_ == extension)
      return i.second->GetAsset();
  }
  const auto record = std::make_shared<AssetRecord>();
  record->folder_ = self_;
  record->asset_type_name_ = type_name;
  record->asset_extension_ = extension;
  record->asset_file_name_ = file_name;
  record->asset_handle_ = Handle();
  record->self_ = record;
  asset_records_[record->asset_handle_] = record;
  auto asset = record->GetAsset();
  record->Save();
  return asset;
}
std::shared_ptr<IAsset> Folder::GetAsset(const Handle& asset_handle) {
  if (const auto search = asset_records_.find(asset_handle); search != asset_records_.end()) {
    return search->second->GetAsset();
  }
  return {};
}
void Folder::MoveAsset(const Handle& asset_handle, const std::shared_ptr<Folder>& dest) {
  const auto search = asset_records_.find(asset_handle);
  if (search == asset_records_.end()) {
    EVOENGINE_ERROR("AssetRecord not exist!")
    return;
  }
  auto asset_record = search->second;
  const auto new_path = dest->GetAbsolutePath() / (asset_record->asset_file_name_ + asset_record->asset_extension_);
  if (std::filesystem::exists(new_path)) {
    EVOENGINE_ERROR("Destination file already exists!")
    return;
  }
  const auto old_path = asset_record->GetAbsolutePath();
  asset_record->DeleteMetadata();
  asset_records_.erase(asset_handle);
  if (std::filesystem::exists(old_path)) {
    std::filesystem::rename(old_path, new_path);
  }
  dest->asset_records_.insert({asset_handle, asset_record});
  asset_record->folder_ = dest;
  asset_record->Save();
}
void Folder::DeleteAsset(const Handle& asset_handle) {
  auto& project_manager = ProjectManager::GetInstance();
  const auto asset_record = asset_records_[asset_handle];
  project_manager.asset_record_registry_.erase(asset_record->asset_handle_);
  project_manager.loaded_assets_.erase(asset_record->asset_handle_);
  const auto asset_path = asset_record->GetAbsolutePath();
  std::filesystem::remove(asset_path);
  asset_record->DeleteMetadata();
  asset_records_.erase(asset_handle);
}
void Folder::Refresh(const std::filesystem::path& parent_absolute_path) {
  auto& project_manager = ProjectManager::GetInstance();
  auto path = parent_absolute_path / name_;
  /**
   * 1. Scan folder for any unregistered folders and assets.
   */
  std::vector<std::filesystem::path> child_folder_metadata_list;
  std::vector<std::filesystem::path> child_folder_list;
  std::vector<std::filesystem::path> asset_metadata_list;
  std::vector<std::filesystem::path> file_list;
  for (const auto& entry : std::filesystem::directory_iterator(path)) {
    if (entry.path().filename() == "." || entry.path().filename() == "..") {
      continue;
    }
    if (std::filesystem::is_directory(entry.path())) {
      child_folder_list.push_back(entry.path());
    } else if (entry.path().extension() == ".evefoldermeta") {
      child_folder_metadata_list.push_back(entry.path());
    } else if (entry.path().extension() == ".evefilemeta") {
      asset_metadata_list.push_back(entry.path());
    } else if (entry.path().filename() != "" && entry.path().extension() != ".eveproj") {
      file_list.push_back(entry.path());
    }
  }
  for (const auto& child_folder_metadata_path : child_folder_metadata_list) {
    auto child_folder_path = child_folder_metadata_path;
    child_folder_path.replace_extension("");
    if (!std::filesystem::exists(child_folder_path) || child_folder_path.filename().string() == "." ||
        child_folder_path.filename().string() == "..") {
      std::filesystem::remove(child_folder_metadata_path);
    } else {
      auto folder_name = child_folder_metadata_path.filename();
      folder_name.replace_extension("");
      std::shared_ptr<Folder> child;
      for (const auto& i : children_) {
        if (i.second->name_ == folder_name) {
          child = i.second;
        }
      }
      if (!child) {
        auto new_folder = std::make_shared<Folder>();
        new_folder->self_ = new_folder;
        new_folder->name_ = folder_name.string();
        new_folder->parent_ = self_;
        new_folder->Load(child_folder_metadata_path);
        children_[new_folder->handle_] = new_folder;

        project_manager.folder_registry_[new_folder->handle_] = new_folder;
      }
    }
  }
  for (const auto& child_folder_path : child_folder_list) {
    auto child_folder = GetOrCreateChild(child_folder_path.filename().string()).lock();
    child_folder->Refresh(path);
  }
  for (const auto& asset_metadata_path : asset_metadata_list) {
    auto asset_name = asset_metadata_path.filename();
    asset_name.replace_extension("").replace_extension("");
    auto asset_extension = asset_metadata_path.filename().replace_extension("").extension();
    bool exist = false;
    for (const auto& i : asset_records_) {
      if (i.second->asset_file_name_ == asset_name && i.second->asset_extension_ == asset_extension) {
        exist = true;
      }
    }

    if (!exist) {
      auto new_asset_record = std::make_shared<AssetRecord>();
      new_asset_record->folder_ = self_.lock();
      new_asset_record->self_ = new_asset_record;
      new_asset_record->Load(asset_metadata_path);
      if (!std::filesystem::exists(new_asset_record->GetAbsolutePath())) {
        std::filesystem::remove(asset_metadata_path);
      } else {
        asset_records_[new_asset_record->asset_handle_] = new_asset_record;
        project_manager.asset_record_registry_[new_asset_record->asset_handle_] = new_asset_record;
      }
    }
  }
  for (const auto& file_path : file_list) {
    auto filename = file_path.filename().replace_extension("").replace_extension("").string();
    auto extension = file_path.extension().string();
    auto type_name = Serialization::GetAssetTypeName(extension);
    if (!HasAsset(filename, extension)) {
      auto new_asset_record = std::make_shared<AssetRecord>();
      new_asset_record->folder_ = self_.lock();
      new_asset_record->asset_type_name_ = type_name;
      new_asset_record->asset_extension_ = extension;
      new_asset_record->asset_file_name_ = filename;
      new_asset_record->asset_handle_ = Handle();
      new_asset_record->self_ = new_asset_record;
      asset_records_[new_asset_record->asset_handle_] = new_asset_record;
      project_manager.asset_record_registry_[new_asset_record->asset_handle_] = new_asset_record;
      new_asset_record->Save();
    }
  }
  /**
   * 2. Clear deleted asset and folder.
   */
  std::vector<Handle> asset_to_remove;
  for (const auto& i : asset_records_) {
    if (auto absolute_path = i.second->GetAbsolutePath(); !std::filesystem::exists(absolute_path)) {
      asset_to_remove.push_back(i.first);
    }
  }
  for (const auto& i : asset_to_remove) {
    DeleteAsset(i);
  }
  std::vector<Handle> folder_to_remove;
  for (const auto& i : children_) {
    if (!std::filesystem::exists(i.second->GetAbsolutePath())) {
      folder_to_remove.push_back(i.first);
    }
  }
  for (const auto& i : folder_to_remove) {
    DeleteChild(i);
  }
}
void Folder::RegisterAsset(const std::shared_ptr<IAsset>& asset, const std::string& file_name,
                           const std::string& extension) {
  auto& project_manager = ProjectManager::GetInstance();
  const auto record = std::make_shared<AssetRecord>();
  record->folder_ = self_;
  record->asset_type_name_ = asset->GetTypeName();
  record->asset_extension_ = extension;
  record->asset_file_name_ = file_name;
  record->asset_handle_ = asset->handle_;
  record->self_ = record;
  record->asset_ = asset;
  asset_records_[record->asset_handle_] = record;
  project_manager.asset_registry_[record->asset_handle_] = asset;
  project_manager.loaded_assets_[record->asset_handle_] = asset;
  project_manager.asset_record_registry_[record->asset_handle_] = record;
  asset->asset_record_ = record;
  asset->saved_ = false;
  record->Save();
}
bool Folder::HasAsset(const std::string& file_name, const std::string& extension) const {
  if (const auto type_name = Serialization::GetAssetTypeName(extension); type_name == "Binary") {
    return false;
  }
  for (const auto& i : asset_records_) {
    if (i.second->asset_file_name_ == file_name && i.second->asset_extension_ == extension)
      return true;
  }
  return false;
}
Folder::~Folder() {
  auto& project_manager = ProjectManager::GetInstance();
  project_manager.folder_registry_.erase(handle_);
}
bool Folder::IsSelfOrAncestor(const Handle& handle) const {
  std::shared_ptr<Folder> walker = self_.lock();
  while (true) {
    if (walker->GetHandle() == handle)
      return true;
    if (walker->parent_.expired())
      return false;
    walker = walker->parent_.lock();
  }
}

std::weak_ptr<Folder> ProjectManager::GetOrCreateFolder(const std::filesystem::path& project_relative_path) {
  const auto& project_manager = GetInstance();
  if (!project_relative_path.is_relative()) {
    EVOENGINE_ERROR("Path not relative!")
    return {};
  }
  auto dir_path = project_manager.project_folder_->GetAbsolutePath().parent_path() / project_relative_path;
  std::shared_ptr<Folder> ret_val = project_manager.project_folder_;
  for (auto it = project_relative_path.begin(); it != project_relative_path.end(); ++it) {
    ret_val = ret_val->GetOrCreateChild(it->filename().string()).lock();
  }
  return ret_val;
}
std::shared_ptr<IAsset> ProjectManager::GetOrCreateAsset(const std::filesystem::path& project_relative_path) {
  if (std::filesystem::is_directory(project_relative_path)) {
    EVOENGINE_ERROR("Path is directory!")
    return {};
  }
  const auto folder = GetOrCreateFolder(project_relative_path.parent_path()).lock();
  auto stem = project_relative_path.stem().string();
  const auto file_name = project_relative_path.filename().string();
  auto extension = project_relative_path.extension().string();
  if (file_name == stem) {
    stem = "";
    extension = file_name;
  }
  return folder->GetOrCreateAsset(stem, extension);
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
  project_manager.project_path_ = project_absolute_path;
  project_manager.asset_registry_.clear();
  project_manager.loaded_assets_.clear();
  project_manager.asset_record_registry_.clear();
  project_manager.folder_registry_.clear();
  Application::Reset();

  std::shared_ptr<Scene> scene;

  project_manager.current_focused_folder_ = project_manager.project_folder_ = std::make_shared<Folder>();
  project_manager.folder_registry_[0] = project_manager.project_folder_;
  project_manager.project_folder_->self_ = project_manager.project_folder_;
  if (!std::filesystem::exists(project_manager.project_folder_->GetAbsolutePath())) {
    std::filesystem::create_directories(project_manager.project_folder_->GetAbsolutePath());
  }
  ScanProject();

  bool found_scene = false;
  if (std::filesystem::exists(project_absolute_path)) {
    std::ifstream stream(project_absolute_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    YAML::Node in = YAML::Load(string_stream.str());
    uint64_t scene_handle = 0;
    if (in["m_startSceneHandle"])
      scene_handle = in["m_startSceneHandle"].as<uint64_t>();
    if (in["start_scene_handle"])
      scene_handle = in["start_scene_handle"].as<uint64_t>();
    if (auto temp = GetAsset(scene_handle)) {
      scene = std::dynamic_pointer_cast<Scene>(temp);
      SetStartScene(scene);
      Application::Attach(scene);
      found_scene = true;
    }
    EVOENGINE_LOG("Found and loaded project")
    if (found_scene && project_manager.scene_post_load_function_.has_value()) {
      project_manager.scene_post_load_function_.value()(scene);
      TransformGraph::CalculateTransformGraphs(scene);
    }
  }
  if (!found_scene) {
    scene = CreateTemporaryAsset<Scene>();
    if (std::filesystem::path new_scene_relative_path = GenerateNewProjectRelativePath("New Scene", ".evescene");
        scene->SetPathAndSave(new_scene_relative_path)) {
      EVOENGINE_LOG("Created new start scene!")
    }
    SetStartScene(scene);
    Application::Attach(scene);

    if (project_manager.new_scene_customizer_.has_value()) {
      project_manager.new_scene_customizer_.value()(scene);
      TransformGraph::CalculateTransformGraphs(scene);
    }
  }
}
void ProjectManager::SaveProject() {
  const auto& project_manager = GetInstance();
  if (const auto directory = project_manager.project_path_.parent_path(); !std::filesystem::exists(directory)) {
    std::filesystem::create_directories(directory);
  }
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "start_scene_handle" << YAML::Value << project_manager.start_scene_->GetHandle();
  out << YAML::EndMap;
  std::ofstream file_out(project_manager.project_path_.string());
  file_out << out.c_str();
  file_out.flush();
}
std::filesystem::path ProjectManager::GetProjectPath() {
  auto& project_manager = GetInstance();
  return project_manager.project_path_;
}
std::string ProjectManager::GetProjectName() {
  const auto& project_manager = GetInstance();
  return project_manager.project_path_.stem().string();
}
std::weak_ptr<Folder> ProjectManager::GetCurrentFocusedFolder() {
  auto& project_manager = GetInstance();
  return project_manager.current_focused_folder_;
}

std::shared_ptr<IAsset> ProjectManager::GetAsset(const Handle& handle) {
  auto& project_manager = GetInstance();
  if (const auto search = project_manager.asset_registry_.find(handle); search != project_manager.asset_registry_.end())
    return search->second.lock();
  if (auto search2 = project_manager.asset_record_registry_.find(handle);
      search2 != project_manager.asset_record_registry_.end())
    return search2->second.lock()->GetAsset();

  if (Resources::IsResource(handle)) {
    return Resources::GetResource<IAsset>(handle);
  }
  return {};
}

std::shared_ptr<IAsset> ProjectManager::CreateTemporaryAsset(const std::string& type_name) {
  size_t hash_code;
  auto ret_val = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(type_name, hash_code, Handle()));
  auto& project_manager = GetInstance();
  project_manager.asset_registry_[ret_val->GetHandle()] = ret_val;
  ret_val->self_ = ret_val;
  ret_val->OnCreate();
  return ret_val;
}
std::shared_ptr<IAsset> ProjectManager::CreateTemporaryAsset(const std::string& type_name, const Handle& handle) {
  size_t hash_code;
  auto ret_val = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(type_name, hash_code, handle));
  if (!ret_val) {
    return nullptr;
  }
  auto& project_manager = GetInstance();
  project_manager.asset_registry_[ret_val->GetHandle()] = ret_val;
  ret_val->self_ = ret_val;
  ret_val->OnCreate();
  return ret_val;
}
bool ProjectManager::IsInProjectFolder(const std::filesystem::path& absolute_path) {
  if (!absolute_path.is_absolute()) {
    EVOENGINE_ERROR("Not absolute path!")
    return false;
  }
  const auto& project_manager = GetInstance();
  auto project_folder_path = project_manager.project_path_.parent_path();
  return std::search(absolute_path.begin(), absolute_path.end(), project_folder_path.begin(),
                     project_folder_path.end()) != absolute_path.end();
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
std::filesystem::path ProjectManager::GenerateNewProjectRelativePath(const std::string& relative_stem,
                                                                     const std::string& postfix) {
  assert(std::filesystem::path(relative_stem + postfix).is_relative());
  const auto& project_manager = GetInstance();
  const auto project_path = project_manager.project_path_.parent_path();
  std::filesystem::path test_path = project_path / (relative_stem + postfix);
  int i = 0;
  while (std::filesystem::exists(test_path)) {
    i++;
    test_path = project_path / (relative_stem + " (" + std::to_string(i) + ")" + postfix);
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

void ProjectManager::ScanProject() {
  const auto& project_manager = GetInstance();
  if (!project_manager.project_folder_)
    return;
  const auto directory = project_manager.project_path_.parent_path().parent_path();
  project_manager.project_folder_->handle_ = 0;
  project_manager.project_folder_->name_ = project_manager.project_path_.parent_path().stem().string();
  project_manager.project_folder_->Refresh(directory);
}

void ProjectManager::OnDestroy() {
  auto& project_manager = GetInstance();

  project_manager.project_folder_.reset();
  project_manager.new_scene_customizer_.reset();

  project_manager.current_focused_folder_.reset();
  project_manager.loaded_assets_.clear();
  project_manager.asset_registry_.clear();
  project_manager.asset_record_registry_.clear();
  project_manager.folder_registry_.clear();
  project_manager.start_scene_.reset();

  project_manager.asset_thumbnails_.clear();
  project_manager.asset_thumbnail_storage_.clear();

  project_manager.inspecting_asset.reset();
}

void ProjectManager::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& project_manager = GetInstance();
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("Project")) {
      ImGui::Text(("Current Project path: " + project_manager.project_path_.string()).c_str());

      FileUtils::SaveFile(
          "Create or load New Project##ProjectManager", "Project", {".eveproj"},
          [](const std::filesystem::path& file_path) {
            try {
              GetOrCreateProject(file_path);
            } catch (const std::exception& e) {
              EVOENGINE_ERROR(std::string(e.what()) + ": Failed to create/load from " + file_path.string())
            }
          },
          false);

      if (ImGui::Button("Save")) {
        SaveProject();
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
      ImGui::Checkbox("Project", &project_manager.show_project_window);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  if (project_manager.show_project_window) {
    if (ImGui::Begin("Project")) {
      if (project_manager.project_folder_) {
        auto current_focused_folder = project_manager.current_focused_folder_.lock();
        auto current_folder_path = current_focused_folder->GetProjectRelativePath();
        if (ImGui::BeginDragDropTarget()) {
          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            Handle handle = *static_cast<Handle*>(payload->Data);

            if (auto asset_search = project_manager.asset_registry_.find(handle);
                asset_search != project_manager.asset_registry_.end() && !asset_search->second.expired()) {
              auto asset = asset_search->second.lock();
              if (asset->IsTemporary()) {
                auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
                auto file_name = "New " + asset->GetTypeName();
                auto file_path = GenerateNewProjectRelativePath(
                    (current_focused_folder->GetProjectRelativePath() / file_name).string(), file_extension);
                asset->SetPathAndSave(file_path);
              } else {
                if (auto asset_record = asset->asset_record_.lock();
                    asset_record->GetFolder().lock().get() != current_focused_folder.get()) {
                  auto file_extension = asset_record->GetAssetExtension();
                  auto file_name = asset_record->GetAssetFileName();
                  auto file_path = GenerateNewProjectRelativePath(
                      (current_focused_folder->GetProjectRelativePath() / file_name).string(), file_extension);
                  asset->SetPathAndSave(file_path);
                }
              }
            } else {
              if (auto asset_record_search = project_manager.asset_record_registry_.find(handle);
                  asset_record_search != project_manager.asset_record_registry_.end() &&
                  !asset_record_search->second.expired()) {
                auto asset_record = asset_record_search->second.lock();
                auto folder = asset_record->GetFolder().lock();
                if (folder.get() != current_focused_folder.get()) {
                  folder->MoveAsset(asset_record->GetAssetHandle(), current_focused_folder);
                }
              }
            }
          }

          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            auto prefab = std::dynamic_pointer_cast<Prefab>(CreateTemporaryAsset<Prefab>());
            auto entity_handle = *static_cast<Handle*>(payload->Data);
            auto scene = Application::GetActiveScene();
            if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
              prefab->FromEntity(entity);
              // If current folder doesn't contain file with same name
              auto file_name = scene->GetEntityName(entity);
              auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
              auto file_path =
                  GenerateNewProjectRelativePath((current_folder_path / file_name).string(), file_extension);
              prefab->SetPathAndSave(file_path);
            }
          }

          ImGui::EndDragDropTarget();
        }
        static glm::vec2 thumbnail_size_padding = {96.0f, 8.0f};
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
        FolderHierarchyHelper(project_manager.project_folder_);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("2", ImVec2(size2 - 5.0f, h), true);
        if (ImGui::ImageButton(editor_layer->AssetIcons()["RefreshButton"]->GetImTextureId(), {16, 16}, {0, 1},
                               {1, 0})) {
          project_manager.ScanProject();
        }

        if (current_focused_folder != project_manager.project_folder_) {
          ImGui::SameLine();
          if (ImGui::ImageButton(editor_layer->AssetIcons()["BackButton"]->GetImTextureId(), {16, 16}, {0, 1},
                                 {1, 0})) {
            project_manager.current_focused_folder_ = current_focused_folder->parent_;
          }
        }
        ImGui::SameLine();
        ImGui::Text(current_focused_folder->GetProjectRelativePath().string().c_str());
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
                const auto prefab = CreateTemporaryAsset<Prefab>();
                if (prefab->Import(path)) {
                  prefab->SetPathAndSave(current_focused_folder->GetProjectRelativePath() /
                                         path.filename().replace_extension(".eveprefab"));
                }
              },
              false);

          if (ImGui::Button("New folder...")) {
            auto new_path = GenerateNewProjectRelativePath(
                (current_focused_folder->GetProjectRelativePath() / "New Folder").string(), "");
            GetOrCreateFolder(new_path);
          }
          if (ImGui::BeginMenu("New asset...")) {
            for (auto& i : Serialization::GetInstance().asset_extensions_) {
              if (ImGui::Button(i.first.c_str())) {
                std::string new_file_name = "New " + i.first;
                std::filesystem::path new_path = GenerateNewProjectRelativePath(
                    (current_focused_folder->GetProjectRelativePath() / new_file_name).string(), i.second.front());
                current_focused_folder->GetOrCreateAsset(new_path.stem().string(), new_path.extension().string());
              }
            }
            ImGui::EndMenu();
          }
          ImGui::EndPopup();
        }

        float panel_width = ImGui::GetWindowContentRegionMax().x;
        int column_count = glm::max(1, static_cast<int>(panel_width / cell_size));
        ImGui::Columns(column_count, nullptr, false);
        if (!updated) {
          for (auto& i : current_focused_folder->children_) {
            ImGui::Image(editor_layer->AssetIcons()["Folder"]->GetImTextureId(),
                         {thumbnail_size_padding.x, thumbnail_size_padding.x}, {0, 1}, {1, 0});
            const std::string tag = "##Folder" + std::to_string(i.second->handle_);
            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
              ImGui::SetDragDropPayload("Folder", &i.second->handle_, sizeof(Handle));
              ImGui::TextColored(ImVec4(0, 0, 1, 1), ("Folder" + tag).c_str());
              ImGui::EndDragDropSource();
            }
            if (i.second->GetHandle() != 0) {
              if (ImGui::BeginPopupContextItem(tag.c_str())) {
                if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
                  static char new_name[256] = {0};
                  ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
                  if (ImGui::Button(("Confirm" + tag).c_str())) {
                    i.second->Rename(std::string(new_name));
                    memset(new_name, 0, 256);
                    ImGui::CloseCurrentPopup();
                  }
                  ImGui::EndMenu();
                }
                if (ImGui::Button(("Remove" + tag).c_str())) {
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
                  if (auto temp = project_manager.folder_registry_[payload_n]; !temp.expired()) {
                    if (auto actual_folder = temp.lock(); !i.second->IsSelfOrAncestor(actual_folder->handle_) &&
                                                          actual_folder->parent_.lock().get() != i.second.get()) {
                      actual_folder->parent_.lock()->MoveChild(actual_folder->GetHandle(), i.second);
                    }
                  }
                }
              }
              if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
                IM_ASSERT(payload->DataSize == sizeof(Handle));
                Handle payload_n = *static_cast<Handle*>(payload->Data);
                if (auto asset_search = project_manager.asset_registry_.find(payload_n);
                    asset_search != project_manager.asset_registry_.end() && !asset_search->second.expired()) {
                  if (auto asset = asset_search->second.lock(); asset->IsTemporary()) {
                    auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
                    auto file_name = "New " + asset->GetTypeName();
                    auto file_path = GenerateNewProjectRelativePath(
                        (i.second->GetProjectRelativePath() / file_name).string(), file_extension);
                    asset->SetPathAndSave(file_path);
                  } else {
                    if (auto asset_record = asset->asset_record_.lock();
                        asset_record->GetFolder().lock().get() != i.second.get()) {
                      auto file_extension = asset_record->GetAssetExtension();
                      auto file_name = asset_record->GetAssetFileName();
                      auto file_path = GenerateNewProjectRelativePath(
                          (i.second->GetProjectRelativePath() / file_name).string(), file_extension);
                      asset->SetPathAndSave(file_path);
                    }
                  }
                } else {
                  if (auto asset_record_search = project_manager.asset_record_registry_.find(payload_n);
                      asset_record_search != project_manager.asset_record_registry_.end() &&
                      !asset_record_search->second.expired()) {
                    auto asset_record = asset_record_search->second.lock();
                    auto folder = asset_record->GetFolder().lock();
                    if (folder.get() != i.second.get()) {
                      folder->MoveAsset(asset_record->GetAssetHandle(), i.second);
                    }
                  }
                }
              }

              if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Binary")) {
                IM_ASSERT(payload->DataSize == sizeof(Handle));
                Handle payload_n = *static_cast<Handle*>(payload->Data);
                if (auto record = project_manager.asset_record_registry_[payload_n]; !record.expired())
                  record.lock()->GetFolder().lock()->MoveAsset(payload_n, i.second);
              }

              if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
                IM_ASSERT(payload->DataSize == sizeof(Handle));
                auto prefab = std::dynamic_pointer_cast<Prefab>(CreateTemporaryAsset<Prefab>());
                auto entity_handle = *static_cast<Handle*>(payload->Data);
                auto scene = Application::GetActiveScene();
                if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
                  // If current folder doesn't contain file with same name
                  auto file_name = scene->GetEntityName(entity);
                  auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
                  auto file_path = GenerateNewProjectRelativePath(
                      (i.second->GetProjectRelativePath() / file_name).string(), file_extension);
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
            ImGui::TextWrapped(i.second->name_.c_str());
            if (item_hovered)
              ImGui::PopStyleColor(1);
            ImGui::NextColumn();
          }
        }
        if (!updated) {
          for (auto& i : current_focused_folder->asset_records_) {
            ImTextureID texture_id = nullptr;
            auto file_name = i.second->GetProjectRelativePath().filename();
            if (file_name.string() == ".eveproj" || file_name.extension().string() == ".eveproj")
              continue;
            if (file_name.extension().string() == ".eveproj") {
              texture_id = editor_layer->AssetIcons()["Project"]->GetImTextureId();
            } else {
              if (auto icon_search = editor_layer->AssetIcons().find(i.second->GetAssetTypeName());
                  icon_search != editor_layer->AssetIcons().end()) {
                texture_id = icon_search->second->GetImTextureId();
              } else {
                texture_id = editor_layer->AssetIcons()["Binary"]->GetImTextureId();
              }
            }
            static Handle focused_asset_handle;
            bool item_focused = false;
            if (focused_asset_handle == i.first.GetValue()) {
              item_focused = true;
            }
            ImGui::Image(texture_id, {thumbnail_size_padding.x, thumbnail_size_padding.x}, {0, 1}, {1, 0});

            bool item_hovered = false;
            if (ImGui::IsItemHovered()) {
              item_hovered = true;
              if (ImGui::IsMouseDoubleClicked(0) && i.second->GetAssetTypeName() != "Binary") {
                // If it's an asset then inspect.
                if (auto asset = i.second->GetAsset())
                  project_manager.inspecting_asset = asset;
              }
            }
            const std::string tag = "##" + i.second->GetAssetTypeName() + std::to_string(i.first.GetValue());
            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
              ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
              ImGui::TextColored(ImVec4(0, 0, 1, 1), i.second->GetAssetFileName().c_str());
              ImGui::EndDragDropSource();
            }

            if (ImGui::BeginPopupContextItem(tag.c_str())) {
              if (ImGui::Button("Duplicate")) {
                auto ptr = i.second->GetAsset();
                auto new_asset = DuplicateAsset(ptr);
              }
              if (i.second->GetAssetTypeName() != "Binary" && ImGui::BeginMenu(("Rename" + tag).c_str())) {
                static char new_name[256] = {};
                ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
                if (ImGui::Button(("Confirm" + tag).c_str())) {
                  auto ptr = i.second->GetAsset();
                  ptr->SetPathAndSave(ptr->GetProjectRelativePath().replace_filename(
                      std::string(new_name) + ptr->GetAssetRecord().lock()->GetAssetExtension()));
                  memset(new_name, 0, 256);
                }
                ImGui::EndMenu();
              }
              if (ImGui::Button(("Delete" + tag).c_str())) {
                current_focused_folder->DeleteAsset(i.first);
                ImGui::EndPopup();
                break;
              }
              ImGui::EndPopup();
            }

            if (item_focused)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 0, 0, 1});
            else if (item_hovered)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 1, 0, 1});
            ImGui::TextWrapped(file_name.string().c_str());
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
}

void ProjectManager::FolderHierarchyHelper(const std::shared_ptr<Folder>& folder) {
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
        auto temp = project_manager.folder_registry_[payload_n];
        if (!temp.expired()) {
          if (auto actual_folder = temp.lock(); !folder->IsSelfOrAncestor(actual_folder->handle_) &&
                                                actual_folder->parent_.lock().get() != folder.get()) {
            actual_folder->parent_.lock()->MoveChild(actual_folder->GetHandle(), folder);
          }
        }
      }
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (auto asset_search = project_manager.asset_registry_.find(payload_n);
          asset_search != project_manager.asset_registry_.end() && !asset_search->second.expired()) {
        if (auto asset = asset_search->second.lock(); asset->IsTemporary()) {
          auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
          auto file_name = "New " + asset->GetTypeName();
          auto file_path =
              GenerateNewProjectRelativePath((folder->GetProjectRelativePath() / file_name).string(), file_extension);
          asset->SetPathAndSave(file_path);
        } else {
          if (auto asset_record = asset->asset_record_.lock(); asset_record->GetFolder().lock().get() != folder.get()) {
            auto file_extension = asset_record->GetAssetExtension();
            auto file_name = asset_record->GetAssetFileName();
            auto file_path =
                GenerateNewProjectRelativePath((folder->GetProjectRelativePath() / file_name).string(), file_extension);
            asset->SetPathAndSave(file_path);
          }
        }
      } else {
        if (auto asset_record_search = project_manager.asset_record_registry_.find(payload_n);
            asset_record_search != project_manager.asset_record_registry_.end() &&
            !asset_record_search->second.expired()) {
          auto asset_record = asset_record_search->second.lock();
          auto previous_folder = asset_record->GetFolder().lock();
          if (folder && previous_folder.get() != folder.get()) {
            previous_folder->MoveAsset(asset_record->GetAssetHandle(), folder);
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
      FolderHierarchyHelper(i.second);
    }
    for (const auto& i : folder->asset_records_) {
      if (ImGui::TreeNodeEx((i.second->GetAssetFileName() + i.second->GetAssetExtension()).c_str(),
                            ImGuiTreeNodeFlags_Bullet)) {
        ImGui::TreePop();
      }
      if (ImGui::IsItemHovered()) {
        if (ImGui::IsMouseDoubleClicked(0) && i.second->GetAssetTypeName() != "Binary") {
          // If it's an asset then inspect.
          if (auto asset = i.second->GetAsset())
            project_manager.inspecting_asset = asset;
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

std::shared_ptr<IAsset> ProjectManager::DuplicateAsset(const std::shared_ptr<IAsset>& target) {
  const auto& project_manager = GetInstance();
  const auto folder = target->GetAssetRecord().lock()->GetFolder().lock();
  const auto path = target->GetProjectRelativePath();
  const auto prefix = (folder->GetProjectRelativePath() / path.stem()).string();
  const auto postfix = path.extension().string();
  const auto new_path = project_manager.GenerateNewProjectRelativePath(prefix, postfix);
  try {
    std::filesystem::copy(target->GetAbsolutePath(), project_manager.GetProjectPath().parent_path() / new_path,
                          std::filesystem::copy_options::overwrite_existing);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
  }
  auto new_asset = folder->GetOrCreateAsset(new_path.stem().string(), new_path.extension().string());
  return new_asset;
}

std::weak_ptr<Scene> ProjectManager::GetStartScene() {
  auto& project_manager = GetInstance();
  return project_manager.start_scene_;
}
void ProjectManager::SetStartScene(const std::shared_ptr<Scene>& scene) {
  auto& project_manager = GetInstance();
  project_manager.start_scene_ = scene;
  SaveProject();
}
std::weak_ptr<Folder> ProjectManager::GetFolder(const Handle& handle) {
  auto& project_manager = GetInstance();
  if (const auto search = project_manager.folder_registry_.find(handle);
      search != project_manager.folder_registry_.end()) {
    return search->second;
  }
  return {};
}
std::filesystem::path ProjectManager::GetPathRelativeToProject(const std::filesystem::path& absolute_path) {
  const auto& project_manager = GetInstance();
  if (!project_manager.project_folder_)
    return {};
  if (!absolute_path.is_absolute())
    return {};
  if (!IsInProjectFolder(absolute_path))
    return {};
  return std::filesystem::relative(absolute_path, project_manager.GetProjectPath().parent_path());
}
