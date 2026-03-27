#include "FileManager.hpp"

#include <stb_image.h>
#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
using namespace evo_engine;

namespace {
std::shared_ptr<Texture2D> GenerateTextureThumbnailFromDisk(const std::filesystem::path& absolute_path) {
  if (!std::filesystem::exists(absolute_path))
    return {};

  stbi_set_flip_vertically_on_load(true);
  int width = 0;
  int height = 0;
  int components = 0;

  const bool is_hdr = absolute_path.extension() == ".hdr" || absolute_path.extension() == ".exr";
  std::vector<glm::vec4> src;

  if (is_hdr) {
    float* data = stbi_loadf(absolute_path.string().c_str(), &width, &height, &components, STBI_rgb_alpha);
    if (!data || width <= 0 || height <= 0) {
      if (data)
        stbi_image_free(data);
      return {};
    }
    src.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
    memcpy(src.data(), data, sizeof(glm::vec4) * src.size());
    stbi_image_free(data);
  } else {
    unsigned char* data = stbi_load(absolute_path.string().c_str(), &width, &height, &components, STBI_rgb_alpha);
    if (!data || width <= 0 || height <= 0) {
      if (data)
        stbi_image_free(data);
      return {};
    }
    src.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
    for (size_t i = 0; i < src.size(); i++) {
      src[i] = glm::vec4(static_cast<float>(data[i * 4]) / 255.0f, static_cast<float>(data[i * 4 + 1]) / 255.0f,
                         static_cast<float>(data[i * 4 + 2]) / 255.0f, static_cast<float>(data[i * 4 + 3]) / 255.0f);
    }
    stbi_image_free(data);
  }

  const glm::uvec2 src_resolution{static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
  if (src_resolution.x == 0 || src_resolution.y == 0) {
    return {};
  }
  const float max_dim = static_cast<float>(glm::max(src_resolution.x, src_resolution.y));
  const float scale = glm::min(1.0f, 128.0f / max_dim);
  glm::uvec2 dst_resolution{glm::max(1u, static_cast<uint32_t>(src_resolution.x * scale)),
                            glm::max(1u, static_cast<uint32_t>(src_resolution.y * scale))};
  std::vector<glm::vec4> dst;
  if (dst_resolution != src_resolution) {
    Texture2D::Resize(src, src_resolution, dst, dst_resolution);
  } else {
    dst = std::move(src);
  }

  const auto thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
  if (!thumbnail)
    return {};
  thumbnail->SetRgbaChannelData(dst, dst_resolution, false);
  return thumbnail;
}
}  // namespace

std::string File::GetAssetTypeName() const {
  return asset_type_name_;
}
std::string File::GetAssetFileName() const {
  return asset_file_name_;
}
std::string File::GetAssetExtension() const {
  return asset_extension_;
}
std::filesystem::path File::GetAssetsFolderRelativePath() const {
  if (folder_.expired()) {
    EVOENGINE_ERROR("Folder expired!")
    return {};
  }
  return folder_.lock()->GetAssetsRelativePath() / (asset_file_name_ + asset_extension_);
}
std::filesystem::path File::GetAbsolutePath() const {
  if (folder_.expired()) {
    EVOENGINE_ERROR("Folder expired!")
    return {};
  }
  return folder_.lock()->GetAbsolutePath() / (asset_file_name_ + asset_extension_);
}
void File::SetAssetFileName(const std::string& new_name) {
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
void File::SetAssetExtension(const std::string& new_extension) {
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
void File::Save() const {
  const auto path = GetAbsolutePath().string() + ".evefilemeta";
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

Handle File::GetAssetHandle() const {
  return asset_handle_;
}
void File::DeleteMetadata() const {
  const auto path = GetAbsolutePath().string() + ".evefilemeta";
  std::filesystem::remove(path);
}
void File::Load(const std::filesystem::path& path) {
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

std::shared_ptr<Texture2D> File::GetThumbnail() {
  // Rate-limit thumbnail generation to avoid loading too many assets in one frame.
  // Loading many assets (especially textures) in a single frame causes excessive
  // GPU work during DeviceSync, which can trigger device-lost errors.
  static uint32_t last_thumbnail_frame = 0;
  static int thumbnails_this_frame = 0;
  constexpr int max_thumbnails_per_frame = 1;
  const uint32_t current_frame = Platform::Initialized() ? Platform::GetFrameCount() : 0;
  if (current_frame != last_thumbnail_frame) {
    last_thumbnail_frame = current_frame;
    thumbnails_this_frame = 0;
  }
  if (!thumbnail_ && asset_type_name_ != "Texture2D") {
    // Keep folder browsing lightweight and safe: avoid loading full assets
    // (e.g. materials recursively loading large textures) just for thumbnails.
    if (const auto icon = EditorLayer::FindIcon(asset_type_name_)) {
      thumbnail_ = icon;
    }
  }
  if (!thumbnail_ && asset_type_name_ == "Texture2D") {
    // Use a small on-disk decode for folder thumbnails to avoid loading full assets.
    thumbnail_ = GenerateTextureThumbnailFromDisk(GetAbsolutePath());
  }
  if (!thumbnail_ && asset_type_name_ == "Texture2D") {
    if (thumbnails_this_frame < max_thumbnails_per_frame) {
      try {
        if (const auto asset = AssetManager::GetAssetImpl(asset_handle_)) {
          thumbnail_ = asset->GenerateThumbnailTexture();
          thumbnails_this_frame++;
        }
      } catch (const std::exception&) {
        thumbnail_.reset();
      }
    }
  }
  if (!thumbnail_) {
    if (const auto icon = EditorLayer::FindIcon(asset_type_name_)) {
      thumbnail_ = icon;
    } else {
      thumbnail_ = EditorLayer::FindIcon("Binary");
    }
  }
  return thumbnail_;
}

std::weak_ptr<Folder> File::GetFolder() const {
  return folder_;
}
std::filesystem::path Folder::GetAssetsRelativePath() const {
  if (parent_.expired()) {
    return "";
  }
  return parent_.lock()->GetAssetsRelativePath() / name_;
}
std::filesystem::path Folder::GetAbsolutePath() const {
  const auto& project_manager = ProjectManager::GetInstance();
  const auto asset_folder_path = project_manager.assets_folder_path;
  const auto relative_path = GetAssetsRelativePath();
  return asset_folder_path / relative_path;
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
  auto& file_manager = FileManager::GetInstance();
  auto new_folder = std::make_shared<Folder>();
  new_folder->name_ = folder_name;
  new_folder->handle_ = Handle();
  new_folder->self_ = new_folder;
  children_[new_folder->handle_] = new_folder;
  file_manager.folder_registry_[new_folder->handle_] = new_folder;
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
    EVOENGINE_ERROR(std::string("Asset type not registered! Ext: ") + extension)
    return {};
  }
  for (const auto& i : files) {
    if (i.second->asset_file_name_ == file_name && i.second->asset_extension_ == extension)
      return AssetManager::GetAssetImpl(i.second->asset_handle_);
  }
  const auto record = std::make_shared<File>();
  record->folder_ = self_;
  record->asset_type_name_ = type_name;
  record->asset_extension_ = extension;
  record->asset_file_name_ = file_name;
  record->asset_handle_ = Handle();
  record->self_ = record;
  files[record->asset_handle_] = record;
  auto& file_manager = FileManager::GetInstance();
  file_manager.file_registry_[record->asset_handle_] = record;
  record->Save();

  auto asset = AssetManager::GetAssetImpl(record->asset_handle_);
  return asset;
}
std::shared_ptr<IAsset> Folder::GetAsset(const Handle& asset_handle) {
  if (const auto search = files.find(asset_handle); search != files.end()) {
    return AssetManager::GetAssetImpl(search->second->asset_handle_);
  }
  return {};
}

std::optional<std::shared_ptr<IAsset>> Folder::Duplicate(const Handle& handle) {
  const auto file_record = files[handle];
  const auto folder = file_record->GetFolder().lock();
  const auto path = file_record->GetAssetsFolderRelativePath();
  const auto prefix = (folder->GetAssetsRelativePath() / path.stem()).string();
  const auto postfix = path.extension().string();
  const auto new_path = ProjectManager::GenerateNewAssetsRelativePath(prefix, postfix);
  try {
    std::filesystem::copy(file_record->GetAbsolutePath(), ProjectManager::GetAssetsFolderPath() / new_path,
                          std::filesystem::copy_options::overwrite_existing);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
  }
  if (file_record->asset_type_name_ != "Binary") {
    return folder->GetOrCreateAsset(new_path.stem().string(), new_path.extension().string());
  }
  const auto record = std::make_shared<File>();
  record->folder_ = self_;
  record->asset_type_name_ = file_record->GetAssetTypeName();
  record->asset_extension_ = new_path.extension().string();
  record->asset_file_name_ = new_path.stem().string();
  record->asset_handle_ = Handle();
  record->self_ = record;
  files[record->asset_handle_] = record;
  record->Save();
  return std::nullopt;
}

std::shared_ptr<File> Folder::MoveAsset(const Handle& asset_handle, const std::shared_ptr<Folder>& dest) {
  const auto search = files.find(asset_handle);
  if (search == files.end()) {
    throw std::invalid_argument("File not exist!");
  }
  auto asset_record = search->second;
  const auto new_path = dest->GetAbsolutePath() / (asset_record->asset_file_name_ + asset_record->asset_extension_);
  if (std::filesystem::exists(new_path)) {
    throw std::invalid_argument("Destination file already exists!");
  }
  const auto old_path = asset_record->GetAbsolutePath();
  asset_record->DeleteMetadata();
  files.erase(asset_handle);
  if (std::filesystem::exists(old_path)) {
    std::filesystem::rename(old_path, new_path);
  }
  dest->files.insert({asset_handle, asset_record});
  asset_record->folder_ = dest;
  asset_record->Save();
  return asset_record;
}
void Folder::RemoveFile(const Handle& asset_handle) {
  auto& file_manager = FileManager::GetInstance();
  const auto asset_record = files[asset_handle];
  file_manager.file_registry_.erase(asset_record->asset_handle_);
  const auto asset_path = asset_record->GetAbsolutePath();
  std::filesystem::remove(asset_path);
  asset_record->DeleteMetadata();
  files.erase(asset_handle);
}
void Folder::Refresh(std::vector<Handle>& assets_pending_loading) {
  auto& file_manager = FileManager::GetInstance();
  auto path = GetAbsolutePath();
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
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
      const DWORD attributes = GetFileAttributes(entry.path().string().c_str());
      SetFileAttributes(entry.path().string().c_str(), attributes | FILE_ATTRIBUTE_HIDDEN);
#endif
    } else if (entry.path().extension() == ".evefilemeta") {
      asset_metadata_list.push_back(entry.path());
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
      const DWORD attributes = GetFileAttributes(entry.path().string().c_str());
      SetFileAttributes(entry.path().string().c_str(), attributes | FILE_ATTRIBUTE_HIDDEN);
#endif
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

        file_manager.folder_registry_[new_folder->handle_] = new_folder;
      }
    }
  }
  for (const auto& child_folder_path : child_folder_list) {
    auto child_folder = GetOrCreateChild(child_folder_path.filename().string()).lock();
    child_folder->Refresh(assets_pending_loading);
  }
  for (const auto& asset_metadata_path : asset_metadata_list) {
    auto asset_name = asset_metadata_path.filename();
    asset_name.replace_extension("").replace_extension("");
    auto asset_extension = asset_metadata_path.filename().replace_extension("").extension();
    bool exist = false;
    for (const auto& i : files) {
      if (i.second->asset_file_name_ == asset_name && i.second->asset_extension_ == asset_extension) {
        exist = true;
      }
    }

    if (!exist) {
      auto new_asset_record = std::make_shared<File>();
      new_asset_record->folder_ = self_;
      new_asset_record->self_ = new_asset_record;
      new_asset_record->Load(asset_metadata_path);
      if (!std::filesystem::exists(new_asset_record->GetAbsolutePath())) {
        std::filesystem::remove(asset_metadata_path);
      } else {
        files[new_asset_record->asset_handle_] = new_asset_record;
        file_manager.file_registry_[new_asset_record->asset_handle_] = new_asset_record;
      }
    }
  }
  for (const auto& file_path : file_list) {
    auto filename = file_path.filename().replace_extension("").replace_extension("").string();
    auto extension = file_path.extension().string();
    auto type_name = Serialization::GetAssetTypeName(extension);
    if (!FileRecorded(filename, extension)) {
      auto new_asset_record = std::make_shared<File>();
      new_asset_record->folder_ = self_;
      new_asset_record->asset_type_name_ = type_name;
      new_asset_record->asset_extension_ = extension;
      new_asset_record->asset_file_name_ = filename;
      new_asset_record->asset_handle_ = Handle();
      new_asset_record->self_ = new_asset_record;
      files[new_asset_record->asset_handle_] = new_asset_record;
      file_manager.file_registry_[new_asset_record->asset_handle_] = new_asset_record;
      new_asset_record->Save();
    }
  }
  /**
   * 2. Clear deleted asset and folder.
   */
  std::vector<Handle> asset_to_remove;
  for (const auto& i : files) {
    if (auto absolute_path = i.second->GetAbsolutePath(); !std::filesystem::exists(absolute_path)) {
      asset_to_remove.push_back(i.first);
    }
  }
  for (const auto& i : asset_to_remove) {
    RemoveFile(i);
  }
  for (const auto& i : files) {
    if (i.second->asset_type_name_ != "Binary" && !i.second->asset_) {
      assets_pending_loading.emplace_back(i.second->asset_handle_);
    }
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
std::weak_ptr<File> Folder::RegisterAsset(const Handle& asset_handle, const std::string& type_name,
                                          const std::string& file_name, const std::string& extension) {
  if (files.find(asset_handle) != files.end()) {
    throw std::invalid_argument("File already exist!");
  }
  auto& file_manager = FileManager::GetInstance();
  const auto file = std::make_shared<File>();
  file->folder_ = self_;
  file->asset_type_name_ = type_name;
  file->asset_extension_ = extension;
  file->asset_file_name_ = file_name;
  file->asset_handle_ = asset_handle;
  file->self_ = file;

  files[file->asset_handle_] = file;
  file_manager.file_registry_[asset_handle] = file;
  file->Save();
  return file;
}
bool Folder::FileRecorded(const std::string& file_name, const std::string& extension) const {
  const auto type_name = Serialization::GetAssetTypeName(extension);
  for (const auto& file : files) {
    if (file.second->asset_file_name_ == file_name && file.second->asset_extension_ == extension)
      return true;
  }
  return false;
}
Folder::~Folder() {
  auto& file_manager = FileManager::GetInstance();
  file_manager.folder_registry_.erase(handle_);
}

void FileManager::Initialize() {
  Clear();
}

void FileManager::OnDestroy() {
  Clear();
}

void FileManager::Clear() {
  auto& file_manager = GetInstance();
  file_manager.file_registry_.clear();
  file_manager.folder_registry_.clear();
}

std::shared_ptr<File> FileManager::GetFile(const Handle& handle) {
  auto& file_manager = GetInstance();
  std::shared_ptr<File> ret_val{};
  file_manager.file_registry_mutex.lock();
  const auto search = file_manager.file_registry_.find(handle);
  if (search != file_manager.file_registry_.end() && !search->second.expired()) {
    ret_val = search->second.lock();
  }
  file_manager.file_registry_mutex.unlock();
  return ret_val;
}

std::shared_ptr<Folder> FileManager::GetFolder(const Handle& handle) {
  auto& file_manager = GetInstance();
  std::shared_ptr<Folder> ret_val{};
  file_manager.file_registry_mutex.lock();
  const auto search = file_manager.folder_registry_.find(handle);
  if (search != file_manager.folder_registry_.end() && !search->second.expired()) {
    ret_val = search->second.lock();
  }
  file_manager.file_registry_mutex.unlock();
  return ret_val;
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
