#pragma once
#include "IAsset.hpp"
#include "Serialization.hpp"
namespace evo_engine {

class Folder;
class File {
 public:
  [[nodiscard]] std::weak_ptr<Folder> GetFolder() const;
  [[nodiscard]] Handle GetAssetHandle() const;
  [[nodiscard]] std::string GetAssetTypeName() const;
  [[nodiscard]] std::string GetAssetFileName() const;
  [[nodiscard]] std::string GetAssetExtension() const;
  void DeleteMetadata() const;

  [[nodiscard]] std::filesystem::path GetProjectRelativePath() const;
  [[nodiscard]] std::filesystem::path GetAbsolutePath() const;
  void SetAssetFileName(const std::string& new_name);
  void SetAssetExtension(const std::string& new_extension);

  void Save() const;
  void Load(const std::filesystem::path& path);

  [[nodiscard]] std::shared_ptr<Texture2D> GetThumbnail();

 private:
  friend class Folder;
  friend class ProjectManager;
  friend class IAsset;
  friend class AssetManager;
  std::string asset_file_name_ = {};
  std::string asset_extension_ = {};
  std::string asset_type_name_ = "Binary";
  Handle asset_handle_ = 0;
  std::weak_ptr<Folder> folder_;
  std::weak_ptr<File> self_;
  std::shared_ptr<Texture2D> thumbnail_;
};

class Folder {
  friend class IAsset;
  friend class EditorLayer;
  friend class ProjectManager;
  friend class AssetManager;
  std::string name_;
  std::unordered_map<Handle, std::shared_ptr<File>> files;
  std::map<Handle, std::shared_ptr<Folder>> children_;
  std::weak_ptr<Folder> parent_;
  Handle handle_ = 0;
  std::weak_ptr<Folder> self_;
  void Refresh();
  std::weak_ptr<File> RegisterAsset(const Handle& asset_handle, const std::string& type_name, const std::string& file_name, const std::string& extension);

 public:
  bool IsSelfOrAncestor(const Handle& handle) const;
  void DeleteMetadata() const;
  [[nodiscard]] Handle GetHandle() const;
  [[nodiscard]] std::filesystem::path GetAssetsRelativePath() const;
  [[nodiscard]] std::filesystem::path GetAbsolutePath() const;
  [[nodiscard]] std::string GetName() const;

  void Rename(const std::string& new_name);

  void MoveChild(const Handle& child_handle, const std::shared_ptr<Folder>& dest);
  void DeleteChild(const Handle& child_handle);
  [[nodiscard]] std::weak_ptr<Folder> GetChild(const Handle& child_handle);
  [[nodiscard]] std::weak_ptr<Folder> GetOrCreateChild(const std::string& folder_name);

  void MoveAsset(const Handle& asset_handle, const std::shared_ptr<Folder>& dest);
  void RemoveFile(const Handle& asset_handle);
  [[nodiscard]] bool FileRecorded(const std::string& file_name, const std::string& extension) const;
  [[maybe_unused]] std::shared_ptr<IAsset> GetOrCreateAsset(const std::string& file_name, const std::string& extension);
  [[nodiscard]] std::shared_ptr<IAsset> GetAsset(const Handle& asset_handle);
  std::optional<std::shared_ptr<IAsset>> Duplicate(const Handle& handle);
  void Save() const;
  void Load(const std::filesystem::path& path);
  virtual ~Folder();
};

class FileManager {
  EVOENGINE_SINGLETON_INSTANCE(FileManager)

  std::unordered_map<Handle, std::weak_ptr<File>> file_registry_;
  std::unordered_map<Handle, std::weak_ptr<Folder>> folder_registry_;

  friend class Folder;
  friend class File;
  friend class Application;
  friend class ProjectManager;
  static void Initialize();
  static void OnDestroy();
  std::mutex file_registry_mutex;

 public:
  static void Clear();

  static std::shared_ptr<File> GetFile(const Handle& handle);
  static std::shared_ptr<Folder> GetFolder(const Handle& handle);
};

}  // namespace evo_engine