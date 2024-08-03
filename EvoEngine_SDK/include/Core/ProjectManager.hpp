#pragma once
#include "IAsset.hpp"
#include "Serialization.hpp"
namespace evo_engine {
class Folder;
class AssetRecord {
  friend class Folder;
  friend class IAsset;
  std::string asset_file_name_ = {};
  std::string asset_extension_ = {};
  std::string asset_type_name_ = "Binary";
  Handle asset_handle_ = 0;
  std::weak_ptr<IAsset> asset_;
  std::weak_ptr<Folder> folder_;
  std::weak_ptr<AssetRecord> self_;

 public:
  [[nodiscard]] std::weak_ptr<Folder> GetFolder() const;
  [[nodiscard]] Handle GetAssetHandle() const;
  [[nodiscard]] std::shared_ptr<IAsset> GetAsset();
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
};

class Folder {
  friend class IAsset;
  friend class EditorLayer;
  friend class ProjectManager;
  std::string name_;
  std::unordered_map<Handle, std::shared_ptr<AssetRecord>> asset_records_;
  std::map<Handle, std::shared_ptr<Folder>> children_;
  std::weak_ptr<Folder> parent_;
  Handle handle_ = 0;
  std::weak_ptr<Folder> self_;
  void Refresh(const std::filesystem::path& parent_absolute_path);
  void RegisterAsset(const std::shared_ptr<IAsset>& asset, const std::string& file_name, const std::string& extension);

 public:
  bool IsSelfOrAncestor(const Handle& handle) const;
  void DeleteMetadata() const;
  [[nodiscard]] Handle GetHandle() const;
  [[nodiscard]] std::filesystem::path GetProjectRelativePath() const;
  [[nodiscard]] std::filesystem::path GetAbsolutePath() const;
  [[nodiscard]] std::string GetName() const;

  void Rename(const std::string& new_name);

  void MoveChild(const Handle& child_handle, const std::shared_ptr<Folder>& dest);
  void DeleteChild(const Handle& child_handle);
  [[nodiscard]] std::weak_ptr<Folder> GetChild(const Handle& child_handle);
  [[nodiscard]] std::weak_ptr<Folder> GetOrCreateChild(const std::string& folder_name);

  void MoveAsset(const Handle& asset_handle, const std::shared_ptr<Folder>& dest);
  void DeleteAsset(const Handle& asset_handle);
  [[nodiscard]] bool HasAsset(const std::string& file_name, const std::string& extension) const;
  [[maybe_unused]] std::shared_ptr<IAsset> GetOrCreateAsset(const std::string& file_name, const std::string& extension);
  [[nodiscard]] std::shared_ptr<IAsset> GetAsset(const Handle& asset_handle);

  void Save() const;
  void Load(const std::filesystem::path& path);
  virtual ~Folder();
};

class Texture2D;

class AssetThumbnail {
  std::shared_ptr<Texture2D> icon_;
};

class ProjectManager {
  EVOENGINE_SINGLETON_INSTANCE(ProjectManager)
  friend class Application;

  friend class EditorLayer;
  friend class AssetRecord;
  friend class Folder;
  friend class PhysicsLayer;
  friend class Resources;
  std::shared_ptr<Folder> project_folder_;
  std::filesystem::path project_path_;
  std::optional<std::function<void(const std::shared_ptr<Scene>&)>> scene_post_load_function_;
  std::optional<std::function<void(const std::shared_ptr<Scene>&)>> new_scene_customizer_;
  std::weak_ptr<Folder> current_focused_folder_;
  std::unordered_map<Handle, std::shared_ptr<IAsset>> loaded_assets_;
  std::unordered_map<Handle, std::weak_ptr<IAsset>> asset_registry_;
  std::unordered_map<Handle, std::weak_ptr<AssetRecord>> asset_record_registry_;
  std::unordered_map<Handle, std::weak_ptr<Folder>> folder_registry_;

  friend class ClassRegistry;
  std::shared_ptr<Scene> start_scene_;

  std::unordered_map<Handle, std::weak_ptr<AssetThumbnail>> asset_thumbnails_;
  std::vector<std::shared_ptr<AssetThumbnail>> asset_thumbnail_storage_;
  int max_thumbnail_size_ = 256;
  friend class AssetRegistry;
  friend class ProjectManager;

  friend class EditorLayer;
  friend class IAsset;
  friend class Scene;
  friend class Prefab;

  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAsset(const std::string& type_name);
  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAsset(const std::string& type_name, const Handle& handle);

  static void FolderHierarchyHelper(const std::shared_ptr<Folder>& folder);

 public:
  [[nodiscard]] static std::shared_ptr<IAsset> DuplicateAsset(const std::shared_ptr<IAsset>& target);
  std::shared_ptr<IAsset> inspecting_asset;
  bool show_project_window = true;
  [[nodiscard]] static std::weak_ptr<Scene> GetStartScene();
  static void SetStartScene(const std::shared_ptr<Scene>& scene);
  static void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  static void SaveProject();
  static void SetActionAfterSceneLoad(const std::function<void(const std::shared_ptr<Scene>&)>& actions);
  static void SetActionAfterNewScene(const std::function<void(const std::shared_ptr<Scene>&)>& actions);
  [[nodiscard]] static std::filesystem::path GenerateNewProjectRelativePath(const std::string& relative_stem,
                                                                            const std::string& postfix);
  [[nodiscard]] static std::filesystem::path GenerateNewAbsolutePath(const std::string& absolute_stem,
                                                                     const std::string& postfix);
  [[nodiscard]] static std::weak_ptr<Folder> GetCurrentFocusedFolder();
  [[nodiscard]] static std::filesystem::path GetProjectPath();
  [[nodiscard]] static std::string GetProjectName();
  [[maybe_unused]] static std::weak_ptr<Folder> GetOrCreateFolder(const std::filesystem::path& project_relative_path);
  [[nodiscard]] static std::shared_ptr<IAsset> GetOrCreateAsset(const std::filesystem::path& project_relative_path);
  [[nodiscard]] static std::shared_ptr<IAsset> GetAsset(const Handle& handle);
  [[nodiscard]] static std::weak_ptr<Folder> GetFolder(const Handle& handle);
  static void GetOrCreateProject(const std::filesystem::path& path);
  [[nodiscard]] static bool IsInProjectFolder(const std::filesystem::path& absolute_path);
  [[nodiscard]] static bool IsValidAssetFileName(const std::filesystem::path& path);
  template <typename T>
  [[nodiscard]] static std::shared_ptr<T> CreateTemporaryAsset();
  static void ScanProject();
  static void OnDestroy();
  [[nodiscard]] static std::filesystem::path GetPathRelativeToProject(const std::filesystem::path& absolute_path);
};
template <typename T>
std::shared_ptr<T> ProjectManager::CreateTemporaryAsset() {
  return std::dynamic_pointer_cast<T>(CreateTemporaryAsset(Serialization::GetSerializableTypeName<T>()));
}
}  // namespace evo_engine