#pragma once
#include "IAsset.hpp"
#include "Serialization.hpp"
namespace evo_engine {
class FolderRecord;
class Texture2D;
class FileRecord {
 public:
  [[nodiscard]] std::weak_ptr<FolderRecord> GetFolder() const;
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

  [[nodiscard]] std::shared_ptr<Texture2D> GetThumbnail();

 private:
  friend class FolderRecord;
  friend class ProjectManager;
  friend class IAsset;
  std::string asset_file_name_ = {};
  std::string asset_extension_ = {};
  std::string asset_type_name_ = "Binary";
  Handle asset_handle_ = 0;
  std::weak_ptr<IAsset> asset_;
  std::weak_ptr<FolderRecord> folder_;
  std::weak_ptr<FileRecord> self_;
  std::shared_ptr<Texture2D> thumbnail_;
};

class FolderRecord {
  friend class IAsset;
  friend class EditorLayer;
  friend class ProjectManager;
  std::string name_;
  std::unordered_map<Handle, std::shared_ptr<FileRecord>> files;
  std::map<Handle, std::shared_ptr<FolderRecord>> children_;
  std::weak_ptr<FolderRecord> parent_;
  Handle handle_ = 0;
  std::weak_ptr<FolderRecord> self_;
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

  void MoveChild(const Handle& child_handle, const std::shared_ptr<FolderRecord>& dest);
  void DeleteChild(const Handle& child_handle);
  [[nodiscard]] std::weak_ptr<FolderRecord> GetChild(const Handle& child_handle);
  [[nodiscard]] std::weak_ptr<FolderRecord> GetOrCreateChild(const std::string& folder_name);

  void MoveAsset(const Handle& asset_handle, const std::shared_ptr<FolderRecord>& dest);
  void RemoveFile(const Handle& asset_handle);
  [[nodiscard]] bool FileRecorded(const std::string& file_name, const std::string& extension) const;
  [[maybe_unused]] std::shared_ptr<IAsset> GetOrCreateAsset(const std::string& file_name, const std::string& extension);
  [[nodiscard]] std::shared_ptr<IAsset> GetAsset(const Handle& asset_handle);
  std::optional<std::shared_ptr<IAsset>> Duplicate(const Handle& handle);
  void Save() const;
  void Load(const std::filesystem::path& path);
  virtual ~FolderRecord();
};

class ProjectManager {
  EVOENGINE_SINGLETON_INSTANCE(ProjectManager)
  friend class Application;

  friend class EditorLayer;
  friend class FileRecord;
  friend class FolderRecord;
  friend class PhysicsLayer;
  friend class Resources;
  std::shared_ptr<FolderRecord> project_folder_;
  std::filesystem::path project_path_;
  std::optional<std::function<void(const std::shared_ptr<Scene>&)>> scene_post_load_function_;
  std::optional<std::function<void(const std::shared_ptr<Scene>&)>> new_scene_customizer_;
  std::weak_ptr<FolderRecord> current_focused_folder_;
  std::unordered_map<Handle, std::shared_ptr<IAsset>> loaded_assets_;
  std::unordered_map<Handle, std::weak_ptr<IAsset>> asset_registry_;
  std::unordered_map<Handle, std::weak_ptr<FileRecord>> file_registry_;
  std::unordered_map<Handle, std::weak_ptr<FolderRecord>> folder_registry_;

  friend class ClassRegistry;
  std::shared_ptr<Scene> start_scene_;
  int max_thumbnail_size_ = 256;
  friend class AssetRegistry;

  friend class EditorLayer;
  friend class IAsset;
  friend class Scene;
  friend class Prefab;

  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAsset(const std::string& type_name);
  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAsset(const std::string& type_name, const Handle& handle);
  bool initialized = false;
  static void FolderHierarchyHelper(const std::shared_ptr<FolderRecord>& folder);

 public:
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
  [[nodiscard]] static std::weak_ptr<FolderRecord> GetCurrentFocusedFolder();
  [[nodiscard]] static std::filesystem::path GetProjectPath();
  [[nodiscard]] static std::string GetProjectName();
  [[maybe_unused]] static std::weak_ptr<FolderRecord> GetOrCreateFolder(
      const std::filesystem::path& project_relative_path);
  [[nodiscard]] static std::shared_ptr<IAsset> GetOrCreateAsset(const std::filesystem::path& project_relative_path);
  [[nodiscard]] static std::shared_ptr<IAsset> GetAsset(const Handle& handle);
  [[nodiscard]] static std::weak_ptr<FolderRecord> GetFolder(const Handle& handle);
  static void GetOrCreateProject(const std::filesystem::path& path);
  [[nodiscard]] static bool IsInProjectFolder(const std::filesystem::path& absolute_path);
  [[nodiscard]] static bool IsValidAssetFileName(const std::filesystem::path& path);
  template <typename T>
  [[nodiscard]] static std::shared_ptr<T> CreateTemporaryAsset();
  static void ScanProject();

  static void Initialize();
  static void OnDestroy();
  [[nodiscard]] static std::filesystem::path GetPathRelativeToProject(const std::filesystem::path& absolute_path);

  static bool StartupGui();
};
template <typename T>
std::shared_ptr<T> ProjectManager::CreateTemporaryAsset() {
  return std::dynamic_pointer_cast<T>(CreateTemporaryAsset(Serialization::GetSerializableTypeName<T>()));
}
}  // namespace evo_engine