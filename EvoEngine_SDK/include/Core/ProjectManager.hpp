#pragma once
#include "AssetManager.hpp"
#include "FileManager.hpp"
#include "IAsset.hpp"
namespace evo_engine {
class ProjectManager {
  EVOENGINE_SINGLETON_INSTANCE(ProjectManager)
  friend class Application;

  friend class EditorLayer;
  friend class File;
  friend class Folder;
  friend class PhysicsLayer;
  friend class Resources;
  std::shared_ptr<Folder> assets_folder_;
  std::filesystem::path project_path_;
  std::filesystem::path assets_folder_path;
  std::optional<std::function<void(const std::shared_ptr<Scene>&)>> scene_post_load_function_;
  std::optional<std::function<void(const std::shared_ptr<Scene>&)>> new_scene_customizer_;
  std::weak_ptr<Folder> current_focused_folder_;

  friend class ClassRegistry;
  std::shared_ptr<Scene> start_scene_;
  int max_thumbnail_size_ = 256;
  friend class AssetRegistry;

  friend class EditorLayer;
  friend class IAsset;
  friend class Scene;
  friend class Prefab;

  bool initialized = false;
  static void FolderHierarchyHelper(const std::shared_ptr<Folder>& folder);
  static void Initialize();
  static void OnDestroy();

  bool scan_assets_pending = false;

  size_t pending_asset_size = 0;
  std::set<Handle> pending_assets;
  static void ScanAssets();
  std::filesystem::path new_project_path_ = "";
  static void SetupDefaultScene();
  static void PreUpdate();
  static void LoadAllPendingAssets();

 public:
  std::shared_ptr<IAsset> inspecting_asset;
  bool show_project_window = true;
  [[nodiscard]] static std::weak_ptr<Scene> GetStartScene();
  static void SetStartScene(const std::shared_ptr<Scene>& scene);
  static void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  static void SaveProject();
  static void SetActionAfterSceneLoad(const std::function<void(const std::shared_ptr<Scene>&)>& actions);
  static void SetActionAfterNewScene(const std::function<void(const std::shared_ptr<Scene>&)>& actions);
  [[nodiscard]] static std::filesystem::path GenerateNewAssetsRelativePath(const std::string& relative_stem,
                                                                           const std::string& postfix);
  [[nodiscard]] static std::filesystem::path GenerateNewAbsolutePath(const std::string& absolute_stem,
                                                                     const std::string& postfix);
  [[nodiscard]] static std::weak_ptr<Folder> GetCurrentFocusedFolder();
  [[nodiscard]] static std::shared_ptr<Folder> GetAssetsFolder();
  [[nodiscard]] static std::filesystem::path GetProjectPath();
  [[nodiscard]] static std::filesystem::path GetAssetsFolderPath();
  [[nodiscard]] static std::string GetProjectName();
  [[maybe_unused]] static std::weak_ptr<Folder> GetOrCreateFolder(const std::filesystem::path& assets_relative_path);
  [[nodiscard]] static std::shared_ptr<IAsset> GetOrCreateAsset(const std::filesystem::path& assets_relative_path);

  [[nodiscard]] static bool IsInAssetsFolder(const std::filesystem::path& absolute_path);
  [[nodiscard]] static bool IsValidAssetFileName(const std::filesystem::path& path);
  static void GetOrCreateProject(const std::filesystem::path& path);
  static void DispatchScanAssetsTask();

  [[nodiscard]] static std::filesystem::path GetAssetsRelativePath(const std::filesystem::path& absolute_path);
};
}  // namespace evo_engine