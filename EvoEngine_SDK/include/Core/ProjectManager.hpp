
#pragma once
#include "AssetManager.hpp"
#include "FileManager.hpp"
#include "IAsset.hpp"

namespace evo_engine {

/**
 * @class ProjectManager
 * @brief A singleton class responsible for managing project-related operations, including asset management,
 *        folder hierarchy, and project settings.
 */
class ProjectManager {
  EVOENGINE_SINGLETON_INSTANCE(ProjectManager)
  friend class Application;
  friend class EditorLayer;
  friend class File;
  friend class Folder;
  friend class PhysicsLayer;
  friend class Resources;

  std::shared_ptr<Folder> assets_folder_;    ///< The root folder for all assets in the project.
  std::filesystem::path project_path_;       ///< The full path to the project directory.
  std::filesystem::path assets_folder_path;  ///< The absolute path to the assets folder in the project.

  std::optional<std::function<void(const std::shared_ptr<Scene>&)>>
      scene_post_load_function_;  ///< Callback function invoked after a scene is loaded.
  std::optional<std::function<void(const std::shared_ptr<Scene>&)>>
      new_scene_customizer_;  ///< Callback function for customizing a new scene.

  std::weak_ptr<Folder> current_focused_folder_;  ///< A weak pointer to the currently focused folder.

  friend class ClassRegistry;
  std::shared_ptr<Scene> start_scene_;  ///< The starting scene of the project.
  int max_thumbnail_size_ = 256;        ///< The maximum size in pixels for asset thumbnails.

  friend class AssetRegistry;
  friend class EditorLayer;
  friend class IAsset;
  friend class Scene;
  friend class Prefab;

  bool initialized = false;  ///< Indicates whether the project manager has been initialized.

  /**
   * @brief Recursive helper function to manage the folder hierarchy.
   * @param folder The folder to process.
   */
  static void FolderHierarchyHelper(const std::shared_ptr<Folder>& folder);

  /**
   * @brief Initializes the project manager.
   */
  static void Initialize();

  /**
   * @brief Cleans up resources when the project manager is destroyed.
   */
  static void OnDestroy();

  bool scan_assets_pending = false;  ///< Indicates if an asset scan is pending.

  size_t pending_asset_size = 0;    ///< The count of assets pending to be processed.
  std::set<Handle> pending_assets;  ///< The list of handles to pending assets.

  /**
   * @brief Scans and updates the asset list based on the current assets folder.
   */
  static void ScanAssets();

  std::filesystem::path new_project_path_ = "";  ///< The path to a newly created project.

  /**
   * @brief Sets up the default scene for the project.
   */
  static void SetupDefaultScene();

  /**
   * @brief Pre-update hook for any project manager operations that need to occur before other updates.
   */
  static void PreUpdate();

  /**
   * @brief Loads all assets that are pending processing.
   */
  static void LoadAllPendingAssets();

 public:
  std::shared_ptr<IAsset> inspecting_asset;  ///< The asset currently being inspected in the editor.
  bool show_project_window = true;           ///< Indicates whether the project window should be shown in the editor.

  /**
   * @brief Retrieves the starting scene of the project.
   * @return A weak pointer to the starting scene.
   */
  [[nodiscard]] static std::weak_ptr<Scene> GetStartScene();

  /**
   * @brief Sets the starting scene for the project.
   * @param scene The new starting scene.
   */
  static void SetStartScene(const std::shared_ptr<Scene>& scene);

  /**
   * @brief Triggers inspection of an asset within the editor layer.
   * @param editor_layer The editor layer handling the inspection process.
   */
  static void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Saves the current project state to disk.
   */
  static void SaveProject();

  /**
   * @brief Sets a custom action to be executed after a scene is loaded.
   * @param actions The function to execute.
   */
  static void SetActionAfterSceneLoad(const std::function<void(const std::shared_ptr<Scene>&)>& actions);

  /**
   * @brief Sets a custom action to be executed for a new scene.
   * @param actions The function to execute.
   */
  static void SetActionAfterNewScene(const std::function<void(const std::shared_ptr<Scene>&)>& actions);

  /**
   * @brief Generates a relative path for a new asset inside the assets folder.
   * @param relative_stem The base path relative to the assets folder.
   * @param postfix The postfix to append to the file name.
   * @return The generated relative path.
   */
  [[nodiscard]] static std::filesystem::path GenerateNewAssetsRelativePath(const std::string& relative_stem,
                                                                           const std::string& postfix);

  /**
   * @brief Generates an absolute path for a new asset.
   * @param absolute_stem The base absolute path.
   * @param postfix The postfix to append to the file name.
   * @return The generated absolute path.
   */
  [[nodiscard]] static std::filesystem::path GenerateNewAbsolutePath(const std::string& absolute_stem,
                                                                     const std::string& postfix);

  /**
   * @brief Retrieves the currently focused folder.
   * @return A weak pointer to the currently focused folder.
   */
  [[nodiscard]] static std::weak_ptr<Folder> GetCurrentFocusedFolder();

  /**
   * @brief Accesses the root assets folder.
   * @return A shared pointer to the root assets folder.
   */
  [[nodiscard]] static std::shared_ptr<Folder> GetAssetsFolder();

  /**
   * @brief Gets the absolute path of the project directory.
   * @return The project's folder path.
   */
  [[nodiscard]] static std::filesystem::path GetProjectPath();

  /**
   * @brief Gets the absolute path of the assets directory.
   * @return The folder path for assets.
   */
  [[nodiscard]] static std::filesystem::path GetAssetsFolderPath();

  /**
   * @brief Retrieves the name of the project.
   * @return The project name as a string.
   */
  [[nodiscard]] static std::string GetProjectName();

  /**
   * @brief Finds or creates a folder using the relative path within the assets directory.
   * @param assets_relative_path The relative path to the folder.
   * @return A weak pointer to the folder.
   */
  [[maybe_unused]] static std::weak_ptr<Folder> GetOrCreateFolder(const std::filesystem::path& assets_relative_path);

  /**
   * @brief Finds or creates an asset using the relative path within the assets directory.
   * @param assets_relative_path The relative path to the asset.
   * @return A shared pointer to the asset.
   */
  [[nodiscard]] static std::shared_ptr<IAsset> GetOrCreateAsset(const std::filesystem::path& assets_relative_path);

  /**
   * @brief Checks if the specified absolute path belongs to the assets folder.
   * @param absolute_path The absolute path to check.
   * @return True if the path is within the assets folder, false otherwise.
   */
  [[nodiscard]] static bool IsInAssetsFolder(const std::filesystem::path& absolute_path);

  /**
   * @brief Validates that a given file name is suitable for an asset.
   * @param path The path of the file to validate.
   * @return True if the file name is valid, false otherwise.
   */
  [[nodiscard]] static bool IsValidAssetFileName(const std::filesystem::path& path);

  /**
   * @brief Finds or creates a project at the specified path.
   * @param path The path where the project resides or should be created.
   */
  static void GetOrCreateProject(const std::filesystem::path& path);

  /**
   * @brief Dispatches a task to scan the assets directory for updates or changes.
   */
  static void DispatchScanAssetsTask();

  /**
   * @brief Converts an absolute path to a relative path within the assets folder.
   * @param absolute_path The absolute path to convert.
   * @return The corresponding assets-relative path.
   */
  [[nodiscard]] static std::filesystem::path GetAssetsRelativePath(const std::filesystem::path& absolute_path);
};

}  // namespace evo_engine

// All methods already documented.
// Task is complete.
