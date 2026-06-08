
#pragma once
#include "AssetManager.hpp"
#include "FileManager.hpp"
#include "IAsset.hpp"

#include <string>
#include <vector>

namespace evo_engine {

enum class ProjectState { NoProject, Loading, Loaded };

struct ProjectLaunchMetadata {
  std::string application_name = "EvoEngine Editor";
  std::vector<std::string> startup_runtime_packages;
  std::string preferred_editor = "EvoEngineEditor";
};

/**
 * @class ProjectManager
 * @brief A singleton class responsible for managing project-related operations, including asset management,
 *        folder hierarchy, and project settings.
 */
class ProjectManager {
 public:
  static ProjectManager& GetInstance();

 private:
  friend class Application;
  friend class EditorLayer;
  friend class ProjectContentBrowserPanel;
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

  ProjectLaunchMetadata project_launch_metadata_;  ///< Launcher/editor metadata persisted in the project file.

  friend class ClassRegistry;
  std::shared_ptr<Scene> start_scene_;  ///< The starting scene of the project.
  int max_thumbnail_size_ = 512;        ///< The maximum size in pixels for asset thumbnails.

  friend class AssetRegistry;
  friend class EditorLayer;
  friend class IAsset;
  friend class Scene;
  friend class Prefab;

  bool initialized = false;  ///< Indicates whether the project manager has been initialized.

  /**
   * @brief Initializes the project manager.
   */
  static void Initialize();

  /**
   * @brief Cleans up resources when the project manager is destroyed.
   */
  static void OnDestroy();

  bool scan_assets_pending = false;  ///< Indicates if an asset scan is pending.

  bool project_asset_load_dispatched = false;  ///< True while a project asset batch is owned by AssetManager.
  size_t pending_asset_size = 0;               ///< The count of assets pending to be processed.
  std::set<Handle> pending_assets;             ///< The list of handles to pending assets.

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

  static void DrawProjectMenu();

 public:
  bool show_project_window = true;  ///< Indicates whether the project window should be shown in the editor.

  /**
   * @brief Retrieves the starting scene of the project.
   * @return A weak pointer to the starting scene.
   */
  [[nodiscard]] static std::weak_ptr<Scene> GetStartScene();

  /**
   * @brief Retrieves the current project loading state.
   */
  [[nodiscard]] static ProjectState GetProjectState();

  /**
   * @brief Returns true once a project path has been selected, even if loading is still in progress.
   */
  [[nodiscard]] static bool HasProject();

  /**
   * @brief Returns true when a project path is selected and its start scene is ready.
   */
  [[nodiscard]] static bool IsProjectLoaded();

  /**
   * @brief Returns true when project scanning, project asset loading, and start-scene setup are complete.
   */
  [[nodiscard]] static bool IsProjectIdle();

  /**
   * @brief Loads launcher/editor metadata from a project file without opening the project.
   * @param path Project file path.
   * @return Parsed metadata, or default metadata if the file is missing metadata.
   */
  [[nodiscard]] static ProjectLaunchMetadata LoadProjectLaunchMetadata(const std::filesystem::path& path);

  /**
   * @brief Saves launcher/editor metadata to a project file without opening the project.
   * @param path Project file path.
   * @param metadata Metadata to persist.
   */
  static void SaveProjectLaunchMetadata(const std::filesystem::path& path, const ProjectLaunchMetadata& metadata);

  /**
   * @brief Returns metadata for the currently selected project.
   */
  [[nodiscard]] static ProjectLaunchMetadata GetProjectLaunchMetadata();

  /**
   * @brief Sets the starting scene for the project.
   * @param scene The new starting scene.
   */
  static void SetStartScene(const std::shared_ptr<Scene>& scene);

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
