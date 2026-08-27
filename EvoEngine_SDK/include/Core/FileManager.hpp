
#pragma once
#include "IAsset.hpp"
#include "Serialization.hpp"

#include <filesystem>
#include <future>

namespace evo_engine {
class EVOENGINE_API ProjectContentBrowserPanel;
class EVOENGINE_API Folder;
/**
 * @brief Represents a file in the asset management system.
 */
class EVOENGINE_API File {
 public:
  /**
   * @brief Retrieves the folder this file belongs to.
   * @return A weak pointer to the parent folder.
   */
  [[nodiscard]] std::weak_ptr<Folder> GetFolder() const;

  /**
   * @brief Retrieves the handle of this asset.
   * @return The asset handle.
   */
  [[nodiscard]] Handle GetAssetHandle() const;

  /**
   * @brief Retrieves the type name of the asset.
   * @return A string containing the asset type name.
   */
  [[nodiscard]] std::string GetAssetTypeName() const;

  /**
   * @brief Retrieves the file name of the asset.
   * @return A string containing the file name of the asset.
   */
  [[nodiscard]] std::string GetAssetFileName() const;

  /**
   * @brief Retrieves the file extension of the asset.
   * @return A string containing the file extension of the asset.
   */
  [[nodiscard]] std::string GetAssetExtension() const;

  /**
   * @brief Deletes the metadata associated with the file.
   */
  void DeleteMetadata() const;

  /**
   * @brief Retrieves the relative path of the asset folder.
   * @return A filesystem path relative to the assets folder.
   */
  [[nodiscard]] std::filesystem::path GetAssetsFolderRelativePath() const;

  /**
   * @brief Retrieves the absolute path of the asset file.
   * @return A filesystem path containing the absolute path.
   */
  [[nodiscard]] std::filesystem::path GetAbsolutePath() const;

  /**
   * @brief Sets a new file name for the asset.
   * @param new_name The new file name to set.
   */
  void SetAssetFileName(const std::string& new_name);

  /**
   * @brief Sets a new file extension for the asset.
   * @param new_extension The new file extension to set.
   */
  void SetAssetExtension(const std::string& new_extension);

  /**
   * @brief Saves the file metadata to its associated storage.
   */
  void Save() const;

  /**
   * @brief Loads the file data from its stored location.
   * @param path The filesystem path to load the file from.
   */
  void Load(const std::filesystem::path& path);

  /**
   * @brief Retrieves the thumbnail representation of the file.
   * @param allow_asset_load Whether this call may enqueue/load/generate the asset-backed thumbnail.
   * @return A shared pointer to the thumbnail texture.
   */
  std::shared_ptr<Texture2D> GetThumbnail(bool allow_asset_load = true);

 private:
  friend class Folder;
  friend class ProjectManager;
  friend class ProjectContentBrowserPanel;
  friend class IAsset;
  friend class AssetManager;

  void InvalidateThumbnail();
  void SyncThumbnailSourceWriteTime();
  [[nodiscard]] std::shared_ptr<Texture2D> GetFallbackThumbnail() const;

  std::string asset_file_name_ = {};       /**< The name of the asset file. */
  std::string asset_extension_ = {};       /**< The extension of the asset file. */
  std::string asset_type_name_ = "Binary"; /**< The type name of the asset. */
  Handle asset_handle_ = 0;                /**< The handle associated with the asset. */
  std::weak_ptr<Folder> folder_;           /**< Weak pointer to the parent folder. */
  std::weak_ptr<File> self_;               /**< Weak pointer to this file instance. */

  std::shared_ptr<IAsset> asset_;                                /**< Pointer to the associated asset. */
  std::shared_ptr<Texture2D> thumbnail_;                         /**< Pointer to the generated file thumbnail. */
  std::shared_future<std::shared_ptr<IAsset>> thumbnail_future_; /**< In-flight asset load for thumbnail generation. */
  std::filesystem::file_time_type thumbnail_source_write_time_;  /**< Last source write time used for thumbnail. */
  bool thumbnail_source_write_time_initialized_ = false;         /**< Whether source write time has been captured. */
  bool thumbnail_asset_reload_required_ = false; /**< Whether the loaded asset must reload before thumbnail. */
};

/**
 * @brief Represents a folder in the asset management system.
 */
class EVOENGINE_API Folder {
  friend class IAsset;
  friend class EditorLayer;
  friend class ProjectManager;
  friend class ProjectContentBrowserPanel;
  friend class AssetManager;

  std::string name_;                                       /**< The name of the folder. */
  std::unordered_map<Handle, std::shared_ptr<File>> files; /**< Files within this folder. */
  std::map<Handle, std::shared_ptr<Folder>> children_;     /**< Child folders. */
  std::weak_ptr<Folder> parent_;                           /**< Weak pointer to the parent folder. */
  Handle handle_ = 0;                                      /**< The handle associated with this folder. */
  std::weak_ptr<Folder> self_;                             /**< Weak pointer to this folder instance. */

  /**
   * @brief Refreshes the folder contents and updates assets pending loading.
   * @param assets_pending_loading A vector of asset handles pending loading.
   */
  void Refresh(std::vector<Handle>& assets_pending_loading);

  /**
   * @brief Registers a new asset within the folder.
   * @param asset_handle The handle of the asset.
   * @param type_name The type name of the asset.
   * @param file_name The file name of the asset.
   * @param extension The file extension of the asset.
   * @return A weak pointer to the registered asset file.
   */
  std::weak_ptr<File> RegisterAsset(const Handle& asset_handle, const std::string& type_name,
                                    const std::string& file_name, const std::string& extension);

 public:
  /**
   * @brief Checks if the folder is itself or an ancestor of the specified handle.
   * @param handle The handle to check.
   * @return True if the folder is itself or an ancestor, false otherwise.
   */
  bool IsSelfOrAncestor(const Handle& handle) const;

  /**
   * @brief Deletes the metadata associated with the folder.
   */
  void DeleteMetadata() const;

  /**
   * @brief Retrieves the handle associated with the folder.
   * @return The folder handle.
   */
  [[nodiscard]] Handle GetHandle() const;

  /**
   * @brief Retrieves the relative path of the folder in the assets directory.
   * @return A filesystem path relative to the assets directory.
   */
  [[nodiscard]] std::filesystem::path GetAssetsRelativePath() const;

  /**
   * @brief Retrieves the absolute path of the folder.
   * @return A filesystem path containing the absolute path.
   */
  [[nodiscard]] std::filesystem::path GetAbsolutePath() const;

  /**
   * @brief Retrieves the name of the folder.
   * @return A string containing the folder name.
   */
  [[nodiscard]] std::string GetName() const;

  /**
   * @brief Renames the folder with a new name.
   * @param new_name The new name for the folder.
   */
  void Rename(const std::string& new_name);

  /**
   * @brief Moves a child folder to another destination folder.
   * @param child_handle The handle of the child folder to move.
   * @param dest The destination folder to move the child into.
   */
  void MoveChild(const Handle& child_handle, const std::shared_ptr<Folder>& dest);

  /**
   * @brief Deletes a child folder by its handle.
   * @param child_handle The handle of the child folder to delete.
   */
  void DeleteChild(const Handle& child_handle);

  /**
   * @brief Retrieves a child folder by its handle.
   * @param child_handle The handle of the child folder to retrieve.
   * @return A weak pointer to the child folder.
   */
  [[nodiscard]] std::weak_ptr<Folder> GetChild(const Handle& child_handle);

  /**
   * @brief Retrieves a child folder by name or creates one if it doesn't exist.
   * @param folder_name The name of the child folder to retrieve or create.
   * @return A weak pointer to the child folder.
   */
  [[nodiscard]] std::weak_ptr<Folder> GetOrCreateChild(const std::string& folder_name);

  /**
   * @brief Moves an asset to a different folder.
   * @param asset_handle The handle of the asset to move.
   * @param dest The destination folder to move the asset into.
   * @return A shared pointer to the moved asset file.
   */
  std::shared_ptr<File> MoveAsset(const Handle& asset_handle, const std::shared_ptr<Folder>& dest);

  /**
   * @brief Removes a file from the folder by its asset handle.
   * @param asset_handle The handle of the asset file to remove.
   */
  void RemoveFile(const Handle& asset_handle);

  /**
   * @brief Checks if a file with a specific name and extension is recorded in the folder.
   * @param file_name The name of the file to check.
   * @param extension The extension of the file to check.
   * @return True if the file is recorded, false otherwise.
   */
  [[nodiscard]] bool FileRecorded(const std::string& file_name, const std::string& extension) const;

  /**
   * @brief Retrieves or creates an asset by its file name and extension.
   * @param file_name The name of the asset file.
   * @param extension The extension of the asset file.
   * @return A shared pointer to the asset.
   */
  [[maybe_unused]] std::shared_ptr<IAsset> GetOrCreateAsset(const std::string& file_name, const std::string& extension);

  /**
   * @brief Retrieves an asset by its handle.
   * @param asset_handle The handle of the asset to retrieve.
   * @return A shared pointer to the asset.
   */
  [[nodiscard]] std::shared_ptr<IAsset> GetAsset(const Handle& asset_handle);

  /**
   * @brief Duplicates the asset specified by its handle.
   * @param handle The handle of the asset to duplicate.
   * @return An optional shared pointer to the duplicated asset.
   */
  std::optional<std::shared_ptr<IAsset>> Duplicate(const Handle& handle);

  /**
   * @brief Saves the folder metadata to its associated storage.
   */
  void Save() const;

  /**
   * @brief Loads the folder data from its stored location.
   * @param path The filesystem path to load the folder from.
   */
  void Load(const std::filesystem::path& path);

  /**
   * @brief Destructor for the Folder class.
   */
  virtual ~Folder();
};

/**
 * @brief Manages files and folders in the asset management system.
 */
class EVOENGINE_API FileManager {
 public:
  static FileManager& GetInstance();

 private:
  std::unordered_map<Handle, std::weak_ptr<File>> file_registry_;     /**< Registry of files by handle. */
  std::unordered_map<Handle, std::weak_ptr<Folder>> folder_registry_; /**< Registry of folders by handle. */

  friend class Folder;
  friend class File;
  friend class Application;
  friend class ProjectManager;

  /**
   * @brief Initializes the file manager.
   */
  static void Initialize();

  /**
   * @brief Cleans up resources used by the file manager.
   */
  static void OnDestroy();

  std::mutex file_registry_mutex; /**< Mutex for thread-safe file registry access. */

 public:
  /**
   * @brief Clears all file and folder records from the file manager.
   */
  static void Clear();

  /**
   * @brief Retrieves a file by its handle.
   * @param handle The handle of the file to retrieve.
   * @return A shared pointer to the file.
   */
  static std::shared_ptr<File> GetFile(const Handle& handle);

  /**
   * @brief Retrieves a folder by its handle.
   * @param handle The handle of the folder to retrieve.
   * @return A shared pointer to the folder.
   */
  static std::shared_ptr<Folder> GetFolder(const Handle& handle);
};

}  // namespace evo_engine
