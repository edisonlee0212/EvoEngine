
#pragma once
#include "IHandle.hpp"
#include "ISerializable.hpp"

namespace evo_engine {

/**
 * Forward declarations of other classes.
 */
class EditorLayer;
class AssetRef;
class File;
class Texture2D;

/**
 * @class IAsset
 * @brief Base class for managing assets in the evo_engine framework. Provides functionality for serialization,
 *        deserialization, and interactions with the asset's file system and the editor.
 */
class IAsset : public ISerializable {
  std::weak_ptr<IAsset>
      self_; /**< Weak reference to the current IAsset instance. Used internally for managing self-references. */

 protected:
  /** @cond DOXYGEN_SHOULD_SKIP_THIS */
  friend class Resources;
  friend class EditorLayer;
  friend class AssetRegistry;
  friend class ProjectManager;
  friend class File;
  friend class Folder;
  friend class AssetManager;
  /** @endcond */

  std::weak_ptr<File> file_record_; /**< Weak reference to the file metadata associated with this asset. */

  /**
   * @brief Gets a shared pointer to the current asset instance.
   * @return A shared pointer to this asset.
   */
  [[nodiscard]] std::shared_ptr<IAsset> GetSelf() const;

  /**
   * @brief Handles asset-specific saving logic.
   * @param path The file path to save the asset to, which may or may not be the local stored path.
   * @return Whether the save operation was successful.
   */
  virtual bool SaveInternal(const std::filesystem::path& path) const;

  /**
   * @brief Handles asset-specific loading logic.
   * @param path The file path to load the asset from, which may or may not be the local stored path.
   * @return Whether the load operation was successful.
   */
  virtual bool LoadInternal(const std::filesystem::path& path);

  bool saved_ = false;   /**< Indicates whether the asset is in a saved state. */
  uint32_t version_ = 0; /**< The version number of the asset. */

 public:
  /**
   * @brief Generates a thumbnail texture for the asset.
   * @return A shared pointer to the generated thumbnail texture.
   */
  [[nodiscard]] virtual std::shared_ptr<Texture2D> GenerateThumbnailTexture();

  /**
   * @brief Gets the version number of the asset.
   * @return The version number.
   */
  [[nodiscard]] uint32_t GetVersion() const;

  /**
   * @brief Sets the file path for the asset and saves it.
   * @param asset_folder_relative_path The relative path to the asset folder.
   * @return Whether the save operation was successful.
   */
  [[maybe_unused]] bool SetPathAndSave(const std::filesystem::path& asset_folder_relative_path);

  /**
   * @brief Gets the file path of the asset relative to the assets folder.
   * @return The relative file path.
   */
  [[nodiscard]] std::filesystem::path GetAssetsFolderRelativePath() const;

  /**
   * @brief Gets the absolute file path of the asset.
   * @return The absolute file path.
   */
  [[nodiscard]] std::filesystem::path GetAbsolutePath() const;

  /**
   * @brief Gets the title of the asset.
   * @return The title string.
   */
  [[nodiscard]] std::string GetTitle() const;

  /**
   * @brief Checks if the asset is temporary.
   * @return True if the asset is temporary, false otherwise.
   */
  [[nodiscard]] bool IsTemporary() const;

  /**
   * @brief Gets the file record associated with the asset.
   * @return A weak pointer to the file record.
   */
  [[nodiscard]] std::weak_ptr<File> GetFileRecord() const;

  /**
   * @brief Invoked right after the asset is created.
   */
  virtual void OnCreate();

  /**
   * @brief Saves the asset to its file path. Does nothing if the path is empty.
   * @return Whether the save operation was successful.
   */
  bool Save();

  /**
   * @brief Loads the asset from its file path. Does nothing if the path is empty.
   * @return Whether the load operation was successful.
   */
  bool Load();

  /**
   * @brief Exports the current asset to a specified path. Does not affect the asset's internal path.
   * @param path The target path for exporting the asset. Must be an absolute path and outside the project folder.
   * @return Whether the export operation was successful.
   */
  [[maybe_unused]] bool Export(const std::filesystem::path& path) const;

  /**
   * @brief Imports the asset from a specified path. Does not affect the asset's internal path.
   * @param path The source path for importing the asset. Must be an absolute path and outside the project folder.
   * @return Whether the import operation was successful.
   */
  [[maybe_unused]] bool Import(const std::filesystem::path& path);

  /**
   * @brief Provides the GUI representation of the asset when inspected in the editor.
   * @param editor_layer A shared pointer to the editor layer.
   * @return Whether the asset was modified during the inspection.
   */
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }

  /**
   * @brief Marks all AssetRef members for serialization during prefab and scene serialization.
   * @param list A list to collect the AssetRef of all members. Push all AssetRef instances of the class members to this
   *             list to ensure proper deserialization behavior.
   */
  virtual void CollectAssetRef(std::vector<AssetRef>& list) {
  }

  /**
   * @brief Flags the asset to be saved at a later time.
   */
  void SetUnsaved();

  /**
   * @brief Checks if the asset is in a saved state.
   * @return True if the asset is saved, false otherwise.
   */
  [[nodiscard]] bool Saved() const;

  /**
   * @brief Destructor for the asset.
   */
  ~IAsset() override;
};

}  // namespace evo_engine
