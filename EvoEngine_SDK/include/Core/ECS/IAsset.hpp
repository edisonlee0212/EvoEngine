
#pragma once
#include "IHandle.hpp"
#include "ISerializable.hpp"
#include "Jobs.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace evo_engine {

/**
 * Forward declarations of other classes.
 */
class EVOENGINE_API EditorLayer;
class EVOENGINE_API AssetRef;
class EVOENGINE_API File;
class EVOENGINE_API Folder;
class EVOENGINE_API ProjectContentBrowserPanel;
class EVOENGINE_API Texture2D;

/**
 * @class StagedAssetLoadPayload
 * @brief CPU-side data produced by an async-safe asset load phase and consumed by the finalization phase.
 */
class StagedAssetLoadPayload {
 public:
  virtual ~StagedAssetLoadPayload() = default;
};

/**
 * @class IAsset
 * @brief Base class for managing assets in the evo_engine framework. Provides functionality for serialization,
 *        deserialization, and interactions with the asset's file system and the editor.
 */
class EVOENGINE_API IAsset : public ISerializable {
  struct PendingGpuWorkState {
    mutable std::mutex mutex;
    std::vector<JobHandle> handles;
  };

  std::weak_ptr<IAsset>
      self_; /**< Weak reference to the current IAsset instance. Used internally for managing self-references. */
  std::shared_ptr<PendingGpuWorkState> pending_gpu_work_state_ = std::make_shared<PendingGpuWorkState>();

 protected:
  /** @cond DOXYGEN_SHOULD_SKIP_THIS */
  friend class Resources;
  friend class EditorLayer;
  friend class AssetRegistry;
  friend class ProjectManager;
  friend class ProjectContentBrowserPanel;
  friend class File;
  friend class Folder;
  friend class AssetManager;
  friend class Serialization;
  /** @endcond */

  std::weak_ptr<File> file_record_; /**< Weak reference to the file metadata associated with this asset. */

  /**
   * @brief Gets a shared pointer to the current asset instance.
   * @return A shared pointer to this asset.
   */
  [[nodiscard]] std::shared_ptr<IAsset> GetSelf() const;

  /**
   * @brief Tracks asynchronous GPU work that must complete before this asset is fully ready.
   */
  void TrackPendingGpuWork(const JobHandle& handle);

  /**
   * @brief Takes tracked GPU work handles for readiness orchestration.
   */
  [[nodiscard]] std::vector<JobHandle> ConsumePendingGpuWorkHandles() const;

  /**
   * @brief Waits for tracked GPU work and clears consumed readiness handles.
   */
  void WaitForPendingGpuWork() const;

  bool saved_ = false;   /**< Indicates whether the asset is in a saved state. */
  uint32_t version_ = 0; /**< The version number of the asset. */

 public:
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
   * @brief Saves the current object state to a YAML emitter.
   *
   * @param name The name under which to save the object.
   * @param out The YAML emitter to output the serialized data.
   */
  void Save(const std::string& name, YAML::Emitter& out) const override;

  /**
   * @brief Loads the object state from a given YAML node.
   *
   * @param name The name under which the object data is stored.
   * @param in The YAML node containing the serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in) override;

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
