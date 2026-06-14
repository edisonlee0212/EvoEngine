
#pragma once
#include "IAsset.hpp"
#include "nlohmann/json.hpp"

namespace evo_engine {
void RegisterJsonHandlers();

/**
 * @class Json
 * @brief A class representing a JSON asset that extends the IAsset interface.
 */
class Json : public IAsset {
 protected:
  /**
   * @brief Saves the asset to the specified path.
   * @param path The file system path where the asset should be saved.
   * @return True if the save operation was successful, false otherwise.
   */
  bool SaveInternal(const std::filesystem::path& path) const;

  /**
   * @brief Loads the asset from the specified path.
   * @param path The file system path from where the asset should be loaded.
   * @return True if the load operation was successful, false otherwise.
   */
  bool LoadInternal(const std::filesystem::path& path);

  /**
   * @brief JSON files can be parsed on the asset-IO executor and applied later.
   */
  [[nodiscard]] bool SupportsStagedLoading() const;

  /**
   * @brief Parses the JSON file into a staged payload.
   */
  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path& path) const;

  /**
   * @brief Applies a parsed JSON staged payload.
   */
  bool ApplyStagedPayloadInternal(const std::filesystem::path& path,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload);

 public:
  /// @brief JSON object representing the data of the asset.
  nlohmann::json m_json;
};

}  // namespace evo_engine
