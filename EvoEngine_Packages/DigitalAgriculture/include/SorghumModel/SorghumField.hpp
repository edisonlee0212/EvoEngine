
#pragma once

namespace digital_agriculture_package {
using namespace evo_engine;

/**
 * @class SorghumGrid
 * @brief Represents a grid structure for generating sorghum plants in a field.
 */
class SorghumGrid {
 public:
  /** Distance between grid points */
  glm::vec2 grid_distance = glm::vec2(1.f);

  /** Mean offset for positional variation. */
  float position_offset_mean = 0.f;

  /** Variance for positional offset. */
  float position_offset_variance = 0.f;

  /** Variance for rotation along the XZ plane. */
  float rotation_variance_xz = 0.f;

  /** Variance for rotation along the Y-axis. */
  float rotation_variance_y = 0.f;

  /** Number of grid points. */
  glm::ivec2 grid_size = glm::ivec2(10);

  /**
   * @brief Generates transformation matrices for the sorghum field.
   * @param matrices_list The output list containing the generated transformation matrices.
   */
  void GenerateField(std::vector<glm::mat4>& matrices_list) const;
};

/**
 * @class SorghumField
 * @brief Represents an asset containing a planted sorghum field.
 */
class SorghumField : public IAsset {
  friend class SorghumLayer;

 public:
  /** Maximum number of sorghum plants in the field. */
  int size_limit = 2000;

  /** Base size of the sorghum plants. */
  float sorghum_size = 1.0f;

  /** List of asset references and their transformation matrices. */
  std::vector<std::pair<AssetRef, glm::mat4>> matrices;

  /**
   * @brief Instantiates the sorghum field using a base seed.
   * @param base_seed The base seed for randomization (default is 0).
   * @return The created entity representing the field.
   */
  Entity InstantiateField(uint32_t base_seed = 0) const;

  /**
   * @brief Generates a thumbnail texture for the sorghum field.
   * @return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();

  /**
   * @brief Collects all asset references used in this field.
   * @param list The output list of asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);
};

}  // namespace digital_agriculture_package
