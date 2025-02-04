
#pragma once
#include "Application.hpp"
#include "Camera.hpp"
#include "Mesh.hpp"

namespace evo_engine {

/**
 * @class PointCloud
 * @brief Represents a point cloud asset with associated properties and functionality for loading, saving, and
 * processing.
 */
class PointCloud : public IAsset {
  glm::dvec3 min_ = glm::dvec3(FLT_MAX);   ///< Minimum bound of the point cloud.
  glm::dvec3 max_ = glm::dvec3(-FLT_MAX);  ///< Maximum bound of the point cloud.

 protected:
  /**
   * @brief Internal method to load the point cloud from a file.
   * @param path The path to the file to load the point cloud from.
   * @return True if the load is successful, otherwise false.
   */
  bool LoadInternal(const std::filesystem::path& path) override;

  /**
   * @brief Internal method to save the point cloud to a file.
   * @param path The path to the file to save the point cloud to.
   * @return True if the save is successful, otherwise false.
   */
  bool SaveInternal(const std::filesystem::path& path) const override;

 public:
  /**
   * @struct PointCloudSaveSettings
   * @brief Settings for saving a point cloud.
   */
  struct PointCloudSaveSettings {
    bool binary = true;             ///< Indicates whether to save the file in binary format.
    bool double_precision = false;  ///< Indicates whether to save positions in double precision.
  };

  /**
   * @struct PointCloudLoadSettings
   * @brief Settings for loading a point cloud.
   */
  struct PointCloudLoadSettings {
    bool binary = true;  ///< Indicates whether to load the file in binary format.
  };

  glm::dvec3 offset;                  ///< Offset applied to the positions in the point cloud.
  bool has_positions = false;         ///< True if the point cloud contains position data.
  bool has_normals = false;           ///< True if the point cloud contains normal data.
  bool has_colors = false;            ///< True if the point cloud contains color data.
  std::vector<glm::dvec3> positions;  ///< Collection of positions in the point cloud.
  std::vector<glm::dvec3> normals;    ///< Collection of normals in the point cloud.
  std::vector<glm::vec4> colors;      ///< Collection of colors in the point cloud.
  float point_size = 0.01f;           ///< Size of each point in the point cloud.
  float compress_factor = 0.01f;      ///< Factor used for compression.

  /**
   * @brief Executes custom logic during the creation of the point cloud asset.
   */
  void OnCreate() override;

  /**
   * @brief Loads the point cloud using the given load settings and file path.
   * @param settings The settings to use when loading.
   * @param path The path to the file containing the point cloud data.
   * @return True if the load is successful, otherwise false.
   */
  bool Load(const PointCloudLoadSettings& settings, const std::filesystem::path& path);

  /**
   * @brief Saves the point cloud using the given save settings and file path.
   * @param settings The settings to use when saving.
   * @param path The path to save the point cloud data to.
   * @return True if the save is successful, otherwise false.
   */
  bool Save(const PointCloudSaveSettings& settings, const std::filesystem::path& path) const;

  /**
   * @brief Inspects the point cloud's properties using an editor layer.
   * @param editor_layer The editor layer to use for inspection.
   * @return True if inspection is successful, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Compresses the given collection of points to reduce data size.
   * @param points The points to compress.
   */
  void Compress(std::vector<glm::dvec3>& points);

  /**
   * @brief Applies the compressed version of the point cloud data.
   */
  void ApplyCompressed();

  /**
   * @brief Restores the original point cloud data.
   */
  void ApplyOriginal() const;

  /**
   * @brief Recalculates the bounding box of the point cloud.
   */
  void RecalculateBoundingBox();

  /**
   * @brief Crops the given collection of points to fit within a bounding box.
   * @param points The points to crop.
   * @param min The minimum bounds of the bounding box.
   * @param max The maximum bounds of the bounding box.
   */
  static void Crop(std::vector<glm::dvec3>& points, const glm::dvec3& min, const glm::dvec3& max);

  /**
   * @brief Serializes the point cloud's state to a YAML emitter.
   * @param out The YAML emitter to write the serialized data to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the point cloud's state from a YAML node.
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace evo_engine
