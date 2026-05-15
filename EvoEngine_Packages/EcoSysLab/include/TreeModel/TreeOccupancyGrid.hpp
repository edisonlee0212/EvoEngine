#pragma once
#include "CubeVolume.hpp"
#include "Skeleton.hpp"
#include "VoxelGrid.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class RadialBoundingVolume
 * @brief Forward declaration of the RadialBoundingVolume class.
 */
class RadialBoundingVolume;

/**
 * @struct OccupancyGridSettings
 * @brief Structure for occupancy grid settings.
 */
struct OccupancyGridSettings {};

/**
 * @struct TreeOccupancyGridMarker
 * @brief Represents a marker in the tree occupancy grid.
 */
struct TreeOccupancyGridMarker {
  glm::vec3 position = glm::vec3(0.0f);  ///< Position of the marker.
  SkeletonNodeHandle node_handle = -1;   ///< Associated skeleton node handle.
};

/**
 * @struct TreeOccupancyGridVoxelData
 * @brief Structure containing occupancy markers for a voxel.
 */
struct TreeOccupancyGridVoxelData {
  std::vector<TreeOccupancyGridMarker> markers;  ///< Markers in a voxel.
};

/**
 * @struct TreeOccupancyGridBasicData
 * @brief Basic occupancy data for a voxel.
 */
struct TreeOccupancyGridBasicData {
  bool occupied = false;  ///< Indicates if the voxel is occupied.
};

/**
 * @class TreeOccupancyGrid
 * @brief Represents an occupancy grid for tree structures.
 */
class TreeOccupancyGrid {
  VoxelGrid<TreeOccupancyGridVoxelData> occupancy_grid_{};  ///< Voxel grid holding occupancy data.
  float removal_distance_factor_ = 2;                       ///< Factor used for determining the removal distance.
  float theta_ = 90.0f;                                     ///< Angle threshold in degrees.
  float detection_distance_factor_ = 4;                     ///< Factor used for detection distance.
  float internode_length_ = 1.0f;                           ///< Length of the internodes.
  size_t markers_per_voxel_ = 5;                            ///< Number of markers per voxel.

 public:
  /**
   * @brief Resets the markers in the occupancy grid.
   */
  void ResetMarkers();

  /**
   * @brief Gets the removal distance factor.
   * @return The removal distance factor.
   */
  [[nodiscard]] float GetRemovalDistanceFactor() const;

  /**
   * @brief Gets the theta value.
   * @return The theta value in degrees.
   */
  [[nodiscard]] float GetTheta() const;

  /**
   * @brief Gets the detection distance factor.
   * @return The detection distance factor.
   */
  [[nodiscard]] float GetDetectionDistanceFactor() const;

  /**
   * @brief Gets the internode length.
   * @return The internode length.
   */
  [[nodiscard]] float GetInternodeLength() const;

  /**
   * @brief Gets the number of markers per voxel.
   * @return The number of markers per voxel.
   */
  [[nodiscard]] size_t GetMarkersPerVoxel() const;

  /**
   * @brief Initializes the occupancy grid.
   * @param min The minimum bounds of the grid.
   * @param max The maximum bounds of the grid.
   * @param internode_length The length of internodes.
   * @param removal_distance_factor Factor for removal distance.
   * @param theta The angle threshold.
   * @param detection_distance_factor Factor for detection distance.
   * @param markers_per_voxel The number of markers per voxel.
   */
  void Initialize(const glm::vec3& min, const glm::vec3& max, float internode_length,
                  float removal_distance_factor = 2.0f, float theta = 90.0f, float detection_distance_factor = 4.0f,
                  size_t markers_per_voxel = 1);

  /**
   * @brief Resizes the occupancy grid.
   * @param min The new minimum bounds of the grid.
   * @param max The new maximum bounds of the grid.
   */
  void Resize(const glm::vec3& min, const glm::vec3& max);

  /**
   * @brief Initializes the occupancy grid using a source voxel grid.
   * @param src_grid The source occupancy grid.
   * @param min The minimum bounds of the grid.
   * @param max The maximum bounds of the grid.
   * @param internode_length The length of internodes.
   * @param removal_distance_factor Factor for removal distance.
   * @param theta The angle threshold.
   * @param detection_distance_factor Factor for detection distance.
   * @param markers_per_voxel The number of markers per voxel.
   */
  void Initialize(const VoxelGrid<TreeOccupancyGridBasicData>& src_grid, const glm::vec3& min, const glm::vec3& max,
                  float internode_length, float removal_distance_factor = 2.0f, float theta = 90.0f,
                  float detection_distance_factor = 4.0f, size_t markers_per_voxel = 1);

  /**
   * @brief Initializes the occupancy grid using a radial bounding volume.
   * @param src_radial_bounding_volume The source radial bounding volume.
   * @param min The minimum bounds of the grid.
   * @param max The maximum bounds of the grid.
   * @param internode_length The length of internodes.
   * @param removal_distance_factor Factor for removal distance.
   * @param theta The angle threshold.
   * @param detection_distance_factor Factor for detection distance.
   * @param markers_per_voxel The number of markers per voxel.
   */
  void Initialize(const std::shared_ptr<RadialBoundingVolume>& src_radial_bounding_volume, const glm::vec3& min,
                  const glm::vec3& max, float internode_length, float removal_distance_factor = 2.0f,
                  float theta = 90.0f, float detection_distance_factor = 4.0f, size_t markers_per_voxel = 1);

  /**
   * @brief Gets a reference to the occupancy grid.
   * @return A reference to the voxel grid.
   */
  [[nodiscard]] VoxelGrid<TreeOccupancyGridVoxelData>& RefGrid();

  /**
   * @brief Gets the minimum bounds of the occupancy grid.
   * @return The minimum bounds as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 GetMin() const;

  /**
   * @brief Gets the maximum bounds of the occupancy grid.
   * @return The maximum bounds as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 GetMax() const;

  /**
   * @brief Inserts an obstacle into the occupancy grid.
   * @param global_transform The global transform of the obstacle.
   * @param cube_volume The volume representing the obstacle.
   */
  void InsertObstacle(const GlobalTransform& global_transform, const std::shared_ptr<CubeVolume>& cube_volume);
};
}  // namespace eco_sys_lab_package