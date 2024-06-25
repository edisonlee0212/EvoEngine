#pragma once
#include "CubeVolume.hpp"
#include "Skeleton.hpp"
#include "VoxelGrid.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class RadialBoundingVolume;

struct OccupancyGridSettings {};

struct TreeOccupancyGridMarker {
  glm::vec3 position = glm::vec3(0.0f);
  SkeletonNodeHandle node_handle = -1;
};

struct TreeOccupancyGridVoxelData {
  std::vector<TreeOccupancyGridMarker> markers;
};

struct TreeOccupancyGridBasicData {
  bool occupied = false;
};

class TreeOccupancyGrid {
  VoxelGrid<TreeOccupancyGridVoxelData> occupancy_grid_{};
  float removal_distance_factor_ = 2;
  float theta_ = 90.0f;
  float detection_distance_factor_ = 4;
  float internode_length_ = 1.0f;
  size_t markers_per_voxel_ = 5;

 public:
  void ResetMarkers();
  [[nodiscard]] float GetRemovalDistanceFactor() const;
  [[nodiscard]] float GetTheta() const;
  [[nodiscard]] float GetDetectionDistanceFactor() const;
  [[nodiscard]] float GetInternodeLength() const;
  [[nodiscard]] size_t GetMarkersPerVoxel() const;

  void Initialize(const glm::vec3& min, const glm::vec3& max, float internode_length, float removal_distance_factor = 2.0f,
                  float theta = 90.0f, float detection_distance_factor = 4.0f, size_t markers_per_voxel = 1);
  void Resize(const glm::vec3& min, const glm::vec3& max);
  void Initialize(const VoxelGrid<TreeOccupancyGridBasicData>& src_grid, const glm::vec3& min, const glm::vec3& max,
                  float internode_length, float removal_distance_factor = 2.0f, float theta = 90.0f,
                  float detection_distance_factor = 4.0f, size_t markers_per_voxel = 1);
  void Initialize(const std::shared_ptr<RadialBoundingVolume>& src_radial_bounding_volume, const glm::vec3& min,
                  const glm::vec3& max, float internode_length, float removal_distance_factor = 2.0f, float theta = 90.0f,
                  float detection_distance_factor = 4.0f, size_t markers_per_voxel = 1);
  [[nodiscard]] VoxelGrid<TreeOccupancyGridVoxelData>& RefGrid();
  [[nodiscard]] glm::vec3 GetMin() const;
  [[nodiscard]] glm::vec3 GetMax() const;

  void InsertObstacle(const GlobalTransform& global_transform, const std::shared_ptr<CubeVolume>& cube_volume);
};
}  // namespace eco_sys_lab