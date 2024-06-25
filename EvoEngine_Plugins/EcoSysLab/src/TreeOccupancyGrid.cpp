#include "TreeOccupancyGrid.hpp"

#include "RadialBoundingVolume.hpp"
using namespace eco_sys_lab;

void TreeOccupancyGrid::ResetMarkers() {
  Jobs::RunParallelFor(occupancy_grid_.GetVoxelCount(), [&](unsigned i) {
    auto& voxel_data = occupancy_grid_.Ref(static_cast<int>(i));

    for (auto& marker : voxel_data.markers) {
      marker.node_handle = -1;
    }
  });
}

float TreeOccupancyGrid::GetRemovalDistanceFactor() const {
  return removal_distance_factor_;
}

float TreeOccupancyGrid::GetTheta() const {
  return theta_;
}

float TreeOccupancyGrid::GetDetectionDistanceFactor() const {
  return detection_distance_factor_;
}

float TreeOccupancyGrid::GetInternodeLength() const {
  return internode_length_;
}

size_t TreeOccupancyGrid::GetMarkersPerVoxel() const {
  return markers_per_voxel_;
}

void TreeOccupancyGrid::Initialize(const glm::vec3& min, const glm::vec3& max, const float internode_length,
                                   const float removal_distance_factor, const float theta,
                                   const float detection_distance_factor, size_t markers_per_voxel) {
  removal_distance_factor_ = removal_distance_factor;
  detection_distance_factor_ = detection_distance_factor;
  theta_ = theta;
  internode_length_ = internode_length;
  markers_per_voxel_ = markers_per_voxel;
  occupancy_grid_.Initialize(removal_distance_factor_ * internode_length, min, max, {});
  const auto voxel_size = occupancy_grid_.GetVoxelSize();
  Jobs::RunParallelFor(occupancy_grid_.GetVoxelCount(), [&](unsigned i) {
    auto& voxel_data = occupancy_grid_.Ref(static_cast<int>(i));
    for (int v = 0; v < markers_per_voxel_; v++) {
      auto& new_marker = voxel_data.markers.emplace_back();
      new_marker.position =
          occupancy_grid_.GetPosition(i) + glm::linearRand(-glm::vec3(voxel_size * 0.5f), glm::vec3(voxel_size * 0.5f));
    }
  });
}

void TreeOccupancyGrid::Resize(const glm::vec3& min, const glm::vec3& max) {
  const auto voxel_size = occupancy_grid_.GetVoxelSize();
  const auto diff_min =
      glm::floor((min - occupancy_grid_.GetMinBound() - detection_distance_factor_ * internode_length_) / voxel_size);
  const auto diff_max =
      glm::ceil((max - occupancy_grid_.GetMaxBound() + detection_distance_factor_ * internode_length_) / voxel_size);
  occupancy_grid_.Resize(-diff_min, diff_max);
  const auto new_resolution = occupancy_grid_.GetResolution();
  Jobs::RunParallelFor(occupancy_grid_.GetVoxelCount(), [&](unsigned i) {
    const auto coordinate = occupancy_grid_.GetCoordinate(i);

    if (coordinate.x < -diff_min.x || coordinate.y < -diff_min.y || coordinate.z < -diff_min.z ||
        coordinate.x >= new_resolution.x - diff_max.x || coordinate.y >= new_resolution.y - diff_max.y ||
        coordinate.z >= new_resolution.z - diff_max.z) {
      auto& voxel_data = occupancy_grid_.Ref(static_cast<int>(i));
      for (int v = 0; v < markers_per_voxel_; v++) {
        auto& new_marker = voxel_data.markers.emplace_back();
        new_marker.position =
            occupancy_grid_.GetPosition(i) + glm::linearRand(-glm::vec3(voxel_size * 0.5f), glm::vec3(voxel_size * 0.5f));
      }
    }
  });
}

void TreeOccupancyGrid::Initialize(const VoxelGrid<TreeOccupancyGridBasicData>& src_grid, const glm::vec3& min,
                                   const glm::vec3& max, const float internode_length, const float removal_distance_factor,
                                   const float theta, const float detection_distance_factor,
                                   const size_t markers_per_voxel) {
  removal_distance_factor_ = removal_distance_factor;
  detection_distance_factor_ = detection_distance_factor;
  theta_ = theta;
  internode_length_ = internode_length;
  markers_per_voxel_ = markers_per_voxel;
  occupancy_grid_.Initialize(removal_distance_factor_ * internode_length, min, max, {});
  const auto voxel_size = occupancy_grid_.GetVoxelSize();

  Jobs::RunParallelFor(occupancy_grid_.GetVoxelCount(), [&](unsigned i) {
    const glm::vec3 normalized_position =
        glm::vec3(occupancy_grid_.GetCoordinate(i)) / glm::vec3(occupancy_grid_.GetResolution()) -
        glm::vec3(0.5f, 0.0f, 0.5f);
    const auto src_grid_size = src_grid.GetMaxBound() - src_grid.GetMinBound();

    if (const auto src_grid_position = normalized_position * src_grid_size; (src_grid.IsValid(src_grid_position) && src_grid.Peek(src_grid.GetIndex(src_grid_position)).occupied) ||
                                                                          (normalized_position.y < 0.8f && glm::length(glm::vec2(normalized_position.x, normalized_position.z)) < 0.02f)) {
      auto& voxel_data = occupancy_grid_.Ref(static_cast<int>(i));
      for (int v = 0; v < markers_per_voxel_; v++) {
        auto& new_marker = voxel_data.markers.emplace_back();
        new_marker.position =
            occupancy_grid_.GetPosition(i) + glm::linearRand(-glm::vec3(voxel_size * 0.5f), glm::vec3(voxel_size * 0.5f));
      }
    }
  });
}

void TreeOccupancyGrid::Initialize(const std::shared_ptr<RadialBoundingVolume>& src_radial_bounding_volume,
                                   const glm::vec3& min, const glm::vec3& max, const float internode_length,
                                   const float removal_distance_factor, const float theta, const float detection_distance_factor,
                                   const size_t markers_per_voxel) {
  removal_distance_factor_ = removal_distance_factor;
  detection_distance_factor_ = detection_distance_factor;
  theta_ = theta;
  internode_length_ = internode_length;
  markers_per_voxel_ = markers_per_voxel;
  occupancy_grid_.Initialize(removal_distance_factor_ * internode_length, min, max, {});
  const auto voxel_size = occupancy_grid_.GetVoxelSize();

  Jobs::RunParallelFor(occupancy_grid_.GetVoxelCount(), [&](unsigned i) {
    if (src_radial_bounding_volume->InVolume(occupancy_grid_.GetPosition(i))) {
      auto& voxel_data = occupancy_grid_.Ref(static_cast<int>(i));
      for (int v = 0; v < markers_per_voxel_; v++) {
        auto& new_marker = voxel_data.markers.emplace_back();
        new_marker.position =
            occupancy_grid_.GetPosition(i) + glm::linearRand(-glm::vec3(voxel_size * 0.5f), glm::vec3(voxel_size * 0.5f));
      }
    }
  });
}

VoxelGrid<TreeOccupancyGridVoxelData>& TreeOccupancyGrid::RefGrid() {
  return occupancy_grid_;
}

glm::vec3 TreeOccupancyGrid::GetMin() const {
  return occupancy_grid_.GetMinBound();
}

glm::vec3 TreeOccupancyGrid::GetMax() const {
  return occupancy_grid_.GetMaxBound();
}

void TreeOccupancyGrid::InsertObstacle(const GlobalTransform& global_transform,
                                       const std::shared_ptr<CubeVolume>& cube_volume) {
  Jobs::RunParallelFor(occupancy_grid_.GetVoxelCount(), [&](unsigned i) {
    const auto center = occupancy_grid_.GetPosition(i);
    if (cube_volume->InVolume(global_transform, center)) {
      occupancy_grid_.Ref(i).markers.clear();
    }
  });
}
