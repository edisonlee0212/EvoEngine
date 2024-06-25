#pragma once
#include "Jobs.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
/* Coordinate system

The voxel position is its center.
Each voxel is dx wide.

                                <-dx ->
                                -------------------------
                                |     |     |     |     |
                                |  x  |  x  |  x  |  x  |
                                |     |     |     |     |
                                -------------------------
                                   |     |     |     |
                                   |     |     |     |
X-Coordinate:   -- 0 --- 1 --- 2 --- 3 -----

The "min_bound_" stores the lower left corner of the lower left voxel.
I.e. for min_bound_ = (0, 0) and resolution_= (2, 2), and m_size = 1,
the voxel centers are at 0.5 and 1.5.

*/
template <typename VoxelData>
class VoxelGrid {
  std::vector<VoxelData> data_;
  glm::vec3 min_bound_ = glm::vec3(0.0f);
  float voxel_size_ = 1.0f;
  glm::ivec3 resolution_ = {0, 0, 0};

 public:
  void Initialize(float voxel_size, const glm::ivec3& resolution, const glm::vec3& min_bound,
                  const VoxelData& default_data = {});
  void Initialize(float voxel_size, const glm::vec3& min_bound, const glm::vec3& max_bound,
                  const VoxelData& default_data = {});

  void Resize(const glm::ivec3& diff_min, const glm::ivec3& diff_max);

  void Reset();
  void ShiftMinBound(const glm::vec3& offset);

  [[nodiscard]] size_t GetVoxelCount() const;
  [[nodiscard]] glm::ivec3 GetResolution() const;
  [[nodiscard]] glm::vec3 GetMinBound() const;
  [[nodiscard]] glm::vec3 GetMaxBound() const;
  [[nodiscard]] float GetVoxelSize() const;

  [[nodiscard]] VoxelData& Ref(int index);
  [[nodiscard]] const VoxelData& Peek(int index) const;
  [[nodiscard]] VoxelData& Ref(const glm::ivec3& coordinate);
  [[nodiscard]] const VoxelData& Peek(const glm::ivec3& coordinate) const;
  [[nodiscard]] VoxelData& Ref(const glm::vec3& position);
  [[nodiscard]] const VoxelData& Peek(const glm::vec3& position) const;

  [[nodiscard]] int GetIndex(const glm::ivec3& coordinate) const;
  [[nodiscard]] int GetIndex(const glm::vec3& position) const;
  [[nodiscard]] glm::ivec3 GetCoordinate(int index) const;
  [[nodiscard]] glm::ivec3 GetCoordinate(const glm::vec3& position) const;
  [[nodiscard]] glm::vec3 GetPosition(int index) const;
  [[nodiscard]] glm::vec3 GetPosition(const glm::ivec3& coordinate) const;

  void ForEach(const glm::vec3& min_bound, const glm::vec3& max_bound,
               const std::function<void(VoxelData& data)>& func);
  void ForEach(const glm::vec3& center, float radius, const std::function<void(VoxelData& data)>& func);
  [[nodiscard]] bool IsValid(const glm::vec3& position) const;

  [[nodiscard]] std::vector<VoxelData>& RefData();
};

template <typename VoxelData>
void VoxelGrid<VoxelData>::Initialize(const float voxel_size, const glm::ivec3& resolution, const glm::vec3& min_bound,
                                      const VoxelData& default_data) {
  resolution_ = resolution;
  voxel_size_ = voxel_size;
  min_bound_ = min_bound;
  auto num_voxels = resolution_.x * resolution_.y * resolution_.z;
  data_.resize(num_voxels);
  std::fill(data_.begin(), data_.end(), default_data);
}

template <typename VoxelData>
void VoxelGrid<VoxelData>::Initialize(const float voxel_size, const glm::vec3& min_bound, const glm::vec3& max_bound,
                                      const VoxelData& default_data) {
  const glm::vec3 regulated_min_bound = glm::floor(min_bound / voxel_size) * voxel_size;
  const glm::vec3 regulated_max_bound = glm::ceil(max_bound / voxel_size) * voxel_size;

  Initialize(voxel_size,
             glm::ivec3(glm::ceil((regulated_max_bound.x - regulated_min_bound.x) / voxel_size) + 1,
                        glm::ceil((regulated_max_bound.y - regulated_min_bound.y) / voxel_size) + 1,
                        glm::ceil((regulated_max_bound.z - regulated_min_bound.z) / voxel_size) + 1),
             regulated_min_bound, default_data);
}

template <typename VoxelData>
void VoxelGrid<VoxelData>::Resize(const glm::ivec3& diff_min, const glm::ivec3& diff_max) {
  const auto original_voxel_data = data_;
  const auto original_resolution = resolution_;
  const auto original_min_bound = min_bound_;
  Initialize(voxel_size_, resolution_ + diff_min + diff_max, min_bound_ - glm::vec3(diff_min) * voxel_size_, {});
  Jobs::RunParallelFor(original_voxel_data.size(), [&](unsigned i) {
    const auto original_coordinate = glm::ivec3(
        i % original_resolution.x, i % (original_resolution.x * original_resolution.y) / original_resolution.x,
        i / (original_resolution.x * original_resolution.y));
    const auto original_position =
        glm::vec3(original_min_bound.x + voxel_size_ / 2.0 + original_coordinate.x * voxel_size_,
                  original_min_bound.y + voxel_size_ / 2.0 + original_coordinate.y * voxel_size_,
                  original_min_bound.z + voxel_size_ / 2.0 + original_coordinate.z * voxel_size_);
    const auto target_index = GetIndex(original_position);
    if (target_index > 0 && target_index < data_.size()) {
      data_[target_index] = original_voxel_data[i];
    }
  });
}

template <typename VoxelData>
void VoxelGrid<VoxelData>::Reset() {
  std::fill(data_.begin(), data_.end(), VoxelData());
}

template <typename VoxelData>
void VoxelGrid<VoxelData>::ShiftMinBound(const glm::vec3& offset) {
  min_bound_ += offset;
}

template <typename VoxelData>
size_t VoxelGrid<VoxelData>::GetVoxelCount() const {
  return data_.size();
}

template <typename VoxelData>
glm::ivec3 VoxelGrid<VoxelData>::GetResolution() const {
  return resolution_;
}

template <typename VoxelData>
glm::vec3 VoxelGrid<VoxelData>::GetMinBound() const {
  return min_bound_;
}

template <typename VoxelData>
glm::vec3 VoxelGrid<VoxelData>::GetMaxBound() const {
  return min_bound_ + glm::vec3(resolution_) * voxel_size_;
}

template <typename VoxelData>
float VoxelGrid<VoxelData>::GetVoxelSize() const {
  return voxel_size_;
}

template <typename VoxelData>
VoxelData& VoxelGrid<VoxelData>::Ref(const int index) {
  return data_[index];
}

template <typename VoxelData>
const VoxelData& VoxelGrid<VoxelData>::Peek(const int index) const {
  return data_[index];
}

template <typename VoxelData>
VoxelData& VoxelGrid<VoxelData>::Ref(const glm::ivec3& coordinate) {
  return Ref(GetIndex(coordinate));
}

template <typename VoxelData>
const VoxelData& VoxelGrid<VoxelData>::Peek(const glm::ivec3& coordinate) const {
  return Peek(GetIndex(coordinate));
}

template <typename VoxelData>
VoxelData& VoxelGrid<VoxelData>::Ref(const glm::vec3& position) {
  return Ref(GetIndex(position));
}

template <typename VoxelData>
const VoxelData& VoxelGrid<VoxelData>::Peek(const glm::vec3& position) const {
  return Peek(GetIndex(position));
}

template <typename VoxelData>
int VoxelGrid<VoxelData>::GetIndex(const glm::ivec3& coordinate) const {
  return coordinate.x + coordinate.y * resolution_.x + coordinate.z * resolution_.x * resolution_.y;
}

template <typename VoxelData>
int VoxelGrid<VoxelData>::GetIndex(const glm::vec3& position) const {
  return GetIndex(GetCoordinate(position));
}

template <typename VoxelData>
glm::ivec3 VoxelGrid<VoxelData>::GetCoordinate(int index) const {
  return {index % resolution_.x, index % (resolution_.x * resolution_.y) / resolution_.x,
          index / (resolution_.x * resolution_.y)};
}

template <typename VoxelData>
glm::ivec3 VoxelGrid<VoxelData>::GetCoordinate(const glm::vec3& position) const {
  return {floor((position.x - min_bound_.x) / voxel_size_), floor((position.y - min_bound_.y) / voxel_size_),
          floor((position.z - min_bound_.z) / voxel_size_)};
}

template <typename VoxelData>
glm::vec3 VoxelGrid<VoxelData>::GetPosition(const glm::ivec3& coordinate) const {
  return {min_bound_.x + voxel_size_ / 2.0 + coordinate.x * voxel_size_,
          min_bound_.y + voxel_size_ / 2.0 + coordinate.y * voxel_size_,
          min_bound_.z + voxel_size_ / 2.0 + coordinate.z * voxel_size_};
}

template <typename VoxelData>
void VoxelGrid<VoxelData>::ForEach(const glm::vec3& min_bound, const glm::vec3& max_bound,
                                   const std::function<void(VoxelData& data)>& func) {
  const auto actual_min_bound = min_bound - min_bound_;
  const auto actual_max_bound = max_bound - min_bound_;
  const auto start = glm::ivec3(glm::floor(actual_min_bound / glm::vec3(voxel_size_)));
  const auto end = glm::ivec3(glm::ceil(actual_max_bound / glm::vec3(voxel_size_)));
  for (int i = start.x; i <= end.x; i++) {
    for (int j = start.y; j <= end.y; j++) {
      for (int k = start.z; k <= end.z; k++) {
        if (i < 0 || i >= resolution_.x || j < 0 || j >= resolution_.y || k < 0 || k >= resolution_.z)
          continue;
        auto index = GetIndex(glm::ivec3(i, j, k));
        func(Ref(index));
      }
    }
  }
}

template <typename VoxelData>
void VoxelGrid<VoxelData>::ForEach(const glm::vec3& center, float radius,
                                   const std::function<void(VoxelData& data)>& func) {
  const auto actual_center = center - min_bound_;
  const auto actual_min_bound = actual_center - glm::vec3(radius);
  const auto actual_max_bound = actual_center + glm::vec3(radius);
  const auto start = glm::ivec3(glm::floor(actual_min_bound / glm::vec3(voxel_size_)));
  const auto end = glm::ivec3(glm::ceil(actual_max_bound / glm::vec3(voxel_size_)));
  for (int i = start.x; i <= end.x; i++) {
    for (int j = start.y; j <= end.y; j++) {
      for (int k = start.z; k <= end.z; k++) {
        if (i < 0 || i >= resolution_.x || j < 0 || j >= resolution_.y || k < 0 || k >= resolution_.z)
          continue;
        auto index = GetIndex(glm::ivec3(i, j, k));
        func(Ref(index));
      }
    }
  }
}

template <typename VoxelData>
bool VoxelGrid<VoxelData>::IsValid(const glm::vec3& position) const {
  if (const auto max_bound = min_bound_ + voxel_size_ * glm::vec3(resolution_);
      position.x < min_bound_.x || position.y < min_bound_.y || position.z < min_bound_.z ||
      position.x >= max_bound.x || position.y >= max_bound.y || position.z >= max_bound.z)
    return false;
  return true;
}

template <typename VoxelData>
std::vector<VoxelData>& VoxelGrid<VoxelData>::RefData() {
  return data_;
}

template <typename VoxelData>
glm::vec3 VoxelGrid<VoxelData>::GetPosition(const int index) const {
  return GetPosition(GetCoordinate(index));
}
}  // namespace eco_sys_lab
