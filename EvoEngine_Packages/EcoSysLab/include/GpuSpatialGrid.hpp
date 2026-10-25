#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "GraphicsResources.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

// Stable, sparse world-cell addresses. Storage chunks are independent of spatial regions.
class GpuSpatialGrid {
 public:
  static constexpr uint32_t chunk_capacity = 4096;
  static constexpr uint32_t bucket_count = 262144;

  struct Coordinate {
    int x, y, z;
    bool operator==(const Coordinate& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct CoordinateHash {
    size_t operator()(const Coordinate& c) const;
  };
  struct Cell {
    glm::ivec4 coordinate_next{};
  };
  static_assert(sizeof(Cell) == 16);

  explicit GpuSpatialGrid(float cell_size = 1.0f, uint32_t value_stride = sizeof(glm::vec4) * 2)
      : cell_size_(cell_size), value_stride_(value_stride) {
  }
  bool RegisterBox(const glm::vec3& lower, const glm::vec3& upper);
  void Reset();
  [[nodiscard]] uint32_t GetCellCount() const {
    return static_cast<uint32_t>(lookup_.size());
  }
  [[nodiscard]] float GetCellSize() const {
    return cell_size_;
  }
  [[nodiscard]] const std::shared_ptr<Buffer>& GetHeads() const {
    return heads_;
  }
  [[nodiscard]] const std::shared_ptr<Buffer>& GetAddresses() const {
    return addresses_;
  }
  [[nodiscard]] uint32_t Find(const Coordinate& coordinate) const {
    const auto found = lookup_.find(coordinate);
    return found == lookup_.end() ? UINT32_MAX : found->second;
  }
  [[nodiscard]] const std::shared_ptr<Buffer>& GetValueChunk(uint32_t chunk) const {
    return value_chunks_[chunk];
  }

 private:
  struct Addresses {
    uint64_t cells = 0;
    uint64_t values = 0;
  };
  static_assert(sizeof(Addresses) == 16);
  uint32_t Hash(const Coordinate& coordinate) const;
  void Add(const Coordinate& coordinate);
  void Upload(uint32_t first_new_cell);

  float cell_size_;
  uint32_t value_stride_;
  std::unordered_map<Coordinate, uint32_t, CoordinateHash> lookup_;
  std::vector<uint32_t> head_indices_;
  std::vector<std::vector<Cell>> cells_;
  std::vector<std::shared_ptr<Buffer>> cell_chunks_;
  std::vector<std::shared_ptr<Buffer>> value_chunks_;
  std::shared_ptr<Buffer> heads_;
  std::shared_ptr<Buffer> addresses_;
  std::vector<std::pair<uint32_t, std::shared_ptr<Buffer>>> retired_buffers_;
};
}  // namespace eco_sys_lab_package
