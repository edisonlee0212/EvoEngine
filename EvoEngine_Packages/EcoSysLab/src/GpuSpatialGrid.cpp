#include "GpuSpatialGrid.hpp"

#include <algorithm>
#include <cmath>

#include "Platform.hpp"

using namespace eco_sys_lab_package;

size_t GpuSpatialGrid::CoordinateHash::operator()(const Coordinate& c) const {
  return static_cast<size_t>(static_cast<uint32_t>(c.x) * 73856093u ^ static_cast<uint32_t>(c.y) * 19349663u ^
                             static_cast<uint32_t>(c.z) * 83492791u);
}

uint32_t GpuSpatialGrid::Hash(const Coordinate& coordinate) const {
  return static_cast<uint32_t>(CoordinateHash{}(coordinate)) % bucket_count;
}

void GpuSpatialGrid::Add(const Coordinate& coordinate) {
  if (lookup_.find(coordinate) != lookup_.end())
    return;
  if (head_indices_.empty())
    head_indices_.assign(bucket_count, UINT32_MAX);
  const uint32_t index = GetCellCount();
  const uint32_t bucket = Hash(coordinate);
  lookup_.emplace(coordinate, index);
  if (index % chunk_capacity == 0)
    cells_.emplace_back();
  cells_.back().push_back(
      {glm::ivec4(coordinate.x, coordinate.y, coordinate.z, static_cast<int32_t>(head_indices_[bucket]))});
  head_indices_[bucket] = index;
}

bool GpuSpatialGrid::RegisterBox(const glm::vec3& lower, const glm::vec3& upper) {
  const Coordinate first{static_cast<int>(std::floor(lower.x / cell_size_)),
                         static_cast<int>(std::floor(lower.y / cell_size_)),
                         static_cast<int>(std::floor(lower.z / cell_size_))};
  const Coordinate last{static_cast<int>(std::floor(upper.x / cell_size_)),
                        static_cast<int>(std::floor(upper.y / cell_size_)),
                        static_cast<int>(std::floor(upper.z / cell_size_))};
  const uint32_t before = GetCellCount();
  for (int z = first.z; z <= last.z; ++z)
    for (int y = first.y; y <= last.y; ++y)
      for (int x = first.x; x <= last.x; ++x)
        Add({x, y, z});
  if (GetCellCount() == before)
    return false;
  Upload(before);
  return true;
}

void GpuSpatialGrid::Upload(uint32_t first_new_cell) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.usage =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  info.size = bucket_count * sizeof(uint32_t);
  auto next_heads = std::make_shared<Buffer>(info, allocation);
  next_heads->UploadVector(head_indices_);
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (i == cell_chunks_.size()) {
      info.size = chunk_capacity * sizeof(Cell);
      cell_chunks_.push_back(std::make_shared<Buffer>(info, allocation));
      info.size = static_cast<VkDeviceSize>(chunk_capacity) * value_stride_;
      value_chunks_.push_back(std::make_shared<Buffer>(info, allocation));
      value_chunks_.back()->UploadVector(std::vector<std::byte>(info.size));
    }
    const size_t first_slot = i == first_new_cell / chunk_capacity ? first_new_cell % chunk_capacity : 0;
    if (i >= first_new_cell / chunk_capacity)
      cell_chunks_[i]->UploadSubData((cells_[i].size() - first_slot) * sizeof(Cell), cells_[i].data() + first_slot,
                                     first_slot * sizeof(Cell));
  }
  std::vector<Addresses> entries;
  entries.reserve(cell_chunks_.size());
  for (size_t i = 0; i < cell_chunks_.size(); ++i)
    entries.push_back({cell_chunks_[i]->GetDeviceAddress(), value_chunks_[i]->GetDeviceAddress()});
  info.size = entries.size() * sizeof(Addresses);
  auto next = std::make_shared<Buffer>(info, allocation);
  next->UploadVector(entries);
  if (heads_)
    retired_buffers_.push_back({Platform::GetFrameCount(), heads_});
  if (addresses_)
    retired_buffers_.push_back({Platform::GetFrameCount(), addresses_});
  heads_ = std::move(next_heads);
  addresses_ = std::move(next);
  const auto frame = Platform::GetFrameCount();
  retired_buffers_.erase(std::remove_if(retired_buffers_.begin(), retired_buffers_.end(),
                                        [&](const auto& retired) {
                                          return frame - retired.first > Platform::GetMaxFramesInFlight() + 1;
                                        }),
                         retired_buffers_.end());
}

void GpuSpatialGrid::Reset() {
  lookup_.clear();
  head_indices_.clear();
  cells_.clear();
  cell_chunks_.clear();
  value_chunks_.clear();
  heads_.reset();
  addresses_.reset();
  retired_buffers_.clear();
}
