#pragma once
#include <algorithm>
#include "GpuTreePhysicsState.hpp"
namespace eco_sys_lab_package::tree_physics_buffers {
inline std::shared_ptr<Buffer> Allocate(size_t bytes, VkBufferUsageFlags extra_usage = 0) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = std::max<size_t>(16, bytes);
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
               VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | extra_usage;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(info, allocation);
}
template <typename T>
std::shared_ptr<Buffer> MakeBuffer(const std::vector<T>& source) {
  auto buffer = Allocate(std::max<size_t>(1, source.size()) * sizeof(T));
  if (source.empty())
    buffer->UploadVector(std::vector<T>(1));
  else
    buffer->UploadVector(source);
  return buffer;
}
}  // namespace eco_sys_lab_package::tree_physics_buffers
