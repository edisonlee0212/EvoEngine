#pragma once
#include <algorithm>
#include <cstring>
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
template <typename T>
std::shared_ptr<Buffer> MakeAttributeBuffer(const std::vector<T>& source, bool arrays) {
  if (!arrays)
    return MakeBuffer(source);
  static_assert(sizeof(T) % 16 == 0);
  constexpr size_t fields = sizeof(T) / 16;
  std::vector<glm::uvec4> packed(source.size() * fields);
  for (size_t i = 0; i < source.size(); ++i)
    for (size_t field = 0; field < fields; ++field)
      std::memcpy(&packed[field * source.size() + i], reinterpret_cast<const char*>(&source[i]) + field * 16, 16);
  return MakeBuffer(packed);
}
}  // namespace eco_sys_lab_package::tree_physics_buffers
