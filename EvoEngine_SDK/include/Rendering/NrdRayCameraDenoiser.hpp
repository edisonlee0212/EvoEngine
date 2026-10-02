#pragma once

#include <cstdint>
#include <memory>

#include <vulkan/vulkan.h>

namespace evo_engine {
struct CameraInfoBlock;
struct RayCameraHistoryResources;

class NrdRayCameraDenoiser final {
 public:
  NrdRayCameraDenoiser();
  ~NrdRayCameraDenoiser();

  bool Initialize(uint32_t width, uint32_t height);
  bool Denoise(VkCommandBuffer command_buffer, const CameraInfoBlock& camera, const RayCameraHistoryResources& history,
               bool reset_history);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace evo_engine
