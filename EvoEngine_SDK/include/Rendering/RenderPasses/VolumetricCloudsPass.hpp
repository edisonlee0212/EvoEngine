#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class Camera;

class VolumetricCloudsPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateRasterDescriptor(const char* dependency);
  [[nodiscard]] static RenderPassDescriptor CreateRayTracingDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
