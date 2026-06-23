#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class Camera;
class DescriptorSet;
class DescriptorSetLayout;
class RayTracingPipeline;

class RayTracingCameraPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<RayTracingPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> ray_tracing_descriptor_set;
    int camera_index = -1;
    uint32_t frame_id = 0;
    RecordCommands record_commands;
    std::shared_ptr<DescriptorSetLayout> output_descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
