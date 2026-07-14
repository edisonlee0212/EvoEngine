#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class Camera;
class ComputePipeline;
class DescriptorSet;
class DescriptorSetLayout;
struct RayCameraHistoryResources;
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
    RecordCommands record_commands;
    std::shared_ptr<DescriptorSet> output_descriptor_set;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    RayCameraHistoryResources* history_resources = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};

class RayQueryCameraPass final {
 public:
  using RecordCommands = RayTracingCameraPass::RecordCommands;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> ray_tracing_descriptor_set;
    int camera_index = -1;
    RecordCommands record_commands;
    std::shared_ptr<DescriptorSet> output_descriptor_set;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    RayCameraHistoryResources* history_resources = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
