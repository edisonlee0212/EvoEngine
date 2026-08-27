#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;
struct RayCameraHistoryResources;
class EVOENGINE_API RayTracingPipeline;

class EVOENGINE_API RayTracingCameraPass final {
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

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(
      const char* pass_name = RenderPassNames::ray_tracing_camera, CameraSettings::RayOutputSettings outputs = {});
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};

class EVOENGINE_API RayQueryCameraPass final {
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

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(CameraSettings::RayOutputSettings outputs = {});
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};

}  // namespace evo_engine
