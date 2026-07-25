#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class Camera;
class DescriptorSet;
class GraphicsPipeline;

class DdgiGatherTimingPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<GraphicsPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> lighting_descriptor_set;
    std::shared_ptr<DescriptorSet> raster_lighting_texture_descriptor_set;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    int camera_index = -1;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
