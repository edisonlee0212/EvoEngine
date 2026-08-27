#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;
class EVOENGINE_API GraphicsPipeline;
class EVOENGINE_API RenderInstanceStorage;

class EVOENGINE_API MotionCoveragePass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<RenderInstanceStorage> render_instances;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<GraphicsPipeline> skinned_pipeline;
    std::shared_ptr<GraphicsPipeline> transparent_pipeline;
    std::shared_ptr<DescriptorSetLayout> motion_coverage_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    int camera_index = 0;
    bool wire_frame = false;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
