#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;

class EVOENGINE_API DeferredMaterialResolvePass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    int camera_index = -1;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(bool ambient_occlusion_enabled,
                                                             bool depth_pyramid_enabled);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
