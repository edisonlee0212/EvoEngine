#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;
class SdfgiResources;

class EVOENGINE_API DeferredComputeLightingPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> lighting_descriptor_set;
    std::shared_ptr<DescriptorSet> raster_lighting_texture_descriptor_set;
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    int camera_index = -1;
    int directional_shadow_camera_index = -1;
    bool reflection_probe_capture = false;
    bool scene_camera = false;
    RecordCommands record_commands;
    std::shared_ptr<SdfgiResources> sdfgi_resources;
    std::shared_ptr<DescriptorSet> sdfgi_descriptor_set;
    uint32_t sdfgi_debug_view = 0;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(bool ambient_occlusion_enabled,
                                                             bool depth_pyramid_enabled);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
