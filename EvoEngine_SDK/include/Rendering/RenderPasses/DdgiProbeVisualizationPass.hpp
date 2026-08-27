#pragma once
#include "RenderGraph.hpp"

#include <functional>
#include <glm/glm.hpp>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API Buffer;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;
class EVOENGINE_API GraphicsPipeline;
class EVOENGINE_API Image;
class EVOENGINE_API Sampler;

struct DdgiProbeVisualizationPushConstant {
  glm::uvec4 camera_selected_mode = glm::uvec4(0);
  glm::vec4 radius_intensity_alpha_selected_scale = glm::vec4(0.08f, 1.0f, 0.95f, 2.0f);
};

class EVOENGINE_API DdgiProbeVisualizationPass final {
 public:
  struct Parameters {
    std::shared_ptr<GraphicsPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    std::shared_ptr<Sampler> atlas_sampler;
    std::shared_ptr<Buffer> probe_metadata_buffer;
    std::shared_ptr<Buffer> probe_state_buffer;
    std::shared_ptr<Image> irradiance_atlas;
    std::shared_ptr<Camera> camera;
    uint32_t probe_count = 0;
    bool depth_test = false;
    DdgiProbeVisualizationPushConstant push_constant;
    std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)> record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
