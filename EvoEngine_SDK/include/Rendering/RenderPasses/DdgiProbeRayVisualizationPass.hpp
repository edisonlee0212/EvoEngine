#pragma once
#include "RenderGraph.hpp"

#include <functional>
#include <glm/glm.hpp>

namespace evo_engine {
class Camera;
class Buffer;
class DescriptorSet;
class DescriptorSetLayout;
class GraphicsPipeline;

struct DdgiProbeRayVisualizationPushConstant {
  glm::uvec2 camera_ray_count = glm::uvec2(0);
  glm::vec2 miss_distance_alpha = glm::vec2(1.0f, 0.85f);
};

class DdgiProbeRayVisualizationPass final {
 public:
  struct Parameters {
    std::shared_ptr<GraphicsPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    std::shared_ptr<Buffer> selected_ray_diagnostics_buffer;
    std::shared_ptr<Camera> camera;
    bool depth_test = false;
    DdgiProbeRayVisualizationPushConstant push_constant;
    std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)> record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
