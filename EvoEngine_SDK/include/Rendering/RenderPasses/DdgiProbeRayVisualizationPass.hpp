#pragma once
#include "RenderGraph.hpp"

#include <functional>
#include <glm/glm.hpp>

namespace evo_engine {
class Camera;
class DescriptorSet;
class DescriptorSetLayout;
class GraphicsPipeline;

struct DdgiProbeRayVisualizationPushConstant {
  glm::uvec4 camera_selected_probe_ray_count_flags = glm::uvec4(0, 0, 0, 0);
  glm::vec4 miss_distance_alpha_padding = glm::vec4(1.0f, 0.85f, 0.0f, 0.0f);
};

class DdgiProbeRayVisualizationPass final {
 public:
  struct Parameters {
    std::shared_ptr<GraphicsPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    std::shared_ptr<Camera> camera;
    uint32_t camera_index = 0;
    bool depth_test = false;
    DdgiProbeRayVisualizationPushConstant push_constant;
    float* record_time_ms = nullptr;
    std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)> record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
