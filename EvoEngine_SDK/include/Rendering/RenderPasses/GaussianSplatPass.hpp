#pragma once
#include "RenderGraph.hpp"

#include <functional>
#include <glm/glm.hpp>

namespace evo_engine {
class Camera;
class DescriptorSet;
class DescriptorSetLayout;
class GraphicsPipeline;
class RenderInstanceStorage;

struct GaussianSplatPushConstant {
  glm::uvec4 camera_instance_count_flags = glm::uvec4(0u);
  glm::vec4 opacity_extent_min_max = glm::vec4(1.0f, 3.0f, 1.0f, 192.0f);
};

class GaussianSplatPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<RenderInstanceStorage> render_instances;
    std::shared_ptr<GraphicsPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    uint32_t camera_index = 0;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
