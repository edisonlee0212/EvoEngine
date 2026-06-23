#pragma once
#include "RenderGraph.hpp"

#include <functional>
#include <glm/glm.hpp>

namespace evo_engine {
class Camera;
class DescriptorSet;
class GraphicsPipeline;

class DeferredLightingPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;
  using ExternalForwardRendering = std::function<void(VkCommandBuffer vk_command_buffer, const glm::ivec4& viewport)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<GraphicsPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> lighting_descriptor_set;
    int camera_index = -1;
    bool fade_selection = false;
    int selection_alpha = 0;
    ExternalForwardRendering external_forward_rendering;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
