#pragma once
#include "RenderGraph.hpp"

#include <functional>
#include <glm/glm.hpp>
#include <vector>

namespace evo_engine {
class Camera;
class DescriptorSet;
class GraphicsPipeline;
class RenderInstanceStorage;

class DeferredGeometryPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;
  using ExternalDeferredRendering = std::function<void(
      VkCommandBuffer vk_command_buffer, const std::vector<VkRenderingAttachmentInfo>& color_attachment_infos,
      const glm::ivec4& viewport)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<RenderInstanceStorage> render_instances;
    std::shared_ptr<GraphicsPipeline> mesh_pipeline;
    std::shared_ptr<GraphicsPipeline> instanced_pipeline;
    std::shared_ptr<GraphicsPipeline> skinned_pipeline;
    std::shared_ptr<GraphicsPipeline> strands_pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> meshlet_descriptor_set;
    int camera_index = -1;
    uint32_t current_frame_index = 0;
    bool use_mesh_shader = false;
    bool enable_indirect_rendering = false;
    bool count_draw_calls = false;
    bool wire_frame = false;
    ExternalDeferredRendering external_deferred_rendering;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
