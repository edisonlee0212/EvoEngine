#pragma once
#include "RenderGraph.hpp"

#include <functional>
#include <glm/glm.hpp>

namespace evo_engine {
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API GraphicsPipeline;
class EVOENGINE_API RenderInstanceStorage;

class EVOENGINE_API DirectionalLightShadowPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;
  using GetDepthAttachment = std::function<VkRenderingAttachmentInfo(uint32_t split, VkAttachmentLoadOp load_op,
                                                                     VkAttachmentStoreOp store_op)>;
  using ExternalShadowRendering =
      std::function<void(VkCommandBuffer vk_command_buffer, int light_index, int split_index,
                         const glm::ivec4& viewport, const glm::mat4& light_space_matrix)>;

  struct Parameters {
    std::shared_ptr<RenderInstanceStorage> render_instances;
    std::shared_ptr<GraphicsPipeline> directional_opaque_pipeline;
    std::shared_ptr<GraphicsPipeline> directional_masked_pipeline;
    std::shared_ptr<GraphicsPipeline> instanced_opaque_pipeline;
    std::shared_ptr<GraphicsPipeline> instanced_masked_pipeline;
    std::shared_ptr<GraphicsPipeline> skinned_opaque_pipeline;
    std::shared_ptr<GraphicsPipeline> skinned_masked_pipeline;
    std::shared_ptr<GraphicsPipeline> strands_opaque_pipeline;
    std::shared_ptr<GraphicsPipeline> strands_masked_pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> meshlet_descriptor_set;
    std::shared_ptr<DescriptorSet> strand_meshlet_descriptor_set;
    int camera_index = -1;
    int max_directional_light_count = 0;
    uint32_t current_frame_index = 0;
    VkExtent2D shadow_map_extent{};
    bool use_mesh_shader = false;
    bool enable_indirect_rendering = false;
    bool count_draw_calls = false;
    GetDepthAttachment get_depth_attachment;
    ExternalShadowRendering external_opaque_shadow_rendering;
    ExternalShadowRendering external_masked_shadow_rendering;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
