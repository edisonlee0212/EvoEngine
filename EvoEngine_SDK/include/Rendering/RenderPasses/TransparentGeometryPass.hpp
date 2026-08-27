#pragma once
#include "RenderGraph.hpp"

#include <functional>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API GraphicsPipeline;
class EVOENGINE_API RenderInstanceStorage;

class EVOENGINE_API TransparentGeometryPass final {
 public:
  using RecordCommands = std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<RenderInstanceStorage> render_instances;
    std::shared_ptr<GraphicsPipeline> mesh_pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> lighting_descriptor_set;
    std::shared_ptr<DescriptorSet> raster_lighting_texture_descriptor_set;
    int camera_index = -1;
    uint32_t current_frame_index = 0;
    bool count_draw_calls = false;
    bool wire_frame = false;
    RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(const char* dependency = nullptr);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
