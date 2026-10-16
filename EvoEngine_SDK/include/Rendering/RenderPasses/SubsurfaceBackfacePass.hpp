#pragma once
#include <array>
#include "RenderPasses/DeferredGeometryPass.hpp"

namespace evo_engine {
class EVOENGINE_API SubsurfaceBackfacePass final {
 public:
  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<RenderInstanceStorage> render_instances;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::array<std::shared_ptr<GraphicsPipeline>, 3> pipelines;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    int camera_index = -1;
    DeferredGeometryPass::RecordCommands record_commands;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
