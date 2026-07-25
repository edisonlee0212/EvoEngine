#pragma once
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

namespace evo_engine {
class Buffer;
class ComputePipeline;
class DescriptorSetLayout;

class DdgiProbeVariabilityPass final {
 public:
  struct DdgiAtlasLayout {
    uint32_t probe_count = 1;
    uint32_t tile_resolution = 1;
    uint32_t columns = 1;
    glm::uvec2 resolution = {1, 1};
    glm::uvec2 reduction_extent = {1, 1};
  };

  struct Parameters {
    std::shared_ptr<ComputePipeline> reduce_pipeline;
    std::shared_ptr<ComputePipeline> extra_reduce_pipeline;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    DdgiAtlasLayout layout;
    std::shared_ptr<Buffer> readback_buffer;
    bool* readback_recorded = nullptr;
    float* record_time_ms = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
