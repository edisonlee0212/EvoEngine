#pragma once
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

#include <limits>

namespace evo_engine {
class Buffer;
class ComputePipeline;
class DescriptorSet;
class DescriptorSetLayout;

class DdgiProbeUpdatePass final {
 public:
  struct Parameters {
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    DdgiProbeAtlasUpdatePushConstant push_constant;
    std::shared_ptr<Buffer> metadata_readback_buffer;
    std::shared_ptr<Buffer> selected_ray_readback_buffer;
    uint32_t selected_ray_local_probe_index = (std::numeric_limits<uint32_t>::max)();
    uint32_t selected_ray_sample_count = 0;
    float* record_time_ms = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
