#pragma once
#include "DdgiHistory.hpp"
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

namespace evo_engine {
class EVOENGINE_API Buffer;
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;

class EVOENGINE_API DdgiProbeUpdatePass final {
 public:
  struct DispatchSize {
    uint32_t x = 0u;
    uint32_t y = 0u;
    bool valid = false;
  };

  struct Parameters {
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<ComputePipeline> parallel_irradiance_pipeline;
    std::shared_ptr<ComputePipeline> parallel_visibility_pipeline;
    bool use_parallel = false;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    DdgiProbeAtlasUpdatePushConstant push_constant;
    std::shared_ptr<Buffer> metadata_readback_buffer;
    bool* metadata_readback_recorded = nullptr;
    uint32_t* recorded_probe_update_count = nullptr;
    bool use_emissive_sampling = false;
    bool clear_history = false;
    bool invalidate_moved_history = false;
  };

  [[nodiscard]] static DispatchSize CalculateDispatchSize(uint32_t probe_count, bool parallel,
                                                          uint32_t max_group_count_x, uint32_t max_group_count_y);
  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(bool use_emissive_sampling = false);
  [[nodiscard]] static RenderPassDescriptor CreateHistoryInvalidationDescriptor(bool relocation, bool classification);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
