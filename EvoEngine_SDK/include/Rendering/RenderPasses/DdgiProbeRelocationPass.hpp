#pragma once
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

namespace evo_engine {
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API DescriptorSetLayout;

class EVOENGINE_API DdgiProbeRelocationPass final {
 public:
  struct Parameters {
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    DdgiProbeRelocationPushConstant reset_push_constant;
    DdgiProbeRelocationPushConstant update_push_constant;
    bool reset_offsets = false;
    bool relocate_probes = false;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
