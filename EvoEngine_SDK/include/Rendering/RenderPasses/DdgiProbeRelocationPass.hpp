#pragma once
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

namespace evo_engine {
class ComputePipeline;
class DescriptorSetLayout;

class DdgiProbeRelocationPass final {
 public:
  struct Parameters {
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    DdgiProbeRelocationPushConstant reset_push_constant;
    DdgiProbeRelocationPushConstant update_push_constant;
    bool reset_offsets = false;
    bool relocate_probes = false;
    float* record_time_ms = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
