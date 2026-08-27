#pragma once
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

namespace evo_engine {
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API DescriptorSetLayout;

class EVOENGINE_API DdgiProbeScrollPass final {
 public:
  struct Parameters {
    std::shared_ptr<ComputePipeline> pipeline;
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    DdgiProbeScrollPushConstant push_constant{};
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
