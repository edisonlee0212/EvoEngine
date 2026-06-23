#pragma once
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

namespace evo_engine {
class DescriptorSet;
class DescriptorSetLayout;
class RayTracingPipeline;
class Sampler;

class DdgiRayDiagnosticsPass final {
 public:
  struct Parameters {
    std::shared_ptr<RayTracingPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> ray_tracing_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> ray_output_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    std::shared_ptr<Sampler> atlas_sampler;
    DdgiProbeRayTracingPushConstant push_constant;
    float* record_time_ms = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
