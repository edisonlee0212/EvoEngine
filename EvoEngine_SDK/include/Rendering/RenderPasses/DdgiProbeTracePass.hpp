#pragma once
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

namespace evo_engine {
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;
class EVOENGINE_API Buffer;
class EVOENGINE_API RayTracingPipeline;
class EVOENGINE_API Sampler;

class EVOENGINE_API DdgiProbeTracePass final {
 public:
  struct CascadeResources {
    std::shared_ptr<Image> irradiance;
    std::shared_ptr<Image> visibility;
    std::shared_ptr<Buffer> probe_state;
  };
  struct Parameters {
    std::shared_ptr<RayTracingPipeline> pipeline;
    std::shared_ptr<DescriptorSet> per_frame_descriptor_set;
    std::shared_ptr<DescriptorSet> ray_tracing_descriptor_set;
    std::shared_ptr<DescriptorSetLayout> ray_output_layout;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    std::shared_ptr<Sampler> atlas_sampler;
    DdgiProbeRayTracingPushConstant push_constant;
    uint32_t probe_update_count = 0;
    std::shared_ptr<Buffer> selected_ray_readback_buffer;
    uint32_t selected_ray_sample_count = 0;
    bool* selected_ray_readback_recorded = nullptr;
    bool use_emissive_sampling = false;
    uint32_t* recorded_ray_sample_count = nullptr;
    std::array<CascadeResources, 8> cascades{};
    std::array<VkDescriptorImageInfo, 3> voxel_occlusion{};
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(bool use_emissive_sampling = false);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
