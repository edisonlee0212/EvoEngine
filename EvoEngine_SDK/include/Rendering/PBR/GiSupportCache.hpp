#pragma once
#include <array>
#include "HddagiGather.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/DeferredComputeLightingPass.hpp"
namespace evo_engine {
class RayTracingPipeline;
EVOENGINE_API std::shared_ptr<DescriptorSetLayout> CreateGiSupportLayout(bool ray_tracing);
struct alignas(16) GiSupportParams {
  HddagiGatherData field;
  glm::uvec2 viewport{1};
  uint32_t camera_index = 0, frame = 0;
  uint32_t generation = 1;
  glm::uvec3 padding{0};
  glm::vec4 sky_color_energy{0}, sky_parameters{0};
};
static_assert(sizeof(GiSupportParams) == 384);
class EVOENGINE_API GiSupportCache {
 public:
  static constexpr uint32_t Capacity = 8192;
  static constexpr uint32_t ProbeBytes = 192;
  static constexpr uint32_t Rays = 16;
  GiSupportCache(const std::shared_ptr<DescriptorSetLayout>& per_frame,
                 const std::shared_ptr<DescriptorSetLayout>& ray_tracing,
                 const std::shared_ptr<DescriptorSetLayout>& lighting, bool hddagi,
                 std::vector<std::shared_ptr<DescriptorSetLayout>> deferred_layouts = {});
  void Execute(const RenderGraphExecutionContext& context, const DeferredComputeLightingPass::Parameters& inputs,
               const std::shared_ptr<const HddagiRuntime>& runtime, const std::shared_ptr<DescriptorSet>& ray_tracing,
               const DdgiProbeRayTracingPushConstant& trace, uint64_t lighting_signature);
  [[nodiscard]] uint64_t AllocationBytes() const;
  [[nodiscard]] std::shared_ptr<DescriptorSet> GetCurrentSet() const;
  [[nodiscard]] std::shared_ptr<Buffer> GetPublishedUniform() const;
  [[nodiscard]] std::shared_ptr<DescriptorSet> GetPublishedSet(const HddagiRuntime& runtime) const;
  [[nodiscard]] const std::shared_ptr<ComputePipeline>& GetDeferredPipeline() const {
    return deferred_pipeline_;
  }

 private:
  struct Frame {
    std::shared_ptr<Buffer> uniform;
    std::shared_ptr<DescriptorSet> set;
    BufferUploadArena uploads{4096};
  };
  bool hddagi_, initialized_ = false;
  uint32_t generation_ = 1;
  uint64_t signature_ = 0;
  std::weak_ptr<const HddagiResources> field_;
  uint64_t content_version_ = 0;
  uint32_t region_version_ = 0;
  bool reset_pending_ = true;
  std::shared_ptr<DescriptorSetLayout> layout_;
  std::array<std::shared_ptr<ComputePipeline>, 6> pipelines_;
  std::shared_ptr<RayTracingPipeline> trace_;
  std::shared_ptr<Buffer> probes_, requests_, pending_, rays_;
  std::shared_ptr<ComputePipeline> deferred_pipeline_;
  std::shared_ptr<Frame> published_;
  std::shared_ptr<Image> mask_;
  std::shared_ptr<ImageView> mask_view_;
  std::shared_ptr<Sampler> sampler_;
  std::vector<std::shared_ptr<Frame>> frames_;
};
}  // namespace evo_engine
