#pragma once
#include "HddagiResources.hpp"
#include "HddagiTypes.hpp"

namespace evo_engine {
class EVOENGINE_API HddagiProbeFrame : public std::enable_shared_from_this<HddagiProbeFrame> {
 public:
  HddagiCascadeBlock cascades;
  HddagiIntegrateParams params;
  SdfgiSkyInput sky;
  std::shared_ptr<Image> sky_image;
  std::shared_ptr<ImageView> sky_view;
  std::shared_ptr<Buffer> cascade_buffer;
  std::shared_ptr<DescriptorSet> set;
  std::shared_ptr<ComputePipeline> pipeline;
  std::shared_ptr<ComputePipeline> filter_pipeline;
  std::shared_ptr<DescriptorSet> filter_set;
  std::shared_ptr<Sampler> sampler;
  std::array<std::shared_ptr<ComputePipeline>, 2> status_pipelines;
  std::vector<std::shared_ptr<DescriptorSet>> status_sets;
  std::shared_ptr<Buffer> status_readback;
  uint32_t written_cascades = 0;
  uint64_t scene_frame = 0;
  bool status_recorded = false;
  uint32_t force_frames_remaining = 0;
  BufferUploadBatch input_uploads;
  BufferUploadArena uploads{4096};

  [[nodiscard]] uint64_t AllocationBytes() const;
  static std::shared_ptr<HddagiProbeFrame> Create(HddagiResources& resources, const std::vector<SdfgiCascade>& cascades,
                                                  const SdfgiSkyInput& sky, uint32_t scene_frame, bool force_update,
                                                  uint32_t written_cascades = 0);
  void AddBeginPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                    const std::shared_ptr<HddagiResources>& resources, const std::string& dependency);
  void ReadStatusAfterFence(HddagiResources& resources) const;
  void AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                 const std::shared_ptr<HddagiResources>& resources, const std::string& dependency);
};
}  // namespace evo_engine
