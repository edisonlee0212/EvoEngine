#pragma once

#include "HddagiResources.hpp"
#include "HddagiScene.hpp"
#include "HddagiTypes.hpp"
#include "SdfgiVoxelizer.hpp"

namespace evo_engine {
class EVOENGINE_API HddagiVoxelFrame : public std::enable_shared_from_this<HddagiVoxelFrame> {
 public:
  std::shared_ptr<SdfgiVoxelFrame> inputs;
  std::shared_ptr<GraphicsPipeline> raster;
  std::shared_ptr<ComputePipeline> region_store;
  std::shared_ptr<DescriptorSet> region_set;
  std::shared_ptr<ComputePipeline> light_store;
  std::vector<std::shared_ptr<DescriptorSet>> light_sets;
  HddagiUpdatePlan update_plan;
  std::vector<HddagiUpdateBounds> update_bounds;
  std::shared_ptr<ComputePipeline> light_scroll;
  std::vector<std::shared_ptr<DescriptorSet>> scroll_sets;
  std::shared_ptr<ComputePipeline> reset_probes;
  std::shared_ptr<DescriptorSet> reset_set;
  uint32_t written_cascades = 0;
  std::shared_ptr<Buffer> status_readback;
  uint64_t scene_frame = 0;
  bool status_recorded = false;
  uint32_t version = 1;

  void ReadStatusAfterFence(HddagiResources& resources) const;
  [[nodiscard]] uint64_t AllocationBytes() const;

  static std::shared_ptr<HddagiVoxelFrame> Create(HddagiResources& resources,
                                                  const std::shared_ptr<DescriptorSetLayout>& host_layout,
                                                  const SdfgiContributorRegistry& contributors,
                                                  const std::vector<SdfgiCascade>& cascades,
                                                  const std::vector<SdfgiPendingRegion>& pending, uint32_t version);
  static std::shared_ptr<HddagiVoxelFrame> Create(HddagiResources& resources,
                                                  const std::shared_ptr<DescriptorSetLayout>& host_layout,
                                                  const SdfgiContributorRegistry& contributors,
                                                  const HddagiUpdatePlan& plan, uint32_t version);
  void AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                 const std::shared_ptr<HddagiResources>& resources);
};
}  // namespace evo_engine
