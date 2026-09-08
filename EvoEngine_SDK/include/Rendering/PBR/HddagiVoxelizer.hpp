#pragma once

#include "HddagiResources.hpp"
#include "SdfgiVoxelizer.hpp"

namespace evo_engine {
class EVOENGINE_API HddagiVoxelFrame : public std::enable_shared_from_this<HddagiVoxelFrame> {
 public:
  std::shared_ptr<SdfgiVoxelFrame> inputs;
  std::shared_ptr<GraphicsPipeline> raster;
  std::shared_ptr<ComputePipeline> region_store;
  std::shared_ptr<DescriptorSet> region_set;
  uint32_t version = 1;

  static std::shared_ptr<HddagiVoxelFrame> Create(HddagiResources& resources,
                                                  const std::shared_ptr<DescriptorSetLayout>& host_layout,
                                                  const SdfgiContributorRegistry& contributors,
                                                  const std::vector<SdfgiCascade>& cascades,
                                                  const std::vector<SdfgiPendingRegion>& pending, uint32_t version);
  void AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                 const std::shared_ptr<HddagiResources>& resources);
};
}  // namespace evo_engine
