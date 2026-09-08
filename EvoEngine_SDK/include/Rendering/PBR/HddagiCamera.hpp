#pragma once

#include "HddagiGather.hpp"
#include "RenderPasses/DeferredComputeLightingPass.hpp"

namespace evo_engine {
class RenderInstanceStorage;
struct alignas(16) HddagiCameraParams {
  glm::uvec2 viewport{1};
  glm::uvec2 gi_size{1};
  uint32_t pixel_stride = 1;
  uint32_t reflection_capture = 0;
  uint32_t filter_radius = 12;
  uint32_t padding = 0;
};
static_assert(sizeof(HddagiCameraParams) == 32);

struct HddagiCameraImages {
  HddagiCameraLayout layout;
  std::map<std::string, HddagiImage> images;
};

EVOENGINE_API std::shared_ptr<HddagiRuntime> SnapshotHddagiCapture(const std::shared_ptr<const HddagiRuntime>& runtime,
                                                                   bool immediate = false);

class EVOENGINE_API HddagiCameraFrame : public std::enable_shared_from_this<HddagiCameraFrame> {
 public:
  HddagiCameraParams params;
  HddagiGatherData metadata;
  std::shared_ptr<HddagiCameraImages> images;
  std::map<std::string, std::shared_ptr<Buffer>> buffers;
  std::vector<uint32_t> receiver_classification;
  std::shared_ptr<DescriptorSet> set;
  std::map<std::string, std::shared_ptr<ComputePipeline>> pipelines;
  BufferUploadBatch input_uploads;
  BufferUploadArena uploads{64 * 1024};
  bool uploaded = false;
  bool filter_reflections = false;

  static std::shared_ptr<HddagiCameraFrame> Create(
      HddagiRuntime& runtime, RenderInstanceStorage& instances,
      const std::vector<std::shared_ptr<DescriptorSetLayout>>& host_layouts, glm::uvec2 viewport, uint64_t camera_id,
      bool reflection_capture);
  void ImportCamera(RenderGraph& graph, RenderGraphResourceRegistry& registry, const HddagiResources& field) const;
  void AddPasses(RenderGraph& graph, const std::shared_ptr<HddagiResources>& field,
                 const std::function<DeferredComputeLightingPass::Parameters()>& parameters,
                 const std::string& dependency);
  [[nodiscard]] uint64_t AllocationBytes() const;
};
}  // namespace evo_engine
