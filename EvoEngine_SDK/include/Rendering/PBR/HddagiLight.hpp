#pragma once

#include "HddagiResources.hpp"
#include "HddagiTypes.hpp"
#include "SdfgiLight.hpp"

namespace evo_engine {
class EVOENGINE_API HddagiLightFrame : public std::enable_shared_from_this<HddagiLightFrame> {
 public:
  HddagiCascadeBlock cascades;
  std::vector<SdfgiCascadeLights> lights;
  std::vector<std::array<std::shared_ptr<DescriptorSet>, 2>> sets;
  std::map<std::string, std::shared_ptr<Buffer>> buffers;
  std::vector<std::shared_ptr<Buffer>> process, dispatch;
  std::array<std::shared_ptr<ComputePipeline>, 2> pipelines;
  std::shared_ptr<Sampler> sampler;
  BufferUploadBatch input_uploads;
  BufferUploadArena uploads{64 * 1024};
  uint32_t written_cascades = 0;
  uint32_t refresh_static = 0;
  uint32_t full_dynamic = 0;
  uint32_t scene_frame = 0;
  float bounce_feedback = 0;
  bool lighting_changed = false;

  [[nodiscard]] uint64_t AllocationBytes() const;
  static std::shared_ptr<HddagiLightFrame> Create(HddagiResources& resources, const std::vector<SdfgiCascade>& cascades,
                                                  const std::vector<SdfgiLightInput>& lights, uint32_t scene_frame,
                                                  uint32_t written_cascades);
  void AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                 const std::shared_ptr<HddagiResources>& resources, const std::string& dependency);
};
}  // namespace evo_engine
