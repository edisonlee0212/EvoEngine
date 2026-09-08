// Godot SDFGI::{update_probes,store_probes} adapter,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#pragma once

#include "SdfgiResources.hpp"
#include "SdfgiScene.hpp"

namespace evo_engine {

class EVOENGINE_API SdfgiProbeDebug : public std::enable_shared_from_this<SdfgiProbeDebug> {
 public:
  uint32_t cascade, probe, cascade_count, history_size;
  uint32_t probe_axis, columns, rows;
  bool recorded = false;
  std::array<std::shared_ptr<Buffer>, 4> data;
  SdfgiProbeDebug(const SdfgiSettings& settings, uint32_t cascade, uint32_t probe);
  void AddPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
               const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency);
  [[nodiscard]] SdfgiFieldStatus ReadStatus() const;
  void StoreToPng(const std::filesystem::path& path) const;
  [[nodiscard]] uint64_t AllocationBytes() const;
};

class EVOENGINE_API SdfgiProbeFrame : public std::enable_shared_from_this<SdfgiProbeFrame> {
 public:
  uint32_t frame_slot = 0;
  uint32_t scene_frame = 0;
  SdfgiSkyInput sky;
  std::shared_ptr<Image> sky_image;
  std::shared_ptr<ImageView> sky_view;
  std::shared_ptr<DescriptorSet> sky_set;
  std::vector<SdfgiIntegratePushConstant> constants;
  static std::shared_ptr<SdfgiProbeFrame> Create(const SdfgiResources& resources,
                                                 const std::vector<SdfgiCascade>& cascades, const SdfgiSkyInput& sky,
                                                 uint32_t scene_frame);
  void AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                 const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency);
};

}  // namespace evo_engine
