// Godot SDFGI::render_region adapter, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt.
#pragma once

#include "SdfgiResources.hpp"

namespace evo_engine {

class EVOENGINE_API SdfgiPreprocessDebug : public std::enable_shared_from_this<SdfgiPreprocessDebug> {
 public:
  uint32_t cascade;
  uint32_t slice;
  bool recorded = false;
  std::array<std::shared_ptr<Buffer>, 2> planes;
  SdfgiPreprocessDebug(uint32_t cascade, uint32_t slice);
  void AddPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
               const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency);
  void StoreToPng(const std::filesystem::path& path) const;
  [[nodiscard]] uint64_t AllocationBytes() const;
};

class EVOENGINE_API SdfgiPreprocessReadback {
 public:
  std::shared_ptr<Buffer> buffer;
  uint32_t cascade_count;
  uint32_t recorded_cascades = 0;
  bool consumed = false;
  explicit SdfgiPreprocessReadback(uint32_t cascade_count);
  // The caller must have recycled this snapshot's frame fence, or explicitly waited for capture.
  void ReadAfterFrameFence(SdfgiResources& resources);
};

EVOENGINE_API void RecordSdfgiPreprocess(VkCommandBuffer command, const SdfgiResources& resources, uint32_t cascade,
                                         glm::ivec3 cascade_position, glm::ivec3 scroll = glm::ivec3(0));
EVOENGINE_API void RecordSdfgiScroll(VkCommandBuffer command, const SdfgiResources& resources, uint32_t cascade,
                                     glm::ivec3 cascade_position, glm::ivec3 scroll, uint32_t frame_slot);
EVOENGINE_API std::string AddSdfgiPreprocessPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                                 const std::shared_ptr<SdfgiResources>& resources,
                                                 const std::shared_ptr<SdfgiPreprocessReadback>& readback,
                                                 uint32_t cascade, glm::ivec3 cascade_position,
                                                 const std::string& dependency, glm::ivec3 scroll = glm::ivec3(0));

}  // namespace evo_engine
