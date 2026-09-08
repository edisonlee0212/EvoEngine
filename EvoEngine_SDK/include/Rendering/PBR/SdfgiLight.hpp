// Direct-light adapter for Godot SDFGI::{render_static_lights,pre_process_gi,update_light},
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#pragma once

#include "SdfgiResources.hpp"
#include "SdfgiScene.hpp"
#include "SdfgiSliceLayout.hpp"

namespace evo_engine {

class EVOENGINE_API SdfgiLightDebug : public std::enable_shared_from_this<SdfgiLightDebug> {
 public:
  uint32_t cascade;
  uint32_t slice;
  bool recorded = false;
  std::array<std::shared_ptr<Buffer>, 3> planes;
  SdfgiSliceLayout slices;
  SdfgiLightDebug(uint32_t cascade, uint32_t slice, glm::ivec3 grid = glm::ivec3(128));
  void AddPass(RenderGraph& graph, RenderGraphResourceRegistry& registry,
               const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency);
  void StoreToPng(const std::filesystem::path& path) const;
  [[nodiscard]] uint64_t AllocationBytes() const;
};

struct SdfgiCascadeLights {
  std::array<std::vector<SdfgiLight>, 2> data;
  std::array<uint32_t, 2> overflow{};
};

EVOENGINE_API SdfgiCascadeLights BuildSdfgiCascadeLights(const std::vector<SdfgiLightInput>& inputs,
                                                         const SdfgiCascade& cascade, uint32_t index, float y_mult,
                                                         uint32_t positional_light_cascade_count);

class EVOENGINE_API SdfgiLightFrame : public std::enable_shared_from_this<SdfgiLightFrame> {
 public:
  uint32_t frame_slot = 0;
  uint32_t scene_frame = 0;
  uint32_t rebuilt_cascades = 0;
  uint32_t static_refresh = 0;
  uint32_t full_dynamic = 0;
  SdfgiSettings settings;
  float bounce_feedback = 0;
  SdfgiCascadeBlock cascades{};
  std::vector<SdfgiCascadeLights> lights;
  BufferUploadBatch input_uploads;
  BufferUploadArena uploads{64 * 1024};
  static std::shared_ptr<SdfgiLightFrame> Create(const SdfgiResources& resources,
                                                 const std::vector<SdfgiCascade>& cascades,
                                                 const std::vector<SdfgiLightInput>& inputs, uint32_t scene_frame,
                                                 uint32_t rebuilt_cascades);
  void AddPasses(RenderGraph& graph, const std::shared_ptr<SdfgiResources>& resources, const std::string& dependency);
};

}  // namespace evo_engine
