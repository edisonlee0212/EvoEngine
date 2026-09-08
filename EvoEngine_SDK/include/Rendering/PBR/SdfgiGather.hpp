// Godot SDFGI::pre_process_gi metadata adapter, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt.
#pragma once
#include "SdfgiResources.hpp"
#include "SdfgiRuntime.hpp"

namespace evo_engine {
class Camera;
EVOENGINE_API std::shared_ptr<SdfgiResources> SelectSdfgiCaptureResources(
    const std::shared_ptr<const SdfgiRuntime>& runtime, IndirectGiProvider provider);
EVOENGINE_API bool IsSdfgiCameraEligible(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Camera>& camera,
                                         const std::shared_ptr<Camera>& editor_camera, bool immediate,
                                         bool reflection_capture, bool custom_recorder);
EVOENGINE_API SdfgiGatherData BuildSdfgiGatherData(const SdfgiSettings& settings,
                                                   const std::vector<SdfgiCascade>& cascades, glm::vec3 anchor_world,
                                                   uint32_t generation, uint32_t max_image_dimension = 16384);

class EVOENGINE_API SdfgiGatherFrame : public std::enable_shared_from_this<SdfgiGatherFrame> {
 public:
  uint32_t frame_slot = 0;
  SdfgiGatherData metadata{};
  BufferUploadBatch input_upload;
  BufferUploadArena uploads{64 * 1024};
  std::shared_ptr<DescriptorSet> descriptor_set;
  static std::shared_ptr<SdfgiGatherFrame> Create(const SdfgiRuntime& runtime, uint32_t generation);
  void AddPublication(RenderGraph& graph, const std::shared_ptr<SdfgiRuntime>& runtime);
  [[nodiscard]] std::vector<RenderResourceAccess> CameraReads() const;
  void ImportCamera(RenderGraph& graph, RenderGraphResourceRegistry& registry, const SdfgiResources& resources) const;
};
}  // namespace evo_engine
