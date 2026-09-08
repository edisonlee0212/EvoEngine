#pragma once

#include "SdfgiResources.hpp"
#include "SdfgiRuntime.hpp"

#include <filesystem>

namespace evo_engine {
class Camera;
struct CameraInfoBlock;
class SdfgiCameraDebugFrame;

class EVOENGINE_API SdfgiDebugRenderer {
 public:
  std::shared_ptr<DescriptorSetLayout> layout;
  std::shared_ptr<ComputePipeline> sdf;
  std::array<std::shared_ptr<GraphicsPipeline>, 3> graphics;
  std::vector<std::vector<std::shared_ptr<SdfgiCameraDebugFrame>>> frames;
  uint32_t last_retirement = UINT32_MAX;
  SdfgiDebugRenderer();
};

enum class SdfgiDebugView : uint32_t {
  None,
  Cascades,
  Sdf,
  Probes,
  Visibility,
  DirtyRegions,
  DistanceSlice,
  Diffuse,
  Specular,
  Fallback,
  Contributors
};
EVOENGINE_API const char* GetSdfgiDebugViewName(SdfgiDebugView view);

struct EVOENGINE_API SdfgiDebugState {
  bool enabled = false;
  bool frozen = false;
  bool single_step = false;
  bool full_redraw = false;
  bool reset_history = false;
  bool depth_test = true;
  uint32_t seed = 0;
  uint32_t cascade = 0;
  uint32_t probe = 2456;
  uint32_t slice = 64;
  uint64_t camera_id = 0;  // Zero selects only the canonical editor Scene camera.
  SdfgiDebugView view = SdfgiDebugView::None;
  uint32_t last_boundary = UINT32_MAX;
  std::map<std::string, uint64_t> invalidations;
  std::map<uint64_t, uint32_t> camera_views;
  std::string last_reason = "Scene/provider activated";
  std::string failure;
  std::array<uint64_t, 4> active_bytes{};
  std::array<uint64_t, 4> retiring_bytes{};
  uint64_t peak_transient_bytes = 0;
  std::weak_ptr<Camera> rendered_camera;
  std::weak_ptr<SdfgiResources> rendered_field;
  glm::mat4 rendered_view_projection{1};
  glm::uvec3 rendered_selection{};
  glm::uvec2 rendered_resolution{};
  uint64_t rendered_camera_selection = 0;
  bool rendered_depth_test = true;
  uint32_t rendered_boundary = UINT32_MAX;
  uint32_t rendered_generation = 0;
  SdfgiDebugView rendered_view = SdfgiDebugView::None;

  bool BeginFrame(uint32_t scene_frame);
  void Invalidate(const char* category, const char* reason);
  bool MatchesCamera(uint64_t id, bool editor_scene, bool ordinary_raster) const;
};

EVOENGINE_API void AddSdfgiHistoryReset(RenderGraph& graph, const std::shared_ptr<SdfgiRuntime>& runtime);
EVOENGINE_API std::string BuildSdfgiDebugSnapshot(const SdfgiRuntime& runtime);
EVOENGINE_API void CaptureSdfgiDebugImage(const SdfgiRuntime& runtime, const std::filesystem::path& path);
EVOENGINE_API void UpdateSdfgiDebugMemory(const SdfgiRuntime& runtime,
                                          const std::vector<std::vector<std::shared_ptr<SdfgiResources>>>& owners);
EVOENGINE_API void AddSdfgiCameraDebug(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                                       const std::shared_ptr<const SdfgiRuntime>& runtime,
                                       const std::shared_ptr<Camera>& camera, const CameraInfoBlock& camera_data,
                                       const char* dependency);
EVOENGINE_API void RetireSdfgiDebugFrame(SdfgiResources& resources, uint32_t frame_slot);
EVOENGINE_API uint64_t GetSdfgiDebugAllocationBytes(const SdfgiResources& resources);
}  // namespace evo_engine
