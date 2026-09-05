#pragma once

#include "SdfgiCapabilities.hpp"
#include "SdfgiScene.hpp"
#include "SdfgiSettings.hpp"

#include <glm/glm.hpp>
#include <limits>
#include <memory>

namespace evo_engine {

class SdfgiResources;
struct SdfgiDebugState;

enum class SdfgiAnchorSource : uint32_t { None, Explicit, MainCamera, EditorScene };

struct SdfgiAnchor {
  uint64_t camera_id = 0;
  glm::vec3 world_position = glm::vec3(0.0f);
  SdfgiAnchorSource source = SdfgiAnchorSource::None;
  bool override_fell_back = false;
};

EVOENGINE_API SdfgiAnchor SelectSdfgiAnchor(const SdfgiAnchor& explicit_camera, const SdfgiAnchor& main_camera,
                                            const SdfgiAnchor& editor_camera, bool override_requested, bool playing,
                                            bool editor_present);

struct EVOENGINE_API SdfgiRuntime {
  SdfgiSettings settings;
  SdfgiCapabilityReport capabilities;
  std::shared_ptr<SdfgiDebugState> debug;
  SdfgiAnchor anchor;
  uint64_t maintenance_count = 0;
  uint32_t last_scene_frame = std::numeric_limits<uint32_t>::max();
  bool missing_anchor = true;
  bool published = false;
  bool allocation_attempted = false;
  bool anchor_replaced = false;
  bool anchor_recovered = false;
  std::vector<SdfgiCascade> cascades;
  std::vector<SdfgiPendingRegion> pending_regions;
  SdfgiSceneSnapshot scene_snapshot;
  SdfgiContributorRegistry contributors;
  std::vector<uint32_t> pending_changes;
  uint32_t payload_cascades = 0;
  std::string invalidation_reason;
  std::string placement_failure;
  std::shared_ptr<SdfgiResources> resources;
  std::string resource_failure;
  std::string fallback_reason = "No complete SDFGI field published";

  SdfgiRuntime(const SdfgiSettings& initial_settings, SdfgiCapabilityReport report);
  bool Maintain(uint32_t scene_frame, const SdfgiAnchor& selected_anchor);
  void UpdateSceneSnapshot(SdfgiSceneSnapshot snapshot);
  void PrepareUpdates(bool has_representation, bool force_full);
  void AcknowledgeChanges(uint32_t cascades_mask);
};

}  // namespace evo_engine
