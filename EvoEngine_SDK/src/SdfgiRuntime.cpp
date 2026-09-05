// Scene ownership adapts Godot render_forward_clustered.cpp::sdfgi_update at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiRuntime.hpp"

using namespace evo_engine;

SdfgiAnchor evo_engine::SelectSdfgiAnchor(const SdfgiAnchor& explicit_camera, const SdfgiAnchor& main_camera,
                                          const SdfgiAnchor& editor_camera, const bool override_requested,
                                          const bool playing, const bool editor_present) {
  SdfgiAnchor result;
  if (explicit_camera.camera_id != 0) {
    result = explicit_camera;
    result.source = SdfgiAnchorSource::Explicit;
  } else if (main_camera.camera_id != 0 && (playing || !editor_present)) {
    result = main_camera;
    result.source = SdfgiAnchorSource::MainCamera;
  } else if (editor_camera.camera_id != 0) {
    result = editor_camera;
    result.source = SdfgiAnchorSource::EditorScene;
  }
  result.override_fell_back = override_requested && result.source != SdfgiAnchorSource::Explicit;
  return result;
}

SdfgiRuntime::SdfgiRuntime(const SdfgiSettings& initial_settings, SdfgiCapabilityReport report)
    : settings(initial_settings), capabilities(std::move(report)) {
}

bool SdfgiRuntime::Maintain(const uint32_t scene_frame, const SdfgiAnchor& selected_anchor) {
  if (last_scene_frame == scene_frame)
    return false;
  last_scene_frame = scene_frame;
  ++maintenance_count;
  anchor_recovered = missing_anchor && selected_anchor.camera_id != 0;
  missing_anchor = selected_anchor.camera_id == 0;
  if (!missing_anchor)
    published = false;
  anchor_replaced = !missing_anchor && anchor.camera_id != 0 && anchor.camera_id != selected_anchor.camera_id;
  if (!resources && (anchor_recovered || anchor_replaced)) {
    allocation_attempted = false;
    resource_failure.clear();
  }
  if (!missing_anchor)
    anchor = selected_anchor;
  anchor.override_fell_back = selected_anchor.override_fell_back;
  if (!missing_anchor) {
    placement_failure = UpdateSdfgiCascades(settings, anchor.world_position, cascades);
  } else {
    for (auto& cascade : cascades) {
      cascade.full_redraw = false;
      cascade.dirty_regions = glm::ivec3(0);
    }
  }
  pending_regions = GetSdfgiPendingRegions(cascades, SdfgiYMultiplier(settings.vertical_scale));
  fallback_reason = settings.Validate();
  if (fallback_reason.empty())
    fallback_reason = placement_failure;
  if (fallback_reason.empty())
    fallback_reason = resource_failure;
  if (fallback_reason.empty() && !capabilities.Supported())
    fallback_reason = capabilities.ToString();
  if (fallback_reason.empty())
    fallback_reason =
        missing_anchor ? "No eligible GI anchor; coverage stationary" : "No complete SDFGI field published";
  return true;
}

void SdfgiRuntime::UpdateSceneSnapshot(SdfgiSceneSnapshot snapshot) {
  scene_snapshot = std::move(snapshot);
  contributors.Update(scene_snapshot.contributors);
  pending_changes.resize(cascades.size());
  const auto affected = contributors.AffectedCascades(cascades, SdfgiYMultiplier(settings.vertical_scale));
  for (size_t c = 0; c < affected.size(); ++c) {
    pending_changes[c] |= affected[c];
    if (affected[c] & SdfgiUncertainBounds)
      invalidation_reason = "Unknown contributor bounds; conservative full-cascade redraw";
    if (missing_anchor && pending_changes[c])
      published = false;
  }
}

void SdfgiRuntime::PrepareUpdates(const bool has_representation, const bool force_full) {
  pending_changes.resize(cascades.size());
  payload_cascades = 0;
  auto raster_cascades = cascades;
  for (uint32_t c = 0; c < cascades.size(); ++c) {
    auto& cascade = cascades[c];
    if (!has_representation || force_full ||
        (pending_changes[c] && (pending_changes[c] != SdfgiPayloadChanged || cascade.dirty_regions != glm::ivec3(0)))) {
      cascade.full_redraw = true;
      cascade.dirty_regions = glm::ivec3(0);
    }
    raster_cascades[c] = cascade;
    if (pending_changes[c] == SdfgiPayloadChanged && !cascade.full_redraw) {
      payload_cascades |= 1u << c;
      raster_cascades[c].full_redraw = true;
    }
  }
  pending_regions = GetSdfgiPendingRegions(raster_cascades, SdfgiYMultiplier(settings.vertical_scale));
}

void SdfgiRuntime::AcknowledgeChanges(const uint32_t cascades_mask) {
  for (uint32_t c = 0; c < pending_changes.size(); ++c)
    if (cascades_mask & (1u << c))
      pending_changes[c] = 0;
}
