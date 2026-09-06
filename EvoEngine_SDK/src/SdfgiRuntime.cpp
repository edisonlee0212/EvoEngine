// Scene ownership adapts Godot render_forward_clustered.cpp::sdfgi_update at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "SdfgiRuntime.hpp"
#include <algorithm>
#include <tuple>
#include "SdfgiDebug.hpp"

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
  debug = std::make_shared<SdfgiDebugState>();
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
    if (anchor_replaced || anchor_recovered || std::any_of(cascades.begin(), cascades.end(), [](const auto& cascade) {
          return cascade.full_redraw || cascade.dirty_regions != glm::ivec3(0);
        }))
      debug->Invalidate("camera", anchor_replaced ? "GI anchor replaced" : "Camera coverage moved or initialized");
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
  const auto light_key = [](const SdfgiLightInput& light) {
    return std::tie(light.id, light.type, light.dynamic, light.casts_shadow, light.color, light.position,
                    light.direction, light.attenuation, light.range, light.cos_inner, light.cos_outer);
  };
  if (debug->enabled && (snapshot.lights.size() != scene_snapshot.lights.size() ||
                         !std::equal(snapshot.lights.begin(), snapshot.lights.end(), scene_snapshot.lights.begin(),
                                     [&](const auto& a, const auto& b) {
                                       return light_key(a) == light_key(b);
                                     })))
    debug->Invalidate("light", "Supported light inputs changed");
  const auto sky_key = [](const SdfgiSkyInput& sky) {
    return std::tie(sky.cubemap, sky.map_id, sky.map_version, sky.cubemap_version, sky.constant_color, sky.color,
                    sky.gamma, sky.rotation, sky.energy);
  };
  if (debug->enabled && sky_key(snapshot.sky) != sky_key(scene_snapshot.sky))
    debug->Invalidate("environment", "Indirect environment changed");
  scene_snapshot = std::move(snapshot);
  contributors.Update(scene_snapshot.contributors);
  if (debug->enabled) {
    uint32_t changed = 0;
    for (const auto& change : contributors.changes)
      changed |= change.flags;
    if (changed & (SdfgiAdded | SdfgiRemoved | SdfgiTransformChanged | SdfgiGeometryChanged | SdfgiUncertainBounds))
      debug->Invalidate("geometry", "Contributor geometry/placement changed");
    if (changed & (SdfgiCoverageChanged | SdfgiPayloadChanged))
      debug->Invalidate("material",
                        changed & SdfgiCoverageChanged ? "Material coverage changed" : "Material payload changed");
  }
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
