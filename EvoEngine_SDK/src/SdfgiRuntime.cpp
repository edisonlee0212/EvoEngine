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
  missing_anchor = selected_anchor.camera_id == 0;
  if (!missing_anchor)
    anchor = selected_anchor;
  anchor.override_fell_back = selected_anchor.override_fell_back;
  fallback_reason = settings.Validate();
  if (fallback_reason.empty() && !capabilities.Supported())
    fallback_reason = capabilities.ToString();
  if (fallback_reason.empty())
    fallback_reason =
        missing_anchor ? "No eligible GI anchor; coverage stationary" : "No complete SDFGI field published";
  return true;
}
