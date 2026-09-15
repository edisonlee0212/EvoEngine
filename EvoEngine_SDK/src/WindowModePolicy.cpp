#include "WindowModePolicy.hpp"

namespace evo_engine {
WindowModePolicy::WindowModePolicy(const WindowPlacement configured_windowed)
    : configured_windowed_(configured_windowed) {
}

bool WindowModePolicy::IsFullscreen(const WindowDisplayMode mode) {
  return mode == WindowDisplayMode::BorderlessFullscreen || mode == WindowDisplayMode::ExclusiveFullscreen;
}

WindowDisplayMode WindowModePolicy::ToggleTarget(const WindowDisplayMode current) const {
  if (IsFullscreen(current)) {
    return last_windowed_mode_.value_or(WindowDisplayMode::Windowed);
  }
  return last_fullscreen_mode_.value_or(WindowDisplayMode::BorderlessFullscreen);
}

void WindowModePolicy::RecordApplied(const WindowDisplayMode mode, const WindowPlacement& placement) {
  if (IsFullscreen(mode)) {
    last_fullscreen_mode_ = mode;
  } else {
    last_windowed_mode_ = mode;
    last_windowed_ = placement;
  }
}

WindowPlacement WindowModePolicy::WindowedPlacement() const {
  return last_windowed_.value_or(configured_windowed_);
}

bool WindowModePolicy::AllowsUserResize() const {
  return allow_user_resize_;
}

bool WindowModePolicy::AllowsResolutionChange() const {
  return allow_resolution_change_;
}

void WindowModePolicy::SetPermissions(const bool user_resize, const bool resolution_change) {
  allow_user_resize_ = user_resize;
  allow_resolution_change_ = resolution_change;
}
}  // namespace evo_engine
