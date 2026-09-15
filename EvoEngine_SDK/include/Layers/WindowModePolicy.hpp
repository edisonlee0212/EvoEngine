#pragma once

#include <optional>
#include "EvoEngineAPI.hpp"
#include "WindowDisplayMode.hpp"

namespace evo_engine {

struct WindowPlacement {
  int x = 0;
  int y = 0;
  int width = 1280;
  int height = 720;
};

class EVOENGINE_API WindowModePolicy {
 public:
  explicit WindowModePolicy(WindowPlacement configured_windowed);

  [[nodiscard]] static bool IsFullscreen(WindowDisplayMode mode);
  [[nodiscard]] WindowDisplayMode ToggleTarget(WindowDisplayMode current) const;
  void RecordApplied(WindowDisplayMode mode, const WindowPlacement& placement);
  [[nodiscard]] WindowPlacement WindowedPlacement() const;
  [[nodiscard]] bool AllowsUserResize() const;
  [[nodiscard]] bool AllowsResolutionChange() const;
  void SetPermissions(bool user_resize, bool resolution_change);

 private:
  WindowPlacement configured_windowed_;
  std::optional<WindowPlacement> last_windowed_;
  std::optional<WindowDisplayMode> last_windowed_mode_;
  std::optional<WindowDisplayMode> last_fullscreen_mode_;
  bool allow_user_resize_ = true;
  bool allow_resolution_change_ = true;
};
}  // namespace evo_engine
