#include "EntitySelectionHighlight.hpp"

#include <algorithm>

using namespace evo_engine;

void EntitySelectionHighlight::SetEnabled(const bool enabled, const bool has_selection) {
  if (enabled && !enabled_ && has_selection) {
    fade_progress_ = 0.0f;
  }
  enabled_ = enabled;
}

void EntitySelectionHighlight::Update(const bool has_selection, const float delta_time) {
  if (!has_selection) {
    fade_progress_ = 0.0f;
  } else {
    if (!had_selection_) {
      fade_progress_ = 0.0f;
    }
    if (enabled_) {
      fade_progress_ = std::min(1.0f, fade_progress_ + std::max(0.0f, delta_time) * 5.0f);
    }
  }
  had_selection_ = has_selection;
}

void EntitySelectionHighlight::Reset() {
  fade_progress_ = 0.0f;
  had_selection_ = false;
}

bool EntitySelectionHighlight::IsEnabled() const {
  return enabled_;
}

EntitySelectionHighlight::Snapshot EntitySelectionHighlight::GetSnapshot() const {
  return {outline_color_, radius_, focus_strength_, fade_progress_, enabled_, enabled_ && had_selection_};
}
