#pragma once

#include <glm/glm.hpp>

namespace evo_engine {

class EntitySelectionHighlight final {
 public:
  struct Snapshot {
    glm::vec4 outline_color{1.0f, 0.75f, 0.0f, 1.0f};
    float radius = 3.0f;
    float focus_strength = 0.5f;
    float fade_progress = 0.0f;
    bool enabled = true;
    bool active = false;
  };

  void SetEnabled(bool enabled, bool has_selection);
  void Update(bool has_selection, float delta_time);
  void Reset();

  [[nodiscard]] bool IsEnabled() const;
  [[nodiscard]] Snapshot GetSnapshot() const;

 private:
  glm::vec4 outline_color_{1.0f, 0.75f, 0.0f, 1.0f};
  float radius_ = 3.0f;
  float focus_strength_ = 0.5f;
  float fade_progress_ = 0.0f;
  bool enabled_ = true;
  bool had_selection_ = false;
};

}  // namespace evo_engine
