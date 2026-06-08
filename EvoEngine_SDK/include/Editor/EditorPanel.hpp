#pragma once

#include <memory>

namespace evo_engine {
class EditorLayer;

class EditorPanel {
 public:
  virtual ~EditorPanel() = default;
  virtual void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) = 0;
};
}  // namespace evo_engine
