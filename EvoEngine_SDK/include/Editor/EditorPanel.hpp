#pragma once

#include <memory>

namespace evo_engine {
class EVOENGINE_API EditorLayer;

class EVOENGINE_API EditorPanel {
 public:
  virtual ~EditorPanel() = default;
  virtual void Draw(const std::shared_ptr<EditorLayer>& editor_layer) = 0;
};
}  // namespace evo_engine
