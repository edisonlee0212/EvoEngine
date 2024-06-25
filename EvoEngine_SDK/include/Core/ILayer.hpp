#pragma once
#include "Input.hpp"

namespace evo_engine {
class Scene;
class EditorLayer;
class ILayer {
  std::weak_ptr<Scene> scene_;
  std::weak_ptr<ILayer> subsequent_layer_;
  friend class Application;
  friend class EditorLayer;
  friend class Input;
  virtual void OnCreate() {
  }
  virtual void OnDestroy() {
  }
  virtual void PreUpdate() {
  }
  virtual void FixedUpdate() {
  }
  virtual void Update() {
  }
  virtual void LateUpdate() {
  }
  virtual void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  }
  virtual void OnInputEvent(const InputEvent& input_event);

 public:
  [[nodiscard]] std::shared_ptr<Scene> GetScene() const;
};
}  // namespace evo_engine