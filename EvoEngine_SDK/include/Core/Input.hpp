#pragma once
#include "ISingleton.hpp"

namespace evo_engine {

class Input final {
  EVOENGINE_SINGLETON_INSTANCE(Input)
 public:
  enum class KeyActionType { Press, Hold, Release, Unknown };
  struct InputEvent {
    int key = GLFW_KEY_UNKNOWN;
    KeyActionType key_action = KeyActionType::Unknown;
  };
  static glm::vec2 GetMousePosition();

 private:
  friend class Graphics;
  friend class Application;
  friend class EditorLayer;
  std::unordered_map<int, KeyActionType> pressed_keys_ = {};
  glm::vec2 mouse_position_ = glm::vec2(0.0f);

  static void KeyCallBack(GLFWwindow* window, int key, int scan_code, int action, int mods);
  static void MouseButtonCallBack(GLFWwindow* window, int button, int action, int mods);
  static void Dispatch(const InputEvent& event);
  static void PreUpdate();

  static KeyActionType GetKey(int key);
};
}  // namespace evo_engine
