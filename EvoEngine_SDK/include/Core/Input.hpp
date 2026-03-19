#pragma once
#include "ISingleton.hpp"
#include <glm/glm.hpp>
#include <set>

namespace evo_engine {

/**
 * @class Input
 * @brief A final class responsible for handling input events such as keyboard and mouse actions in the engine.
 */
class Input final {
  EVOENGINE_SINGLETON_INSTANCE(Input)
 public:
  /**
   * @enum KeyActionType
   * @brief Represents the type of key actions such as Press, Hold, Release or Unknown.
   */
  enum class KeyActionType {
    Press,   /**< Key is pressed. */
    Hold,    /**< Key is held down. */
    Release, /**< Key is released. */
    Unknown  /**< Key action is unknown. */
  };

  /**
   * @struct InputEvent
   * @brief Represents an input event containing key and its action.
   */
  struct InputEvent {
    int key = GLFW_KEY_UNKNOWN;                        /**< The key associated with the input event. */
    KeyActionType key_action = KeyActionType::Unknown; /**< The action type of the key. */
  };

  /**
   * @brief Gets the current mouse position.
   * @return A glm::vec2 representing the mouse position on the screen.
   */
  static glm::vec2 GetMousePosition();

  /**
   * @brief Returns whether the cursor is currently captured (hidden/locked).
   */
  static bool IsCursorCaptured();

 private:
  friend class Platform;    /**< Grants Platform class access to private members of Input. */
  friend class Application; /**< Grants Application class access to private members of Input. */
  friend class EditorLayer; /**< Grants EditorLayer class access to private members of Input. */

  std::unordered_map<int, KeyActionType> pressed_keys_ = {}; /**< Stores the state of keys pressed. */
  glm::vec2 mouse_position_ = glm::vec2(0.0f);               /**< Stores the current mouse position. */

  // Cursor capture (hide / lock) support
  bool cursor_captured_ = false;             /**< Whether cursor is currently captured. */
  glm::vec2 saved_cursor_pos_ = glm::vec2(0);/**< Saved cursor position to restore after release. */
  std::set<int> cursor_capture_buttons_;     /**< Buttons that requested capture (MIDDLE/RIGHT). */

  /**
   * @brief Callback function to handle keyboard events.
   * @param window The GLFWwindow where the event occurred.
   * @param key The key code of the pressed key.
   * @param scan_code The system-specific scancode of the key.
   * @param action The action performed on the key (Press, Release, etc.).
   * @param mods Bit field describing which modifier keys were held down.
   */
  static void KeyCallBack(GLFWwindow* window, int key, int scan_code, int action, int mods);

  /**
   * @brief Callback function to handle mouse button events.
   * @param window The GLFWwindow where the event occurred.
   * @param button The mouse button that was acted upon.
   * @param action The action performed on the button (Press, Release, etc.).
   * @param mods Bit field describing which modifier keys were held down.
   */
  static void MouseButtonCallBack(GLFWwindow* window, const int button, const int action, int mods);

  /**
   * @brief Dispatches an InputEvent to the input system.
   * @param event The input event to be dispatched.
   */
  static void Dispatch(const InputEvent& event);

  /**
   * @brief Pre-update function to prepare the input handling system before each update loop.
   */
  static void PreUpdate();

  /**
   * @brief Retrieves the current action type for a given key.
   * @param key The key to query.
   * @return The KeyActionType representing the current state of the key.
   */
  static KeyActionType GetKey(int key);
};

}  // namespace evo_engine
