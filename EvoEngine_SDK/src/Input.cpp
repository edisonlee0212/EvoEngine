#include "Input.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;

void Input::KeyCallBack(GLFWwindow* window, int key, int scan_code, int action, int mods) {
  auto& input = GetInstance();
  if (action == GLFW_PRESS) {
    input.pressed_keys_[key] = KeyActionType::Press;
    Dispatch({key, KeyActionType::Press});
  } else if (action == GLFW_RELEASE) {
    if (input.pressed_keys_.find(key) != input.pressed_keys_.end()) {
      // Dispatch hold if the key is already pressed.
      input.pressed_keys_.erase(key);
      Dispatch({key, KeyActionType::Release});
    }
  }
}

void Input::MouseButtonCallBack(GLFWwindow* window, const int button, const int action, int mods) {
  auto& input = GetInstance();
  if (action == GLFW_PRESS) {
    input.pressed_keys_[button] = KeyActionType::Press;
    Dispatch({button, KeyActionType::Press});

    // If middle or right mouse pressed -> start capture (hide + lock)
    if ((button == GLFW_MOUSE_BUTTON_MIDDLE || button == GLFW_MOUSE_BUTTON_RIGHT)) {
      // remember button request
      input.cursor_capture_buttons_.insert(button);
      if (!input.cursor_captured_) {
        if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
          double sx, sy;
          glfwGetCursorPos(window_layer->GetGlfwWindow(), &sx, &sy);
          input.saved_cursor_pos_ = glm::vec2(static_cast<float>(sx), static_cast<float>(sy));
          glfwSetInputMode(window_layer->GetGlfwWindow(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
          ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
          input.cursor_captured_ = true;
        }
      }
    }
  } else if (action == GLFW_RELEASE) {
    if (input.pressed_keys_.find(button) != input.pressed_keys_.end()) {
      // Dispatch hold if the key is already pressed.
      input.pressed_keys_.erase(button);
      Dispatch({button, KeyActionType::Release});
    }

    // stop capture for this button
    if ((button == GLFW_MOUSE_BUTTON_MIDDLE || button == GLFW_MOUSE_BUTTON_RIGHT)) {
      input.cursor_capture_buttons_.erase(button);
      if (input.cursor_capture_buttons_.empty() && input.cursor_captured_) {
        if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
          glfwSetInputMode(window_layer->GetGlfwWindow(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
          ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouseCursorChange;
          // restore cursor position
          double rx = input.saved_cursor_pos_.x;
          double ry = input.saved_cursor_pos_.y;
          glfwSetCursorPos(window_layer->GetGlfwWindow(), rx, ry);
          input.cursor_captured_ = false;
        }
      }
    }
  }
}

void Input::Dispatch(const InputEvent& event) {
  if (const auto& layers = Application::GetLayers(); !layers.empty()) {
    layers[0]->OnInputEvent(event);
  }
  if (!Application::GetLayer<EditorLayer>()) {
    const auto active_scene = Application::GetActiveScene();

    auto& scene_pressed_keys = active_scene->pressed_keys_;
    if (event.key_action == KeyActionType::Press) {
      scene_pressed_keys[event.key] = KeyActionType::Press;
    } else if (event.key_action == KeyActionType::Release) {
      if (scene_pressed_keys.find(event.key) != scene_pressed_keys.end()) {
        // Dispatch hold if the key is already pressed.
        scene_pressed_keys.erase(event.key);
      }
    }
  }
}

void Input::PreUpdate() {
  auto& input = GetInstance();
  input.mouse_position_ = {FLT_MIN, FLT_MIN};

  for (auto& i : input.pressed_keys_) {
    i.second = KeyActionType::Hold;
  }
  if (const auto scene = Application::GetActiveScene()) {
    for (auto& i : scene->pressed_keys_) {
      i.second = KeyActionType::Hold;
    }
  }
  if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
    glfwPollEvents();
    double x = FLT_MIN;
    double y = FLT_MIN;
    // If cursor is captured, glfwGetCursorPos returns virtual position; still use it as delta source.
    glfwGetCursorPos(window_layer->GetGlfwWindow(), &x, &y);
    input.mouse_position_ = {static_cast<float>(x), static_cast<float>(y)};
  }
}

glm::vec2 Input::GetMousePosition() {
  const auto& input = GetInstance();
  return input.mouse_position_;
}

Input::KeyActionType Input::GetKey(const int key) {
  const auto& input = GetInstance();
  if (const auto search = input.pressed_keys_.find(key); search != input.pressed_keys_.end())
    return search->second;
  return KeyActionType::Release;
}

bool Input::IsCursorCaptured() {
  return GetInstance().cursor_captured_;
}
