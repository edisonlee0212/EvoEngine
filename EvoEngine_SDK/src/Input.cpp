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

void Input::MouseButtonCallBack(GLFWwindow* window, int button, int action, int mods) {
  auto& input = GetInstance();
  if (action == GLFW_PRESS) {
    input.pressed_keys_[button] = KeyActionType::Press;
    Dispatch({button, KeyActionType::Press});
  } else if (action == GLFW_RELEASE) {
    if (input.pressed_keys_.find(button) != input.pressed_keys_.end()) {
      // Dispatch hold if the key is already pressed.
      input.pressed_keys_.erase(button);
      Dispatch({button, KeyActionType::Release});
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
    glfwGetCursorPos(window_layer->GetGlfwWindow(), &x, &y);
    input.mouse_position_ = {x, y};
  }
}

glm::vec2 Input::GetMousePosition() {
  const auto& input = GetInstance();
  return input.mouse_position_;
}

KeyActionType Input::GetKey(const int key) {
  const auto& input = GetInstance();
  if (const auto search = input.pressed_keys_.find(key); search != input.pressed_keys_.end())
    return search->second;
  return KeyActionType::Release;
}
