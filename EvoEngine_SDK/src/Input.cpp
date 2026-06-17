#include "Input.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;

namespace {
bool IsCursorCaptureButton(const int button) {
  return button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_MIDDLE;
}

enum class CameraViewport { None, Scene, Main };

CameraViewport ActiveCameraViewport() {
  const auto application = ApplicationContext::TryGet();
  if (!application) {
    return CameraViewport::None;
  }
  const auto editor_layer = application->GetLayer<EditorLayer>();
  if (!editor_layer) {
    return CameraViewport::None;
  }
  if (editor_layer->SceneCameraWindowHovered()) {
    return CameraViewport::Scene;
  }
  if (editor_layer->MainCameraWindowHovered()) {
    return CameraViewport::Main;
  }
  if (editor_layer->SceneCameraWindowFocused()) {
    return CameraViewport::Scene;
  }
  if (editor_layer->MainCameraWindowFocused()) {
    return CameraViewport::Main;
  }
  return CameraViewport::None;
}

}  // namespace

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
    const auto active_camera_viewport = ActiveCameraViewport();
    if (IsCursorCaptureButton(button) && (input.cursor_captured_ || active_camera_viewport != CameraViewport::None)) {
      if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        input.right_mouse_cursor_capture_ = true;
      } else {
        input.middle_mouse_cursor_capture_ = true;
      }
      if (!input.cursor_captured_ && window) {
        input.scene_camera_cursor_capture_ = active_camera_viewport == CameraViewport::Scene;
        input.main_camera_cursor_capture_ = active_camera_viewport == CameraViewport::Main;
        double x = 0.0;
        double y = 0.0;
        glfwGetCursorPos(window, &x, &y);
        input.saved_cursor_position_ = {x, y};
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        if (ImGui::GetCurrentContext()) {
          auto& io = ImGui::GetIO();
          io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
          io.MousePos = ImVec2(-FLT_MAX, -FLT_MAX);
        }
        input.cursor_captured_ = true;
      }
    }
  } else if (action == GLFW_RELEASE) {
    if (input.pressed_keys_.find(button) != input.pressed_keys_.end()) {
      // Dispatch hold if the key is already pressed.
      input.pressed_keys_.erase(button);
      Dispatch({button, KeyActionType::Release});
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
      input.right_mouse_cursor_capture_ = false;
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
      input.middle_mouse_cursor_capture_ = false;
    }
    if (input.cursor_captured_ && !input.right_mouse_cursor_capture_ && !input.middle_mouse_cursor_capture_ && window) {
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      if (ImGui::GetCurrentContext()) {
        ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouseCursorChange;
      }
      glfwSetCursorPos(window, input.saved_cursor_position_.x, input.saved_cursor_position_.y);
      input.scene_camera_cursor_capture_ = false;
      input.main_camera_cursor_capture_ = false;
      input.cursor_captured_ = false;
    }
  }
}

void Input::Dispatch(const InputEvent& event) {
  if (const auto& layers = ApplicationContext::Get().GetLayers(); !layers.empty()) {
    layers[0]->OnInputEvent(event);
  }
  if (!ApplicationContext::Get().GetLayer<EditorLayer>()) {
    const auto active_scene = ApplicationContext::Get().GetActiveScene();
    if (!active_scene) {
      return;
    }

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
  if (const auto scene = ApplicationContext::Get().GetActiveScene()) {
    for (auto& i : scene->pressed_keys_) {
      i.second = KeyActionType::Hold;
    }
  }
  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
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

Input::KeyActionType Input::GetKey(const int key) {
  const auto& input = GetInstance();
  if (const auto search = input.pressed_keys_.find(key); search != input.pressed_keys_.end())
    return search->second;
  return KeyActionType::Release;
}
