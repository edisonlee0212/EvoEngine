#include "PlayerController.hpp"
#include "Application.hpp"

#include "Camera.hpp"
#include "Scene.hpp"
#include "Times.hpp"
using namespace evo_engine;

void PlayerController::OnCreate() {
  start_mouse_ = false;
}
void PlayerController::LateUpdate() {
  const auto scene = GetScene();

#pragma region Scene Camera Controller
  auto transform = scene->GetDataComponent<Transform>(GetOwner());
  const auto rotation = transform.GetRotation();
  auto position = transform.GetPosition();
  const auto front = rotation * glm::vec3(0, 0, -1);
  const auto right = rotation * glm::vec3(1, 0, 0);
  auto moved = false;
  if (scene->GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
    position += front * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
    position -= front * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
    position -= right * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
    position += right * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
    position.y += velocity * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime());
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
    position.y -= velocity * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime());
    moved = true;
  }
  if (moved) {
    transform.SetPosition(position);
  }
  const glm::vec2 mouse_position = Input::GetMousePosition();
  float x_offset = 0;
  float y_offset = 0;
  if (mouse_position.x > FLT_MIN) {
    if (!start_mouse_) {
      last_x_ = mouse_position.x;
      last_y_ = mouse_position.y;
      start_mouse_ = true;
    }
    x_offset = mouse_position.x - last_x_;
    y_offset = -mouse_position.y + last_y_;
    last_x_ = mouse_position.x;
    last_y_ = mouse_position.y;
  }
  if (scene->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Hold) {
    if (x_offset != 0 || y_offset != 0) {
      moved = true;
      scene_camera_yaw_angle_ += x_offset * sensitivity;
      scene_camera_pitch_angle_ += y_offset * sensitivity;

      if (scene_camera_pitch_angle_ > 89.0f)
        scene_camera_pitch_angle_ = 89.0f;
      if (scene_camera_pitch_angle_ < -89.0f)
        scene_camera_pitch_angle_ = -89.0f;

      transform.SetRotation(Camera::ProcessMouseMovement(scene_camera_yaw_angle_, scene_camera_pitch_angle_, false));
    }
  }
  if (moved) {
    scene->SetDataComponent(GetOwner(), transform);
  }
#pragma endregion
}

void PlayerController::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
float& PlayerController::RefSceneCameraYawAngle() {
  return scene_camera_yaw_angle_;
}

float& PlayerController::RefSceneCameraPitchAngle() {
  return scene_camera_pitch_angle_;
}

float PlayerController::GetSceneCameraYawAngle() const {
  return scene_camera_yaw_angle_;
}

float PlayerController::GetSceneCameraPitchAngle() const {
  return scene_camera_pitch_angle_;
}
