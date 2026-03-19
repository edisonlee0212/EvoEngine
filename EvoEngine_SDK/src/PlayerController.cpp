#include "PlayerController.hpp"

#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "Input.hpp"
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
  
  float current_velocity = velocity;
  if (scene->GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
    current_velocity *= 5.0f;
  } else if (scene->GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
    current_velocity *= 0.2f;
  }

  if (scene->GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
    position += front * static_cast<float>(Times::DeltaTime()) * current_velocity;
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
    position -= front * static_cast<float>(Times::DeltaTime()) * current_velocity;
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
    position -= right * static_cast<float>(Times::DeltaTime()) * current_velocity;
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
    position += right * static_cast<float>(Times::DeltaTime()) * current_velocity;
    moved = true;
  }
  // Allow E/Q movement even when left mouse not held (already implemented)
  if (scene->GetKey(GLFW_KEY_E) == Input::KeyActionType::Hold) {
    position.y += current_velocity * static_cast<float>(Times::DeltaTime());
    moved = true;
  }
  if (scene->GetKey(GLFW_KEY_Q) == Input::KeyActionType::Hold) {
    position.y -= current_velocity * static_cast<float>(Times::DeltaTime());
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
  // Right mouse button rotation
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
  // Middle mouse drag: also rotate, with double sensitivity
  if (scene->GetKey(GLFW_MOUSE_BUTTON_MIDDLE) == Input::KeyActionType::Hold) {
    if (x_offset != 0 || y_offset != 0) {
      moved = true;
      const float middle_sensitivity = sensitivity * 2.0f;
      scene_camera_yaw_angle_ += x_offset * middle_sensitivity;
      scene_camera_pitch_angle_ += y_offset * middle_sensitivity;

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
void PlayerController::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "velocity" << YAML::Value << velocity;
  out << YAML::Key << "sensitivity" << YAML::Value << sensitivity;
  out << YAML::Key << "scene_camera_yaw_angle_" << YAML::Value << scene_camera_yaw_angle_;
  out << YAML::Key << "scene_camera_pitch_angle_" << YAML::Value << scene_camera_pitch_angle_;
}
void PlayerController::Deserialize(const YAML::Node& in) {
  if (in["velocity"]) 
    velocity = in["velocity"].as<float>();
  if (in["sensitivity"]) 
    sensitivity = in["sensitivity"].as<float>();
  if (in["scene_camera_yaw_angle_"]) 
    scene_camera_yaw_angle_ = in["scene_camera_yaw_angle_"].as<float>();
  if (in["scene_camera_pitch_angle_"]) 
    scene_camera_pitch_angle_ = in["scene_camera_pitch_angle_"].as<float>();
}
bool PlayerController::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragFloat("Velocity", &velocity, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Mouse sensitivity", &sensitivity, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Yaw angle", &scene_camera_yaw_angle_, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Pitch angle", &scene_camera_pitch_angle_, 0.01f))
    changed = true;

  return changed;
}