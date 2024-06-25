//
// Created by lllll on 8/16/2021.
//

#include "ObjectRotator.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "Transform.hpp"
using namespace eco_sys_lab;
void ObjectRotator::FixedUpdate() {
  auto scene = GetScene();
  auto transform = scene->GetDataComponent<Transform>(GetOwner());
  rotation.y += Times::FixedDeltaTime() * rotate_speed;
  transform.SetEulerRotation(glm::radians(rotation));
  scene->SetDataComponent(GetOwner(), transform);
}

bool ObjectRotator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragFloat("Speed", &rotate_speed);
  ImGui::DragFloat3("Rotation", &rotation.x);
  return false;
}

void ObjectRotator::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "rotate_speed" << YAML::Value << rotate_speed;
  out << YAML::Key << "rotation" << YAML::Value << rotation;
}

void ObjectRotator::Deserialize(const YAML::Node& in) {
  if (in["rotate_speed"])
    rotate_speed = in["rotate_speed"].as<float>();
  if (in["rotation"])
    rotation = in["m_rotation"].as<glm::vec3>();
}
