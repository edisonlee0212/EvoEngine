//
// Created by lllll on 8/16/2021.
//

#include "EcoSysLabSerializationAdapters.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "Transform.hpp"
using namespace eco_sys_lab_package;

void ObjectRotator::FixedUpdate() {
  auto scene = GetScene();
  auto transform = scene->GetDataComponent<Transform>(GetOwner());
  rotation.y += ApplicationContext::Get().GetTimes().FixedDeltaTime() * rotate_speed;
  transform.SetEulerRotation(glm::radians(rotation));
  scene->SetDataComponent(GetOwner(), transform);
}

bool ObjectRotator::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragFloat("Speed", &rotate_speed);
  ImGui::DragFloat3("Rotation", &rotation.x);
  return false;
}

void eco_sys_lab_package::SerializeObjectRotator(YAML::Emitter& out, const ObjectRotator& target) {
  out << YAML::Key << "rotate_speed" << YAML::Value << target.rotate_speed;
  out << YAML::Key << "rotation" << YAML::Value << target.rotation;
}

void eco_sys_lab_package::DeserializeObjectRotator(const YAML::Node& in, ObjectRotator& target) {
  if (in["rotate_speed"])
    target.rotate_speed = in["rotate_speed"].as<float>();
  if (in["rotation"])
    target.rotation = in["m_rotation"].as<glm::vec3>();
}
