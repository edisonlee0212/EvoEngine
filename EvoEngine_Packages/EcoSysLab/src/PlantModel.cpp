#include "PlantModel.hpp"
using namespace eco_sys_lab_package;

void PlantModel::ApplyTropism(const glm::vec3& target_dir, float tropism, glm::vec3& front, glm::vec3& up) {
  const glm::vec3 dir = glm::normalize(target_dir);
  if (const float dot_p = glm::abs(glm::dot(front, dir)); dot_p < 0.99f && dot_p > -0.99f) {
    const glm::vec3 left = glm::cross(front, dir);
    const float max_angle = glm::acos(dot_p);
    const float rotate_angle = max_angle * tropism;
    front = glm::normalize(glm::rotate(front, glm::min(max_angle, rotate_angle), left));
    up = glm::normalize(glm::cross(glm::cross(front, up), front));
  }
}

void PlantModel::ApplyTropism(const glm::vec3& target_dir, float tropism, glm::quat& rotation) {
  auto front = rotation * glm::vec3(0, 0, -1);
  auto up = rotation * glm::vec3(0, 1, 0);
  ApplyTropism(target_dir, tropism, front, up);
  rotation = glm::quatLookAt(front, up);
}