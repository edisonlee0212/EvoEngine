#include "PlantModel.hpp"
using namespace eco_sys_lab_package;

void PlantModel::ApplyTropism(const glm::vec3& target_dir, float tropism, glm::vec3& front, glm::vec3& up) {
  const glm::vec3 dir = glm::normalize(target_dir);
  if (const float dot_p = glm::abs(glm::dot(front, dir)); dot_p < 0.99f && dot_p > -0.99f) {
    const glm::vec3 left = glm::cross(front, dir);
    const float max_angle = glm::acos(dot_p);
    const float rotate_angle = max_angle * tropism;
    const float angle = glm::min(max_angle, rotate_angle);
    const float c = glm::cos(angle), s = glm::sin(angle);
    const auto axis = glm::normalize(left);
    const auto temp = (1.f - c) * axis;
    // Same rotation coefficients as GLM, without multiplying a 4x4 identity matrix.
    glm::mat3 rotation;
    rotation[0] = {c + temp.x * axis.x, temp.x * axis.y + s * axis.z, temp.x * axis.z - s * axis.y};
    rotation[1] = {temp.y * axis.x - s * axis.z, c + temp.y * axis.y, temp.y * axis.z + s * axis.x};
    rotation[2] = {temp.z * axis.x + s * axis.y, temp.z * axis.y - s * axis.x, c + temp.z * axis.z};
    front = glm::normalize(rotation * front);
    up = glm::normalize(glm::cross(glm::cross(front, up), front));
  }
}

void PlantModel::ApplyTropism(const glm::vec3& target_dir, float tropism, glm::quat& rotation) {
  auto front = rotation * glm::vec3(0, 0, -1);
  auto up = rotation * glm::vec3(0, 1, 0);
  ApplyTropism(target_dir, tropism, front, up);
  rotation = glm::quatLookAt(front, up);
}
