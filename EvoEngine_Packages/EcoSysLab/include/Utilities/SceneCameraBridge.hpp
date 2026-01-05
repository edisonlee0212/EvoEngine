#pragma once
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

namespace eco_sys_lab_package {
using SceneCameraApplyFn = void (*)(const glm::vec3& position, const glm::quat& rotation, bool set_fov, float fov);

void SetSceneCameraApplyFn(SceneCameraApplyFn fn);
void ApplySceneCamera(const glm::vec3& position, const glm::quat& rotation, bool set_fov, float fov);
}  // namespace eco_sys_lab_package
