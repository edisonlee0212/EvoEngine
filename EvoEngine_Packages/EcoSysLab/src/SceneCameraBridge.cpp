#include "SceneCameraBridge.hpp"

namespace eco_sys_lab_package {
namespace {
SceneCameraApplyFn scene_camera_apply_fn = nullptr;
}

void SetSceneCameraApplyFn(const SceneCameraApplyFn fn) {
  scene_camera_apply_fn = fn;
}

void ApplySceneCamera(const glm::vec3& position, const glm::quat& rotation, const bool set_fov, const float fov) {
  if (scene_camera_apply_fn) {
    scene_camera_apply_fn(position, rotation, set_fov, fov);
  }
}
}  // namespace eco_sys_lab_package
