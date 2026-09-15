#include "UniverseEditorLayer.hpp"
#include "EditorLayer.hpp"
#include "UniverseLayer.hpp"
using namespace evo_engine;
using namespace universe_package;

void UniverseEditorLayer::OnCreate() {
  enable_inspection = false;
  if (const auto runtime = GetApplication().GetLayer<UniverseLayer>();
      runtime && GetApplication().GetLayer<EditorLayer>())
    runtime->SetViewHost(std::dynamic_pointer_cast<UniverseViewHost>(GetSelf()));
}
void UniverseEditorLayer::OnDestroy() {
  if (const auto runtime = GetApplication().GetLayer<UniverseLayer>())
    runtime->SetViewHost(nullptr);
  demo_camera_.Restore();
}
UniverseViewportInput UniverseEditorLayer::GetViewportInput(const std::shared_ptr<Scene>& scene) const {
  UniverseViewportInput result;
  const auto editor = GetApplication().GetLayer<EditorLayer>();
  if (!editor)
    return result;
  const bool use_scene = editor->GetSceneViewportInput().focused;
  const auto& input = use_scene ? editor->GetSceneViewportInput() : editor->GetMainCameraViewportInput();
  result.camera = use_scene ? editor->GetSceneCamera() : scene->main_camera.Get<Camera>();
  result.camera_name = use_scene ? "Scene camera" : "Main camera";
  result.request.cursor_uv = input.cursor_uv;
  result.request.display_size = input.image_size;
  result.request.image_origin = input.image_origin;
  result.request.valid =
      input.visible && input.cursor_valid && input.scene.lock() == scene && input.camera.lock() == result.camera;
  result.click_sequence = input.click_sequence;
  result.follow_toggle_sequence = input.follow_toggle_sequence;
  return result;
}
std::shared_ptr<Camera> UniverseEditorLayer::GetCamera() const {
  const auto editor = GetApplication().GetLayer<EditorLayer>();
  return editor ? editor->GetSceneCamera() : nullptr;
}
StarFollowCameraPose UniverseEditorLayer::GetPose() const {
  if (const auto editor = GetApplication().GetLayer<EditorLayer>())
    return {glm::dvec3(editor->GetSceneCameraPosition()), glm::dquat(editor->GetSceneCameraRotation()), true};
  return {};
}
void UniverseEditorLayer::Move(const StarFollowCameraPose& pose) {
  if (const auto editor = GetApplication().GetLayer<EditorLayer>(); editor && pose.valid)
    editor->MoveCamera(glm::quat(pose.rotation), glm::vec3(pose.position));
}
void UniverseEditorLayer::Rebase(const glm::dmat4& transform) {
  if (const auto editor = GetApplication().GetLayer<EditorLayer>())
    editor->RebaseSceneCamera(transform);
}
void UniverseEditorLayer::SetDemoCamera(const bool enabled) {
  if (enabled)
    demo_camera_.Apply(GetCamera());
  else
    demo_camera_.Restore();
}
