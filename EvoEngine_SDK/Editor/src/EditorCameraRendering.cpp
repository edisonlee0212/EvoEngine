#include "EditorLayer.hpp"
#include "RenderLayer.hpp"

using namespace evo_engine;

void EditorLayer::RegisterCameraProvider() {
  const auto render = GetApplication().GetLayer<RenderLayer>();
  if (!render)
    return;
  RenderLayer::AuxiliaryCameraProvider provider;
  provider.collect = [this](std::vector<RenderLayer::CameraView>& cameras) {
    for (const auto& [handle, view] : editor_cameras_) {
      if (!view.camera || !view.camera->IsEnabled())
        continue;
      GlobalTransform transform;
      transform.SetValue(view.position, view.rotation, glm::vec3(1));
      cameras.emplace_back(transform, view.camera);
    }
  };
  provider.primary = [this] {
    GlobalTransform transform;
    const auto camera = GetSceneCamera();
    if (camera)
      transform.SetValue(GetSceneCameraPosition(), GetSceneCameraRotation(), glm::vec3(1));
    return RenderLayer::CameraView{transform, camera};
  };
  render->SetAuxiliaryCameraProvider(std::move(provider));
}
