#include "Application.hpp"
#include "EditorDialogBridge.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "SceneCameraBridge.hpp"

namespace {
void ApplyEditorSceneCamera(const glm::vec3& position, const glm::quat& rotation, const bool set_fov, const float fov) {
  const auto editor_layer = evo_engine::ApplicationContext::Get().GetLayer<evo_engine::EditorLayer>();
  if (!editor_layer) {
    return;
  }
  editor_layer->SetSceneCameraPosition(position);
  editor_layer->SetSceneCameraRotation(rotation);
  if (set_fov) {
    if (const auto camera = editor_layer->GetSceneCamera()) {
      camera->camera_settings.fov = fov;
    }
  }
}

struct SceneCameraBridgeRegistration {
  SceneCameraBridgeRegistration() {
    eco_sys_lab_package::SetSceneCameraApplyFn(&ApplyEditorSceneCamera);
  }
};

const SceneCameraBridgeRegistration scene_camera_bridge_registration{};

void OpenEditorFile(const std::string& dialog_title, const std::string& file_type,
                    const std::vector<std::string>& extensions,
                    const std::function<void(const std::filesystem::path&)>& func, const bool project_dir_check) {
  evo_engine::EditorFileDialogs::OpenFile(dialog_title, file_type, extensions, func, project_dir_check);
}

void SaveEditorFile(const std::string& dialog_title, const std::string& file_type,
                    const std::vector<std::string>& extensions,
                    const std::function<void(const std::filesystem::path&)>& func, const bool project_dir_check) {
  evo_engine::EditorFileDialogs::SaveFile(dialog_title, file_type, extensions, func, project_dir_check);
}

struct EditorDialogBridgeRegistration {
  EditorDialogBridgeRegistration() {
    eco_sys_lab_package::SetEditorOpenFileFn(&OpenEditorFile);
    eco_sys_lab_package::SetEditorSaveFileFn(&SaveEditorFile);
  }
};

const EditorDialogBridgeRegistration editor_dialog_bridge_registration{};
}  // namespace
