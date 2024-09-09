#pragma once
#include "Application.hpp"
#include "Camera.hpp"
#include "Entity.hpp"
#include "GraphicsResources.hpp"
#include "ILayer.hpp"
#include "ISystem.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "PrivateComponentRef.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Strands.hpp"
#include "Texture2D.hpp"
namespace evo_engine {
enum class ConsoleMessageType { Log, Warning, Error };
struct ConsoleMessage {
  ConsoleMessageType m_type = ConsoleMessageType::Log;
  std::string m_value;
  double m_time = 0;
};

struct GizmoSettings {
  DrawSettings draw_settings;
  enum class ColorMode { Default, VertexColor, NormalColor } color_mode = ColorMode::Default;
  bool depth_test = false;
  bool depth_write = false;
  void ApplySettings(GraphicsPipelineStates& global_pipeline_state) const;
};
struct GizmosPushConstant {
  glm::mat4 model;
  glm::vec4 color;
  float size;
  int32_t camera_index;
};

struct EditorCamera {
  glm::quat rotation = glm::quat(glm::radians(glm::vec3(0.0f, 0.0f, 0.0f)));
  glm::vec3 position = glm::vec3(0, 2, 5);
  std::shared_ptr<Camera> camera;
};

struct GizmoMeshTask {
  std::shared_ptr<Mesh> mesh;
  std::shared_ptr<Camera> editor_camera_component;
  glm::vec4 color;
  glm::mat4 model;
  float size;
  GizmoSettings gizmo_settings;
};

struct GizmoInstancedMeshTask {
  std::shared_ptr<Mesh> mesh;
  std::shared_ptr<Camera> editor_camera_component;
  std::shared_ptr<ParticleInfoList> instanced_data;
  glm::mat4 model;
  float size;
  GizmoSettings gizmo_settings;
};

struct GizmoStrandsTask {
  std::shared_ptr<Strands> m_strands;
  std::shared_ptr<Camera> editor_camera_component;
  glm::vec4 color;
  glm::mat4 model;
  float m_size;
  GizmoSettings gizmo_settings;
};

class EditorLayer : public ILayer {
  void LoadIcons();
  void OnCreate() override;
  void OnDestroy() override;
  void PreUpdate() override;
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void LateUpdate() override;

  void SceneCameraWindow();
  void MainCameraWindow();
  void OnInputEvent(const Input::InputEvent& input_event) override;
  void ResizeCameras();
  Handle scene_camera_handle_ = 0;
  std::unordered_map<Handle, EditorCamera> editor_cameras_;

  std::vector<GizmoMeshTask> gizmo_mesh_tasks_;
  std::vector<GizmoInstancedMeshTask> gizmo_instanced_mesh_tasks_;
  std::vector<GizmoStrandsTask> gizmo_strands_tasks_;

  std::vector<ConsoleMessage> console_messages_;
  std::mutex console_message_mutex_;

  bool enable_console_logs_ = true;
  bool enable_console_errors_ = true;
  bool enable_console_warnings_ = true;
  friend class Console;

 public:
  bool show_console_window = true;
  std::vector<ConsoleMessage>& GetConsoleMessages();

  [[nodiscard]] bool SceneCameraWindowFocused() const;
  [[nodiscard]] bool MainCameraWindowFocused() const;
  bool enable_view_gizmos = true;
  bool enable_gizmos = true;
  bool transform_read_only = false;

  void RegisterEditorCamera(const std::shared_ptr<Camera>& camera);

  [[nodiscard]] glm::vec2 GetMouseSceneCameraPosition() const;

  [[nodiscard]] static Input::KeyActionType GetKey(int key);
  [[nodiscard]] std::shared_ptr<Camera> GetSceneCamera();
  [[nodiscard]] glm::vec3 GetSceneCameraPosition() const;
  [[nodiscard]] glm::quat GetSceneCameraRotation() const;

  void SetCameraPosition(const std::shared_ptr<Camera>& camera, const glm::vec3& target_position);
  void SetCameraRotation(const std::shared_ptr<Camera>& camera, const glm::quat& target_rotation);
  void MoveCamera(const glm::quat& target_rotation, const glm::vec3& target_position,
                  const float& transition_time = 1.0f);

  static auto UpdateTextureId(ImTextureID& target, VkSampler image_sampler, VkImageView image_view,
                              VkImageLayout image_layout) -> void;
  [[nodiscard]] Entity GetSelectedEntity() const;
  void SetSelectedEntity(const Entity& entity, bool open_menu = true);
  float scene_camera_resolution_multiplier = 1.0f;
  [[nodiscard]] bool GetLockEntitySelection() const;

  void SetLockEntitySelection(bool value);
  bool show_scene_window = true;
  bool show_camera_window = true;
  bool show_camera_info = false;
  bool show_scene_info = true;

  bool show_entity_explorer_window = true;
  bool show_entity_inspector_window = true;

  bool main_camera_focus_override = false;
  bool scene_camera_focus_override = false;

  int selected_hierarchy_display_mode = 1;
  float velocity = 10.0f;
  float sensitivity = 0.1f;
  bool apply_transform_to_main_camera = false;
  bool lock_camera;

  glm::quat default_scene_camera_rotation = glm::quat(glm::radians(glm::vec3(0.0f, 0.0f, 0.0f)));
  glm::vec3 default_scene_camera_position = glm::vec3(0, 2, 5);

  int main_camera_resolution_x = 1;
  int main_camera_resolution_y = 1;
  bool main_camera_allow_auto_resize = true;

  glm::vec3& UnsafeGetPreviouslyStoredPosition();

  glm::vec3& UnsafeGetPreviouslyStoredRotation();

  glm::vec3& UnsafeGetPreviouslyStoredScale();

  [[nodiscard]] bool LocalPositionSelected() const;

  [[nodiscard]] bool LocalRotationSelected() const;

  [[nodiscard]] bool LocalScaleSelected() const;

#pragma region ImGui Helpers
  void CameraWindowDragAndDrop() const;
  [[nodiscard]] bool DrawEntityMenu(const bool& enabled, const Entity& entity) const;
  void DrawEntityNode(const Entity& entity, const unsigned& hierarchy_level);
  void InspectComponentData(Entity entity, IDataComponent* data, const DataComponentType& type, bool is_root);

  std::map<std::string, std::shared_ptr<Texture2D>>& AssetIcons();

  template <typename T1 = IDataComponent>
  void RegisterComponentDataInspector(
      const std::function<bool(Entity entity, IDataComponent* data, bool is_root)>& func);

  static bool DragAndDropButton(AssetRef& target, const std::string& name,
                                const std::vector<std::string>& acceptable_type_names, bool modifiable = true);
  static bool DragAndDropButton(PrivateComponentRef& target, const std::string& name,
                                const std::vector<std::string>& acceptable_type_names, bool modifiable = true);

  template <typename T = IAsset>
  static bool DragAndDropButton(AssetRef& target, const std::string& name, bool modifiable = true);
  template <typename T = IPrivateComponent>
  static bool DragAndDropButton(PrivateComponentRef& target, const std::string& name, bool modifiable = true);
  static bool DragAndDropButton(EntityRef& entity_ref, const std::string& name, bool modifiable = true);

  template <typename T = IAsset>
  static void Draggable(AssetRef& target);
  template <typename T = IPrivateComponent>
  static void Draggable(PrivateComponentRef& target);
  static void Draggable(EntityRef& entity_ref);

  template <typename T = IAsset>
  static void DraggableAsset(const std::shared_ptr<T>& target);
  template <typename T = IPrivateComponent>
  static void DraggablePrivateComponent(const std::shared_ptr<T>& target);
  static void DraggableEntity(const Entity& entity);

  static bool UnsafeDroppableAsset(AssetRef& target, const std::vector<std::string>& type_names);
  static bool UnsafeDroppablePrivateComponent(PrivateComponentRef& target, const std::vector<std::string>& type_names);

  template <typename T = IAsset>
  static bool Droppable(AssetRef& target);
  template <typename T = IPrivateComponent>
  static bool Droppable(PrivateComponentRef& target);
  static bool Droppable(EntityRef& entity_ref);

  template <typename T = IAsset>
  static bool Rename(AssetRef& target);
  static bool Rename(EntityRef& entity_ref);

  template <typename T = IAsset>
  static bool RenameAsset(const std::shared_ptr<T>& target);
  [[nodiscard]] static bool RenameEntity(const Entity& entity);

  template <typename T = IAsset>
  static bool Remove(AssetRef& target);
  template <typename T = IPrivateComponent>
  static bool Remove(PrivateComponentRef& target);
  [[nodiscard]] static bool Remove(EntityRef& entity_ref);

#pragma endregion

#pragma region Gizmos
  void DrawGizmoMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Camera>& editor_camera_component,
                     const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                     const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  void DrawGizmoStrands(const std::shared_ptr<Strands>& strands, const std::shared_ptr<Camera>& editor_camera_component,
                        const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                        const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  void DrawGizmoMeshInstancedColored(const std::shared_ptr<Mesh>& mesh,
                                     const std::shared_ptr<Camera>& editor_camera_component,
                                     const std::shared_ptr<ParticleInfoList>& instanced_data,
                                     const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                                     const GizmoSettings& gizmo_settings = {});

  void DrawGizmoMeshInstancedColored(const std::shared_ptr<Mesh>& mesh,
                                     const std::shared_ptr<ParticleInfoList>& instanced_data,
                                     const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                                     const GizmoSettings& gizmo_settings = {});

  void DrawGizmoMesh(const std::shared_ptr<Mesh>& mesh, const glm::vec4& color = glm::vec4(1.0f),
                     const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                     const GizmoSettings& gizmo_settings = {});

  void DrawGizmoStrands(const std::shared_ptr<Strands>& strands, const glm::vec4& color = glm::vec4(1.0f),
                        const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                        const GizmoSettings& gizmo_settings = {});

  void DrawGizmoCubes(const std::shared_ptr<ParticleInfoList>& instanced_data, const glm::mat4& model = glm::mat4(1.0f),
                      const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  void DrawGizmoCube(const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                     const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  void DrawGizmoSpheres(const std::shared_ptr<ParticleInfoList>& instanced_data,
                        const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                        const GizmoSettings& gizmo_settings = {});

  void DrawGizmoSphere(const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                       const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  void DrawGizmoCylinders(const std::shared_ptr<ParticleInfoList>& instanced_data,
                          const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                          const GizmoSettings& gizmo_settings = {});

  void DrawGizmoCylinder(const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                         const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});
#pragma endregion
 private:
  int selection_alpha_ = 0;
  bool using_gizmo_ = false;
  glm::detail::hdata* mapped_entity_index_data_;
  std::unique_ptr<Buffer> entity_index_read_buffer_;
  void MouseEntitySelection();
  [[nodiscard]] Entity MouseEntitySelection(const std::shared_ptr<Camera>& target_camera,
                                            const glm::vec2& mouse_position) const;

  EntityArchetype basic_entity_archetype_;
  Transform previously_stored_transform_;
  glm::vec3 previously_stored_position_;
  glm::vec3 previously_stored_rotation_;
  glm::vec3 previously_stored_scale_;
  bool local_position_selected_ = true;
  bool local_rotation_selected_ = false;
  bool local_scale_selected_ = false;

  bool scene_camera_window_focused_ = false;
  bool main_camera_window_focused_ = false;
#pragma region Registrations
  friend class ClassRegistry;
  friend class RenderLayer;
  friend class Application;
  friend class ProjectManager;
  friend class Scene;
  std::map<std::string, std::shared_ptr<Texture2D>> assets_icons_;
  std::map<size_t, std::function<bool(Entity entity, IDataComponent* data, bool is_root)>>
      component_data_inspector_map_;

  std::vector<std::weak_ptr<AssetRecord>> asset_record_bus_;
  std::map<std::string, std::vector<AssetRef>> asset_ref_bus_;
  std::map<std::string, std::vector<PrivateComponentRef>> private_component_ref_bus_;
  std::map<std::string, std::vector<EntityRef>> entity_ref_bus_;
#pragma endregion
#pragma region Transfer

  glm::quat previous_rotation_;
  glm::vec3 previous_position_;
  glm::quat target_rotation_;
  glm::vec3 target_position_;
  float transition_time_;
  float transition_timer_;
#pragma endregion
  std::vector<Entity> selected_entity_hierarchy_list_;

  int scene_camera_resolution_x_ = 1;
  int scene_camera_resolution_y_ = 1;

  bool lock_entity_selection_ = false;

  bool highlight_selection_ = true;

  Entity selected_entity_;

  glm::vec2 mouse_scene_window_position_;
  glm::vec2 mouse_camera_window_position_;

  float main_camera_resolution_multiplier_ = 1.0f;
};

#pragma region ImGui Helpers

template <typename T1>
void EditorLayer::RegisterComponentDataInspector(
    const std::function<bool(Entity entity, IDataComponent* data, bool is_root)>& func) {
  component_data_inspector_map_.insert_or_assign(typeid(T1).hash_code(), func);
}

template <typename T>
bool EditorLayer::DragAndDropButton(AssetRef& target, const std::string& name, bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  const auto ptr = target.Get<IAsset>();
  bool status_changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.5f, 0, 1));
  if (ptr) {
    const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
    ImGui::Button((ptr->GetTitle() + tag).c_str());
    Draggable(target);
    if (modifiable) {
      status_changed = Rename(target);
      status_changed = Remove(target) || status_changed;
    }
    if (!status_changed && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
      ProjectManager::GetInstance().inspecting_asset = ptr;
    }
  } else {
    ImGui::Button("none");
  }
  ImGui::PopStyleColor(1);
  status_changed = Droppable<T>(target) || status_changed;
  return status_changed;
}

template <typename T>
bool EditorLayer::DragAndDropButton(PrivateComponentRef& target, const std::string& name, const bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  bool status_changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.3f, 0, 1));
  if (const auto ptr = target.Get<IPrivateComponent>()) {
    const auto scene = Application::GetActiveScene();
    if (!scene->IsEntityValid(ptr->GetOwner())) {
      target.Clear();
      ImGui::Button("none");
      return true;
    }
    ImGui::Button(scene->GetEntityName(ptr->GetOwner()).c_str());
    Draggable(target);
    if (modifiable) {
      status_changed = Remove(target) || status_changed;
    }
  } else {
    ImGui::Button("none");
  }
  ImGui::PopStyleColor(1);
  status_changed = Droppable<T>(target) || status_changed;
  return status_changed;
}
template <typename T>
void EditorLayer::DraggablePrivateComponent(const std::shared_ptr<T>& target) {
  if (const auto ptr = std::dynamic_pointer_cast<IPrivateComponent>(target)) {
    const auto type = ptr->GetTypeName();
    auto entity = ptr->GetOwner();
    if (const auto scene = Application::GetActiveScene(); scene->IsEntityValid(entity)) {
      if (ImGui::BeginDragDropSource()) {
        auto handle = scene->GetEntityHandle(entity);
        ImGui::SetDragDropPayload("PrivateComponent", &handle, sizeof(Handle));
        ImGui::TextColored(ImVec4(0, 0, 1, 1), type.c_str());
        ImGui::EndDragDropSource();
      }
    }
  }
}
template <typename T>
void EditorLayer::DraggableAsset(const std::shared_ptr<T>& target) {
  if (ImGui::BeginDragDropSource()) {
    const auto ptr = std::dynamic_pointer_cast<IAsset>(target);
    if (ptr) {
      const auto title = ptr->GetTitle();
      ImGui::SetDragDropPayload("Asset", &ptr->handle_, sizeof(Handle));
      ImGui::TextColored(ImVec4(0, 0, 1, 1), title.c_str());
    }
    ImGui::EndDragDropSource();
  }
}
template <typename T>
void EditorLayer::Draggable(AssetRef& target) {
  DraggableAsset(target.Get<IAsset>());
}
template <typename T>
bool EditorLayer::Droppable(AssetRef& target) {
  return UnsafeDroppableAsset(target, {Serialization::GetSerializableTypeName<T>()});
}

template <typename T>
bool EditorLayer::Rename(AssetRef& target) {
  return RenameAsset(target.Get<IAsset>());
}
template <typename T>
bool EditorLayer::Remove(AssetRef& target) {
  bool status_changed = false;
  if (const auto ptr = target.Get<IAsset>()) {
    const std::string type = ptr->GetTypeName();
    const std::string tag = "##" + type + std::to_string(ptr->GetHandle());
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::Button(("Remove" + tag).c_str())) {
        target.Clear();
        status_changed = true;
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}
template <typename T>
bool EditorLayer::Remove(PrivateComponentRef& target) {
  bool status_changed = false;
  if (const auto ptr = target.Get<IPrivateComponent>()) {
    const std::string type = ptr->GetTypeName();
    const std::string tag = "##" + type + std::to_string(ptr->GetHandle());
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::Button(("Remove" + tag).c_str())) {
        target.Clear();
        status_changed = true;
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}

template <typename T>
bool EditorLayer::RenameAsset(const std::shared_ptr<T>& target) {
  constexpr bool status_changed = false;
  auto ptr = std::dynamic_pointer_cast<IAsset>(target);
  const std::string type = ptr->GetTypeName();
  const std::string tag = "##" + type + std::to_string(ptr->GetHandle());
  if (ImGui::BeginPopupContextItem(tag.c_str())) {
    if (!ptr->IsTemporary()) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256];
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          if (bool succeed = ptr->SetPathAndSave(ptr->GetProjectRelativePath().replace_filename(
                  std::string(new_name) + ptr->GetAssetRecord().lock()->GetAssetExtension())))
            memset(new_name, 0, 256);
        }
        ImGui::EndMenu();
      }
    }
    ImGui::EndPopup();
  }
  return status_changed;
}
template <typename T>
void EditorLayer::Draggable(PrivateComponentRef& target) {
  DraggablePrivateComponent(target.Get<IPrivateComponent>());
}

template <typename T>
bool EditorLayer::Droppable(PrivateComponentRef& target) {
  return UnsafeDroppablePrivateComponent(target, {Serialization::GetSerializableTypeName<T>()});
}

#pragma endregion
}  // namespace evo_engine
