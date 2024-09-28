#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Cubemap.hpp"
#include "EnvironmentalMap.hpp"
#include "Platform.hpp"
#include "ILayer.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "StrandsRenderer.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;
void EditorLayer::OnCreate() {
  if (!Application::GetLayer<WindowLayer>()) {
    throw std::runtime_error("EditorLayer requires WindowLayer!");
  }

  basic_entity_archetype_ = Entities::CreateEntityArchetype("General", GlobalTransform(), Transform());
  RegisterComponentDataInspector<GlobalTransform>([](Entity, IDataComponent* data, bool) {
    const auto* ltw = reinterpret_cast<GlobalTransform*>(data);
    glm::vec3 er;
    glm::vec3 t;
    glm::vec3 s;
    ltw->Decompose(t, er, s);
    er = glm::degrees(er);
    ImGui::DragFloat3("Position##Global", &t.x, 0.1f, 0, 0, "%.3f", ImGuiSliderFlags_ReadOnly);
    ImGui::DragFloat3("Rotation##Global", &er.x, 0.1f, 0, 0, "%.3f", ImGuiSliderFlags_ReadOnly);
    ImGui::DragFloat3("Scale##Global", &s.x, 0.1f, 0, 0, "%.3f", ImGuiSliderFlags_ReadOnly);
    return false;
  });
  RegisterComponentDataInspector<Transform>([&](const Entity entity, IDataComponent* data, bool) {
    static Entity previous_entity{};
    auto* ltp = static_cast<Transform*>(static_cast<void*>(data));
    bool edited = false;
    const auto scene = Application::GetActiveScene();
    const auto status = scene->GetDataComponent<TransformUpdateFlag>(entity);
    const bool reload = previous_entity != entity || previously_stored_transform_.value != ltp->value ||
                        status.transform_modified || status.global_transform_modified;
    if (reload) {
      previous_entity = entity;
      ltp->Decompose(previously_stored_position_, previously_stored_rotation_, previously_stored_scale_);
      previously_stored_rotation_ = glm::degrees(previously_stored_rotation_);
      local_position_selected_ = true;
      local_rotation_selected_ = false;
      local_scale_selected_ = false;

      previously_stored_transform_ = *ltp;
    }
    if (ImGui::DragFloat3("##LocalPosition", &previously_stored_position_.x, 0.01f, 0, 0, "%.3f",
                          reload ? ImGuiSliderFlags_ReadOnly : 0))
      edited = true;
    ImGui::SameLine();
    if (ImGui::Selectable("Position##Local", &local_position_selected_) && local_position_selected_) {
      local_rotation_selected_ = false;
      local_scale_selected_ = false;
    }
    if (ImGui::DragFloat3("##LocalRotation", &previously_stored_rotation_.x, 1.0f, 0, 0, "%.3f",
                          reload ? ImGuiSliderFlags_ReadOnly : 0))
      edited = true;
    ImGui::SameLine();
    if (ImGui::Selectable("Rotation##Local", &local_rotation_selected_) && local_rotation_selected_) {
      local_position_selected_ = false;
      local_scale_selected_ = false;
    }
    if (ImGui::DragFloat3("##LocalScale", &previously_stored_scale_.x, 0.01f, 0, 0, "%.3f",
                          reload ? ImGuiSliderFlags_ReadOnly : 0))
      edited = true;
    ImGui::SameLine();
    if (ImGui::Selectable("Scale##Local", &local_scale_selected_) && local_scale_selected_) {
      local_rotation_selected_ = false;
      local_position_selected_ = false;
    }
    if (edited) {
      ltp->value = glm::translate(previously_stored_position_) *
                   glm::mat4_cast(glm::quat(glm::radians(previously_stored_rotation_))) *
                   glm::scale(previously_stored_scale_);
      previously_stored_transform_ = *ltp;
    }
    transform_read_only = false;
    return edited;
  });

  RegisterComponentDataInspector<Ray>([&](Entity, IDataComponent* data, bool) {
    auto* ray = static_cast<Ray*>(static_cast<void*>(data));
    bool changed = false;
    if (ImGui::InputFloat3("Start", &ray->start.x))
      changed = true;
    if (ImGui::InputFloat3("Direction", &ray->direction.x))
      changed = true;
    if (ImGui::InputFloat("Length", &ray->length))
      changed = true;
    return changed;
  });

  LoadIcons();

  VkBufferCreateInfo entity_index_read_buffer{};
  entity_index_read_buffer.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  entity_index_read_buffer.size = sizeof(glm::detail::hdata) * 4;
  entity_index_read_buffer.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  entity_index_read_buffer.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo entity_index_read_buffer_create_info{};
  entity_index_read_buffer_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  entity_index_read_buffer_create_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  entity_index_read_buffer_ = std::make_unique<Buffer>(entity_index_read_buffer, entity_index_read_buffer_create_info);
  vmaMapMemory(Platform::GetVmaAllocator(), entity_index_read_buffer_->GetVmaAllocation(),
               static_cast<void**>(static_cast<void*>(&mapped_entity_index_data_)));

  const auto scene_camera = Serialization::ProduceSerializable<Camera>();
  scene_camera->clear_color = glm::vec3(59.0f / 255.0f, 85 / 255.0f, 143 / 255.f);
  scene_camera->use_clear_color = false;
  scene_camera->OnCreate();
  RegisterEditorCamera(scene_camera);
  scene_camera_handle_ = scene_camera->GetHandle();
}

void EditorLayer::OnDestroy() {
  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

void EditorLayer::PreUpdate() {
  gizmo_mesh_tasks_.clear();
  gizmo_instanced_mesh_tasks_.clear();
  gizmo_strands_tasks_.clear();

  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();

#pragma region Dock
  static bool opt_fullscreen_persistent = true;
  bool opt_fullscreen = opt_fullscreen_persistent;
  static ImGuiDockNodeFlags dock_space_flags = ImGuiDockNodeFlags_None;

  // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
  // because it would be confusing to have two docking targets within each others.
  ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
  if (opt_fullscreen) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    window_flags |=
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
  }

  // When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
  // and handle the pass-thru hole, so we ask Begin() to not render a background.
  if (dock_space_flags & ImGuiDockNodeFlags_PassthruCentralNode)
    window_flags |= ImGuiWindowFlags_NoBackground;

  // Important: note that we proceed even if Begin() returns false (aka window is collapsed).
  // This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
  // all active windows docked into it will lose their parent and become undocked.
  // We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
  // any change of dock space/settings would lead to windows being stuck in limbo and never being visible.

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  static bool open_dock = true;
  ImGui::Begin("Root DockSpace", &open_dock, window_flags);
  ImGui::PopStyleVar();
  if (opt_fullscreen)
    ImGui::PopStyleVar(2);
  const ImGuiID dock_space_id = ImGui::GetID("MyDockSpace");
  ImGui::DockSpace(dock_space_id, ImVec2(0.0f, 0.0f), dock_space_flags);
  ImGui::End();
#pragma endregion

  main_camera_focus_override = false;
  scene_camera_focus_override = false;
  if (ImGui::BeginMainMenuBar()) {
    switch (Application::GetApplicationStatus()) {
      case ApplicationStatus::Stop: {
        if (ImGui::ImageButton(assets_icons_["PlayButton"]->GetImTextureId(), {15, 15}, {0, 1}, {1, 0})) {
          Application::Play();
        }
        if (ImGui::ImageButton(assets_icons_["StepButton"]->GetImTextureId(), {15, 15}, {0, 1}, {1, 0})) {
          Application::Step();
        }
        break;
      }
      case ApplicationStatus::Playing: {
        if (ImGui::ImageButton(assets_icons_["PauseButton"]->GetImTextureId(), {15, 15}, {0, 1}, {1, 0})) {
          Application::Pause();
        }
        if (ImGui::ImageButton(assets_icons_["StopButton"]->GetImTextureId(), {15, 15}, {0, 1}, {1, 0})) {
          Application::Stop();
        }
        break;
      }
      case ApplicationStatus::Pause: {
        if (ImGui::ImageButton(assets_icons_["PlayButton"]->GetImTextureId(), {15, 15}, {0, 1}, {1, 0})) {
          Application::Play();
        }
        if (ImGui::ImageButton(assets_icons_["StepButton"]->GetImTextureId(), {15, 15}, {0, 1}, {1, 0})) {
          Application::Step();
        }
        if (ImGui::ImageButton(assets_icons_["StopButton"]->GetImTextureId(), {15, 15}, {0, 1}, {1, 0})) {
          Application::Stop();
        }
        break;
      }
      case ApplicationStatus::Uninitialized:
        break;
      case ApplicationStatus::NoProject:
        break;
      case ApplicationStatus::Step:
        break;
      case ApplicationStatus::OnDestroy:
        break;
    }

    ImGui::Separator();
    if (ImGui::BeginMenu("Project")) {
      ImGui::EndMenu();
    }
    /*
    if (ImGui::BeginMenu("File"))
    {
            ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit"))
    {
            ImGui::EndMenu();
    }
    */
    if (ImGui::BeginMenu("View")) {
      ImGui::EndMenu();
    }
    /*
    if (ImGui::BeginMenu("Help"))
    {
            ImGui::EndMenu();
    }
    */
    ImGui::EndMainMenuBar();
  }

  mouse_scene_window_position_ = glm::vec2(FLT_MAX, -FLT_MAX);
  if (show_scene_window) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
    if (ImGui::Begin("Scene")) {
      if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
        // Using a Child allow to fill all the space of the window.
        // It also allows customization
        if (scene_camera_window_focused_) {
          const auto mp = ImGui::GetMousePos();
          const auto wp = ImGui::GetWindowPos();
          mouse_scene_window_position_ = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        }
      }
      ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleVar();
  }

  mouse_camera_window_position_ = glm::vec2(FLT_MAX, -FLT_MAX);
  if (show_camera_window) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
    if (ImGui::Begin("Camera")) {
      if (ImGui::BeginChild("MainCameraRenderer", ImVec2(0, 0), false)) {
        // Using a Child allow to fill all the space of the window.
        // It also allows customization
        if (main_camera_window_focused_) {
          auto mp = ImGui::GetMousePos();
          auto wp = ImGui::GetWindowPos();
          mouse_camera_window_position_ = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        }
      }
      ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleVar();
  }
  if (show_scene_window)
    ResizeCameras();

  if (!main_camera_window_focused_) {
    const auto active_scene = Application::GetActiveScene();
    auto& pressed_keys = active_scene->pressed_keys_;
    pressed_keys.clear();
  }

  if (apply_transform_to_main_camera && !Application::IsPlaying()) {
    const auto scene = Application::GetActiveScene();
    if (const auto camera = scene->main_camera.Get<Camera>(); camera && scene->IsEntityValid(camera->GetOwner())) {
      auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
      GlobalTransform global_transform;
      global_transform.SetPosition(sceneCameraPosition);
      global_transform.SetRotation(sceneCameraRotation);
      scene->SetDataComponent(camera->GetOwner(), global_transform);
    }
  }
  const auto& scene = Application::GetActiveScene();
  if (!scene->IsEntityValid(selected_entity_)) {
    SetSelectedEntity(Entity());
  }
  if (const auto render_layer = Application::GetLayer<RenderLayer>();
      render_layer && render_layer->need_fade_ != 0 && selection_alpha_ < 256) {
    selection_alpha_ += static_cast<int>(static_cast<float>(Times::DeltaTime()) * 5120);
  }

  selection_alpha_ = glm::clamp(selection_alpha_, 0, 256);
}
const char* hierarchy_display_mode[]{"Archetype", "Hierarchy"};
void EditorLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (const auto& window_layer = Application::GetLayer<WindowLayer>(); !window_layer)
    return;

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      if (ImGui::BeginMenu("Editor")) {
        if (ImGui::BeginMenu("Scene")) {
          ImGui::Checkbox("Show Scene Window", &show_scene_window);
          if (show_scene_window) {
            ImGui::Checkbox("Show Scene Window Info", &show_scene_info);
            ImGui::Checkbox("View Gizmos", &enable_view_gizmos);
          }
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Camera")) {
          ImGui::Checkbox("Show Camera Window", &show_camera_window);
          if (show_camera_window) {
            ImGui::Checkbox("Show Camera Window Info", &show_camera_info);
          }
          ImGui::EndMenu();
        }
        ImGui::Checkbox("Show Entity Explorer", &show_entity_explorer_window);
        ImGui::Checkbox("Show Entity Inspector", &show_entity_inspector_window);
        ImGui::Checkbox("Show Console", &show_console_window);
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  const auto scene = GetScene();
  if (scene && show_entity_explorer_window) {
    ImGui::Begin("Entity Explorer");
    if (ImGui::BeginPopupContextWindow("NewEntityPopup")) {
      if (ImGui::Button("Create new entity")) {
        scene->CreateEntity(basic_entity_archetype_);
      }
      ImGui::EndPopup();
    }
    ImGui::Combo("Display mode", &selected_hierarchy_display_mode, hierarchy_display_mode,
                 IM_ARRAYSIZE(hierarchy_display_mode));
    std::string title = scene->GetTitle();
    if (ImGui::CollapsingHeader(title.c_str(), ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_OpenOnArrow)) {
      DraggableAsset(scene);
      RenameAsset(scene);
      if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
        ProjectManager::GetInstance().inspecting_asset = scene;
      }
      if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
          IM_ASSERT(payload->DataSize == sizeof(Handle));
          const auto payload_n = *static_cast<Handle*>(payload->Data);
          const auto new_entity = scene->GetEntity(payload_n);
          if (const auto parent = scene->GetParent(new_entity); parent.GetIndex() != 0)
            scene->RemoveChild(new_entity, parent);
        }
        ImGui::EndDragDropTarget();
      }
      if (selected_hierarchy_display_mode == 0) {
        scene->UnsafeForEachEntityStorage([&](int i, const std::string& name, const DataComponentStorage& storage) {
          if (i == 0)
            return;
          ImGui::Separator();
          const std::string title1 = std::to_string(i) + ". " + name;
          if (ImGui::TreeNode(title1.c_str())) {
            ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.2, 0.3, 0.2, 1.0));
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.2, 0.2, 0.2, 1.0));
            ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.2, 0.2, 0.3, 1.0));
            for (int j = 0; j < storage.entity_alive_count; j++) {
              Entity entity = storage.chunk_array.entity_array.at(j);
              std::string title2 = std::to_string(entity.GetIndex()) + ": ";
              title2 += scene->GetEntityName(entity);
              const bool enabled = scene->IsEntityEnabled(entity);
              if (enabled) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4({1, 1, 1, 1}));
              }
              ImGui::TreeNodeEx(
                  title2.c_str(),
                  ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                      (selected_entity_ == entity ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
              if (enabled) {
                ImGui::PopStyleColor();
              }
              DrawEntityMenu(enabled, entity);
              if (!lock_entity_selection_ && ImGui::IsItemHovered() && ImGui::IsMouseClicked(0)) {
                SetSelectedEntity(entity, false);
              }
            }
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
            ImGui::TreePop();
          }
        });
      } else if (selected_hierarchy_display_mode == 1) {
        ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.2, 0.3, 0.2, 1.0));
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.2, 0.2, 0.2, 1.0));
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.2, 0.2, 0.3, 1.0));
        scene->ForAllEntities([&](int, const Entity entity) {
          if (scene->GetParent(entity).GetIndex() == 0)
            DrawEntityNode(entity, 0);
        });
        selected_entity_hierarchy_list_.clear();
        ImGui::PopStyleColor();
        ImGui::PopStyleColor();
        ImGui::PopStyleColor();
      }
    }
    ImGui::End();
  }
  if (scene && show_entity_inspector_window) {
    ImGui::Begin("Entity Inspector");
    ImGui::Text("Selection:");
    ImGui::SameLine();
    ImGui::Checkbox("Lock", &lock_entity_selection_);
    ImGui::SameLine();
    ImGui::Checkbox("Focus", &highlight_selection_);
    ImGui::SameLine();
    ImGui::Checkbox("Gizmos", &enable_gizmos);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      SetSelectedEntity({});
    }
    ImGui::Separator();
    if (scene->IsEntityValid(selected_entity_)) {
      std::string title = std::to_string(selected_entity_.GetIndex()) + ": ";
      title += scene->GetEntityName(selected_entity_);
      bool enabled = scene->IsEntityEnabled(selected_entity_);
      if (ImGui::Checkbox((title + "##EnabledCheckbox").c_str(), &enabled)) {
        if (scene->IsEntityEnabled(selected_entity_) != enabled) {
          scene->SetEnable(selected_entity_, enabled);
        }
      }
      ImGui::SameLine();
      bool is_static = scene->IsEntityStatic(selected_entity_);
      if (ImGui::Checkbox("Static##StaticCheckbox", &is_static)) {
        if (scene->IsEntityStatic(selected_entity_) != is_static) {
          scene->SetEntityStatic(selected_entity_, enabled);
        }
      }

      if (const bool deleted = DrawEntityMenu(scene->IsEntityEnabled(selected_entity_), selected_entity_); !deleted) {
        if (ImGui::CollapsingHeader("Data components", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::BeginPopupContextItem("DataComponentInspectorPopup")) {
            ImGui::Text("Add data component: ");
            ImGui::Separator();

            for (const auto& i : Serialization::GetInstance().data_component_ids_) {
              const auto id = i.second;
              const auto name = i.first;
              if (id == typeid(Transform).hash_code() || id == typeid(GlobalTransform).hash_code() ||
                  id == typeid(TransformUpdateFlag).hash_code())
                continue;

              if (!scene->HasDataComponent(selected_entity_, id) && ImGui::Button(name.c_str())) {
                scene->AddDataComponent(selected_entity_, id);
              }
            }
            ImGui::Separator();
            ImGui::EndPopup();
          }
          bool skip = false;
          int i = 0;
          scene->UnsafeForEachDataComponent(selected_entity_, [&](const DataComponentType& type, void* data) {
            if (skip)
              return;
            std::string info = type.type_name;
            info += " Size: " + std::to_string(type.type_size);
            ImGui::Text(info.c_str());
            ImGui::PushID(i);
            if (ImGui::BeginPopupContextItem(("DataComponentDeletePopup" + std::to_string(i)).c_str())) {
              if (ImGui::Button("Remove")) {
                skip = true;
                scene->RemoveDataComponent(selected_entity_, type.type_index);
              }
              ImGui::EndPopup();
            }
            ImGui::PopID();
            InspectComponentData(selected_entity_, static_cast<IDataComponent*>(data), type,
                                 scene->GetParent(selected_entity_).GetIndex() != 0);
            ImGui::Separator();
            i++;
          });
        }

        if (ImGui::CollapsingHeader("Private components", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::BeginPopupContextItem("PrivateComponentInspectorPopup")) {
            ImGui::Text("Add private component: ");
            ImGui::Separator();
            for (const auto& i : Serialization::GetInstance().private_component_ids_) {
              const auto id = i.second;
              const auto name = i.first;
              if (!scene->HasPrivateComponent(selected_entity_, id) && ImGui::Button(name.c_str())) {
                scene->AddPrivateComponent(selected_entity_, id);
              }
            }
            ImGui::Separator();
            ImGui::EndPopup();
          }

          int i = 0;
          bool skip = false;
          scene->ForEachPrivateComponent(selected_entity_, [&](const PrivateComponentElement& data) {
            if (skip)
              return;
            ImGui::Checkbox(data.private_component_data->GetTypeName().c_str(), &data.private_component_data->enabled_);
            DraggablePrivateComponent(data.private_component_data);
            const std::string tag = "##" + data.private_component_data->GetTypeName() +
                                    std::to_string(data.private_component_data->GetHandle());
            if (ImGui::BeginPopupContextItem(tag.c_str())) {
              if (ImGui::Button(("Remove" + tag).c_str())) {
                skip = true;
                scene->RemovePrivateComponent(selected_entity_, data.type_index);
              }
              ImGui::EndPopup();
            }
            if (!skip) {
              if (ImGui::TreeNodeEx(("Component Settings##" + std::to_string(i)).c_str(),
                                    ImGuiTreeNodeFlags_DefaultOpen)) {
                if (data.private_component_data->OnInspect(editor_layer))
                  scene->SetUnsaved();
                ImGui::TreePop();
              }
            }
            ImGui::Separator();
            i++;
          });
        }
      }
    } else {
      SetSelectedEntity(Entity());
    }
    ImGui::End();
  }
  if (show_console_window) {
    if (ImGui::Begin("Console")) {
      ImGui::Checkbox("Log", &enable_console_logs_);
      ImGui::SameLine();
      ImGui::Checkbox("Warning", &enable_console_warnings_);
      ImGui::SameLine();
      ImGui::Checkbox("Error", &enable_console_errors_);
      ImGui::SameLine();
      if (ImGui::Button("Clear all")) {
        console_messages_.clear();
      }
      int i = 0;
      for (auto msg = console_messages_.rbegin(); msg != console_messages_.rend(); ++msg) {
        if (i > 999)
          break;
        i++;
        switch (msg->m_type) {
          case ConsoleMessageType::Log:
            if (enable_console_logs_) {
              ImGui::TextColored(ImVec4(0, 0, 1, 1), "%.2f: ", msg->m_time);
              ImGui::SameLine();
              ImGui::TextColored(ImVec4(1, 1, 1, 1), msg->m_value.c_str());
              ImGui::Separator();
            }
            break;
          case ConsoleMessageType::Warning:
            if (enable_console_warnings_) {
              ImGui::TextColored(ImVec4(0, 0, 1, 1), "%.2f: ", msg->m_time);
              ImGui::SameLine();
              ImGui::TextColored(ImVec4(1, 1, 0, 1), msg->m_value.c_str());
              ImGui::Separator();
            }
            break;
          case ConsoleMessageType::Error:
            if (enable_console_errors_) {
              ImGui::TextColored(ImVec4(0, 0, 1, 1), "%.2f: ", msg->m_time);
              ImGui::SameLine();
              ImGui::TextColored(ImVec4(1, 0, 0, 1), msg->m_value.c_str());
              ImGui::Separator();
            }
            break;
        }
      }
    }
    ImGui::End();
  }

  if (scene && scene_camera_window_focused_ && Input::GetKey(GLFW_KEY_DELETE) == Input::KeyActionType::Press) {
    if (scene->IsEntityValid(selected_entity_)) {
      scene->DeleteEntity(selected_entity_);
    }
  }

  if (show_scene_window)
    SceneCameraWindow();
  if (show_camera_window)
    MainCameraWindow();

  ProjectManager::OnInspect(editor_layer);
  Resources::OnInspect(editor_layer);
}

void EditorLayer::LateUpdate() {
  if (lock_camera) {
    auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
    const float elapsed_time = static_cast<float>(Times::Now()) - transition_timer_;
    float a = 1.0f - glm::pow(1.0 - elapsed_time / transition_time_, 4.0f);
    if (elapsed_time >= transition_time_)
      a = 1.0f;
    sceneCameraRotation = glm::mix(previous_rotation_, target_rotation_, a);
    sceneCameraPosition = glm::mix(previous_position_, target_position_, a);
    if (a >= 1.0f) {
      lock_camera = false;
      sceneCameraRotation = target_rotation_;
      sceneCameraPosition = target_position_;
      // Camera::ReverseAngle(target_rotation_, m_sceneCameraPitchAngle, m_sceneCameraYawAngle);
    }
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    Platform::EverythingBarrier(vk_command_buffer);
    Platform::TransitImageLayout(vk_command_buffer, Platform::GetSwapchain()->GetVkImage(),
                                 Platform::GetSwapchain()->GetImageFormat(), 1, VK_IMAGE_LAYOUT_UNDEFINED,
                                 VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR);

    constexpr VkClearValue clear_color = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent = Platform::GetSwapchain()->GetImageExtent();

    VkRenderingAttachmentInfo color_attachment_info{};
    color_attachment_info.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    color_attachment_info.imageView = Platform::GetSwapchain()->GetVkImageView();
    color_attachment_info.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR;
    color_attachment_info.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_attachment_info.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_attachment_info.clearValue = clear_color;

    VkRenderingInfo render_info{};
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea = render_area;
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = 1;
    render_info.pColorAttachments = &color_attachment_info;

    vkCmdBeginRendering(vk_command_buffer, &render_info);
    ImGui::Render();
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), vk_command_buffer);

    // Update and Render additional Platform Windows
    // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this
    // code elsewhere.
    //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow* backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    } 

    vkCmdEndRendering(vk_command_buffer);
    Platform::TransitImageLayout(vk_command_buffer, Platform::GetSwapchain()->GetVkImage(),
                                 Platform::GetSwapchain()->GetImageFormat(), 1, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL_KHR,
                                 VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
  });
}

bool EditorLayer::DrawEntityMenu(const bool& enabled, const Entity& entity) const {
  bool deleted = false;
  if (ImGui::BeginPopupContextItem(std::to_string(entity.GetIndex()).c_str())) {
    const auto scene = GetScene();
    ImGui::Text(("Handle: " + std::to_string(scene->GetEntityHandle(entity).GetValue())).c_str());
    if (ImGui::Button("Delete")) {
      scene->DeleteEntity(entity);
      deleted = true;
    }
    if (!deleted && ImGui::Button(enabled ? "Disable" : "Enable")) {
      if (enabled) {
        scene->SetEnable(entity, false);
      } else {
        scene->SetEnable(entity, true);
      }
    }
    if (const std::string tag = "##Entity" + std::to_string(scene->GetEntityHandle(entity));
        !deleted && ImGui::BeginMenu(("Rename" + tag).c_str())) {
      static char new_name[256];
      ImGui::InputText("New name", new_name, 256);
      if (ImGui::Button("Confirm")) {
        scene->SetEntityName(entity, std::string(new_name));
        memset(new_name, 0, 256);
      }
      ImGui::EndMenu();
    }
    ImGui::EndPopup();
  }
  return deleted;
}

void EditorLayer::DrawEntityNode(const Entity& entity, const unsigned& hierarchy_level) {
  auto scene = GetScene();
  std::string title = std::to_string(entity.GetIndex()) + ": ";
  title += scene->GetEntityName(entity);
  const bool enabled = scene->IsEntityEnabled(entity);
  if (enabled) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4({1, 1, 1, 1}));
  }
  if (const int index = selected_entity_hierarchy_list_.size() - hierarchy_level - 1;
      !selected_entity_hierarchy_list_.empty() && index >= 0 && index < selected_entity_hierarchy_list_.size() &&
      selected_entity_hierarchy_list_[index] == entity) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      title.c_str(), ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
                         ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                         (selected_entity_ == entity ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
  if (ImGui::BeginDragDropSource()) {
    auto handle = scene->GetEntityHandle(entity);
    ImGui::SetDragDropPayload("Entity", &handle, sizeof(Handle));
    ImGui::TextColored(ImVec4(0, 0, 1, 1), title.c_str());
    ImGui::EndDragDropSource();
  }
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      scene->SetParent(scene->GetEntity(*static_cast<Handle*>(payload->Data)), entity, true);
    }
    ImGui::EndDragDropTarget();
  }
  if (enabled) {
    ImGui::PopStyleColor();
  }
  if (!lock_entity_selection_ && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    SetSelectedEntity(entity, false);
  }
  if (const bool deleted = DrawEntityMenu(enabled, entity); opened && !deleted) {
    ImGui::TreePush(title.c_str());
    scene->ForEachChild(entity, [=](const Entity child) {
      DrawEntityNode(child, hierarchy_level + 1);
    });
    ImGui::TreePop();
  }
}

void EditorLayer::InspectComponentData(const Entity entity, IDataComponent* data, const DataComponentType& type,
                                       const bool is_root) {
  if (component_data_inspector_map_.find(type.type_index) != component_data_inspector_map_.end()) {
    if (component_data_inspector_map_.at(type.type_index)(entity, data, is_root)) {
      const auto scene = GetScene();
      scene->SetUnsaved();
    }
  }
}

void EditorLayer::SceneCameraWindow() {
  const auto scene = GetScene();
  auto window_layer = Application::GetLayer<WindowLayer>();
  const auto& graphics = Platform::GetInstance();
  auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
#pragma region Scene Window
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Scene")) {
    ImVec2 view_port_size;
    // Using a Child allow to fill all the space of the window.
    // It also allows customization
    static int corner = 1;
    if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
      view_port_size = ImGui::GetWindowSize();
      scene_camera_resolution_x_ = view_port_size.x * scene_camera_resolution_multiplier;
      scene_camera_resolution_y_ = view_port_size.y * scene_camera_resolution_multiplier;
      const ImVec2 overlay_pos = ImGui::GetWindowPos();
      if (sceneCamera && sceneCamera->rendered_) {
        // Because I use the texture from OpenGL, I need to invert the V from the UV.
        ImGui::Image(sceneCamera->GetRenderTexture()->GetColorImTextureId(), ImVec2(view_port_size.x, view_port_size.y),
                     ImVec2(0, 1), ImVec2(1, 0));
        CameraWindowDragAndDrop();
      } else {
        ImGui::Text("No active scene camera!");
      }
      const auto window_pos = ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : (overlay_pos.x),
                                     (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y));

      if (show_scene_info) {
        const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        ImGui::SetNextWindowBgAlpha(0.35f);
        constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                  ImGuiWindowFlags_NoSavedSettings |
                                                  ImGuiWindowFlags_NoFocusOnAppearing;
        if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_None;
            ImGui::BeginChild("Info", ImVec2(200, 350), child_flags, window_flags)) {
          ImGui::Text("Info & Settings");
          ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
          std::string draw_call_info = {};
          const auto current_frame_index = Platform::GetCurrentFrameIndex();
          if (graphics.triangles[current_frame_index] < 999)
            draw_call_info += std::to_string(graphics.triangles[current_frame_index]);
          else if (graphics.triangles[current_frame_index] < 999999)
            draw_call_info += std::to_string(static_cast<int>(graphics.triangles[current_frame_index] / 1000)) + "K";
          else
            draw_call_info += std::to_string(static_cast<int>(graphics.triangles[current_frame_index] / 1000000)) + "M";
          draw_call_info += " tris";
          ImGui::Text(draw_call_info.c_str());
          ImGui::Text("%d drawcall", graphics.draw_call[current_frame_index]);
          ImGui::Text("Idle: %.3f", graphics.cpu_wait_time);
          ImGui::Separator();
          if (ImGui::IsMousePosValid()) {
            const auto pos = Input::GetMousePosition();
            ImGui::Text("Mouse Pos: (%.1f,%.1f)", pos.x, pos.y);
          } else {
            ImGui::Text("Mouse Pos: <invalid>");
          }

          if (ImGui::Button("Reset camera")) {
            MoveCamera(default_scene_camera_rotation, default_scene_camera_position);
          }
          if (ImGui::Button("Set default")) {
            default_scene_camera_position = sceneCameraPosition;
            default_scene_camera_rotation = sceneCameraRotation;
          }
          ImGui::PushItemWidth(100);
          ImGui::DragFloat("Background Intensity", &sceneCamera->background_intensity, 0.01f, 0.0f, 10.f);
          ImGui::Checkbox("Use clear color", &sceneCamera->use_clear_color);
          if (sceneCamera->use_clear_color) {
            ImGui::ColorEdit3("Clear Color", (float*)(void*)&sceneCamera->clear_color);
          } else {
            DragAndDropButton<Cubemap>(sceneCamera->skybox, "Skybox");
          }
          ImGui::SliderFloat("FOV", &sceneCamera->fov, 1.0f, 359.f, "%.1f");
          ImGui::DragFloat3("Position", &sceneCameraPosition.x, 0.1f, 0, 0, "%.1f");
          ImGui::DragFloat("Speed", &velocity, 0.1f, 0, 0, "%.1f");
          ImGui::DragFloat("Sensitivity", &sensitivity, 0.1f, 0, 0, "%.1f");
          ImGui::Checkbox("Copy Transform", &apply_transform_to_main_camera);
          ImGui::DragFloat("Resolution", &scene_camera_resolution_multiplier, 0.1f, 0.1f, 4.0f);
          ImGui::PopItemWidth();
        }
        ImGui::EndChild();
      }
      if (scene_camera_window_focused_) {
#pragma region Scene Camera Controller
        static bool is_dragging_previously = false;
        bool mouse_drag = true;
        if (mouse_scene_window_position_.x < 0 || mouse_scene_window_position_.y < 0 ||
            mouse_scene_window_position_.x > view_port_size.x || mouse_scene_window_position_.y > view_port_size.y ||
            Input::GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold) {
          mouse_drag = false;
        }
        static float prev_x = 0;
        static float prev_y = 0;
        if (mouse_drag && !is_dragging_previously) {
          prev_x = mouse_scene_window_position_.x;
          prev_y = mouse_scene_window_position_.y;
        }
        const float x_offset = mouse_scene_window_position_.x - prev_x;
        const float y_offset = mouse_scene_window_position_.y - prev_y;
        prev_x = mouse_scene_window_position_.x;
        prev_y = mouse_scene_window_position_.y;
        is_dragging_previously = mouse_drag;

        if (mouse_drag && !lock_camera) {
          glm::vec3 front = sceneCameraRotation * glm::vec3(0, 0, -1);
          const glm::vec3 right = sceneCameraRotation * glm::vec3(1, 0, 0);
          if (Input::GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
            sceneCameraPosition += front * static_cast<float>(Times::DeltaTime()) * velocity;
          }
          if (Input::GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
            sceneCameraPosition -= front * static_cast<float>(Times::DeltaTime()) * velocity;
          }
          if (Input::GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
            sceneCameraPosition -= right * static_cast<float>(Times::DeltaTime()) * velocity;
          }
          if (Input::GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
            sceneCameraPosition += right * static_cast<float>(Times::DeltaTime()) * velocity;
          }
          if (Input::GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
            sceneCameraPosition.y += velocity * static_cast<float>(Times::DeltaTime());
          }
          if (Input::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
            sceneCameraPosition.y -= velocity * static_cast<float>(Times::DeltaTime());
          }
          if (x_offset != 0.0f || y_offset != 0.0f) {
            front = glm::rotate(front, glm::radians(-x_offset * sensitivity), glm::vec3(0, 1, 0));
            const glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
            if ((front.y < 0.99f && y_offset < 0.0f) || (front.y > -0.99f && y_offset > 0.0f)) {
              front = glm::rotate(front, glm::radians(-y_offset * sensitivity), right);
            }
            const glm::vec3 up = glm::normalize(glm::cross(right, front));
            sceneCameraRotation = glm::quatLookAt(front, up);
          }
#pragma endregion
        }
      }
    }
#pragma region Gizmos and Entity Selection
    using_gizmo_ = false;
    if (enable_gizmos) {
      ImGuizmo::SetOrthographic(false);
      ImGuizmo::SetDrawlist();
      float view_manipulate_left = ImGui::GetWindowPos().x;
      float view_manipulate_top = ImGui::GetWindowPos().y;
      ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, view_port_size.x, view_port_size.y);
      glm::mat4 camera_view = glm::inverse(glm::translate(sceneCameraPosition) * glm::mat4_cast(sceneCameraRotation));
      glm::mat4 camera_projection = sceneCamera->GetProjection();
      const auto op = local_position_selected_   ? ImGuizmo::OPERATION::TRANSLATE
                      : local_rotation_selected_ ? ImGuizmo::OPERATION::ROTATE
                                                 : ImGuizmo::OPERATION::SCALE;
      if (scene->IsEntityValid(selected_entity_)) {
        auto transform = scene->GetDataComponent<Transform>(selected_entity_);
        GlobalTransform parent_global_transform;
        if (Entity parent_entity = scene->GetParent(selected_entity_); parent_entity.GetIndex() != 0) {
          parent_global_transform = scene->GetDataComponent<GlobalTransform>(scene->GetParent(selected_entity_));
        }
        auto global_transform = scene->GetDataComponent<GlobalTransform>(selected_entity_);

        ImGuizmo::Manipulate(glm::value_ptr(camera_view), glm::value_ptr(camera_projection), op, ImGuizmo::LOCAL,
                             glm::value_ptr(global_transform.value));
        if (ImGuizmo::IsUsing()) {
          transform.value = glm::inverse(parent_global_transform.value) * global_transform.value;
          scene->SetDataComponent(selected_entity_, transform);
          transform.Decompose(previously_stored_position_, previously_stored_rotation_, previously_stored_scale_);
          using_gizmo_ = true;
        }
      }
      if (false) {
        ImGuizmo::ViewManipulate(glm::value_ptr(camera_view), 1.0f, ImVec2(view_manipulate_left, view_manipulate_top),
                                 ImVec2(96, 96), 0);
        GlobalTransform gl;
        gl.value = glm::inverse(camera_view);
        sceneCameraRotation = gl.GetRotation();
      }
    }

#pragma endregion
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
      scene_camera_window_focused_ = true;
    } else {
      scene_camera_window_focused_ = false;
    }
    ImGui::EndChild();
  } else {
    scene_camera_window_focused_ = false;
  }
  sceneCamera->SetRequireRendering(
      !(ImGui::GetCurrentWindowRead()->Hidden && !ImGui::GetCurrentWindowRead()->Collapsed));

  ImGui::End();

  ImGui::PopStyleVar();

#pragma endregion
}

void EditorLayer::MainCameraWindow() {
  if (const auto render_layer = Application::GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto& graphics = Platform::GetInstance();
  const auto scene = GetScene();
#pragma region Window
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Camera")) {
    static int corner = 1;
    // Using a Child allow to fill all the space of the window.
    // It also allows customization
    if (ImGui::BeginChild("MainCameraRenderer", ImVec2(0, 0), false)) {
      const ImVec2 view_port_size = ImGui::GetWindowSize();
      main_camera_resolution_x = view_port_size.x * main_camera_resolution_multiplier_;
      main_camera_resolution_y = view_port_size.y * main_camera_resolution_multiplier_;
      //  Get the size of the child (i.e. the whole draw size of the windows).
      const ImVec2 overlay_pos = ImGui::GetWindowPos();
      // Because I use the texture from OpenGL, I need to invert the V from the UV.
      const auto main_camera = scene->main_camera.Get<Camera>();
      if (main_camera && main_camera->rendered_) {
        ImGui::Image(main_camera->GetRenderTexture()->GetColorImTextureId(), ImVec2(view_port_size.x, view_port_size.y),
                     ImVec2(0, 1), ImVec2(1, 0));
        CameraWindowDragAndDrop();
      } else {
        ImGui::Text("No active main camera!");
      }

      const auto window_pos = ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : (overlay_pos.x),
                                     (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y));
      if (show_camera_info) {
        const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        ImGui::SetNextWindowBgAlpha(0.35f);
        constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                  ImGuiWindowFlags_NoSavedSettings |
                                                  ImGuiWindowFlags_NoFocusOnAppearing;
        if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_None;
            ImGui::BeginChild("Render Info", ImVec2(300, 150), child_flags, window_flags)) {
          ImGui::Text("Info & Settings");
          ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
          ImGui::PushItemWidth(100);
          ImGui::Checkbox("Auto resize", &main_camera_allow_auto_resize);
          if (main_camera_allow_auto_resize) {
            ImGui::DragFloat("Resolution multiplier", &main_camera_resolution_multiplier_, 0.1f, 0.1f, 4.0f);
          }
          ImGui::PopItemWidth();
          std::string draw_call_info = {};
          const auto current_frame_index = Platform::GetCurrentFrameIndex();
          if (graphics.triangles[current_frame_index] < 999)
            draw_call_info += std::to_string(graphics.triangles[current_frame_index]);
          else if (graphics.triangles[current_frame_index] < 999999)
            draw_call_info += std::to_string(static_cast<int>(graphics.triangles[current_frame_index] / 1000)) + "K";
          else
            draw_call_info += std::to_string(static_cast<int>(graphics.triangles[current_frame_index] / 1000000)) + "M";
          draw_call_info += " tris";
          ImGui::Text(draw_call_info.c_str());
          ImGui::Text("%d drawcall", graphics.draw_call[current_frame_index]);
          ImGui::Separator();
          if (ImGui::IsMousePosValid()) {
            const auto pos = Input::GetMousePosition();
            ImGui::Text("Mouse Pos: (%.1f,%.1f)", pos.x, pos.y);
          } else {
            ImGui::Text("Mouse Pos: <invalid>");
          }
        }
        ImGui::EndChild();
      }

      if (main_camera_window_focused_ && !lock_entity_selection_ &&
          Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
        SetSelectedEntity(Entity());
      }
      if (!Application::IsPlaying() && main_camera_window_focused_ && !lock_entity_selection_ &&
          Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press &&
          !(mouse_camera_window_position_.x < 0 || mouse_camera_window_position_.y < 0 ||
            mouse_camera_window_position_.x > view_port_size.x || mouse_camera_window_position_.y > view_port_size.y)) {
        if (const auto focused_entity = MouseEntitySelection(main_camera, mouse_camera_window_position_);
            focused_entity == Entity()) {
          SetSelectedEntity(Entity());
        } else {
          Entity walker = focused_entity;
          bool found = false;
          while (walker.GetIndex() != 0) {
            if (walker == selected_entity_) {
              found = true;
              break;
            }
            walker = scene->GetParent(walker);
          }
          if (found) {
            walker = scene->GetParent(walker);
            if (walker.GetIndex() == 0) {
              SetSelectedEntity(focused_entity);
            } else {
              SetSelectedEntity(walker);
            }
          } else {
            SetSelectedEntity(focused_entity);
          }
        }
      }
    }

    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
      main_camera_window_focused_ = true;
    } else {
      main_camera_window_focused_ = false;
    }

    ImGui::EndChild();
  } else {
    main_camera_window_focused_ = false;
  }
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->SetRequireRendering(!ImGui::GetCurrentWindowRead()->Hidden &&
                                     !ImGui::GetCurrentWindowRead()->Collapsed);
  }

  ImGui::End();
  ImGui::PopStyleVar();
#pragma endregion
}

void EditorLayer::OnInputEvent(const Input::InputEvent& input_event) {
  // If main camera is focused, we pass the event to the scene.
  if (main_camera_window_focused_ && Application::IsPlaying()) {
    const auto active_scene = Application::GetActiveScene();
    auto& pressed_keys = active_scene->pressed_keys_;
    if (input_event.key_action == Input::KeyActionType::Press) {
      if (const auto search = pressed_keys.find(input_event.key); search != active_scene->pressed_keys_.end()) {
        // Dispatch hold if the key is already pressed.
        search->second = Input::KeyActionType::Hold;
      } else {
        // Dispatch press if the key is previously released.
        pressed_keys.insert({input_event.key, Input::KeyActionType::Press});
      }
    } else if (input_event.key_action == Input::KeyActionType::Release) {
      if (pressed_keys.find(input_event.key) != pressed_keys.end()) {
        // Dispatch hold if the key is already pressed.
        pressed_keys.erase(input_event.key);
      }
    }
  }
}

void EditorLayer::ResizeCameras() {
  if (const auto render_layer = Application::GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto& scene_camera = GetSceneCamera();
  if (const auto resolution = scene_camera->GetSize();
      scene_camera_resolution_x_ != 0 && scene_camera_resolution_y_ != 0 &&
      (resolution.x != scene_camera_resolution_x_ || resolution.y != scene_camera_resolution_y_)) {
    scene_camera->Resize({scene_camera_resolution_x_, scene_camera_resolution_y_});
  }
  const auto scene = Application::GetActiveScene();
  if (const std::shared_ptr<Camera> main_camera = scene->main_camera.Get<Camera>()) {
    if (main_camera_allow_auto_resize)
      main_camera->Resize({main_camera_resolution_x, main_camera_resolution_y});
  }
}

std::vector<ConsoleMessage>& EditorLayer::GetConsoleMessages() {
  return console_messages_;
}

bool EditorLayer::SceneCameraWindowFocused() const {
  return scene_camera_window_focused_;
}

bool EditorLayer::MainCameraWindowFocused() const {
  return main_camera_window_focused_;
}

void EditorLayer::RegisterEditorCamera(const std::shared_ptr<Camera>& camera) {
  if (editor_cameras_.find(camera->GetHandle()) == editor_cameras_.end()) {
    editor_cameras_[camera->GetHandle()] = {};
    editor_cameras_.at(camera->GetHandle()).camera = camera;
  }
}

glm::vec2 EditorLayer::GetMouseSceneCameraPosition() const {
  return mouse_scene_window_position_;
}

Input::KeyActionType EditorLayer::GetKey(const int key) {
  return Input::GetKey(key);
}

std::shared_ptr<Camera> EditorLayer::GetSceneCamera() {
  return editor_cameras_.at(scene_camera_handle_).camera;
}

glm::vec3 EditorLayer::GetSceneCameraPosition() const {
  return editor_cameras_.at(scene_camera_handle_).position;
}

glm::quat EditorLayer::GetSceneCameraRotation() const {
  return editor_cameras_.at(scene_camera_handle_).rotation;
}

void EditorLayer::SetCameraPosition(const std::shared_ptr<Camera>& camera, const glm::vec3& target_position) {
  editor_cameras_.at(camera->GetHandle()).position = target_position;
}

void EditorLayer::SetCameraRotation(const std::shared_ptr<Camera>& camera, const glm::quat& target_rotation) {
  editor_cameras_.at(camera->GetHandle()).rotation = target_rotation;
}

void EditorLayer::UpdateTextureId(ImTextureID& target, const VkSampler image_sampler, const VkImageView image_view,
                                  const VkImageLayout image_layout) {
  if (!Application::GetLayer<EditorLayer>())
    return;
  if (target != VK_NULL_HANDLE)
    ImGui_ImplVulkan_RemoveTexture(static_cast<VkDescriptorSet>(target));
  target = ImGui_ImplVulkan_AddTexture(image_sampler, image_view, image_layout);
}

Entity EditorLayer::GetSelectedEntity() const {
  return selected_entity_;
}

void EditorLayer::SetSelectedEntity(const Entity& entity, const bool open_menu) {
  if (entity == selected_entity_)
    return;
  selected_entity_hierarchy_list_.clear();
  const auto scene = GetScene();
  const auto previous_descendants = scene->GetDescendants(selected_entity_);
  for (const auto& i : previous_descendants) {
    scene->GetEntityMetadata(i).ancestor_selected = false;
  }
  if (scene->IsEntityValid(selected_entity_))
    scene->GetEntityMetadata(selected_entity_).ancestor_selected = false;
  if (entity.GetIndex() == 0) {
    selected_entity_ = Entity();
    lock_entity_selection_ = false;
    selection_alpha_ = 0;
    return;
  }

  if (!scene->IsEntityValid(entity))
    return;
  selected_entity_ = entity;
  const auto descendants = scene->GetDescendants(selected_entity_);

  for (const auto& i : descendants) {
    scene->GetEntityMetadata(i).ancestor_selected = true;
  }
  scene->GetEntityMetadata(selected_entity_).ancestor_selected = true;
  if (!open_menu)
    return;
  auto walker = entity;
  while (walker.GetIndex() != 0) {
    selected_entity_hierarchy_list_.push_back(walker);
    walker = scene->GetParent(walker);
  }
}

bool EditorLayer::GetLockEntitySelection() const {
  return lock_entity_selection_;
}

void EditorLayer::SetLockEntitySelection(const bool value) {
  const auto scene = GetScene();
  if (!value)
    lock_entity_selection_ = false;
  else if (scene->IsEntityValid(selected_entity_)) {
    lock_entity_selection_ = true;
  }
}

bool EditorLayer::UnsafeDroppableAsset(AssetRef& target, const std::vector<std::string>& type_names) {
  bool status_changed = false;
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      const std::shared_ptr<IAsset> ptr = target.Get<IAsset>();
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (!ptr || payload_n.GetValue() != target.GetAssetHandle().GetValue()) {
        auto asset = ProjectManager::GetAsset(payload_n);
        for (const auto& type_name : type_names) {
          if (asset && asset->GetTypeName() == type_name) {
            target.Clear();
            target.asset_handle_ = payload_n;
            target.Update();
            status_changed = true;
            break;
          }
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
  return status_changed;
}

bool EditorLayer::UnsafeDroppablePrivateComponent(PrivateComponentRef& target,
                                                  const std::vector<std::string>& type_names) {
  bool status_changed = false;
  if (ImGui::BeginDragDropTarget()) {
    const auto current_scene = Application::GetActiveScene();
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      auto payload_n = *static_cast<Handle*>(payload->Data);
      auto entity = current_scene->GetEntity(payload_n);
      if (current_scene->IsEntityValid(entity)) {
        for (const auto& type_name : type_names) {
          if (current_scene->HasPrivateComponent(entity, type_name)) {
            const auto ptr = target.Get<IPrivateComponent>();
            const auto new_private_component = current_scene->GetPrivateComponent(entity, type_name).lock();
            target = new_private_component;
            status_changed = true;
            break;
          }
        }
      }
    } else if (const ImGuiPayload* payload2 = ImGui::AcceptDragDropPayload("PrivateComponent")) {
      IM_ASSERT(payload2->DataSize == sizeof(Handle));
      const auto payload_n = *static_cast<Handle*>(payload2->Data);
      const auto entity = current_scene->GetEntity(payload_n);
      for (const auto& type_name : type_names) {
        if (current_scene->HasPrivateComponent(entity, type_name)) {
          target = current_scene->GetPrivateComponent(entity, type_name).lock();
          status_changed = true;
          break;
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
  return status_changed;
}

std::map<std::string, std::shared_ptr<Texture2D>>& EditorLayer::AssetIcons() {
  return assets_icons_;
}

bool EditorLayer::DragAndDropButton(EntityRef& entity_ref, const std::string& name, bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  bool status_changed = false;
  if (const auto entity = entity_ref.Get(); entity.GetIndex() != 0) {
    const auto scene = Application::GetActiveScene();
    ImGui::Button(scene->GetEntityName(entity).c_str());
    Draggable(entity_ref);
    if (modifiable) {
      status_changed = Rename(entity_ref);
      status_changed = Remove(entity_ref) || status_changed;
    }
  } else {
    ImGui::Button("none");
  }
  status_changed = Droppable(entity_ref) || status_changed;
  return status_changed;
}
bool EditorLayer::Droppable(EntityRef& entity_ref) {
  bool status_changed = false;
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      const auto scene = Application::GetActiveScene();
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      auto payload_n = *static_cast<Handle*>(payload->Data);
      if (const auto new_entity = scene->GetEntity(payload_n); scene->IsEntityValid(new_entity)) {
        entity_ref = new_entity;
        status_changed = true;
      }
    }
    ImGui::EndDragDropTarget();
  }
  return status_changed;
}
void EditorLayer::Draggable(EntityRef& entity_ref) {
  auto entity = entity_ref.Get();
  if (entity.GetIndex() != 0) {
    DraggableEntity(entity);
  }
}
void EditorLayer::DraggableEntity(const Entity& entity) {
  if (ImGui::BeginDragDropSource()) {
    auto scene = Application::GetActiveScene();
    auto handle = scene->GetEntityHandle(entity);
    ImGui::SetDragDropPayload("Entity", &handle, sizeof(Handle));
    ImGui::TextColored(ImVec4(0, 0, 1, 1), scene->GetEntityName(entity).c_str());
    ImGui::EndDragDropSource();
  }
}
bool EditorLayer::Rename(EntityRef& entity_ref) {
  const auto entity = entity_ref.Get();
  const bool status_changed = RenameEntity(entity);
  return status_changed;
}
bool EditorLayer::Remove(EntityRef& entity_ref) {
  bool status_changed = false;
  const auto entity = entity_ref.Get();
  if (const auto scene = Application::GetActiveScene(); scene->IsEntityValid(entity)) {
    const std::string tag = "##Entity" + std::to_string(scene->GetEntityHandle(entity));
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::Button(("Remove" + tag).c_str())) {
        entity_ref.Clear();
        status_changed = true;
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}

void EditorLayer::MouseEntitySelection() {
  const auto scene = GetScene();
  auto window_layer = Application::GetLayer<WindowLayer>();
  auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
#pragma region Scene Window
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Scene")) {
    ImVec2 view_port_size;
    // Using a Child allow to fill all the space of the window.
    // It also allows customization
    if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
      view_port_size = ImGui::GetWindowSize();
    }
#pragma region Gizmos and Entity Selection
    if (scene_camera_window_focused_ && !lock_entity_selection_ &&
        Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
      SetSelectedEntity(Entity());
    }
    if (scene_camera_window_focused_ && !lock_entity_selection_ && !using_gizmo_ &&
        Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press &&
        !(mouse_scene_window_position_.x < 0 || mouse_scene_window_position_.y < 0 ||
          mouse_scene_window_position_.x > view_port_size.x || mouse_scene_window_position_.y > view_port_size.y)) {
      if (const auto focused_entity = MouseEntitySelection(sceneCamera, mouse_scene_window_position_);
          focused_entity == Entity()) {
        SetSelectedEntity(Entity());
      } else {
        Entity walker = focused_entity;
        bool found = false;
        while (walker.GetIndex() != 0) {
          if (walker == selected_entity_) {
            found = true;
            break;
          }
          walker = scene->GetParent(walker);
        }
        if (found) {
          walker = scene->GetParent(walker);
          if (walker.GetIndex() == 0) {
            SetSelectedEntity(focused_entity);
          } else {
            SetSelectedEntity(walker);
          }
        } else {
          SetSelectedEntity(focused_entity);
        }
      }
    }
#pragma endregion
    ImGui::EndChild();
  }
  ImGui::End();
  ImGui::PopStyleVar();
#pragma endregion
}

Entity EditorLayer::MouseEntitySelection(const std::shared_ptr<Camera>& target_camera,
                                         const glm::vec2& mouse_position) const {
  Entity ret_val;
  const auto& g_buffer_normal = target_camera->g_buffer_normal_;
  const glm::vec2 resolution = target_camera->GetSize();
  glm::vec2 point = resolution;
  point.x = mouse_position.x;
  point.y -= mouse_position.y;
  if (point.x >= 0 && point.x < resolution.x && point.y >= 0 && point.y < resolution.y) {
    VkBufferImageCopy image_copy;
    image_copy.bufferOffset = 0;
    image_copy.bufferRowLength = 0;
    image_copy.bufferImageHeight = 0;
    image_copy.imageSubresource.layerCount = 1;
    image_copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    image_copy.imageSubresource.baseArrayLayer = 0;
    image_copy.imageSubresource.mipLevel = 0;
    image_copy.imageExtent.width = 1;
    image_copy.imageExtent.height = 1;
    image_copy.imageExtent.depth = 1;
    image_copy.imageOffset.x = point.x;
    image_copy.imageOffset.y = point.y;
    image_copy.imageOffset.z = 0;
    entity_index_read_buffer_->CopyFromImage(*g_buffer_normal, image_copy);
    if (const float instance_index_with_one_added =
            glm::roundEven(glm::detail::toFloat32(mapped_entity_index_data_[3]));
        instance_index_with_one_added > 0) {
      const auto render_layer = Application::GetLayer<RenderLayer>();
      const auto scene = GetScene();
      const auto handle = render_layer->render_instances_list[Platform::GetCurrentFrameIndex()]->GetInstanceHandle(static_cast<uint32_t>(instance_index_with_one_added - 1));
      if (handle != 0)
        ret_val = scene->GetEntity(handle);
    }
  }
  return ret_val;
}

bool EditorLayer::RenameEntity(const Entity& entity) {
  constexpr bool status_changed = false;
  if (const auto scene = Application::GetActiveScene(); scene->IsEntityValid(entity)) {
    const std::string tag = "##Entity" + std::to_string(scene->GetEntityHandle(entity));
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256];
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          scene->SetEntityName(entity, std::string(new_name));
          memset(new_name, 0, 256);
        }
        ImGui::EndMenu();
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}

bool EditorLayer::DragAndDropButton(AssetRef& target, const std::string& name,
                                    const std::vector<std::string>& acceptable_type_names, bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  const auto ptr = target.Get<IAsset>();
  bool status_changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.5f, 0, 1));
  if (ptr) {
    const auto title = ptr->GetTitle();
    ImGui::Button(title.c_str());
    DraggableAsset(ptr);
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
  status_changed = UnsafeDroppableAsset(target, acceptable_type_names) || status_changed;
  return status_changed;
}
bool EditorLayer::DragAndDropButton(PrivateComponentRef& target, const std::string& name,
                                    const std::vector<std::string>& acceptable_type_names, const bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  bool status_changed = false;
  const auto ptr = target.Get<IPrivateComponent>();
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.3f, 0, 1));
  if (ptr) {
    const auto scene = Application::GetActiveScene();
    ImGui::Button(scene->GetEntityName(ptr->GetOwner()).c_str());
    const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
    DraggablePrivateComponent(ptr);
    if (modifiable) {
      status_changed = Remove(target);
    }
  } else {
    ImGui::Button("none");
  }
  ImGui::PopStyleColor(1);
  status_changed = UnsafeDroppablePrivateComponent(target, acceptable_type_names) || status_changed;
  return status_changed;
}

void EditorLayer::LoadIcons() {
  assets_icons_["Project"] = Resources::CreateResource<Texture2D>("PROJECT_ICON");
  assets_icons_["Project"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/project.png");

  assets_icons_["Scene"] = Resources::CreateResource<Texture2D>("SCENE_ICON");
  assets_icons_["Scene"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/scene.png");

  assets_icons_["Binary"] = Resources::CreateResource<Texture2D>("BINARY_ICON");
  assets_icons_["Binary"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/binary.png");

  assets_icons_["Folder"] = Resources::CreateResource<Texture2D>("FOLDER_ICON");
  assets_icons_["Folder"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/folder.png");

  assets_icons_["Material"] = Resources::CreateResource<Texture2D>("MATERIAL_ICON");
  assets_icons_["Material"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/material.png");

  assets_icons_["Mesh"] = Resources::CreateResource<Texture2D>("MESH_ICON");
  assets_icons_["Mesh"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/mesh.png");

  assets_icons_["Prefab"] = Resources::CreateResource<Texture2D>("PREFAB_ICON");
  assets_icons_["Prefab"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/prefab.png");

  assets_icons_["Texture2D"] = Resources::CreateResource<Texture2D>("TEXTURE2D_ICON");
  assets_icons_["Texture2D"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/texture2d.png");

  assets_icons_["PlayButton"] = Resources::CreateResource<Texture2D>("PLAY_BUTTON_ICON");
  assets_icons_["PlayButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Navigation/PlayButton.png");

  assets_icons_["PauseButton"] = Resources::CreateResource<Texture2D>("PAUSE_BUTTON_ICON");
  assets_icons_["PauseButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                             "Editor/Navigation/PauseButton.png");

  assets_icons_["StopButton"] = Resources::CreateResource<Texture2D>("STOP_BUTTON_ICON");
  assets_icons_["StopButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Navigation/StopButton.png");

  assets_icons_["StepButton"] = Resources::CreateResource<Texture2D>("STEP_BUTTON_ICON");
  assets_icons_["StepButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Navigation/StepButton.png");

  assets_icons_["BackButton"] = Resources::CreateResource<Texture2D>("BACK_BUTTON_ICON");
  assets_icons_["BackButton"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Navigation/back.png");

  assets_icons_["LeftButton"] = Resources::CreateResource<Texture2D>("LEFT_BUTTON_ICON");
  assets_icons_["LeftButton"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Navigation/left.png");

  assets_icons_["RightButton"] = Resources::CreateResource<Texture2D>("RIGHT_BUTTON_ICON");
  assets_icons_["RightButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                             "Editor/Navigation/right.png");

  assets_icons_["RefreshButton"] = Resources::CreateResource<Texture2D>("REFRESH_BUTTON_ICON");
  assets_icons_["RefreshButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                               "Editor/Navigation/refresh.png");

  assets_icons_["InfoButton"] = Resources::CreateResource<Texture2D>("INFO_BUTTON_ICON");
  assets_icons_["InfoButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Console/InfoButton.png");

  assets_icons_["ErrorButton"] = Resources::CreateResource<Texture2D>("ERROR_BUTTON_ICON");
  assets_icons_["ErrorButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                             "Editor/Console/ErrorButton.png");

  assets_icons_["WarningButton"] = Resources::CreateResource<Texture2D>("WARNING_BUTTON_ICON");
  assets_icons_["WarningButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                               "Editor/Console/WarningButton.png");
}

void EditorLayer::CameraWindowDragAndDrop() const {
  if (AssetRef asset_ref;
      UnsafeDroppableAsset(asset_ref, {"Scene", "Prefab", "Mesh", "Strands", "Cubemap", "EnvironmentalMap"})) {
    const auto scene = GetScene();
    if (const auto asset = asset_ref.Get<IAsset>(); !Application::IsPlaying() && asset->GetTypeName() == "Scene") {
      const auto new_scene = std::dynamic_pointer_cast<Scene>(asset);
      ProjectManager::SetStartScene(new_scene);
      Application::Attach(new_scene);
    }

    else if (asset->GetTypeName() == "Prefab") {
      const auto entity = std::dynamic_pointer_cast<Prefab>(asset)->ToEntity(scene, true, true);
      scene->SetEntityName(entity, asset->GetTitle());
    } else if (asset->GetTypeName() == "Mesh") {
      const auto entity = scene->CreateEntity(asset->GetTitle());
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      mesh_renderer->mesh.Set<Mesh>(std::dynamic_pointer_cast<Mesh>(asset));
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      mesh_renderer->material.Set<Material>(material);
    }

    else if (asset->GetTypeName() == "Strands") {
      const auto entity = scene->CreateEntity(asset->GetTitle());
      const auto strands_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
      strands_renderer->strands.Set<Strands>(std::dynamic_pointer_cast<Strands>(asset));
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      strands_renderer->material.Set<Material>(material);
    } else if (asset->GetTypeName() == "EnvironmentalMap") {
      scene->environment.environmental_map = std::dynamic_pointer_cast<EnvironmentalMap>(asset);
    } else if (asset->GetTypeName() == "Cubemap") {
      const auto main_camera = scene->main_camera.Get<Camera>();
      main_camera->skybox = std::dynamic_pointer_cast<Cubemap>(asset);
    }
  }
}

void EditorLayer::MoveCamera(const glm::quat& target_rotation, const glm::vec3& target_position,
                             const float& transition_time) {
  auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
  previous_rotation_ = sceneCameraRotation;
  previous_position_ = sceneCameraPosition;
  transition_time_ = transition_time;
  transition_timer_ = static_cast<float>(Times::Now());
  target_rotation_ = target_rotation;
  target_position_ = target_position;
  lock_camera = true;
}

bool EditorLayer::LocalPositionSelected() const {
  return local_position_selected_;
}

bool EditorLayer::LocalRotationSelected() const {
  return local_rotation_selected_;
}

bool EditorLayer::LocalScaleSelected() const {
  return local_scale_selected_;
}

glm::vec3& EditorLayer::UnsafeGetPreviouslyStoredPosition() {
  return previously_stored_position_;
}

glm::vec3& EditorLayer::UnsafeGetPreviouslyStoredRotation() {
  return previously_stored_rotation_;
}

glm::vec3& EditorLayer::UnsafeGetPreviouslyStoredScale() {
  return previously_stored_scale_;
}