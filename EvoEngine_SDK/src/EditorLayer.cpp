#include "EditorLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Cubemap.hpp"
#include "EnvironmentalMap.hpp"
#include "ILayer.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "StrandsRenderer.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"
#include <cmath>
#include <yaml-cpp/yaml.h>
using namespace evo_engine;
void EditorLayer::OnCreate() {
  const auto window_layer = Application::GetLayer<WindowLayer>();
  if (!window_layer) {
    throw std::runtime_error("WindowLayer not present!");
  }
#pragma region Default ImGui Style
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowRounding = 5.3f;
  style.FrameRounding = 2.3f;
  style.ScrollbarRounding = 0;

  style.Colors[ImGuiCol_Text] = ImVec4(0.90f, 0.90f, 0.90f, 0.90f);
  style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.30f, 0.30f, 0.30f, 0.90f);
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.09f, 0.09f, 0.15f, 1.00f);
  style.Colors[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  style.Colors[ImGuiCol_PopupBg] = ImVec4(0.05f, 0.05f, 0.10f, 0.85f);
  style.Colors[ImGuiCol_Border] = ImVec4(0.70f, 0.70f, 0.70f, 0.65f);
  style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  style.Colors[ImGuiCol_FrameBg] = ImVec4(0.00f, 0.00f, 0.01f, 1.00f);
  style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.90f, 0.80f, 0.80f, 0.40f);
  style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.90f, 0.65f, 0.65f, 0.45f);
  style.Colors[ImGuiCol_TitleBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.83f);
  style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.40f, 0.40f, 0.80f, 0.20f);
  style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.00f, 0.00f, 0.00f, 0.87f);
  style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.01f, 0.01f, 0.02f, 0.80f);
  style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.20f, 0.25f, 0.30f, 0.60f);
  style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.55f, 0.53f, 0.55f, 0.51f);
  style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.56f, 1.00f);
  style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.56f, 0.56f, 0.56f, 0.91f);
  style.Colors[ImGuiCol_CheckMark] = ImVec4(0.90f, 0.90f, 0.90f, 0.83f);
  style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.70f, 0.70f, 0.70f, 0.62f);
  style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.30f, 0.30f, 0.30f, 0.84f);
  style.Colors[ImGuiCol_Button] = ImVec4(0.48f, 0.72f, 0.89f, 0.49f);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.50f, 0.69f, 0.99f, 0.68f);
  style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.80f, 0.50f, 0.50f, 1.00f);
  style.Colors[ImGuiCol_Header] = ImVec4(0.30f, 0.69f, 1.00f, 0.53f);
  style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.44f, 0.61f, 0.86f, 1.00f);
  style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.38f, 0.62f, 0.83f, 1.00f);
  style.Colors[ImGuiCol_TabSelectedOverline] = ImVec4(0.70f, 0.30f, 0.30f, 1.00f);
  style.Colors[ImGuiCol_TabHovered] = ImVec4(0.70f, 0.00f, 0.30f, 1.00f);
  style.Colors[ImGuiCol_TabSelected] = ImVec4(0.50f, 0.00f, 0.50f, 1.00f);
  style.Colors[ImGuiCol_ResizeGrip] = ImVec4(1.00f, 1.00f, 1.00f, 0.85f);
  style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(1.00f, 1.00f, 1.00f, 0.60f);
  style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(1.00f, 1.00f, 1.00f, 0.90f);
  style.Colors[ImGuiCol_PlotLines] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
  style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.00f, 0.00f, 1.00f, 0.35f);
#pragma endregion

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
      // local_position_selected_ = true;
      // local_rotation_selected_ = false;
      // local_scale_selected_ = false;

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
  switch (Platform::Constants::texture_2d) {
    case VK_FORMAT_R32G32B32A32_SFLOAT: {
      entity_index_read_buffer.size = sizeof(float) * 4;
      break;
    }
    case VK_FORMAT_R16G16B16A16_SFLOAT: {
      entity_index_read_buffer.size = sizeof(glm::detail::hdata) * 4;
      break;
    }
  }

  entity_index_read_buffer.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  entity_index_read_buffer.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo entity_index_read_buffer_create_info{};
  entity_index_read_buffer_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  entity_index_read_buffer_create_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  entity_index_read_buffer_ = std::make_unique<Buffer>(entity_index_read_buffer, entity_index_read_buffer_create_info);
  vmaMapMemory(Platform::GetVmaAllocator(), entity_index_read_buffer_->GetVmaAllocation(),
               static_cast<void**>(static_cast<void*>(&mapped_entity_index_data_)));

  const auto scene_camera = Serialization::ProduceSerializable<Camera>();
  scene_camera->camera_settings.clear_color = glm::vec4(59.0f / 255.0f, 85 / 255.0f, 143 / 255.f, 1.f);
  scene_camera->camera_settings.use_clear_color = false;
  scene_camera->OnCreate();
  scene_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  RegisterEditorCamera(scene_camera);
  scene_camera_handle_ = scene_camera->GetHandle();
  auto& editor_camera = editor_cameras_[scene_camera_handle_];
  editor_camera.position = default_scene_camera_position;
  editor_camera.rotation = default_scene_camera_rotation;
}

void EditorLayer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "enable_inspection" << YAML::Value << enable_inspection;
  out << YAML::Key << "show_console_window" << YAML::Value << show_console_window;
  out << YAML::Key << "show_scene_camera_debug" << YAML::Value << show_scene_camera_debug;
  out << YAML::Key << "show_scene_window" << YAML::Value << show_scene_window;
  out << YAML::Key << "show_camera_window" << YAML::Value << show_camera_window;
  out << YAML::Key << "show_camera_info" << YAML::Value << show_camera_info;
  out << YAML::Key << "show_play_buttons" << YAML::Value << show_play_buttons;
  out << YAML::Key << "show_scene_info" << YAML::Value << show_scene_info;
  out << YAML::Key << "show_entity_explorer_window" << YAML::Value << show_entity_explorer_window;
  out << YAML::Key << "show_entity_inspector_window" << YAML::Value << show_entity_inspector_window;
  out << YAML::Key << "main_camera_focus_override" << YAML::Value << main_camera_focus_override;
  out << YAML::Key << "scene_camera_focus_override" << YAML::Value << scene_camera_focus_override;
  out << YAML::Key << "selected_hierarchy_display_mode" << YAML::Value << selected_hierarchy_display_mode;
  out << YAML::Key << "velocity" << YAML::Value << velocity;
  out << YAML::Key << "sensitivity" << YAML::Value << sensitivity;
  out << YAML::Key << "apply_transform_to_main_camera" << YAML::Value << apply_transform_to_main_camera;
  out << YAML::Key << "default_scene_camera_rotation" << YAML::Value << default_scene_camera_rotation;
  out << YAML::Key << "default_scene_camera_position" << YAML::Value << default_scene_camera_position;
  
  out << YAML::Key << "lock_entity_selection" << YAML::Value << lock_entity_selection_;
  out << YAML::Key << "highlight_selection" << YAML::Value << highlight_selection_;
  out << YAML::Key << "enable_gizmos" << YAML::Value << enable_gizmos;
  out << YAML::Key << "enable_view_gizmos" << YAML::Value << enable_view_gizmos;
  out << YAML::Key << "transform_read_only" << YAML::Value << transform_read_only;
  out << YAML::Key << "enable_console_logs" << YAML::Value << enable_console_logs_;
  out << YAML::Key << "enable_console_errors" << YAML::Value << enable_console_errors_;
  out << YAML::Key << "enable_console_warnings" << YAML::Value << enable_console_warnings_;
  out << YAML::Key << "main_camera_resolution_x" << YAML::Value << main_camera_resolution_x;
  out << YAML::Key << "main_camera_resolution_y" << YAML::Value << main_camera_resolution_y;
  out << YAML::Key << "main_camera_allow_auto_resize" << YAML::Value << main_camera_allow_auto_resize;
  out << YAML::Key << "scene_camera_resolution_multiplier" << YAML::Value << scene_camera_resolution_multiplier;
  out << YAML::Key << "main_camera_resolution_multiplier" << YAML::Value << main_camera_resolution_multiplier_;
  out << YAML::Key << "local_position_selected" << YAML::Value << local_position_selected_;
  out << YAML::Key << "local_rotation_selected" << YAML::Value << local_rotation_selected_;
  out << YAML::Key << "local_scale_selected" << YAML::Value << local_scale_selected_;

  if (editor_cameras_.find(scene_camera_handle_) != editor_cameras_.end()) {
    auto& cam = editor_cameras_.at(scene_camera_handle_);
    out << YAML::Key << "scene_camera_position" << YAML::Value << cam.position;
    out << YAML::Key << "scene_camera_rotation" << YAML::Value << cam.rotation;
    if (cam.camera) {
      out << YAML::Key << "scene_camera_settings" << YAML::Value << YAML::BeginMap;
      cam.camera->Serialize(out);
      out << YAML::EndMap;
    }
  }

  uint64_t entity_handle = 0;
  if (const auto scene = Application::GetActiveScene()) {
    if (scene->IsEntityValid(selected_entity_)) {
      entity_handle = scene->GetEntityHandle(selected_entity_).GetValue();
    }
  }
  out << YAML::Key << "selected_entity" << YAML::Value << entity_handle;

  uint64_t asset_handle = 0;
  if (inspecting_asset) {
    asset_handle = inspecting_asset->GetHandle().GetValue();
  }
  out << YAML::Key << "inspecting_asset" << YAML::Value << asset_handle;

  if (const char* ini_settings = ImGui::SaveIniSettingsToMemory()) {
    out << YAML::Key << "ImGuiIni" << YAML::Value << std::string(ini_settings);
  }
}

void EditorLayer::Deserialize(const YAML::Node& in) {
  if (in["enable_inspection"])
    enable_inspection = in["enable_inspection"].as<bool>();
  if (in["show_console_window"])
    show_console_window = in["show_console_window"].as<bool>();
  if (in["show_scene_camera_debug"])
    show_scene_camera_debug = in["show_scene_camera_debug"].as<bool>();
  if (in["show_scene_window"])
    show_scene_window = in["show_scene_window"].as<bool>();
  if (in["show_camera_window"])
    show_camera_window = in["show_camera_window"].as<bool>();
  if (in["show_camera_info"])
    show_camera_info = in["show_camera_info"].as<bool>();
  if (in["show_play_buttons"])
    show_play_buttons = in["show_play_buttons"].as<bool>();
  if (in["show_scene_info"])
    show_scene_info = in["show_scene_info"].as<bool>();
  if (in["show_entity_explorer_window"])
    show_entity_explorer_window = in["show_entity_explorer_window"].as<bool>();
  if (in["show_entity_inspector_window"])
    show_entity_inspector_window = in["show_entity_inspector_window"].as<bool>();
  if (in["main_camera_focus_override"])
    main_camera_focus_override = in["main_camera_focus_override"].as<bool>();
  if (in["scene_camera_focus_override"])
    scene_camera_focus_override = in["scene_camera_focus_override"].as<bool>();
  if (in["selected_hierarchy_display_mode"])
    selected_hierarchy_display_mode = in["selected_hierarchy_display_mode"].as<int>();
  if (in["velocity"])
    velocity = in["velocity"].as<float>();
  if (in["sensitivity"])
    sensitivity = in["sensitivity"].as<float>();
  if (in["apply_transform_to_main_camera"])
    apply_transform_to_main_camera = in["apply_transform_to_main_camera"].as<bool>();

  if (in["lock_entity_selection"])
    lock_entity_selection_ = in["lock_entity_selection"].as<bool>();
  if (in["highlight_selection"])
    highlight_selection_ = in["highlight_selection"].as<bool>();
  if (in["enable_gizmos"])
    enable_gizmos = in["enable_gizmos"].as<bool>();
  if (in["enable_view_gizmos"])
    enable_view_gizmos = in["enable_view_gizmos"].as<bool>();
  if (in["transform_read_only"])
    transform_read_only = in["transform_read_only"].as<bool>();
  if (in["enable_console_logs"])
    enable_console_logs_ = in["enable_console_logs"].as<bool>();
  if (in["enable_console_errors"])
    enable_console_errors_ = in["enable_console_errors"].as<bool>();
  if (in["enable_console_warnings"])
    enable_console_warnings_ = in["enable_console_warnings"].as<bool>();
  if (in["main_camera_resolution_x"])
    main_camera_resolution_x = in["main_camera_resolution_x"].as<int>();
  if (in["main_camera_resolution_y"])
    main_camera_resolution_y = in["main_camera_resolution_y"].as<int>();
  if (in["main_camera_allow_auto_resize"])
    main_camera_allow_auto_resize = in["main_camera_allow_auto_resize"].as<bool>();
  if (in["scene_camera_resolution_multiplier"])
    scene_camera_resolution_multiplier = in["scene_camera_resolution_multiplier"].as<float>();
  if (in["main_camera_resolution_multiplier"])
    main_camera_resolution_multiplier_ = in["main_camera_resolution_multiplier"].as<float>();
  if (in["local_position_selected"])
    local_position_selected_ = in["local_position_selected"].as<bool>();
  if (in["local_rotation_selected"])
    local_rotation_selected_ = in["local_rotation_selected"].as<bool>();
  if (in["local_scale_selected"])
    local_scale_selected_ = in["local_scale_selected"].as<bool>();

  if (in["default_scene_camera_rotation"])
    default_scene_camera_rotation = in["default_scene_camera_rotation"].as<glm::quat>();
  if (in["default_scene_camera_position"])
    default_scene_camera_position = in["default_scene_camera_position"].as<glm::vec3>();

  if (in["scene_camera_position"]) {
    auto pos = in["scene_camera_position"].as<glm::vec3>();
    default_scene_camera_position = pos;
    if (editor_cameras_.find(scene_camera_handle_) != editor_cameras_.end()) {
      editor_cameras_[scene_camera_handle_].position = pos;
    }
  }
  if (in["scene_camera_rotation"]) {
    auto rot = in["scene_camera_rotation"].as<glm::quat>();
    default_scene_camera_rotation = rot;
    if (editor_cameras_.find(scene_camera_handle_) != editor_cameras_.end()) {
      editor_cameras_[scene_camera_handle_].rotation = rot;
    }
  }
  
  if (in["scene_camera_settings"]) {
    if (editor_cameras_.find(scene_camera_handle_) != editor_cameras_.end() && editor_cameras_[scene_camera_handle_].camera) {
      editor_cameras_[scene_camera_handle_].camera->Deserialize(in["scene_camera_settings"]);
    }
  }

  if (in["selected_entity"]) {
    uint64_t handle = in["selected_entity"].as<uint64_t>();
    if (handle != 0) {
      Application::RegisterPostAttachSceneFunction([handle](const std::shared_ptr<Scene>& scene) {
        if (auto editor_layer = Application::GetLayer<EditorLayer>()) {
          if (auto entity = scene->GetEntity(Handle(handle)); scene->IsEntityValid(entity)) {
            editor_layer->SetSelectedEntity(entity);
          }
        }
      });
    }
  }

  if (in["inspecting_asset"]) {
    uint64_t handle = in["inspecting_asset"].as<uint64_t>();
    if (handle != 0) {
      Application::RegisterPostAttachSceneFunction([handle](const std::shared_ptr<Scene>& scene) {
        if (auto editor_layer = Application::GetLayer<EditorLayer>()) {
          if (auto asset = AssetManager::GetAssetImpl(Handle(handle))) {
            editor_layer->inspecting_asset = asset;
          }
        }
      });
    }
  }

  if (in["ImGuiIni"]) {
    const std::string ini_settings = in["ImGuiIni"].as<std::string>();
    ImGui::LoadIniSettingsFromMemory(ini_settings.c_str(), ini_settings.size());
  }
}

void EditorLayer::OnDestroy() {
  try {
    Platform::WaitForDeviceIdle();
    editor_cameras_.clear();
    gizmo_mesh_tasks_.clear();
    gizmo_instanced_mesh_tasks_.clear();
    gizmo_strands_tasks_.clear();
    vmaUnmapMemory(Platform::GetVmaAllocator(), entity_index_read_buffer_->GetVmaAllocation());
    entity_index_read_buffer_.reset();
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImNodes::DestroyContext();
    ImGui::DestroyContext();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to unload resources: " + std::string(e.what()));
  } catch (...) {
    EVOENGINE_ERROR("Unknown error during EditorLayer::OnDestroy");
  }
}

void EditorLayer::PreUpdate() {
  InitializeImGui();

  const bool save_shortcut_pressed =
      (Input::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold ||
       Input::GetKey(GLFW_KEY_RIGHT_CONTROL) == Input::KeyActionType::Hold) &&
      Input::GetKey(GLFW_KEY_S) == Input::KeyActionType::Press;

  const auto scene = Application::GetActiveScene();
  if (lock_camera) {
    auto& [sceneCameraRotation, sceneCameraPosition, scene_camera] = editor_cameras_.at(scene_camera_handle_);
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
  gizmo_mesh_tasks_.clear();
  gizmo_instanced_mesh_tasks_.clear();
  gizmo_strands_tasks_.clear();
  main_camera_focus_override = false;
  scene_camera_focus_override = false;
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 5));
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      if (ImGui::BeginMenu("Layer Inspection")) {
        for (const auto& layer : Application::GetLayers()) {
          ImGui::Checkbox(layer->layer_name_.c_str(), &layer->enable_inspection);
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Project")) {
      ImGui::EndMenu();
    }
    ImGui::Separator();
    switch (Application::GetApplicationStatus()) {
      case Application::ExecutionStatus::NotPlaying: {
        if (ImGui::ImageButton(editor_icons_["PlayButton"]->GetImTextureId(), {20, 20}, {0, 1}, {1, 0}, 2)) {
          Application::Play();
        }
        if (ImGui::ImageButton(editor_icons_["StepButton"]->GetImTextureId(), {20, 20}, {0, 1}, {1, 0}, 2)) {
          Application::Step();
        }
        break;
      }
      case Application::ExecutionStatus::Playing: {
        if (ImGui::ImageButton(editor_icons_["PauseButton"]->GetImTextureId(), {20, 20}, {0, 1}, {1, 0}, 2)) {
          Application::Pause();
        }
        if (ImGui::ImageButton(editor_icons_["StopButton"]->GetImTextureId(), {20, 20}, {0, 1}, {1, 0}, 2)) {
          Application::Stop();
        }
        break;
      }
      case Application::ExecutionStatus::Pause: {
        if (ImGui::ImageButton(editor_icons_["PlayButton"]->GetImTextureId(), {20, 20}, {0, 1}, {1, 0}, 2)) {
          Application::Play();
        }
        if (ImGui::ImageButton(editor_icons_["StepButton"]->GetImTextureId(), {20, 20}, {0, 1}, {1, 0}, 2)) {
          Application::Step();
        }
        if (ImGui::ImageButton(editor_icons_["StopButton"]->GetImTextureId(), {20, 20}, {0, 1}, {1, 0}, 2)) {
          Application::Stop();
        }
        break;
      }
      case Application::ExecutionStatus::Uninitialized:
        break;
      case Application::ExecutionStatus::Step:
        break;
      case Application::ExecutionStatus::OnDestroy:
        break;
    }
    ImGui::EndMainMenuBar();
  }
  ImGui::PopStyleVar(1);
  mouse_scene_window_position_ = glm::vec2(FLT_MAX, -FLT_MAX);
  if (show_scene_window) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
    if (ImGui::Begin("Scene")) {
      if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
        // Using a Child allow to fill all the space of the window.
        // It also allows customization
        if (scene_camera_window_focused_ || scene_camera_window_hovered_) {
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
        if (main_camera_window_focused_ || main_camera_window_hovered_) {
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
  if (scene && show_scene_window)
    ResizeCameras();

  if (scene && !main_camera_window_focused_) {
    auto& pressed_keys = scene->pressed_keys_;
    pressed_keys.clear();
  }

  if (scene && apply_transform_to_main_camera && !Application::IsPlaying()) {
    if (const auto camera = scene->main_camera.Get<Camera>(); camera && scene->IsEntityValid(camera->GetOwner())) {
      auto& [sceneCameraRotation, sceneCameraPosition, scene_camera] = editor_cameras_.at(scene_camera_handle_);
      GlobalTransform global_transform;
      global_transform.SetPosition(sceneCameraPosition);
      global_transform.SetRotation(sceneCameraRotation);
      scene->SetDataComponent(camera->GetOwner(), global_transform);
    }
  }

  if (scene && !scene->IsEntityValid(selected_entity_)) {
    SetSelectedEntity(Entity());
  }
  if (const auto render_layer = Application::GetLayer<RenderLayer>();
      render_layer && render_layer->need_fade_ != 0 && selection_alpha_ < 256) {
    selection_alpha_ += static_cast<int>(static_cast<float>(Times::DeltaTime()) * 1280);
  }

  selection_alpha_ = glm::clamp(selection_alpha_, 0, 256);

  const auto editor_layer = std::dynamic_pointer_cast<EditorLayer>(GetSelf());
  if (show_entity_explorer_window) {
    ImGui::Begin("Entity Explorer");
    if (scene) {
      if (ImGui::BeginPopupContextWindow("NewEntityPopup")) {
        if (ImGui::Button("Create new entity")) {
          scene->CreateEntity(basic_entity_archetype_);
        }
        ImGui::EndPopup();
      }
      const char* hierarchy_display_mode[]{"Archetype", "Hierarchy"};

      ImGui::Combo("Display mode", &selected_hierarchy_display_mode, hierarchy_display_mode,
                   IM_ARRAYSIZE(hierarchy_display_mode));
      std::string title = scene->GetTitle();
      if (ImGui::CollapsingHeader(title.c_str(), ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_OpenOnArrow)) {
        DraggableAsset(scene);
        RenameAsset(scene);
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
          inspecting_asset = scene;
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
    } else {
      ImGui::Text("No Scene!");
    }
    ImGui::End();
  }
  if (show_entity_inspector_window) {
    ImGui::Begin("Entity Inspector");
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
    if (scene) {
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
              if (info == "TransformUpdateFlag" || info == "GlobalTransform")
                return;
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
              ImGui::Checkbox(data.private_component_data->GetTypeName().c_str(),
                              &data.private_component_data->enabled_);
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
    } else {
      ImGui::Text("No Scene!");
    }
    ImGui::PopItemWidth();
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
  if (scene && show_scene_camera_debug) {
    if (ImGui::Begin("Scene Camera Debug")) {
      static float debug_scale = 0.25f;
      ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
      debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
      auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
      sceneCamera->DebugViews(debug_scale);
    }
    ImGui::End();
  }

  if (scene) {
    for (const auto& layer : Application::GetLayers()) {
      if (layer->enable_inspection) {
        ImGui::Begin(layer->layer_name_.c_str());
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
        layer->OnInspect(editor_layer);
        ImGui::PopItemWidth();
        ImGui::End();
      }
    }
  }

  Resources::OnInspect(editor_layer);
  AssetManager::OnInspect(editor_layer);
  ProjectManager::OnInspect(editor_layer);

  if (save_shortcut_pressed) {
    ProjectManager::SaveProject();
  }
}
void EditorLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Scene Window", &show_scene_window);
  if (show_scene_window) {
    ImGui::Checkbox("Scene Camera Debug Window", &show_scene_camera_debug);
    ImGui::Checkbox("Scene Window Info", &show_scene_info);
  }
  ImGui::Checkbox("Main Camera Window", &show_camera_window);
  if (show_camera_window) {
    ImGui::Checkbox("Main Camera Window Info", &show_camera_info);
  }
  ImGui::Checkbox("Entity Explorer", &show_entity_explorer_window);
  ImGui::Checkbox("Entity Inspector", &show_entity_inspector_window);
  ImGui::Checkbox("Console", &show_console_window);

  if (ImGui::TreeNode("Scene camera settings")) {
    ImGui::Checkbox("View Gizmos", &enable_view_gizmos);
    if (ImGui::Button("Reset camera")) {
      MoveCamera(default_scene_camera_rotation, default_scene_camera_position);
    }
    auto& [sceneCameraRotation, sceneCameraPosition, scene_camera] = editor_cameras_.at(scene_camera_handle_);
    if (ImGui::Button("Set default camera position")) {
      default_scene_camera_position = sceneCameraPosition;
      default_scene_camera_rotation = sceneCameraRotation;
    }
    ImGui::DragFloat("Speed", &velocity, 0.1f, 0, 0, "%.1f");
    ImGui::DragFloat("Sensitivity", &sensitivity, 0.1f, 0, 0, "%.1f");
    ImGui::Checkbox("Copy Transform", &apply_transform_to_main_camera);
    ImGui::DragFloat("Resolution", &scene_camera_resolution_multiplier, 0.1f, 0.1f, 4.0f);

    if (ImGui::TreeNode("Camera settings")) {
      scene_camera->OnInspect(editor_layer);
      ImGui::TreePop();
    }

    ImGui::TreePop();
  }
}

void EditorLayer::InitializeImGui() {
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();

#pragma region Dock
  static bool opt_fullscreen_persistent = true;
  const bool opt_fullscreen = opt_fullscreen_persistent;
  static ImGuiDockNodeFlags dock_space_flags = ImGuiDockNodeFlags_None;

  // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
  // because it would be confusing to have two docking targets within each others.
  ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  if (opt_fullscreen) {
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
  dock_space_id = ImGui::GetID("MyDockSpace");
  ImGui::DockSpace(dock_space_id, ImVec2(0.0f, 0.0f), dock_space_flags);
  ImGui::End();
#pragma endregion
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
  const auto scene = GetScene();
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
    const auto handle = scene->GetEntityHandle(entity);
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
  auto& [sceneCameraRotation, sceneCameraPosition, scene_camera] = editor_cameras_.at(scene_camera_handle_);
#pragma region Scene Window

  scene_camera_window_focused_ = false;
  scene_camera_window_hovered_ = false;
  if (ImGui::Begin("Scene")) {
    if (scene) {
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
      ImVec2 view_port_size;
      static int corner = 1;
      if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
          scene_camera_window_focused_ = true;
        }
        if (ImGui::IsWindowHovered()) {
          scene_camera_window_hovered_ = true;
        }
        const bool scene_activate_click = scene_camera_window_hovered_ &&
                                          (Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press ||
                                           Input::GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Press ||
                                           Input::GetKey(GLFW_MOUSE_BUTTON_MIDDLE) == Input::KeyActionType::Press);
        if (scene_activate_click) {
          ImGui::SetWindowFocus();
          scene_camera_window_focused_ = true;
        }
        {
          const auto mp = ImGui::GetMousePos();
          const auto wp = ImGui::GetWindowPos();
          mouse_scene_window_position_ = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        }
        view_port_size = ImGui::GetWindowSize();
        scene_camera_resolution_x_ = view_port_size.x * scene_camera_resolution_multiplier;
        scene_camera_resolution_y_ = view_port_size.y * scene_camera_resolution_multiplier;
        const ImVec2 overlay_pos = ImGui::GetWindowPos();
        if (scene_camera && scene_camera->rendered_) {
          ImGui::Image(scene_camera->GetRenderTexture()->GetColorImTextureId(),
                       ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));
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
              ImGui::BeginChild("Info", ImVec2(150, 150), child_flags, window_flags)) {
            ImGui::Text("Info:");
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            std::string draw_call_info = {};
            const auto current_frame_index = Platform::GetCurrentFrameIndex();
            if (graphics.prim_count[current_frame_index] < 999)
              draw_call_info += std::to_string(graphics.prim_count[current_frame_index]);
            else if (graphics.prim_count[current_frame_index] < 999999)
              draw_call_info += std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000)) + "K";
            else
              draw_call_info +=
                  std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000000)) + "M";
            draw_call_info += " tris";
            ImGui::Text(draw_call_info.c_str());
            ImGui::Text("%d drawcall", graphics.draw_call[current_frame_index]);
            ImGui::Text("Idle: %.3f", graphics.cpu_wait_time);
            ImGui::Separator();
            if (ImGui::IsMousePosValid()) {
              const auto pos = Input::GetMousePosition();
              ImGui::Text("Mouse: [%.0f,%.0f]", pos.x, pos.y);
            } else {
              ImGui::Text("Mouse: <invalid>");
            }
            uint32_t mode = static_cast<uint32_t>(scene_camera->camera_render_mode);
            if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
              scene_camera->camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
              scene_camera->frame_count_ = 0;
            }
          }
          ImGui::EndChild();
        }
        if (scene_camera_window_focused_ || scene_camera_window_hovered_) {
#pragma region Scene Camera Controller
          static bool is_dragging_previously = false;
          bool mouse_out_of_bounds = mouse_scene_window_position_.x < 0 || mouse_scene_window_position_.y < 0 ||
                                     mouse_scene_window_position_.x > view_port_size.x || mouse_scene_window_position_.y > view_port_size.y;
          bool mouse_drag = true;
          if (Input::GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold) {
            mouse_drag = false;
          } else if (!is_dragging_previously && mouse_out_of_bounds) {
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

            float current_velocity = velocity;
            if (Input::GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
              current_velocity *= 5.0f;
            }

            if (Input::GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
              sceneCameraPosition += front * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (Input::GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
              sceneCameraPosition -= front * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (Input::GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
              sceneCameraPosition -= right * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (Input::GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
              sceneCameraPosition += right * static_cast<float>(Times::DeltaTime()) * current_velocity;
            }
            if (Input::GetKey(GLFW_KEY_E) == Input::KeyActionType::Hold) {
              sceneCameraPosition.y += current_velocity * static_cast<float>(Times::DeltaTime());
            }
            if (Input::GetKey(GLFW_KEY_Q) == Input::KeyActionType::Hold) {
              sceneCameraPosition.y -= current_velocity * static_cast<float>(Times::DeltaTime());
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
          }

          bool has_selected_center = false;
          glm::vec3 selected_center(0.0f);
          glm::vec3 orbit_center(0.0f);
          const auto is_finite_vec3 = [](const glm::vec3& value) {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
          };
          if (scene->IsEntityValid(selected_entity_)) {
            const auto selected_bound = scene->GetEntityBoundingBox(selected_entity_);
            selected_center = selected_bound.Center();
            if (!is_finite_vec3(selected_center)) {
              selected_center = scene->GetDataComponent<GlobalTransform>(selected_entity_).GetPosition();
            }
            has_selected_center = true;
            orbit_center = selected_center;
            last_orbit_target_entity_ = selected_entity_;
          } else if (scene->IsEntityValid(last_orbit_target_entity_)) {
            orbit_center = scene->GetEntityBoundingBox(last_orbit_target_entity_).Center();
            if (!is_finite_vec3(orbit_center)) {
              orbit_center = scene->GetDataComponent<GlobalTransform>(last_orbit_target_entity_).GetPosition();
            }
          }

          const bool middle_mouse_orbit = Input::GetKey(GLFW_MOUSE_BUTTON_MIDDLE) == Input::KeyActionType::Hold;
          if (!lock_camera && middle_mouse_orbit) {
            const auto& mouse_delta = ImGui::GetIO().MouseDelta;
            if (mouse_delta.x != 0.0f || mouse_delta.y != 0.0f) {
              const float orbit_sensitivity = sensitivity * 2.0f;
              auto offset = sceneCameraPosition - orbit_center;
              float distance = glm::length(offset);
              distance = glm::max(distance, 0.001f);
              offset = glm::rotate(offset, glm::radians(-mouse_delta.x * orbit_sensitivity), glm::vec3(0, 1, 0));

              const auto view_dir = glm::normalize(orbit_center - sceneCameraPosition);
              auto orbit_right = glm::cross(view_dir, glm::vec3(0, 1, 0));
              if (glm::length(orbit_right) > glm::epsilon<float>()) {
                orbit_right = glm::normalize(orbit_right);
                const auto candidate_offset =
                    glm::rotate(offset, glm::radians(-mouse_delta.y * orbit_sensitivity), orbit_right);
                const auto candidate_dir = glm::normalize(-candidate_offset);
                if (glm::abs(glm::dot(candidate_dir, glm::vec3(0, 1, 0))) < 0.99f) {
                  offset = candidate_offset;
                }
              }

              sceneCameraPosition = orbit_center + offset;
              const auto front = glm::normalize(orbit_center - sceneCameraPosition);
              const auto right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
              const auto up = glm::normalize(glm::cross(right, front));
              sceneCameraRotation = glm::quatLookAt(front, up);
            }
          }

          if (scene_camera_window_hovered_ || mouse_drag) {
            const bool ctrl_held = Input::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold ||
                                   Input::GetKey(GLFW_KEY_RIGHT_CONTROL) == Input::KeyActionType::Hold;
            if (const float scroll = ImGui::GetIO().MouseWheel; scroll != 0.0f && !ctrl_held) {
              const bool right_mouse_held = Input::GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Hold;
              if (right_mouse_held) {
                velocity = glm::max(0.001f, velocity * (1.0f + scroll * 0.1f));
              } else if (has_selected_center) {
                const glm::vec3 front = sceneCameraRotation * glm::vec3(0, 0, -1);
                auto offset = sceneCameraPosition - selected_center;
                float distance = glm::length(offset);
                float move_amount = scroll * glm::max(0.05f, distance * 0.025f);
                if (move_amount > distance - 0.01f) {
                  move_amount = distance - 0.01f;
                }
                sceneCameraPosition += front * move_amount;
              } else if (!lock_camera) {
                const glm::vec3 front = sceneCameraRotation * glm::vec3(0, 0, -1);
                const float move_amount = scroll * glm::max(0.05f, velocity * 0.125f);
                sceneCameraPosition += front * move_amount;
              }
            }
          }

          if (!lock_camera && Input::GetKey(GLFW_KEY_KP_DECIMAL) == Input::KeyActionType::Press) {
            if (Input::GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold || Input::GetKey(GLFW_KEY_RIGHT_SHIFT) == Input::KeyActionType::Hold) {
              scene_camera->camera_settings = CameraSettings();
              MoveCamera(default_scene_camera_rotation, default_scene_camera_position, 0.2f);
            } else if (scene->IsEntityValid(selected_entity_)) {
              const auto bound = scene->GetEntityBoundingBox(selected_entity_);
              const auto center = bound.Center();
              float radius = glm::length(bound.Size()) * 0.5f;
              radius = glm::max(radius, 0.1f);

              const float aspect = glm::max(scene_camera->GetSizeRatio(), 0.001f);    
              const float fov_y = glm::radians(scene_camera->camera_settings.fov * 0.5f);
              const float half_fov_y = glm::max(0.001f, fov_y * 0.5f);
              const float half_fov_x = glm::atan(glm::tan(half_fov_y) * aspect);
              const float distance_y = radius / glm::tan(half_fov_y);
              const float distance_x = radius / glm::tan(half_fov_x);
              const float fit_distance = glm::max(distance_x, distance_y) * 1.15f;

              const glm::vec3 backward = sceneCameraRotation * glm::vec3(0, 0, 1);
              MoveCamera(sceneCameraRotation, center + backward * fit_distance, 0.2f);
            }
          }
#pragma endregion
        }
#pragma region Gizmos and Entity Selection
        gizmo_using_ = false;
        gizmo_displaying_ = false;
        if (enable_gizmos) {
          ImGuizmo::SetOrthographic(false);
          ImGuizmo::SetDrawlist();
          float view_manipulate_left = ImGui::GetWindowPos().x;
          float view_manipulate_top = ImGui::GetWindowPos().y;
          ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, view_port_size.x, view_port_size.y);
          glm::mat4 camera_view = glm::inverse(glm::translate(sceneCameraPosition) * glm::mat4_cast(sceneCameraRotation));
          glm::mat4 camera_projection = scene_camera->GetProjection();
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
            gizmo_displaying_ = true;
            if (ImGuizmo::IsUsing()) {
              transform.value = glm::inverse(parent_global_transform.value) * global_transform.value;
              scene->SetDataComponent(selected_entity_, transform);
              transform.Decompose(previously_stored_position_, previously_stored_rotation_, previously_stored_scale_);
              previously_stored_rotation_ = glm::degrees(previously_stored_rotation_);
              gizmo_using_ = true;
            }
          }
          if (enable_view_gizmos) {
            ImGuizmo::ViewManipulate(glm::value_ptr(camera_view), 1.0f, ImVec2(view_manipulate_left, view_manipulate_top),
                                     ImVec2(96, 96), 0);
            GlobalTransform gl;
            gl.value = glm::inverse(camera_view);
            sceneCameraRotation = gl.GetRotation();
          }
        }
#pragma endregion
      }  // end BeginChild
      scene_camera->SetRequireRendering(
          !(ImGui::GetCurrentWindowRead()->Hidden && !ImGui::GetCurrentWindowRead()->Collapsed));
      ImGui::PopStyleVar();
    } else {
      ImGui::Text("No Scene!");
    }
  }
  ImGui::End();

#pragma endregion
}

bool EditorLayer::IsGizmosDisplaying() const {
  return gizmo_displaying_;
}

bool EditorLayer::IsGizmosUsing() const {
  return gizmo_using_;
}
void EditorLayer::MainCameraWindow() {
  if (const auto render_layer = Application::GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto& graphics = Platform::GetInstance();
  const auto scene = GetScene();
#pragma region Window
  main_camera_window_focused_ = false;
  main_camera_window_hovered_ = false;
  if (ImGui::Begin("Camera")) {
    if (scene) {
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
      static int corner = 1;
      // Using a Child allow to fill all the space of the window.
      // It also allows customization
      if (ImGui::BeginChild("MainCameraRenderer", ImVec2(0, 0), false)) {
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
          main_camera_window_focused_ = true;
        }
        if (ImGui::IsWindowHovered()) {
          main_camera_window_hovered_ = true;
        }
        const bool main_activate_click = main_camera_window_hovered_ &&
                                         (Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press ||
                                          Input::GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Press ||
                                          Input::GetKey(GLFW_MOUSE_BUTTON_MIDDLE) == Input::KeyActionType::Press);
        if (main_activate_click) {
          ImGui::SetWindowFocus();
          main_camera_window_focused_ = true;
        }
        {
          const auto mp = ImGui::GetMousePos();
          const auto wp = ImGui::GetWindowPos();
          mouse_camera_window_position_ = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        }
        const ImVec2 view_port_size = ImGui::GetWindowSize();
        main_camera_resolution_x = view_port_size.x * main_camera_resolution_multiplier_;
        main_camera_resolution_y = view_port_size.y * main_camera_resolution_multiplier_;
        //  Get the size of the child (i.e. the whole draw size of the windows).
        const ImVec2 overlay_pos = ImGui::GetWindowPos();
        // Because I use the texture from OpenGL, I need to invert the V from the UV.
        const auto main_camera = scene->main_camera.Get<Camera>();
        if (main_camera && main_camera->rendered_) {
          ImGui::Image(main_camera->GetRenderTexture()->GetColorImTextureId(),
                       ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));
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
            if (graphics.prim_count[current_frame_index] < 999)
              draw_call_info += std::to_string(graphics.prim_count[current_frame_index]);
            else if (graphics.prim_count[current_frame_index] < 999999)
              draw_call_info += std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000)) + "K";
            else
              draw_call_info +=
                  std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000000)) + "M";
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
            uint32_t mode = static_cast<uint32_t>(main_camera->camera_render_mode);
            if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
              main_camera->camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
              main_camera->frame_count_ = 0;
            }
          }
          ImGui::EndChild();
        }

        if ((main_camera_window_focused_ || main_camera_window_hovered_) && !lock_entity_selection_ &&
            Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
          SetSelectedEntity(Entity());
        }
        if (!Application::IsPlaying() && (main_camera_window_focused_ || main_camera_window_hovered_) && !lock_entity_selection_ &&
            Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press &&
            !(mouse_camera_window_position_.x < 0 || mouse_camera_window_position_.y < 0 ||
              mouse_camera_window_position_.x > view_port_size.x ||
              mouse_camera_window_position_.y > view_port_size.y)) {
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

      ImGui::EndChild();
      if (const auto main_camera = scene->main_camera.Get<Camera>()) {
        main_camera->SetRequireRendering(!ImGui::GetCurrentWindowRead()->Hidden &&
                                         !ImGui::GetCurrentWindowRead()->Collapsed);
      }
      ImGui::PopStyleVar();
    } else {
      ImGui::Text("No Scene!");
    }
  }
  ImGui::End();

#pragma endregion
}

void EditorLayer::OnInputEvent(const Input::InputEvent& input_event) {
  if (input_event.key_action == Input::KeyActionType::Press) {
    switch (input_event.key) {
      case GLFW_KEY_SPACE:
        switch (Application::GetApplicationStatus()) {
          case Application::ExecutionStatus::NotPlaying:
            Application::Play();
            break;
          case Application::ExecutionStatus::Playing:
            Application::Pause();
            break;
          case Application::ExecutionStatus::Pause:
            Application::Play();
            break;
          default:
            break;
        }
        break;
      case GLFW_KEY_ESCAPE:
        Application::Stop();
        break;
      case GLFW_KEY_S:
        if (Input::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
          ProjectManager::SaveProject();
        }
        break;
      default:
        break;
    }
  }
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
  // Forward to subsequent layers.
  ILayer::OnInputEvent(input_event);
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

std::shared_ptr<Texture2D> EditorLayer::FindIcon(const std::string& name) {
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    if (const auto search = editor_layer->editor_icons_.find(name); search != editor_layer->editor_icons_.end())
      return search->second;
  }
  return {};
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

bool EditorLayer::SceneCameraWindowHovered() const {
  return scene_camera_window_hovered_;
}

bool EditorLayer::MainCameraWindowHovered() const {
  return main_camera_window_hovered_;
}

void EditorLayer::RegisterEditorCamera(const std::shared_ptr<Camera>& camera) {
  if (editor_cameras_.find(camera->GetHandle()) == editor_cameras_.end()) {
    editor_cameras_[camera->GetHandle()] = {};
    editor_cameras_.at(camera->GetHandle()).camera = camera;
  }
}

glm::vec3& EditorLayer::RefEditorCameraPosition(const Handle& handle) {
  return editor_cameras_.at(handle).position;
}

glm::quat& EditorLayer::RefEditorCameraRotation(const Handle& handle) {
  return editor_cameras_.at(handle).rotation;
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

void EditorLayer::SetSceneCameraPosition(const glm::vec3& target_position) {
  editor_cameras_.at(scene_camera_handle_).position = target_position;
}

void EditorLayer::SetSceneCameraRotation(const glm::quat& target_rotation) {
  editor_cameras_.at(scene_camera_handle_).rotation = target_rotation;
}

void EditorLayer::UpdateTextureId(ImTextureID& target, const VkSampler image_sampler, const VkImageView image_view,
                                  const VkImageLayout image_layout) {
  if (!Application::GetLayer<EditorLayer>())
    return;
  // Do not remove the previous descriptor here. ImGui draw data can still reference
  // the old TextureId within the current frame, and immediate removal may cause
  // invalid descriptor binds during ImGui_ImplVulkan_RenderDrawData.
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
  if (!scene) {
    selected_entity_ = Entity();
    lock_entity_selection_ = false;
    selection_alpha_ = 0;
    return;
  }
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
  last_orbit_target_entity_ = entity;
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
      const Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (!ptr || payload_n.GetValue() != target.GetAssetHandle().GetValue()) {
        const auto asset = AssetManager::GetAssetImpl(payload_n);
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
      const auto payload_n = *static_cast<Handle*>(payload->Data);
      const auto entity = current_scene->GetEntity(payload_n);
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
    if (!status_changed && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
      selected_entity_ = entity;
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
      const auto payload_n = *static_cast<Handle*>(payload->Data);
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
  const auto entity = entity_ref.Get();
  if (entity.GetIndex() != 0) {
    DraggableEntity(entity);
  }
}
void EditorLayer::DraggableEntity(const Entity& entity) {
  if (ImGui::BeginDragDropSource()) {
    const auto scene = Application::GetActiveScene();
    const auto handle = scene->GetEntityHandle(entity);
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
    if ((scene_camera_window_focused_ || scene_camera_window_hovered_) && !lock_entity_selection_ &&
        Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
      SetSelectedEntity(Entity());
    }
    if ((scene_camera_window_focused_ || scene_camera_window_hovered_) && !lock_entity_selection_ && !gizmo_using_ &&
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
  point.y = resolution.y - mouse_position.y;
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
    float val = -1;
    switch (Platform::Constants::texture_2d) {
      case VK_FORMAT_R32G32B32A32_SFLOAT: {
        const auto* ptr = static_cast<float*>(mapped_entity_index_data_);
        val = glm::round(ptr[3]);
        break;
      }
      case VK_FORMAT_R16G16B16A16_SFLOAT: {
        const auto* ptr = static_cast<glm::detail::hdata*>(mapped_entity_index_data_);
        val = glm::round(glm::detail::toFloat32(ptr[3]));
        break;
      }
    }
    if (const int32_t instance_index = static_cast<int>(val); instance_index > 0) {
      const auto render_layer = Application::GetLayer<RenderLayer>();
      const auto scene = GetScene();
      if (const auto handle = render_layer->GetCurrentRenderInstanceStorage()->GetInstanceEntityHandle(instance_index);
          handle != 0)
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
        if (ImGui::Button("Confirm")) {
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
      inspecting_asset = ptr;
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
  editor_icons_["Scene"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["Scene"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/Scene.png");

  editor_icons_["Binary"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["Binary"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/Binary.png");

  editor_icons_["Folder"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["Folder"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/Folder.png");

  editor_icons_["Material"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["Material"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/Material.png");

  editor_icons_["Mesh"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["Mesh"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/Mesh.png");

  editor_icons_["Prefab"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["Prefab"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/Prefab.png");

  editor_icons_["Texture2D"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["Texture2D"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Assets/Texture2D.png");
  editor_icons_["PlayButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["PlayButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Navigation/PlayButton.png");

  editor_icons_["PauseButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["PauseButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                             "Editor/Navigation/PauseButton.png");

  editor_icons_["StopButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["StopButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Navigation/StopButton.png");

  editor_icons_["StepButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["StepButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Navigation/StepButton.png");

  editor_icons_["BackButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["BackButton"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Navigation/back.png");

  editor_icons_["LeftButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["LeftButton"]->LoadInternal(std::filesystem::path("./DefaultResources") / "Editor/Navigation/left.png");

  editor_icons_["RightButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["RightButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                             "Editor/Navigation/right.png");

  editor_icons_["RefreshButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["RefreshButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                               "Editor/Navigation/refresh.png");

  editor_icons_["InfoButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["InfoButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                            "Editor/Console/InfoButton.png");

  editor_icons_["ErrorButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["ErrorButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                             "Editor/Console/ErrorButton.png");

  editor_icons_["WarningButton"] = AssetManager::CreateTemporaryAsset<Texture2D>();
  editor_icons_["WarningButton"]->LoadInternal(std::filesystem::path("./DefaultResources") /
                                               "Editor/Console/WarningButton.png");
}

void EditorLayer::CameraWindowDragAndDrop() const {
  if (AssetRef asset_ref;
      UnsafeDroppableAsset(asset_ref, {"Scene", "Prefab", "Mesh", "Strands", "Cubemap", "EnvironmentalMap"})) {
    const auto scene = GetScene();
    if (const auto asset = asset_ref.Get<IAsset>(); !Application::IsPlaying() && asset->GetTypeName() == "Scene") {
      const auto new_scene = std::dynamic_pointer_cast<Scene>(asset);
      ProjectManager::SetStartScene(new_scene);
      ProjectManager::SaveProject();
      Application::Attach(new_scene);
    } else if (asset->GetTypeName() == "Prefab") {
      const auto entity = std::dynamic_pointer_cast<Prefab>(asset)->ToEntity(scene, true, true);
      scene->SetEntityName(entity, asset->GetTitle());
    } else if (asset->GetTypeName() == "Mesh") {
      const auto entity = scene->CreateEntity(asset->GetTitle());
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      mesh_renderer->mesh.Set<Mesh>(std::dynamic_pointer_cast<Mesh>(asset));
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      mesh_renderer->material.Set<Material>(material);
    } else if (asset->GetTypeName() == "Strands") {
      const auto entity = scene->CreateEntity(asset->GetTitle());
      const auto strands_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
      strands_renderer->strands.Set<Strands>(std::dynamic_pointer_cast<Strands>(asset));
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
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
  auto& [sceneCameraRotation, sceneCameraPosition, scene_camera] = editor_cameras_.at(scene_camera_handle_);
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