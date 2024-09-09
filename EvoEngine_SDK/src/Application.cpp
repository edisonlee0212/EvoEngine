#include "Application.hpp"

#include "AnimationPlayer.hpp"
#include "ClassRegistry.hpp"
#include "CpuRayTracerCamera.hpp"
#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
#include "GpuRayTracerCamera.hpp"
#include "Platform.hpp"
#include "Input.hpp"
#include "Jobs.hpp"
#include "Json.hpp"
#include "LODGroup.hpp"
#include "LightProbe.hpp"
#include "Lights.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "PlayerController.hpp"
#include "PointCloud.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "ReflectionProbe.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "Times.hpp"
#include "TransformGraph.hpp"
#include "UnknownPrivateComponent.hpp"
#include "Utilities.hpp"
#include "WayPoints.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;
DataComponentRegistration<Transform> transform_registry("Transform");
DataComponentRegistration<GlobalTransform> global_transform_registry("GlobalTransform");
DataComponentRegistration<TransformUpdateFlag> transform_update_status_registry("TransformUpdateFlag");

const auto ray_registry = DataComponentRegistration<Ray>("Ray");
const auto camera_registry = PrivateComponentRegistration<Camera>("Camera");
const auto animation_player_registry = PrivateComponentRegistration<AnimationPlayer>("AnimationPlayer");
const auto player_controller_registry = PrivateComponentRegistration<PlayerController>("PlayerController");
const auto particles_registry = PrivateComponentRegistration<Particles>("Particles");
const auto mesh_renderer_registry = PrivateComponentRegistration<MeshRenderer>("MeshRenderer");
const auto strands_renderer_registry = PrivateComponentRegistration<StrandsRenderer>("StrandsRenderer");
const auto skinned_mesh_renderer_registry = PrivateComponentRegistration<SkinnedMeshRenderer>("SkinnedMeshRenderer");
const auto animator_registry = PrivateComponentRegistration<Animator>("Animator");
const auto point_light_registry = PrivateComponentRegistration<PointLight>("PointLight");
const auto spot_light_registry = PrivateComponentRegistration<SpotLight>("SpotLight");
const auto directional_light_registry = PrivateComponentRegistration<DirectionalLight>("DirectionalLight");
const auto way_points_registry = PrivateComponentRegistration<WayPoints>("WayPoints");
const auto lod_group_registry = PrivateComponentRegistration<LodGroup>("LodGroup");
const auto unknown_registry = PrivateComponentRegistration<UnknownPrivateComponent>("UnknownPrivateComponent");

const auto pps_registry = AssetRegistration<PostProcessingStack>("PostProcessingStack", {".evepostprocessingstack"});
const auto i_asset_registry = AssetRegistration<IAsset>("IAsset", {".eveasset"});
const auto material_registry = AssetRegistration<Material>("Material", {".evematerial"});

const auto cubemap_registry = AssetRegistration<Cubemap>("Cubemap", {".evecubemap"});
const auto registry = AssetRegistration<LightProbe>("LightProbe", {".evelightprobe"});
const auto reflection_probe_registry = AssetRegistration<ReflectionProbe>("ReflectionProbe", {".evereflectionprobe"});
const auto environmental_map_registry =
    AssetRegistration<EnvironmentalMap>("EnvironmentalMap", {".eveenvironmentalmap"});
const auto shader_registry = AssetRegistration<Shader>("Shader", {".eveshader"});
const auto mesh_registry = AssetRegistration<Mesh>("Mesh", {".evemesh"});
const auto strands_registry = AssetRegistration<Strands>("Strands", {".evestrands", ".hair"});
const auto prefab_registry = AssetRegistration<Prefab>(
    "Prefab", {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d", ".OBJ", ".FBX"});
const auto texture_2d_registry =
    AssetRegistration<Texture2D>("Texture2D", {".evetexture2d", ".png", ".jpg", ".jpeg", ".tga", ".hdr", ".TGA", ".PNG", ".JPG"});
const auto scene_registry = AssetRegistration<Scene>("Scene", {".evescene"});
const auto particle_info_list_registry =
    AssetRegistration<ParticleInfoList>("ParticleInfoList", {".eveparticleinfolist"});
const auto animation_registry = AssetRegistration<Animation>("Animation", {".eveanimation"});
const auto skinned_mesh_registry = AssetRegistration<SkinnedMesh>("SkinnedMesh", {".eveskinnedmesh"});

const auto point_cloud_registry = AssetRegistration<PointCloud>("PointCloud", {".evepointcloud"});

const auto json_registry = AssetRegistration<Json>("Json", {".json"});

const auto cpu_ray_tracer_camera_registry = PrivateComponentRegistration<CpuRayTracerCamera>("CpuRayTracerCamera");
const auto gpu_ray_tracer_camera_registry = PrivateComponentRegistration<GpuRayTracerCamera>("GpuRayTracerCamera");

void Application::PreUpdateInternal() {
  auto& application = GetInstance();
  const auto now = std::chrono::system_clock::now();
  const std::chrono::duration<double> delta_time = now - Times::last_update_time_;
  Times::delta_time_ = delta_time.count();
  Times::last_update_time_ = std::chrono::system_clock::now();
  if (application.application_status_ == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (application.application_status_ == ApplicationStatus::OnDestroy)
    return;

  application.application_execution_status_ = ApplicationExecutionStatus::PreUpdate;
  Input::PreUpdate();
  Platform::PreUpdate();

  if (application.application_status_ == ApplicationStatus::NoProject)
    return;
  TransformGraph::CalculateTransformGraphs(application.active_scene_);
  for (const auto& i : application.external_pre_update_functions_)
    i();
  if (application.application_status_ == ApplicationStatus::Playing ||
      application.application_status_ == ApplicationStatus::Step) {
    application.active_scene_->Start();
  }
  for (const auto& i : application.layers_) {
    i->PreUpdate();
  }
  if (Times::steps_ == 0) {
    Times::last_fixed_update_time_ = std::chrono::system_clock::now();
    Times::steps_ = 1;
  }
  const auto last_fixed_update_time = Times::last_fixed_update_time_;
  std::chrono::duration<double> duration = std::chrono::system_clock::now() - last_fixed_update_time;
  size_t step = 1;
  while (duration.count() >= step * Times::time_step_) {
    for (const auto& i : application.external_fixed_update_functions_)
      i();
    for (const auto& i : application.layers_) {
      i->FixedUpdate();
    }
    if (application.application_status_ == ApplicationStatus::Playing ||
        application.application_status_ == ApplicationStatus::Step) {
      application.active_scene_->FixedUpdate();
    }
    duration = std::chrono::system_clock::now() - last_fixed_update_time;
    step++;
    const auto current_time = std::chrono::system_clock::now();
    const std::chrono::duration<double> fixed_delta_time = current_time - Times::last_fixed_update_time_;
    Times::fixed_delta_time_ = fixed_delta_time.count();
    Times::last_fixed_update_time_ = std::chrono::system_clock::now();
    if (step > 10) {
      EVOENGINE_WARNING("Fixed update timeout!")
    }
    break;
  }
}

void Application::UpdateInternal() {
  auto& application = GetInstance();
  if (application.application_status_ == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (application.application_status_ == ApplicationStatus::OnDestroy)
    return;
  if (application.application_status_ == ApplicationStatus::NoProject)
    return;
  application.application_execution_status_ = ApplicationExecutionStatus::Update;
  for (const auto& i : application.external_update_functions_)
    i();

  for (auto& i : application.layers_) {
    i->Update();
  }
  if (application.application_status_ == ApplicationStatus::Playing ||
      application.application_status_ == ApplicationStatus::Step) {
    application.active_scene_->Update();
  }
}

void Application::LateUpdateInternal() {
  auto& application = GetInstance();
  if (application.application_status_ == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (application.application_status_ == ApplicationStatus::OnDestroy)
    return;

  if (application.application_status_ == ApplicationStatus::NoProject) {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGuizmo::BeginFrame();
    if (const auto window_layer = GetLayer<WindowLayer>()) {
      ImGuiFileDialog::Instance()->OpenDialog("ChooseProjectKey", "Choose Project", ".eveproj", ".");
#pragma region Dock
      static bool opt_fullscreen_persistant = true;
      bool opt_fullscreen = opt_fullscreen_persistant;
      static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

      // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
      // because it would be confusing to have two docking targets within each others.
      ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
      if (opt_fullscreen) {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                        ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
      }

      // When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
      // and handle the pass-thru hole, so we ask Begin() to not render a background.
      if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
        window_flags |= ImGuiWindowFlags_NoBackground;

      // Important: note that we proceed even if Begin() returns false (aka window is collapsed).
      // This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
      // all active windows docked into it will lose their parent and become undocked.
      // We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
      // any change of dockspace/settings would lead to windows being stuck in limbo and never being visible.

      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
      static bool open_dock = true;
      ImGui::Begin("Root DockSpace", &open_dock, window_flags);
      ImGui::PopStyleVar();
      if (opt_fullscreen)
        ImGui::PopStyleVar(2);
      ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
      ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
      ImGui::End();
#pragma endregion
      ImGui::SetNextWindowDockID(dockspace_id);
      // display
      if (ImGuiFileDialog::Instance()->Display("ChooseProjectKey")) {
        // action if OK
        if (ImGuiFileDialog::Instance()->IsOk()) {
          // action
          std::filesystem::path path = ImGuiFileDialog::Instance()->GetFilePathName();
          ProjectManager::GetOrCreateProject(path);
          if (ProjectManager::GetInstance().project_folder_) {
            window_layer->ResizeWindow(application.application_info_.default_window_size.x,
                                       application.application_info_.default_window_size.y);
            application.application_status_ = ApplicationStatus::Stop;
          }
        }
        // close
        ImGuiFileDialog::Instance()->Close();
      }
    }

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      Platform::EverythingBarrier(vk_command_buffer);
      constexpr VkClearValue clear_color = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
      VkRect2D render_area;
      render_area.offset = {0, 0};
      render_area.extent = Platform::GetSwapchain()->GetImageExtent();
      Platform::TransitImageLayout(vk_command_buffer, Platform::GetSwapchain()->GetVkImage(),
                                   Platform::GetSwapchain()->GetImageFormat(), 1, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

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

      vkCmdEndRendering(vk_command_buffer);
      Platform::TransitImageLayout(vk_command_buffer, Platform::GetSwapchain()->GetVkImage(),
                                   Platform::GetSwapchain()->GetImageFormat(), 1,
                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    });

  } else {
    if (const auto editor_layer = GetLayer<EditorLayer>()) {
      for (const auto& layer : application.layers_)
        layer->OnInspect(editor_layer);
    }
    application.application_execution_status_ = ApplicationExecutionStatus::LateUpdate;
    for (const auto& i : application.external_late_update_functions_)
      i();
    if (const auto render_layer = GetLayer<RenderLayer>()) {
      render_layer->RenderAllCameras();
    }
    if (application.application_status_ == ApplicationStatus::Playing ||
        application.application_status_ == ApplicationStatus::Step) {
      application.active_scene_->LateUpdate();
    }
    for (auto i = application.layers_.rbegin(); i != application.layers_.rend(); ++i) {
      (*i)->LateUpdate();
    }
    if (const auto render_layer = GetLayer<RenderLayer>()) {
      render_layer->ClearAllCameras();
    }
    if (application.application_status_ == ApplicationStatus::Step)
      application.application_status_ = ApplicationStatus::Pause;
  }
  Platform::LateUpdate();
}

const ApplicationInfo& Application::GetApplicationInfo() {
  auto& application = GetInstance();
  return application.application_info_;
}

const ApplicationStatus& Application::GetApplicationStatus() {
  const auto& application = GetInstance();
  return application.application_status_;
}

std::shared_ptr<Scene> Application::GetActiveScene() {
  auto& application = GetInstance();
  return application.active_scene_;
}

void Application::Reset() {
  auto& application = GetInstance();
  application.application_status_ = ApplicationStatus::Stop;
  Times::steps_ = Times::frames_ = 0;
}

void Application::Initialize(const ApplicationInfo& application_create_info) {
  auto& application = GetInstance();

  if (application.application_status_ != ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application is not uninitialzed!")
    return;
  }
  application.application_info_ = application_create_info;
  const auto window_layer = GetLayer<WindowLayer>();
  const auto editor_layer = GetLayer<EditorLayer>();
  if (!application.application_info_.project_path.empty()) {
    if (application.application_info_.project_path.extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project file extension is not eveproj!")
      return;
    }
  } else if (!window_layer || !editor_layer) {
    EVOENGINE_ERROR("Project filepath must present when there's no EditorLayer or WindowLayer!")
    return;
  }
  const auto default_thread_size = std::thread::hardware_concurrency();
  Jobs::Initialize(default_thread_size - 2);
  Entities::Initialize();
  TransformGraph::Initialize();
  Platform::Initialize();
  Resources::Initialize();
  Platform::PostResourceLoadingInitialization();
  Resources::InitializeEnvironmentalMap();

  for (const auto& layer : application.layers_) {
    layer->OnCreate();
  }

  if (!application.application_info_.project_path.empty()) {
    ProjectManager::GetOrCreateProject(application.application_info_.project_path);
    if (ProjectManager::GetInstance().project_folder_) {
      if (window_layer) {
        window_layer->ResizeWindow(application.application_info_.default_window_size.x,
                                   application.application_info_.default_window_size.y);
      }
      application.application_status_ = ApplicationStatus::Stop;
    }
  } else {
    application.application_status_ = ApplicationStatus::NoProject;
    if (window_layer) {
      window_layer->ResizeWindow(800, 600);
    }
  }
}

void Application::Start() {
  Times::start_time_ = std::chrono::system_clock::now();
  Times::steps_ = Times::frames_ = 0;
  if (const auto editor_layer = GetLayer<EditorLayer>(); !editor_layer)
    Play();
}

void Application::Run() {
  while (Loop());
}

bool Application::Loop() {
  const auto& application = GetInstance();
  if (application.application_status_ != ApplicationStatus::OnDestroy) {
    PreUpdateInternal();
    UpdateInternal();
    LateUpdateInternal();
    return true;
  }
  return false;
}

void Application::End() {
  GetInstance().application_status_ = ApplicationStatus::OnDestroy;
}

void Application::Terminate() {
  const auto& application = GetInstance();
  for (auto i = application.layers_.rbegin(); i != application.layers_.rend(); ++i) {
    (*i)->OnDestroy();
  }
}

const std::vector<std::shared_ptr<ILayer>>& Application::GetLayers() {
  const auto& application = GetInstance();
  return application.layers_;
}

void Application::Attach(const std::shared_ptr<Scene>& scene) {
  auto& application = GetInstance();
  if (application.application_status_ == ApplicationStatus::Playing) {
    EVOENGINE_ERROR("Stop Application to attach scene")
  }

  application.active_scene_ = scene;
  for (auto& func : application.post_attach_scene_functions_) {
    func(scene);
  }
  for (const auto& layer : application.layers_) {
    layer->scene_ = scene;
  }
}

void Application::Play() {
  auto& application = GetInstance();
  if (application.application_status_ == ApplicationStatus::NoProject ||
      application.application_status_ == ApplicationStatus::OnDestroy)
    return;
  if (application.application_status_ != ApplicationStatus::Pause &&
      application.application_status_ != ApplicationStatus::Stop)
    return;
  if (application.application_status_ == ApplicationStatus::Stop) {
    const auto copied_scene = ProjectManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  application.application_status_ = ApplicationStatus::Playing;
}
void Application::Stop() {
  auto& application = GetInstance();
  if (application.application_status_ == ApplicationStatus::NoProject ||
      application.application_status_ == ApplicationStatus::OnDestroy)
    return;
  if (application.application_status_ == ApplicationStatus::Stop)
    return;
  application.application_status_ = ApplicationStatus::Stop;
  Attach(ProjectManager::GetStartScene().lock());
}
void Application::Pause() {
  auto& application = GetInstance();
  if (application.application_status_ == ApplicationStatus::NoProject ||
      application.application_status_ == ApplicationStatus::OnDestroy)
    return;
  if (application.application_status_ != ApplicationStatus::Playing)
    return;
  application.application_status_ = ApplicationStatus::Pause;
}

void Application::Step() {
  auto& application = GetInstance();
  if (application.application_status_ != ApplicationStatus::Pause &&
      application.application_status_ != ApplicationStatus::Stop)
    return;
  if (application.application_status_ == ApplicationStatus::Stop) {
    const auto copied_scene = ProjectManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  application.application_status_ = ApplicationStatus::Step;
}

ApplicationExecutionStatus Application::GetApplicationExecutionStatus() {
  return GetInstance().application_execution_status_;
}

void Application::RegisterPreUpdateFunction(const std::function<void()>& func) {
  GetInstance().external_pre_update_functions_.push_back(func);
}

void Application::RegisterUpdateFunction(const std::function<void()>& func) {
  GetInstance().external_update_functions_.push_back(func);
}

void Application::RegisterLateUpdateFunction(const std::function<void()>& func) {
  GetInstance().external_late_update_functions_.push_back(func);
}
void Application::RegisterFixedUpdateFunction(const std::function<void()>& func) {
  GetInstance().external_fixed_update_functions_.push_back(func);
}

void Application::RegisterPostAttachSceneFunction(
    const std::function<void(const std::shared_ptr<Scene>& new_scene)>& func) {
  GetInstance().post_attach_scene_functions_.push_back(func);
}

bool Application::IsPlaying() {
  const auto& application = GetInstance();
  return application.application_status_ == ApplicationStatus::Playing;
}
