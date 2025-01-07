#include "Application.hpp"

#include "AnimationPlayer.hpp"
#include "ClassRegistry.hpp"
#include "CpuRayTracerCamera.hpp"
#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
#include "Input.hpp"
#include "Jobs.hpp"
#include "Json.hpp"
#include "LightProbe.hpp"
#include "Lights.hpp"
#include "LodGroup.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "Platform.hpp"
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

#include "GpuRayTracerCamera.hpp"

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
const auto shader_registry = AssetRegistration<Shader>(
    "Shader", {".eveshader", ".glsl", ".vert", ".frag", ".comp", ".geom", ".task", ".mesh", ".tesc", ".tese"});
const auto mesh_registry = AssetRegistration<Mesh>("Mesh", {".evemesh"});
const auto strands_registry = AssetRegistration<Strands>("Strands", {".evestrands", ".hair"});
const auto prefab_registry = AssetRegistration<Prefab>(
    "Prefab", {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d", ".OBJ", ".FBX"});
const auto texture_2d_registry = AssetRegistration<Texture2D>(
    "Texture2D", {".evetexture2d", ".png", ".jpg", ".jpeg", ".tga", ".hdr", ".TGA", ".PNG", ".JPG"});
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
  if (const auto render_layer = GetLayer<RenderLayer>()) {
    Platform::PreUpdate();
  }
  if (const auto editor_layer = GetLayer<EditorLayer>()) {
    EditorLayer::InitializeImGui();
  }
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
  if (application.application_status_ == ApplicationStatus::NoProject) {
    if (const auto window_layer = GetLayer<WindowLayer>()) {
      if (ProjectManager::StartupGui()) {
        window_layer->ResizeWindow(application.application_info_.default_window_size.x,
                                   application.application_info_.default_window_size.y);
        application.application_status_ = ApplicationStatus::NotPlaying;
      }
    }
    return;
  }
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
  const auto render_layer = GetLayer<RenderLayer>();
  if (const auto editor_layer = GetLayer<EditorLayer>()) {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("View")) {
        if (ImGui::BeginMenu("Layer Inspection")) {
          for (const auto& layer : application.layers_) {
            ImGui::Checkbox(layer->layer_name_.c_str(), &layer->enable_inspection);
          }
          ImGui::EndMenu();
        }
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    EditorLayer::OnGui(editor_layer);
    for (const auto& layer : application.layers_) {
      if (layer->enable_inspection) {
        ImGui::Begin(layer->layer_name_.c_str());
        layer->OnInspect(editor_layer);
        ImGui::End();
      }
    }
  }
  if (render_layer) {
    render_layer->PrepareForRendering();
    render_layer->ClearAllEditorCameras();
    render_layer->ClearAllCameras();
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
  const auto render_layer = GetLayer<RenderLayer>();
  const auto editor_layer = GetLayer<EditorLayer>();
  const auto window_layer = GetLayer<WindowLayer>();
  if (application.application_status_ != ApplicationStatus::NoProject) {
    application.application_execution_status_ = ApplicationExecutionStatus::LateUpdate;
    for (const auto& i : application.external_late_update_functions_)
      i();

    if (application.application_status_ == ApplicationStatus::Playing ||
        application.application_status_ == ApplicationStatus::Step) {
      application.active_scene_->LateUpdate();
    }
    for (auto i = application.layers_.rbegin(); i != application.layers_.rend(); ++i) {
      (*i)->LateUpdate();
    }
    if (render_layer) {
      render_layer->RenderAll();
      render_layer->RenderGizmos();
    }
  }

  if (window_layer) {
    window_layer->Render();
  }
  if (render_layer) {
    
    Platform::LateUpdate();
  }
  if (application.application_status_ == ApplicationStatus::Step)
    application.application_status_ = ApplicationStatus::Pause;
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
  application.application_status_ = ApplicationStatus::NotPlaying;
  Times::steps_ = Times::frames_ = 0;
}

void Application::Initialize(const ApplicationInfo& application_create_info) {
  auto& application = GetInstance();

  if (application.application_status_ != ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application is not uninitialzed!")
    return;
  }
  application.application_info_ = application_create_info;
  const auto render_layer = GetLayer<RenderLayer>();
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
  ProjectManager::Initialize();
  if (render_layer) {
    Platform::Initialize();
  }
  Resources::Initialize();
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
      application.application_status_ = ApplicationStatus::NotPlaying;
    }
  } else {
    application.application_status_ = ApplicationStatus::NoProject;
    if (window_layer) {
      window_layer->ResizeWindow(800, 600);
    }
  }
}

void Application::Start(bool autoplay) {
  Times::start_time_ = std::chrono::system_clock::now();
  Times::steps_ = Times::frames_ = 0;
  if (const auto editor_layer = GetLayer<EditorLayer>(); !editor_layer && autoplay)
    Play();
}

void Application::Run() {
  while (Loop()) {
  }
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
  auto& application = GetInstance();
  const bool has_render_layer = GetLayer<RenderLayer>() != nullptr;
  for (auto i = application.layers_.rbegin(); i != application.layers_.rend(); ++i) {
    (*i)->OnDestroy();
  }
  application.layers_.clear();
  Jobs::OnDestroy();
  ProjectManager::OnDestroy();
  Resources::OnDestroy();
  application.active_scene_.reset();
  TextureStorage::OnDestroy();
  GeometryStorage::OnDestroy();
  if (has_render_layer) {
    Platform::OnDestroy();
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
      application.application_status_ != ApplicationStatus::NotPlaying)
    return;
  if (application.application_status_ == ApplicationStatus::NotPlaying) {
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
  if (application.application_status_ == ApplicationStatus::NotPlaying)
    return;
  application.application_status_ = ApplicationStatus::NotPlaying;
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
      application.application_status_ != ApplicationStatus::NotPlaying)
    return;
  if (application.application_status_ == ApplicationStatus::NotPlaying) {
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
