#include "Application.hpp"

#include "AnimationPlayer.hpp"
#include "ClassRegistry.hpp"
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
#include "PointCloudScanner.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProceduralNoise.hpp"
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

void Application::PreUpdateInternal() {
  auto& application = GetInstance();
  const auto now = std::chrono::system_clock::now();
  const std::chrono::duration<double> delta_time = now - Times::last_update_time_;
  Times::delta_time_ = delta_time.count();
  Times::last_update_time_ = std::chrono::system_clock::now();
  if (application.execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (application.execution_status_ == ExecutionStatus::OnDestroy)
    return;

  application.execution_order = ExecutionOrder::PreUpdate;
  Input::PreUpdate();
  if (const auto render_layer = GetLayer<RenderLayer>()) {
    Platform::PreUpdate();
  }
  ProjectManager::PreUpdate();
  if (application.active_scene_) {
    TransformGraph::CalculateTransformGraphs(application.active_scene_);
    for (const auto& i : application.external_pre_update_functions_)
      i();
    if (application.execution_status_ == ExecutionStatus::Playing ||
        application.execution_status_ == ExecutionStatus::Step) {
      application.active_scene_->Start();
    }
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
    if (application.execution_status_ == ExecutionStatus::Playing ||
        application.execution_status_ == ExecutionStatus::Step) {
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
  if (application.execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (application.execution_status_ == ExecutionStatus::OnDestroy)
    return;

  application.execution_order = ExecutionOrder::Update;
  if (application.active_scene_) {
    if (application.execution_status_ == ExecutionStatus::Playing ||
        application.execution_status_ == ExecutionStatus::Step) {
      application.active_scene_->Update();
    }
  }

  for (const auto& i : application.layers_) {
    i->Update();
  }
  for (const auto& i : application.external_update_functions_)
    i();

  if (const auto render_layer = GetLayer<RenderLayer>()) {
    render_layer->PrepareForRendering();
    render_layer->ClearAllEditorCameras();
    render_layer->ClearAllCameras();
  }
}

void Application::LateUpdateInternal() {
  auto& application = GetInstance();
  if (application.execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (application.execution_status_ == ExecutionStatus::OnDestroy)
    return;
  for (const auto& i : application.external_late_update_functions_)
    i();
  for (auto i = application.layers_.rbegin(); i != application.layers_.rend(); ++i) {
    (*i)->LateUpdate();
  }

  const auto render_layer = GetLayer<RenderLayer>();
  const auto editor_layer = GetLayer<EditorLayer>();
  const auto window_layer = GetLayer<WindowLayer>();

  if (application.active_scene_) {
    application.execution_order = ExecutionOrder::LateUpdate;

    if (application.execution_status_ == ExecutionStatus::Playing ||
        application.execution_status_ == ExecutionStatus::Step) {
      application.active_scene_->LateUpdate();
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
  if (application.execution_status_ == ExecutionStatus::Step)
    application.execution_status_ = ExecutionStatus::Pause;
}

const ApplicationInitializationSettings& Application::GetApplicationInfo() {
  auto& application = GetInstance();
  return application.initialization_settings;
}

const Application::ExecutionStatus& Application::GetApplicationStatus() {
  const auto& application = GetInstance();
  return application.execution_status_;
}

std::shared_ptr<Scene> Application::GetActiveScene() {
  auto& application = GetInstance();
  return application.active_scene_;
}

void Application::Reset() {
  auto& application = GetInstance();
  application.execution_status_ = ExecutionStatus::NotPlaying;
  Times::steps_ = Times::frames_ = 0;
}

void Application::Initialize(const ApplicationInitializationSettings& application_create_info) {
#pragma region Reflection
  DataComponentRegistration<Transform> transform_registry("Transform");
  DataComponentRegistration<GlobalTransform> global_transform_registry("GlobalTransform");
  DataComponentRegistration<TransformUpdateFlag> transform_update_status_registry("TransformUpdateFlag");

  DataComponentRegistration<Ray> ray_registry("Ray");
  PrivateComponentRegistration<Camera> camera_registry("Camera");
  PrivateComponentRegistration<AnimationPlayer> animation_player_registry("AnimationPlayer");
  PrivateComponentRegistration<PlayerController> player_controller_registry("PlayerController");
  PrivateComponentRegistration<Particles> particles_registry("Particles");
  PrivateComponentRegistration<MeshRenderer> mesh_renderer_registry("MeshRenderer");
  PrivateComponentRegistration<StrandsRenderer> strands_renderer_registry("StrandsRenderer");
  PrivateComponentRegistration<SkinnedMeshRenderer> skinned_mesh_renderer_registry("SkinnedMeshRenderer");
  PrivateComponentRegistration<Animator> animator_registry("Animator");
  PrivateComponentRegistration<PointLight> point_light_registry("PointLight");
  PrivateComponentRegistration<SpotLight> spot_light_registry("SpotLight");
  PrivateComponentRegistration<DirectionalLight> directional_light_registry("DirectionalLight");
  PrivateComponentRegistration<WayPoints> way_points_registry("WayPoints");
  PrivateComponentRegistration<LodGroup> lod_group_registry("LodGroup");
  PrivateComponentRegistration<PointCloudScanner> point_cloud_scanner_registry("PointCloudScanner");
  PrivateComponentRegistration<UnknownPrivateComponent> unknown_registry("UnknownPrivateComponent");

  AssetRegistration<PostProcessingStack> pps_registry("PostProcessingStack", {".evepostprocessingstack"});
  AssetRegistration<IAsset> i_asset_registry("IAsset", {".eveasset"});
  AssetRegistration<Material> material_registry("Material", {".evematerial"});
  AssetRegistration<procedural_noise::ProceduralNoise2D> procedural_noise_2d_registry("ProceduralNoise2D",
                                                                                      {".evenoise2d"});
  AssetRegistration<procedural_noise::ProceduralNoise3D> procedural_noise_3d_registry("ProceduralNoise3D",
                                                                                      {".evenoise3d"});
  AssetRegistration<procedural_noise::ProceduralNoise4D> procedural_noise_4d_registry("ProceduralNoise4D",
                                                                                      {".evenoise4d"});

  AssetRegistration<Cubemap> cubemap_registry("Cubemap", {".evecubemap"});
  AssetRegistration<LightProbe> light_probe_registry("LightProbe", {".evelightprobe"});
  AssetRegistration<ReflectionProbe> reflection_probe_registry("ReflectionProbe", {".evereflectionprobe"});
  AssetRegistration<EnvironmentalMap> environmental_map_registry("EnvironmentalMap", {".eveenvironmentalmap"});
  AssetRegistration<Shader> shader_registry(
      "Shader", {".eveshader", ".glsl", ".vert", ".frag", ".comp", ".geom", ".task", ".mesh", ".tesc", ".tese"});
  AssetRegistration<Mesh> mesh_registry("Mesh", {".evemesh"});
  AssetRegistration<Strands> strands_registry("Strands", {".evestrands", ".hair"});
  AssetRegistration<Prefab> prefab_registry(
      "Prefab", {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d", ".OBJ", ".FBX"});
  AssetRegistration<Texture2D> texture_2d_registry(
      "Texture2D", {".evetexture2d", ".png", ".jpg", ".jpeg", ".tga", ".hdr", ".TGA", ".PNG", ".JPG"});
  AssetRegistration<Scene> scene_registry("Scene", {".evescene"});
  AssetRegistration<ParticleInfoList> particle_info_list_registry("ParticleInfoList", {".eveparticleinfolist"});
  AssetRegistration<Animation> animation_registry("Animation", {".eveanimation"});
  AssetRegistration<SkinnedMesh> skinned_mesh_registry("SkinnedMesh", {".eveskinnedmesh"});

  AssetRegistration<PointCloud> point_cloud_registry("PointCloud", {".evepointcloud"});

  AssetRegistration<Json> json_registry("Json", {".json"});
#pragma endregion

  auto& application = GetInstance();

  if (application.execution_status_ != ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application is not uninitialzed!")
    return;
  }
  application.initialization_settings = application_create_info;
  const auto render_layer = GetLayer<RenderLayer>();
  const auto window_layer = GetLayer<WindowLayer>();
  const auto editor_layer = GetLayer<EditorLayer>();
  if (!application.initialization_settings.project_path.empty()) {
    if (application.initialization_settings.project_path.extension().string() != ".eveproj") {
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
  AssetManager::Initialize();
  FileManager::Initialize();
  ProjectManager::Initialize();
  if (render_layer) {
    Platform::Initialize(application.initialization_settings);
  }
  Resources::Initialize();
  for (const auto& layer : application.layers_) {
    layer->OnCreate();
  }
  if (window_layer) {
    window_layer->ResizeWindow(application.initialization_settings.default_window_size.x,
                               application.initialization_settings.default_window_size.y);
    if (application.initialization_settings.icon_paths.empty()) {
      GLFWimage images[4];
      images[0].pixels =
          stbi_load(std::filesystem::absolute("./DefaultResources/Icons/EvoEngine16.png").string().c_str(),
                    &images[0].width, &images[0].height, nullptr, 4);  // rgba channels
      images[1].pixels =
          stbi_load(std::filesystem::absolute("./DefaultResources/Icons/EvoEngine24.png").string().c_str(),
                    &images[1].width, &images[1].height, nullptr, 4);  // rgba channels
      images[2].pixels =
          stbi_load(std::filesystem::absolute("./DefaultResources/Icons/EvoEngine32.png").string().c_str(),
                    &images[2].width, &images[2].height, nullptr, 4);  // rgba channels
      images[3].pixels =
          stbi_load(std::filesystem::absolute("./DefaultResources/Icons/EvoEngine64.png").string().c_str(),
                    &images[3].width, &images[3].height, nullptr, 4);  // rgba channels
      glfwSetWindowIcon(window_layer->window_, 4, images);
      stbi_image_free(images[0].pixels);
      stbi_image_free(images[1].pixels);
      stbi_image_free(images[2].pixels);
      stbi_image_free(images[3].pixels);
    } else {
      std::vector<GLFWimage> images;
      for (const auto& i : application.initialization_settings.icon_paths) {
        if (std::filesystem::exists(i)) {
          auto& image = images.emplace_back();
          image.pixels = stbi_load(std::filesystem::absolute(i).string().c_str(), &image.width, &image.height, nullptr,
                                   4);  // rgba channels
        }
      }
      glfwSetWindowIcon(window_layer->window_, images.size(), images.data());
      for (const auto& i : images) {
        stbi_image_free(i.pixels);
      }
    }
  }
  application.execution_status_ = ExecutionStatus::NotPlaying;

  if (!application.initialization_settings.project_path.empty()) {
    ProjectManager::GetOrCreateProject(application.initialization_settings.project_path);
  }
}

void Application::Start(const bool autoplay) {
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
  if (application.execution_status_ != ExecutionStatus::OnDestroy) {
    PreUpdateInternal();
    UpdateInternal();
    LateUpdateInternal();
    return true;
  }
  return false;
}

void Application::End() {
  auto& application = GetInstance();
  application.execution_status_ = ExecutionStatus::OnDestroy;
}

void Application::Terminate() {
  auto& application = GetInstance();
  const bool has_render_layer = GetLayer<RenderLayer>() != nullptr;
  ProjectManager::SaveProject();
  for (auto i = application.layers_.rbegin(); i != application.layers_.rend(); ++i) {
    (*i)->OnDestroy();
  }
  application.layers_.clear();
  Jobs::OnDestroy();
  ProjectManager::OnDestroy();
  FileManager::OnDestroy();
  Resources::OnDestroy();
  application.active_scene_.reset();
  TextureStorage::OnDestroy();
  GeometryStorage::OnDestroy();

  AssetManager::OnDestroy();
  if (has_render_layer) {
    Platform::OnDestroy();
  }

  Serialization::OnDestroy();

  application.execution_status_ = ExecutionStatus::Uninitialized;
}

const std::vector<std::shared_ptr<ILayer>>& Application::GetLayers() {
  const auto& application = GetInstance();
  return application.layers_;
}

void Application::Attach(const std::shared_ptr<Scene>& scene) {
  auto& application = GetInstance();
  if (application.execution_status_ == ExecutionStatus::Playing) {
    EVOENGINE_ERROR("Stop Application to attach scene")
  }

  application.active_scene_ = scene;
  for (const auto& layer : application.layers_) {
    layer->scene_ = scene;
  }
  for (auto& func : application.post_attach_scene_functions_) {
    func(scene);
  }
}

void Application::Play() {
  auto& application = GetInstance();
  if (!application.active_scene_ || application.execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (application.execution_status_ != ExecutionStatus::Pause &&
      application.execution_status_ != ExecutionStatus::NotPlaying)
    return;
  if (application.execution_status_ == ExecutionStatus::NotPlaying) {
    const auto copied_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  application.execution_status_ = ExecutionStatus::Playing;
}
void Application::Stop() {
  auto& application = GetInstance();
  if (!application.active_scene_ || application.execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (application.execution_status_ == ExecutionStatus::NotPlaying)
    return;
  application.execution_status_ = ExecutionStatus::NotPlaying;
  Attach(ProjectManager::GetStartScene().lock());
}
void Application::Pause() {
  auto& application = GetInstance();
  if (!application.active_scene_ || application.execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (application.execution_status_ != ExecutionStatus::Playing)
    return;
  application.execution_status_ = ExecutionStatus::Pause;
}

void Application::Step() {
  auto& application = GetInstance();
  if (application.execution_status_ != ExecutionStatus::Pause &&
      application.execution_status_ != ExecutionStatus::NotPlaying)
    return;
  if (application.execution_status_ == ExecutionStatus::NotPlaying) {
    const auto copied_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  application.execution_status_ = ExecutionStatus::Step;
}

Application::ExecutionOrder Application::GetApplicationExecutionStatus() {
  const auto& application = GetInstance();
  return application.execution_order;
}

void Application::RegisterPreUpdateFunction(const std::function<void()>& func) {
  auto& application = GetInstance();
  application.external_pre_update_functions_.push_back(func);
}

void Application::RegisterUpdateFunction(const std::function<void()>& func) {
  auto& application = GetInstance();
  application.external_update_functions_.push_back(func);
}

void Application::RegisterLateUpdateFunction(const std::function<void()>& func) {
  auto& application = GetInstance();
  application.external_late_update_functions_.push_back(func);
}
void Application::RegisterFixedUpdateFunction(const std::function<void()>& func) {
  auto& application = GetInstance();
  application.external_fixed_update_functions_.push_back(func);
}

void Application::RegisterPostAttachSceneFunction(
    const std::function<void(const std::shared_ptr<Scene>& new_scene)>& func) {
  auto& application = GetInstance();
  application.post_attach_scene_functions_.push_back(func);
}

bool Application::IsPlaying() {
  const auto& application = GetInstance();
  return application.execution_status_ == ExecutionStatus::Playing;
}
