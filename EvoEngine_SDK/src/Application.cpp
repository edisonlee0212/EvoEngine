#include "Application.hpp"

#include "ApplicationContext.hpp"

#include "AnimationPlayer.hpp"
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
#include "PackageManager.hpp"
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

Application::Application()
    : asset_manager_(std::make_unique<AssetManager>()),
      console_(std::make_unique<Console>()),
      entities_(std::make_unique<Entities>()),
      file_manager_(std::make_unique<FileManager>()),
      geometry_storage_(std::make_unique<GeometryStorage>()),
      input_(std::make_unique<Input>()),
      jobs_(std::make_unique<Jobs>()),
      package_manager_(std::make_unique<PackageManager>()),
      platform_(std::make_unique<Platform>()),
      project_manager_(std::make_unique<ProjectManager>()),
      resources_(std::make_unique<Resources>()),
      texture_storage_(std::make_unique<TextureStorage>()),
      times_(std::make_unique<Times>()),
      transform_graph_(std::make_unique<TransformGraph>()) {
  ApplicationContext::Set(this);
}

Application::~Application() {
  if (execution_status_ != ExecutionStatus::Uninitialized) {
    Terminate();
  }
  if (ApplicationContext::TryGet() == this) {
    ApplicationContext::Set(nullptr);
  }
}

Serialization& Application::GetSerialization() {
  return serialization_registry_;
}

const Serialization& Application::GetSerialization() const {
  return serialization_registry_;
}

AssetManager& Application::GetAssetManager() {
  return *asset_manager_;
}

Console& Application::GetConsole() {
  return *console_;
}

Entities& Application::GetEntities() {
  return *entities_;
}

FileManager& Application::GetFileManager() {
  return *file_manager_;
}

GeometryStorage& Application::GetGeometryStorage() {
  return *geometry_storage_;
}

Input& Application::GetInput() {
  return *input_;
}

Jobs& Application::GetJobs() {
  return *jobs_;
}

PackageManager& Application::GetPackageManager() {
  return *package_manager_;
}

Platform& Application::GetPlatform() {
  return *platform_;
}

ProjectManager& Application::GetProjectManager() {
  return *project_manager_;
}

Resources& Application::GetResources() {
  return *resources_;
}

TextureStorage& Application::GetTextureStorage() {
  return *texture_storage_;
}

Times& Application::GetTimes() {
  return *times_;
}

TransformGraph& Application::GetTransformGraph() {
  return *transform_graph_;
}

void Application::PreUpdateInternal() {
  ApplicationContextScope application_scope(*this);
  const auto now = std::chrono::system_clock::now();
  const std::chrono::duration<double> delta_time = now - Times::last_update_time_;
  Times::delta_time_ = delta_time.count();
  Times::last_update_time_ = std::chrono::system_clock::now();
  if (this->execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (this->execution_status_ == ExecutionStatus::OnDestroy)
    return;

  this->execution_order = ExecutionOrder::PreUpdate;
  Input::PreUpdate();
  if (const auto render_layer = GetLayer<RenderLayer>()) {
    Platform::PreUpdate();
  }
  ProjectManager::PreUpdate();
  if (this->active_scene_) {
    TransformGraph::CalculateTransformGraphs(this->active_scene_);
    for (const auto& i : this->external_pre_update_functions_)
      i();
    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      this->active_scene_->Start();
    }
  }

  for (const auto& i : this->layers_) {
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
    for (const auto& i : this->external_fixed_update_functions_)
      i();
    for (const auto& i : this->layers_) {
      i->FixedUpdate();
    }
    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      this->active_scene_->FixedUpdate();
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
  ApplicationContextScope application_scope(*this);
  if (this->execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (this->execution_status_ == ExecutionStatus::OnDestroy)
    return;

  this->execution_order = ExecutionOrder::Update;
  if (this->active_scene_) {
    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      this->active_scene_->Update();
    }
  }

  for (const auto& i : this->layers_) {
    i->Update();
  }
  for (const auto& i : this->external_update_functions_)
    i();

  if (const auto render_layer = GetLayer<RenderLayer>()) {
    render_layer->PrepareForRendering();
    render_layer->ClearAllEditorCameras();
    render_layer->ClearAllCameras();
  }
}

void Application::LateUpdateInternal() {
  ApplicationContextScope application_scope(*this);
  if (this->execution_status_ == ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application uninitialized!")
    return;
  }
  if (this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  for (const auto& i : this->external_late_update_functions_)
    i();
  for (auto i = this->layers_.rbegin(); i != this->layers_.rend(); ++i) {
    (*i)->LateUpdate();
  }

  const auto render_layer = GetLayer<RenderLayer>();
  const auto editor_layer = GetLayer<EditorLayer>();
  const auto window_layer = GetLayer<WindowLayer>();

  if (this->active_scene_) {
    this->execution_order = ExecutionOrder::LateUpdate;

    if (this->execution_status_ == ExecutionStatus::Playing || this->execution_status_ == ExecutionStatus::Step) {
      this->active_scene_->LateUpdate();
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
  if (this->execution_status_ == ExecutionStatus::Step)
    this->execution_status_ = ExecutionStatus::Pause;
}

const ApplicationInitializationSettings& Application::GetApplicationInfo() const {
  return this->initialization_settings;
}

const Application::ExecutionStatus& Application::GetApplicationStatus() const {
  return this->execution_status_;
}

std::shared_ptr<Scene> Application::GetActiveScene() const {
  return this->active_scene_;
}

void Application::Reset() {
  ApplicationContextScope application_scope(*this);
  this->execution_status_ = ExecutionStatus::NotPlaying;
  Times::steps_ = Times::frames_ = 0;
}

void Application::Initialize(const ApplicationInitializationSettings& application_create_info) {
  ApplicationContextScope application_scope(*this);
#pragma region Reflection
  RegisterDataComponent<Transform>("Transform");
  RegisterDataComponent<GlobalTransform>("GlobalTransform");
  RegisterDataComponent<TransformUpdateFlag>("TransformUpdateFlag");
  RegisterDataComponent<Ray>("Ray");

  RegisterPrivateComponent<Camera>("Camera");
  RegisterPrivateComponent<AnimationPlayer>("AnimationPlayer");
  RegisterPrivateComponent<PlayerController>("PlayerController");
  RegisterPrivateComponent<Particles>("Particles");
  RegisterPrivateComponent<MeshRenderer>("MeshRenderer");
  RegisterPrivateComponent<StrandsRenderer>("StrandsRenderer");
  RegisterPrivateComponent<SkinnedMeshRenderer>("SkinnedMeshRenderer");
  RegisterPrivateComponent<Animator>("Animator");
  RegisterPrivateComponent<PointLight>("PointLight");
  RegisterPrivateComponent<SpotLight>("SpotLight");
  RegisterPrivateComponent<DirectionalLight>("DirectionalLight");
  RegisterPrivateComponent<WayPoints>("WayPoints");
  RegisterPrivateComponent<LodGroup>("LodGroup");
  RegisterPrivateComponent<PointCloudScanner>("PointCloudScanner");
  RegisterPrivateComponent<UnknownPrivateComponent>("UnknownPrivateComponent");

  RegisterAsset<PostProcessingStack>("PostProcessingStack", {".evepostprocessingstack"});
  RegisterAsset<IAsset>("IAsset", {".eveasset"});
  RegisterAsset<Material>("Material", {".evematerial"});
  RegisterAsset<procedural_noise::ProceduralNoise2D>("ProceduralNoise2D", {".evenoise2d"});
  RegisterAsset<procedural_noise::ProceduralNoise3D>("ProceduralNoise3D", {".evenoise3d"});
  RegisterAsset<procedural_noise::ProceduralNoise4D>("ProceduralNoise4D", {".evenoise4d"});
  RegisterAsset<Cubemap>("Cubemap", {".evecubemap"});
  RegisterAsset<LightProbe>("LightProbe", {".evelightprobe"});
  RegisterAsset<ReflectionProbe>("ReflectionProbe", {".evereflectionprobe"});
  RegisterAsset<EnvironmentalMap>("EnvironmentalMap", {".eveenvironmentalmap"});
  RegisterAsset<Shader>(
      "Shader", {".eveshader", ".glsl", ".vert", ".frag", ".comp", ".geom", ".task", ".mesh", ".tesc", ".tese"});
  RegisterAsset<Mesh>("Mesh", {".evemesh"});
  RegisterAsset<Strands>("Strands", {".evestrands", ".hair"});
  RegisterAsset<Prefab>(
      "Prefab", {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d", ".OBJ", ".FBX"});
  RegisterAsset<Texture2D>("Texture2D",
                           {".evetexture2d", ".png", ".jpg", ".jpeg", ".tga", ".hdr", ".TGA", ".PNG", ".JPG"});
  RegisterAsset<Scene>("Scene", {".evescene"});
  RegisterAsset<ParticleInfoList>("ParticleInfoList", {".eveparticleinfolist"});
  RegisterAsset<Animation>("Animation", {".eveanimation"});
  RegisterAsset<SkinnedMesh>("SkinnedMesh", {".eveskinnedmesh"});
  RegisterAsset<PointCloud>("PointCloud", {".evepointcloud"});
  RegisterAsset<Json>("Json", {".json"});
#pragma endregion

  if (this->execution_status_ != ExecutionStatus::Uninitialized) {
    EVOENGINE_ERROR("Application is not uninitialzed!")
    return;
  }
  this->initialization_settings = application_create_info;
  const auto render_layer = GetLayer<RenderLayer>();
  const auto window_layer = GetLayer<WindowLayer>();
  const auto editor_layer = GetLayer<EditorLayer>();
  if (!this->initialization_settings.project_path.empty()) {
    if (this->initialization_settings.project_path.extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project file extension is not eveproj!")
      return;
    }
  } else if (!window_layer || !editor_layer) {
    EVOENGINE_ERROR("Project filepath must present when there's no EditorLayer or WindowLayer!")
    return;
  }
  const auto default_thread_size = std::thread::hardware_concurrency();
  for (const auto& layer : this->layers_) {
    layer->RegisterTypes(*this);
  }
  Jobs::Initialize(default_thread_size - 2);
  Entities::Initialize();
  TransformGraph::Initialize();
  AssetManager::Initialize();
  FileManager::Initialize();
  ProjectManager::Initialize();
  if (render_layer) {
    Platform::Initialize(this->initialization_settings);
  }
  Resources::Initialize();
  if (this->initialization_settings.enable_runtime_packages) {
    PackageManager::Initialize(this->initialization_settings.package_search_paths);
  }
  for (const auto& layer : this->layers_) {
    layer->OnCreate();
  }
  if (window_layer) {
    window_layer->ResizeWindow(this->initialization_settings.default_window_size.x,
                               this->initialization_settings.default_window_size.y);
    if (this->initialization_settings.icon_paths.empty()) {
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
      for (const auto& i : this->initialization_settings.icon_paths) {
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
  this->execution_status_ = ExecutionStatus::NotPlaying;

  if (!this->initialization_settings.project_path.empty()) {
    ProjectManager::GetOrCreateProject(this->initialization_settings.project_path);
  }
}

void Application::Start(const bool autoplay) {
  ApplicationContextScope application_scope(*this);
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
  const ApplicationContextScope application_scope(*this);
  if (this->execution_status_ != ExecutionStatus::OnDestroy) {
    PreUpdateInternal();
    UpdateInternal();
    LateUpdateInternal();
    return true;
  }
  return false;
}

void Application::End() {
  ApplicationContextScope application_scope(*this);
  this->execution_status_ = ExecutionStatus::OnDestroy;
}

void Application::Terminate() {
  ApplicationContextScope application_scope(*this);
  const bool has_render_layer = GetLayer<RenderLayer>() != nullptr;
  for (auto i = this->layers_.rbegin(); i != this->layers_.rend(); ++i) {
    (*i)->OnDestroy();
  }
  this->layers_.clear();
  Jobs::OnDestroy();
  ProjectManager::OnDestroy();
  FileManager::OnDestroy();
  Resources::OnDestroy();
  this->active_scene_.reset();
  TextureStorage::OnDestroy();
  GeometryStorage::OnDestroy();

  AssetManager::OnDestroy();
  if (has_render_layer) {
    Platform::OnDestroy();
  }

  PackageManager::UnloadAll();
  Serialization::OnDestroy();

  this->execution_status_ = ExecutionStatus::Uninitialized;
}

const std::vector<std::shared_ptr<ILayer>>& Application::GetLayers() const {
  return this->layers_;
}

void Application::Attach(const std::shared_ptr<Scene>& scene) {
  ApplicationContextScope application_scope(*this);
  if (this->execution_status_ == ExecutionStatus::Playing) {
    EVOENGINE_ERROR("Stop Application to attach scene")
  }

  this->active_scene_ = scene;
  for (auto& func : this->post_attach_scene_functions_) {
    func(scene);
  }
  for (const auto& layer : this->layers_) {
    layer->scene_ = scene;
  }
}

void Application::Play() {
  ApplicationContextScope application_scope(*this);
  if (!this->active_scene_ || this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (this->execution_status_ != ExecutionStatus::Pause && this->execution_status_ != ExecutionStatus::NotPlaying)
    return;
  if (this->execution_status_ == ExecutionStatus::NotPlaying) {
    const auto copied_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  this->execution_status_ = ExecutionStatus::Playing;
}
void Application::Stop() {
  ApplicationContextScope application_scope(*this);
  if (!this->active_scene_ || this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (this->execution_status_ == ExecutionStatus::NotPlaying)
    return;
  this->execution_status_ = ExecutionStatus::NotPlaying;
  Attach(ProjectManager::GetStartScene().lock());
}
void Application::Pause() {
  ApplicationContextScope application_scope(*this);
  if (!this->active_scene_ || this->execution_status_ == ExecutionStatus::OnDestroy)
    return;
  if (this->execution_status_ != ExecutionStatus::Playing)
    return;
  this->execution_status_ = ExecutionStatus::Pause;
}

void Application::Step() {
  ApplicationContextScope application_scope(*this);
  if (this->execution_status_ != ExecutionStatus::Pause && this->execution_status_ != ExecutionStatus::NotPlaying)
    return;
  if (this->execution_status_ == ExecutionStatus::NotPlaying) {
    const auto copied_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(ProjectManager::GetStartScene().lock(), copied_scene);
    Attach(copied_scene);
  }
  this->execution_status_ = ExecutionStatus::Step;
}

Application::ExecutionOrder Application::GetApplicationExecutionStatus() const {
  return this->execution_order;
}

void Application::RegisterPreUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_pre_update_functions_.push_back(func);
}

void Application::RegisterUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_update_functions_.push_back(func);
}

void Application::RegisterLateUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_late_update_functions_.push_back(func);
}
void Application::RegisterFixedUpdateFunction(const std::function<void()>& func) {
  ApplicationContextScope application_scope(*this);
  this->external_fixed_update_functions_.push_back(func);
}

void Application::RegisterPostAttachSceneFunction(
    const std::function<void(const std::shared_ptr<Scene>& new_scene)>& func) {
  ApplicationContextScope application_scope(*this);
  this->post_attach_scene_functions_.push_back(func);
}

bool Application::IsPlaying() const {
  return this->execution_status_ == ExecutionStatus::Playing;
}
