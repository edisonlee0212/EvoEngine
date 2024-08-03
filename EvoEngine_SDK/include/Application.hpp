#pragma once
#include "Console.hpp"
#include "ILayer.hpp"
#include "ISingleton.hpp"

namespace evo_engine {

struct ApplicationInfo {
  std::filesystem::path project_path;
  std::string application_name = "evo_engine";
  glm::ivec2 default_window_size = {1280, 720};
  bool enable_docking = true;
  bool enable_viewport = false;
  bool full_screen = false;
};

enum class ApplicationStatus {
  Uninitialized,
  NoProject,

  Stop,
  Pause,
  Step,
  Playing,

  OnDestroy
};

enum class ApplicationExecutionStatus { Stop, PreUpdate, Update, LateUpdate };

class Application final {
  EVOENGINE_SINGLETON_INSTANCE(Application)
  friend class ProjectManager;
  ApplicationInfo application_info_;
  ApplicationStatus application_status_ = ApplicationStatus::Uninitialized;

  static void PreUpdateInternal();
  static void UpdateInternal();
  static void LateUpdateInternal();

  std::vector<std::shared_ptr<ILayer>> layers_;
  std::shared_ptr<Scene> active_scene_;

  std::vector<std::function<void()>> external_pre_update_functions_;
  std::vector<std::function<void()>> external_update_functions_;
  std::vector<std::function<void()>> external_fixed_update_functions_;
  std::vector<std::function<void()>> external_late_update_functions_;

  std::vector<std::function<void(const std::shared_ptr<Scene>& new_scene)>> post_attach_scene_functions_;

  ApplicationExecutionStatus application_execution_status_ = ApplicationExecutionStatus::Stop;

 public:
  [[nodiscard]] static ApplicationExecutionStatus GetApplicationExecutionStatus();
  static void RegisterPreUpdateFunction(const std::function<void()>& func);
  static void RegisterUpdateFunction(const std::function<void()>& func);
  static void RegisterLateUpdateFunction(const std::function<void()>& func);
  static void RegisterFixedUpdateFunction(const std::function<void()>& func);
  static void RegisterPostAttachSceneFunction(const std::function<void(const std::shared_ptr<Scene>& new_scene)>& func);
  static bool IsPlaying();
  static const ApplicationInfo& GetApplicationInfo();
  static const ApplicationStatus& GetApplicationStatus();
  template <typename T>
  static std::shared_ptr<T> PushLayer();
  template <typename T>
  static std::shared_ptr<T> GetLayer();
  template <typename T>
  static void PopLayer();
  static void Reset();
  static void Initialize(const ApplicationInfo& application_create_info);
  static void Start();
  static void Run();
  [[maybe_unused]] static bool Loop();
  static void End();
  static void Terminate();
  static const std::vector<std::shared_ptr<ILayer>>& GetLayers();
  static void Attach(const std::shared_ptr<Scene>& scene);
  static std::shared_ptr<Scene> GetActiveScene();
  static void Play();
  static void Pause();
  static void Step();

  static void Stop();
};

template <typename T>
std::shared_ptr<T> Application::PushLayer() {
  auto& application = GetInstance();
  if (application.application_status_ != ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Unable to push layer! Application already started!");
    return nullptr;
  }
  auto test = GetLayer<T>();
  if (!test) {
    test = std::make_shared<T>();
    if (!std::dynamic_pointer_cast<ILayer>(test)) {
      EVOENGINE_ERROR("Not a layer!");
      return nullptr;
    }
    if (!application.layers_.empty())
      application.layers_.back()->subsequent_layer_ = test;
    application.layers_.push_back(std::dynamic_pointer_cast<ILayer>(test));
  }
  return test;
}

template <typename T>
std::shared_ptr<T> Application::GetLayer() {
  const auto& application = GetInstance();
  for (auto& i : application.layers_) {
    if (auto test = std::dynamic_pointer_cast<T>(i))
      return test;
  }
  return nullptr;
}
template <typename T>
void Application::PopLayer() {
  auto& application = GetInstance();
  int index = 0;
  for (auto& i : application.layers_) {
    if (auto test = std::dynamic_pointer_cast<T>(i)) {
      std::dynamic_pointer_cast<ILayer>(i)->OnDestroy();
      application.layers_.erase(application.layers_.begin() + index);
    }
    index++;
  }
}
}  // namespace evo_engine
