
#pragma once
#include "Console.hpp"
#include "ILayer.hpp"
#include "ISingleton.hpp"

namespace evo_engine {

/**
 * @brief Structure for storing information about the application.
 */
struct ApplicationInfo {
  std::filesystem::path project_path;            /**< The path to the application's project. */
  std::string application_name = "Evo Engine";   /**< The name of the application. */
  std::vector<std::filesystem::path> icon_paths; /**< Paths to application icons. */
  glm::ivec2 default_window_size = {1280, 720};  /**< The default size of the application window. */
  bool enable_docking = true;                    /**< Whether to enable docking in the application. */
  bool enable_viewport = true;                   /**< Whether to enable the viewport feature. */
  bool full_screen = false;                      /**< Whether the application starts in full-screen mode. */
};

/**
 * @brief Enum representing the various statuses of the application.
 */
enum class ApplicationStatus {
  Uninitialized, /**< The application has not been initialized. */

  NotPlaying, /**< The application is not currently playing. */
  Pause,      /**< The application is paused. */
  Step,       /**< The application is stepping through updates. */
  Playing,    /**< The application is currently playing. */

  OnDestroy /**< The application is being destroyed. */
};

/**
 * @brief Enum representing the execution status of the application.
 */
enum class ApplicationExecutionStatus {
  NotPlaying, /**< The application is not in play mode. */
  PreUpdate,  /**< Pre-update phase of the application. */
  Update,     /**< Update phase of the application. */
  LateUpdate  /**< Late update phase of the application. */
};

/**
 * @brief The main application class responsible for managing the entire engine lifecycle.
 */
class Application final {
  EVOENGINE_SINGLETON_INSTANCE(Application)
  friend class ProjectManager;

  ApplicationInfo application_info_; /**< Information related to the application configuration. */
  ApplicationStatus application_status_ = ApplicationStatus::Uninitialized; /**< Current status of the application. */

  static void PreUpdateInternal();  /**< Perform internal pre-update tasks. */
  static void UpdateInternal();     /**< Perform internal update tasks. */
  static void LateUpdateInternal(); /**< Perform internal late-update tasks. */

  std::vector<std::shared_ptr<ILayer>> layers_; /**< List of all layers added to the application. */
  std::shared_ptr<Scene> active_scene_;         /**< The currently active scene. */

  std::vector<std::function<void()>> external_pre_update_functions_;   /**< External pre-update functions. */
  std::vector<std::function<void()>> external_update_functions_;       /**< External update functions. */
  std::vector<std::function<void()>> external_fixed_update_functions_; /**< External fixed update functions. */
  std::vector<std::function<void()>> external_late_update_functions_;  /**< External late update functions. */

  std::vector<std::function<void(const std::shared_ptr<Scene>& new_scene)>>
      post_attach_scene_functions_; /**< Functions called after a scene is attached. */

  ApplicationExecutionStatus application_execution_status_ =
      ApplicationExecutionStatus::NotPlaying; /**< Current execution status of the application. */

 public:
  /**
   * @brief Get the current execution status of the application.
   * @return The current ApplicationExecutionStatus.
   */
  [[nodiscard]] static ApplicationExecutionStatus GetApplicationExecutionStatus();

  /**
   * @brief Register a function to be called during the pre-update phase.
   * @param func The callback function to register.
   */
  static void RegisterPreUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called during the update phase.
   * @param func The callback function to register.
   */
  static void RegisterUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called during the late update phase.
   * @param func The callback function to register.
   */
  static void RegisterLateUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called during the fixed update phase.
   * @param func The callback function to register.
   */
  static void RegisterFixedUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called after attaching a new scene.
   * @param func The callback function to register.
   */
  static void RegisterPostAttachSceneFunction(const std::function<void(const std::shared_ptr<Scene>& new_scene)>& func);

  /**
   * @brief Checks if the application is currently playing.
   * @return True if the application is playing, false otherwise.
   */
  static bool IsPlaying();

  /**
   * @brief Get the information about the application configuration.
   * @return A const reference to the ApplicationInfo structure.
   */
  static const ApplicationInfo& GetApplicationInfo();

  /**
   * @brief Get the current status of the application.
   * @return A const reference to the ApplicationStatus enum.
   */
  static const ApplicationStatus& GetApplicationStatus();

  /**
   * @brief Add a new layer to the application.
   * @tparam T The type of the layer to add.
   * @param layer_name The name of the layer.
   * @return A shared pointer to the newly added layer.
   */
  template <typename T>
  static std::shared_ptr<T> PushLayer(const std::string& layer_name);

  /**
   * @brief Retrieve a layer of a specific type.
   * @tparam T The type of the layer to retrieve.
   * @return A shared pointer to the layer of type T, or nullptr if not found.
   */
  template <typename T>
  static std::shared_ptr<T> GetLayer();

  /**
   * @brief Remove a layer of a specific type from the application.
   * @tparam T The type of the layer to remove.
   */
  template <typename T>
  static void PopLayer();

  /**
   * @brief Reset the application state.
   */
  static void Reset();

  /**
   * @brief Initialize the application with the specified configuration.
   * @param application_create_info The configuration to initialize the application with.
   */
  static void Initialize(const ApplicationInfo& application_create_info);

  /**
   * @brief Start the application.
   * @param autoplay Whether to start in autoplay mode (default: true).
   */
  static void Start(bool autoplay = true);

  /**
   * @brief Run the main application loop.
   */
  static void Run();

  /**
   * @brief Execute a single application loop iteration.
   * @return True if the loop should continue, false otherwise.
   */
  [[maybe_unused]] static bool Loop();

  /**
   * @brief End the application loop and perform cleanup.
   */
  static void End();

  /**
   * @brief Terminate the application.
   */
  static void Terminate();

  /**
   * @brief Get the list of all layers added to the application.
   * @return A const reference to a vector of shared pointers to ILayer objects.
   */
  static const std::vector<std::shared_ptr<ILayer>>& GetLayers();

  /**
   * @brief Attach a new scene to the application.
   * @param scene The scene to attach.
   */
  static void Attach(const std::shared_ptr<Scene>& scene);

  /**
   * @brief Get the currently active scene.
   * @return A shared pointer to the active scene.
   */
  static std::shared_ptr<Scene> GetActiveScene();

  /**
   * @brief Start playing the application.
   */
  static void Play();

  /**
   * @brief Pause the application.
   */
  static void Pause();

  /**
   * @brief Step through a single update iteration of the application.
   */
  static void Step();

  /**
   * @brief Stop the application.
   */
  static void Stop();
};

/**
 * @brief Add a new layer to the application.
 * @tparam T The type of the layer to add.
 * @param layer_name The name of the layer.
 * @return A shared pointer to the newly added layer.
 */
template <typename T>
std::shared_ptr<T> Application::PushLayer(const std::string& layer_name) {
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
    application.layers_.back()->self_ = test;
  }
  std::dynamic_pointer_cast<ILayer>(test)->layer_name_ = layer_name;
  return test;
}

/**
 * @brief Retrieve a layer of a specific type.
 * @tparam T The type of the layer to retrieve.
 * @return A shared pointer to the layer of type T, or nullptr if not found.
 */
template <typename T>
std::shared_ptr<T> Application::GetLayer() {
  const auto& application = GetInstance();
  for (auto& i : application.layers_) {
    if (auto test = std::dynamic_pointer_cast<T>(i))
      return test;
  }
  return nullptr;
}

/**
 * @brief Remove a layer of a specific type from the application.
 * @tparam T The type of the layer to remove.
 */
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