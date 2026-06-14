
#pragma once
#include <functional>
#include <memory>

#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "Console.hpp"
#include "ILayer.hpp"
#include "InspectorRegistry.hpp"
#include "Serialization.hpp"
namespace evo_engine {
class AssetManager;
class Console;
class Entities;
class FileManager;
class GeometryStorage;
class Input;
class Jobs;
class PackageManager;
class Platform;
class ProjectManager;
class Resources;
class TextureStorage;
class Times;
class TransformGraph;

/**
 * @brief The main application class responsible for managing the entire engine lifecycle.
 */
class Application final {
  friend class Serialization;
  friend class ProjectManager;

 public:
  Application();
  ~Application();
  Application(const Application&) = delete;
  Application& operator=(const Application&) = delete;
  Application(Application&&) = delete;
  Application& operator=(Application&&) = delete;

  /**
   * @brief Enum representing the various statuses of the application.
   */
  enum class ExecutionStatus {
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
  enum class ExecutionOrder {
    NotPlaying, /**< The application is not in play mode. */
    PreUpdate,  /**< Pre-update phase of the application. */
    Update,     /**< Update phase of the application. */
    LateUpdate  /**< Late update phase of the application. */
  };

 private:
  ApplicationInitializationSettings
      initialization_settings;             /**< Information related to the application configuration. */
  Serialization serialization_registry_{}; /**< Application-owned type and serialization registry. */
  std::unique_ptr<AssetManager> asset_manager_;
  std::unique_ptr<Console> console_;
  std::unique_ptr<Entities> entities_;
  std::unique_ptr<FileManager> file_manager_;
  std::unique_ptr<GeometryStorage> geometry_storage_;
  std::unique_ptr<Input> input_;
  std::unique_ptr<Jobs> jobs_;
  std::unique_ptr<PackageManager> package_manager_;
  std::unique_ptr<Platform> platform_;
  std::unique_ptr<ProjectManager> project_manager_;
  std::unique_ptr<Resources> resources_;
  std::unique_ptr<TextureStorage> texture_storage_;
  std::unique_ptr<Times> times_;
  std::unique_ptr<TransformGraph> transform_graph_;
  ExecutionStatus execution_status_ = ExecutionStatus::Uninitialized; /**< Current status of the application. */

  void PreUpdateInternal();  /**< Perform internal pre-update tasks. */
  void UpdateInternal();     /**< Perform internal update tasks. */
  void LateUpdateInternal(); /**< Perform internal late-update tasks. */
  void ExecuteEndOfLoopActions();

  std::vector<std::shared_ptr<ILayer>> layers_; /**< List of all layers added to the application. */
  std::shared_ptr<Scene> active_scene_;         /**< The currently active scene. */

  std::vector<std::function<void()>> external_pre_update_functions_;   /**< External pre-update functions. */
  std::vector<std::function<void()>> external_update_functions_;       /**< External update functions. */
  std::vector<std::function<void()>> external_fixed_update_functions_; /**< External fixed update functions. */
  std::vector<std::function<void()>> external_late_update_functions_;  /**< External late update functions. */
  std::vector<std::function<void()>> end_of_loop_actions_;             /**< One-shot actions executed after a loop. */

  std::vector<std::function<void(const std::shared_ptr<Scene>& new_scene)>>
      post_attach_scene_functions_; /**< Functions called after a scene is attached. */

  ExecutionOrder execution_order = ExecutionOrder::NotPlaying; /**< Current execution status of the application. */

 public:
  [[nodiscard]] Serialization& GetSerialization();
  [[nodiscard]] const Serialization& GetSerialization() const;
  [[nodiscard]] AssetManager& GetAssetManager();
  [[nodiscard]] Console& GetConsole();
  [[nodiscard]] Entities& GetEntities();
  [[nodiscard]] FileManager& GetFileManager();
  [[nodiscard]] GeometryStorage& GetGeometryStorage();
  [[nodiscard]] Input& GetInput();
  [[nodiscard]] Jobs& GetJobs();
  [[nodiscard]] PackageManager& GetPackageManager();
  [[nodiscard]] Platform& GetPlatform();
  [[nodiscard]] ProjectManager& GetProjectManager();
  [[nodiscard]] Resources& GetResources();
  [[nodiscard]] TextureStorage& GetTextureStorage();
  [[nodiscard]] Times& GetTimes();
  [[nodiscard]] TransformGraph& GetTransformGraph();

  template <typename T>
  void RegisterDataComponent(const std::string& name);
  template <typename T>
  void RegisterPrivateComponent(const std::string& name);
  template <typename T>
  void RegisterAsset(const std::string& name, const std::vector<std::string>& external_extensions);
  template <typename T>
  void RegisterSystem(const std::string& name);

  /**
   * @brief Get the current execution status of the application.
   * @return The current ApplicationExecutionStatus.
   */
  [[nodiscard]] ExecutionOrder GetApplicationExecutionStatus() const;

  /**
   * @brief Register a function to be called during the pre-update phase.
   * @param func The callback function to register.
   */
  void RegisterPreUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called during the update phase.
   * @param func The callback function to register.
   */
  void RegisterUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called during the late update phase.
   * @param func The callback function to register.
   */
  void RegisterLateUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Queue a one-shot action that will run after the current application loop completes.
   * @param func The callback function to execute at the end of the loop.
   */
  void QueueEndOfLoopAction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called during the fixed update phase.
   * @param func The callback function to register.
   */
  void RegisterFixedUpdateFunction(const std::function<void()>& func);

  /**
   * @brief Register a function to be called after attaching a new scene.
   * @param func The callback function to register.
   */
  void RegisterPostAttachSceneFunction(const std::function<void(const std::shared_ptr<Scene>& new_scene)>& func);

  /**
   * @brief Checks if the application is currently playing.
   * @return True if the application is playing, false otherwise.
   */
  bool IsPlaying() const;

  /**
   * @brief Get the information about the application configuration.
   * @return A const reference to the ApplicationInfo structure.
   */
  const ApplicationInitializationSettings& GetApplicationInfo() const;

  /**
   * @brief Get the current status of the application.
   * @return A const reference to the ApplicationStatus enum.
   */
  const ExecutionStatus& GetApplicationStatus() const;

  /**
   * @brief Add a new layer to the application.
   * @tparam T The type of the layer to add.
   * @param layer_name The name of the layer.
   * @return A shared pointer to the newly added layer.
   */
  template <typename T>
  std::shared_ptr<T> PushLayer(const std::string& layer_name = "", const std::string& package_owner = "");

  /**
   * @brief Retrieve a layer of a specific type.
   * @tparam T The type of the layer to retrieve.
   * @return A shared pointer to the layer of type T, or nullptr if not found.
   */
  template <typename T>
  std::shared_ptr<T> GetLayer() const;

  /**
   * @brief Remove a layer of a specific type from the application.
   * @tparam T The type of the layer to remove.
   */
  template <typename T>
  void PopLayer();

  /**
   * @brief Reset the application state.
   */
  void Reset();

  /**
   * @brief Initialize the application with the specified configuration.
   * @param application_create_info The configuration to initialize the application with.
   */
  void Initialize(const ApplicationInitializationSettings& application_create_info);

  /**
   * @brief Start the application.
   * @param autoplay Whether to start in autoplay mode (default: true).
   */
  void Start(bool autoplay = true);

  /**
   * @brief Run the main application loop.
   */
  void Run();

  /**
   * @brief Execute a single application loop iteration.
   * @return True if the loop should continue, false otherwise.
   */
  [[maybe_unused]] bool Loop();

  /**
   * @brief End the application loop and perform cleanup.
   */
  void End();

  /**
   * @brief Terminate the application.
   */
  void Terminate();

  /**
   * @brief Get the list of all layers added to the application.
   * @return A const reference to a vector of shared pointers to ILayer objects.
   */
  const std::vector<std::shared_ptr<ILayer>>& GetLayers() const;

  /**
   * @brief Destroy and remove all layers owned by a runtime package.
   * @param package_name Name of the owning runtime package.
   * @return True if all package-owned layers were removed safely.
   */
  bool RemoveLayersOwnedByPackage(const std::string& package_name);

  /**
   * @brief Attach a new scene to the application.
   * @param scene The scene to attach.
   */
  void Attach(const std::shared_ptr<Scene>& scene);

  /**
   * @brief Get the currently active scene.
   * @return A shared pointer to the active scene.
   */
  std::shared_ptr<Scene> GetActiveScene() const;

  /**
   * @brief Start playing the application.
   */
  void Play();

  /**
   * @brief Pause the application.
   */
  void Pause();

  /**
   * @brief Step through a single update iteration of the application.
   */
  void Step();

  /**
   * @brief Stop the application.
   */
  void Stop();
};

template <typename T>
void Application::RegisterDataComponent(const std::string& name) {
  Serialization::RegisterDataComponentType<T>(name);
}

template <typename T>
void Application::RegisterPrivateComponent(const std::string& name) {
  Serialization::RegisterSerializableType<T>(name);
  Serialization::RegisterPrivateComponentType<T>(name);
  Serialization::RegisterDefaultSerializationHandler<T>({}, name);
  Serialization::RegisterDefaultSerializationSupportHandler<T>({}, name);
  InspectorRegistry::GetInstance().RegisterDefaultInspector<T>({}, name);
}

template <typename T>
void Application::RegisterAsset(const std::string& name, const std::vector<std::string>& external_extensions) {
  Serialization::RegisterAssetType<T>(name, external_extensions);
  Serialization::RegisterDefaultSerializationHandler<T>({}, name);
  Serialization::RegisterDefaultSerializationSupportHandler<T>({}, name);
  Serialization::RegisterDefaultAssetIoHandler<T>({}, name);
  Serialization::RegisterDefaultAssetPreviewHandler<T>({}, name);
  InspectorRegistry::GetInstance().RegisterDefaultInspector<T>({}, name);
}

template <typename T>
void Application::RegisterSystem(const std::string& name) {
  Serialization::RegisterSerializableType<T>(name);
  Serialization::RegisterSystemType<T>(name);
  Serialization::RegisterDefaultSerializationHandler<T>({}, name);
  Serialization::RegisterDefaultSerializationSupportHandler<T>({}, name);
  InspectorRegistry::GetInstance().RegisterDefaultInspector<T>({}, name);
}

/**
 * @brief Add a new layer to the application.
 * @tparam T The type of the layer to add.
 * @param layer_name The name of the layer.
 * @return A shared pointer to the newly added layer.
 */
template <typename T>
std::shared_ptr<T> Application::PushLayer(const std::string& layer_name, const std::string& package_owner) {
  if (execution_status_ == ExecutionStatus::OnDestroy) {
    EVOENGINE_ERROR("Unable to push layer! Application is being destroyed!");
    return nullptr;
  }
  if (execution_status_ != ExecutionStatus::Uninitialized && package_owner.empty()) {
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
    if (!layers_.empty())
      layers_.back()->subsequent_layer_ = test;
    layers_.push_back(std::dynamic_pointer_cast<ILayer>(test));
    layers_.back()->self_ = test;
    layers_.back()->application_ = this;
    layers_.back()->package_owner_ = package_owner;
    InspectorRegistry::GetInstance().RegisterDefaultInspector<T>(package_owner, layer_name);
    if (this->active_scene_) {
      layers_.back()->scene_ = this->active_scene_;
    }
    if (execution_status_ != ExecutionStatus::Uninitialized) {
      if (package_owner.empty()) {
        layers_.back()->RegisterTypes(*this);
      }
      layers_.back()->OnCreate();
    }
  } else if (!package_owner.empty()) {
    const auto existing_layer = std::dynamic_pointer_cast<ILayer>(test);
    if (existing_layer->package_owner_ != package_owner) {
      EVOENGINE_ERROR("Unable to push runtime package layer! Layer type is already owned by another module.")
      return nullptr;
    }
  }
  if (!layer_name.empty())
    std::dynamic_pointer_cast<ILayer>(test)->layer_name_ = layer_name;
  return test;
}

/**
 * @brief Retrieve a layer of a specific type.
 * @tparam T The type of the layer to retrieve.
 * @return A shared pointer to the layer of type T, or nullptr if not found.
 */
template <typename T>
std::shared_ptr<T> Application::GetLayer() const {
  for (auto& i : layers_) {
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
  int index = 0;
  for (auto& i : layers_) {
    if (auto test = std::dynamic_pointer_cast<T>(i)) {
      std::dynamic_pointer_cast<ILayer>(i)->OnDestroy();
      layers_.erase(layers_.begin() + index);
    }
    index++;
  }
}

}  // namespace evo_engine
