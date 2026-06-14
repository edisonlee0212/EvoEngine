
#pragma once
#include "Input.hpp"

namespace evo_engine {
class Application;

/**
 * @class Scene
 * @brief Forward declaration of the Scene class.
 * This class is used in the ILayer class as a weak pointer.
 */
class Scene;

/**
 * @class EditorLayer
 * @brief Forward declaration of the EditorLayer class.
 * This class is used in the ILayer class as a friend class and shared pointer.
 */
class EditorLayer;

/**
 * @class ILayer
 * @brief Base class for layers in the Evo Engine framework.
 *
 * Provides an interface for creating, updating, and destroying layers within a Scene.
 * Layers can handle various events and operations such as updates, input events, and editor inspection.
 */
class ILayer {
  /**
   * @brief The name of the layer, defaults to "Unknown Layer".
   */
  std::string layer_name_ = "Unknown Layer";

  /**
   * @brief Weak pointer to the Scene to which this layer belongs.
   */
  std::weak_ptr<Scene> scene_;

  /**
   * @brief Weak pointer to the subsequent layer that allows chaining within the layer management system.
   */
  std::weak_ptr<ILayer> subsequent_layer_;

  /**
   * @brief Weak pointer to self.
   */
  std::weak_ptr<ILayer> self_;

  Application* application_ = nullptr;
  std::string package_owner_{};

  /**
   * @brief Grants Application class access to private and protected members of ILayer.
   */
  friend class Application;

  /**
   * @brief Grants EditorLayer class access to private and protected members of ILayer.
   */
  friend class EditorLayer;

  /**
   * @brief Grants Input class access to private and protected members of ILayer.
   */
  friend class Input;

  /**
   * @brief Invoked when the layer is created.
   *
   * This is a virtual function that can be overridden by derived classes to perform custom initialization.
   */
  virtual void OnCreate() {
  }

  virtual void RegisterTypes(Application& application) {
  }

  /**
   * @brief Invoked when the layer is destroyed.
   *
   * This is a virtual function that can be overridden by derived classes to perform custom cleanup.
   */
  virtual void OnDestroy() {
  }

  /**
   * @brief Invoked before the main update loop.
   *
   * Allows derived classes to perform operations that should occur before the primary update logic.
   */
  virtual void PreUpdate() {
  }

  /**
   * @brief Invoked during the fixed update loop.
   *
   * This function is typically used for physics-based updates or other deterministic operations at a fixed interval.
   */
  virtual void FixedUpdate() {
  }

  /**
   * @brief The main update function of the layer.
   *
   * This function is called during the standard update phase and can be overridden to implement custom update logic.
   */
  virtual void Update() {
  }

  /**
   * @brief Invoked during the late update phase.
   *
   * Allows derived classes to perform operations that should occur after all other updates have been completed.
   */
  virtual void LateUpdate() {
  }

  /**
   * @brief Processes input events received by the layer.
   *
   * Derived classes can override this function to handle specific input events.
   *
   * @param input_event The input event to process.
   */
  virtual void OnInputEvent(const Input::InputEvent& input_event);

 public:
  [[nodiscard]] Application& GetApplication() const;

  [[nodiscard]] std::shared_ptr<ILayer> GetSelf() const;

  /**
   * @brief Retrieves the name of the layer.
   *
   * @return A string representing the name of the layer.
   */
  [[nodiscard]] std::string GetLayerName() const;

  /**
   * @brief Indicates whether the layer enables editor inspection.
   */
  bool enable_inspection = true;

  /**
   * @brief Retrieves the Scene associated with this layer.
   *
   * @return A shared pointer to the Scene object.
   */
  [[nodiscard]] std::shared_ptr<Scene> GetScene() const;
};

}  // namespace evo_engine
