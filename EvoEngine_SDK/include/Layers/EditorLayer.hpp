
#pragma once
#include "Application.hpp"
#include "Camera.hpp"
#include "EditorPanelManager.hpp"
#include "Entity.hpp"
#include "GraphicsResources.hpp"
#include "ILayer.hpp"
#include "ISystem.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "PrivateComponentRef.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Strands.hpp"
#include "Texture2D.hpp"

#include <filesystem>
#include <future>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace evo_engine {

/**
 * @brief Enumeration of console message types.
 */
enum class ConsoleMessageType {
  Log,     /**< Log type message. */
  Warning, /**< Warning type message. */
  Error    /**< Error type message. */
};

/**
 * @brief Structure representing a console message.
 */
struct ConsoleMessage {
  ConsoleMessageType m_type = ConsoleMessageType::Log; /**< The type of the console message. */
  std::string m_value;                                 /**< The value/content of the console message. */
  double m_time = 0;                                   /**< The timestamp of the console message. */
};

/**
 * @brief Structure representing the settings for drawing gizmos.
 */
struct GizmoSettings {
  DrawSettings draw_settings; /**< The draw settings for the gizmo. */

  /**
   * @brief Enumeration of color modes for gizmos.
   */
  enum class ColorMode {
    Default,                         /**< Use the default color mode. */
    VertexColor,                     /**< Use the vertex color mode. */
    NormalColor                      /**< Use the normal color mode. */
  } color_mode = ColorMode::Default; /**< The color mode for the gizmo. */

  bool depth_test = false;  /**< Indicates whether depth testing is enabled. */
  bool depth_write = false; /**< Indicates whether depth writing is enabled. */

  /**
   * @brief Applies the gizmo settings to a global pipeline state.
   *
   * @param global_pipeline_state Reference to the global pipeline states that will be updated.
   */
  void ApplySettings(GraphicsPipelineStates& global_pipeline_state) const;
};

/**
 * @brief Structure representing push constants for gizmos.
 */
struct GizmosPushConstant {
  glm::mat4 model;      /**< The model matrix for the gizmos. */
  glm::vec4 color;      /**< The color vector for the gizmos. */
  float size;           /**< The size of the gizmos. */
  int32_t camera_index; /**< Index of the camera associated with the gizmos. */
};

/**
 * @brief Structure representing an editor camera.
 */
struct EditorCamera {
  glm::quat rotation = glm::quat(glm::radians(glm::vec3(0.0f, 0.0f, 0.0f))); /**< The rotation of the editor camera. */
  glm::vec3 position = glm::vec3(0, 2, 5);                                   /**< The position of the editor camera. */
  std::shared_ptr<Camera> camera; /**< Shared pointer to the camera entity. */
};

/**
 * @brief Structure representing a gizmo mesh task.
 */
struct GizmoMeshTask {
  std::shared_ptr<Mesh> mesh;                      /**< Shared pointer to the mesh associated with the gizmo. */
  std::shared_ptr<Camera> editor_camera_component; /**< Shared pointer to the editor camera component. */
  glm::vec4 color;                                 /**< The color of the gizmo mesh. */
  glm::mat4 model;                                 /**< The model matrix for the gizmo mesh. */
  float size;                                      /**< The size of the gizmo mesh. */
  GizmoSettings gizmo_settings;                    /**< The settings for the gizmo. */
};

/**
 * @brief Structure representing a gizmo instanced mesh task.
 */
struct GizmoInstancedMeshTask {
  std::shared_ptr<Mesh> mesh;                           /**< Shared pointer to the instanced mesh. */
  std::shared_ptr<Camera> editor_camera_component;      /**< Shared pointer to the editor camera component. */
  std::shared_ptr<ParticleInfoList> particle_info_list; /**< Shared pointer to particle instance data. */
  glm::mat4 model;                                      /**< The model matrix for the instanced gizmo mesh. */
  float size;                                           /**< The size of the instanced gizmo mesh. */
  GizmoSettings gizmo_settings;                         /**< The settings for the gizmo. */
};

/**
 * @brief Structure representing a gizmo strands task.
 */
struct GizmoStrandsTask {
  std::shared_ptr<Strands> strands;                /**< Shared pointer to the strands associated with the gizmo. */
  std::shared_ptr<Camera> editor_camera_component; /**< Shared pointer to the editor camera component. */
  glm::vec4 color;                                 /**< The color of the gizmo strands. */
  glm::mat4 model;                                 /**< The model matrix for the gizmo strands. */
  float size;                                      /**< The size of the gizmo strands. */
  GizmoSettings gizmo_settings;                    /**< Settings for the gizmo strands. */
};

/**
 * @brief The main layer for the editor functionalities.
 */
class EditorLayer : public ILayer {
 public:
  /**
   * @brief Finds and retrieves an icon texture by its name.
   *
   * @param name The name of the icon to find.
   * @return A shared pointer to the found Texture2D.
   */
  static std::shared_ptr<Texture2D> FindIcon(const std::string& name);
  std::shared_ptr<IAsset> inspecting_asset;  ///< The asset currently being inspected in the editor.

  bool show_console_window = true; /**< Indicates whether the console window is visible. */

  /**
   * @brief Retrieves the list of console messages.
   *
   * @return A reference to the vector of console messages.
   */
  std::vector<ConsoleMessage>& GetConsoleMessages();

  [[nodiscard]] bool SceneCameraWindowFocused() const; /**< Checks if the Scene Camera window is focused. */
  [[nodiscard]] bool MainCameraWindowFocused() const;  /**< Checks if the Main Camera window is focused. */

  bool enable_view_gizmos = false;  /**< Indicates if view gizmos are enabled. */
  bool enable_gizmos = true;        /**< Indicates if gizmos are enabled. */
  bool transform_read_only = false; /**< Indicates if transformations are read-only. */

  /**
   * @brief Registers an editor camera.
   *
   * @param camera A shared pointer to the camera to register.
   */
  void RegisterEditorCamera(const std::shared_ptr<Camera>& camera);

  /**
   * @brief Provides a reference to the position of the editor camera.
   *
   * @param handle The handle associated with the editor camera.
   * @return A reference to the position vector of the editor camera.
   */
  glm::vec3& RefEditorCameraPosition(const Handle& handle);

  /**
   * @brief Provides a reference to the rotation of the editor camera.
   *
   * @param handle The handle associated with the editor camera.
   * @return A reference to the rotation quaternion of the editor camera.
   */
  glm::quat& RefEditorCameraRotation(const Handle& handle);

  /**
   * @brief Retrieves the mouse position in the Scene Camera window.
   *
   * @return A 2D vector representing the mouse position in the Scene Camera window.
   */
  [[nodiscard]] glm::vec2 GetMouseSceneCameraPosition() const;

  [[nodiscard]] static Input::KeyActionType GetKey(int key); /**< Gets the key action type for a given key. */

  [[nodiscard]] std::shared_ptr<Camera> GetSceneCamera(); /**< Retrieves the scene camera. */

  [[nodiscard]] glm::vec3 GetSceneCameraPosition() const; /**< Retrieves the position of the scene camera. */

  [[nodiscard]] glm::quat GetSceneCameraRotation() const; /**< Retrieves the rotation of the scene camera. */

  /**
   * @brief Sets the position of the scene camera.
   *
   * @param target_position The new position of the scene camera.
   */
  void SetSceneCameraPosition(const glm::vec3& target_position);

  /**
   * @brief Sets the rotation of the scene camera.
   *
   * @param target_rotation The new rotation of the scene camera.
   */
  void SetSceneCameraRotation(const glm::quat& target_rotation);

  /**
   * @brief Moves the camera to a new position and rotation over a specified transition time.
   *
   * @param target_rotation The target rotation for the camera.
   * @param target_position The target position for the camera.
   * @param transition_time The time it takes to transition to the new position and rotation.
   */
  void MoveCamera(const glm::quat& target_rotation, const glm::vec3& target_position,
                  const float& transition_time = 1.0f);

  /**
   * @brief Updates the texture ID with the provided Vulkan image properties.
   *
   * @param target Reference to the ImTextureID to update.
   * @param image_sampler Vulkan sampler associated with the texture.
   * @param image_view Vulkan image view associated with the texture.
   * @param image_layout Vulkan image layout associated with the texture.
   */
  static auto UpdateTextureId(ImTextureID& target, VkSampler image_sampler, VkImageView image_view,
                              VkImageLayout image_layout) -> void;

  /**
   * @brief Retrieves the currently selected entity.
   *
   * @return The selected entity.
   */
  [[nodiscard]] Entity GetSelectedEntity() const;

  /**
   * @brief Sets the selected entity and optionally opens the context menu.
   *
   * @param entity The entity to set as selected.
   * @param open_menu Whether to open the context menu after selection.
   */
  void SetSelectedEntity(const Entity& entity, bool open_menu = true);

  float scene_camera_resolution_multiplier = 1.0f; /**< Multiplier for the scene camera resolution. */

  /**
   * @brief Checks if entity selection is locked.
   *
   * @return True if entity selection is locked; otherwise, false.
   */
  [[nodiscard]] bool GetLockEntitySelection() const;

  /**
   * @brief Sets the lock state for entity selection.
   *
   * @param value True to lock entity selection, false to unlock.
   */
  void SetLockEntitySelection(bool value);

  bool show_scene_camera_debug = false;     /**< Indicates whether the scene camera debug window is visible. */
  bool show_scene_window = true;            /**< Indicates whether the scene window is visible. */
  bool show_camera_window = true;           /**< Indicates whether the camera window is visible. */
  bool show_camera_info = false;            /**< Indicates whether the camera info window is visible. */
  bool show_play_buttons = true;            /**< Indicates whether the play buttons are visible. */
  bool show_scene_info = true;              /**< Indicates whether the scene info window is visible. */
  bool show_entity_explorer_window = true;  /**< Indicates whether the entity explorer window is visible. */
  bool show_entity_inspector_window = true; /**< Indicates whether the entity inspector window is visible. */
  bool show_package_manager_window = false; /**< Indicates whether the runtime package manager window is visible. */
  bool show_layer_inspector_window = true;  /**< Indicates whether the layer inspector window is visible. */
  bool main_camera_focus_override = false;  /**< Indicates if the main camera focus has been overridden. */
  bool scene_camera_focus_override = false; /**< Indicates if the scene camera focus has been overridden. */

  int selected_hierarchy_display_mode = 1;     /**< Selected display mode for the entity hierarchy. */
  float velocity = 10.0f;                      /**< Velocity for camera movement. */
  float sensitivity = 0.1f;                    /**< Sensitivity for camera controls. */
  bool apply_transform_to_main_camera = false; /**< Indicates whether transformations apply to the main camera. */
  bool lock_camera = false;                    /**< Indicates whether the camera is locked. */

  glm::quat default_scene_camera_rotation =
      glm::quat(glm::radians(glm::vec3(0.0f, 0.0f, 0.0f)));     /**< Default rotation of the scene camera. */
  glm::vec3 default_scene_camera_position = glm::vec3(0, 2, 5); /**< Default position of the scene camera. */

  int main_camera_resolution_x = 1;          /**< Resolution width of the main camera. */
  int main_camera_resolution_y = 1;          /**< Resolution height of the main camera. */
  bool main_camera_allow_auto_resize = true; /**< Indicates if the main camera allows automatic resizing. */

  /**
   * @brief Retrieves a previously stored position (unsafe).
   *
   * @return Reference to the previously stored position vector.
   */
  glm::vec3& UnsafeGetPreviouslyStoredPosition();

  /**
   * @brief Retrieves a previously stored rotation (unsafe).
   *
   * @return Reference to the previously stored rotation vector.
   */
  glm::vec3& UnsafeGetPreviouslyStoredRotation();

  /**
   * @brief Retrieves a previously stored scale (unsafe).
   *
   * @return Reference to the previously stored scale vector.
   */
  glm::vec3& UnsafeGetPreviouslyStoredScale();

  /**
   * @brief Checks if the local position is selected.
   *
   * @return True if the local position is selected, false otherwise.
   */
  [[nodiscard]] bool LocalPositionSelected() const;

  /**
   * @brief Checks if the local rotation is selected.
   *
   * @return True if the local rotation is selected, false otherwise.
   */
  [[nodiscard]] bool LocalRotationSelected() const;

  /**
   * @brief Checks if the local scale is selected.
   *
   * @return True if the local scale is selected, false otherwise.
   */
  [[nodiscard]] bool LocalScaleSelected() const;

#pragma region ImGui Helpers
  /**
   * @brief Handles drag-and-drop operations for the camera window.
   */
  void CameraWindowDragAndDrop() const;

  /**
   * @brief Draws an entity menu.
   *
   * @param enabled Whether the menu is enabled.
   * @param entity The entity associated with the menu.
   * @return True if the menu was successfully drawn, false otherwise.
   */
  [[maybe_unused]] bool DrawEntityMenu(const bool& enabled, const Entity& entity) const;

  /**
   * @brief Draws a node for the given entity in the hierarchy.
   *
   * @param entity The entity to draw.
   * @param hierarchy_level The level of the entity in the hierarchy.
   */
  void DrawEntityNode(const Entity& entity, const unsigned& hierarchy_level);

  /**
   * @brief Inspects data of a component associated with a given entity.
   *
   * @param entity The entity whose component data is being inspected.
   * @param data Pointer to the data component.
   * @param type The type of the data component.
   * @param is_root Whether the component is the root component.
   */
  void InspectComponentData(Entity entity, IDataComponent* data, const DataComponentType& type, bool is_root);

  /**
   * @brief Registers a component data inspector for a component type.
   *
   * @tparam T1 The IDataComponent type for inspector registration.
   * @param func Function to be used for inspecting the component data.
   */
  template <typename T1 = IDataComponent>
  void RegisterComponentDataInspector(
      const std::function<bool(Entity entity, IDataComponent* data, bool is_root)>& func);

  /**
   * @brief Draws a drag-and-drop button for an asset reference.
   *
   * @tparam T The type of asset.
   * @param target Reference to the AssetRef to display and modify.
   * @param name The name to display on the button.
   * @param acceptable_type_names A list of acceptable asset type names.
   * @param modifiable Whether the asset ref can be modified via the button.
   * @return True if the status of the asset reference changed, false otherwise.
   */
  bool DragAndDropButton(AssetRef& target, const std::string& name,
                         const std::vector<std::string>& acceptable_type_names, bool modifiable = true);

  /**
   * @brief Draws a drag-and-drop button for a private component reference.
   *
   * @tparam T The type of private component.
   * @param target Reference to the PrivateComponentRef to display and modify.
   * @param name The name to display on the button.
   * @param acceptable_type_names A list of acceptable private component type names.
   * @param modifiable Whether the private component ref can be modified via the button.
   * @return True if the status of the private component reference changed, false otherwise.
   */
  bool DragAndDropButton(PrivateComponentRef& target, const std::string& name,
                         const std::vector<std::string>& acceptable_type_names, bool modifiable = true);

  /**
   * @brief Draws a drag-and-drop button for an asset reference.
   *
   * @tparam T The type of asset.
   * @param target Reference to the AssetRef to display and modify.
   * @param name The name to display on the button.
   * @param modifiable Whether the asset ref can be modified via the button.
   * @return True if the status of the asset reference changed, false otherwise.
   */
  template <typename T = IAsset>
  bool DragAndDropButton(AssetRef& target, const std::string& name, bool modifiable = true);

  /**
   * @brief Draws a drag-and-drop button for a private component reference.
   *
   * @tparam T The type of private component.
   * @param target Reference to the PrivateComponentRef to display and modify.
   * @param name The name to display on the button.
   * @param modifiable Whether the private component ref can be modified via the button.
   * @return True if the status of the private component reference changed, false otherwise.
   */
  template <typename T = IPrivateComponent>
  bool DragAndDropButton(PrivateComponentRef& target, const std::string& name, bool modifiable = true);

  /**
   * @brief Draws a drag-and-drop button for an entity reference.
   *
   * @param entity_ref Reference to the EntityRef to display and modify.
   * @param name The name to display on the button.
   * @param modifiable Whether the entity ref can be modified via the button.
   * @return True if the status of the entity reference changed, false otherwise.
   */
  bool DragAndDropButton(EntityRef& entity_ref, const std::string& name, bool modifiable = true);

  /**
   * @brief Makes an asset reference draggable for drag-and-drop operations.
   *
   * @tparam T The type of asset.
   * @param target The AssetRef to make draggable.
   */
  template <typename T = IAsset>
  static void Draggable(AssetRef& target);

  /**
   * @brief Makes a private component reference draggable for drag-and-drop operations.
   *
   * @tparam T The type of private component.
   * @param target The PrivateComponentRef to make draggable.
   */
  template <typename T = IPrivateComponent>
  static void Draggable(PrivateComponentRef& target);

  /**
   * @brief Makes an entity reference draggable for drag-and-drop operations.
   *
   * @param entity_ref The EntityRef to make draggable.
   */
  static void Draggable(EntityRef& entity_ref);

  /**
   * @brief Makes a shared pointer to an asset draggable for drag-and-drop operations.
   *
   * @tparam T The type of asset.
   * @param target The shared pointer to an asset to make draggable.
   */
  template <typename T = IAsset>
  static void DraggableAsset(const std::shared_ptr<T>& target);

  /**
   * @brief Makes a shared pointer to a private component draggable for drag-and-drop operations.
   *
   * @tparam T The type of private component.
   * @param target The shared pointer to a private component to make draggable.
   */
  template <typename T = IPrivateComponent>
  static void DraggablePrivateComponent(const std::shared_ptr<T>& target);

  /**
   * @brief Makes an entity draggable for drag-and-drop operations.
   *
   * @param entity The entity to make draggable.
   */
  static void DraggableEntity(const Entity& entity);

  /**
   * @brief Drops an asset onto an unsafe droppable area.
   *
   * @param target Reference to the AssetRef target.
   * @param type_names List of acceptable type names for the assets.
   * @return True if the asset was successfully dropped, false otherwise.
   */
  static bool UnsafeDroppableAsset(AssetRef& target, const std::vector<std::string>& type_names);

  /**
   * @brief Drops a private component onto an unsafe droppable area.
   *
   * @param target Reference to the PrivateComponentRef target.
   * @param type_names List of acceptable type names for the private components.
   * @return True if the private component was successfully dropped, false otherwise.
   */
  static bool UnsafeDroppablePrivateComponent(PrivateComponentRef& target, const std::vector<std::string>& type_names);

  /**
   * @brief Drops an asset onto a droppable area.
   *
   * @tparam T The type of asset.
   * @param target Reference to the AssetRef target.
   * @return True if the asset was successfully dropped, false otherwise.
   */
  template <typename T = IAsset>
  static bool Droppable(AssetRef& target);

  /**
   * @brief Drops a private component onto a droppable area.
   *
   * @tparam T The type of private component.
   * @param target Reference to the PrivateComponentRef target.
   * @return True if the private component was successfully dropped, false otherwise.
   */
  template <typename T = IPrivateComponent>
  static bool Droppable(PrivateComponentRef& target);

  /**
   * @brief Drops an entity reference onto a droppable area.
   *
   * @param entity_ref Reference to the EntityRef target.
   * @return True if the entity reference was successfully dropped, false otherwise.
   */
  static bool Droppable(EntityRef& entity_ref);

  /**
   * @brief Renames an asset within a controlled environment.
   *
   * @tparam T The type of asset.
   * @param target Reference to the AssetRef target.
   * @return True if the asset was successfully renamed, false otherwise.
   */
  template <typename T = IAsset>
  static bool Rename(AssetRef& target);

  /**
   * @brief Renames an entity reference.
   *
   * @param entity_ref Reference to the EntityRef target.
   * @return True if the entity was successfully renamed, false otherwise.
   */
  [[nodiscard]] static bool Rename(EntityRef& entity_ref);

  /**
   * @brief Renames a shared pointer to an asset within a controlled environment.
   *
   * @tparam T The type of asset.
   * @param target Shared pointer to the target asset.
   * @return True if the asset was successfully renamed, false otherwise.
   */
  template <typename T = IAsset>
  static bool RenameAsset(const std::shared_ptr<T>& target);

  /**
   * @brief Renames an entity within a controlled environment.
   *
   * @param entity The Entity to rename.
   * @return True if the entity was successfully renamed, false otherwise.
   */
  [[nodiscard]] static bool RenameEntity(const Entity& entity);

  /**
   * @brief Removes an asset from the editor.
   *
   * @tparam T The type of asset.
   * @param target Reference to the AssetRef target.
   * @return True if the asset was successfully removed, false otherwise.
   */
  template <typename T = IAsset>
  static bool Remove(AssetRef& target);

  /**
   * @brief Removes a private component from the editor.
   *
   * @tparam T The type of private component.
   * @param target Reference to the PrivateComponentRef target.
   * @return True if the private component was successfully removed, false otherwise.
   */
  template <typename T = IPrivateComponent>
  static bool Remove(PrivateComponentRef& target);

  /**
   * @brief Removes an entity reference from the editor.
   *
   * @param entity_ref Reference to the EntityRef target.
   * @return True if the EntityRef target was successfully removed, false otherwise.
   */
  [[nodiscard]] static bool Remove(EntityRef& entity_ref);

#pragma endregion

#pragma region Gizmos

  /**
   * @brief Draws a gizmo mesh with specified settings.
   *
   * @param mesh Shared pointer to the mesh.
   * @param editor_camera_component Shared pointer to the editor camera component.
   * @param color The color of the gizmo mesh (default: white).
   * @param model The model matrix for the mesh (default: identity matrix).
   * @param size The size of the gizmo mesh (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo (default: {}).
   */
  void DrawGizmoMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Camera>& editor_camera_component,
                     const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                     const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws a gizmo mesh with specified settings.
   *
   * @param gizmo_mesh_task Structure representing a gizmo mesh task.
   */
  void DrawGizmoMesh(const GizmoMeshTask& gizmo_mesh_task);

  /**
   * @brief Draws gizmo strands with specified settings.
   *
   * @param strands Shared pointer to the strands.
   * @param editor_camera_component Shared pointer to the editor camera component.
   * @param color The color of the gizmo strands (default: white).
   * @param model The model matrix for the strands (default: identity matrix).
   * @param size The size of the gizmo strands (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo strands (default: {}).
   */
  void DrawGizmoStrands(const std::shared_ptr<Strands>& strands, const std::shared_ptr<Camera>& editor_camera_component,
                        const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                        const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws gizmo strands with specified settings.
   *
   * @param gizmo_strands_task Structure representing a gizmo strands task.
   */
  void DrawGizmoStrands(const GizmoStrandsTask& gizmo_strands_task);

  /**
   * @brief Draws an instanced, colored gizmo mesh with specified settings.
   *
   * @param gizmo_instanced_mesh_task Structure representing a gizmo instanced mesh task..
   */
  void DrawGizmoMeshInstancedColored(const GizmoInstancedMeshTask& gizmo_instanced_mesh_task);

  /**
   * @brief Draws an instanced, colored gizmo mesh with specified settings.
   *
   * @param mesh Shared pointer to the instanced mesh.
   * @param editor_camera_component Shared pointer to the editor camera component.
   * @param particle_info_list Shared pointer to particle instance data.
   * @param model The model matrix for the mesh (default: identity matrix).
   * @param size The size of the gizmo mesh (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo mesh (default: {}).
   */
  void DrawGizmoMeshInstancedColored(const std::shared_ptr<Mesh>& mesh,
                                     const std::shared_ptr<Camera>& editor_camera_component,
                                     const std::shared_ptr<ParticleInfoList>& particle_info_list,
                                     const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                                     const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws an instanced, colored gizmo mesh without camera association.
   *
   * @param mesh Shared pointer to the instanced mesh.
   * @param particle_info_list Shared pointer to particle instance data.
   * @param model The model matrix for the mesh (default: identity matrix).
   * @param size The size of the gizmo mesh (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo mesh (default: {}).
   */
  void DrawGizmoMeshInstancedColored(const std::shared_ptr<Mesh>& mesh,
                                     const std::shared_ptr<ParticleInfoList>& particle_info_list,
                                     const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                                     const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws a gizmo mesh with specified settings (color and vertices).
   *
   * @param mesh Shared pointer to the mesh.
   * @param color The color of the gizmo mesh (default: white).
   * @param model The model matrix for the mesh (default: identity matrix).
   * @param size The size of the gizmo mesh (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo (default: {}).
   */
  void DrawGizmoMesh(const std::shared_ptr<Mesh>& mesh, const glm::vec4& color = glm::vec4(1.0f),
                     const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                     const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws gizmo strands without camera association.
   *
   * @param strands Shared pointer to the strands.
   * @param color The color of the gizmo strands (default: white).
   * @param model The model matrix for the strands (default: identity matrix).
   * @param size The size of the gizmo strands (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo strands (default: {}).
   */
  void DrawGizmoStrands(const std::shared_ptr<Strands>& strands, const glm::vec4& color = glm::vec4(1.0f),
                        const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                        const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws gizmo cubes using instanced particle data.
   *
   * @param particle_info_list Shared pointer to particle instance data.
   * @param model The model matrix for the cubes (default: identity matrix).
   * @param size The size of the gizmo cubes (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo cubes (default: {}).
   */
  void DrawGizmoCubes(const std::shared_ptr<ParticleInfoList>& particle_info_list,
                      const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                      const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws a single gizmo cube with specified settings.
   *
   * @param color The color of the gizmo cube (default: white).
   * @param model The model matrix for the cube (default: identity matrix).
   * @param size The size of the gizmo cube (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo cube (default: {}).
   */
  void DrawGizmoCube(const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                     const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws gizmo spheres using instanced particle data.
   *
   * @param particle_info_list Shared pointer to particle instance data.
   * @param model The model matrix for the spheres (default: identity matrix).
   * @param size The size of the gizmo spheres (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo spheres (default: {}).
   */
  void DrawGizmoSpheres(const std::shared_ptr<ParticleInfoList>& particle_info_list,
                        const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                        const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws a single gizmo sphere with specified settings.
   *
   * @param color The color of the gizmo sphere (default: white).
   * @param model The model matrix for the sphere (default: identity matrix).
   * @param size The size of the gizmo sphere (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo sphere (default: {}).
   */
  void DrawGizmoSphere(const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                       const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws gizmo cylinders using instanced particle data.
   *
   * @param particle_info_list Shared pointer to particle instance data.
   * @param model The model matrix for the cylinders (default: identity matrix).
   * @param size The size of the gizmo cylinders (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo cylinders (default: {}).
   */
  void DrawGizmoCylinders(const std::shared_ptr<ParticleInfoList>& particle_info_list,
                          const glm::mat4& model = glm::mat4(1.0f), const float& size = 1.0f,
                          const GizmoSettings& gizmo_settings = {});

  /**
   * @brief Draws a single gizmo cylinder with specified settings.
   *
   * @param color The color of the gizmo cylinder (default: white).
   * @param model The model matrix for the cylinder (default: identity matrix).
   * @param size The size of the gizmo cylinder (default: 1.0f).
   * @param gizmo_settings The settings for the gizmo cylinder (default: {}).
   */
  void DrawGizmoCylinder(const glm::vec4& color = glm::vec4(1.0f), const glm::mat4& model = glm::mat4(1.0f),
                         const float& size = 1.0f, const GizmoSettings& gizmo_settings = {});

#pragma endregion

  /**
   * @brief Checks whether any gizmos are currently being displayed.
   *
   * @return True if gizmos are being displayed, false otherwise.
   */
  [[nodiscard]] bool IsGizmosDisplaying() const;

  /**
   * @brief Checks whether any gizmos are currently being used.
   *
   * @return True if gizmos are being used, false otherwise.
   */
  [[nodiscard]] bool IsGizmosUsing() const;

 private:
  /**
   * @brief Loads icons for the editor.
   */
  void LoadIcons();
  void RegisterEditorPanels();

  /**
   * @brief Called during the creation of the EditorLayer.
   */
  void OnCreate() override;

  /**
   * @brief Called during the destruction of the EditorLayer.
   */
  void OnDestroy() override;

  /**
   * @brief Performs pre-update operations for the EditorLayer.
   */
  void PreUpdate() override;

  /**
   * @brief Inspects the editor layer along with its associated components.
   *
   * @param editor_layer Shared pointer to the EditorLayer being inspected.
   */
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  ImGuiID dock_space_id;
  /**
   * @brief Draws the root ImGui dockspace for the editor.
   */
  void DrawDockspace();

  /**
   * @brief Draws the editor menu bar.
   */
  void DrawMainMenuBar();
  void RequestDefaultEditorLayout();

  void UpdateCameraTransition();
  void PrepareFrameState();
  void CaptureSceneWindowMousePosition();
  void CaptureMainCameraWindowMousePosition();
  void UpdateSceneState(const std::shared_ptr<Scene>& scene);
  void DrawEntityExplorerWindow(const std::shared_ptr<Scene>& scene);
  void DrawEntityInspectorWindow(const std::shared_ptr<Scene>& scene, const std::shared_ptr<EditorLayer>& editor_layer);
  void DrawConsoleWindow();
  void DrawRuntimePackageManagerWindow();
  void HandleSceneDeleteShortcut(const std::shared_ptr<Scene>& scene);
  void DrawSceneCameraDebugWindow(const std::shared_ptr<Scene>& scene);
  void DrawLayerInspectionWindows(const std::shared_ptr<Scene>& scene,
                                  const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Displays the scene camera window.
   */
  void SceneCameraWindow();

  /**
   * @brief Displays the main camera window.
   */
  void MainCameraWindow();

  /**
   * @brief Handles input events for the EditorLayer.
   *
   * @param input_event The input event to be processed.
   */
  void OnInputEvent(const Input::InputEvent& input_event) override;

  /**
   * @brief Resizes the cameras based on updated dimensions.
   */
  void ResizeCameras();

  Handle scene_camera_handle_ = 0;                          /**< Handle to the scene camera. */
  std::unordered_map<Handle, EditorCamera> editor_cameras_; /**< Map of handles to editor cameras. */

  std::vector<GizmoMeshTask> gizmo_mesh_tasks_;                    /**< List of tasks for gizmo meshes. */
  std::vector<GizmoInstancedMeshTask> gizmo_instanced_mesh_tasks_; /**< List of tasks for instanced gizmo meshes. */
  std::vector<GizmoStrandsTask> gizmo_strands_tasks_;              /**< List of tasks for gizmo strands. */

  std::vector<ConsoleMessage> console_messages_; /**< List of console messages. */
  std::mutex console_message_mutex_;             /**< Mutex for accessing console messages. */
  EditorPanelManager editor_panel_manager_;
  bool dock_layout_reset_pending_ = false;

  bool runtime_package_manager_scanned_ = false;                   /**< Whether package manifests were scanned. */
  std::unordered_set<std::string> selected_runtime_package_names_; /**< Selected runtime packages for bulk loading. */
  enum class RuntimePackageInspectionSource { Available, Loaded };
  RuntimePackageInspectionSource inspected_runtime_package_source_ = RuntimePackageInspectionSource::Available;
  std::string inspected_runtime_package_name_;

  struct RuntimePackageBuildResult {
    bool success = false;
    int exit_code = -1;
    std::string command;
    std::string output;
    std::string error;
  };

  struct RuntimePackageBuildJob {
    std::future<RuntimePackageBuildResult> future;
    std::optional<RuntimePackageBuildResult> result;
  };

  void PollRuntimePackageBuildJobs();
  [[nodiscard]] bool HasActiveRuntimePackageBuild() const;

  std::unordered_map<std::string, RuntimePackageBuildJob> runtime_package_build_jobs_;

  bool enable_console_logs_ = true;     /**< Indicates if console logs are enabled. */
  bool enable_console_errors_ = true;   /**< Indicates if console errors are enabled. */
  bool enable_console_warnings_ = true; /**< Indicates if console warnings are enabled. */

  friend class Console;
  friend class ProjectManager;
  friend class RenderInstanceStorage;

  int selection_alpha_ = 0;                          /**< Alpha value for the selected entity. */
  bool gizmo_displaying_ = false;                    /**< Indicates if any gizmo is being displayed. */
  bool gizmo_using_ = false;                         /**< Indicates if any gizmo is being used. */
  void* mapped_entity_index_data_;                   /**< Pointer to mapped entity index data. */
  std::unique_ptr<Buffer> entity_index_read_buffer_; /**< Buffer for reading entity index. */

  /**
   * @brief Handles mouse-based entity selection.
   */
  void MouseEntitySelection();

  /**
   * @brief Performs entity selection based on mouse position and a given camera.
   *
   * @param target_camera Shared pointer to the target camera.
   * @param mouse_position The mouse position to use for selection.
   * @return The selected entity based on the provided camera and mouse coordinates.
   */
  [[nodiscard]] Entity MouseEntitySelection(const std::shared_ptr<Camera>& target_camera,
                                            const glm::vec2& mouse_position) const;

  EntityArchetype basic_entity_archetype_;        /**< Archetype for basic entities. */
  Entity previous_transform_inspection_entity_{}; /**< Previously inspected transform entity. */
  Transform previously_stored_transform_;         /**< Previously stored transform. */
  glm::vec3 previously_stored_position_;          /**< Previously stored position. */
  glm::vec3 previously_stored_rotation_;          /**< Previously stored rotation. */
  glm::vec3 previously_stored_scale_;             /**< Previously stored scale. */
  bool local_position_selected_ = true;           /**< Indicates if the local position is selected. */
  bool local_rotation_selected_ = false;          /**< Indicates if the local rotation is selected. */
  bool local_scale_selected_ = false;             /**< Indicates if the local scale is selected. */

  bool scene_camera_window_focused_ = false; /**< Indicates if the scene camera window is focused. */
  bool main_camera_window_focused_ = false;  /**< Indicates if the main camera window is focused. */

#pragma region Registrations

  friend class ClassRegistry;
  friend class RenderLayer;
  friend class Application;
  friend class ProjectManager;
  friend class Scene;

  std::unordered_map<std::string, std::shared_ptr<Texture2D>> editor_icons_; /**< Map of editor icons by name. */
  std::map<size_t, std::function<bool(Entity entity, IDataComponent* data, bool is_root)>>
      component_data_inspector_map_; /**< Map of component data inspectors by type hash. */

  std::vector<std::weak_ptr<File>> asset_record_bus_;          /**< Weak pointer to asset records bus. */
  std::map<std::string, std::vector<AssetRef>> asset_ref_bus_; /**< Map of asset references by type name. */
  std::map<std::string, std::vector<PrivateComponentRef>>
      private_component_ref_bus_; /**< Map of private component references by type name. */
  std::map<std::string, std::vector<EntityRef>> entity_ref_bus_; /**< Map of entity references by type name. */

#pragma endregion

#pragma region Transfer

  glm::quat previous_rotation_; /**< Previous camera rotation. */
  glm::vec3 previous_position_; /**< Previous camera position. */
  glm::quat target_rotation_;   /**< Target camera rotation. */
  glm::vec3 target_position_;   /**< Target camera position. */
  float transition_time_;       /**< Transition time for camera movement. */
  float transition_timer_;      /**< Timer for camera movement transition. */

#pragma endregion

  std::vector<Entity> selected_entity_hierarchy_list_; /**< List of selected entity hierarchies. */

  int scene_camera_resolution_x_ = 1; /**< Scene camera resolution width. */
  int scene_camera_resolution_y_ = 1; /**< Scene camera resolution height. */

  bool lock_entity_selection_ = false; /**< Indicates if entity selection is locked. */

  bool highlight_selection_ = true; /**< Indicates if selection highlighting is enabled. */

  Entity selected_entity_; /**< Currently selected entity. */

  glm::vec2 mouse_scene_window_position_;  /**< Mouse position in the scene window. */
  glm::vec2 mouse_camera_window_position_; /**< Mouse position in the camera window. */

  float main_camera_resolution_multiplier_ = 1.0f; /**< Multiplier for main camera resolution. */
};

#pragma region ImGui Helpers

template <typename T1>
void EditorLayer::RegisterComponentDataInspector(
    const std::function<bool(Entity entity, IDataComponent* data, bool is_root)>& func) {
  component_data_inspector_map_.insert_or_assign(typeid(T1).hash_code(), func);
}

template <typename T>
bool EditorLayer::DragAndDropButton(AssetRef& target, const std::string& name, const bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  const auto ptr = target.Get<IAsset>();
  bool status_changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.5f, 0, 1));
  if (ptr) {
    const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
    ImGui::Button((ptr->GetTitle() + tag).c_str());
    Draggable(target);
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
  status_changed = Droppable<T>(target) || status_changed;
  return status_changed;
}

template <typename T>
bool EditorLayer::DragAndDropButton(PrivateComponentRef& target, const std::string& name, const bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  bool status_changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.3f, 0, 1));
  if (const auto ptr = target.Get<IPrivateComponent>()) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    if (!scene->IsEntityValid(ptr->GetOwner())) {
      target.Clear();
      ImGui::Button("none");
      ImGui::PopStyleColor(1);
      return true;
    }
    ImGui::Button(scene->GetEntityName(ptr->GetOwner()).c_str());
    Draggable(target);
    if (modifiable) {
      status_changed = Remove(target) || status_changed;
    }
    if (!status_changed && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
      selected_entity_ = ptr->GetOwner();
    }
  } else {
    ImGui::Button("none");
  }
  ImGui::PopStyleColor(1);
  status_changed = Droppable<T>(target) || status_changed;
  return status_changed;
}
template <typename T>
void EditorLayer::DraggablePrivateComponent(const std::shared_ptr<T>& target) {
  if (const auto ptr = std::dynamic_pointer_cast<IPrivateComponent>(target)) {
    const auto type = ptr->GetTypeName();
    auto entity = ptr->GetOwner();
    if (const auto scene = ApplicationContext::Get().GetActiveScene(); scene->IsEntityValid(entity)) {
      if (ImGui::BeginDragDropSource()) {
        auto handle = scene->GetEntityHandle(entity);
        ImGui::SetDragDropPayload("PrivateComponent", &handle, sizeof(Handle));
        ImGui::TextColored(ImVec4(0, 0, 1, 1), type.c_str());
        ImGui::EndDragDropSource();
      }
    }
  }
}
template <typename T>
void EditorLayer::DraggableAsset(const std::shared_ptr<T>& target) {
  if (ImGui::BeginDragDropSource()) {
    const auto ptr = std::dynamic_pointer_cast<IAsset>(target);
    if (ptr) {
      const auto title = ptr->GetTitle();
      ImGui::SetDragDropPayload("Asset", &ptr->handle_, sizeof(Handle));
      ImGui::TextColored(ImVec4(0, 0, 1, 1), title.c_str());
    }
    ImGui::EndDragDropSource();
  }
}
template <typename T>
void EditorLayer::Draggable(AssetRef& target) {
  DraggableAsset(target.Get<IAsset>());
}
template <typename T>
bool EditorLayer::Droppable(AssetRef& target) {
  bool status_changed = false;
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      const std::shared_ptr<IAsset> ptr = target.Get<IAsset>();
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      const Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (!ptr || payload_n.GetValue() != target.GetAssetHandle().GetValue()) {
        const auto asset = AssetManager::GetAssetImpl(payload_n);
        if (std::dynamic_pointer_cast<T>(asset)) {
          target.Clear();
          target.asset_handle_ = payload_n;
          target.Update();
          status_changed = true;
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
  return status_changed;
}

template <typename T>
bool EditorLayer::Rename(AssetRef& target) {
  return RenameAsset(target.Get<IAsset>());
}
template <typename T>
bool EditorLayer::Remove(AssetRef& target) {
  bool status_changed = false;
  if (const auto ptr = target.Get<IAsset>()) {
    const std::string type = ptr->GetTypeName();
    const std::string tag = "##" + type + std::to_string(ptr->GetHandle());
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::Button(("Remove" + tag).c_str())) {
        target.Clear();
        status_changed = true;
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}
template <typename T>
bool EditorLayer::Remove(PrivateComponentRef& target) {
  bool status_changed = false;
  if (const auto ptr = target.Get<IPrivateComponent>()) {
    const std::string type = ptr->GetTypeName();
    const std::string tag = "##" + type + std::to_string(ptr->GetHandle());
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::Button(("Remove" + tag).c_str())) {
        target.Clear();
        status_changed = true;
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}

template <typename T>
bool EditorLayer::RenameAsset(const std::shared_ptr<T>& target) {
  constexpr bool status_changed = false;
  auto ptr = std::dynamic_pointer_cast<IAsset>(target);
  const std::string type = ptr->GetTypeName();
  const std::string tag = "##" + type + std::to_string(ptr->GetHandle());
  if (ImGui::BeginPopupContextItem(tag.c_str())) {
    if (!ptr->IsTemporary()) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256];
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          if (bool succeed = ptr->SetPathAndSave(ptr->GetAssetsFolderRelativePath().replace_filename(
                  std::string(new_name) + ptr->GetFileRecord().lock()->GetAssetExtension())))
            memset(new_name, 0, 256);
        }
        ImGui::EndMenu();
      }
    }
    ImGui::EndPopup();
  }
  return status_changed;
}
template <typename T>
void EditorLayer::Draggable(PrivateComponentRef& target) {
  DraggablePrivateComponent(target.Get<IPrivateComponent>());
}

template <typename T>
bool EditorLayer::Droppable(PrivateComponentRef& target) {
  return UnsafeDroppablePrivateComponent(target, {Serialization::GetSerializableTypeName<T>()});
}

#pragma endregion
}  // namespace evo_engine
