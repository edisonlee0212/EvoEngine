
#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {

/**
 * @class PlayerController
 * @brief A component responsible for managing player input and interaction for controlling a scene camera.
 */
class EVOENGINE_API PlayerController : public IPrivateComponent {
  float last_x_ = 0;                    ///< Last recorded mouse X position.
  float last_y_ = 0;                    ///< Last recorded mouse Y position.
  float last_scroll_y_ = 0;             ///< Last recorded scroll value on the Y-axis.
  bool start_mouse_ = false;            ///< Indicates whether the initial mouse input has been registered.
  float scene_camera_yaw_angle_ = -89;  ///< Yaw angle of the camera in the scene.
  float scene_camera_pitch_angle_ = 0;  ///< Pitch angle of the camera in the scene.

 public:
  float velocity = 1.0f;     ///< Camera movement velocity multiplier.
  float sensitivity = 0.1f;  ///< Sensitivity for mouse input impacting camera rotation.

  /**
   * @brief Invoked when the component is created.
   */
  void OnCreate() override;

  /**
   * @brief Performs updates that are executed after the main update.
   */
  void LateUpdate() override;

  [[nodiscard]] float& RefSceneCameraYawAngle();

  [[nodiscard]] float& RefSceneCameraPitchAngle();

  [[nodiscard]] float GetSceneCameraYawAngle() const;

  [[nodiscard]] float GetSceneCameraPitchAngle() const;

  /**
   * @brief Executes actions after this component is cloned.
   * @param target The cloned instance of the component.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

}  // namespace evo_engine
