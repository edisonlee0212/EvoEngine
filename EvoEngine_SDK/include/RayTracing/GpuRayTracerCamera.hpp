
#pragma once
#include "AssetRef.hpp"
#include "Camera.hpp"
#include "IPrivateComponent.hpp"
#include "Texture2D.hpp"

namespace evo_engine {

/**
 * @class GpuRayTracerCamera
 * @brief A GPU-based ray tracer camera component.
 *        Provides functionality to manage ray tracing
 *        parameters, camera settings, and capturing frames.
 */
class GpuRayTracerCamera : public IPrivateComponent {
 public:
  /**
   * @struct CaptureParameters
   * @brief Holds parameters for the ray tracing capture process.
   */
  struct CaptureParameters {
    uint32_t sampled_count = 0;  ///< The count of samples taken for rendering.
    uint32_t bounce = 5;         ///< Number of bounces for ray tracing.
  };

  /// Parameters controlling the ray tracing capture.
  CaptureParameters capture_parameters{};

  /**
   * @brief Retrieves the size ratio of the camera.
   * @return The size ratio as a float value.
   */
  [[nodiscard]] float GetSizeRatio() const;

  /**
   * @brief Updates the camera info block with transformation data.
   * @param camera_info_block The block of information to update.
   * @param global_transform The global transformation of the camera.
   */
  void UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform) const;

  float near_distance = 0.1f;   ///< The near clipping distance for the camera.
  float far_distance = 200.0f;  ///< The far clipping distance for the camera.
  float fov = 120;              ///< The field of view (FOV) for the camera, in degrees.

  /**
   * @brief Called when the component is created.
   */
  void OnCreate() override;

  /// Reference to an asset texture.
  AssetRef texture_ref;

  /**
   * @brief Called when the component is destroyed.
   */
  void OnDestroy() override;

  bool per_frame_capture = false;  ///< Flag indicating if each frame should be captured.

  /**
   * @brief Called during the late update phase of the component's lifecycle.
   */
  void LateUpdate() override;

  /**
   * @brief Initiates a capture process using the ray tracer.
   */
  void Capture();

  glm::uvec2 resolution = {128, 128};  ///< Resolution (width x height) for the ray tracing output.

  /**
   * @brief Displays the component in the editor and handles inspect operations.
   * @param editor_layer A shared pointer to the editor layer interface.
   * @return True if inspection was handled successfully, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace evo_engine
