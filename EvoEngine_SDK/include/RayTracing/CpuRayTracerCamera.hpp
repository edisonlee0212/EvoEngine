
#pragma once
#include "AssetRef.hpp"
#include "Camera.hpp"
#include "IPrivateComponent.hpp"
#include "Texture2D.hpp"

namespace evo_engine {

/**
 * @class CpuRayTracerCamera
 * @brief A class implementing a CPU-based ray tracer camera.
 *
 * This class is used to manage CPU ray tracing based on camera settings.
 * It provides functionalities like camera parameter updates, texture capturing,
 * and integration with an editor interface.
 */
class CpuRayTracerCamera : public IPrivateComponent {
 public:
  /**
   * @struct CaptureParameters
   * @brief Parameters for controlling the capture settings.
   *
   * This struct defines the number of samples and bounces for ray tracing captures.
   */
  struct CaptureParameters {
    size_t sample = 1;  ///< Number of samples per pixel.
    size_t bounce = 5;  ///< Number of bounces allowed per ray.
  };

  CaptureParameters capture_parameters{};  ///< The capture parameters for the camera.

  /**
   * @brief Retrieves the size ratio of the camera.
   *
   * @return The size ratio as a floating-point value.
   */
  [[nodiscard]] float GetSizeRatio() const;

  /**
   * @brief Updates the camera's information block.
   *
   * This method updates the given camera information block with the correct parameters
   * based on the provided global transform.
   *
   * @param camera_info_block The camera information block to be updated.
   * @param global_transform The global transform of the camera.
   */
  void UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform) const;

  float near_distance = 0.1f;   ///< The near clipping distance of the camera.
  float far_distance = 200.0f;  ///< The far clipping distance of the camera.
  float fov = 120;              ///< The field of view (FOV) of the camera in degrees.

  /**
   * @brief Called when the component is created.
   *
   * This method is executed during the creation phase of the component.
   */
  void OnCreate() override;

  AssetRef texture_ref;  ///< A reference to the texture asset associated with the camera.

  /**
   * @brief Captures a texture using the specified parameters.
   *
   * This method performs a CPU ray tracing capture and stores the result in the given
   * target texture.
   *
   * @param parameters The capture parameters to guide the ray tracing process.
   * @param target_texture The target texture in which the captured image will be stored.
   */
  void Capture(const CaptureParameters& parameters, const std::shared_ptr<Texture2D>& target_texture) const;

  glm::uvec2 resolution = {64, 64};  ///< The resolution of the camera's output image.

  /**
   * @brief Handles the inspection of the component in the editor.
   *
   * This method allows for interactive inspection and adjustment of the component properties
   * through an editor interface.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if the inspection handled changes successfully, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace evo_engine
